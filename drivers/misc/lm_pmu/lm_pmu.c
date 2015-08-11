
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/power_supply.h>

#include "muxprotocol.h"
#include "rtc_proto.h"
#include "ina219_proto.h"
#include "stm32fwu.h"


struct lm_pmu_config {
	bool is_lb;				/* Linkbox? */
	bool pm_power_control;	/* True if it shuts down power */
	bool has_hwmon;			/* True if PMU has power measurement */
};

static struct lm_pmu_config lb_pdata = {
	.is_lb = true,
	.pm_power_control = false,
	.has_hwmon = true,
};

static struct lm_pmu_config sp_pdata = {
	.is_lb = false,
	.pm_power_control = true,
	.has_hwmon = false,
};

struct lm_pmu_pdata {
	int gpio_boot0;
	int gpio_reset;
	int gpio_msg_complete;
	int gpio_irq;
	int gpio_alert;
	int gpio_12v_manikin;
	int gpio_5v_manikin;
};
const struct spi_device_id lm_pmu_ids[] = {
	{ "lm_pmu", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, lm_pmu_ids);

static struct of_device_id lm_pmu_dt_ids[] = {
	{ .compatible = "datarespons,lm-pmu-lb",  .data = &lb_pdata, },
	{ .compatible = "datarespons,lm-pmu-sp",  .data = &sp_pdata, },
	{}
};
MODULE_DEVICE_TABLE(of, lm_pmu_dt_ids);

#define PROTO_BUF_SIZE 1024
struct lm_pmu_private;

/* Singleton for power function */
static struct lm_pmu_private *the_one_and_only;

struct lm_pmu_private {
	struct lm_pmu_pdata *pdata;
	const struct lm_pmu_config *config;
	u8 outgoing_buffer[PROTO_BUF_SIZE];
	u8 incoming_buffer[PROTO_BUF_SIZE];
	struct mutex	serial_lock;
	struct spi_device *spi_dev;
	struct work_struct response_work;
	wait_queue_head_t wait;
	bool acked;
	struct rtc_device *rtc;
	Ina219Msg_t ina_values;
	unsigned long last_ina_update;
	bool ina_valid;
	struct miscdevice fw_dev;
	struct stm32fwu_fw *fw;
	struct power_supply *ps_dcin;
	struct power_supply *ps_manikin[2];
	int psu_valids;						/* Bit field fpr valid[1:3] */
	bool manikin_12v_power_on;
	bool manikin_5v_power_on;
};

static void lm_pmu_reset(struct lm_pmu_private *priv)
{
	gpio_set_value(priv->pdata->gpio_reset, 0);
	msleep_interruptible(300);
	gpio_set_value(priv->pdata->gpio_reset, 1);
	msleep_interruptible(10);
}

static int lm_pmu_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t lm_pmu_update(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	int retries=2;
	int res=0;
	int fw_version;
	struct miscdevice *msdev = file->private_data;
	struct lm_pmu_private *priv = dev_get_drvdata(msdev->parent);
	char *buffer = kmalloc(len, GFP_KERNEL);
	if (buffer == 0) {
		dev_err(msdev->parent, "%s: failed to kmalloc buffer of size %d\n", __func__, len);
		return -ENOMEM;
	}
	res = copy_from_user(buffer, data, len);
	if ( res  ) {
		dev_err(msdev->parent, "%s: failed to copy buffer of size %d\n", __func__, len);
		kfree(buffer);
		return -EIO;
	}
	priv->fw = stm32fwu_init(msdev->parent, STM32_SPI, buffer, len);
	if (priv->fw == 0) {
		dev_err(msdev->parent, "%s: failed to allocate fw structure\n", __func__);
		kfree(buffer);
		return -ENOMEM;
	}
	mutex_lock(&priv->serial_lock);
	gpio_direction_output(priv->pdata->gpio_boot0, 1);
	lm_pmu_reset(priv);

	while (retries) {
		if ( stm32fwu_send_sync(priv->fw) >= 0 )
			break;
		retries--;
		if (retries == 0) {
			dev_err(msdev->parent, "%s: failed get %d\n", __func__, res);
			goto restore;
		}
		msleep(1);
	}
	fw_version = stm32fwu_get_version(priv->fw);
	if (fw_version < 0) {
		dev_err(msdev->parent, "%s: failed to get fw version\n", __func__);

	}
	else {
		dev_info(msdev->parent, "%s: fw version %d\n", __func__, fw_version);
	}
restore:
	gpio_set_value(priv->pdata->gpio_boot0, 0);
	gpio_direction_input(priv->pdata->gpio_boot0);
	lm_pmu_reset(priv);
	mutex_unlock(&priv->serial_lock);
	stm32fwu_destroy(priv->fw);
	kfree(buffer);
	return len;
}

static const struct file_operations lm_pmu_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = lm_pmu_update,
	.open = lm_pmu_open,
};

#ifdef DEBUG
static void lm_pmu_show_msg(struct lm_pmu_private *priv, const char *hdr, u8 *buf, int sz)
{
	int n;
	static char msg[1024];
	char *p=msg;
	int max = 1024/8;
	if ( sz < max )
		max = sz;
	for (n=0; n < max; n++)
		p += sprintf(p, "0x%02x,", buf[n]);
	dev_info(&priv->spi_dev->dev, "%s %s\n", hdr, msg);
}
#endif

static int lm_pmu_exchange(struct lm_pmu_private *priv,
		MpuMsgType_t proto,
		u8 *tx_buffer,
		int tx_len,
		u8 *rx_buffer,
		int rx_len)
{
	int status=0;
	int sz;
	MpuMsgHeader_t *msg_hdr;
	dev_dbg(&priv->spi_dev->dev, "p=%d, txl=%d, rxl=%d\n", proto, tx_len, rx_len);
	mutex_lock(&priv->serial_lock);
	sz = mpu_create_message(proto, priv->outgoing_buffer, tx_buffer, tx_len );
	if (status == 0 && priv->pdata->gpio_msg_complete >= 0)
		gpio_set_value(priv->pdata->gpio_msg_complete, 0);
	status = spi_write(priv->spi_dev, (const void*)priv->outgoing_buffer, sz);
	if (status == 0 && priv->pdata->gpio_msg_complete >= 0) {
		gpio_set_value(priv->pdata->gpio_msg_complete, 1);
	}
#ifdef DEBUG
	lm_pmu_show_msg(priv, "Send:", priv->outgoing_buffer, sz);
#endif
	if (status < 0) {
		dev_err(&priv->spi_dev->dev, "%s: Could not write to pmu\n", __func__);
		goto exit_unlock;
	}
	priv->acked = false;

	status = wait_event_interruptible_timeout(priv->wait, priv->acked, msecs_to_jiffies(1000) );

	if (status <= 0) {
		dev_err(&priv->spi_dev->dev, "%s: Timed out waiting for pmu\n", __func__);
		status = -ETIMEDOUT;
		goto exit_unlock;
	}

	status = spi_read(priv->spi_dev, priv->incoming_buffer, rx_len+sizeof(MpuMsgHeader_t));
	if (status < 0) {
		dev_err(&priv->spi_dev->dev, "%s: SPI read error\n", __func__);
		goto exit_unlock;
	}
#ifdef DEBUG
	lm_pmu_show_msg(priv, "Recv:", priv->incoming_buffer, rx_len+sizeof(MpuMsgHeader_t));
#endif
	memcpy(rx_buffer, mpu_get_payload(priv->incoming_buffer), rx_len);
	msg_hdr = mpu_message_header(priv->incoming_buffer);
	if (msg_hdr->type & MPU_MSG_ERROR_SET) {
		dev_warn(&priv->spi_dev->dev, "%s: Transaction error from MPU\n", __func__);
		status = -EINVAL;
	}

exit_unlock:
	mutex_unlock(&priv->serial_lock);
	return status;

}

irqreturn_t lm_pmu_spi_irq_handler(int irq, void *dev_id)
{
	struct lm_pmu_private *priv = dev_id;
	if (priv != the_one_and_only)
		pr_err("%s: IRQ BUG\n", __func__);
	priv->acked = true;
	wake_up_interruptible(&priv->wait);
	dev_dbg(&priv->spi_dev->dev, "SPI IRQ\n");
	return IRQ_HANDLED;

}

irqreturn_t lm_pmu_alert_irq_handler(int irq, void *dev_id)
{
	struct lm_pmu_private *priv = dev_id;
	if (priv != the_one_and_only)
		pr_err("%s: IRQ BUG\n", __func__);
	else
		dev_info(&priv->spi_dev->dev, "ALERT\n");
	return IRQ_HANDLED;
}

static int lm_pmu_get_datetime(struct device *dev, struct rtc_time *tm)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lm_pmu_private *priv = spi_get_drvdata(spi);
	RtcMsg_t rtc_msg;
	int tx_len, status;
	u8 tx_buffer[sizeof(RtcMsgHeader_t)];
	u8 rx_buffer[sizeof(RtcMsgHeader_t) + sizeof(RtcMsg_t)];
	tx_len = rtc_create_message(msg_rtc_get_time, tx_buffer, 0);
	status = lm_pmu_exchange(priv, msg_rtc, tx_buffer, tx_len, rx_buffer, sizeof(rx_buffer));
	if (status < 0)
		return status;

	if ( rtc_get_payload(rx_buffer, &rtc_msg) == 0) {
		tm->tm_sec = rtc_msg.tm_sec;
		tm->tm_min = rtc_msg.tm_min;
		tm->tm_hour = rtc_msg.tm_hour;
		tm->tm_mday = rtc_msg.tm_mday;
		tm->tm_wday = rtc_msg.tm_wday;
		tm->tm_mon = rtc_msg.tm_mon-1;
		tm->tm_year = rtc_msg.tm_year+100;
		return rtc_valid_tm(tm);
	}
	return -EINVAL;
}

static int lm_pmu_set_datetime(struct device *dev, struct rtc_time *tm)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lm_pmu_private *priv = spi_get_drvdata(spi);
	RtcMsg_t rtc_msg;
	int tx_len, status;
	u8 rx_buffer[sizeof(RtcMsgHeader_t)];
	u8 tx_buffer[sizeof(RtcMsgHeader_t) + sizeof(RtcMsg_t)];

	rtc_msg.tm_sec = tm->tm_sec;
	rtc_msg.tm_min = tm->tm_min;
	rtc_msg.tm_hour = tm->tm_hour;
	rtc_msg.tm_mday = tm->tm_mday;
	rtc_msg.tm_wday = tm->tm_wday;
	rtc_msg.tm_mon = tm->tm_mon+1;
	rtc_msg.tm_year = tm->tm_year % 100;
	tx_len = rtc_create_message(msg_rtc_set_time, tx_buffer, &rtc_msg);
	status = lm_pmu_exchange(priv, msg_rtc, tx_buffer, tx_len, rx_buffer, sizeof(rx_buffer));

	return status;
}

struct rtc_class_ops lm_pmu_rtc_ops = {
	.read_time = lm_pmu_get_datetime,
	.set_time = lm_pmu_set_datetime,
};


static int lm_pmu_get_version(struct lm_pmu_private *priv, MpuVersionHeader_t *ver)
{
	int status=0;
	u8 rx_buffer[sizeof(MpuVersionHeader_t)];
	status = lm_pmu_exchange(priv, msg_version, 0, 0, rx_buffer, sizeof(rx_buffer));
	if (status < 0)
		dev_err(&priv->spi_dev->dev, "%s: failed\n", __func__);
	else {
		ver = mpu_get_version_header(rx_buffer);
		dev_info(&priv->spi_dev->dev, "%s: Version %d.%d found\n", __func__, ver->ver_major, ver->ver_minor);
	}
	return status;
}

static void lm_pmu_poweroff(void)
{
	int status = lm_pmu_exchange(the_one_and_only, msg_poweroff, 0, 0, 0, 0);
	if (status < 0)
		dev_err(&the_one_and_only->spi_dev->dev, "%s: failed\n", __func__);
	else
		dev_info(&the_one_and_only->spi_dev->dev, "%s: OK\n", __func__);
}

static int lm_pmu_get_valids(struct lm_pmu_private *priv)
{
	int status=0;
	u8 rx_buffer[4];
	status = lm_pmu_exchange(priv, msg_valid, 0, 0, rx_buffer, sizeof(rx_buffer));
	if (status < 0)
		dev_err(&priv->spi_dev->dev, "%s: failed\n", __func__);
	else {
		priv->psu_valids = le32_to_cpu(*(u32*)rx_buffer);
		dev_dbg(&priv->spi_dev->dev, "%s: Valids are 0x%x found\n", __func__, priv->psu_valids);
	}
	return status;
}

#if 0
static void lm_pmu_response(struct work_struct *work)
{
	struct lm_pmu_private *priv = container_of(work, struct lm_pmu_private, response_work);
}
#endif

/* Sensors */
/* worst case is 68.10 ms (~14.6Hz, ina219) */
#define INA2XX_CONVERSION_RATE		5	/* Use max 5 Hz */

static int pmu_update_ina_values(struct lm_pmu_private *priv)
{
	int tx_len, status=0;
	u8 *payload;
	u8 tx_buffer[sizeof(Ina219MsgHeader_t)];
	u8 rx_buffer[sizeof(Ina219MsgHeader_t) + sizeof(Ina219Msg_t)];



	if (time_after(jiffies, priv->last_ina_update + HZ/INA2XX_CONVERSION_RATE) || !priv->ina_valid ) {
		tx_len = ina219_create_message(msg_ina219_show_value, tx_buffer, 0);
		status = lm_pmu_exchange(priv, msg_ina, tx_buffer, tx_len, rx_buffer, sizeof(rx_buffer));
		priv->last_ina_update = jiffies;
		priv->ina_valid = true;

		if (status < 0)
			goto cleanup;

		payload = ina219_get_payload(rx_buffer);
		if (payload) {
			memcpy(&priv->ina_values, payload, sizeof(priv->ina_values));
		}
		else
			dev_err(&priv->spi_dev->dev, "%s: Failed updating ina values\n", __func__);

	}

cleanup:
	return status;
}

static enum power_supply_property dcin_props_lb[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static enum power_supply_property manikin_12v_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static enum power_supply_property dcin_props_sp[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property manikin_5v_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int lm_pmu_mains_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);
	lm_pmu_get_valids(priv);
	if (priv->config->is_lb)
		pmu_update_ina_values(priv);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		if (priv->psu_valids & 1)
		val->intval = priv->psu_valids & 1 ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = priv->ina_values.bus_voltage_uV[0];
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = priv->ina_values.current_uA[0];
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = priv->ina_values.power_uW[0];
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int lm_pmu_manikin_12v_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);
	if (priv->config->is_lb)
		pmu_update_ina_values(priv);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = priv->manikin_12v_power_on ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = priv->ina_values.bus_voltage_uV[1];
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = priv->ina_values.current_uA[1];
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = priv->ina_values.power_uW[1];
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int lm_pmu_manikin_5v_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = priv->manikin_5v_power_on ? 1 : 0;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int lm_pmu_manikin_prop_is_writable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		return 1;
		break;

	default:
		return 0;
		break;
	}
}

static int lm_pmu_manikin_12v_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		gpio_set_value(priv->pdata->gpio_12v_manikin, val->intval);
		priv->manikin_12v_power_on = val->intval ? true : false;
		power_supply_changed(psy);
		break;

	default:
		return -EPERM;
	}

	return 0;
}

static int lm_pmu_manikin_5v_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		gpio_set_value(priv->pdata->gpio_5v_manikin, val->intval);
		priv->manikin_5v_power_on = val->intval ? true : false;
		power_supply_changed(psy);
		break;

	default:
		return -EPERM;
	}

	return 0;
}

static struct lm_pmu_pdata *lm_pmu_pdata_from_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct lm_pmu_pdata *pdata = dev->platform_data;

	if (!np || pdata)
		return pdata;

	pdata = devm_kzalloc(dev, sizeof(struct lm_pmu_pdata), GFP_KERNEL);
	if (!pdata)
		return pdata;

	pdata->gpio_irq = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_irq)) {
		dev_err(dev, "%s: invalid irq-gpio\n", __func__);
		return NULL;
	}

	pdata->gpio_boot0 = of_get_named_gpio(np, "boot0-gpio", 0);
	if (gpio_is_valid(pdata->gpio_boot0)) {
		if (devm_gpio_request_one(dev, pdata->gpio_boot0, GPIOF_DIR_IN, "pmu-boot0")) {
			dev_warn(dev, "%s: unable to request GPIO pmu-boot0 [%d]\n", __func__, pdata->gpio_boot0);
			pdata->gpio_boot0 = -1;
		}
	}
	else {
		dev_warn(dev, "%s: no GPIO for BOOT0 pin to PMU\n", __func__);
	}

	pdata->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(pdata->gpio_reset)) {
		if (devm_gpio_request_one(dev, pdata->gpio_reset, GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN, "pmu-reset")) {
			dev_warn(dev, "%s: unable to request GPIO reset-gpio [%d]\n", __func__, pdata->gpio_reset);
			pdata->gpio_boot0 = -1;
		}
	}
	else {
		dev_warn(dev, "%s: no GPIO for RESET pin to PMU\n", __func__);
	}

	pdata->gpio_msg_complete = of_get_named_gpio(np, "msg-complete-gpio", 0);
	if (gpio_is_valid(pdata->gpio_msg_complete)) {
		if (devm_gpio_request_one(dev, pdata->gpio_msg_complete, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "msg-complete-gpio")) {
				dev_err(dev, "%s: unable to request GPIO msg-complete-gpio [%d]\n", __func__, pdata->gpio_msg_complete);
				return NULL;
		}
		else
			gpio_export(pdata->gpio_msg_complete, 0);
	}
	else {
		dev_err(dev, "%s: no GPIO for message complete pin to PMU\n", __func__);
		return NULL;
	}

	pdata->gpio_alert = of_get_named_gpio(np, "alert-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_alert))
		dev_warn(dev, "%s: Invalid gpio pin for alert [%d]\n", __func__, pdata->gpio_alert);

	pdata->gpio_12v_manikin = of_get_named_gpio(np, "manikin-12v-gpio", 0);
	if (gpio_is_valid(pdata->gpio_12v_manikin)) {
		if (devm_gpio_request_one(dev, pdata->gpio_12v_manikin, GPIOF_OUT_INIT_LOW, "manikin-12v")) {
			dev_err(dev, "%s: unable to request GPIO manikin-12v-gpio [%d]\n", __func__, pdata->gpio_12v_manikin);
			return NULL;
		}
	}
	pdata->gpio_5v_manikin = of_get_named_gpio(np, "manikin-5v-gpio", 0);
	if (gpio_is_valid(pdata->gpio_5v_manikin)) {
		if (devm_gpio_request_one(dev, pdata->gpio_5v_manikin, GPIOF_OUT_INIT_LOW, "manikin-5v")) {
			dev_err(dev, "%s: unable to request GPIO manikin-5v-gpio [%d]\n", __func__, pdata->gpio_5v_manikin);
			return NULL;
		}
	}
	return pdata;
}

static int lm_pmu_probe(struct spi_device *spi)
{
	struct lm_pmu_private *priv;
	int ret=0;
	int trials=3;
	struct lm_pmu_pdata *pdata = spi->dev.platform_data;
	MpuVersionHeader_t version;
	const struct of_device_id *of_id = of_match_device(lm_pmu_dt_ids, &spi->dev);

	if (!of_id) {
		dev_err(&spi->dev, "%s, of match failed on %s\n", __func__, spi->dev.of_node->name);
	}

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (!pdata) {
		dev_info(&spi->dev, "Getting platform data from OF\n");
		pdata = lm_pmu_pdata_from_dt(&spi->dev);
		if (!pdata) {
			dev_err(&spi->dev, "%s: Failed to obtain platform data\n", __func__);
			return -EINVAL;
		}
	}

	the_one_and_only = priv;

	if (of_id)
		priv->config = of_id->data;
	priv->pdata = pdata;
	priv->spi_dev = spi;


	mutex_init(&priv->serial_lock);
	/* INIT_WORK(&priv->response_work, lm_pmu_response); */
	init_waitqueue_head(&priv->wait);
	spi_set_drvdata(spi, priv);

	priv->fw_dev.minor = 250;
	priv->fw_dev.name = "lm_pmu_fwupdate";
	priv->fw_dev.fops = &lm_pmu_fops;
	priv->fw_dev.parent = &spi->dev;
	priv->manikin_12v_power_on = false;
	priv->manikin_5v_power_on = false;

	ret = devm_request_irq(&spi->dev, gpio_to_irq(pdata->gpio_irq),
			lm_pmu_spi_irq_handler, IRQF_TRIGGER_FALLING,
			"lm_pmu", priv);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: not able to get irq for gpio %d, err %d\n", __func__, priv->pdata->gpio_irq, ret);
		return -EINVAL;
	}

	ret = misc_register(&priv->fw_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register fw device\n");
		goto cleanup;
	}

	while (trials) {
		ret = lm_pmu_get_version(priv, &version);
		if (ret == 0)
			break;
		trials--;
	}
	if (ret == 0) {
		priv->rtc = devm_rtc_device_register(&spi->dev, "lm_pmu", &lm_pmu_rtc_ops, THIS_MODULE);
		if (IS_ERR(priv->rtc)) {
			ret = PTR_ERR(priv->rtc);
			goto cleanup;
		}

		if (priv->config && priv->config->pm_power_control) {
			dev_info(&spi->dev,"PMU will perform poweroff\n");
			pm_power_off = lm_pmu_poweroff;
		}

		priv->ps_dcin = devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
		if (!priv->ps_dcin) {
			ret = -ENOMEM;
			goto cleanup;
		}
		priv->ps_dcin->name = "DCIN";
		priv->ps_dcin->type = POWER_SUPPLY_TYPE_MAINS;
		priv->ps_dcin->get_property = lm_pmu_mains_get_property;
		if (priv->config->is_lb) {
			priv->ps_dcin->num_properties = ARRAY_SIZE(dcin_props_lb);
			priv->ps_dcin->properties = dcin_props_lb;
		}
		else {
			priv->ps_dcin->num_properties = ARRAY_SIZE(dcin_props_sp);
			priv->ps_dcin->properties = dcin_props_sp;
		}

		ret = power_supply_register(&spi->dev, priv->ps_dcin);
		if (ret < 0) {
			dev_err(&spi->dev, "Unable to register MAINS PS\n");
		}

		/* Setup Manikin psu's */
		if (priv->config->is_lb) {
			priv->ps_manikin[0]= devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
			priv->ps_manikin[1]= devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
			if (priv->ps_manikin[0] == 0 || priv->ps_manikin[1] == 0) {
				ret = -ENOMEM;
				goto cleanup;
			}
			priv->ps_manikin[0]->name = "MANIKIN_12V";
			priv->ps_manikin[0]->type = POWER_SUPPLY_TYPE_UNKNOWN;
			priv->ps_manikin[0]->get_property = lm_pmu_manikin_12v_get_property;
			priv->ps_manikin[0]->set_property = lm_pmu_manikin_12v_set_property;
			priv->ps_manikin[0]->num_properties = ARRAY_SIZE(manikin_12v_props);
			priv->ps_manikin[0]->properties = manikin_12v_props;
			if (gpio_is_valid(priv->pdata->gpio_12v_manikin))
				priv->ps_manikin[0]->property_is_writeable = lm_pmu_manikin_prop_is_writable;

			ret = power_supply_register(&spi->dev, priv->ps_manikin[0]);
			if (ret < 0) {
				dev_err(&spi->dev, "Unable to register MANIKIN 12V PS\n");
			}

			priv->ps_manikin[1]->name = "MANIKIN_5V";
			priv->ps_manikin[1]->type = POWER_SUPPLY_TYPE_UNKNOWN;
			priv->ps_manikin[1]->get_property = lm_pmu_manikin_5v_get_property;
			priv->ps_manikin[1]->set_property = lm_pmu_manikin_5v_set_property;
			priv->ps_manikin[1]->num_properties = ARRAY_SIZE(manikin_5v_props);
			priv->ps_manikin[1]->properties = manikin_5v_props;
			if (gpio_is_valid(priv->pdata->gpio_5v_manikin))
				priv->ps_manikin[1]->property_is_writeable = lm_pmu_manikin_prop_is_writable;

			ret = power_supply_register(&spi->dev, priv->ps_manikin[1]);
			if (ret < 0) {
				dev_err(&spi->dev, "Unable to register MANIKIN 5V PS\n");
			}

		}
	}

	return 0;

cleanup:
	return ret;
}

static int lm_pmu_remove(struct spi_device *spi)
{
	struct lm_pmu_private *priv = dev_get_drvdata(&spi->dev);
	misc_deregister(&priv->fw_dev);
	return 0;
}



static struct spi_driver lm_pmu_driver = {
	.driver = {
		.name	= "lm_pmu",
		.owner	= THIS_MODULE,
		.of_match_table = lm_pmu_dt_ids,
	},
	.probe	= lm_pmu_probe,
	.remove	= lm_pmu_remove,
	.id_table = lm_pmu_ids,
};

module_spi_driver(lm_pmu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus uC driver");
