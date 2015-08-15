

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

#include "lm_pmu.h"

#define RESET_BY_MESSAGE

/* Defines the major version level supported in the driver */
static int pmu_protocol_major = 0;

int lm_pmu_exchange(struct lm_pmu_private *priv,
		MpuMsgType_t proto,
		u8 *tx_buffer,
		int tx_len,
		u8 *rx_buffer,
		int rx_len);

/* Singleton for power function */
static struct lm_pmu_private *the_one_and_only;

void lm_pmu_set_subclass_data(struct lm_pmu_private* priv, void* _data)
{
	priv->subclass_data = _data;
}

void *lm_pmu_get_subclass_data(struct lm_pmu_private *priv)
{
	return priv->subclass_data;
}

static int lm_pmu_reset(struct lm_pmu_private *priv)
{
	int status = 0;
#ifdef RESET_BY_MESSAGE
	status = lm_pmu_exchange(priv, msg_reset, 0, 0, 0, 0);
#else
	gpio_set_value(priv->gpio_reset, 0);
	msleep_interruptible(300);
	gpio_set_value(priv->gpio_reset, 1);
	msleep_interruptible(10);
#endif
	return status;
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

	gpio_direction_output(priv->gpio_boot0, 1);
	res = lm_pmu_reset(priv);
	if (res) {
		dev_err(msdev->parent, "%s: failed to reset PMU\n", __func__);
		goto restore;
	}
	mutex_lock(&priv->serial_lock);
	msleep(100);

	while (retries) {
		if ( stm32fwu_send_sync(priv->fw) >= 0 )
			break;
		retries--;
		if (retries == 0) {
			dev_err(msdev->parent, "%s: failed get %d\n", __func__, res);
			goto restore_unlock;
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
restore_unlock:
	mutex_unlock(&priv->serial_lock);
restore:
	gpio_set_value(priv->gpio_boot0, 0);
	gpio_direction_input(priv->gpio_boot0);

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
#else
static void lm_pmu_show_msg(struct lm_pmu_private *priv, const char *hdr, u8 *buf, int sz) {}
#endif

int lm_pmu_exchange(struct lm_pmu_private *priv,
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
	priv->acked = false;
	sz = mpu_create_message(proto, priv->outgoing_buffer, tx_buffer, tx_len );
	status = spi_write(priv->spi_dev, (const void*)priv->outgoing_buffer, sz);
	usleep_range(200, 300);
	gpio_set_value(priv->gpio_msg_complete, 0);
	usleep_range(200, 300);
	gpio_set_value(priv->gpio_msg_complete, 1);
	lm_pmu_show_msg(priv, "Send:", priv->outgoing_buffer, sz);
	if (status < 0) {
		dev_err(&priv->spi_dev->dev, "%s: Could not write to pmu\n", __func__);
		goto exit_unlock;
	}

	status = wait_event_interruptible_timeout(priv->wait, priv->acked, msecs_to_jiffies(1000) );

	if (status <= 0) {
		dev_err(&priv->spi_dev->dev, "%s: Timed out waiting for pmu\n", __func__);
		status = -ETIMEDOUT;
		goto exit_unlock;
	}

	status = spi_read(priv->spi_dev, priv->incoming_buffer, sizeof(MpuMsgHeader_t));

	if (status < 0) {
		dev_err(&priv->spi_dev->dev, "%s: SPI header read error\n", __func__);
		goto exit_unlock;
	}
	msg_hdr = mpu_message_header(priv->incoming_buffer);
	if (msg_hdr->type == msg_nack) {
		lm_pmu_show_msg(priv, "Recv:", priv->incoming_buffer, sizeof(MpuMsgHeader_t));
		dev_warn(&priv->spi_dev->dev, "%s: Transaction error from MPU\n", __func__);
		status = -EINVAL;
		goto exit_unlock;
	}
	/* Get the rest if ok */
	if (rx_len > 0) {
		status = spi_read(priv->spi_dev, priv->incoming_buffer+sizeof(MpuMsgHeader_t), rx_len);
		memcpy(rx_buffer, mpu_get_payload(priv->incoming_buffer), rx_len);
	}
#ifdef DEBUG
	lm_pmu_show_msg(priv, "Recv:", priv->incoming_buffer, rx_len+sizeof(MpuMsgHeader_t));
#endif

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
		dev_dbg(&priv->spi_dev->dev, "ALERT\n");
	if (priv->alert_cb)
		return (*priv->alert_cb)(priv->subclass_data);

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


static int lm_pmu_get_version(struct lm_pmu_private *priv)
{
	int status=0;
	status = lm_pmu_exchange(priv, msg_version, 0, 0, (u8*)&priv->version, sizeof(priv->version));
	if (status < 0)
		dev_err(&priv->spi_dev->dev, "%s: failed\n", __func__);
	else {
		dev_info(&priv->spi_dev->dev, "Version %d.%d [%s]\n",
				priv->version.ver_major, priv->version.ver_minor,
				priv->version.git_info);
	}
	return status;
}

int lm_pmu_set_charge(struct lm_pmu_private *priv, ChargeMessageType_t t, u16 mask)
{
	int status=0;
	u8 tx_buffer[sizeof(ChargeMessage_t)];

	int n = charge_create_message(t, tx_buffer, mask);
	status = lm_pmu_exchange(priv, msg_charge, tx_buffer, n, 0, 0);
	if (status < 0)
		dev_err(&priv->spi_dev->dev, "%s: failed\n", __func__);
	return status;
}

void lm_pmu_poweroff(void)
{
	int status = lm_pmu_exchange(the_one_and_only, msg_poweroff, 0, 0, 0, 0);
	if (status < 0)
		dev_err(&the_one_and_only->spi_dev->dev, "%s: failed\n", __func__);
	else
		dev_info(&the_one_and_only->spi_dev->dev, "%s: OK\n", __func__);
}



#if 0
static void lm_pmu_response(struct work_struct *work)
{
	struct lm_pmu_private *priv = container_of(work, struct lm_pmu_private, response_work);
}
#endif


/***********************************************************
 *
 * Version
 */

static ssize_t lm_pmu_show_version(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);

	if (strcmp(attr->attr.name, "pmu_git_id") == 0) {
		return sprintf(buf, "%s\n", priv->version.git_info);
	}
	if (strcmp(attr->attr.name, "pmu_version_sw") == 0) {
		return sprintf(buf, "%d.%d\n", priv->version.ver_major, priv->version.ver_minor);
	}
	return -EINVAL;
}

DEVICE_ATTR(pmu_version_sw, 0444, lm_pmu_show_version, NULL);
DEVICE_ATTR(pmu_git_id, 0444, lm_pmu_show_version, NULL);

static struct attribute *version_sysfs_attr[] = {
	&dev_attr_pmu_version_sw.attr,
	&dev_attr_pmu_git_id.attr,
	NULL,
};

static const struct attribute_group version_sysfs_attr_group = {
	.attrs = version_sysfs_attr,
};

/**********************************************************************
 *  Parse DT
 */

static int lm_pmu_from_dt(struct lm_pmu_private *priv)
{
	struct device *dev = &priv->spi_dev->dev;
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;


	priv->gpio_irq = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(priv->gpio_irq)) {
		dev_err(dev, "%s: invalid irq-gpio\n", __func__);
		return -EINVAL;
	}

	priv->gpio_boot0 = of_get_named_gpio(np, "boot0-gpio", 0);
	if (gpio_is_valid(priv->gpio_boot0)) {
		if (devm_gpio_request_one(dev, priv->gpio_boot0, GPIOF_IN, "pmu-boot0")) {
			dev_warn(dev, "%s: unable to request GPIO pmu-boot0 [%d]\n", __func__, priv->gpio_boot0);
			priv->gpio_boot0 = -1;
		}
	}
	else {
		dev_warn(dev, "%s: no GPIO for BOOT0 pin to PMU\n", __func__);
	}

	priv->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(priv->gpio_reset)) {
		if (devm_gpio_request_one(dev, priv->gpio_reset, GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN, "pmu-reset")) {
			dev_warn(dev, "%s: unable to request GPIO reset-gpio [%d]\n", __func__, priv->gpio_reset);
			priv->gpio_reset = -1;
		}
	}
	else {
		dev_warn(dev, "%s: no GPIO for RESET pin to PMU\n", __func__);
	}

	priv->gpio_msg_complete = of_get_named_gpio(np, "msg-complete-gpio", 0);
	if (gpio_is_valid(priv->gpio_msg_complete)) {
		if (devm_gpio_request_one(dev, priv->gpio_msg_complete, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "msg-complete-gpio")) {
				dev_err(dev, "%s: unable to request GPIO msg-complete-gpio [%d]\n", __func__, priv->gpio_msg_complete);
				return -EINVAL;
		}
		else
			gpio_export(priv->gpio_msg_complete, 0);
	}
	else {
		dev_err(dev, "%s: no GPIO for message complete pin to PMU\n", __func__);
		return -EINVAL;
	}

	priv->gpio_alert = of_get_named_gpio(np, "alert-gpio", 0);
	if (!gpio_is_valid(priv->gpio_alert))
		dev_warn(dev, "%s: Invalid gpio pin for alert [%d]\n", __func__, priv->gpio_alert);

	return 0;
}

struct lm_pmu_private *lm_pmu_init(struct spi_device *spi)
{
	struct lm_pmu_private *priv;
	int ret=0;
	int trials=3;

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	the_one_and_only = priv;
	priv->spi_dev = spi;

	dev_info(&spi->dev, "Getting platform data from OF\n");
	ret = lm_pmu_from_dt(priv);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Failed to obtain platform data\n", __func__);
		return NULL;
	}

	mutex_init(&priv->serial_lock);
	/* INIT_WORK(&priv->response_work, lm_pmu_response); */
	init_waitqueue_head(&priv->wait);
	spi_set_drvdata(spi, priv);

	priv->fw_dev.minor = 250;
	priv->fw_dev.name = "lm_pmu_fwupdate";
	priv->fw_dev.fops = &lm_pmu_fops;
	priv->fw_dev.parent = &spi->dev;

	priv->gpio_irq_nr = gpio_to_irq(priv->gpio_irq);
	ret = devm_request_irq(&spi->dev, priv->gpio_irq_nr,
			lm_pmu_spi_irq_handler, IRQF_TRIGGER_FALLING,
			"lm_pmu", priv);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: not able to get irq for gpio %d, err %d\n", __func__, priv->gpio_irq, ret);
		return NULL;
	}

	if (gpio_is_valid(priv->gpio_alert)) {
		priv->gpio_alert_irq_nr = gpio_to_irq(priv->gpio_alert);
		ret = devm_request_irq(&spi->dev, priv->gpio_alert_irq_nr,
				lm_pmu_alert_irq_handler, IRQF_TRIGGER_FALLING,
				"lm_pmu_alert", priv);
		if (ret < 0) {
			dev_err(&spi->dev, "%s: not able to get irq for gpio %d, err %d\n", __func__, priv->gpio_alert, ret);
			return NULL;
		}
	}
	ret = misc_register(&priv->fw_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register firmware device\n");
		priv->fw_dev_ok = false;
		goto cleanup;
	}

	priv->fw_dev_ok = true;
	priv->pmu_ready = false;

	while (trials) {
		ret = lm_pmu_get_version(priv);
		if (ret == 0)
			break;
		trials--;
	}
	if (ret == 0) {
		priv->pmu_ready = true;
		ret = sysfs_create_group(&priv->fw_dev.this_device->kobj, &version_sysfs_attr_group);
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to register firmware sysfs\n");
		}
		priv->rtc = devm_rtc_device_register(&spi->dev, "lm_pmu", &lm_pmu_rtc_ops, THIS_MODULE);
		if (IS_ERR(priv->rtc)) {
			ret = PTR_ERR(priv->rtc);
			goto cleanup;
		}
	}

	return priv;

cleanup:
	return 0;
}

int lm_pmu_deinit(struct lm_pmu_private *priv)
{
	disable_irq(priv->gpio_irq_nr);
	if (priv->pmu_ready)
		sysfs_remove_group(&priv->fw_dev.this_device->kobj, &version_sysfs_attr_group);
	if (priv->fw_dev_ok) {
		misc_deregister(&priv->fw_dev);
	}
	priv->fw_dev_ok = false;
	return 0;
}
