#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <linux/power_supply.h>

#include "muxprotocol.h"
#include "rtc_proto.h"
#include "ina219_proto.h"

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

int lm_pmu_reset_message(void)
{
	if (the_one_and_only)
		return lm_pmu_exchange(the_one_and_only, msg_reset, 0, 0, 0, 0);
	else
		return -EINVAL;
}

EXPORT_SYMBOL(lm_pmu_reset_message);


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
	dev_info(&priv->i2c_dev->dev, "%s %s\n", hdr, msg);
}

int lm_pmu_exchange(struct lm_pmu_private *priv,
		MpuMsgType_t proto,
		u8 *tx_buffer,
		int tx_len,
		u8 *rx_buffer,
		int rx_len)
{
	int status=0;
	int sz;
	MpuMsgHeader_t msg_hdr;
	mutex_lock(&priv->serial_lock);
	dev_dbg(&priv->i2c_dev->dev, "p=%d, txl=%d, rxl=%d\n", proto, tx_len, rx_len);
	priv->acked = false;
	sz = mpu_create_message(proto, priv->outgoing_buffer, tx_buffer, tx_len );
	status = i2c_master_send(priv->i2c_dev, priv->outgoing_buffer, sz );
	if (status < 0) {
		dev_err(&priv->i2c_dev->dev, "%s: Could not write to pmu, err = %d\n", __func__, status);
		goto exit_unlock;
	}
	usleep_range(200, 300);
	gpio_set_value(priv->gpio_msg_complete, 0);
	usleep_range(200, 300);
	gpio_set_value(priv->gpio_msg_complete, 1);

	status = wait_event_interruptible_timeout(priv->wait, priv->acked, msecs_to_jiffies(1000) );

	if (status <= 0) {
		dev_err(&priv->i2c_dev->dev, "Timed out [%d] waiting for pmu message %d\n", status, proto);
		lm_pmu_show_msg(priv, "Send:", priv->outgoing_buffer, sz);
		status = -ETIMEDOUT;
		goto exit_unlock;
	}

	status = i2c_master_recv(priv->i2c_dev, priv->incoming_buffer, PROTO_BUF_SIZE );

	if (status < 0) {
		dev_err(&priv->i2c_dev->dev, "%s: SPI header read error\n", __func__);
		goto exit_unlock;
	}
	msg_hdr = mpu_message_header(priv->incoming_buffer);
	if (msg_hdr.type == msg_nack || status < (rx_len + sizeof(MpuMsgHeader_t))) {
		lm_pmu_show_msg(priv, "Recv:", priv->incoming_buffer, sizeof(MpuMsgHeader_t));
		dev_warn(&priv->i2c_dev->dev, "%s: Transaction error from PMU on message %d\n", __func__, proto);
		status = -EINVAL;
		goto exit_unlock;
	}
	/* Get the rest if ok */
	if (rx_len > 0) {
		//status = pmu_spi_read(priv->i2c_dev, priv->incoming_buffer+sizeof(MpuMsgHeader_t), rx_len);
		memcpy(rx_buffer, mpu_get_payload(priv->incoming_buffer), rx_len);
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
	dev_dbg(&priv->i2c_dev->dev, "SPI IRQ\n");
	return IRQ_HANDLED;

}

irqreturn_t lm_pmu_alert_irq_handler(int irq, void *dev_id)
{
	struct lm_pmu_private *priv = dev_id;
	if (priv != the_one_and_only)
		pr_err("%s: IRQ BUG\n", __func__);
	else
		dev_dbg(&priv->i2c_dev->dev, "ALERT\n");
	if (priv->alert_cb)
		return (*priv->alert_cb)(priv->subclass_data);

	return IRQ_HANDLED;
}

static int lm_pmu_get_datetime(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm_pmu_private *priv = i2c_get_clientdata(client);
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
	struct i2c_client *client = to_i2c_client(dev);
	struct lm_pmu_private *priv = i2c_get_clientdata(client);
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
	u8 buffer[sizeof(MpuVersionHeader_t)];
	status = lm_pmu_exchange(priv, msg_version, 0, 0, buffer, sizeof(buffer));
	if (status < 0)
		dev_err(&priv->i2c_dev->dev, "%s: failed\n", __func__);
	else {
		priv->version = *mpu_get_version_header(buffer);
		dev_info(&priv->i2c_dev->dev, "Version %d.%d [%s]\n",
				priv->version.ver_major, priv->version.ver_minor,
				priv->version.git_info);
	}
	return status;
}

static int lm_pmu_get_init(struct lm_pmu_private *priv)
{
	int status=0;
	u8 buffer[sizeof(InitMessage_t)];
	status = lm_pmu_exchange(priv, msg_init, 0, 0, buffer, sizeof(buffer));
	if (status < 0)
		dev_err(&priv->i2c_dev->dev, "%s: failed\n", __func__);
	else {
		priv->init_status = init_get_message(buffer);
		dev_info(&priv->i2c_dev->dev, "Init status is 0x%x\n", priv->init_status.event_mask);
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
		dev_err(&priv->i2c_dev->dev, "%s: failed\n", __func__);
	return status;
}

void lm_pmu_poweroff(void)
{
	int status = lm_pmu_exchange(the_one_and_only, msg_poweroff, 0, 0, 0, 0);
	if (status < 0)
		dev_err(&the_one_and_only->i2c_dev->dev, "%s: failed\n", __func__);
	else
		dev_info(&the_one_and_only->i2c_dev->dev, "%s: OK\n", __func__);
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
	else if (strcmp(attr->attr.name, "pmu_version_sw") == 0) {
		return sprintf(buf, "%d.%d\n", priv->version.ver_major, priv->version.ver_minor);
	}
	else if (strcmp(attr->attr.name, "pmu_reason_booted") == 0) {
		return sprintf(buf, "%d\n", priv->init_status.event_mask);
	}
	return -EINVAL;
}

DEVICE_ATTR(pmu_version_sw, 0444, lm_pmu_show_version, NULL);
DEVICE_ATTR(pmu_git_id, 0444, lm_pmu_show_version, NULL);
DEVICE_ATTR(pmu_reason_booted, 0444, lm_pmu_show_version, NULL);

static struct attribute *version_sysfs_attr[] = {
	&dev_attr_pmu_version_sw.attr,
	&dev_attr_pmu_git_id.attr,
	&dev_attr_pmu_reason_booted.attr,
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
	struct device *dev = &priv->i2c_dev->dev;
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;


	priv->gpio_irq = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(priv->gpio_irq)) {
		dev_err(dev, "%s: invalid irq-gpio\n", __func__);
		return -EINVAL;
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

struct lm_pmu_private *lm_pmu_init(struct i2c_client *client)
{
	struct lm_pmu_private *priv;
	int ret=0;
	int trials=3;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	the_one_and_only = priv;
	priv->i2c_dev = client;

	dev_info(&client->dev, "Getting platform data from OF\n");
	ret = lm_pmu_from_dt(priv);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Failed to obtain platform data\n", __func__);
		return NULL;
	}

	mutex_init(&priv->serial_lock);
	/* INIT_WORK(&priv->response_work, lm_pmu_response); */
	init_waitqueue_head(&priv->wait);
	i2c_set_clientdata(client, priv);



	priv->gpio_irq_nr = gpio_to_irq(priv->gpio_irq);
	ret = devm_request_irq(&client->dev, priv->gpio_irq_nr,
			lm_pmu_spi_irq_handler, IRQF_TRIGGER_FALLING,
			"lm_pmu", priv);
	if (ret < 0) {
		dev_err(&client->dev, "%s: not able to get irq for gpio %d, err %d\n", __func__, priv->gpio_irq, ret);
		return NULL;
	}

	if (gpio_is_valid(priv->gpio_alert)) {
		priv->gpio_alert_irq_nr = gpio_to_irq(priv->gpio_alert);
		ret = devm_request_irq(&client->dev, priv->gpio_alert_irq_nr,
				lm_pmu_alert_irq_handler, IRQF_TRIGGER_FALLING,
				"lm_pmu_alert", priv);
		if (ret < 0) {
			dev_err(&client->dev, "%s: not able to get irq for gpio %d, err %d\n", __func__, priv->gpio_alert, ret);
			return NULL;
		}
	}

	priv->pmu_ready = false;

	/* Send a dummy interrupt to sync with PMU */
	gpio_set_value(priv->gpio_msg_complete, 0);
	usleep_range(200, 300);
	gpio_set_value(priv->gpio_msg_complete, 1);
	msleep(50);
	while (trials) {
		ret = lm_pmu_get_version(priv);
		if (ret == 0)
			break;
		trials--;
	}
	if (ret == 0) {
		priv->pmu_ready = true;
		lm_pmu_get_init(priv);
		ret = sysfs_create_group(&priv->i2c_dev->dev.kobj, &version_sysfs_attr_group);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to register firmware sysfs\n");
		}
		priv->rtc = devm_rtc_device_register(&client->dev, "lm_pmu", &lm_pmu_rtc_ops, THIS_MODULE);
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
		sysfs_remove_group(&priv->i2c_dev->dev.kobj, &version_sysfs_attr_group);
	return 0;
}
