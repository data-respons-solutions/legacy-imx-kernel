
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

#include "muxprotocol.h"
#include "rtc_proto.h"

const struct spi_device_id lm_pmu_ids[] = {
	{ "lm_pmu",  0 },
	{},
};

#define PROTO_BUF_SIZE 1024
struct lm_pmu_private;



struct lm_pmu_private {
	u8 outgoing_buffer[PROTO_BUF_SIZE];
	u8 incoming_buffer[PROTO_BUF_SIZE];
	struct mutex	serial_lock;
	struct mutex	block_lock;
	struct spi_device *spi_dev;
	struct work_struct response_work;
	wait_queue_head_t wait;
	bool acked;
	int gpio_irq;
	bool use_poll;
	int gpio_boot0;
	int gpio_reset;
	struct rtc_device *rtc;
};

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
	dev_info(&priv->spi_dev->dev, "p=%d, txl=%d, rxl=%d\n", proto, tx_len, rx_len);
	mutex_lock(&priv->serial_lock);
	sz = mpu_create_message(proto, priv->outgoing_buffer, tx_buffer, tx_len );
	status = spi_write(priv->spi_dev, (const void*)priv->outgoing_buffer, sz);
	lm_pmu_show_msg(priv, "Send:", priv->outgoing_buffer, sz);
	if (status < 0) {
		dev_err(&priv->spi_dev->dev, "%s: Could not write to pmu\n", __func__);
		goto exit_unlock;
	}
	priv->acked = false;

	if ( priv->use_poll ) {
		msleep(50);
	}
	else {
		wait_event_interruptible_timeout(priv->wait, priv->acked, msecs_to_jiffies(1000) );
		if (!priv->acked) {
			dev_err(&priv->spi_dev->dev, "%s: Timed out waiting for pmu\n", __func__);
			status = -ETIMEDOUT;
			goto exit_unlock;
		}
	}
	status = spi_read(priv->spi_dev, priv->incoming_buffer, rx_len+sizeof(MpuMsgHeader_t));
	if (status < 0) {
		dev_err(&priv->spi_dev->dev, "%s: SPI read error\n", __func__);
		goto exit_unlock;
	}
	lm_pmu_show_msg(priv, "Recv:", priv->incoming_buffer, rx_len+sizeof(MpuMsgHeader_t));
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

irqreturn_t lm_pmu_irq_handler(int irq, void *dev_id)
{
	struct lm_pmu_private *priv = dev_id;
	priv->acked = true;
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

#if 0
static void lm_pmu_response(struct work_struct *work)
{
	struct lm_pmu_private *priv = container_of(work, struct lm_pmu_private, response_work);
}
#endif

static int lm_pmu_probe(struct spi_device *spi)
{
	struct lm_pmu_private *priv;
	int ret=0;
	struct device_node *np = spi->dev.of_node;
	MpuVersionHeader_t version;

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi_dev = spi;
	priv->use_poll = true;

	priv->gpio_irq = of_get_named_gpio(np, "irq-gpio", 0);
	if (gpio_is_valid(priv->gpio_irq)) {
		ret = devm_request_irq(&spi->dev, gpio_to_irq(priv->gpio_irq),
			lm_pmu_irq_handler, IRQF_TRIGGER_RISING,
			"lm_pmu", priv);
		if (ret < 0)
			dev_err(&spi->dev, "%s: not able to get irq for gpio %d, err %d\n", __func__, priv->gpio_irq, ret);
	}
	if (priv->gpio_irq < 0 || ret < 0)
		dev_info(&spi->dev, "%s: no GPIO for interrupt, revert to polling\n", __func__);
	else
		priv->use_poll = false;

	priv->gpio_boot0 = of_get_named_gpio(np, "boot0-gpio", 0);
	if (gpio_is_valid(priv->gpio_boot0)) {
		if (gpio_request_one(priv->gpio_boot0, GPIOF_DIR_IN, "pmu-boot0")) {
			dev_err(&spi->dev, "%s: unable to request GPIO pmu-boot0 [%d]\n", __func__, priv->gpio_boot0);
			priv->gpio_boot0 = -1;
		}
	}
	else {
		dev_err(&spi->dev, "%s: no GPIO for BOOT0 pin to PMU\n", __func__);
	}

	priv->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(priv->gpio_reset)) {
		if (gpio_request_one(priv->gpio_reset, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "pmu-reset")) {
			dev_err(&spi->dev, "%s: unable to request GPIO reset-gpio [%d]\n", __func__, priv->gpio_reset);
			priv->gpio_boot0 = -1;
		}
	}
	else {
		dev_err(&spi->dev, "%s: no GPIO for RESET pin to PMU\n", __func__);
	}

	mutex_init(&priv->serial_lock);
	mutex_init(&priv->block_lock);
	/* INIT_WORK(&priv->response_work, lm_pmu_response); */
	init_waitqueue_head(&priv->wait);
	spi_set_drvdata(spi, priv);

	ret = lm_pmu_get_version(priv, &version);
	if (ret < 0)
		goto cleanup;

	priv->rtc = devm_rtc_device_register(&spi->dev, "lm_pmu", &lm_pmu_rtc_ops, THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		ret = PTR_ERR(priv->rtc);
		goto cleanup;
	}
	return 0;

cleanup:
	if (priv->gpio_boot0 >= 0)
		gpio_free(priv->gpio_boot0);
	if (priv->gpio_reset >= 0)
		gpio_free(priv->gpio_reset);
	return ret;
}

static int lm_pmu_remove(struct spi_device *spi)
{
	return 0;
}

static struct of_device_id lm_pmu_dt_ids[] = {
	{ .compatible = "datarespons,lm_pmu" },
	{}
};

static struct spi_driver lm_pmu_driver = {
	.driver = {
		.name	= "lm_pmu",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(lm_pmu_dt_ids),
	},
	.probe	= lm_pmu_probe,
	.remove	= lm_pmu_remove,
};

module_spi_driver(lm_pmu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus uC driver");
