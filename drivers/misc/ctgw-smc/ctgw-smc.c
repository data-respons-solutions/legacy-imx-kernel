/*
 * Copyright (c) 2017  Data Respons
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/sched/signal.h>
#include <linux/power_supply.h>

#include "mcu_proto.h"

#define CARGOTEC_SMC_LED_MAX_NAME_LEN	30
#define MAX_GPIOS 6

struct smc_led {
	char name[CARGOTEC_SMC_LED_MAX_NAME_LEN + 1]; /* name of the LED in sysfs */
	struct led_classdev led_dev; /* led classdev */
	int led_id;
	int registered;
};

struct smc_data {
	struct i2c_client *client;
	struct mutex transaction_mutex;

	struct smc_led led_status_green; /* led classdev - Status LED Green */
	struct smc_led led_status_red; /* led classdev - Status LED Led */
	int irq;
	bool is_ready;
	struct gpio_chip gpio_ctl;
	struct gpio_desc *gpio_descs[MAX_GPIOS];
	u8 gpio_is_input[MAX_GPIOS];
	struct device *gpio_dev[MAX_GPIOS];

	wait_queue_head_t readq; /* used by write to wake blk.read */
	MpuVersionHeader_t version;
	MpuVersionHeader_t driver_version;

	struct rtc_device *rtc;
	struct work_struct alert_work;
	struct workqueue_struct *wq;
	u8 *rx_buffer;
	int rx_result;
	u8 *transaction_read_buffer;
	int debug_mode;
	int read_pending;
	SensorMsg_t sensor;
	unsigned long sensor_jiffies;
	PowerSupplyMsg_t powersupplies;
	struct power_supply *psy;
	struct power_supply *psy_gpo1;
	struct power_supply *psy_gpo2;
	struct power_supply *psy_dcout1;
	struct power_supply *psy_dcout2;
	unsigned long psy_jiffies;
	InitEventType_t start_cause;
	int alarm_pending;
	u32 alarm_in_seconds;
	int rtc_updated;
};

/*
 * Defines
 */

#define xstr(s) str(s)
#define str(s) #s

#define DRIVER_VERSION xstr(fwVersionMajor.fwVersionMinor)

#define LED_USER_SPACE_NAME_STATUS_GREEN	"status-green"
#define LED_USER_SPACE_NAME_STATUS_RED		"status-red"

static int compare_version(MpuVersionHeader_t ver, u16 maj, u16 min)
{
	u32 numv = (maj << 16) | min;
	u32 hdr_numv = (ver.ver_major << 16) | ver.ver_minor;
	if (hdr_numv > numv)
		return 1;
	else if (hdr_numv < numv)
		return -1;
	else
		return 0;
}

static int smc_send(struct i2c_client *client, u8 *outbuf, int outbuf_size)
{
	int ret = 0;
	struct i2c_msg msg_tx[1];
	struct smc_data *data = i2c_get_clientdata(client);

	outbuf[outbuf_size] = mpu_compute_checksum(outbuf, outbuf_size);
	msg_tx[0].addr = client->addr; 		  // 7-bit address
	msg_tx[0].flags = 0;          // Write transaction, beginning with START
	msg_tx[0].len = outbuf_size + 1;  // Send one byte following the address
	msg_tx[0].buf = outbuf;                // Transfer from this address

	data->is_ready = false;

	dev_dbg(&client->dev, "%s: msg %d, sz %d\n", __func__, outbuf[0],
		outbuf_size);
	ret = i2c_transfer(client->adapter, msg_tx, 1);
	if (ret < 1) {
		dev_err(&client->dev, "i2c_transfer: [tx] , ret = %d\n", ret);
		return ret;
	}
	return 0;
}

static int smc_recv(struct i2c_client *client)
{
	int ret = 0;
	struct smc_data *data = i2c_get_clientdata(client);
	struct i2c_msg msg_rx[1];
	int rx_len = 0;
	char buf[I2C_SMBUS_BLOCK_MAX + 4];

	/* The data will get returned in this structure */
	msg_rx[0].addr = client->addr;
	msg_rx[0].flags = I2C_M_RD | I2C_M_RECV_LEN;
	msg_rx[0].len = 1;
	msg_rx[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg_rx, 1);
	if (ret <= 0) {
		dev_err(&client->dev, "i2c_transfer: [rx], ret = %d\n", ret);
		return ret;
	}
	rx_len = buf[0];
	dev_dbg(&client->dev, "%s: RX len is %d\n", __func__, rx_len);
	if (rx_len > (I2C_SMBUS_BLOCK_MAX)) {
		dev_warn(&client->dev, "i2c_transfer [rx] encoded len=%d\n",
			rx_len);
		return -EIO;
	}
	if (buf[rx_len] != mpu_compute_checksum(buf + 1, rx_len - 1)) {
		dev_warn(&client->dev,
			"i2c rcv msg[%d] checksum error, len=%d\n", buf[1],
			rx_len - 1);
		return -EINVAL;
	}

	memcpy(data->rx_buffer, buf + 1, rx_len - 1);
	return rx_len - 1;
}

static int smc_transaction(struct i2c_client *client, u8 *outbuf,
	int outbuf_size, u8 *result, int result_size)
{
	int ret;
	int retries = 2;
	MpuMsgHeader_t hdr_in, hdr_out;
	struct smc_data *data = i2c_get_clientdata(client);
	ret = mutex_lock_interruptible(&data->transaction_mutex);
	if (ret < 0)
		return ret;

	if (result_size > (I2C_SMBUS_BLOCK_MAX + 1)) {
		dev_err(&client->dev,
			"Transactation read size of %d to big, max %d\n",
			result_size, I2C_SMBUS_BLOCK_MAX + 1);
		mutex_unlock(&data->transaction_mutex);
		return -EINVAL;
	}
	hdr_in = mpu_message_header(outbuf);
	data->transaction_read_buffer = result;
	do {
		data->read_pending = 1;

		ret = smc_send(client, outbuf, outbuf_size);

		if (ret < 0) {
			data->read_pending = 0;
			continue;
		}

		ret = wait_event_interruptible_timeout(data->readq,
			data->is_ready, msecs_to_jiffies(2000));

		if (ret <= 0) {
			data->read_pending = 0;
			dev_warn(&client->dev,
				"Timeout waiting for command reply\n");
			if (ret == 0)
				ret = -ETIMEDOUT;
			continue;
		}
		if (data->rx_result < 0) {
			dev_warn(&client->dev, "%s: Rx Error %d\n", __func__,
				data->rx_result);
			ret = data->rx_result;
			continue;
		}
		hdr_out = mpu_message_header(result);
		if (hdr_in.type != hdr_out.type) {
			dev_err(&client->dev,
				"Msg in type %d differs from reply %d\n",
				hdr_in.type, hdr_out.type);
			ret = -EINVAL;
		}
	} while (ret && retries--);
	mutex_unlock(&data->transaction_mutex);
	return ret >= 0 ? 0 : ret;
}

static int set_led(struct smc_data *data, int led,
	enum led_brightness brightness)
{
	int ret = 0;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 reply[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 ledCmd[2];
	ledCmd[0] = led;
	ledCmd[1] = brightness > 0 ? 1 : 0;
	ret = mpu_create_message(msg_led, mpu_status_ok, outbuf, ledCmd, 2);

	ret = smc_transaction(data->client, outbuf, ret, reply, sizeof(reply));

	return ret;
}

static int smc_led_brightness_set(struct led_classdev *cdev,
	enum led_brightness brightness)
{
	struct smc_data *priv = dev_get_drvdata(cdev->dev->parent);

	if (strcmp(cdev->name, LED_USER_SPACE_NAME_STATUS_GREEN) == 0)
		return set_led(priv, LedStatusGreen, brightness);
	else if (strcmp(cdev->name, LED_USER_SPACE_NAME_STATUS_RED) == 0)
		return set_led(priv, LedStatusRed, brightness);
	return -EINVAL;
}

static int smc_register_led(struct smc_data *data, struct smc_led *led,
	const char *name, int id, const char *trigger)
{
	int err;

	strncpy(led->name, name, sizeof(led->name));
	led->name[sizeof(led->name) - 1] = 0;
	led->led_dev.name = led->name;
	led->led_dev.default_trigger = trigger;
	led->led_dev.brightness_set_blocking = smc_led_brightness_set;
	led->led_id = id;

	err = led_classdev_register((struct device * )&data->client->dev,
		&led->led_dev);
	if (err) {
		dev_warn(&data->client->dev, "Could not register LED %s\n",
			name);
		led->registered = 0;
	} else
		led->registered = 1;

	return err;
}

static void smc_unregister_led(struct smc_data *data, struct smc_led *led)
{
	if (led->registered)
		led_classdev_unregister(&led->led_dev);
}

static void smc_unregister_leds(struct smc_data *data)
{
	smc_unregister_led(data, &data->led_status_green);
	smc_unregister_led(data, &data->led_status_red);
}

static void smc_init_leds(struct smc_data *data)
{
	// initializing leds
	int ret = 0;
	char name[CARGOTEC_SMC_LED_MAX_NAME_LEN + 1];

	// https://www.kernel.org/doc/Documentation/leds/leds-class.txt
	// LED Device Naming
	// =================
	// Is currently of the form:
	// "devicename:colour:function"

	snprintf(name, sizeof(name), LED_USER_SPACE_NAME_STATUS_GREEN);
	ret = smc_register_led(data, &data->led_status_green, name,
		LedStatusGreen, "none");

	snprintf(name, sizeof(name), LED_USER_SPACE_NAME_STATUS_RED);
	ret = smc_register_led(data, &data->led_status_red, name, LedStatusRed,
		"none");
}

static ssize_t get_ignition_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	const u8 *ignRes;
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 ignMsg[2];

	ignMsg[0] = IgnitionDelayGet;
	ignMsg[1] = 0;
	ret = mpu_create_message(msg_ignition, mpu_status_ok, outbuf, ignMsg,
		2);
	ret = smc_transaction(client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + 2);

	if (ret < 0)
		return -EINVAL;

	ignRes = mpu_get_payload(result);
	return sprintf(buf, "%d\n", ignRes[1]);
}

static ssize_t set_ignition_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	int ignition_delay = 0;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 ignMsg[2];
	ret = kstrtoint(buf, 10, &ignition_delay);
	if (ret < 0 || ret > 10)
		return -EINVAL;

	ignMsg[0] = IgnitionDelaySet;
	ignMsg[1] = ignition_delay;
	ret = mpu_create_message(msg_ignition, mpu_status_ok, outbuf, ignMsg,
		2);
	ret = smc_transaction(client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + 2);

	return count;
}
static DEVICE_ATTR(ignition_delay, S_IWUSR | S_IRUGO, get_ignition_delay, set_ignition_delay);

static ssize_t get_debug(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", priv->debug_mode);
}

static ssize_t set_debug(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	int dbg = 0;
	struct smc_data *priv = dev_get_drvdata(dev);

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 msg[2];

	if (compare_version(priv->version, 2, 1) < 0)
		return -EPROTONOSUPPORT;

	ret = kstrtoint(buf, 10, &dbg);
	if (ret < 0 || ret > 10)
		return -EINVAL;
	msg[0] = dbg;
	ret = mpu_create_message(msg_debug, mpu_status_ok, outbuf, msg, 1);
	ret = smc_transaction(client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + 1);
	if (ret > 0)
		priv->debug_mode = dbg;
	return count;
}

static DEVICE_ATTR(debug, S_IWUSR | S_IRUGO, get_debug, set_debug);

static ssize_t command(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	MpuMsgHeader_t hdr;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	if (sysfs_streq(buf, "reset")) {

		dev_warn(dev, "Reset MCU\n");
		ret = mpu_create_message(msg_reset, mpu_status_ok, outbuf, 0,
			0);
		ret = smc_transaction(client, outbuf, ret, result,
			sizeof(result));
		hdr = mpu_message_header(result);
		if (hdr.replyStatus == mpu_status_ok)
			dev_info(dev, "RESET in progress\n");
		else
			dev_info(dev, "Command %s failed with %d\n", buf,
				hdr.replyStatus);
	} else if (sysfs_streq(buf, "defaults")) {
		ret = mpu_create_message(msg_defaults, mpu_status_ok, outbuf, 0,
			0);
		ret = smc_transaction(client, outbuf, ret, result,
			sizeof(result));
	} else
		return -EINVAL;
	if (ret < 0)
		return ret;
	return count;
}
static DEVICE_ATTR(command, S_IWUSR, 0, command);

static int fetch_firmware_version(struct smc_data *priv)
{
	int ret = 0;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));

	ret = mpu_create_message(msg_version, mpu_status_ok, outbuf, 0, 0);
	ret = smc_transaction(priv->client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + sizeof(MpuVersionHeader_t));
	if (ret < 0)
		return ret;

	priv->version = *mpu_get_version_header(mpu_get_payload(result));
	return 0;
}

static int send_async(struct smc_data *priv)
{
	int ret = 0;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	ret = mpu_create_message(msg_async, mpu_status_ok, outbuf, 0, 0);
	ret = smc_transaction(priv->client, outbuf, ret, result, ret);

	if (ret < 0)
		return ret;

	return 0;
}

static int send_reboot(struct smc_data *priv)
{
	int ret = 0;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	ret = mpu_create_message(msg_reboot, mpu_status_ok, outbuf, 0, 0);
	ret = smc_transaction(priv->client, outbuf, ret, result, ret);

	if (ret < 0)
		return ret;

	return 0;
}

static ssize_t smc_get_sensors(struct smc_data *priv)
{
	int ret = 0;
	SensorMsg_t *s;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));

	ret = mpu_create_message(msg_sensors, mpu_status_ok, outbuf, 0, 0);
	ret = smc_transaction(priv->client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + 2);

	if (ret < 0)
		return -EINVAL;

	s = (SensorMsg_t*) mpu_get_payload(result);
	priv->sensor.voltage_in = le32_to_cpu(s->voltage_in);
	priv->sensor.current_sys = le32_to_cpu(s->current_sys);
	priv->sensor.voltage_scap = le32_to_cpu(s->voltage_scap);
	priv->sensor.power_good = le16_to_cpu(s->power_good);
	return 0;
}

static ssize_t smc_get_psy(struct smc_data *priv)
{
	int ret = 0;
	PowerSupplyMsg_t *s;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));

	ret = mpu_create_message(msg_power_supply, mpu_status_ok, outbuf, 0, 0);
	ret = smc_transaction(priv->client, outbuf, ret, result,
		mpu_max_message_size);

	if (ret < 0)
		return -EINVAL;

	s = (PowerSupplyMsg_t*) mpu_get_payload(result);
	priv->powersupplies.voltage_in = le32_to_cpu(s->voltage_in);
	priv->powersupplies.dcout_sense1 = le16_to_cpu(s->dcout_sense1);
	priv->powersupplies.dcout_sense2 = le16_to_cpu(s->dcout_sense2);
	priv->powersupplies.gpo1_sense1 = le16_to_cpu(s->gpo1_sense1);
	priv->powersupplies.gpo1_sense2 = le16_to_cpu(s->gpo1_sense2);
	priv->powersupplies.dcout_online1 = le16_to_cpu(s->dcout_online1);
	priv->powersupplies.dcout_online2 = le16_to_cpu(s->dcout_online2);
	return 0;
}

static ssize_t smc_get_startup_cause(struct smc_data *priv)
{
	int ret = 0;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	const u8 *res;

	if (priv->start_cause != msg_init_none)
		return 0;
	ret = mpu_create_message(msg_init, mpu_status_ok, outbuf, 0, 0);
	ret = smc_transaction(priv->client, outbuf, ret, result,
		mpu_max_message_size);

	if (ret < 0)
		return -EINVAL;

	res = mpu_get_payload(result);
	priv->start_cause = le32_to_cpu(*((u32* )res));
	return 0;
}

static ssize_t get_firmware_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	return sprintf(buf, "%d.%d\n", priv->version.ver_major,
		priv->version.ver_minor);
}

static ssize_t get_scap_volt(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int res;
	struct smc_data *priv = dev_get_drvdata(dev);
	if (compare_version(priv->version, 2, 2) >= 0) {
		res = smc_get_sensors(priv);
		if (res < 0)
			return res;
	}
	return sprintf(buf, "%d\n", priv->sensor.voltage_scap);
}

static DEVICE_ATTR(firmware_version, S_IRUGO, get_firmware_version, 0);

static DEVICE_ATTR(scap_voltage, S_IRUGO, get_scap_volt, 0);

static ssize_t get_startup_cause(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int res;
	struct smc_data *priv = dev_get_drvdata(dev);
	if (compare_version(priv->version, 3, 1) >= 0) {
		res = smc_get_startup_cause(priv);
		if (res < 0)
			return res;
	}
	switch (priv->start_cause) {
	case msg_init_ignition:
		return sprintf(buf, "ignition\n");
		break;

	case msg_init_gpi:
		return sprintf(buf, "gpi-wakeup\n");
		break;

	case msg_init_acc1:
		return sprintf(buf, "acc1\n");
		break;

	case msg_init_acc2:
		return sprintf(buf, "acc2\n");
		break;

	case msg_init_rtc:
		return sprintf(buf, "rtc\n");
		break;

	case msg_init_cold:
		return sprintf(buf, "power\n");
		break;

	default:
		break;

	}
	return sprintf(buf, "unknown\n");
}

static DEVICE_ATTR(startup_cause, S_IRUGO, get_startup_cause, 0);

static void smc_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct smc_data *priv = gpiochip_get_data(chip);
	struct i2c_client *client = priv->client;
	int ret = 0;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 msg[3];

	msg[2] = value;
	msg[1] = offset;
	msg[0] = GpoSet;

	ret = mpu_create_message(msg_gpo, mpu_status_ok, outbuf, msg, 3);
	ret = smc_transaction(client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + 3);
}

static int smc_get(struct gpio_chip *chip, unsigned offset)
{
	MpuMsgHeader_t hdr;
	int ret = 0;
	struct smc_data *priv = gpiochip_get_data(chip);
	struct i2c_client *client = priv->client;
	const u8* resultPtr;
	int value;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 result[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 msg[3];

	msg[0] = GpoGet;
	msg[1] = offset;
	msg[2] = 0;

	ret = mpu_create_message(msg_gpo, mpu_status_ok, outbuf, msg, 3);
	ret = smc_transaction(client, outbuf, ret, result,
		sizeof(MpuMsgHeader_t) + 3);
	if (ret < 0)
		return ret;
	resultPtr = mpu_get_payload(result);
	hdr = mpu_message_header(result);
	if (hdr.replyStatus != mpu_status_ok)
		return -EIO;
	value = resultPtr[2];
	return value;
}

static int smc_get_direction(struct gpio_chip *chip, unsigned offset)
{
	switch (offset) {
	case 0:
	case 1:
		return GPIOF_DIR_OUT;
		break;

	case 2:
	case 3:
		return GPIOF_DIR_IN;
		break;

	default:
		return -EINVAL;
	}
}

static int smc_direction_input(struct gpio_chip *chip, unsigned offset)
{
	switch (offset) {
	case 0:
	case 1:
		return -EINVAL;
		break;

	case 2:
	case 3:
		return 0;
		break;

	default:
		return -EINVAL;
		break;
	}
}

static int smc_direction_output(struct gpio_chip *chip, unsigned offset,
	int value)
{
	switch (offset) {
	case 0:
	case 1:
		return 0;
		break;

	case 2:
	case 3:
		return -EINVAL;
		break;

	default:
		return -EINVAL;
		break;
	}
}

static void smc_alert_handler(struct work_struct *work)
{
	int res;
	struct smc_data
	*priv = container_of(work, struct smc_data, alert_work);
	MpuMsgHeader_t hdr;
	NotificationMsg_t notify;
	res = smc_recv(priv->client);
	if (res < 0) {
		dev_err(&priv->client->dev,
			"%s: Receive error %d, pending op %s\n", __func__, res,
			priv->read_pending ? "yes" : "no");
		return;
	}
	priv->rx_result = res;
	hdr = mpu_message_header(priv->rx_buffer);
	if (hdr.type == msg_notify) {
		notify = notify_get_message(mpu_get_payload(priv->rx_buffer));
		dev_info(&priv->client->dev,
			"%s: Notification for %d with value %d\n", __func__,
			notify.id, notify.value);
		switch (notify.id) {
		case GpiIgnition:
			if (priv->gpio_dev[2]
				&& (gpiod_get_direction(priv->gpio_descs[2]) > 0)) {
				kobject_uevent(&priv->gpio_dev[2]->kobj,
					KOBJ_CHANGE);
			}
			break;

		case GpiWakeup:
			if (priv->gpio_dev[3]
				&& (gpiod_get_direction(priv->gpio_descs[3]) > 0)) {
				kobject_uevent(&priv->gpio_dev[3]->kobj,
					KOBJ_CHANGE);
			}
			break;

		case PowerFail:
			dev_warn(&priv->client->dev, "%s: POWER FAIL!!",
				__func__);	// Todo send event
			kill_cad_pid(SIGINT, 1);
			break;

		default:
			dev_warn(&priv->client->dev,
				"%s: Unexpected notification id %d received\n",
				__func__, notify.id);
		}
	} else if (hdr.type < msg_max && priv->read_pending) {
		dev_dbg(&priv->client->dev, "Transaction IRQ\n");
		priv->is_ready = true;
		memcpy(priv->transaction_read_buffer, priv->rx_buffer,
			priv->rx_result);
		priv->read_pending = 0;
		wake_up_interruptible(&priv->readq);
	} else {
		dev_warn(&priv->client->dev,
			"%s: Unexpected message %d received\n", __func__,
			hdr.type);
	}

}

static irqreturn_t smc_irq(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct smc_data *priv = i2c_get_clientdata(client);
	queue_work(priv->wq, &priv->alert_work);
	return IRQ_HANDLED;
}

static int smc_rtc_read(struct device *dev, struct rtc_time *rtctime)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	int status;
	RtcMsg_t msg;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 inbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	int sz = rtc_create_message(msg_rtc_get_time,
		outbuf + sizeof(MpuMsgHeader_t), 0);
	sz = mpu_create_message(msg_rtc, 0, outbuf, 0, sz);
	status = smc_transaction(priv->client, outbuf, sz, inbuf,
		sizeof(inbuf));
	if (status < 0)
		return status;

	rtc_get_payload(mpu_get_payload(inbuf), &msg);
	rtctime->tm_sec = bcd2bin(msg.tm_sec);
	rtctime->tm_min = bcd2bin(msg.tm_min);
	rtctime->tm_hour = bcd2bin(msg.tm_hour);
	rtctime->tm_mday = bcd2bin(msg.tm_mday);
	rtctime->tm_mon = bcd2bin(msg.tm_mon) - 1;
	rtctime->tm_year = bcd2bin(msg.tm_year) + 100;
	rtctime->tm_wday = msg.tm_wday % 7;
	rtctime->tm_yday = 0;
	rtctime->tm_isdst = 0;
	dev_dbg(dev, "RTC time read: %02d.%02d.%02d : %02d.%02d.%02d, wd=%d\n",
		(rtctime->tm_hour), (rtctime->tm_min), (rtctime->tm_sec),
		(rtctime->tm_mday), (rtctime->tm_mon), (rtctime->tm_year),
		rtctime->tm_wday);
	return 0;
}

static int smc_send_alarm(struct smc_data *priv)
{
	int status, sz;
	RtcAlarm_t msg;
	struct rtc_time rtc_now, rtc_alarm;
	time64_t now;
	RtcMsgType_t msg_type;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 inbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	if (priv->alarm_in_seconds == 0)
		msg_type = msg_rtc_cancel_alarm;
	else {
		msg_type = msg_rtc_set_alarm;

		status = rtc_read_time(priv->rtc, &rtc_now);
		if (status < 0)
			return status;

		now = rtc_tm_to_time64(&rtc_now);
		rtc_time64_to_tm(now + priv->alarm_in_seconds, &rtc_alarm);

		msg.tm_sec = bin2bcd(rtc_alarm.tm_sec);
		msg.tm_min = bin2bcd(rtc_alarm.tm_min);
		msg.tm_hour = bin2bcd(rtc_alarm.tm_hour);
		msg.tm_mday = bin2bcd(rtc_alarm.tm_mday);
		msg.pending = 0;
		msg.enable = 1;
		dev_info(&priv->client->dev, "Alarm set at %02d.%02d.%02d %02d:%02d:%02d\n",
			rtc_alarm.tm_mday, rtc_alarm.tm_mon + 1,
			rtc_alarm.tm_year - 100, rtc_alarm.tm_hour,
			rtc_alarm.tm_min, rtc_alarm.tm_sec);
	}
	sz = rtc_create_alarm_message(msg_type, outbuf + sizeof(MpuMsgHeader_t),
		&msg);
	sz = mpu_create_message(msg_rtc, 0, outbuf, 0, sz);
	status = smc_transaction(priv->client, outbuf, sz, inbuf,
		sizeof(inbuf));
	return status;
}

static int smc_rtc_set(struct device *dev, struct rtc_time *rtctime)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	int status, sz;
	RtcMsg_t msg;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 inbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	msg.tm_sec = bin2bcd(rtctime->tm_sec);
	msg.tm_min = bin2bcd(rtctime->tm_min);
	msg.tm_hour = bin2bcd(rtctime->tm_hour);
	msg.tm_mday = bin2bcd(rtctime->tm_mday);
	msg.tm_mon = bin2bcd(rtctime->tm_mon + 1);
	msg.tm_year = bin2bcd(rtctime->tm_year - 100);
	msg.tm_wday = (!rtctime->tm_wday) ? 7 : rtctime->tm_wday;
	msg.sub_second = 0;
	dev_info(dev, "RTC set to %02d.%02d.%02d %02d:%02d:%02d\n", rtctime->tm_mday,
		rtctime->tm_mon + 1, rtctime->tm_year - 100, rtctime->tm_hour,
		rtctime->tm_min, rtctime->tm_sec);
	sz = rtc_create_message(msg_rtc_set_time,
		outbuf + sizeof(MpuMsgHeader_t), &msg);
	sz = mpu_create_message(msg_rtc, 0, outbuf, 0, sz);
	status = smc_transaction(priv->client, outbuf, sz, inbuf,
		sizeof(inbuf));
	return status;
}

static ssize_t smc_set_wakeup(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	int status;
	u32 val;

	if (compare_version(priv->version, 3, 0) < 0)
		return -EINVAL;
	if (strncmp(buf, "cancel", 6) == 0) {
		val = 0;
	} else {
		status = kstrtouint(buf, 10, &val);
		if (status < 0)
			return status;
		else if (val > wakeup_max_seconds) {
			dev_err(dev, "%s: Can not set alarm beyond %d days\n",
				__func__, wakeup_max_seconds / (24 * 3600));
			return -EINVAL;
		}
	}
	priv->alarm_in_seconds = val;
	if (priv->alarm_in_seconds == 0) {
		if (priv->alarm_pending) {
			priv->alarm_pending = 0;
			dev_info(dev, "Cancelled pending alarm\n");
		} else {
			status = smc_send_alarm(priv);
			dev_info(dev, "Cancelled alarm\n");
		}
	} else {
		if (priv->rtc_updated) {
			priv->alarm_pending = 0;
			status = smc_send_alarm(priv);
		} else {
			priv->alarm_pending = 1;
			dev_info(dev, "Alarm pending %d seconds\n", priv->alarm_in_seconds);
		}
	}
	if (status < 0)
		return status;
	else
		return count;
}

static ssize_t smc_get_wakeup(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	int status, sz;
	MpuMsgHeader_t hdr;
	RtcAlarm_t alarm;

	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 inbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	if (compare_version(priv->version, 3, 0) < 0)
		return -EINVAL;
	if (priv->alarm_pending)
		return sprintf(buf, "Alarm pending at %d seconds\n", priv->alarm_in_seconds);
	sz = rtc_create_alarm_message(msg_rtc_get_alarm,
		outbuf + sizeof(MpuMsgHeader_t), 0);
	sz = mpu_create_message(msg_rtc, 0, outbuf, 0, sz);
	status = smc_transaction(priv->client, outbuf, sz, inbuf,
		sizeof(inbuf));
	if (status < 0)
		return status;
	hdr = mpu_message_header(inbuf);
	if (hdr.replyStatus == mpu_status_ok) {
		rtc_alarm_get_payload(mpu_get_payload(inbuf), &alarm);
		if (alarm.pending)
			return sprintf(buf, "%d %02d:%02d:%02d\n",
				bcd2bin(alarm.tm_mday), bcd2bin(alarm.tm_hour),
				bcd2bin(alarm.tm_min), bcd2bin(alarm.tm_sec));
		else
			return sprintf(buf, "No alarm pending\n");
	} else
		return -EINVAL;
}

static DEVICE_ATTR(wakeup_in_seconds, S_IWUSR | S_IRUGO, smc_get_wakeup , smc_set_wakeup);

static ssize_t smc_set_start_options(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	int status, sz;
	MpuMsgHeader_t hdr;
	InitMessage_t init;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 inbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	init.event_mask = 0;
	init.set_mask_cmd = 1;
	if (compare_version(priv->version, 3, 1) < 0)
		return -EINVAL;

	if (strstr(buf, "gpi"))
		init.event_mask |= msg_init_gpi;
	if (strstr(buf, "acc1"))
		init.event_mask |= msg_init_acc1;
	if (strstr(buf, "acc2"))
		init.event_mask |= msg_init_acc2;

	if (init.event_mask == 0 && strncmp(buf, "none", 4) != 0)
		return -EINVAL;

	init.set_mask_cmd = cpu_to_le16(init.set_mask_cmd);
	init.event_mask = cpu_to_le16(init.event_mask);

	sz = mpu_create_message(msg_set_start_options, 0, outbuf, (u8*) &init,
		sizeof(InitMessage_t));
	status = smc_transaction(priv->client, outbuf, sz, inbuf,
		sizeof(inbuf));
	if (status < 0)
		return status;
	hdr = mpu_message_header(inbuf);
	if (hdr.replyStatus != mpu_status_ok)
		return -EINVAL;

	return count;
}

static ssize_t smc_get_start_options(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct smc_data *priv = dev_get_drvdata(dev);
	int status, sz;
	MpuMsgHeader_t hdr;
	InitMessage_t init;
	u8 outbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	u8 inbuf[mpu_max_message_size] __attribute__((aligned(0x10)));
	if (compare_version(priv->version, 3, 1) < 0)
		return sprintf(buf, "not supported\n");
	init.set_mask_cmd = 0;
	init.event_mask = 0;
	sz = mpu_create_message(msg_set_start_options, 0, outbuf, (u8*) &init,
		sizeof(InitMessage_t));
	status = smc_transaction(priv->client, outbuf, sz, inbuf,
		sizeof(inbuf));
	if (status < 0)
		return status;
	hdr = mpu_message_header(inbuf);
	if (hdr.replyStatus != mpu_status_ok)
		return -ENOENT;

	memcpy(&init, mpu_get_payload(inbuf), sizeof(init));
	init.event_mask = le16_to_cpu(init.event_mask);
	dev_info(dev, "Startup mask =0x%x\n", init.event_mask);
	if (init.event_mask & msg_init_gpi)
		strcpy(buf, "gpi");
	if (init.event_mask & msg_init_acc1) {
		if (strlen(buf) > 0)
			strcat(buf, ",");
		strcat(buf, "acc1");
	}
	if (init.event_mask & msg_init_acc2) {
		if (strlen(buf) > 0)
			strcat(buf, ",");
		strcat(buf, "acc2");
	}
	strcat(buf, "\n");

	return strlen(buf);
}

static DEVICE_ATTR(start_options, S_IWUSR | S_IRUGO, smc_get_start_options , smc_set_start_options);

static struct attribute *smc_attrs[] = {
	&dev_attr_ignition_delay.attr, &dev_attr_firmware_version.attr,
	&dev_attr_command.attr, &dev_attr_debug.attr,
	&dev_attr_scap_voltage.attr, &dev_attr_wakeup_in_seconds.attr,
	&dev_attr_startup_cause.attr, &dev_attr_start_options.attr,
	NULL };

static const struct attribute_group smc_attr_group = { .attrs = smc_attrs, };
static struct rtc_class_ops smc_rtc_ops = {
	.read_time = smc_rtc_read, .set_time = smc_rtc_set, };

/*
 * Power supply
 */

static enum power_supply_property ps_props[] = {
	POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX, POWER_SUPPLY_PROP_CURRENT_NOW, };

static int smc_ps_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int err;
	struct smc_data *priv = dev_get_drvdata(psy->dev.parent);
	unsigned long jiffies_passed = jiffies - priv->sensor_jiffies;
	if (jiffies_to_msecs(jiffies_passed) > 100
		|| priv->sensor_jiffies == 0) {
		priv->sensor_jiffies = jiffies;
		err = smc_get_sensors(priv);
		if (err)
			return err;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = priv->sensor.power_good ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		if (priv->sensor.power_good) {
			if (priv->sensor.voltage_scap >= 5000)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = priv->sensor.voltage_in * 1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = priv->sensor.current_sys * 1000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = 6500000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 36000000;
		break;

	default:
		return -ENOENT;

	}

	return 0;
}
static const struct power_supply_desc ps_desc = { .name = "mains", .type =
	POWER_SUPPLY_TYPE_MAINS, .properties = ps_props, .num_properties =
	ARRAY_SIZE(ps_props), .get_property = smc_ps_get_property, };

static int smc_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int err;
	struct smc_data *priv = dev_get_drvdata(psy->dev.parent);

	unsigned long jiffies_passed = jiffies - priv->psy_jiffies;
	if (jiffies_to_msecs(jiffies_passed) > 100 || priv->psy_jiffies == 0) {
		priv->psy_jiffies = jiffies;
		err = smc_get_psy(priv);
		if (err)
			return err;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = priv->powersupplies.voltage_in * 1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (psy == priv->psy_dcout1)
			val->intval = priv->powersupplies.dcout_sense1 * 1000;
		else if (psy == priv->psy_dcout2)
			val->intval = priv->powersupplies.dcout_sense2 * 1000;
		else if (psy == priv->psy_gpo1)
			val->intval = priv->powersupplies.gpo1_sense1 * 1000;
		else if (psy == priv->psy_gpo2)
			val->intval = priv->powersupplies.gpo1_sense2 * 1000;
		else
			return -ENOENT;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		if (psy == priv->psy_dcout1)
			val->intval = priv->powersupplies.dcout_online1;
		else if (psy == priv->psy_dcout2)
			val->intval = priv->powersupplies.dcout_online2;
		else
			return -ENOENT;
		break;

	default:
		return -ENOENT;

	}

	return 0;
}
static enum power_supply_property power_props_dcout[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_ONLINE, };

static enum power_supply_property power_props_gpo[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_CURRENT_NOW, };

static const struct power_supply_desc ps_desc_gpo1 = {
	.name = "gpo1", .type = POWER_SUPPLY_TYPE_MAINS, .properties =
		power_props_gpo, .num_properties = ARRAY_SIZE(power_props_gpo),
	.get_property = smc_power_get_property, };
static const struct power_supply_desc ps_desc_gpo2 = {
	.name = "gpo2", .type = POWER_SUPPLY_TYPE_MAINS, .properties =
		power_props_gpo, .num_properties = ARRAY_SIZE(power_props_gpo),
	.get_property = smc_power_get_property, };
static const struct power_supply_desc ps_desc_dcout1 = {
	.name = "dcout1", .type = POWER_SUPPLY_TYPE_MAINS, .properties =
		power_props_dcout, .num_properties = ARRAY_SIZE(
		power_props_dcout), .get_property = smc_power_get_property, };
static const struct power_supply_desc ps_desc_dcout2 = {
	.name = "can", .type = POWER_SUPPLY_TYPE_MAINS, .properties =
		power_props_dcout, .num_properties = ARRAY_SIZE(
		power_props_dcout), .get_property = smc_power_get_property, };

static int smc_setup_gpio(struct smc_data *priv)
{
	const char *gpio_names[MAX_GPIOS];
	int err, n;
	struct device_node *np = priv->client->dev.of_node;
	if (np == 0)
		return -EINVAL;
	n = of_property_count_strings(np, "smc-gpios");
	if (n <= 0)
		return -ENOKEY;
	err = of_property_read_string_array(np, "smc-gpios", gpio_names, n);
	if (err < 0) {
		dev_warn(&priv->client->dev, "%s: No DT gpios\n", __func__);
		return err;
	}

	priv->gpio_ctl.ngpio = n;

	err = of_property_read_u8_array(np, "smc-gpio-input",
		priv->gpio_is_input, priv->gpio_ctl.ngpio);
	if (err < 0) {
		dev_warn(&priv->client->dev,
			"%s: DT gpios io defs do not match\n", __func__);
		return err;
	}
	priv->gpio_ctl.label = "smc-gpio";
	priv->gpio_ctl.owner = THIS_MODULE;
	priv->gpio_ctl.can_sleep = true;
	priv->gpio_ctl.parent = &priv->client->dev;
	priv->gpio_ctl.base = 300;
	priv->gpio_ctl.get_direction = smc_get_direction;
	priv->gpio_ctl.direction_input = smc_direction_input;
	priv->gpio_ctl.direction_output = smc_direction_output;
	priv->gpio_ctl.set = smc_set;
	priv->gpio_ctl.get = smc_get;

	err = devm_gpiochip_add_data(&priv->client->dev, &priv->gpio_ctl, priv);

	for (n = 0; n < priv->gpio_ctl.ngpio; n++) {
		priv->gpio_descs[n] = gpiochip_request_own_desc(&priv->gpio_ctl,
			n, gpio_names[n]);
		gpiod_export(priv->gpio_descs[n], true);
		priv->gpio_dev[n] = gpiod_to_dev(priv->gpio_descs[n]);
		dev_info(&priv->client->dev, "Add GPIO %s from DT\n",
			gpio_names[n]);
		if (priv->gpio_is_input[n])
			gpiod_direction_input(priv->gpio_descs[n]);
	}
	return err;
}

static int smc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	int retries;
	const char * irq_name = { "mcu-irq" };
	struct smc_data *priv;
	struct power_supply_config psy_cfg = { };

	priv = devm_kzalloc(&client->dev, sizeof(struct smc_data), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit;
	}

	priv->rx_buffer = devm_kmalloc(&client->dev, 256, GFP_KERNEL);
	priv->driver_version.ver_major = fwVersionMajor;
	priv->driver_version.ver_minor = fwVersionMinor;
	priv->client = client;
	priv->client->dev = client->dev;
	i2c_set_clientdata(client, priv);
	dev_info(&client->dev, "Driver version %s\n", DRIVER_VERSION);
	mutex_init(&priv->transaction_mutex);
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &smc_attr_group);
	if (err)
		goto exit;

	init_waitqueue_head(&priv->readq);
	priv->wq = alloc_ordered_workqueue("smc_wq", 0);
	INIT_WORK(&priv->alert_work, smc_alert_handler);

	// get interrupts
	priv->irq = client->irq;
	if (client->irq > 0) {
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			smc_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, irq_name, client);
		if (err) {
			dev_err(&client->dev, "unable to request %s\n",
				irq_name);
			goto exit;
		}

	}

	for (retries = 0; retries < 3; retries++) {
		err = fetch_firmware_version(priv);
		if (err == 0)
			break;
	}
	if (err < 0) {
		dev_err(&client->dev, "no firmware version available\n");
		goto exit_sysfsg;
	} else {
		dev_info(&client->dev, "MCU FW %d.%d\n",
			priv->version.ver_major, priv->version.ver_minor);
		if (of_property_read_bool(client->dev.of_node, "status-led")) {
			dev_info(&client->dev, "Status LED supported\n");
			smc_init_leds(priv);
		}
		if (of_property_read_bool(client->dev.of_node, "has-rtc")) {
			if (priv->version.ver_major >= 3) {
				dev_info(&client->dev,
					"RTC supported with alarm\n");
				if (priv->irq)
					device_init_wakeup(&client->dev, 1);
			} else
				dev_info(&client->dev, "RTC supported\n");
			priv->rtc = devm_rtc_device_register(&client->dev,
				"mcu", &smc_rtc_ops, THIS_MODULE);
		}
	}

	if ((err = smc_setup_gpio(priv)) < 0)
		dev_warn(&client->dev, "GPIO setup error %d\n", err);
	err = 0;

	if (priv->version.ver_major >= 2) {
		dev_info(&client->dev, "Enabling async notifications\n");
		err = send_async(priv);
	}
	if (compare_version(priv->version, 2, 2) >= 0) {
		psy_cfg.of_node = client->dev.of_node;
		psy_cfg.drv_data = priv;
		priv->psy = devm_power_supply_register(&client->dev, &ps_desc,
			&psy_cfg);
		if (IS_ERR(priv->psy)) {
			dev_err(&client->dev,
				"Failed to create power supply\n");
			err = PTR_ERR(priv->psy);
			goto exit_sysfsg;
		}
	}
	if (compare_version(priv->version, 3, 1) >= 0) {
		psy_cfg.of_node = client->dev.of_node;
		psy_cfg.drv_data = priv;
		priv->psy_dcout1 = devm_power_supply_register(&client->dev,
			&ps_desc_dcout1, &psy_cfg);
		priv->psy_dcout2 = devm_power_supply_register(&client->dev,
			&ps_desc_dcout2, &psy_cfg);
		priv->psy_gpo1 = devm_power_supply_register(&client->dev,
			&ps_desc_gpo1, &psy_cfg);
		priv->psy_gpo2 = devm_power_supply_register(&client->dev,
			&ps_desc_gpo2, &psy_cfg);
		if (IS_ERR(priv->psy_dcout1) || IS_ERR(priv->psy_dcout2)
			|| IS_ERR(priv->psy_gpo1) || IS_ERR(priv->psy_gpo2)) {
			dev_err(&client->dev,
				"Failed to create power supply\n");
			err = -ENOMEM;
			goto exit_sysfsg;
		}
	}

	exit: return err;
	exit_sysfsg: cancel_work_sync(&priv->alert_work);
	sysfs_remove_group(&client->dev.kobj, &smc_attr_group);
	return err;
}

static int smc_remove(struct i2c_client *client)
{
	int n;
	struct smc_data *priv = i2c_get_clientdata(client);
	cancel_work_sync(&priv->alert_work);
	if (priv->version.ver_major < 1)
		smc_unregister_leds(priv);

	sysfs_remove_group(&client->dev.kobj, &smc_attr_group);
	for (n = 0; n < priv->gpio_ctl.ngpio; n++) {
		gpiod_unexport(priv->gpio_descs[n]);
		gpiochip_free_own_desc(priv->gpio_descs[n]);
	}
	destroy_workqueue(priv->wq);
	return 0;
}

static void smc_shutdown(struct i2c_client *client)
{
	struct smc_data *priv = i2c_get_clientdata(client);
	int ret;
	if (compare_version(priv->version, 3, 0) >= 0 && priv->alarm_pending) {
		ret = smc_send_alarm(priv);
		if (ret < 0)
			dev_warn(&client->dev, "%s: Failed [%d] to send pending alarm\n",
				__func__, ret);
		else
			dev_info(&client->dev, "%s: Send pending alarm @%d seconds\n",
				__func__, priv->alarm_in_seconds);
		priv->alarm_pending = 0;
	}
	ret = send_reboot(priv);
	if (ret < 0)
		dev_err(&client->dev, "%s: Failed to call SMC\n", __func__);
	cancel_work_sync(&priv->alert_work);
}

#ifdef CONFIG_PM_SLEEP
static int smc_suspend(struct device *dev)
{
	struct smc_data *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
	enable_irq_wake(priv->irq);

	return 0;
}

static int smc_resume(struct device *dev)
{
	struct smc_data *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
	disable_irq_wake(priv->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(smc_pm_ops, smc_suspend, smc_resume);

static const struct of_device_id of_smc_match[] = { { .compatible =
	"datarespons,cargotec-gw-mcu" }, { /* Sentinel */} };

static struct i2c_device_id smc_id[] = { { "cargotec-gw-mcu", 0 }, { } };

MODULE_DEVICE_TABLE( i2c, smc_id);

static struct i2c_driver ct_smc_driver = { .driver = {
	.owner = THIS_MODULE, .name = "cargotec_gw_mcu", .of_match_table =
		of_smc_match, .pm = &smc_pm_ops, },

.id_table = smc_id, .probe = smc_probe, .remove = smc_remove, .shutdown =
	smc_shutdown, };

module_i2c_driver( ct_smc_driver);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Cargotec GW System Management Controller");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
