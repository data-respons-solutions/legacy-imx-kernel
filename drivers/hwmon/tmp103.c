/* Texas Instruments tmp103 SMBus temperature sensor driver
 *
 * Copyright (C) 2010 Steven King <sfking@fdwdc.com>
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
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/thermal.h>
#include <linux/of.h>

#define	DRIVER_NAME "tmp103"

#define	TMP103_TEMP_REG			0x00
#define	TMP103_CONF_REG			0x01
#define	TMP103_TLOW_REG			0x02
#define	TMP103_THIGH_REG		0x03

#define TMP103_CONF_SD		0x0
#define TMP103_CONF_RUN		0x2
#define TMP103_CONF_ONESHOT	0x1

struct tmp103 {
	struct device *hwmon_dev;
	struct thermal_zone_device *tz;
	struct mutex lock;
	u8 config_orig;
	unsigned long last_update;
	int temp[3];
};

static const u8 tmp103_reg[] = {
	TMP103_TEMP_REG,
	TMP103_TLOW_REG,
	TMP103_THIGH_REG,
};

static int tmp103_read_reg(struct i2c_client *client, int reg)
{
	int status = i2c_smbus_write_byte(client, reg);
	if (status)
		return status;
	return i2c_smbus_read_byte(client);

}

static int tmp103_write_reg(struct i2c_client *client, int reg, u8 val)
{
	int status = i2c_smbus_write_byte(client, reg);
	if (status)
		return status;
	return i2c_smbus_write_byte_data(client, reg, val);

}

static inline int tmp103_reg_to_mC(s8 val)
{
	return val * 1000;
}

static inline s8 tmp103_mC_to_reg(int val)
{
	return val / 1000;
}

static struct tmp103 *tmp103_update_device(struct i2c_client *client)
{
	struct tmp103 *tmp103 = i2c_get_clientdata(client);
	int status;
	mutex_lock(&tmp103->lock);
	if (time_after(jiffies, tmp103->last_update + HZ / 3)) {

		status = tmp103_read_reg(client, TMP103_TEMP_REG);
		if (status > -1)
			tmp103->temp[0] = tmp103_reg_to_mC(status);

		status = tmp103_read_reg(client, TMP103_TEMP_REG);
		if (status > -1)
			tmp103->temp[1] = tmp103_reg_to_mC(status);

		status = tmp103_read_reg(client, TMP103_TEMP_REG);
		if (status > -1)
			tmp103->temp[2] = tmp103_reg_to_mC(status);

		tmp103->last_update = jiffies;
	}

	mutex_unlock(&tmp103->lock);
	return tmp103;
}

static int tmp103_read_temp(void *dev, long *temp)
{
	struct tmp103 *tmp103 = tmp103_update_device(to_i2c_client(dev));

	*temp = tmp103->temp[0];

	return 0;
}

static ssize_t tmp103_show_temp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct tmp103 *tmp103 = tmp103_update_device(to_i2c_client(dev));

	return sprintf(buf, "%d\n", tmp103->temp[sda->index]);
}

static ssize_t tmp103_set_temp(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp103 *tmp103 = i2c_get_clientdata(client);
	long val;
	int status;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;
	val = clamp_val(val, -256000, 255000);

	mutex_lock(&tmp103->lock);
	tmp103->temp[sda->index] = val;
	status = tmp103_write_reg(client, tmp103_reg[sda->index], tmp103_mC_to_reg(val));
	mutex_unlock(&tmp103->lock);
	return status ? : count;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, tmp103_show_temp, NULL , 0);

static struct attribute *tmp103_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};

static const struct attribute_group tmp103_attr_group = {
	.attrs = tmp103_attributes,
};

#define tmp103_CONFIG  (tmp103_CONF_TM | tmp103_CONF_EM | tmp103_CONF_CR1)
#define tmp103_CONFIG_RD_ONLY (tmp103_CONF_R0 | tmp103_CONF_R1 | tmp103_CONF_AL)

static int tmp103_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct tmp103 *tmp103;
	int status;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
			"adapter doesn't support I2C_FUNC_SMBUS_BYTE transactions\n");
		return -ENODEV;
	}

	tmp103 = devm_kzalloc(&client->dev, sizeof(*tmp103), GFP_KERNEL);
	if (!tmp103)
		return -ENOMEM;

	i2c_set_clientdata(client, tmp103);

	status = tmp103_read_reg(client, TMP103_CONF_REG);
	if (status < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return status;
	}
	tmp103->config_orig = status;
	tmp103->last_update = jiffies - HZ;
	mutex_init(&tmp103->lock);

	status = sysfs_create_group(&client->dev.kobj, &tmp103_attr_group);
	if (status) {
		dev_dbg(&client->dev, "could not create sysfs files\n");
		return status;
	}
	tmp103->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(tmp103->hwmon_dev)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		status = PTR_ERR(tmp103->hwmon_dev);
		goto fail_remove_sysfs;
	}

	tmp103->tz = thermal_zone_of_sensor_register(&client->dev, 0,
						     &client->dev,
						     tmp103_read_temp, NULL);
	if (IS_ERR(tmp103->tz))
		tmp103->tz = NULL;

	dev_info(&client->dev, "initialized\n");

	return 0;

fail_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &tmp103_attr_group);
	return status;
}

static int tmp103_remove(struct i2c_client *client)
{
	struct tmp103 *tmp103 = i2c_get_clientdata(client);

	thermal_zone_of_sensor_unregister(&client->dev, tmp103->tz);
	hwmon_device_unregister(tmp103->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &tmp103_attr_group);

	return 0;
}

#ifdef CONFIG_PM
static int tmp103_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	return tmp103_write_reg(client, TMP103_CONF_REG, TMP103_CONF_SD);
}

static int tmp103_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return tmp103_write_reg(client, TMP103_CONF_REG, TMP103_CONF_RUN);
}

static const struct dev_pm_ops tmp103_dev_pm_ops = {
	.suspend	= tmp103_suspend,
	.resume		= tmp103_resume,
};

#define tmp103_DEV_PM_OPS (&tmp103_dev_pm_ops)
#else
#define	tmp103_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id tmp103_id[] = {
	{ "tmp103", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp103_id);

static struct i2c_driver tmp103_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.pm	= tmp103_DEV_PM_OPS,
	.probe		= tmp103_probe,
	.remove		= tmp103_remove,
	.id_table	= tmp103_id,
};

module_i2c_driver(tmp103_driver);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Texas Instruments tmp103 temperature sensor driver");
MODULE_LICENSE("GPL");
