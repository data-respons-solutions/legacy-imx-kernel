/*
 * Maxim max31913 driver
 *
 * Copyright (c) 2016 DataRespons
 *  Artur Siewierski, <asi@datarespons.no>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/spi/max31913.h>

#include <linux/of.h>
#include <linux/of_device.h>

static inline int max31913_read(struct max31913 *inst)
{
	int err = 0;
	err = spi_read(inst->spi, &inst->rx_buff, 1);
	if (err < 0)
	{
	   pr_err ("| %d | %s | spi_read failed with status %d\n", __LINE__, __FUNCTION__, err);
	   return -err;
	}

	//pr_err ("| %d | %s | inst->rx_buff[0] = %d\n", __LINE__, __FUNCTION__, (int)inst->rx_buff[0]);

	return (int)inst->rx_buff[0];
}

static ssize_t max31913_din_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max31913 *inst = dev_get_drvdata(dev);
	char *bp = buf;
	int ret;

	mutex_lock(&inst->lock);
	ret = max31913_read(inst);
	dev_dbg(dev, "max31913_read() returned %d\n", ret);
	if (ret < 0) {
		mutex_unlock(&inst->lock);
		return ret;
	}

	bp += sprintf(bp, "%d", ret);

	mutex_unlock(&inst->lock);
	return bp - buf;
}

static DEVICE_ATTR(din_show, S_IRUGO, max31913_din_show, NULL);

static int max31913_probe(struct spi_device *spi)
{
	struct max31913 *inst = NULL;
	int ret;
	int err;

	inst = kzalloc(sizeof(struct max31913), GFP_KERNEL);
	if (inst == NULL) {
		dev_err(&spi->dev, "no memory for device\n");
		return -ENOMEM;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;

	err = spi_setup(spi);
	if (err < 0)
	{
		pr_err ("spi_setup failed with error %d\n", err);
		return err;
	}

	inst->spi = spi;
	spi_set_drvdata(spi, inst);

	pr_err ("MAX31913 initialized\n");

	mutex_init(&inst->lock);

	ret = device_create_file(&spi->dev, &dev_attr_din_show);
	if (ret) {
		dev_err(&spi->dev, "cannot create DIN attribute\n");
		goto err_din_show;
	}

	return 0;

 err_din_show:
	device_remove_file(&spi->dev, &dev_attr_din_show);
	kfree(inst);

	return ret;
}

static int max31913_remove(struct spi_device *spi)
{
	struct max31913 *inst = spi_get_drvdata(spi);

	device_remove_file(&spi->dev, &dev_attr_din_show);
	kfree(inst);
	return 0;
}



static const struct spi_device_id max31913_id[] = {
	{"max31913", 0},
	{ }
};

MODULE_DEVICE_TABLE(spi, max31913_id);

#ifdef CONFIG_OF
static const struct of_device_id max31913_of_match[] = {
	{ .compatible = "maxim,max31913" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max31913_of_match);
#endif

static struct spi_driver max31913_driver = {
	.driver = {
		.name	= "max31913",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(max31913_of_match),
	},
	.probe		= max31913_probe,
	.remove		= max31913_remove,
};

module_spi_driver(max31913_driver);

MODULE_AUTHOR("Artur Siewierski <asi@datarespons.no>");
MODULE_DESCRIPTION("MAX31913 SPI driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:max31913");
