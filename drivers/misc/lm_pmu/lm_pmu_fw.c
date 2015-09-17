/*
 * lm_pmu_fw.c
 *
 *  Created on: Sep 3, 2015
 *      Author: hcl
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include "stm32fwu.h"
#include "stm32_core.h"

int lm_pmu_reset_message(void);

struct lm_pmu_fw {
	int gpio_boot0;
	int gpio_reset;
	struct spi_device *spi_dev;
	struct stm32fwu_fw *fw;
	struct miscdevice fw_dev;
	bool fw_dev_ok;
};

const struct spi_device_id lm_pmu_fw_ids[] = {
	{ "lm_pmu_fw", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, lm_pmu_fw_ids);

static struct of_device_id lm_pmu_fw_dt_ids[] = {
	{ .compatible = "datarespons,lm-pmu-fw",  .data = 0, },
	{}
};
MODULE_DEVICE_TABLE(of, lm_pmu_fw_dt_ids);

static int lm_pmu_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int lm_pmu_reset(struct lm_pmu_fw *priv)
{
	int status = 0;
	gpio_set_value(priv->gpio_reset, 0);
	msleep_interruptible(300);
	gpio_set_value(priv->gpio_reset, 1);
	msleep_interruptible(10);
	return status;
}

static ssize_t lm_pmu_update(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	int retries=5;
	int res=0;
	int fw_version;
	struct miscdevice *msdev = file->private_data;
	struct lm_pmu_fw *priv = dev_get_drvdata(msdev->parent);
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
	if (lm_pmu_reset_message() < 0) {
		dev_warn(msdev->parent, "%s: Failed to reset PMU over I2C interface\n", __func__);
		res = lm_pmu_reset(priv);
	}
	if (res) {
		dev_err(msdev->parent, "%s: failed to reset PMU\n", __func__);
		goto restore;
	}

	while (retries) {
		msleep(200);
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
	gpio_set_value(priv->gpio_boot0, 0);
	gpio_direction_input(priv->gpio_boot0);

	stm32fwu_destroy(priv->fw);
	kfree(buffer);
	return len;
}

static struct file_operations lm_pmu_fops = {
	.open = lm_pmu_open,
	.write = lm_pmu_update,
};

static int lm_pmu_fw_parse_of(struct lm_pmu_fw *pmu)
{
	struct device *dev = &pmu->spi_dev->dev;
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;

	pmu->gpio_boot0 = of_get_named_gpio(np, "boot0-gpio", 0);
	if (gpio_is_valid(pmu->gpio_boot0)) {
		if (devm_gpio_request_one(dev, pmu->gpio_boot0, GPIOF_IN, "pmu-boot0")) {
			dev_warn(dev, "%s: unable to request GPIO pmu-boot0 [%d]\n", __func__, pmu->gpio_boot0);
			pmu->gpio_boot0 = -1;
		}
	}
	else {
		dev_warn(dev, "%s: no GPIO for BOOT0 pin to PMU\n", __func__);
	}

	pmu->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(pmu->gpio_reset)) {
		if (devm_gpio_request_one(dev, pmu->gpio_reset, GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN, "pmu-reset")) {
			dev_warn(dev, "%s: unable to request GPIO reset-gpio [%d]\n", __func__, pmu->gpio_reset);
			pmu->gpio_reset = -1;
		}
	}
	else {
		dev_warn(dev, "%s: no GPIO for RESET pin to PMU\n", __func__);
	}
	return 0;
}

static int lm_pmu_fw_probe(struct spi_device *spi)
{
	struct lm_pmu_fw *pmu;
	int ret=0;

	pmu = devm_kzalloc(&spi->dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;

	pmu->spi_dev = spi;
	spi_set_drvdata(spi, pmu);
	ret = lm_pmu_fw_parse_of(pmu);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Failed to obtain platform data\n", __func__);
		goto cleanup;
	}



	pmu->fw_dev.minor = 250;
	pmu->fw_dev.name = "lm_pmu_fwupdate";
	pmu->fw_dev.fops = &lm_pmu_fops;
	pmu->fw_dev.parent = &spi->dev;
	ret = misc_register(&pmu->fw_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register firmware device\n");
		pmu->fw_dev_ok = false;
		goto cleanup;
	}

	pmu->fw_dev_ok = true;
	return 0;

cleanup:
	return ret;
}

static int lm_pmu_fw_remove(struct spi_device *spi)
{
	struct lm_pmu_fw *priv = dev_get_drvdata(&spi->dev);
	if (priv->fw_dev_ok)
		misc_deregister(&priv->fw_dev);
	return 0;
}

static void lm_pmu_fw_shutdown(struct spi_device *spi)
{
	struct lm_pmu_fw *pmu = dev_get_drvdata(&spi->dev);
	gpio_set_value(pmu->gpio_reset, 1);
	gpio_direction_input(pmu->gpio_boot0);

}


static struct spi_driver lm_pmu_fw_driver = {
	.driver = {
		.name	= "lm_pmu_fw",
		.owner	= THIS_MODULE,
		.of_match_table = lm_pmu_fw_dt_ids,
	},
	.probe	= lm_pmu_fw_probe,
	.remove	= lm_pmu_fw_remove,
	.shutdown = lm_pmu_fw_shutdown,
	.id_table = lm_pmu_fw_ids,
};

module_spi_driver(lm_pmu_fw_driver);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("STM32 upgrade for Laerdal PMU");
MODULE_LICENSE("GPL");

