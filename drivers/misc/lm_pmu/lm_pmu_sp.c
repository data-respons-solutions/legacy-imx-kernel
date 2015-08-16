
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
#include "lm_pmu.h"

const struct spi_device_id lm_pmu_sp_ids[] = {
	{ "lm_pmu_sp", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, lm_pmu_sp_ids);

static struct of_device_id lm_pmu_sp_dt_ids[] = {
	{ .compatible = "datarespons,lm-pmu-sp",  .data = 0, },
	{}
};
MODULE_DEVICE_TABLE(of, lm_pmu_sp_dt_ids);

struct lm_pmu_sp {
	struct lm_pmu_private *priv;
	struct power_supply *ps_dcin;
	int psu_valids;						/* Bit field fpr valid[1:3] */
	int charge_enable_gpio;
	bool charge_enable_gpio_active_low;
	int charge_iset_gpio;
	bool charge_iset_gpio_active_low;
	bool charge_high_current;
	bool charge_enable;
	struct work_struct alert_work;
};

static enum power_supply_property dcin_props_sp[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int lm_pmu_get_valids(struct lm_pmu_sp *pmu)
{
	int status=0;
	u8 rx_buffer[4];
	status = lm_pmu_exchange(pmu->priv, msg_valid, 0, 0, rx_buffer, sizeof(rx_buffer));
	if (status < 0)
		dev_err(&pmu->priv->spi_dev->dev, "%s: failed\n", __func__);
	else {
		pmu->psu_valids = le32_to_cpu(*(u32*)rx_buffer);
		dev_dbg(&pmu->priv->spi_dev->dev, "%s: Valids are 0x%x found\n", __func__, pmu->psu_valids);
	}
	return status;
}

static int lm_pmu_mains_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);
	struct lm_pmu_sp *pmu = lm_pmu_get_subclass_data(priv);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pmu->psu_valids & 1 ? 1 : 0;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t lm_pmu_show_bat(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_sp *pmu = lm_pmu_get_subclass_data(priv);

	if (strcmp(attr->attr.name, "bat_charge_en") == 0) {
		return sprintf(buf, "%d\n", pmu->charge_enable);
	}
	if (strcmp(attr->attr.name, "bat_high_current") == 0) {
		return sprintf(buf, "%d\n", pmu->charge_high_current);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_set_bat(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_sp *pmu = lm_pmu_get_subclass_data(priv);
	bool enable = buf[0] == '1' ? 1 : 0;

	if (strcmp(attr->attr.name, "bat_charge_en") == 0) {
		pmu->charge_enable = enable;
		if (enable)
			gpio_set_value(pmu->charge_enable_gpio, pmu->charge_enable_gpio_active_low ? 0 : 1);

		else
			gpio_set_value(pmu->charge_enable_gpio, pmu->charge_enable_gpio_active_low ? 1 : 0);
		return count;
	}
	else if (strcmp(attr->attr.name, "bat_high_current") == 0) {
		pmu->charge_high_current = enable;
		if (enable)
			gpio_set_value(pmu->charge_iset_gpio, pmu->charge_iset_gpio_active_low ? 0 : 1);
		else
			gpio_set_value(pmu->charge_iset_gpio, pmu->charge_iset_gpio_active_low ? 1 : 0);
		return count;
	}

	return -EINVAL;
}


DEVICE_ATTR(bat_charge_en, 0644, lm_pmu_show_bat, lm_pmu_set_bat);
DEVICE_ATTR(bat_high_current, 0644, lm_pmu_show_bat, lm_pmu_set_bat);

static struct attribute *bat_sysfs_attr[] = {
	&dev_attr_bat_charge_en.attr,
	&dev_attr_bat_high_current.attr,
	NULL,
};

static const struct attribute_group bat_sysfs_attr_group = {
	.attrs = bat_sysfs_attr,
};

static irqreturn_t alert_irq(void *_ptr)
{
	struct lm_pmu_sp *pmu = _ptr;
	dev_dbg(&pmu->priv->spi_dev->dev, "%s\n", __func__);
	schedule_work(&pmu->alert_work);
	return IRQ_HANDLED;
}

static void alert_handler(struct work_struct *ws)
{
	struct lm_pmu_sp *pmu = container_of(ws, struct lm_pmu_sp, alert_work);
	dev_dbg(&pmu->priv->spi_dev->dev, "%s\n", __func__);
	if (lm_pmu_get_valids(pmu) == 0) {
		power_supply_changed(pmu->ps_dcin);
	}

}
/**********************************************************************
 *  Parse DT
 */

static int lm_pmu_dt(struct lm_pmu_sp *pmu)
{
	struct device *dev = &pmu->priv->spi_dev->dev;
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flag;
	pmu->charge_enable_gpio = of_get_named_gpio_flags(np, "charge-enable-gpio", 0, &flag);
	if (gpio_is_valid(pmu->charge_enable_gpio)) {
		if (devm_gpio_request_one(dev, pmu->charge_enable_gpio,
				flag == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
						"charge-en")) {
			dev_err(dev, "%s: unable to request GPIO charge-enable-gpio [%d]\n", __func__, pmu->charge_enable_gpio);
			return EINVAL;
		}
		pmu->charge_enable_gpio_active_low = flag == OF_GPIO_ACTIVE_LOW;
	}
	else {
		dev_err(dev, "%s: invalid GPIO charge-enable-gpio [%d]\n", __func__, pmu->charge_enable_gpio);
		return -EINVAL;
	}

	pmu->charge_iset_gpio = of_get_named_gpio_flags(np, "charge-iset-gpio", 0, &flag);
	if (gpio_is_valid(pmu->charge_iset_gpio)) {
		if (devm_gpio_request_one(dev, pmu->charge_iset_gpio,
				flag == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
						"charge-en")) {
			dev_err(dev, "%s: unable to request GPIO charge-enable-gpio [%d]\n", __func__, pmu->charge_iset_gpio);
			return EINVAL;
		}
		pmu->charge_iset_gpio_active_low = flag == OF_GPIO_ACTIVE_LOW;
	}
	else {
		dev_err(dev, "%s: invalid GPIO charge-enable-gpio [%d]\n", __func__, pmu->charge_iset_gpio);
		return -EINVAL;
	}
	return 0;
}

static int lm_pmu_sp_probe(struct spi_device *spi)
{
	struct lm_pmu_sp *pmu;
	int ret=0;

	pmu = devm_kzalloc(&spi->dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;

	pmu->priv = lm_pmu_init(spi);
	if (!pmu->priv) {
		dev_err(&spi->dev, "%s: failed at lm_pmu_init\n", __func__);
		return -EINVAL;
	}
	lm_pmu_set_subclass_data(pmu->priv, pmu);
	ret = lm_pmu_dt(pmu);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Failed to obtain platform data\n", __func__);
		return -EINVAL;
	}

	if (!pmu->priv->pmu_ready)
		return 0;

	lm_pmu_get_valids(pmu);
	pm_power_off = lm_pmu_poweroff;

	pmu->ps_dcin = devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
	if (!pmu->ps_dcin) {
		ret = -ENOMEM;
		goto cleanup;
	}
	pmu->ps_dcin->name = "DCIN";
	pmu->ps_dcin->type = POWER_SUPPLY_TYPE_MAINS;
	pmu->ps_dcin->get_property = lm_pmu_mains_get_property;
	pmu->ps_dcin->num_properties = ARRAY_SIZE(dcin_props_sp);
	pmu->ps_dcin->properties = dcin_props_sp;

	ret = power_supply_register(&spi->dev, pmu->ps_dcin);
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to register MAINS PS\n");
		goto cleanup;
	}
	else {
		ret = sysfs_create_group(&pmu->ps_dcin->dev->kobj, &bat_sysfs_attr_group);
	}

	INIT_WORK(&pmu->alert_work, alert_handler);
	pmu->priv->alert_cb = alert_irq;
	return 0;

cleanup:
	lm_pmu_deinit(pmu->priv);
	return ret;
}

static int lm_pmu_sp_remove(struct spi_device *spi)
{
	struct lm_pmu_private *priv = dev_get_drvdata(&spi->dev);
	struct lm_pmu_sp *pmu = lm_pmu_get_subclass_data(priv);

	sysfs_remove_group(&pmu->ps_dcin->dev->kobj, &bat_sysfs_attr_group);
	lm_pmu_deinit(priv);
	return 0;
}

static void lm_pmu_sp_shutdown(struct spi_device *spi)
{
	struct lm_pmu_private *priv = dev_get_drvdata(&spi->dev);
	struct lm_pmu_sp *pmu = lm_pmu_get_subclass_data(priv);
	gpio_set_value(pmu->charge_enable_gpio, pmu->charge_enable_gpio_active_low ? 1 : 0);
	gpio_set_value(pmu->charge_iset_gpio, pmu->charge_iset_gpio_active_low ? 1 : 0);
}

static struct spi_driver lm_pmu_sp_driver = {
	.driver = {
		.name	= "lm_pmu_sp",
		.owner	= THIS_MODULE,
		.of_match_table = lm_pmu_sp_dt_ids,
	},
	.probe	= lm_pmu_sp_probe,
	.remove	= lm_pmu_sp_remove,
	.shutdown = lm_pmu_sp_shutdown,
	.id_table = lm_pmu_sp_ids,
};

module_spi_driver(lm_pmu_sp_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus Simpad uC driver");
