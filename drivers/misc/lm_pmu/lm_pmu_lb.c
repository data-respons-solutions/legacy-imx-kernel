
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

const struct spi_device_id lm_pmu_lb_ids[] = {
	{ "lm_pmu_lb", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, lm_pmu_lb_ids);

static struct of_device_id lm_pmu_lb_dt_ids[] = {
	{ .compatible = "datarespons,lm-pmu-lb",  .data = 0, },
	{}
};
MODULE_DEVICE_TABLE(of, lm_pmu_lb_dt_ids);

#define VALID_MASK_DCIN 0x1
#define VALID_MASK_BAT1 0x2
#define VALID_MASK_BAT2 0x4

struct lm_pmu_lb {
	struct lm_pmu_private *priv;
	Ina219Msg_t ina_values;
	unsigned long last_ina_update;
	bool ina_valid;
	struct power_supply *ps_dcin;
	struct power_supply *ps_manikin[2];
	int psu_valids;						/* Bit field fpr valid[1:3] */
	bool manikin_12v_power_on;
	bool manikin_5v_power_on;
	int gpio_12v_manikin[2];
	bool gpio_12v_manikin_active_low[2];
	int gpio_5v_manikin;
	bool gpio_5v_manikin_active_low;
	int gpio_bat_det[2];
	bool gpio_bat_det_active_low[2];
	int gpio_bat_disable[2];
	bool gpio_bat_disable_active_low[2];
	bool bat_disabled[2];
	bool bat_ce[2];
	bool bat_detect[2];
	struct notifier_block ps_dcin_nb;
};

/* Sensors */
/* worst case is 68.10 ms (~14.6Hz, ina219) */
#define INA2XX_CONVERSION_RATE		5	/* Use max 5 Hz */

static int pmu_update_ina_values(struct lm_pmu_lb *pmu)
{
	int tx_len, status=0;
	u8 *payload;
	u8 tx_buffer[sizeof(Ina219MsgHeader_t)];
	u8 rx_buffer[sizeof(Ina219MsgHeader_t) + sizeof(Ina219Msg_t)];

	if (time_after(jiffies, pmu->last_ina_update + HZ/INA2XX_CONVERSION_RATE) || !pmu->ina_valid ) {
		tx_len = ina219_create_message(msg_ina219_show_value, tx_buffer, 0);
		status = lm_pmu_exchange(pmu->priv, msg_ina, tx_buffer, tx_len, rx_buffer, sizeof(rx_buffer));
		pmu->last_ina_update = jiffies;
		pmu->ina_valid = true;

		if (status < 0)
			goto cleanup;

		payload = ina219_get_payload(rx_buffer);
		if (payload) {
			memcpy(&pmu->ina_values, payload, sizeof(pmu->ina_values));
		}
		else
			dev_err(&pmu->priv->spi_dev->dev, "%s: Failed updating ina values\n", __func__);

	}

cleanup:
	return status;
}

static int lm_pmu_get_valids(struct lm_pmu_lb *pmu)
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


static enum power_supply_property manikin_5v_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int lm_pmu_mains_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lm_pmu_private *priv = dev_get_drvdata(psy->dev->parent);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	lm_pmu_get_valids(pmu);
	pmu_update_ina_values(pmu);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pmu->psu_valids & VALID_MASK_DCIN ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pmu->ina_values.bus_voltage_uV[0];
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = pmu->ina_values.current_uA[0];
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = pmu->ina_values.power_uW[0];
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
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	pmu_update_ina_values(pmu);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pmu->manikin_12v_power_on ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pmu->ina_values.bus_voltage_uV[1];
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = pmu->ina_values.current_uA[1];
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = pmu->ina_values.power_uW[1];
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
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pmu->manikin_5v_power_on ? 1 : 0;
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
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);

	int gpioval[2];
	int n;
	for (n=0; n < 2; n++)
		gpioval[n] = pmu->gpio_12v_manikin_active_low[n] ? (val->intval ? 0 : 1) : (val->intval ? 1 : 0);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		gpio_set_value(pmu->gpio_12v_manikin[0], gpioval[0]);
		gpio_set_value(pmu->gpio_12v_manikin[1], gpioval[1]);
		pmu->manikin_12v_power_on = val->intval ? true : false;
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
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	int gpioval = pmu->gpio_5v_manikin_active_low ? (val->intval ? 0 : 1) : (val->intval ? 1 : 0);
	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		gpio_set_value(pmu->gpio_5v_manikin, gpioval);
		pmu->manikin_5v_power_on = val->intval ? true : false;
		power_supply_changed(psy);
		break;

	default:
		return -EPERM;
	}

	return 0;
}

static int lm_pmu_notifier_call(struct notifier_block *nb,
		unsigned long val, void *v)
{
	struct lm_pmu_lb *pmu = container_of(nb, struct lm_pmu_lb, ps_dcin_nb);
	struct power_supply *psy = v;

	if (strncmp(psy->name, "ds2781-battery", 14) == 0) {
		pr_info( "Found %s to %d\n", psy->name, (int)val);

	}
	return 0;
}

static void lm_pmu_update_bat_detect(struct lm_pmu_lb *pmu)
{
	int n;
	for (n=0; n < 2; n++) {
		if (pmu->gpio_bat_det_active_low[n])
			pmu->bat_detect[n] = gpio_get_value(pmu->gpio_bat_det[n]) ? 0 : 1;
		else
			pmu->bat_detect[n] = gpio_get_value(pmu->gpio_bat_det[n]) ? 1 : 0;
	}
}

static ssize_t lm_pmu_show_bat_det(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	lm_pmu_update_bat_detect(pmu);

	if (strcmp(attr->attr.name, "bat_det1") == 0) {
		return sprintf(buf, "%d\n", pmu->bat_detect[0]);
	}
	if (strcmp(attr->attr.name, "bat_det2") == 0) {
		return sprintf(buf, "%d\n", pmu->bat_detect[1]);
	}
	if (strcmp(attr->attr.name, "bat_valid1") == 0) {
		lm_pmu_get_valids(pmu);
		return sprintf(buf, "%d\n", (pmu->psu_valids & VALID_MASK_BAT1) ? 1 : 0);
	}

	if (strcmp(attr->attr.name, "bat_valid2") == 0) {
		lm_pmu_get_valids(pmu);
		return sprintf(buf, "%d\n", (pmu->psu_valids & VALID_MASK_BAT2) ? 1 : 0);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_show_bat_disable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);

	if (strcmp(attr->attr.name, "bat_disable1") == 0) {
		return sprintf(buf, "%d\n", pmu->bat_disabled[0]);
	}
	if (strcmp(attr->attr.name, "bat_disable2") == 0) {
		return sprintf(buf, "%d\n", pmu->bat_disabled[1]);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_set_bat_disable(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);

	bool disable = buf[0] == '1' ? 1 : 0;
	int n = -1;
	int mask;
	lm_pmu_get_valids(pmu);
	if (strcmp(attr->attr.name, "bat_disable1") == 0) {
		mask = VALID_MASK_DCIN | VALID_MASK_BAT2;
		n = 0;
	}
	else if (strcmp(attr->attr.name, "bat_disable2") == 0) {
		mask = VALID_MASK_DCIN | VALID_MASK_BAT1;
		n = 1;
	}
	if ( n < 0 )
		return -EINVAL;

	if (disable) {
		if (pmu->psu_valids & mask) {
			gpio_set_value(pmu->gpio_bat_disable[n], pmu->gpio_bat_disable_active_low[n] ? 0 : 1);
			pmu->bat_disabled[n] = true;
			return count;
		}
		else {
			dev_warn(dev, "Can not turn off all power sources\n");
			return -EINVAL;
		}
	}
	else {
		pmu->bat_disabled[n] = false;
		gpio_set_value(pmu->gpio_bat_disable[n], pmu->gpio_bat_disable_active_low[n] ? 1 : 0);
		return count;
	}
	return -EINVAL;
}


static ssize_t lm_pmu_show_bat_ce(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);

	if (strcmp(attr->attr.name, "bat_ce1") == 0) {
		return sprintf(buf, "%d\n", pmu->bat_ce[0]);
	}
	if (strcmp(attr->attr.name, "bat_ce2") == 0) {
		return sprintf(buf, "%d\n", pmu->bat_ce[1]);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_set_bat_ce(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct lm_pmu_private *priv = dev_get_drvdata(dev->parent);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	int status;
	bool enable = buf[0] == '1' ? 1 : 0;
	int n = -1;
	u16 mask;
	lm_pmu_update_bat_detect(pmu);
	if (strcmp(attr->attr.name, "bat_ce1") == 0) {
		mask = 1;
		n = 0;
	}
	else if (strcmp(attr->attr.name, "bat_ce2") == 0) {
		mask = 2;
		n = 1;
	}
	if ( n < 0 )
		return -EINVAL;

	if (enable) {
		if (pmu->bat_detect[n]) {
			status = lm_pmu_set_charge(priv, msg_chargeEnable, mask);
			if (status < 0)
				return status;
			pmu->bat_ce[n] = true;
			return count;
		}
		else {
			dev_warn(dev, "Can not charge non-present battery %d\n", n+1);
			return -EINVAL;
		}
	}
	else {
		status = lm_pmu_set_charge(priv, msg_chargeDisable, mask);
		if (status < 0)
			return status;
		pmu->bat_ce[n] = false;
		return count;
	}
	return -EINVAL;
}


DEVICE_ATTR(bat_det1, S_IRUGO, lm_pmu_show_bat_det, NULL);
DEVICE_ATTR(bat_det2, S_IRUGO, lm_pmu_show_bat_det, NULL);
DEVICE_ATTR(bat_disable1, 0644, lm_pmu_show_bat_disable, lm_pmu_set_bat_disable);
DEVICE_ATTR(bat_disable2, 0644, lm_pmu_show_bat_disable, lm_pmu_set_bat_disable);
DEVICE_ATTR(bat_valid1, S_IRUGO, lm_pmu_show_bat_det, NULL);
DEVICE_ATTR(bat_valid2, S_IRUGO, lm_pmu_show_bat_det, NULL);
DEVICE_ATTR(bat_ce1, 0644, lm_pmu_show_bat_ce, lm_pmu_set_bat_ce);
DEVICE_ATTR(bat_ce2, 0644, lm_pmu_show_bat_ce, lm_pmu_set_bat_ce);

static struct attribute *bat_sysfs_attr[] = {
	&dev_attr_bat_det1.attr,
	&dev_attr_bat_det2.attr,
	&dev_attr_bat_disable1.attr,
	&dev_attr_bat_disable2.attr,
	&dev_attr_bat_valid1.attr,
	&dev_attr_bat_valid2.attr,
	&dev_attr_bat_ce1.attr,
	&dev_attr_bat_ce2.attr,
	NULL,
};

static const struct attribute_group bat_sysfs_attr_group = {
	.attrs = bat_sysfs_attr,
};

/**********************************************************************
 *  Parse DT
 */

static int lm_pmu_dt(struct lm_pmu_lb *pmu)
{
	struct device *dev = &pmu->priv->spi_dev->dev;
	struct device_node *np = dev->of_node;
	int n, num_gpio;
	enum of_gpio_flags flag;
	unsigned long rflags;

	pmu->gpio_12v_manikin[0] = pmu->gpio_12v_manikin[1] = -1;
	num_gpio = of_gpio_named_count(np, "manikin-12v-gpio");
	if (num_gpio > 2) {
		dev_err(dev, "%s: More than 2 gpios for manikin-12v-gpio [%d]\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n=0; n < num_gpio; n++) {
		pmu->gpio_12v_manikin[n] = of_get_named_gpio_flags(np, "manikin-12v-gpio", n, &flag);
		if (gpio_is_valid(pmu->gpio_12v_manikin[n])) {
			pmu->gpio_12v_manikin_active_low[n] = flag == OF_GPIO_ACTIVE_LOW;
			rflags = pmu->gpio_12v_manikin_active_low[n] ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
			if (devm_gpio_request_one(dev, pmu->gpio_12v_manikin[n], rflags, 0) ) {
				dev_err(dev, "%s: unable to request GPIO manikin-12v-gpio [%d]\n", __func__, pmu->gpio_12v_manikin[n]);
				return -EINVAL;
			}
		}
		else {
			dev_err(dev, "%s: GPIO manikin-12v-gpio [%d] invalid\n", __func__, pmu->gpio_12v_manikin[n]);
			return -EINVAL;
		}
	}

	pmu->gpio_5v_manikin = of_get_named_gpio_flags(np, "manikin-5v-gpio", 0, &flag);
	if (gpio_is_valid(pmu->gpio_5v_manikin)) {
		if (devm_gpio_request_one(dev, pmu->gpio_5v_manikin,
				flag == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
						"manikin-5v")) {
			dev_err(dev, "%s: unable to request GPIO manikin-5v-gpio [%d]\n", __func__, pmu->gpio_5v_manikin);
			return EINVAL;
		}
		pmu->gpio_5v_manikin_active_low = flag == OF_GPIO_ACTIVE_LOW;
	}

	pmu->gpio_bat_det[0] = pmu->gpio_bat_det[1] = -1;
	num_gpio = of_gpio_named_count(np, "bat-detect-gpios");
	if (num_gpio > 2) {
		dev_err(dev, "%s: More than gpios for bat-detect-gpios [%d]\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n=0; n < num_gpio; n++) {
		pmu->gpio_bat_det[n] = of_get_named_gpio_flags(np, "bat-detect-gpios", n, &flag);
		if (!gpio_is_valid(pmu->gpio_bat_det[n]) ||
				devm_gpio_request_one(dev, pmu->gpio_bat_det[n], GPIOF_DIR_IN, 0)) {
			dev_err(dev, "%s: Unable to request bat_detect pin %d\n", __func__, pmu->gpio_bat_det[n]);
			return EINVAL;
		}
		pmu->gpio_bat_det_active_low[n] = flag == OF_GPIO_ACTIVE_LOW;
	}

	pmu->gpio_bat_disable[0] = pmu->gpio_bat_disable[1] = -1;
	num_gpio = of_gpio_named_count(np, "bat-disable-gpios");
	if (num_gpio > 2) {
		dev_err(dev, "%s: More than gpios for bat-disable-gpios [%d]\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n=0; n < num_gpio; n++) {
		pmu->gpio_bat_disable[n] = of_get_named_gpio_flags(np, "bat-disable-gpios", n, &flag);
		if (!gpio_is_valid(pmu->gpio_bat_disable[n]) ||
				devm_gpio_request_one(dev, pmu->gpio_bat_disable[n], GPIOF_DIR_IN, 0)) {
			dev_err(dev, "%s: Unable to request bat_disable pin %d\n", __func__, pmu->gpio_bat_disable[n]);
			return EINVAL;
		}
		pmu->gpio_bat_disable_active_low[n] = flag == OF_GPIO_ACTIVE_LOW;
	}
	return 0;
}

static int lm_pmu_lb_probe(struct spi_device *spi)
{
	struct lm_pmu_lb *pmu;
	int ret=0;

	pmu = devm_kzalloc(&spi->dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;

	pmu->priv = lm_pmu_init(spi);
	if (!pmu->priv)
		return -EINVAL;

	lm_pmu_set_subclass_data(pmu->priv, pmu);
	ret = lm_pmu_dt(pmu);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Failed to obtain platform data\n", __func__);
		goto cleanup;
	}

	pmu->manikin_12v_power_on = false;
	pmu->manikin_5v_power_on = false;


	pmu->ps_dcin = devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
	if (!pmu->ps_dcin) {
		ret = -ENOMEM;
		goto cleanup;
	}
	pmu->ps_dcin->name = "DCIN";
	pmu->ps_dcin->type = POWER_SUPPLY_TYPE_MAINS;
	pmu->ps_dcin->get_property = lm_pmu_mains_get_property;
	pmu->ps_dcin->num_properties = ARRAY_SIZE(dcin_props_lb);
	pmu->ps_dcin->properties = dcin_props_lb;

	ret = power_supply_register(&spi->dev, pmu->ps_dcin);
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to register MAINS PS\n");
	}
	else {
		pmu->ps_dcin_nb.notifier_call = lm_pmu_notifier_call;
		power_supply_reg_notifier(&pmu->ps_dcin_nb);
		ret = sysfs_create_group(&pmu->ps_dcin->dev->kobj, &bat_sysfs_attr_group);
	}


	pmu->ps_manikin[0]= devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
	pmu->ps_manikin[1]= devm_kzalloc(&spi->dev, sizeof(struct power_supply), GFP_KERNEL);
	if (pmu->ps_manikin[0] == 0 || pmu->ps_manikin[1] == 0) {
		ret = -ENOMEM;
		goto cleanup;
	}
	pmu->ps_manikin[0]->name = "MANIKIN_12V";
	pmu->ps_manikin[0]->type = POWER_SUPPLY_TYPE_UNKNOWN;
	pmu->ps_manikin[0]->get_property = lm_pmu_manikin_12v_get_property;
	pmu->ps_manikin[0]->set_property = lm_pmu_manikin_12v_set_property;
	pmu->ps_manikin[0]->num_properties = ARRAY_SIZE(manikin_12v_props);
	pmu->ps_manikin[0]->properties = manikin_12v_props;
	pmu->ps_manikin[0]->property_is_writeable = lm_pmu_manikin_prop_is_writable;

	ret = power_supply_register(&spi->dev, pmu->ps_manikin[0]);
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to register MANIKIN 12V PS\n");
	}

	pmu->ps_manikin[1]->name = "MANIKIN_5V";
	pmu->ps_manikin[1]->type = POWER_SUPPLY_TYPE_UNKNOWN;
	pmu->ps_manikin[1]->get_property = lm_pmu_manikin_5v_get_property;
	pmu->ps_manikin[1]->set_property = lm_pmu_manikin_5v_set_property;
	pmu->ps_manikin[1]->num_properties = ARRAY_SIZE(manikin_5v_props);
	pmu->ps_manikin[1]->properties = manikin_5v_props;
	if (gpio_is_valid(pmu->gpio_5v_manikin))
		pmu->ps_manikin[1]->property_is_writeable = lm_pmu_manikin_prop_is_writable;

	ret = power_supply_register(&spi->dev, pmu->ps_manikin[1]);
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to register MANIKIN 5V PS\n");
	}
	return 0;

cleanup:
	lm_pmu_deinit(pmu->priv);
	return ret;
}

static int lm_pmu_lb_remove(struct spi_device *spi)
{
	struct lm_pmu_private *priv = dev_get_drvdata(&spi->dev);
	struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	sysfs_remove_group(&pmu->ps_dcin->dev->kobj, &bat_sysfs_attr_group);
	lm_pmu_deinit(priv);
	return 0;
}



static struct spi_driver lm_pmu_lb_driver = {
	.driver = {
		.name	= "lm_pmu_lb",
		.owner	= THIS_MODULE,
		.of_match_table = lm_pmu_lb_dt_ids,
	},
	.probe	= lm_pmu_lb_probe,
	.remove	= lm_pmu_lb_remove,
	.id_table = lm_pmu_lb_ids,
};

module_spi_driver(lm_pmu_lb_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus Linkbox uC driver");
