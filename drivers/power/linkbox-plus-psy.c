#define DEBUG

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
#include <linux/platform_device.h>
#include <linux/power_supply.h>



static struct of_device_id linkbox_plus_psy_dt_ids[] = {
	{ .compatible = "datarespons,linkbox-plus-psy",  .data = 0, },
	{}
};
MODULE_DEVICE_TABLE(of, linkbox_plus_psy_dt_ids);

#define VALID_MASK_DCIN 0x1
#define VALID_MASK_BAT1 0x2
#define VALID_MASK_BAT2 0x4


static char *bat_disable_names[2] = { "bat_disable1", "bat_disable2" };
static char *bat_detect_names[2] = { "bat_detect1", "bat_detect2" };
static char *valid_names[3] = { "valid1", "valid2", "valid3" };
static char *bat_ce_names[2] = { "bat_ce1", "bat_ce2" };
static char *manikin_12v_names[2] = { "12v boost", "12v aux" };

struct lbp_priv {
	struct platform_device *pdev;
	struct i2c_adapter *i2c_adapter;
	struct i2c_client *ina219_dcin;
	struct i2c_client *ina219_manikin;
	unsigned long last_ina_update;
	bool ina_valid;
	struct power_supply *ps_dcin;
	struct power_supply *ps_manikin[2];
	bool manikin_12v_power_on;
	bool manikin_5v_power_on;
	int gpio_12v_manikin[2];
	bool gpio_12v_manikin_active_low[2];
	int gpio_5v_manikin;
	bool gpio_5v_manikin_active_low;
	int gpio_bat_det[2];
	bool gpio_bat_det_active_low[2];
	int battery_detect_irqs[2];
	int gpio_bat_disable[2];
	bool gpio_bat_disable_active_low[2];
	bool bat_disable[2];
	int gpio_bat_ce[2];
	bool gpio_bat_ce_active_low[2];
	bool bat_ce[2];;
	int gpio_valid[3];
	bool gpio_valid_active_low[3];
	int gpio_5w_sd;
	bool gpio_5w_sd_active_low;
	int gpio_spkr_sd;
	bool gpio_spkr_sd_active_low;
	struct notifier_block ps_dcin_nb;
	struct work_struct alert_work;

};

/* worst case is 68.10 ms (~14.6Hz, ina219) */
#define INA2XX_CONVERSION_RATE		5	/* Use max 5 Hz */

static inline int is_set(int gpio, bool alow)
{
	int val = gpio_get_value(gpio);
	if (alow)
		return val ? 0 : 1;
	else
		return val ? 1 : 0;
}

static inline void set_val(int gpio, bool alow, int val)
{
	if (alow)
		gpio_set_value(gpio, val ? 0 : 1);
	else
		gpio_set_value(gpio, val ? 1 : 0);
}

static int pmu_update_ina_values(struct lbp_priv *pmu)
{
	return 0;
}

static int get_valids(struct lbp_priv *priv)
{
	int status=0;
	int n=0;
	for (n=0; n < 3; n++) {
		status |= is_set(priv->gpio_valid[n], priv->gpio_valid_active_low[n]) << n;
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
	int status;
	struct lbp_priv *priv = dev_get_drvdata(psy->dev->parent);
	int valids = get_valids(priv);
	status = pmu_update_ina_values(priv);
	if (status < 0)
		return status;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = valids & VALID_MASK_DCIN ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 0; /* TODO: Assign */
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval =0;
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
	int status;
	struct lbp_priv *priv = dev_get_drvdata(psy->dev->parent);
	status = pmu_update_ina_values(priv);
	if (status < 0)
		return status;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = priv->manikin_12v_power_on ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 0; /* TODO: Assign */
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = 0;
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
	struct lbp_priv *priv = dev_get_drvdata(psy->dev->parent);

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
	struct lbp_priv *priv = dev_get_drvdata(psy->dev->parent);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval) {
			gpio_set_value(priv->gpio_12v_manikin[0], priv->gpio_12v_manikin_active_low[0] ? 0 : 1);
			msleep(2);
			gpio_set_value(priv->gpio_12v_manikin[1], priv->gpio_12v_manikin_active_low[1] ? 0 : 1);
			msleep(10);
			gpio_set_value(priv->gpio_5w_sd, priv->gpio_5w_sd_active_low ? 1 : 0);
			msleep(5);
			gpio_set_value(priv->gpio_spkr_sd, priv->gpio_spkr_sd_active_low ? 1 : 0);
		}
		else  {
			gpio_set_value(priv->gpio_12v_manikin[1], priv->gpio_12v_manikin_active_low[1] ? 1 : 0);
			gpio_set_value(priv->gpio_12v_manikin[0], priv->gpio_12v_manikin_active_low[0] ? 1 : 0);
			gpio_set_value(priv->gpio_5w_sd, priv->gpio_5w_sd_active_low ? 0 : 1);
			gpio_set_value(priv->gpio_spkr_sd, priv->gpio_spkr_sd_active_low ? 0 : 1);
		}
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
	struct lbp_priv *priv = dev_get_drvdata(psy->dev->parent);
	int gpioval = priv->gpio_5v_manikin_active_low ? (val->intval ? 0 : 1) : (val->intval ? 1 : 0);
	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		gpio_set_value(priv->gpio_5v_manikin, gpioval);
		priv->manikin_5v_power_on = val->intval ? true : false;
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
	struct lbp_priv *pmu = container_of(nb, struct lbp_priv, ps_dcin_nb);
	struct power_supply *psy = v;

	if (strncmp(psy->name, "ds2781-battery", 14) == 0) {
		pr_info( "Found %s to %d\n", psy->name, (int)val);

	}
	return 0;
}

static ssize_t lm_pmu_show_bat_det(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lbp_priv *priv = dev_get_drvdata(dev->parent);
	int valids = get_valids(priv);

	if (strcmp(attr->attr.name, "bat_det1") == 0) {
		return sprintf(buf, "%d\n", is_set(priv->gpio_bat_det[0], priv->gpio_bat_det_active_low[0]));
	}
	if (strcmp(attr->attr.name, "bat_det2") == 0) {
		return sprintf(buf, "%d\n", is_set(priv->gpio_bat_det[1], priv->gpio_bat_det_active_low[1]));
	}
	if (strcmp(attr->attr.name, "bat_valid1") == 0) {
		return sprintf(buf, "%d\n", (valids & VALID_MASK_BAT1) ? 1 : 0);
	}

	if (strcmp(attr->attr.name, "bat_valid2") == 0) {
		return sprintf(buf, "%d\n", (valids & VALID_MASK_BAT2) ? 1 : 0);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_show_bat_disable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lbp_priv *priv = dev_get_drvdata(dev->parent);

	if (strcmp(attr->attr.name, "bat_disable1") == 0) {
		return sprintf(buf, "%d\n", priv->bat_disable[0]);
	}
	if (strcmp(attr->attr.name, "bat_disable2") == 0) {
		return sprintf(buf, "%d\n", priv->bat_disable[1]);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_set_bat_disable(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct lbp_priv *priv = dev_get_drvdata(dev->parent);
	int valids = get_valids(priv);
	bool disable = buf[0] == '1' ? 1 : 0;
	int n = -1;
	int mask;
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
		if (valids & mask) {
			set_val(priv->gpio_bat_disable[n], priv->gpio_bat_disable_active_low[n], 1);
			priv->bat_disable[n] = true;
			return count;
		}
		else {
			dev_warn(dev, "Can not turn off all power sources\n");
			return -EINVAL;
		}
	}
	else {
		priv->bat_disable[n] = false;
		set_val(priv->gpio_bat_disable[n], priv->gpio_bat_disable_active_low[n], 0);
		return count;
	}
	return -EINVAL;
}


static ssize_t lm_pmu_show_bat_ce(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct lbp_priv *priv = dev_get_drvdata(dev->parent);

	if (strcmp(attr->attr.name, "bat_ce1") == 0) {
		return sprintf(buf, "%d\n", priv->bat_ce[0]);
	}
	if (strcmp(attr->attr.name, "bat_ce2") == 0) {
		return sprintf(buf, "%d\n", priv->bat_ce[1]);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_set_bat_ce(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct lbp_priv *priv = dev_get_drvdata(dev->parent);
	bool enable = buf[0] == '1' ? 1 : 0;
	if (strcmp(attr->attr.name, "bat_ce1") == 0) {
		set_val(priv->gpio_bat_ce[0], priv->gpio_bat_ce_active_low[0], enable);
		return count;
	}
	else if (strcmp(attr->attr.name, "bat_ce2") == 0) {
		set_val(priv->gpio_bat_ce[1], priv->gpio_bat_ce_active_low[1], enable);
		return count;
	}

	return -EINVAL;
}


static DEVICE_ATTR(bat_det1, S_IRUGO, lm_pmu_show_bat_det, NULL);
static DEVICE_ATTR(bat_det2, S_IRUGO, lm_pmu_show_bat_det, NULL);
static DEVICE_ATTR(bat_disable1, 0644, lm_pmu_show_bat_disable, lm_pmu_set_bat_disable);
static DEVICE_ATTR(bat_disable2, 0644, lm_pmu_show_bat_disable, lm_pmu_set_bat_disable);
static DEVICE_ATTR(bat_valid1, S_IRUGO, lm_pmu_show_bat_det, NULL);
static DEVICE_ATTR(bat_valid2, S_IRUGO, lm_pmu_show_bat_det, NULL);
static DEVICE_ATTR(bat_ce1, 0644, lm_pmu_show_bat_ce, lm_pmu_set_bat_ce);
static DEVICE_ATTR(bat_ce2, 0644, lm_pmu_show_bat_ce, lm_pmu_set_bat_ce);

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

static int lm_pmu_dt(struct lbp_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	struct device_node *np = dev->of_node;
	int n, num_gpio;
	enum of_gpio_flags flag;
	unsigned long rflags;
	struct device_node *i2c_np;

	priv->gpio_12v_manikin[0] = priv->gpio_12v_manikin[1] = -1;
	num_gpio = of_gpio_named_count(np, "manikin-12v-gpio");
	if (num_gpio > 2) {
		dev_err(dev, "%s: More than 2 gpios for manikin-12v-gpio [%d]\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n=0; n < num_gpio; n++) {
		priv->gpio_12v_manikin[n] = of_get_named_gpio_flags(np, "manikin-12v-gpio", n, &flag);
		if (gpio_is_valid(priv->gpio_12v_manikin[n])) {
			priv->gpio_12v_manikin_active_low[n] = flag == OF_GPIO_ACTIVE_LOW;
			rflags = priv->gpio_12v_manikin_active_low[n] ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
			if (devm_gpio_request_one(dev, priv->gpio_12v_manikin[n], rflags, manikin_12v_names[n]) ) {
				dev_err(dev, "%s: unable to request GPIO manikin-12v-gpio [%d]\n", __func__, priv->gpio_12v_manikin[n]);
				return -EINVAL;
			}
		}
		else {
			dev_err(dev, "%s: GPIO manikin-12v-gpio [%d] invalid\n", __func__, priv->gpio_12v_manikin[n]);
			return -EINVAL;
		}
	}

	priv->gpio_5v_manikin = of_get_named_gpio_flags(np, "manikin-5v-gpio", 0, &flag);
	if (gpio_is_valid(priv->gpio_5v_manikin)) {
		if (devm_gpio_request_one(dev, priv->gpio_5v_manikin,
				flag == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
						"manikin-5v")) {
			dev_err(dev, "%s: unable to request GPIO manikin-5v-gpio [%d]\n", __func__, priv->gpio_5v_manikin);
			return -EINVAL;
		}
		priv->gpio_5v_manikin_active_low = flag == OF_GPIO_ACTIVE_LOW;
	}
	else {
		dev_err(dev, "%s: Invalid GPIO manikin-5v-gpio [%d]\n", __func__, priv->gpio_5v_manikin);
		return -EINVAL;
	}

	priv->gpio_bat_det[0] = priv->gpio_bat_det[1] = -1;
	num_gpio = of_gpio_named_count(np, "bat-detect-gpios");
	if (num_gpio > 2) {
		dev_err(dev, "%s: More than gpios for bat-detect-gpios [%d]\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n=0; n < num_gpio; n++) {
		priv->gpio_bat_det[n] = of_get_named_gpio_flags(np, "bat-detect-gpios", n, &flag);
		if (!gpio_is_valid(priv->gpio_bat_det[n]) ||
				devm_gpio_request_one(dev, priv->gpio_bat_det[n], GPIOF_DIR_IN, bat_detect_names[n])) {
			dev_err(dev, "%s: Unable to request bat_detect pin %d\n", __func__, priv->gpio_bat_det[n]);
			return -EINVAL;
		}
		priv->gpio_bat_det_active_low[n] = flag == OF_GPIO_ACTIVE_LOW;
	}

	priv->gpio_bat_disable[0] = priv->gpio_bat_disable[1] = -1;
	num_gpio = of_gpio_named_count(np, "bat-disable-gpios");
	if (num_gpio > 2) {
		dev_err(dev, "%s: More than gpios for bat-disable-gpios [%d]\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n=0; n < num_gpio; n++) {
		priv->gpio_bat_disable[n] = of_get_named_gpio_flags(np, "bat-disable-gpios", n, &flag);
		if (!gpio_is_valid(priv->gpio_bat_disable[n]) ||
				devm_gpio_request_one(dev, priv->gpio_bat_disable[n], GPIOF_DIR_IN, bat_disable_names[n])) {
			dev_err(dev, "%s: Unable to request bat_disable pin %d\n", __func__, priv->gpio_bat_disable[n]);
			return -EINVAL;
		}
		priv->gpio_bat_disable_active_low[n] = flag == OF_GPIO_ACTIVE_LOW;
	}

	priv->gpio_5w_sd = of_get_named_gpio_flags(np, "spksd-gpio", 0, &flag);
	if (gpio_is_valid(priv->gpio_5w_sd)) {
		priv->gpio_5w_sd_active_low = flag == OF_GPIO_ACTIVE_LOW ? 1 : 0;
		if (devm_gpio_request_one(dev, priv->gpio_5w_sd,
				priv->gpio_5w_sd_active_low ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
						"spksd-gpio")) {
			dev_err(dev, "Unable to request spksd-gpio %d\n", priv->gpio_5w_sd);
			return -EINVAL;
		}
	}

	priv->gpio_spkr_sd = of_get_named_gpio_flags(np, "amp-shutdown-gpio", 0, &flag);
	if (gpio_is_valid(priv->gpio_spkr_sd)) {
		priv->gpio_spkr_sd_active_low = flag == OF_GPIO_ACTIVE_LOW ? 1 : 0;
		if (devm_gpio_request_one(dev, priv->gpio_spkr_sd,
				priv->gpio_spkr_sd_active_low ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
						"amp-shutdown-gpio")) {
			dev_err(dev, "Unable to request amp-shutdown-gpio %d\n", priv->gpio_spkr_sd);
			return -EINVAL;
		}
	}

	i2c_np = of_parse_phandle(np, "i2c-adapter", 0);
	priv->i2c_adapter = of_find_i2c_adapter_by_node(i2c_np);
	return 0;
}


static int linkbox_plus_psy_probe(struct platform_device *pdev)
{
	struct lbp_priv *priv;
	int ret=0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;


	ret = lm_pmu_dt(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: Failed to obtain platform data\n", __func__);
		goto cleanup;
	}

	priv->manikin_12v_power_on = false;
	priv->manikin_5v_power_on = false;

	priv->ps_dcin = devm_kzalloc(&pdev->dev, sizeof(struct power_supply), GFP_KERNEL);
	if (!priv->ps_dcin) {
		ret = -ENOMEM;
		goto cleanup;
	}

	priv->pdev = pdev;
	priv->ps_dcin->name = "DCIN";
	priv->ps_dcin->type = POWER_SUPPLY_TYPE_MAINS;
	priv->ps_dcin->get_property = lm_pmu_mains_get_property;
	priv->ps_dcin->num_properties = ARRAY_SIZE(dcin_props_lb);
	priv->ps_dcin->properties = dcin_props_lb;

	platform_set_drvdata(pdev, priv);


	ret = power_supply_register(&pdev->dev, priv->ps_dcin);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register MAINS PS\n");
	}
	else {
		priv->ps_dcin_nb.notifier_call = lm_pmu_notifier_call;
		power_supply_reg_notifier(&priv->ps_dcin_nb);
		ret = sysfs_create_group(&priv->ps_dcin->dev->kobj, &bat_sysfs_attr_group);
	}


	priv->ps_manikin[0]= devm_kzalloc(&pdev->dev, sizeof(struct power_supply), GFP_KERNEL);
	priv->ps_manikin[1]= devm_kzalloc(&pdev->dev, sizeof(struct power_supply), GFP_KERNEL);
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
	priv->ps_manikin[0]->property_is_writeable = lm_pmu_manikin_prop_is_writable;

	ret = power_supply_register(&pdev->dev, priv->ps_manikin[0]);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register MANIKIN 12V PS\n");
	}

	priv->ps_manikin[1]->name = "MANIKIN_5V";
	priv->ps_manikin[1]->type = POWER_SUPPLY_TYPE_UNKNOWN;
	priv->ps_manikin[1]->get_property = lm_pmu_manikin_5v_get_property;
	priv->ps_manikin[1]->set_property = lm_pmu_manikin_5v_set_property;
	priv->ps_manikin[1]->num_properties = ARRAY_SIZE(manikin_5v_props);
	priv->ps_manikin[1]->properties = manikin_5v_props;
	if (gpio_is_valid(priv->gpio_5v_manikin))
		priv->ps_manikin[1]->property_is_writeable = lm_pmu_manikin_prop_is_writable;

	ret = power_supply_register(&pdev->dev, priv->ps_manikin[1]);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register MANIKIN 5V PS\n");
	}

	return 0;

cleanup:
	return ret;
}

static int lm_pmu_lb_remove(struct platform_device *pdev)
{
	struct lbp_priv *priv = platform_get_drvdata(pdev);
	sysfs_remove_group(&priv->ps_dcin->dev->kobj, &bat_sysfs_attr_group);
	return 0;
}

static void lm_pmu_lb_shutdown(struct platform_device *pdev)
{
	struct lbp_priv *priv = platform_get_drvdata(pdev);
	gpio_set_value(priv->gpio_bat_disable[0], priv->gpio_bat_disable_active_low[0] ? 1 : 0);
	gpio_set_value(priv->gpio_bat_disable[1], priv->gpio_bat_disable_active_low[1] ? 1 : 0);

}

static struct platform_driver linkbox_plus_psy_driver = {
	.driver = {
		.name = "linkbox-plus-psy",
		.owner = THIS_MODULE,
		.of_match_table = linkbox_plus_psy_dt_ids,
	},
	.probe = linkbox_plus_psy_probe,
	.remove = lm_pmu_lb_remove,
	.shutdown = lm_pmu_lb_shutdown,
};

module_platform_driver(linkbox_plus_psy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus Linkbox Power Supply");
