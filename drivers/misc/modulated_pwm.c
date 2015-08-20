/*
 * Simple PWM buzzer
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>

struct modulated_pwm_data {
	struct pwm_device *pwm;
	struct device *dev;
	unsigned int pwmId;
	unsigned int period;
	u8 onValue;
};

static DEFINE_MUTEX(sysfs_lock);

static ssize_t modulated_pwm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct modulated_pwm_data *pb = dev_get_drvdata(dev);
	ssize_t status;
	mutex_lock(&sysfs_lock);
	status = sprintf(buf, "%d\n", pb->onValue);
	mutex_unlock(&sysfs_lock);
	return status;
}

static ssize_t modulated_pwm_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t sz)
{
	struct modulated_pwm_data *pb = dev_get_drvdata(dev);
	size_t status = sz;
	int ret, duty_period;
	u8 val;

	mutex_lock(&sysfs_lock);
	ret = kstrtou8(buf, 10, &val);
	if (ret < 0) {
		status = ret;
		goto exit;
	}

	duty_period = ((long) pb->period * val) / 255;

	if ((ret = pwm_config(pb->pwm, duty_period, pb->period)) < 0) {
		dev_err(dev, "%s: Could not configure PWM", __func__);
		status = ret;
		goto exit;
	} else {
		pb->onValue = val;
	}

	exit: mutex_unlock(&sysfs_lock);
	return status;
}

static DEVICE_ATTR(period, 0644, modulated_pwm_show, modulated_pwm_store);

static int modulated_pwm_probe(struct platform_device *pdev)
{
	struct modulated_pwm_data *pb;
	int ret;
	struct device_node *np = pdev->dev.of_node;
	bool inverted = of_property_read_bool(np, "inverted");

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	if (!np) {
		dev_err(&pdev->dev, "No DT node found\n");
		return -EINVAL;
	}

	pb->dev = &pdev->dev;
	pb->onValue = 0;

	pb->pwm = devm_of_pwm_get(&pdev->dev, np, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM\n");
		return PTR_ERR(pb->pwm);
	}
	pb->period = pwm_get_period(pb->pwm);
	dev_dbg(&pdev->dev, "%s: got pwm %d, period %d ns\n", __func__, pb->pwmId,
			pb->period);
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_period.attr);
	if (inverted)
		pwm_set_polarity(pb->pwm, PWM_POLARITY_INVERSED);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_enable(pb->pwm);
	platform_set_drvdata(pdev, pb);
	return 0;
}

static int modulated_pwm_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct modulated_pwm_data *pb = dev_get_drvdata(&bl->dev);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_period.attr);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	return 0;
}

static struct of_device_id modulated_pwm_of_match[] = {
	{
		.compatible = "modulated-pwm",
	},
	{}
};

MODULE_DEVICE_TABLE(of, modulated_pwm_of_match);

static struct platform_driver modulated_pwm_driver = {
	.driver =
	{
		.name = "modulated-pwm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(modulated_pwm_of_match),
	},
	.probe = modulated_pwm_probe,
	.remove = modulated_pwm_remove,
};

module_platform_driver(modulated_pwm_driver);

MODULE_DESCRIPTION("Modulated PWM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:modulated-pwm");
