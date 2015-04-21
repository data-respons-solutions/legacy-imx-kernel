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
#include <linux/pwm_buzzer.h>
#include <linux/slab.h>

struct pwm_buzzer_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		pwmId;
	unsigned int		period;
	unsigned int		isOn;
	char				name[80];
};

static DEFINE_MUTEX(sysfs_lock);

static ssize_t pwm_buzzer_show(struct device *dev,  struct device_attribute *attr, char *buf) {

	struct pwm_buzzer_data *pb = dev_get_drvdata(dev);
	ssize_t status;
	mutex_lock(&sysfs_lock);
	if (pb->isOn)
		status = sprintf(buf, "on\n");
	else
		status = sprintf(buf, "off\n");
	mutex_unlock(&sysfs_lock);
	return status;
}

static ssize_t pwm_buzzer_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t sz) {
	struct pwm_buzzer_data *pb = dev_get_drvdata(dev);
	ssize_t status = sz;
	mutex_lock(&sysfs_lock);
	if (sysfs_streq(buf, "on")) {
		pb->isOn = 1;
		pwm_config(pb->pwm, pb->period/2, pb->period);
		pwm_enable(pb->pwm);
	}
	else if (sysfs_streq(buf, "off")) {
		pb->isOn = 0;
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	}
	else
		status = -EINVAL;
	mutex_unlock(&sysfs_lock);
	return status;
}

static DEVICE_ATTR(buzz, 0644, pwm_buzzer_show, pwm_buzzer_store);


static int pwm_buzzer_probe(struct platform_device *pdev)
{
	/* struct platform_pwm_buzzer_data *data = pdev->dev.platform_data; */
	struct pwm_buzzer_data *pb;
	int ret;
	struct device_node *np = pdev->dev.of_node;

	dev_dbg(&pdev->dev, "%s:\n", __func__);

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	strcpy(pb->name, "buzzer");

	if (np) {
		if ( of_property_read_string(np, "pwm-name", (const char**)&pb->name))
			dev_info(&pdev->dev, "No given name in DT, using buzzer\n");
	}

	pb->dev = &pdev->dev;
	pb->isOn = 0;

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for buzzer %s\n", pb->name);
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	}
	pb->period = pwm_get_period(pb->pwm);
	dev_info(&pdev->dev, "%s: got pwm %d, period %d ns\n", __func__, pb->pwmId, pb->period);

	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_buzz.attr);
	platform_set_drvdata(pdev, pb);
	return 0;

err_pwm:
err_alloc:

	return ret;
}

static int pwm_buzzer_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_buzzer_data *pb = dev_get_drvdata(&bl->dev);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_buzz.attr);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_buzzer_suspend(struct device *dev)
{
	struct pwm_buzzer_data *pb = dev_get_drvdata(dev);

	pwm_disable(pb->pwm);
	return 0;
}

static int pwm_buzzer_resume(struct device *dev)
{
	struct pwm_buzzer_data *pb = dev_get_drvdata(dev);
	pwm_enable(pb->pwm);
	return 0;
}
#else
#define pwm_buzzer_suspend	NULL
#define pwm_buzzer_resume	NULL
#endif

static struct of_device_id pwm_buzzer_of_match[] = {
	{ .compatible = "pwm-buzzer", },
	{ }
};

static SIMPLE_DEV_PM_OPS(pwm_buzzer_pm_ops, pwm_buzzer_suspend, pwm_buzzer_resume);

MODULE_DEVICE_TABLE(of, pwm_buzzer_of_match);

static struct platform_driver pwm_buzzer_driver = {
	.driver		= {
		.name	= "pwm-buzzer",
		.owner	= THIS_MODULE,
		.pm	    = &pwm_buzzer_pm_ops,
		.of_match_table = of_match_ptr(pwm_buzzer_of_match),
	},
	.probe		= pwm_buzzer_probe,
	.remove		= pwm_buzzer_remove,
};

module_platform_driver(pwm_buzzer_driver);

MODULE_DESCRIPTION("PWM based Buzzer Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-buzzer");

