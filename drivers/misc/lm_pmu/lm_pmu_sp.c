
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
	//lm_pmu_get_valids(pmu);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		if (pmu->psu_valids & 1)
		val->intval = pmu->psu_valids & 1 ? 1 : 0;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}



/**********************************************************************
 *  Parse DT
 */

static int lm_pmu_dt(struct lm_pmu_sp *pmu)
{
	struct device *dev = &pmu->priv->spi_dev->dev;
	struct device_node *np = dev->of_node;
	int n, num_gpio;

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
	}

	return 0;

cleanup:
	lm_pmu_deinit(pmu->priv);
	return ret;
}

static int lm_pmu_sp_remove(struct spi_device *spi)
{
	struct lm_pmu_private *priv = dev_get_drvdata(&spi->dev);
	//struct lm_pmu_lb *pmu = lm_pmu_get_subclass_data(priv);
	lm_pmu_deinit(priv);
	return 0;
}

static struct spi_driver lm_pmu_sp_driver = {
	.driver = {
		.name	= "lm_pmu_sp",
		.owner	= THIS_MODULE,
		.of_match_table = lm_pmu_sp_dt_ids,
	},
	.probe	= lm_pmu_sp_probe,
	.remove	= lm_pmu_sp_remove,
	.id_table = lm_pmu_sp_ids,
};

module_spi_driver(lm_pmu_sp_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus Simpad uC driver");
