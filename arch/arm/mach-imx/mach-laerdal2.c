/*
 * Copyright 2015 Data Respons AS
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

#ifndef __KERNEL__
#define __init
#define __initdata
#endif

#define ESAI_AUDIO_MCLK 24576000

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}


static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	dev_info(&dev->dev, "ar8031_phy_fixup\n");
	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

#if 0
	/* Enable PLL */
	phy_write(dev, 0x1d, 0x1f);
	val = phy_read(dev, 0x1e);
	val |= 0x0004;
	phy_write(dev, 0x1e, val);
#endif
	return 0;
}

#define PHY_ID_AR8031	0x004dd074

static void __init laerdal2_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
	}
}

static int imx_get_boot_mode_reg(u32 *cfg, u32* bmr)
{
	struct device_node *np;
	void __iomem *src_base;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx51-src");
	if (!np)
		return 0;
	src_base = of_iomap(np, 0);
	WARN_ON(!src_base);
	if (!src_base)
		return -ENODEV;

	*cfg = readl_relaxed(src_base + 0x04);
	*bmr = readl_relaxed(src_base + 0x1c);
	pr_info("%s: boot mode 0x%08x\n", __func__, *bmr);
	pr_info("%s: cfg reg 0x%08x\n", __func__, *cfg);
	return 0;
}

static void __init imx6q_audio_clock_init(void)
{
	struct clk *pll4_audio_div, *esai_extal;
	pr_info("%s: Setting ESAI clock frequency to %d\n", __func__, ESAI_AUDIO_MCLK );
	pll4_audio_div = clk_get_sys(NULL, "pll4_audio_div");
	esai_extal = clk_get_sys(NULL, "esai_extal");
	if (IS_ERR(pll4_audio_div) || IS_ERR(esai_extal)) {
		pr_err("%s: clocks are %p %p\n", __func__,
				pll4_audio_div, esai_extal);
		return;
	}

	if (clk_set_rate(pll4_audio_div, 786432000))
		pr_err("%s: clk_set_rate(pll4_audio_div, 786432000)\n", __func__);
	if (clk_set_rate(esai_extal, ESAI_AUDIO_MCLK))
		pr_err("%s: clk_set_rate(esai_extal, %d)\n", __func__, ESAI_AUDIO_MCLK);
}

static struct platform_device laerdal2_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init imx6q_1588_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

}

static inline void imx6q_enet_init(void)
{
#if 0
	int ret;
	struct clk *enet_ref = clk_get_sys(NULL, "enet_ref");
	if ( IS_ERR(enet_ref) )
		pr_err("%s: Unable to get enet_ref clock [%d]\n", __func__, (int)enet_ref);
	else {
		ret = clk_set_rate(enet_ref, 125000000);
		if (ret)
			pr_err("%s: Unable to set enet_ref clock\n", __func__);
	}
#endif
	imx6_enet_mac_init("fsl,imx6q-fec");
	laerdal2_enet_phy_init();
	//imx6q_1588_init();
}

static void __init laerdal2_init_machine(void)
{
	u32 bmr, cfg;
	struct device *parent;
	u32 val;
	int ret;
	int gpio_nr;
	unsigned long flags;
	struct device_node *user_gpios, *it;

	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());

	mxc_arch_reset_init_dt();
	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");


	of_platform_populate(NULL, of_default_bus_match_table, NULL, parent);

	user_gpios = of_find_node_by_name(NULL, "user-gpios");
	if (user_gpios) {	/* Iterate nodes */
		it = NULL;
		while ((it = of_get_next_available_child(user_gpios, it))) {
			pr_info("%s: Setting up gpio %s\n", __func__, of_node_full_name(it));
			gpio_nr = of_get_gpio(it, 0);

			if (!gpio_is_valid(gpio_nr)) {
				pr_err("%s: Could not get gpio for %s\n", __func__, of_node_full_name(it));
				continue;
			}
			if (of_property_read_u32(it, "value", &val) == 0)	/* Output pin */
				flags = val ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_DIR_IN;

			if ( of_property_read_bool(it, "bidir"))
				flags |= GPIOF_EXPORT_CHANGEABLE;

			if (of_property_read_bool(it, "opendrain"))
				flags |= GPIOF_OPEN_DRAIN;

			flags |= GPIOF_EXPORT;
			ret = gpio_request_one(gpio_nr, flags, it->name);
			if (ret < 0) {
				pr_err("%s: Could not request gpio %d\n", __func__, gpio_nr);
				continue;
			}
		}
		of_node_put(user_gpios);
	}

	imx6q_enet_init();
	imx_anatop_init();
	cpu_is_imx6q() ?  imx6q_pm_init() : imx6dl_pm_init();

	imx_get_boot_mode_reg(&cfg, &bmr);
	/* imx6q_audio_clock_init(); */
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_996MHZ		0x2
#define OCOTP_CFG3_SPEED_852MHZ		0x1

static void __init laerdal2_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz;
	 * 2b'10: 996000000Hz;
	 * 2b'01: 852000000Hz; -- i.MX6Q Only, exclusive with 996MHz.
	 * 2b'00: 792000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;

	if (val != OCOTP_CFG3_SPEED_1P2GHZ)
		if (dev_pm_opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");
	if (val < OCOTP_CFG3_SPEED_996MHZ)
		if (dev_pm_opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 996 MHz OPP\n");
	if (cpu_is_imx6q()) {
		if (val != OCOTP_CFG3_SPEED_852MHZ)
			if (dev_pm_opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 852 MHz OPP\n");
	}

	if (IS_ENABLED(CONFIG_MX6_VPU_352M)) {
		if (dev_pm_opp_disable(cpu_dev, 396000000))
			pr_warn("failed to disable 396MHz OPP\n");
		pr_info("remove 396MHz OPP for VPU running at 352MHz!\n");
	}

put_node:
	of_node_put(np);
}

static void __init laerdal2_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	laerdal2_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static void __init laerdal2_init_late(void)
{
	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		laerdal2_opp_init();
		platform_device_register(&laerdal2_cpufreq_pdev);
	}

}

static void __init laerdal2_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
	imx6_busfreq_map_io();
}

static void __init laerdal2_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}



static const char *laerdal2_dt_compat[] __initdata = {
	"datarespons,laerdal2",
	NULL,
};

DT_MACHINE_START(laerdal2, "LAERDAL2 DT")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= laerdal2_map_io,
	.init_irq	= laerdal2_init_irq,
	.init_machine	= laerdal2_init_machine,
	.init_late      = laerdal2_init_late,
	.dt_compat	= laerdal2_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
