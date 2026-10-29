// SPDX-License-Identifier: GPL-2.0+
/*
 * Common clock driver for Actions Semi SoCs.
 *
 * Copyright (C) 2015 Actions Semi Co., Ltd.
 * Copyright (C) 2018 Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <dm.h>
#include <dm/lists.h>
#include <reset-uclass.h>
#include "clk_owl.h"
#include <asm/io.h>
#if defined(CONFIG_MACH_S900)
#include <asm/arch-owl/regs_s900.h>
#include <dt-bindings/clock/actions,s900-cmu.h>
#include <dt-bindings/reset/actions,s900-reset.h>
#elif defined(CONFIG_MACH_S700)
#include <asm/arch-owl/regs_s700.h>
#include <dt-bindings/clock/actions,s700-cmu.h>
#endif
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#if defined(CONFIG_MACH_S900)
#define CMU_DEVCLKEN0_SD(x)	BIT(5 + (x))	/* SD0..SD2: bits 5..7 */
#define CMU_DEVCLKEN0_DMAC	BIT(1)
#define CMU_DEVCLKEN0_DE	BIT(8)
#define CMU_DEVCLKEN0_HDMIA	BIT(22)
/* ASSIST_PLL, enabled by owl_clk_init(), runs at a fixed 500 MHz */
#define S900_ASSISTPLL_RATE	500000000
#else
#define CMU_DEVCLKEN0_SD(x)	BIT(22 + (x))	/* SD0..SD2: bits 22..24 */
#define CMU_DEVCLKEN0_DMAC	BIT(17)
#endif

/* SDx_CLK: bits 4..0 divide by 1..32, bit 8 adds a divide by 128 */
#define CMU_SDCLK_DIV_MASK	0x1f
#define CMU_SDCLK_DIV128	BIT(8)
#define CMU_SDCLK_SRC_NANDPLL	BIT(9)	/* else DEV_PLL */
#define CMU_SDCLK_DIV_MAX	32

/* PLL multiplier field of CMU_COREPLL / CMU_DEVPLL */
#define CMU_PLL_FACTOR_MASK	0xff

void owl_clk_init(struct owl_clk_priv *priv)
{
	u32 bus_clk = 0, core_pll, dev_pll;

#if defined(CONFIG_MACH_S900)
	/* Enable ASSIST_PLL */
	setbits_le32(priv->base + CMU_ASSISTPLL, BIT(0));
	udelay(PLL_STABILITY_WAIT_US);
#endif

	/* Source HOSC to DEV_CLK */
	clrbits_le32(priv->base + CMU_DEVPLL, CMU_DEVPLL_CLK);

	/* Configure BUS_CLK */
	bus_clk |= (CMU_PDBGDIV_DIV | CMU_PERDIV_DIV | CMU_NOCDIV_DIV |
			CMU_DMMCLK_SRC | CMU_APBCLK_DIV | CMU_AHBCLK_DIV |
			CMU_NOCCLK_SRC | CMU_CORECLK_HOSC);
	writel(bus_clk, priv->base + CMU_BUSCLK);

	udelay(PLL_STABILITY_WAIT_US);

	/*
	 * Configure CORE_PLL. Replace the multiplier instead of OR-ing into
	 * whatever the first-stage loader left there.
	 */
	core_pll = readl(priv->base + CMU_COREPLL);
	core_pll &= ~CMU_PLL_FACTOR_MASK;
	core_pll |= (CMU_COREPLL_EN | CMU_COREPLL_HOSC_EN | CMU_COREPLL_OUT);
	writel(core_pll, priv->base + CMU_COREPLL);

	udelay(PLL_STABILITY_WAIT_US);

	/* Configure DEV_PLL */
	dev_pll = readl(priv->base + CMU_DEVPLL);
	dev_pll &= ~CMU_PLL_FACTOR_MASK;
	dev_pll |= (CMU_DEVPLL_EN | CMU_DEVPLL_OUT);
	writel(dev_pll, priv->base + CMU_DEVPLL);

	udelay(PLL_STABILITY_WAIT_US);

	/* Source CORE_PLL for CORE_CLK */
	clrsetbits_le32(priv->base + CMU_BUSCLK, CMU_CORECLK_MASK,
			CMU_CORECLK_CPLL);

	/* Source DEV_PLL for DEV_CLK */
	setbits_le32(priv->base + CMU_DEVPLL, CMU_DEVPLL_CLK);

	udelay(PLL_STABILITY_WAIT_US);
}

#if defined(CONFIG_MACH_S900)
/* I2C0..I2C5 gates as in Linux owl-s900.c: I2C4 in DEVCLKEN0, the rest in DEVCLKEN1 */
static void s900_i2c_clk_gate(struct owl_clk_priv *priv, int i2c, bool on)
{
	static const u8 bits[] = { 14, 15, 30, 31, 17, 1 };
	u32 reg = i2c == 4 ? CMU_DEVCLKEN0 : CMU_DEVCLKEN1;

	if (on)
		setbits_le32(priv->base + reg, BIT(bits[i2c]));
	else
		clrbits_le32(priv->base + reg, BIT(bits[i2c]));
}
#endif

int owl_clk_enable(struct clk *clk)
{
	struct owl_clk_priv *priv = dev_get_priv(clk->dev);
	enum owl_soc model = dev_get_driver_data(clk->dev);

	switch (clk->id) {
	case CLK_UART5:
		if (model != S900)
			return -EINVAL;
		/* Source HOSC for UART5 interface */
		clrbits_le32(priv->base + CMU_UART5CLK, CMU_UARTCLK_SRC_DEVPLL);
		/* Enable UART5 interface clock */
		setbits_le32(priv->base + CMU_DEVCLKEN1, CMU_DEVCLKEN1_UART5);
		break;
	case CLK_UART3:
		if (model != S700)
			return -EINVAL;
		/* Source HOSC for UART3 interface */
		clrbits_le32(priv->base + CMU_UART3CLK, CMU_UARTCLK_SRC_DEVPLL);
		/* Enable UART3 interface clock */
		setbits_le32(priv->base + CMU_DEVCLKEN1, CMU_DEVCLKEN1_UART3);
		break;
	case CLK_RMII_REF:
	case CLK_ETHERNET:
		setbits_le32(priv->base + CMU_DEVCLKEN1, CMU_DEVCLKEN1_ETH);
		setbits_le32(priv->base + CMU_ETHERNETPLL, 5);
		break;
	case CLK_SD0:
	case CLK_SD1:
	case CLK_SD2:
		setbits_le32(priv->base + CMU_DEVCLKEN0,
			     CMU_DEVCLKEN0_SD(clk->id - CLK_SD0));
		break;
	case CLK_DMAC:
		setbits_le32(priv->base + CMU_DEVCLKEN0, CMU_DEVCLKEN0_DMAC);
		break;
#if defined(CONFIG_MACH_S900)
	case CLK_DE0:
		setbits_le32(priv->base + CMU_DEVCLKEN0, CMU_DEVCLKEN0_DE);
		break;
	case CLK_HDMI_AUDIO:
		setbits_le32(priv->base + CMU_DEVCLKEN0, CMU_DEVCLKEN0_HDMIA);
		break;
	case CLK_I2C0:
	case CLK_I2C1:
	case CLK_I2C2:
	case CLK_I2C3:
	case CLK_I2C4:
	case CLK_I2C5:
		s900_i2c_clk_gate(priv, clk->id - CLK_I2C0, true);
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

int owl_clk_disable(struct clk *clk)
{
	struct owl_clk_priv *priv = dev_get_priv(clk->dev);
	enum owl_soc model = dev_get_driver_data(clk->dev);

	switch (clk->id) {
	case CLK_UART5:
		if (model != S900)
			return -EINVAL;
		/* Disable UART5 interface clock */
		clrbits_le32(priv->base + CMU_DEVCLKEN1, CMU_DEVCLKEN1_UART5);
		break;
	case CLK_UART3:
		if (model != S700)
			return -EINVAL;
		/* Disable UART3 interface clock */
		clrbits_le32(priv->base + CMU_DEVCLKEN1, CMU_DEVCLKEN1_UART3);
		break;
	case CLK_RMII_REF:
	case CLK_ETHERNET:
		clrbits_le32(priv->base + CMU_DEVCLKEN1, CMU_DEVCLKEN1_ETH);
		break;
	case CLK_SD0:
	case CLK_SD1:
	case CLK_SD2:
		clrbits_le32(priv->base + CMU_DEVCLKEN0,
			     CMU_DEVCLKEN0_SD(clk->id - CLK_SD0));
		break;
	case CLK_DMAC:
		/* Linux and the vendor kernel expect the DMAC clock running */
		break;
#if defined(CONFIG_MACH_S900)
	case CLK_DE0:
	case CLK_HDMI_AUDIO:
		/* The HDMI scanout continues into the OS (simple-framebuffer) */
		break;
	case CLK_I2C0:
	case CLK_I2C1:
	case CLK_I2C2:
	case CLK_I2C3:
	case CLK_I2C4:
	case CLK_I2C5:
		s900_i2c_clk_gate(priv, clk->id - CLK_I2C0, false);
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static ulong get_sd_parent_rate(struct owl_clk_priv *priv, u32 dev_index)
{
	ulong rate;
	u32 reg;

	reg = readl(priv->base + (CMU_SD0CLK + dev_index * 0x4));
	/* Clock output of DEV/NAND_PLL
	 * Range: 48M ~ 756M
	 * Frequency= PLLCLK * 6
	 */
	if (reg & 0x200)
		rate = readl(priv->base + CMU_NANDPLL) & 0x7f;
	else
		rate = readl(priv->base + CMU_DEVPLL) & 0x7f;

	rate *= 6000000;

	return rate;
}

static ulong owl_get_sd_clk_rate(struct owl_clk_priv *priv, int sd_index)
{
	uint div, val;
	ulong parent_rate = get_sd_parent_rate(priv, sd_index);

	val = readl(priv->base + (CMU_SD0CLK + sd_index * 0x4));
	div = (val & CMU_SDCLK_DIV_MASK) + 1;
	if (val & CMU_SDCLK_DIV128)
		div *= 128;

	return (parent_rate / div);
}

static ulong owl_set_sd_clk_rate(struct owl_clk_priv *priv, ulong rate,
				 int sd_index)
{
	uint div, val;
	ulong parent_rate;

	if (rate == 0)
		return rate;

	/*
	 * Run from DEV_PLL, as the vendor U-Boot does, rather than from
	 * whichever source the first-stage loader left selected: NAND_PLL
	 * may not be running. Switch through the slowest divider.
	 */
	clrsetbits_le32(priv->base + (CMU_SD0CLK + sd_index * 0x4),
			CMU_SDCLK_SRC_NANDPLL,
			CMU_SDCLK_DIV128 | CMU_SDCLK_DIV_MASK);
	parent_rate = get_sd_parent_rate(priv, sd_index);

	/* Round up so the card never sees more than the requested rate */
	div = DIV_ROUND_UP(parent_rate, rate);

	val = readl(priv->base + (CMU_SD0CLK + sd_index * 0x4));
	val &= ~(CMU_SDCLK_DIV128 | CMU_SDCLK_DIV_MASK);
	if (div > CMU_SDCLK_DIV_MAX) {
		/* Only 1..32 and 128 * (1..32) can be programmed */
		div = min_t(uint, DIV_ROUND_UP(div, 128), CMU_SDCLK_DIV_MAX);
		val |= CMU_SDCLK_DIV128;
	}
	val |= (div - 1) & CMU_SDCLK_DIV_MASK;
	writel(val, priv->base + (CMU_SD0CLK + sd_index * 0x4));

	return owl_get_sd_clk_rate(priv, sd_index);
}

static ulong owl_clk_get_rate(struct clk *clk)
{
	struct owl_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate;

	switch (clk->id) {
	case CLK_SD0:
	case CLK_SD1:
	case CLK_SD2:
		rate = owl_get_sd_clk_rate(priv, clk->id - CLK_SD0);
		break;
#if defined(CONFIG_MACH_S900)
	case CLK_I2C0:
	case CLK_I2C1:
	case CLK_I2C2:
	case CLK_I2C3:
	case CLK_I2C4:
	case CLK_I2C5:
		rate = S900_ASSISTPLL_RATE / 5;
		break;
#endif
	default:
		return -ENOENT;
	}

	return rate;
}

static ulong owl_clk_set_rate(struct clk *clk, ulong rate)
{
	struct owl_clk_priv *priv = dev_get_priv(clk->dev);
	ulong new_rate;

	switch (clk->id) {
	case CLK_SD0:
	case CLK_SD1:
	case CLK_SD2:
		new_rate = owl_set_sd_clk_rate(priv, rate, clk->id - CLK_SD0);
		break;
	default:
		return -ENOENT;
	}

	return new_rate;
}

#if CONFIG_IS_ENABLED(DM_RESET) && defined(CONFIG_MACH_S900)
/*
 * Reset lines in CMU_DEVRST0/1, active low (Linux owl-reset.c). Only the
 * lines U-Boot drivers use are listed; anything else is rejected.
 */
struct owl_reset_map {
	u16 reg;
	u32 bit;
};

static const struct owl_reset_map owl_resets[] = {
	[RESET_DMAC]	= { CMU_DEVRST0, BIT(0) },
	[RESET_DE]	= { CMU_DEVRST0, BIT(7) },
	[RESET_SD0]	= { CMU_DEVRST0, BIT(4) },
	[RESET_SD1]	= { CMU_DEVRST0, BIT(5) },
	[RESET_SD2]	= { CMU_DEVRST0, BIT(9) },
	[RESET_HDMI]	= { CMU_DEVRST1, BIT(2) },
	[RESET_I2C0]	= { CMU_DEVRST1, BIT(12) },
	[RESET_I2C1]	= { CMU_DEVRST1, BIT(13) },
	[RESET_I2C2]	= { CMU_DEVRST1, BIT(18) },
	[RESET_I2C3]	= { CMU_DEVRST1, BIT(19) },
	[RESET_I2C4]	= { CMU_DEVRST1, BIT(22) },
	[RESET_I2C5]	= { CMU_DEVRST1, BIT(23) },
};

static int owl_reset_request(struct reset_ctl *reset_ctl)
{
	if (reset_ctl->id >= ARRAY_SIZE(owl_resets) ||
	    !owl_resets[reset_ctl->id].bit)
		return -EINVAL;

	return 0;
}

static int owl_reset_set(struct reset_ctl *reset_ctl, bool deassert)
{
	const struct owl_reset_map *map = &owl_resets[reset_ctl->id];
	phys_addr_t base = dev_read_addr(reset_ctl->dev);

	if (deassert)
		setbits_le32(base + map->reg, map->bit);
	else
		clrbits_le32(base + map->reg, map->bit);

	return 0;
}

static int owl_reset_assert(struct reset_ctl *reset_ctl)
{
	return owl_reset_set(reset_ctl, false);
}

static int owl_reset_deassert(struct reset_ctl *reset_ctl)
{
	return owl_reset_set(reset_ctl, true);
}

static const struct reset_ops owl_reset_ops = {
	.request	= owl_reset_request,
	.rst_assert	= owl_reset_assert,
	.rst_deassert	= owl_reset_deassert,
};

U_BOOT_DRIVER(owl_reset) = {
	.name		= "owl_reset",
	.id		= UCLASS_RESET,
	.ops		= &owl_reset_ops,
};

static int owl_clk_bind(struct udevice *dev)
{
	/* The CMU node is also the reset controller (#reset-cells) */
	return device_bind_driver_to_node(dev, "owl_reset", "reset",
					  dev_ofnode(dev), NULL);
}
#else
static int owl_clk_bind(struct udevice *dev)
{
	return 0;
}
#endif

static int owl_clk_probe(struct udevice *dev)
{
	struct owl_clk_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr(dev);
	if (priv->base == FDT_ADDR_T_NONE)
		return -EINVAL;

	/* setup necessary clocks */
	owl_clk_init(priv);

	return 0;
}

static const struct clk_ops owl_clk_ops = {
	.enable = owl_clk_enable,
	.disable = owl_clk_disable,
	.get_rate = owl_clk_get_rate,
	.set_rate = owl_clk_set_rate,
};

static const struct udevice_id owl_clk_ids[] = {
#if defined(CONFIG_MACH_S900)
	{ .compatible = "actions,s900-cmu", .data = S900 },
#elif defined(CONFIG_MACH_S700)
	{ .compatible = "actions,s700-cmu", .data = S700 },
#endif
	{ }
};

U_BOOT_DRIVER(clk_owl) = {
	.name		= "clk_owl",
	.id		= UCLASS_CLK,
	.of_match	= owl_clk_ids,
	.ops		= &owl_clk_ops,
	.priv_auto	= sizeof(struct owl_clk_priv),
	.bind		= owl_clk_bind,
	.probe		= owl_clk_probe,
};
