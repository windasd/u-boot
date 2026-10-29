// SPDX-License-Identifier: GPL-2.0+
/*
 * Actions Semi S900 display engine: video layer 0 on path 0 scans the
 * framebuffer out to the HDMI transmitter. Order and values as in the vendor
 * U-Boot 2015.07 drivers/video/owl/dss/de/de_s900.c.
 */

#include <log.h>
#include <time.h>
#include <asm/io.h>
#include <asm/arch-owl/sps.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include "s900_display.h"

#define DE_QOS			0x0014
#define DE_MAX_OUTSTANDING	0x0018
#define DE_PATH_CTL(n)		(0x0100 + (n) * 0x100)	/* bit n: video layer n */
#define DE_PATH_FCR(n)		(0x0104 + (n) * 0x100)	/* bit 0: latch, self-clearing */
#define DE_PATH_EN(n)		(0x0108 + (n) * 0x100)
#define DE_PATH_BK(n)		(0x010c + (n) * 0x100)
#define DE_PATH_SIZE(n)		(0x0110 + (n) * 0x100)
#define DE_OVL_COOR(p, n)	(0x0120 + (p) * 0x100 + (n) * 4)
#define DE_OVL_ALPHA_CFG(p, n)	(0x0130 + (p) * 0x100 + (n) * 4)
#define DE_OVL_CFG(n)		(0x0400 + (n) * 0x100)
#define DE_OVL_ISIZE(n)		(0x0404 + (n) * 0x100)
#define DE_OVL_OSIZE(n)		(0x0408 + (n) * 0x100)
#define DE_OVL_SR(n)		(0x040c + (n) * 0x100)
#define DE_OVL_SCOEF(n, i)	(0x0410 + (n) * 0x100 + (i) * 4)
#define DE_OVL_BA0(n)		(0x0430 + (n) * 0x100)
#define DE_OVL_STR(n)		(0x0448 + (n) * 0x100)

#define DE_OVL_CFG_FMT		GENMASK(3, 0)
#define DE_OVL_CFG_FMT_BGRA32	4	/* x8r8g8b8 in memory */
#define DE_OVL_CFG_ROTATE	GENMASK(21, 20)

#define S900_SHARESRAM_CTL		0xe0240004
#define S900_DMM_AXI_NORMAL_PRIORITY	0xe029002c
#define S900_DMM_AXI_DE_PRIORITY	0xe0290074

#define DE_GO_TIMEOUT_MS	60	/* three frames at 50 Hz */
#define DE_SCOEF_NUM		8

/* Polyphase scaler coefficients, by input/output area ratio (x10) */
static const u32 de_coef_zoomin[DE_SCOEF_NUM] = {	/* <= 1 */
	0x00004000, 0xff073efc, 0xfe1038fa, 0xfc1b30f9,
	0xfa2626fa, 0xf9301bfc, 0xfa3810fe, 0xfc3e07ff,
};

static const u32 de_coef_half[DE_SCOEF_NUM] = {	/* <= 2 */
	0x00004000, 0x00083800, 0x00103000, 0x00182800,
	0x00202000, 0x00281800, 0x00301000, 0x00380800,
};

static const u32 de_coef_smaller[DE_SCOEF_NUM] = {	/* > 2, e.g. 1080p to 720p */
	0x00102010, 0x02121e0e, 0x04141c0c, 0x06161a0a,
	0x08181808, 0x0a1a1606, 0x0c1c1404, 0x0e1e1202,
};

/*
 * An access to the DE while its clock is gated, its power domain is off or it
 * is held in reset hangs the bus, and the board resets.
 */
bool s900_de_accessible(void)
{
	return (readl(S900_CMU_DEVCLKEN0) & DEVCLKEN0_DE) &&
	       (readl(S900_CMU_DEVRST0) & DEVRST0_DE) &&
	       (readl(S900_SPS_PG_ACK) & SPS_ACK_DE);
}

static bool de_check(struct s900_display *d, u32 reg)
{
	if (d->err)
		return false;
	if (s900_de_accessible())
		return true;
	log_err("s900: DE not clocked or powered, refusing access to %#x\n", reg);
	d->err = -EIO;
	return false;
}

static void de_writel(struct s900_display *d, u32 reg, u32 val)
{
	if (de_check(d, reg))
		writel(val, d->de + reg);
}

static u32 de_readl(struct s900_display *d, u32 reg)
{
	return de_check(d, reg) ? readl(d->de + reg) : 0;
}

static void de_clrsetbits(struct s900_display *d, u32 reg, u32 clr, u32 set)
{
	de_writel(d, reg, (de_readl(d, reg) & ~clr) | set);
}

/* Reset, power, clock: vendor de_s900_device_power_on() */
int s900_de_power_on(struct s900_display *d)
{
	int ret;

	reset_assert(&d->de_rst);
	ret = s900_sps_power_on(SPS_PG_DE, SPS_ACK_DE, 100);
	if (ret) {
		log_err("s900: DE power domain not acknowledged\n");
		return ret;
	}

	/* DE_CLK from ASSIST_PLL (500 MHz), all dividers 1 */
	writel(0, S900_CMU_DECLK);
	ret = clk_enable(&d->de_clk);
	if (ret)
		return ret;
	mdelay(1);
	reset_deassert(&d->de_rst);
	mdelay(1);

	return s900_de_accessible() ? 0 : -EIO;
}

/* Bus set-up the vendor kernel expects too: vendor de_s900_device_init() */
void s900_de_init(struct s900_display *d)
{
	if (!(readl(S900_CMU_DEVCLKEN0) & DEVCLKEN0_SHARESRAM))
		setbits_le32(S900_CMU_DEVCLKEN0, DEVCLKEN0_SHARESRAM);
	setbits_le32(S900_SHARESRAM_CTL, 0x3);	/* HDMI and eDP share the SRAM */

	de_writel(d, DE_MAX_OUTSTANDING, 0x1f1f);
	de_writel(d, DE_QOS, 0xcccc);

	writel(0, S900_DMM_AXI_DE_PRIORITY);
	clrbits_le32(S900_DMM_AXI_NORMAL_PRIORITY, BIT(4));
	setbits_le32(S900_DMM_AXI_NORMAL_PRIORITY, BIT(4));
}

/* Path 0 at the output size, layer 0 scaling the framebuffer to it */
void s900_de_setup(struct s900_display *d, const struct s900_mode *mode)
{
	u32 osize = (mode->height - 1) << 16 | (mode->width - 1);
	u32 w_factor = DIV_ROUND_UP(S900_FB_WIDTH * 8192, mode->width);
	u32 h_factor = DIV_ROUND_UP(S900_FB_HEIGHT * 8192, mode->height);
	u32 ratio = S900_FB_WIDTH * S900_FB_HEIGHT * 10 /
		    (mode->width * mode->height);
	const u32 *coef;
	int i;

	if (ratio <= 10)
		coef = de_coef_zoomin;
	else if (ratio <= 20)
		coef = de_coef_half;
	else
		coef = de_coef_smaller;

	de_writel(d, DE_PATH_SIZE(0), osize);
	de_writel(d, DE_PATH_BK(0), 0);

	de_writel(d, DE_OVL_BA0(0), S900_FB_BASE);
	de_writel(d, DE_OVL_STR(0), S900_FB_STRIDE / 8);
	de_clrsetbits(d, DE_OVL_CFG(0), DE_OVL_CFG_FMT | DE_OVL_CFG_ROTATE,
		      DE_OVL_CFG_FMT_BGRA32);
	de_writel(d, DE_OVL_ISIZE(0),
		  (S900_FB_HEIGHT - 1) << 16 | (S900_FB_WIDTH - 1));
	de_writel(d, DE_OVL_OSIZE(0), osize);
	de_writel(d, DE_OVL_SR(0), h_factor << 16 | w_factor);
	for (i = 0; i < DE_SCOEF_NUM; i++)
		de_writel(d, DE_OVL_SCOEF(0, i), coef[i]);
	de_writel(d, DE_OVL_COOR(0, 0), 0);
	de_writel(d, DE_OVL_ALPHA_CFG(0, 0), 0x1ff);
}

/*
 * Enable layer 0 and path 0, then latch the settings. The latch bit clears
 * at the next frame, so it staying set means the DE gets no timing from the
 * HDMI side.
 */
bool s900_de_go(struct s900_display *d)
{
	ulong start;

	de_clrsetbits(d, DE_PATH_CTL(0), 0, BIT(0));
	de_clrsetbits(d, DE_PATH_EN(0), 0, BIT(0));
	de_clrsetbits(d, DE_PATH_FCR(0), 0, BIT(0));

	start = get_timer(0);
	while (de_readl(d, DE_PATH_FCR(0)) & BIT(0)) {
		if (d->err || get_timer(start) > DE_GO_TIMEOUT_MS)
			return false;
		udelay(100);
	}

	return true;
}
