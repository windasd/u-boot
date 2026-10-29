// SPDX-License-Identifier: GPL-2.0+
/*
 * Actions Semi S900 HDMI transmitter, RGB 4:4:4 at 24 bit from the DE, for
 * the 720p and 1080p CEA modes. Order and values as in the vendor U-Boot
 * 2015.07 drivers/video/owl/dss/controller/hdmic/ip-sx00.c
 * (ip_sx00_video_enable()).
 */

#include <log.h>
#include <time.h>
#include <asm/io.h>
#include <asm/arch-owl/sps.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include "s900_display.h"

#define HDMI_VICTL		0x0000
#define HDMI_VIVSYNC		0x0004
#define HDMI_VIVHSYNC		0x0008
#define HDMI_VIALSEOF		0x000c
#define HDMI_VIALSEEF		0x0010
#define HDMI_VIADLSE		0x0014
#define HDMI_CR			0x0054
#define HDMI_SCHCR		0x0058
#define HDMI_ICR		0x005c
#define HDCP_KOWR		0x00a8
#define HDCP_OWR		0x00ac
#define TMDS_EODR0		0x00c0
#define HDMI_GCPCR		0x00e4
#define HDMI_RPCR		0x00e8
#define HDMI_OPCR		0x00f0
#define HDMI_DIPCCR		0x00f4
#define HDMI_ORP6PH		0x00f8
#define HDMI_ORSP6W(n)		(0x00fc + (n) * 4)
#define HDMI_TX_1		0x0154
#define HDMI_TX_2		0x0158
#define CEC_DDC_HPD		0x015c

#define HDMI_CR_VIDEO_EN	BIT(0)
#define HDMI_CR_HPD_EN		BIT(28)	/* hotplug detection */
#define HDMI_CR_HPD		BIT(29)	/* sink present */
#define HDMI_CR_HPD_PENDING	BIT(30)	/* write 1 to clear */
#define HDMI_CR_HPD_IRQ		BIT(31)
#define HDMI_TX_2_LANES		GENMASK(11, 8)
#define CEC_DDC_HPD_SINK	GENMASK(15, 8)	/* any bit set: sink present */
#define CEC_DDC_HPD_CAL_START	BIT(20)
#define CEC_DDC_HPD_CAL_DONE	BIT(24)
#define HDMI_OPCR_WRITE		BIT(31)	/* copy ORP6PH..ORSP6W7 into packet RAM */

#define TVOUTPLL_EN		BIT(0)
#define TVOUTPLL_HOSC		BIT(8)	/* 24 MHz reference */
#define TVOUTPLL_SEL		GENMASK(18, 16)
#define TVOUTPLLDEBUG0_LOCK	BIT(14)

#define HPD_TIMEOUT_MS		200
#define CAL_TIMEOUT_MS		20	/* the vendor waits forever */

/* AVI InfoFrame v2: RGB, 16:9 picture, aspect as picture */
#define AVI_TYPE		0x82
#define AVI_VERSION		0x02
#define AVI_LENGTH		0x0d
#define AVI_PB2			0x68
#define AVI_SLOT		0

/*
 * As for the DE: a gated, held or unpowered HDMI block hangs the bus. The
 * vendor U-Boot always clocks the DE before it touches HDMI; do the same.
 */
bool s900_hdmi_accessible(void)
{
	return (readl(S900_CMU_DEVCLKEN1) & DEVCLKEN1_HDMI) &&
	       (readl(S900_CMU_DEVCLKEN0) & DEVCLKEN0_DE) &&
	       (readl(S900_CMU_DEVRST1) & DEVRST1_HDMI) &&
	       (readl(S900_SPS_PG_ACK) & SPS_ACK_DE);
}

static bool hdmi_check(struct s900_display *d, u32 reg)
{
	if (d->err)
		return false;
	if (s900_hdmi_accessible())
		return true;
	log_err("s900: HDMI not clocked or held in reset, refusing access to %#x\n",
		reg);
	d->err = -EIO;
	return false;
}

static void hdmi_writel(struct s900_display *d, u32 reg, u32 val)
{
	if (hdmi_check(d, reg))
		writel(val, d->hdmi + reg);
}

static u32 hdmi_readl(struct s900_display *d, u32 reg)
{
	return hdmi_check(d, reg) ? readl(d->hdmi + reg) : 0;
}

static void hdmi_clrsetbits(struct s900_display *d, u32 reg, u32 clr, u32 set)
{
	hdmi_writel(d, reg, (hdmi_readl(d, reg) & ~clr) | set);
}

static bool hdmi_sink_present(struct s900_display *d)
{
	return (hdmi_readl(d, HDMI_CR) & HDMI_CR_HPD) &&
	       (hdmi_readl(d, CEC_DDC_HPD) & CEC_DDC_HPD_SINK);
}

/*
 * Clock the transmitter and look for a sink. Returns 1 with a sink, 0
 * without, or a negative error. HDMI_CR and CEC_DDC_HPD are returned for the
 * status line. Only the clocks, the HDMI reset, the DE power domain and
 * SPS_LDO_CTL are touched before HDMI_CR; the DE registers are not.
 */
int s900_hdmi_detect(struct s900_display *d, u32 *cr, u32 *ddc)
{
	ulong start;
	int ret;

	/* Nothing says the HDMI block sits outside the DE power domain */
	ret = s900_sps_power_on(SPS_PG_DE, SPS_ACK_DE, 100);
	if (ret)
		return ret;
	ret = clk_enable(&d->de_clk);
	if (!ret)
		ret = clk_enable(&d->hdmia_clk);
	if (ret)
		return ret;
	setbits_le32(S900_CMU_DEVCLKEN1, DEVCLKEN1_HDMI);
	if (!(readl(S900_CMU_DEVRST1) & DEVRST1_HDMI)) {
		reset_deassert(&d->hdmi_rst);
		udelay(10);
	}
	clrsetbits_le32(S900_SPS_LDO_CTL, 0xf, 0xa);

	d->cr_entry = hdmi_readl(d, HDMI_CR);
	if (!(d->cr_entry & HDMI_CR_HPD_EN))
		hdmi_clrsetbits(d, HDMI_CR, HDMI_CR_HPD_IRQ | HDMI_CR_HPD_PENDING,
				HDMI_CR_HPD_EN);

	start = get_timer(0);
	while (!hdmi_sink_present(d) && !d->err &&
	       get_timer(start) < HPD_TIMEOUT_MS)
		mdelay(5);

	*cr = hdmi_readl(d, HDMI_CR);
	*ddc = hdmi_readl(d, CEC_DDC_HPD);
	if (d->err)
		return d->err;

	return hdmi_sink_present(d);
}

/* No sink: switch hotplug detection back off if it was off on entry */
void s900_hdmi_undetect(struct s900_display *d)
{
	if (!(d->cr_entry & HDMI_CR_HPD_EN))
		hdmi_clrsetbits(d, HDMI_CR, HDMI_CR_HPD_EN | HDMI_CR_HPD_PENDING, 0);
}

u32 s900_hdmi_cr(struct s900_display *d)
{
	return hdmi_readl(d, HDMI_CR);
}

/* For a chain from a stage that left the output running */
void s900_hdmi_stop(struct s900_display *d)
{
	hdmi_clrsetbits(d, HDMI_CR, HDMI_CR_VIDEO_EN, 0);
	hdmi_clrsetbits(d, HDMI_TX_2, HDMI_TX_2_LANES, 0);
}

/* CMU_TVOUTPLL[18:16] for a pixel clock, from the vendor's table */
static int hdmi_pll_sel(u32 pixclk_khz)
{
	switch (pixclk_khz) {
	case 74250:
		return 4;
	case 148500:
		return 6;
	default:
		return -EINVAL;
	}
}

static void hdmi_pll_enable(int sel)
{
	clrbits_le32(S900_CMU_TVOUTPLL, TVOUTPLL_EN);
	setbits_le32(S900_CMU_TVOUTPLL, TVOUTPLL_HOSC);
	clrsetbits_le32(S900_CMU_TVOUTPLL, TVOUTPLL_SEL,
			FIELD_PREP(TVOUTPLL_SEL, sel));
	mdelay(1);
	/* The vendor's CMU_TVOUTPLLDEBUG0 write has its arguments swapped: skip it */
	clrsetbits_le32(S900_SPS_LDO_CTL, 0xf, 0xa);
	setbits_le32(S900_CMU_TVOUTPLL, TVOUTPLL_EN);
	mdelay(1);
}

/* TMDS PHY, lanes still off, then the S900 TMDS clock calibration */
static bool hdmi_phy_enable(struct s900_display *d, const struct s900_mode *mode)
{
	ulong start;

	hdmi_clrsetbits(d, TMDS_EODR0, 0, BIT(31));
	hdmi_writel(d, HDMI_TX_1, mode->tx1);
	hdmi_writel(d, HDMI_TX_2, mode->tx2 & ~HDMI_TX_2_LANES);
	udelay(500);

	hdmi_clrsetbits(d, CEC_DDC_HPD, CEC_DDC_HPD_CAL_START, 0);
	udelay(10);
	hdmi_clrsetbits(d, CEC_DDC_HPD, 0, CEC_DDC_HPD_CAL_START);

	start = get_timer(0);
	while (!(hdmi_readl(d, CEC_DDC_HPD) & CEC_DDC_HPD_CAL_DONE)) {
		if (d->err || get_timer(start) > CAL_TIMEOUT_MS)
			return false;
		udelay(100);
	}

	return true;
}

/*
 * The S900 timing generator counts from the start of the syncs: a line
 * starts with hsync, then the back porch, active pixels and front porch, and
 * frames likewise. Same values as the vendor U-Boot, whose vsync starts on
 * the first line.
 */
static void hdmi_video_config(struct s900_display *d,
			      const struct s900_mode *mode)
{
	u32 htotal = mode->hactive + mode->hfp + mode->hsync + mode->hbp;
	u32 vtotal = mode->vactive + mode->vfp + mode->vsync + mode->vbp;
	u32 hstart = mode->hsync + mode->hbp - 1;	/* last blank pixel */
	u32 vstart = mode->vsync + mode->vbp - 1;	/* last blank line */

	/* Syncs active high, progressive, no pixel repetition */
	hdmi_clrsetbits(d, HDMI_SCHCR, BIT(2) | BIT(1), 0);
	hdmi_writel(d, HDMI_VICTL, (htotal - 1) << 15 | (vtotal - 1));
	hdmi_writel(d, HDMI_VIVSYNC, 0);
	hdmi_writel(d, HDMI_VIVHSYNC, (mode->vsync - 1) << 24 |
		    (vtotal - 1) << 12 | (mode->hsync - 1));
	hdmi_writel(d, HDMI_VIALSEOF, (vstart + mode->vactive) << 12 | vstart);
	hdmi_writel(d, HDMI_VIALSEEF, 0);
	hdmi_writel(d, HDMI_VIADLSE, (hstart + mode->hactive) << 16 | hstart);
	hdmi_writel(d, HDMI_DIPCCR, mode->dipccr);

	/* Input from the DE */
	hdmi_clrsetbits(d, HDMI_ICR, BIT(24), 0);
	/*
	 * RGB 4:4:4 [5:4], one preline [23:20], 24 bit [17:16], HDMI rather
	 * than DVI [19:18], no 3D [8], no bit or channel inversion [29:28]
	 */
	hdmi_clrsetbits(d, HDMI_SCHCR,
			GENMASK(29, 28) | GENMASK(23, 16) | BIT(8) | GENMASK(5, 4),
			1 << 18);
	/* HDMI mode needs the HDCP keep-out windows set */
	hdmi_writel(d, HDCP_KOWR, 0x2a << 24 | 0x28b << 12 | 0x1f9);
	hdmi_writel(d, HDCP_OWR, 0x20e << 12 | 0x1fe);
	/* General control packet: 24 bit, clear AVMUTE */
	hdmi_clrsetbits(d, HDMI_GCPCR, BIT(30) | GENMASK(7, 4), BIT(31) | BIT(1));
}

/* Write the AVI InfoFrame to its packet RAM slot and send it every frame */
static void hdmi_send_avi(struct s900_display *d, u8 vic)
{
	u8 sum = AVI_TYPE + AVI_VERSION + AVI_LENGTH + AVI_PB2 + vic;
	ulong start;
	int i;

	hdmi_writel(d, HDMI_RPCR, 0);	/* stop all packets while rewriting */
	hdmi_writel(d, HDMI_OPCR, BIT(8) | (126 + AVI_SLOT * 14));
	hdmi_writel(d, HDMI_ORP6PH, AVI_LENGTH << 16 | AVI_VERSION << 8 | AVI_TYPE);
	hdmi_writel(d, HDMI_ORSP6W(0), AVI_PB2 << 16 | (u8)(0x100 - sum));
	hdmi_writel(d, HDMI_ORSP6W(1), vic);
	for (i = 2; i < 8; i++)
		hdmi_writel(d, HDMI_ORSP6W(i), 0);
	hdmi_clrsetbits(d, HDMI_OPCR, 0, HDMI_OPCR_WRITE);

	start = timer_get_us();
	while ((hdmi_readl(d, HDMI_OPCR) & HDMI_OPCR_WRITE) && !d->err &&
	       timer_get_us() - start < 100)
		udelay(1);

	/* Enable slot 0, period 1 (every frame) */
	hdmi_writel(d, HDMI_RPCR, 1 << (AVI_SLOT * 4 + 8) | BIT(AVI_SLOT));
}

/*
 * PLL, PHY, timing, infoframe, then video and TMDS lanes on. *calibrated is
 * false if the TMDS calibration did not finish, which happens when the HDMI
 * 1.8 V supply (PMIC LDO4) is off.
 */
int s900_hdmi_enable(struct s900_display *d, const struct s900_mode *mode,
		     bool *calibrated)
{
	int sel = hdmi_pll_sel(mode->pixclk_khz);

	*calibrated = false;
	if (d->err)
		return d->err;
	if (sel < 0)
		return sel;

	mdelay(10);
	hdmi_pll_enable(sel);
	mdelay(10);

	hdmi_clrsetbits(d, HDMI_CR, HDMI_CR_VIDEO_EN, 0);
	*calibrated = hdmi_phy_enable(d, mode);
	if (!*calibrated && !d->err)
		log_warning("s900: HDMI TMDS calibration timed out\n");

	hdmi_video_config(d, mode);
	hdmi_send_avi(d, mode->vic);

	hdmi_clrsetbits(d, HDMI_CR, 0, HDMI_CR_VIDEO_EN);
	hdmi_clrsetbits(d, HDMI_TX_2, 0, HDMI_TX_2_LANES);

	return d->err;
}

bool s900_hdmi_pll_locked(void)
{
	return readl(S900_CMU_TVOUTPLLDEBUG0) & TVOUTPLLDEBUG0_LOCK;
}
