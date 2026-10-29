/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Actions Semi S900 display engine (DE) and HDMI transmitter, HDMI output
 * only. The sequence and the values follow the vendor U-Boot 2015.07
 * (drivers/video/owl/dss) and what it leaves programmed on a Bubblegum-96.
 */

#ifndef __S900_DISPLAY_H
#define __S900_DISPLAY_H

#include <clk.h>
#include <reset.h>
#include <linux/bitops.h>
#include <linux/types.h>

struct udevice;

/* CMU and SPS registers: always accessible */
#define S900_CMU_TVOUTPLL	0xe0160018
#define S900_CMU_DECLK		0xe0160030
#define S900_CMU_DEVCLKEN0	0xe01600a0
#define S900_CMU_DEVCLKEN1	0xe01600a4
#define S900_CMU_DEVRST0	0xe01600a8
#define S900_CMU_DEVRST1	0xe01600ac
#define S900_CMU_TVOUTPLLDEBUG0	0xe01600ec	/* read only: bit 14 probably PLL lock */
#define S900_SPS_LDO_CTL	0xe012e014

#define DEVCLKEN0_DE		BIT(8)
#define DEVCLKEN0_HDMIA		BIT(22)
#define DEVCLKEN0_SHARESRAM	BIT(28)
#define DEVCLKEN1_HDMI		BIT(3)
#define DEVRST0_DE		BIT(7)	/* active low */
#define DEVRST1_HDMI		BIT(2)	/* active low */
#define SPS_PG_DE		BIT(13)
#define SPS_ACK_DE		BIT(11)

/*
 * The vendor U-Boot's framebuffer, 1920x1080 XRGB8888, which the DE scales
 * to the output mode (1:1 at 1080p). Linux takes it over as a
 * simple-framebuffer.
 */
#define S900_FB_BASE		0x19000000
#define S900_FB_WIDTH		1920
#define S900_FB_HEIGHT		1080
#define S900_FB_STRIDE		(S900_FB_WIDTH * 4)
#define S900_FB_SIZE		(S900_FB_STRIDE * S900_FB_HEIGHT)
#define S900_FB_RESERVED	0x800000	/* no-map in both device trees */

/*
 * A progressive CEA-861 mode with both syncs active high. The TMDS PHY
 * settings and HDMI_DIPCCR are the vendor U-Boot's for its pixel clock.
 */
struct s900_mode {
	const char *name;
	u8 vic;
	u32 pixclk_khz;
	u16 hactive, hfp, hsync, hbp;
	u16 vactive, vfp, vsync, vbp;
	u32 tx1;	/* HDMI_TX_1 */
	u32 tx2;	/* HDMI_TX_2, TMDS lanes on */
	u32 dipccr;	/* HDMI_DIPCCR */
};

#define S900_EDID_SIZE		256	/* base block and one extension */

int s900_edid_read(struct udevice *dev, u8 *edid);
bool s900_edid_has_mode(const u8 *edid, const struct s900_mode *mode);
bool s900_edid_rgb_selectable(const u8 *edid);

struct s900_display {
	void __iomem *de;
	void __iomem *hdmi;
	struct clk de_clk;
	struct clk hdmia_clk;
	struct reset_ctl de_rst;
	struct reset_ctl hdmi_rst;
	u32 cr_entry;	/* HDMI_CR as found */
	int err;	/* first refused access, see s900_*_accessible() */
};

bool s900_de_accessible(void);
int s900_de_power_on(struct s900_display *d);
void s900_de_init(struct s900_display *d);
void s900_de_setup(struct s900_display *d, const struct s900_mode *mode);
bool s900_de_go(struct s900_display *d);

bool s900_hdmi_accessible(void);
int s900_hdmi_detect(struct s900_display *d, u32 *cr, u32 *ddc);
void s900_hdmi_undetect(struct s900_display *d);
u32 s900_hdmi_cr(struct s900_display *d);
void s900_hdmi_stop(struct s900_display *d);
int s900_hdmi_enable(struct s900_display *d, const struct s900_mode *mode,
		     bool full_range, bool *calibrated);
bool s900_hdmi_pll_locked(void);

#endif
