// SPDX-License-Identifier: GPL-2.0+
/*
 * Actions Semi S900 HDMI console: the display engine scans a fixed
 * 1920x1080 XRGB8888 framebuffer at 0x19000000 out to the HDMI transmitter
 * at a 1080p or 720p CEA mode, as the vendor U-Boot 2015.07 does. Linux
 * takes the framebuffer over as a simple-framebuffer (s900_video_ft_fixup()).
 *
 * Environment:
 *   bg96_hdmi       auto (default): light the output only if a sink is seen
 *                   force: light it without a sink (plug in later)
 *                   off: leave the display hardware untouched
 *   bg96_hdmi_mode  auto (default): the first of 1080p60, 1080p50, 720p60
 *                   and 720p50 that the sink's EDID lists, else 720p50;
 *                   or one of those names
 */

#include <cpu_func.h>
#include <dm.h>
#include <env.h>
#include <fdt_support.h>
#include <fdtdec.h>
#include <log.h>
#include <vsprintf.h>
#include <video.h>
#include <asm/io.h>
#include <asm/arch-owl/video.h>
#include <dm/uclass-internal.h>
#include <linux/errno.h>
#include <linux/libfdt.h>
#include <linux/string.h>
#include "s900_display.h"

/*
 * CEA-861 timings, best first. The TMDS PHY values (HDMI_TX_1/2) and
 * HDMI_DIPCCR are the vendor U-Boot's for 74.25 and 148.5 MHz.
 */
static const struct s900_mode s900_modes[] = {
	/* name      vic  kHz     hact  hfp  hs   hbp  vact vfp vs  vbp */
	{ "1080p60", 16, 148500, 1920, 88,  44,  148, 1080, 4,  5,  36,
	  0x80842846, 0x00000fc1, 0x1105 },
	{ "1080p50", 31, 148500, 1920, 528, 44,  148, 1080, 4,  5,  36,
	  0x80842846, 0x00000fc1, 0x1107 },
	{ "720p60",  4,  74250,  1280, 110, 40,  220, 720,  5,  5,  20,
	  0x80882904, 0x00f00fc1, 0x1107 },
	{ "720p50",  19, 74250,  1280, 440, 40,  220, 720,  5,  5,  20,
	  0x80882904, 0x00f00fc1, 0x1107 },
};

/* The vendor U-Boot's mode, for a sink without a usable EDID */
#define S900_MODE_FALLBACK	(&s900_modes[ARRAY_SIZE(s900_modes) - 1])

/*
 * A failed probe is retried for stdout and for stderr: keep the verdict so
 * the hardware is only touched once. The status line goes to the Linux DT.
 */
static bool s900_tried;
static int s900_result;
static char s900_status[128] = "not-probed";

/*
 * A mode named in the environment, else the first one the EDID lists.
 * *full_range: the EDID says the sink takes Q=full in the AVI InfoFrame.
 */
static const struct s900_mode *s900_pick_mode(struct udevice *dev, bool *full_range,
					      char *why, size_t len)
{
	const char *name = env_get("bg96_hdmi_mode");
	u8 edid[S900_EDID_SIZE];
	int i, blocks;

	*full_range = false;
	for (i = 0; name && i < ARRAY_SIZE(s900_modes); i++) {
		if (!strcmp(name, s900_modes[i].name)) {
			snprintf(why, len, "env");
			return &s900_modes[i];
		}
	}

	blocks = s900_edid_read(dev, edid);
	if (blocks < 0) {
		snprintf(why, len, "edid-error%d", blocks);
		return S900_MODE_FALLBACK;
	}
	*full_range = s900_edid_rgb_selectable(edid);

	for (i = 0; i < ARRAY_SIZE(s900_modes); i++) {
		if (s900_edid_has_mode(edid, &s900_modes[i])) {
			snprintf(why, len, "edid%d", blocks);
			return &s900_modes[i];
		}
	}
	snprintf(why, len, "edid%d-none", blocks);

	return S900_MODE_FALLBACK;
}

static int s900_get_resources(struct udevice *dev, struct s900_display *d)
{
	int ret;

	d->de = dev_remap_addr_name(dev, "de");
	d->hdmi = dev_remap_addr_name(dev, "hdmi");
	if (!d->de || !d->hdmi)
		return -EINVAL;

	ret = clk_get_by_name(dev, "de", &d->de_clk);
	if (!ret)
		ret = clk_get_by_name(dev, "hdmia", &d->hdmia_clk);
	if (!ret)
		ret = reset_get_by_name(dev, "de", &d->de_rst);
	if (!ret)
		ret = reset_get_by_name(dev, "hdmi", &d->hdmi_rst);

	return ret;
}

/*
 * Undo s900_hdmi_detect() when there is no sink and leave the CMU as found:
 * the reset first, while the HDMI block is still clocked.
 */
static void s900_restore_gates(struct s900_display *d, u32 en0, u32 en1, u32 rst1)
{
	const u32 en0_mask = DEVCLKEN0_DE | DEVCLKEN0_HDMIA;

	s900_hdmi_undetect(d);
	clrsetbits_le32(S900_CMU_DEVRST1, DEVRST1_HDMI, rst1 & DEVRST1_HDMI);
	clrsetbits_le32(S900_CMU_DEVCLKEN1, DEVCLKEN1_HDMI, en1 & DEVCLKEN1_HDMI);
	clrsetbits_le32(S900_CMU_DEVCLKEN0, en0_mask, en0 & en0_mask);
}

static int s900_light(struct udevice *dev, struct s900_display *d)
{
	const char *policy = env_get("bg96_hdmi") ?: "auto";
	const struct s900_mode *mode;
	u32 en0 = readl(S900_CMU_DEVCLKEN0);
	u32 en1 = readl(S900_CMU_DEVCLKEN1);
	u32 rst1 = readl(S900_CMU_DEVRST1);
	bool calibrated, full_range, go;
	u32 cr = 0, ddc = 0;
	char why[24];
	int sink, ret;

	log_debug("s900: entry DEVCLKEN0 %#x DEVCLKEN1 %#x DEVRST1 %#x\n",
		  en0, en1, rst1);
	if (!strcmp(policy, "off")) {
		snprintf(s900_status, sizeof(s900_status), "off");
		return -ENODEV;
	}

	ret = s900_get_resources(dev, d);
	if (ret) {
		snprintf(s900_status, sizeof(s900_status), "error resources %d", ret);
		return ret;
	}

	sink = s900_hdmi_detect(d, &cr, &ddc);
	if (sink < 0) {
		snprintf(s900_status, sizeof(s900_status), "error detect %d", sink);
		return sink;
	}
	if (!sink && strcmp(policy, "force")) {
		s900_restore_gates(d, en0, en1, rst1);
		snprintf(s900_status, sizeof(s900_status), "nosink cr=%#x ddc=%#x",
			 cr, ddc);
		log_debug("s900: no HDMI sink\n");
		return -ENODEV;
	}
	mode = s900_pick_mode(dev, &full_range, why, sizeof(why));

	ret = s900_de_power_on(d);
	if (ret) {
		snprintf(s900_status, sizeof(s900_status), "error de-power %d", ret);
		return ret;
	}
	s900_de_init(d);
	if (d->err) {
		snprintf(s900_status, sizeof(s900_status), "error access %d", d->err);
		return d->err;
	}

	/* Never scan out stale DRAM */
	memset((void *)S900_FB_BASE, 0, S900_FB_SIZE);
	flush_dcache_range(S900_FB_BASE, S900_FB_BASE + S900_FB_SIZE);

	s900_hdmi_stop(d);
	s900_de_setup(d, mode);
	ret = s900_hdmi_enable(d, mode, full_range, &calibrated);
	go = !ret && s900_de_go(d);
	if (d->err) {
		snprintf(s900_status, sizeof(s900_status), "error access %d", d->err);
		return d->err;
	}

	snprintf(s900_status, sizeof(s900_status),
		 "ok mode=%s/%s q=%s hpd=%d calib=%s pll=%s go=%s cr=%#x ddc=%#x",
		 mode->name, why, full_range ? "full" : "default", sink,
		 calibrated ? "ok" : "timeout",
		 s900_hdmi_pll_locked() ? "lock" : "nolock", go ? "ok" : "stuck",
		 s900_hdmi_cr(d), ddc);

	return 0;
}

static int s900_video_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *uc = dev_get_uclass_priv(dev);

	if (!s900_tried) {
		s900_tried = true;
		s900_result = s900_light(dev, dev_get_priv(dev));
		printf("HDMI:  %s\n", s900_status);
	}
	if (s900_result)
		return s900_result;

	plat->base = S900_FB_BASE;
	plat->size = S900_FB_SIZE;
	uc->xsize = S900_FB_WIDTH;
	uc->ysize = S900_FB_HEIGHT;
	uc->line_length = S900_FB_STRIDE;
	uc->bpix = VIDEO_BPP32;
	uc->format = VIDEO_X8R8G8B8;
	video_set_flush_dcache(dev, true);

	return 0;
}

/*
 * Called from ft_board_setup(): record the status, then hand a lit
 * framebuffer to Linux. The Linux DT carries a disabled simple-framebuffer
 * node under a /chosen with two address and two size cells, holding the DE
 * and HDMI clocks. Never fails the boot.
 */
void s900_video_ft_fixup(void *blob)
{
	struct fdt_memory fb = {
		.start = S900_FB_BASE,
		.end = S900_FB_BASE + S900_FB_RESERVED - 1,
	};
	struct udevice *dev;
	int chosen, node, parent;

	chosen = fdt_find_or_add_subnode(blob, 0, "chosen");
	if (chosen >= 0)
		fdt_setprop_string(blob, chosen, "u-boot,video-status", s900_status);

	node = fdt_node_offset_by_compatible(blob, -1, "simple-framebuffer");
	if (node < 0)
		return;

	uclass_find_first_device(UCLASS_VIDEO, &dev);
	if (!dev || !device_active(dev))
		goto disable;

	/* Linux translates reg through the parent: 2+2 cells and ranges */
	parent = fdt_parent_offset(blob, node);
	if (fdt_address_cells(blob, parent) != 2 || fdt_size_cells(blob, parent) != 2 ||
	    !fdt_getprop(blob, parent, "ranges", NULL)) {
		log_err("s900: the DTB's simple-framebuffer parent needs 2+2 cells and ranges\n");
		goto disable;
	}

	/* Without the no-map reservation Linux cannot map the framebuffer */
	if (fdtdec_add_reserved_memory(blob, "framebuffer", &fb, NULL, 0, NULL,
				       FDTDEC_RESERVED_MEMORY_NO_MAP))
		goto disable;

	node = fdt_node_offset_by_compatible(blob, -1, "simple-framebuffer");
	if (node >= 0 &&
	    !fdt_setup_simplefb_node(blob, node, S900_FB_BASE, S900_FB_WIDTH,
				     S900_FB_HEIGHT, S900_FB_STRIDE, "x8r8g8b8"))
		return;

disable:
	node = fdt_node_offset_by_compatible(blob, -1, "simple-framebuffer");
	if (node >= 0)
		fdt_setprop_string(blob, node, "status", "disabled");
}

static const struct udevice_id s900_video_ids[] = {
	{ .compatible = "actions,s900-display" },
	{ }
};

U_BOOT_DRIVER(s900_video) = {
	.name		= "s900_video",
	.id		= UCLASS_VIDEO,
	.of_match	= s900_video_ids,
	.probe		= s900_video_probe,
	.priv_auto	= sizeof(struct s900_display),
};
