// SPDX-License-Identifier: GPL-2.0+
/*
 * EDID for the S900 HDMI console: read it over the DDC bus that the display
 * node's ddc-i2c-bus names, and look the modes of s900_video up in it.
 *
 * On the Bubblegum-96 only one-byte DDC reads are reliable: in a longer read
 * the sink stops sending after the first byte. So the EDID is read one byte
 * per transfer (offset write, repeated start, one-byte read).
 */

#include <dm.h>
#include <i2c.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include "s900_display.h"

#define EDID_ADDR		0x50
#define EDID_BLOCK		128
#define EDID_EXTENSIONS		126	/* extension block count */
#define EDID_DTD_FIRST		54	/* four detailed timings in the base block */
#define EDID_DTD_SIZE		18
#define CEA_TAG			0x02
#define CEA_DB_VIDEO		2	/* data block of short video descriptors */
#define CEA_DB_EXTENDED	7
#define CEA_EXT_VCDB		0	/* extended tag: video capability data block */
#define VCDB_QS			BIT(6)	/* RGB quantization range selectable */
#define EDID_TRIES		3	/* per byte: the DDC here is marginal */
/*
 * A sink woken from standby can stop answering for a while in the middle of
 * the EDID: keep trying, in steps, for up to this long per block.
 */
#define EDID_BACKOFF_MS	50
#define EDID_WAIT_MS		1000

static const u8 edid_header[8] = { 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0 };

static bool edid_block_ok(const u8 *block)
{
	u8 sum = 0;
	int i;

	for (i = 0; i < EDID_BLOCK; i++)
		sum += block[i];

	return !sum;
}

static int edid_read_byte(struct udevice *chip, uint offset, u8 *val)
{
	int i, ret = -EIO;

	for (i = 0; i < EDID_TRIES; i++) {
		ret = dm_i2c_read(chip, offset, val, 1);
		if (!ret)
			break;
	}

	return ret;
}

/*
 * Read the base block and the first extension, if any: more would need the
 * E-DDC segment pointer. Returns the number of valid blocks; an extension
 * that cannot be read or has a bad checksum is zeroed.
 */
int s900_edid_read(struct udevice *dev, u8 *edid)
{
	struct udevice *bus, *chip;
	int i, len = EDID_BLOCK, ret;
	uint waited = 0;

	if (!IS_ENABLED(CONFIG_DM_I2C))
		return -ENOSYS;

	memset(edid, 0, S900_EDID_SIZE);
	ret = uclass_get_device_by_phandle(UCLASS_I2C, dev, "ddc-i2c-bus", &bus);
	if (!ret)
		ret = i2c_get_chip(bus, EDID_ADDR, 1, &chip);
	if (ret)
		return ret;

	for (i = 0; i < len; i++) {
		if (i == EDID_BLOCK)
			waited = 0;	/* per block */
		ret = edid_read_byte(chip, i, &edid[i]);
		if (ret && waited < EDID_WAIT_MS) {
			mdelay(EDID_BACKOFF_MS);
			waited += EDID_BACKOFF_MS;
			i--;	/* the same byte again */
			continue;
		}
		if (ret && i < EDID_BLOCK)
			return ret;
		if (ret)
			break;	/* keep the base block */
		if (i != EDID_BLOCK - 1)
			continue;
		if (memcmp(edid, edid_header, sizeof(edid_header)) ||
		    !edid_block_ok(edid))
			return -EBADMSG;
		if (edid[EDID_EXTENSIONS])
			len = S900_EDID_SIZE;
	}

	if (len == EDID_BLOCK)
		return 1;
	if (!ret && edid_block_ok(edid + EDID_BLOCK))
		return 2;
	memset(edid + EDID_BLOCK, 0, EDID_BLOCK);

	return 1;
}

/* A detailed timing descriptor with exactly the timing of the mode */
static bool edid_dtd_is(const u8 *t, const struct s900_mode *m)
{
	u32 clk_khz = (t[0] | t[1] << 8) * 10;
	u16 hactive = t[2] | (t[4] & 0xf0) << 4;
	u16 hblank = t[3] | (t[4] & 0x0f) << 8;
	u16 vactive = t[5] | (t[7] & 0xf0) << 4;
	u16 vblank = t[6] | (t[7] & 0x0f) << 8;
	u16 hfp = t[8] | (t[11] & 0xc0) << 2;
	u16 hsync = t[9] | (t[11] & 0x30) << 4;
	u16 vfp = t[10] >> 4 | (t[11] & 0x0c) << 2;
	u16 vsync = (t[10] & 0x0f) | (t[11] & 0x03) << 4;

	return clk_khz == m->pixclk_khz && !(t[17] & BIT(7)) &&
	       hactive == m->hactive && hblank == m->hfp + m->hsync + m->hbp &&
	       hfp == m->hfp && hsync == m->hsync &&
	       vactive == m->vactive && vblank == m->vfp + m->vsync + m->vbp &&
	       vfp == m->vfp && vsync == m->vsync;
}

/* The VIC in a short video descriptor, or a detailed timing, of a CEA block */
static bool edid_cea_has(const u8 *b, const struct s900_mode *m)
{
	int dtd = b[2], i, j, len = 0;
	u8 vic;

	if (b[0] != CEA_TAG || dtd < 4 || dtd >= EDID_BLOCK)
		return false;

	for (i = 4; b[1] >= 3 && i < dtd; i += len + 1) {
		len = b[i] & 0x1f;
		if (b[i] >> 5 != CEA_DB_VIDEO)
			continue;
		for (j = i + 1; j <= i + len && j < dtd; j++) {
			/* 129..192: VIC 1..64 with the native flag */
			vic = b[j] >= 129 && b[j] <= 192 ? b[j] & 0x7f : b[j];
			if (vic == m->vic)
				return true;
		}
	}

	for (i = dtd; i + EDID_DTD_SIZE < EDID_BLOCK; i += EDID_DTD_SIZE)
		if (edid_dtd_is(b + i, m))
			return true;

	return false;
}

bool s900_edid_has_mode(const u8 *edid, const struct s900_mode *mode)
{
	int i;

	for (i = EDID_DTD_FIRST; i < EDID_EXTENSIONS; i += EDID_DTD_SIZE)
		if (edid_dtd_is(edid + i, mode))
			return true;

	return edid_cea_has(edid + EDID_BLOCK, mode);
}

/* The sink takes the RGB quantization range from the AVI InfoFrame (VCDB QS) */
bool s900_edid_rgb_selectable(const u8 *edid)
{
	const u8 *b = edid + EDID_BLOCK;
	int dtd = b[2], i, len = 0;

	if (b[0] != CEA_TAG || b[1] < 3 || dtd < 4 || dtd >= EDID_BLOCK)
		return false;

	for (i = 4; i < dtd; i += len + 1) {
		len = b[i] & 0x1f;
		if (b[i] >> 5 == CEA_DB_EXTENDED && len >= 2 && i + len < dtd &&
		    b[i + 1] == CEA_EXT_VCDB)
			return b[i + 2] & VCDB_QS;
	}

	return false;
}
