/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Actions Semi S900 HDMI console handoff to the OS.
 */

#ifndef __ASM_ARCH_OWL_VIDEO_H
#define __ASM_ARCH_OWL_VIDEO_H

/* Fill in the OS DT's simple-framebuffer node if U-Boot lit HDMI */
void s900_video_ft_fixup(void *blob);

#endif
