/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Actions Semi S900 power gates (SPS).
 */

#ifndef __ASM_ARCH_OWL_SPS_H
#define __ASM_ARCH_OWL_SPS_H

#include <linux/types.h>

#define S900_SPS_PG_CTL		0xe012e000
#define S900_SPS_PG_ACK		0xe012e004

/*
 * Power up the domain gated by @pg and wait up to @timeout_ms for its
 * acknowledge bit @ack. Returns 0 or -ETIMEDOUT; nothing in the domain may
 * be accessed after a timeout, the bus would hang.
 */
int s900_sps_power_on(u32 pg, u32 ack, uint timeout_ms);

#endif
