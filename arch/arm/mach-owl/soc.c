// SPDX-License-Identifier: GPL-2.0+
/*
 * Actions Semi Owl SoCs platform support.
 *
 * Copyright (C) 2018 Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <config.h>
#include <cpu_func.h>
#include <init.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <linux/arm-smccc.h>
#include <linux/psci.h>
#include <asm/io.h>
#include <asm/arch-owl/sps.h>
#include <asm/mach-types.h>
#include <asm/psci.h>
#include <linux/bitops.h>
#include <wait_bit.h>

#define DMM_INTERLEAVE_PER_CH_CFG	0xe0290028

DECLARE_GLOBAL_DATA_PTR;

unsigned int owl_get_ddrcap(void)
{
	unsigned int val, cap;

	/* ddr capacity register initialized by ddr driver
	 * in early bootloader
	 */
#if defined(CONFIG_MACH_S700)
	val = (readl(DMM_INTERLEAVE_PER_CH_CFG) >> 8) & 0x7;
	cap =  (val + 1) * 256;
#elif defined(CONFIG_MACH_S900)
	val = (readl(DMM_INTERLEAVE_PER_CH_CFG) >> 8) & 0xf;
	cap =  64 * (1 << val);
#endif

	return cap;
}

/*
 * dram_init - sets uboots idea of sdram size
 */
int dram_init(void)
{
	gd->ram_size = owl_get_ddrcap() * 1024 * 1024;
	return 0;
}

/* This is called after dram_init() so use get_ram_size result */
int dram_init_banksize(void)
{
	gd->dram[0].start = CFG_SYS_SDRAM_BASE;
	gd->dram[0].size = gd->ram_size;

	return 0;
}

#define S900_CMU_DEVCLKEN0	0xe01600a0
#define S900_CMU_DEVRST0	0xe01600a8
#define S900_MFP_CTL3		0xe01b004c
#define S900_PAD_PULLCTL2	0xe01b0068
#define S900_NAND_ALG0_CTR_IO	0xe03000a8

#define SPS_PG_NAND		BIT(14)
#define SPS_PG_ACK_NAND		BIT(12)
#define DEVCLKEN0_NANDC1	BIT(11)
#define DEVCLKEN0_NANDC0	BIT(4)
#define DEVRST0_NANDC0		BIT(3)	/* active low */
#define MFP_CTL3_SD2		BIT(27)
#define PAD_PULLCTL2_SD2	(BIT(15) | BIT(14))
#define NAND_ALG_PAD_MASK	(BIT(21) | BIT(20))
#define NAND_ALG_PAD_SD		BIT(21)
#define NAND_ALG_EN_V18_R	BIT(25)
#define NAND_ALG_EN_V18_W	BIT(24)

int s900_sps_power_on(u32 pg, u32 ack, uint timeout_ms)
{
	setbits_le32(S900_SPS_PG_CTL, pg);

	return wait_for_bit_le32((void *)S900_SPS_PG_ACK, ack, true,
				 timeout_ms, false);
}

/*
 * The eMMC (SD2) runs on the NAND pads. Mux them, power the NAND domain and
 * set the pad mode as the vendor U-Boot 2015.07 does (owl_mmc_config_mfp()).
 * Linux keeps this state and never touches these registers.
 */
static void s900_emmc_pads_init(void)
{
	u32 val;

	setbits_le32(S900_MFP_CTL3, MFP_CTL3_SD2);
	setbits_le32(S900_PAD_PULLCTL2, PAD_PULLCTL2_SD2);

	clrbits_le32(S900_CMU_DEVRST0, DEVRST0_NANDC0);
	setbits_le32(S900_CMU_DEVCLKEN0, DEVCLKEN0_NANDC0 | DEVCLKEN0_NANDC1);
	if (s900_sps_power_on(SPS_PG_NAND, SPS_PG_ACK_NAND, 10))
		printf("S900: NAND power domain not acknowledged\n");
	setbits_le32(S900_CMU_DEVRST0, DEVRST0_NANDC0);

	val = readl(S900_NAND_ALG0_CTR_IO);
	val = (val & ~NAND_ALG_PAD_MASK) | NAND_ALG_PAD_SD;
	if (val & NAND_ALG_EN_V18_R)
		val |= NAND_ALG_EN_V18_W;
	else
		val &= ~NAND_ALG_EN_V18_W;
	writel(val, S900_NAND_ALG0_CTR_IO);
}

static void show_psci_version(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(ARM_PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0, 0, 0, 0, 0, &res);

	printf("PSCI:  v%ld.%ld\n",
		PSCI_VERSION_MAJOR(res.a0),
		PSCI_VERSION_MINOR(res.a0));
}

int board_init(void)
{
	show_psci_version();
	if (IS_ENABLED(CONFIG_MACH_S900))
		s900_emmc_pads_init();

	return 0;
}

void reset_cpu(void)
{
	psci_system_reset();
}
