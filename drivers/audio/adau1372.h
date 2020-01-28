/*
 * Analog Devices ADAU1372 Audio Codec driver
 *
 * largely copied from:
 *   https://github.com/analogdevicesinc/linux/blob/asoc-adau1372/sound/soc/codecs/adau1372.c
 *
 * Which has this copyright and license:
 *  Copyright 2016 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *  Licensed under the GPL-2
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_ADAU1372_H_
#define ZEPHYR_DRIVERS_AUDIO_ADAU1372_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Copied from the linux driver */
#define ADAU1372_REG_CLK_CTRL		0x00
#define ADAU1372_REG_PLL(x)		(0x01 + (x))
#define ADAU1372_REG_DAC_SOURCE		0x11
#define ADAU1372_REG_SOUT_SOURCE_0_1	0x13
#define ADAU1372_REG_SOUT_SOURCE_2_3	0x14
#define ADAU1372_REG_SOUT_SOURCE_4_5	0x15
#define ADAU1372_REG_SOUT_SOURCE_6_7	0x16
#define ADAU1372_REG_ADC_SDATA_CH	0x17
#define ADAU1372_REG_ASRCO_SOURCE_0_1	0x18
#define ADAU1372_REG_ASRCO_SOURCE_2_3	0x19
#define ADAU1372_REG_ASRC_MODE		0x1a
#define ADAU1372_REG_ADC_CTRL0		0x1b
#define ADAU1372_REG_ADC_CTRL1		0x1c
#define ADAU1372_REG_ADC_CTRL2		0x1d
#define ADAU1372_REG_ADC_CTRL3		0x1e
#define ADAU1372_REG_ADC_VOL(x)		(0x1f + (x))
#define ADAU1372_REG_PGA_CTRL(x)	(0x23 + (x))
#define ADAU1372_REG_PGA_BOOST		0x28
#define ADAU1372_REG_MICBIAS		0x2d
#define ADAU1372_REG_DAC_CTRL		0x2e
#define ADAU1372_REG_DAC_VOL(x)		(0x2f + (x))
#define ADAU1372_REG_OP_STAGE_MUTE	0x31
#define ADAU1372_REG_SAI0		0x32
#define ADAU1372_REG_SAI1		0x33
#define ADAU1372_REG_SOUT_CTRL		0x34
#define ADAU1372_REG_MODE_MP(x)		(0x38 + (x))
#define ADAU1372_REG_OP_STAGE_CTRL	0x43
#define ADAU1372_REG_DECIM_PWR		0x44
#define ADAU1372_REG_INTERP_PWR		0x45
#define ADAU1372_REG_BIAS_CTRL0		0x46
#define ADAU1372_REG_BIAS_CTRL1		0x47

#define ADAU1372_CLK_CTRL_PLL_EN	BIT(7)
#define ADAU1372_CLK_CTRL_XTAL_DIS	BIT(4)
#define ADAU1372_CLK_CTRL_CLKSRC	BIT(3)
#define ADAU1372_CLK_CTRL_CC_MDIV	BIT(1)
#define ADAU1372_CLK_CTRL_MCLK_EN	BIT(0)

#define ADAU1372_SAI0_DELAY1		(0x0 << 6)
#define ADAU1372_SAI0_DELAY0		(0x1 << 6)
#define ADAU1372_SAI0_DELAY_MASK	(0x3 << 6)
#define ADAU1372_SAI0_SAI_I2S		(0x0 << 4)
#define ADAU1372_SAI0_SAI_TDM2		(0x1 << 4)
#define ADAU1372_SAI0_SAI_TDM4		(0x2 << 4)
#define ADAU1372_SAI0_SAI_TDM8		(0x3 << 4)
#define ADAU1372_SAI0_SAI_MASK		(0x3 << 4)
#define ADAU1372_SAI0_FS_48		0x0
#define ADAU1372_SAI0_FS_8		0x1
#define ADAU1372_SAI0_FS_12		0x2
#define ADAU1372_SAI0_FS_16		0x3
#define ADAU1372_SAI0_FS_24		0x4
#define ADAU1372_SAI0_FS_32		0x5
#define ADAU1372_SAI0_FS_96		0x6
#define ADAU1372_SAI0_FS_192		0x7
#define ADAU1372_SAI0_FS_MASK		0xf

#define ADAU1372_SAI1_TDM_TS		BIT(7)
#define ADAU1372_SAI1_BCLK_TDMC		BIT(6)
#define ADAU1372_SAI1_LR_MODE		BIT(5)
#define ADAU1372_SAI1_LR_POL		BIT(4)
#define ADAU1372_SAI1_BCLKRATE		BIT(2)
#define ADAU1372_SAI1_BCLKEDGE		BIT(1)
#define ADAU1372_SAI1_MS		BIT(0)

  
/* everything from here on is from tlv320, so it's all bogus i think */
/* Register addresses */
#define PAGE_CONTROL_ADDR	0

/* Register addresses {page, address} and fields */
#define SOFT_RESET_ADDR		(struct reg_addr){0, 1}
#define SOFT_RESET_ASSERT	(1)

#define NDAC_DIV_ADDR		(struct reg_addr){0, 11}
#define NDAC_POWER_UP		BIT(7)
#define NDAC_POWER_UP_MASK	BIT(7)
#define NDAC_DIV_MASK		BIT_MASK(7)
#define NDAC_DIV(val)		((val) & NDAC_DIV_MASK)

#define MDAC_DIV_ADDR		(struct reg_addr){0, 12}
#define MDAC_POWER_UP		BIT(7)
#define MDAC_POWER_UP_MASK	BIT(7)
#define MDAC_DIV_MASK		BIT_MASK(7)
#define MDAC_DIV(val)		((val) & MDAC_DIV_MASK)

#define DAC_PROC_CLK_FREQ_MAX	49152000	/* 49.152 MHz */

#define OSR_MSB_ADDR		(struct reg_addr){0, 13}
#define OSR_MSB_MASK		BIT_MASK(2)

#define OSR_LSB_ADDR		(struct reg_addr){0, 14}
#define OSR_LSB_MASK		BIT_MASK(8)

#define DAC_MOD_CLK_FREQ_MIN	2800000	/* 2.8 MHz */
#define DAC_MOD_CLK_FREQ_MAX	6200000 /* 6.2 MHz */

#define IF_CTRL1_ADDR		(struct reg_addr){0, 27}
#define IF_CTRL_IFTYPE_MASK	BIT_MASK(2)
#define IF_CTRL_IFTYPE_I2S	0
#define IF_CTRL_IFTYPE_DSP	1
#define IF_CTRL_IFTYPE_RJF	2
#define IF_CTRL_IFTYPE_LJF	3
#define IF_CTRL_IFTYPE(val)	(((val) & IF_CTRL_IFTYPE_MASK) << 6)
#define IF_CTRL_WLEN_MASK	BIT_MASK(2)
#define IF_CTRL_WLEN(val)	(((val) & IF_CTRL_WLEN_MASK) << 4)
#define IF_CTRL_WLEN_16		0
#define IF_CTRL_WLEN_20		1
#define IF_CTRL_WLEN_24		2
#define IF_CTRL_WLEN_32		3
#define IF_CTRL_BCLK_OUT	BIT(3)
#define IF_CTRL_WCLK_OUT	BIT(2)

#define BCLK_DIV_ADDR		(struct reg_addr){0, 30}
#define BCLK_DIV_POWER_UP	BIT(7)
#define BCLK_DIV_POWER_UP_MASK	BIT(7)
#define BCLK_DIV_MASK		BIT_MASK(7)
#define BCLK_DIV(val)		((val) & MDAC_DIV_MASK)

#define OVF_FLAG_ADDR		(struct reg_addr){0, 39}

#define PROC_BLK_SEL_ADDR	(struct reg_addr){0, 60}
#define	PROC_BLK_SEL_MASK	BIT_MASK(5)
#define	PROC_BLK_SEL(val)	((val) & PROC_BLK_SEL_MASK)

#define DATA_PATH_SETUP_ADDR	(struct reg_addr){0, 63}
#define DAC_LR_POWERUP_DEFAULT	(BIT(7) | BIT(6) | BIT(4) | BIT(2))
#define DAC_LR_POWERDN_DEFAULT	(BIT(4) | BIT(2))

#define VOL_CTRL_ADDR		(struct reg_addr){0, 64}
#define VOL_CTRL_UNMUTE_DEFAULT	(0)
#define VOL_CTRL_MUTE_DEFAULT	(BIT(3) | BIT(2))

#define L_DIG_VOL_CTRL_ADDR	(struct reg_addr){0, 65}
#define DRC_CTRL1_ADDR		(struct reg_addr){0, 68}
#define L_BEEP_GEN_ADDR		(struct reg_addr){0, 71}
#define BEEP_GEN_EN_BEEP	(BIT(7))
#define R_BEEP_GEN_ADDR		(struct reg_addr){0, 72}
#define BEEP_LEN_MSB_ADDR	(struct reg_addr){0, 73}
#define BEEP_LEN_MIB_ADDR	(struct reg_addr){0, 74}
#define BEEP_LEN_LSB_ADDR	(struct reg_addr){0, 75}

/* Page 1 registers */
#define HEADPHONE_DRV_ADDR	(struct reg_addr){1, 31}
#define HEADPHONE_DRV_POWERUP	(BIT(7) | BIT(6))
#define HEADPHONE_DRV_CM_MASK	(BIT_MASK(2) << 3)
#define HEADPHONE_DRV_CM(val)	(((val) << 3) & HEADPHONE_DRV_CM_MASK)
#define HEADPHONE_DRV_RESERVED	(BIT(2))

#define HP_OUT_POP_RM_ADDR	(struct reg_addr){1, 33}
#define HP_OUT_POP_RM_ENABLE	(BIT(7))

#define OUTPUT_ROUTING_ADDR	(struct reg_addr){1, 35}
#define OUTPUT_ROUTING_HPL	(2 << 6)
#define OUTPUT_ROUTING_HPR	(2 << 2)

#define HPL_ANA_VOL_CTRL_ADDR	(struct reg_addr){1, 36}
#define HPR_ANA_VOL_CTRL_ADDR	(struct reg_addr){1, 37}
#define HPX_ANA_VOL_ENABLE	(BIT(7))
#define HPX_ANA_VOL_MASK	(BIT_MASK(7))
#define HPX_ANA_VOL(val)	(((val) & HPX_ANA_VOL_MASK) |	\
		HPX_ANA_VOL_ENABLE)
#define HPX_ANA_VOL_MAX		(0)
#define HPX_ANA_VOL_DEFAULT	(64)
#define HPX_ANA_VOL_MIN		(127)
#define HPX_ANA_VOL_MUTE	(HPX_ANA_VOL_MIN | ~HPX_ANA_VOL_ENABLE)
#define HPX_ANA_VOL_LOW_THRESH	(105)
#define HPX_ANA_VOL_FLOOR	(144)

#define HPL_DRV_GAIN_CTRL_ADDR	(struct reg_addr){1, 40}
#define HPR_DRV_GAIN_CTRL_ADDR	(struct reg_addr){1, 41}
#define	HPX_DRV_UNMUTE		(BIT(2))

#define HEADPHONE_DRV_CTRL_ADDR	(struct reg_addr){1, 44}
#define HEADPHONE_DRV_LINEOUT	(BIT(1) | BIT(2))

/* Page 3 registers */
#define TIMER_MCLK_DIV_ADDR	(struct reg_addr){3, 16}
#define TIMER_MCLK_DIV_EN_EXT	(BIT(7))
#define TIMER_MCLK_DIV_MASK	(BIT_MASK(7))
#define TIMER_MCLK_DIV_VAL(val)	((val) & TIMER_MCLK_DIV_MASK)

struct reg_addr {
	u8_t page; 		/* page number */
	u8_t reg_addr; 		/* register address */
};

enum proc_block {
	/* highest performance class with each decimation filter */
	PRB_P25_DECIMATION_A = 25,
	PRB_P10_DECIMATION_B = 10,
	PRB_P18_DECIMATION_C = 18,
};

enum osr_multiple {
	OSR_MULTIPLE_8 = 8,
	OSR_MULTIPLE_4 = 4,
	OSR_MULTIPLE_2 = 2,
};

enum cm_voltage {
	CM_VOLTAGE_1P35 = 0,
	CM_VOLTAGE_1P5 = 1,
	CM_VOLTAGE_1P65 = 2,
	CM_VOLTAGE_1P8 = 3,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_AUDIO_ADAU1372_H_ */
