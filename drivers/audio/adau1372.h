/*
 * Analog Devices ADAU1372 Audio Codec driver
 *
 * copied from:
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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_AUDIO_ADAU1372_H_ */
