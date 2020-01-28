/*
 * Copyright (c) 2020 Nik Langrind
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <sys/util.h>

#include <device.h>
#include <drivers/i2c.h>

#include <audio/codec.h>
#include "adau1372.h"

#define LOG_LEVEL CONFIG_AUDIO_CODEC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adau1372);

#define CODEC_OUTPUT_VOLUME_MAX		0
#define CODEC_OUTPUT_VOLUME_MIN		(-78 * 2)

#define CODEC_RESET_PIN_ASSERT		0
#define CODEC_RESET_PIN_DEASSERT	1

/*
 * According to the Zephyr Driver Model, struct device is runtime mutable
 * and struct device_config is read-only. We are supposed to split our
 * instance data accordingly, but I haven't really done that because
 * this was assembled by cutting and pasting. So this is an area for
 * clean up.
 */

struct adau1372 {
  struct device   *i2c_device;
  const char      *i2c_dev_name;
  uint8_t          i2c_address;
  bool             use_pll;    // from linux driver
  bool             enabled;    // from linux driver
  unsigned int     slot_width; // from linux driver
};

struct adau1372_driver_data {
  int unused;
};
static struct adau1372 device_config = {
  .i2c_device	= NULL,
  .i2c_dev_name	= DT_INST_0_ADI_ADAU1372_BUS_NAME,
  .i2c_address	= DT_INST_0_ADI_ADAU1372_BASE_ADDRESS,
#if 0
  .i2c_dev_name	= "I2C_3", /* want DT_INST_0_ADA1372_BUS_NAME, */
  .i2c_address	= 0x3c,    /* want DT_INST_0_ADA1372_BASE_ADDRESS, */
#endif
};

static struct adau1372_driver_data driver_data = {
  .unused = 0
};

#define DEV_CFG(dev)  ((struct adau1372 *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct adau1372_driver_data *const)(dev)->driver_data)


static void codec_soft_reset(struct device *dev);
static int codec_configure_dai(struct device *dev, audio_dai_cfg_t *cfg);
static int codec_configure_clocks(struct device *dev,
		struct audio_codec_cfg *cfg);
static int codec_configure_filters(struct device *dev, audio_dai_cfg_t *cfg);
static enum osr_multiple codec_get_osr_multiple(audio_dai_cfg_t *cfg);
static void codec_configure_output(struct device *dev);
static int codec_set_output_volume(struct device *dev, int vol);

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void codec_read_all_regs(struct device *dev);
#define CODEC_DUMP_REGS(dev)	codec_read_all_regs((dev))
#else
#define CODEC_DUMP_REGS(dev)
#endif

/* Perform the two byte address write, and N-byte read */
static int adau1372_i2c_read(struct adau1372 *const dev_cfg, uint16_t startAddr, uint8_t *buf, uint32_t numBytes)
{
  uint8_t addrBuffer[2];

  addrBuffer[1] = startAddr & 0xFF;
  addrBuffer[0] = startAddr >> 8;
  return i2c_write_read(dev_cfg->i2c_device, dev_cfg->i2c_address, addrBuffer, sizeof(addrBuffer), buf, numBytes);
}

static int adau1372_i2c_write(struct adau1372 *const dev_cfg, uint16_t startAddr, uint8_t *buf, uint32_t numDataBytes)
{
  size_t   bufSize = sizeof startAddr + numDataBytes;
  uint8_t *txBuf = alloca(bufSize);

  txBuf[0] = startAddr >> 8;
  txBuf[1] = startAddr & 0xFF;
  memcpy(&txBuf[sizeof startAddr], buf, numDataBytes);

  return i2c_write(dev_cfg->i2c_device, txBuf, bufSize, dev_cfg->i2c_address);
}

/* wrapper for single-byte operation */
static int adau1372_read_reg(struct adau1372 *const dev_cfg, uint16_t regAddr, uint8_t *val)
{
  return adau1372_i2c_read(dev_cfg, regAddr, val, 1);
}

/* wrapper for single-byte operation */
static int adau1372_write_reg(struct adau1372 *const dev_cfg, uint16_t regAddr, uint8_t val)
{
  return adau1372_i2c_write(dev_cfg, regAddr, &val, 1);
}

/* to make it slightly easier to port routines from the linux driver: */
static int regmap_update_bits(struct adau1372 *const dev_cfg, uint16_t regAddr, uint8_t mask, uint8_t val)
{
  uint8_t currVal;
  int rc = adau1372_read_reg(dev_cfg, regAddr, &currVal);
  if (rc) {
    return rc;
  }

  currVal &= ~mask;
  val &= mask;
  currVal |= val;

  return adau1372_write_reg(dev_cfg, regAddr, currVal);
}

static int adau1372_initialize(struct device *dev)
{
  struct adau1372 *const dev_cfg = DEV_CFG(dev);

  /* bind I2C */
  dev_cfg->i2c_device = device_get_binding(dev_cfg->i2c_dev_name);

  if (dev_cfg->i2c_device == NULL) {
    LOG_ERR("I2C device binding error");
    return -ENXIO;
  }

  printk("\nadau1372_initialize bound to %s\n", dev_cfg->i2c_dev_name);

  dev_cfg->use_pll = false;

  return 0;
}

/* Not going to be used at the moment */
static void adau1372_enable_pll(struct adau1372 *adau1372)
{
  uint8_t val = 0;
  int timeout = 0;
  int ret;

  regmap_update_bits(adau1372, ADAU1372_REG_CLK_CTRL, ADAU1372_CLK_CTRL_PLL_EN, ADAU1372_CLK_CTRL_PLL_EN);

  do {
    /* Takes about 1ms to lock */
    k_sleep(1);
    ret = adau1372_read_reg(adau1372, ADAU1372_REG_PLL(5), &val);
    if (ret)
      break;
    timeout++;
  } while (!(val & 1) && timeout < 3);

  if (ret < 0 || !(val & 1)) {
    printk("%s: Failed to lock PLL\n", __FUNCTION__);
  }
}

static void adau1372_set_power(struct adau1372 *const adau1372, bool enable)
{
  if (adau1372->enabled == enable)
    return;

  if (enable) {
    uint8_t clk_ctrl = ADAU1372_CLK_CTRL_MCLK_EN;

    printk("%s: enabling\n", __FUNCTION__);
    /*
     * Clocks needs to be enabled before any other register can be
     * accessed.
     */
    if (adau1372->use_pll) {
      printk("%s: enabling PLL. This code path is not correct\n", __FUNCTION__);
      adau1372_enable_pll(adau1372);
      clk_ctrl |= ADAU1372_CLK_CTRL_CLKSRC;
    }
    else {
      clk_ctrl |= ADAU1372_CLK_CTRL_CC_MDIV;
    }
    
    regmap_update_bits(adau1372, ADAU1372_REG_CLK_CTRL, clk_ctrl, clk_ctrl );

  } else {
    regmap_update_bits(adau1372, ADAU1372_REG_CLK_CTRL, ADAU1372_CLK_CTRL_MCLK_EN | ADAU1372_CLK_CTRL_PLL_EN, 0);
  }

  adau1372->enabled = enable;
}

static void codec_start_output(struct device *dev)
{
#if 0
	/* powerup DAC channels */
	codec_write_reg(dev, DATA_PATH_SETUP_ADDR, DAC_LR_POWERUP_DEFAULT);

	/* unmute DAC channels */
	codec_write_reg(dev, VOL_CTRL_ADDR, VOL_CTRL_UNMUTE_DEFAULT);

	CODEC_DUMP_REGS(dev);
#endif
}

static void codec_stop_output(struct device *dev)
{
#if 0
  /* mute DAC channels */
	codec_write_reg(dev, VOL_CTRL_ADDR, VOL_CTRL_MUTE_DEFAULT);

	/* powerdown DAC channels */
	codec_write_reg(dev, DATA_PATH_SETUP_ADDR, DAC_LR_POWERDN_DEFAULT);
#endif
}

static void codec_mute_output(struct device *dev)
{
#if 0
  /* mute DAC channels */
	codec_write_reg(dev, VOL_CTRL_ADDR, VOL_CTRL_MUTE_DEFAULT);
#endif
}

static void codec_unmute_output(struct device *dev)
{
#if 0
	/* unmute DAC channels */
	codec_write_reg(dev, VOL_CTRL_ADDR, VOL_CTRL_UNMUTE_DEFAULT);
#endif
}

static int codec_set_property(struct device *dev,
		audio_property_t property, audio_channel_t channel,
		audio_property_value_t val)
{
#if 0  
	/* individual channel control not currently supported */
	if (channel != AUDIO_CHANNEL_ALL) {
		LOG_ERR("channel %u invalid. must be AUDIO_CHANNEL_ALL",
			channel);
		return -EINVAL;
	}

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		return codec_set_output_volume(dev, val.vol);

	case AUDIO_PROPERTY_OUTPUT_MUTE:
		if (val.mute) {
			codec_mute_output(dev);
		} else {
			codec_unmute_output(dev);
		}
		return 0;

	default:
		break;
	}
#endif
	return -EINVAL;
}

static int codec_apply_properties(struct device *dev)
{
	/* nothing to do because there is nothing cached */
	return 0;
}

static void codec_soft_reset(struct device *dev)
{
#if 0
  /* soft reset the DAC */
  codec_write_reg(dev, SOFT_RESET_ADDR, SOFT_RESET_ASSERT);
#endif
}

#if 0
static int codec_configure_dai(struct device *dev, audio_dai_cfg_t *cfg)
{
	u8_t val;

	/* configure I2S interface */
	val = IF_CTRL_IFTYPE(IF_CTRL_IFTYPE_I2S);
	if (cfg->i2s.options & I2S_OPT_BIT_CLK_MASTER) {
		val |= IF_CTRL_BCLK_OUT;
	}

	if (cfg->i2s.options & I2S_OPT_FRAME_CLK_MASTER) {
		val |= IF_CTRL_WCLK_OUT;
	}

	switch (cfg->i2s.word_size) {
	case AUDIO_PCM_WIDTH_16_BITS:
		val |= IF_CTRL_WLEN(IF_CTRL_WLEN_16);
		break;
	case AUDIO_PCM_WIDTH_20_BITS:
		val |= IF_CTRL_WLEN(IF_CTRL_WLEN_20);
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		val |= IF_CTRL_WLEN(IF_CTRL_WLEN_24);
		break;
	case AUDIO_PCM_WIDTH_32_BITS:
		val |= IF_CTRL_WLEN(IF_CTRL_WLEN_32);
		break;
	default:
		LOG_ERR("Unsupported PCM sample bit width %u",
				cfg->i2s.word_size);
		return -EINVAL;
	}

	codec_write_reg(dev, IF_CTRL1_ADDR, val);
	return 0;
}
#endif




static void codec_configure_output(struct device *dev)
{
#if 0
  u8_t val;

	/*
	 * set common mode voltage to 1.65V (half of AVDD)
	 * AVDD is typically 3.3V
	 */
	codec_read_reg(dev, HEADPHONE_DRV_ADDR, &val);
	val &= ~HEADPHONE_DRV_CM_MASK;
	val |= HEADPHONE_DRV_CM(CM_VOLTAGE_1P65) | HEADPHONE_DRV_RESERVED;
	codec_write_reg(dev, HEADPHONE_DRV_ADDR, val);

	/* enable pop removal on power down/up */
	codec_read_reg(dev, HP_OUT_POP_RM_ADDR, &val);
	codec_write_reg(dev, HP_OUT_POP_RM_ADDR, val | HP_OUT_POP_RM_ENABLE);

	/* route DAC output to Headphone */
	val = OUTPUT_ROUTING_HPL | OUTPUT_ROUTING_HPR;
	codec_write_reg(dev, OUTPUT_ROUTING_ADDR, val);

	/* enable volume control on Headphone out */
	codec_write_reg(dev, HPL_ANA_VOL_CTRL_ADDR,
			HPX_ANA_VOL(HPX_ANA_VOL_DEFAULT));
	codec_write_reg(dev, HPR_ANA_VOL_CTRL_ADDR,
			HPX_ANA_VOL(HPX_ANA_VOL_DEFAULT));

	/* set headphone outputs as line-out */
	codec_write_reg(dev, HEADPHONE_DRV_CTRL_ADDR, HEADPHONE_DRV_LINEOUT);

	/* unmute headphone drivers */
	codec_write_reg(dev, HPL_DRV_GAIN_CTRL_ADDR, HPX_DRV_UNMUTE);
	codec_write_reg(dev, HPR_DRV_GAIN_CTRL_ADDR, HPX_DRV_UNMUTE);

	/* power up headphone drivers */
	codec_read_reg(dev, HEADPHONE_DRV_ADDR, &val);
	val |= HEADPHONE_DRV_POWERUP | HEADPHONE_DRV_RESERVED;
	codec_write_reg(dev, HEADPHONE_DRV_ADDR, val);
#endif
}

static int codec_set_output_volume(struct device *dev, int vol)
{
#if 0
  u8_t vol_val;
	int vol_index;
	u8_t vol_array[] = {
		107, 108, 110, 113, 116, 120, 125, 128, 132, 138, 144
	};

	if ((vol > CODEC_OUTPUT_VOLUME_MAX) ||
			(vol < CODEC_OUTPUT_VOLUME_MIN)) {
		LOG_ERR("Invalid volume %d.%d dB",
				vol >> 1, ((u32_t)vol & 1) ? 5 : 0);
		return -EINVAL;
	}

	/* remove sign */
	vol = -vol;

	/* if volume is near floor, set minimum */
	if (vol > HPX_ANA_VOL_FLOOR) {
		vol_val = HPX_ANA_VOL_FLOOR;
	} else if (vol > HPX_ANA_VOL_LOW_THRESH) {
		/* lookup low volume values */
		for (vol_index = 0; vol_index < ARRAY_SIZE(vol_array); vol_index++) {
			if (vol_array[vol_index] >= vol) {
				break;
			}
		}
		vol_val = HPX_ANA_VOL_LOW_THRESH + vol_index + 1;
	} else {
		vol_val = (u8_t)vol;
	}

	codec_write_reg(dev, HPL_ANA_VOL_CTRL_ADDR, HPX_ANA_VOL(vol_val));
	codec_write_reg(dev, HPR_ANA_VOL_CTRL_ADDR, HPX_ANA_VOL(vol_val));
#endif
	return 0;
}

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void codec_read_all_regs(struct device *dev)
{
#if 0
  u8_t val;

	codec_read_reg(dev, SOFT_RESET_ADDR, &val);
	codec_read_reg(dev, NDAC_DIV_ADDR, &val);
	codec_read_reg(dev, MDAC_DIV_ADDR, &val);
	codec_read_reg(dev, OSR_MSB_ADDR, &val);
	codec_read_reg(dev, OSR_LSB_ADDR, &val);
	codec_read_reg(dev, IF_CTRL1_ADDR, &val);
	codec_read_reg(dev, BCLK_DIV_ADDR, &val);
	codec_read_reg(dev, OVF_FLAG_ADDR, &val);
	codec_read_reg(dev, PROC_BLK_SEL_ADDR, &val);
	codec_read_reg(dev, DATA_PATH_SETUP_ADDR, &val);
	codec_read_reg(dev, VOL_CTRL_ADDR, &val);
	codec_read_reg(dev, L_DIG_VOL_CTRL_ADDR, &val);
	codec_read_reg(dev, DRC_CTRL1_ADDR, &val);
	codec_read_reg(dev, L_BEEP_GEN_ADDR, &val);
	codec_read_reg(dev, R_BEEP_GEN_ADDR, &val);
	codec_read_reg(dev, BEEP_LEN_MSB_ADDR, &val);
	codec_read_reg(dev, BEEP_LEN_MIB_ADDR, &val);
	codec_read_reg(dev, BEEP_LEN_LSB_ADDR, &val);

	codec_read_reg(dev, HEADPHONE_DRV_ADDR, &val);
	codec_read_reg(dev, HP_OUT_POP_RM_ADDR, &val);
	codec_read_reg(dev, OUTPUT_ROUTING_ADDR, &val);
	codec_read_reg(dev, HPL_ANA_VOL_CTRL_ADDR, &val);
	codec_read_reg(dev, HPR_ANA_VOL_CTRL_ADDR, &val);
	codec_read_reg(dev, HPL_DRV_GAIN_CTRL_ADDR, &val);
	codec_read_reg(dev, HPR_DRV_GAIN_CTRL_ADDR, &val);
	codec_read_reg(dev, HEADPHONE_DRV_CTRL_ADDR, &val);

	codec_read_reg(dev, TIMER_MCLK_DIV_ADDR, &val);
#endif
}
#endif

#if 0
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

static int adau_calc_pll_cfg(unsigned int freq_in, unsigned int freq_out,
			     uint8_t regs[5])
{
  unsigned int r, n, m, i, j;
  unsigned int div;

  if (!freq_out) {
    r = 0;
    n = 0;
    m = 0;
    div = 0;
  } else {
    if (freq_out % freq_in != 0) {
      div = DIV_ROUND_UP(freq_in, 13500000);
      freq_in /= div;
      r = freq_out / freq_in;
      i = freq_out % freq_in;
      j = gcd(i, freq_in);
      n = i / j;
      m = freq_in / j;
      div--;
    } else {
      r = freq_out / freq_in;
      n = 0;
      m = 0;
      div = 0;
    }
    if (n > 0xffff || m > 0xffff || div > 3 || r > 8 || r < 2)
      return -EINVAL;
  }

  regs[0] = m >> 8;
  regs[1] = m & 0xff;
  regs[2] = n >> 8;
  regs[3] = n & 0xff;
  regs[4] = (r << 3) | (div << 1);
  if (m != 0)
    regs[4] |= 1; /* Fractional mode */

  return 0;
}
#endif

static int adau1372_configure(struct device *dev, struct audio_codec_cfg *cfg)
{
  struct adau1372 *const dev_cfg = DEV_CFG(dev);

  int rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_CLK_CTRL, 0);
  if (rc) {
    return rc;
  }

  k_sleep(100);

  adau1372_set_power(dev_cfg, true);

  /* ADC Config */

  /* Set the MODE_MP1 regsister 0x39 to 0 to enable serial output 0 */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_MODE_MP(1), 0);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_MODE_MP(1)) failed: %d\n", __FUNCTION__, rc);
    return rc;
  }

  /* Write 0xFF to the DECIM_PWR_MODES regsister 0x44 to enable all the ASRCs and Sync Filters */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_DECIM_PWR, 0xff);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_DECIM_PWR) failed: %d\n", __FUNCTION__, rc);
    return rc;
  }

  /* Enable ADC0 and ADC1 in the ADC_CONTROL2 register 0x1d */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ADC_CTRL2, 0x03);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ADC_CTRL2) failed: %d\n", __FUNCTION__, rc);
    return rc;
  }

  /* Enable the output ASRCs in the ASRC_MODE register 0x1a */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ASRC_MODE, 0x02);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ASRC_MODE) failed: %d\n", __FUNCTION__, rc);
    return rc;
  }

  /* Select a source for the Quad ASRCs using the ASRC0_SOURCE_0_1 register 0x18 */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ASRCO_SOURCE_0_1, 0x54);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ASRCO_SOURCE_0_1) failed: %d\n", __FUNCTION__, rc);
    return rc;
  }

  /* Unmute ADC0 and ADC1 in the ADC_CONTROL0 register 0x1b - sample rate at 192 Khz */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ADC_CTRL0, 0x01);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ADC_CTRL0) failed: %d\n", __FUNCTION__, rc);
    return rc;
  }

  /* DAC TBD */
  return 0;
}

static const struct audio_codec_api codec_driver_api = {
	.configure		= adau1372_configure,
	.start_output		= codec_start_output,
	.stop_output		= codec_stop_output,
	.set_property		= codec_set_property,
	.apply_properties	= codec_apply_properties,
};

/* want to use DT_INST_0_ADAU1372_LABEL, but instead using "ADAU1372" literally. This
 * matches, as I think it is supposed to, the statement:
 *    label = "ADAU1372";
 * in mimxrt1064_evk.dts
 */

DEVICE_AND_API_INIT(adau1372, "ADAU1372", adau1372_initialize,
		    &driver_data, &device_config, POST_KERNEL,
		    CONFIG_AUDIO_CODEC_INIT_PRIORITY, &codec_driver_api);
