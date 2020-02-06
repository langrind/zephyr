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
#include "str_codec.h"
#include "adau1372.h"

#define LOG_LEVEL CONFIG_AUDIO_CODEC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adau1372);

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
};

static struct adau1372_driver_data driver_data = {
  .unused = 0
};

#define DEV_CFG(dev)  ((struct adau1372 *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct adau1372_driver_data *const)(dev)->driver_data)

/* Perform the two byte address write, and N-byte read */
static int adau1372_i2c_read(struct adau1372 *const dev_cfg, uint16_t startAddr,
			     uint8_t *buf, uint32_t numBytes)
{
  uint8_t addrBuffer[2];

  addrBuffer[1] = startAddr & 0xFF;
  addrBuffer[0] = startAddr >> 8;
  return i2c_write_read(dev_cfg->i2c_device, dev_cfg->i2c_address, addrBuffer,
			sizeof(addrBuffer), buf, numBytes);
}

static int adau1372_i2c_write(struct adau1372 *const dev_cfg, uint16_t startAddr,
			      uint8_t *buf, uint32_t numDataBytes)
{
  size_t   bufSize = sizeof startAddr + numDataBytes;
  uint8_t *txBuf = alloca(bufSize);

  txBuf[0] = startAddr >> 8;
  txBuf[1] = startAddr & 0xFF;
  memcpy(&txBuf[sizeof startAddr], buf, numDataBytes);

  return i2c_write(dev_cfg->i2c_device, txBuf, bufSize, dev_cfg->i2c_address);
}

/* wrapper for single-byte operation */
static int adau1372_read_reg(struct adau1372 *const dev_cfg, uint16_t regAddr,
			     uint8_t *val)
{
  return adau1372_i2c_read(dev_cfg, regAddr, val, 1);
}

/* wrapper for single-byte operation */
static int adau1372_write_reg(struct adau1372 *const dev_cfg, uint16_t regAddr,
			      uint8_t val)
{
  return adau1372_i2c_write(dev_cfg, regAddr, &val, 1);
}

/* to make it slightly easier to port routines from the linux driver: */
static int regmap_update_bits(struct adau1372 *const dev_cfg, uint16_t regAddr,
			      uint8_t mask, uint8_t val)
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
    regmap_update_bits(adau1372, ADAU1372_REG_CLK_CTRL,
		       ADAU1372_CLK_CTRL_MCLK_EN | ADAU1372_CLK_CTRL_PLL_EN, 0);
  }

  adau1372->enabled = enable;
}

static void adau1372_start_output(struct device *dev)
{
}

static void adau1372_stop_output(struct device *dev)
{
}

static int adau1372_set_property(struct device *dev, audio_property_t property,
				 audio_channel_t channel,
				 audio_property_value_t val)
{
  /* individual channel control not currently supported */
  if (channel != AUDIO_CHANNEL_ALL) {
    LOG_ERR("channel %u invalid. must be AUDIO_CHANNEL_ALL",
	    channel);
    return -EINVAL;
  }

  switch (property) {
  case AUDIO_PROPERTY_OUTPUT_VOLUME:
    break;
  case AUDIO_PROPERTY_OUTPUT_MUTE:
    break;
  default:
    return -EINVAL;
    break;
  }
  return 0;
}

static int adau1372_apply_properties(struct device *dev)
{
  /* nothing to do because there is nothing cached */
  return 0;
}

#if 0
static int adau1372_configure_dai(struct device *dev, audio_dai_cfg_t *cfg)
{
  /* configure I2S interface */
  if (cfg->i2s.options & I2S_OPT_BIT_CLK_MASTER) {
  }

  if (cfg->i2s.options & I2S_OPT_FRAME_CLK_MASTER) {
  }

  switch (cfg->i2s.word_size) {
  case AUDIO_PCM_WIDTH_16_BITS:
    break;
  case AUDIO_PCM_WIDTH_20_BITS:
    break;
  case AUDIO_PCM_WIDTH_24_BITS:
    break;
  case AUDIO_PCM_WIDTH_32_BITS:
    break;
  default:
    LOG_ERR("Unsupported PCM sample bit width %u", cfg->i2s.word_size);
    return -EINVAL;
  }

  return 0;
}
#endif

/*
 * Page 25 of the data sheet lists three stages of init:
 * 1) Configure Clocks
 * 2) Set up ADCs, DACs and multifunction pins
 * 3) Set up serial ports and ASRCs
 */
static int adau1372_configure(struct device *dev, struct audio_codec_cfg *cfg)
{
  printk("%s\n", __FUNCTION__);

  // cfg.dai_cfg.i2s is struct i2s_config 

  struct adau1372 *const dev_cfg = DEV_CFG(dev);

  int rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_CLK_CTRL, 0);
  if (rc) {
    return rc;
  }

  k_sleep(100);

  adau1372_set_power(dev_cfg, true);

  /* ADC (input path) Config */

  /* Set the MODE_MP1 regsister 0x39 to 0 to enable serial output 0 */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_MODE_MP(1), 0);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_MODE_MP(1)) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Write 0xFF to the DECIM_PWR_MODES regsister 0x44 to enable all the ASRCs and
   * Sync Filters
   */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_DECIM_PWR, 0xff);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_DECIM_PWR) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Enable ADC0 and ADC1 in the ADC_CONTROL2 register 0x1d  */
  /* Disable ADC1 for debugging = 0x01, 0therwise 0x03 */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ADC_CTRL2, 0x01);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ADC_CTRL2) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Enable the output ASRCs in the ASRC_MODE register 0x1a */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ASRC_MODE, 0x02);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ASRC_MODE) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Select a source for the Quad ASRCs using the ASRC0_SOURCE_0_1 register 0x18 */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ASRCO_SOURCE_0_1, 0x54);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ASRCO_SOURCE_0_1) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Unmute ADC0 and ADC1 in the ADC_CONTROL0 register 0x1b - sample rate at 96 Khz */
  /* Mute ADC1 for debugging = 0x10, otherwise 0x00 */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ADC_CTRL0, 0x10);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ADC_CTRL0) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* DAC (output path) Config */

  /* Enable ASRC and DAC modulator power using the INTERP_PWR_MODES (0x45) */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_INTERP_PWR, 0x0F);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_INTERP_PWR) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }
  
  /* Enable the input ASRCs in the ASRC_MODE register (0x1A) */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_ASRC_MODE, 0x03);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_ASRC_MODE) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }
  
  /* Input ASRCs -> DAC Sources: ASRC CH0->DAC_SOURCE0 and CH1->DAC_SOURCE1 (0x11)*/
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_DAC_SOURCE, 0xDC);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_DAC_SOURCE) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Enable the DACS in the DAC_CONTROTL1 register (0x2E) but leave muted */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_DAC_CTRL, 0x0F);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_DAC_CTRL) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /*
   * Enable the power to the HPOUTLP/LOUTLP output and the HPOUTLN/LOUTLN output
   * in the OP_STAGE_CTRL register (0x43)
   *
   * XXX For now I am using headphone mode because I think it will be convenient
   * for testing
   */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_OP_STAGE_CTRL, 0x1A);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_OP_STAGE_CTRL) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* wait 6 milliseconds - data sheet says to do this */
  k_sleep(6);
  
  /* Unmute the DACS using the DAC_CONTROTL1 register (0x2E) */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_DAC_CTRL, 0x03);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_DAC_CTRL) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* wait 6 milliseconds data sheet says to do this */
  k_sleep(6);

  /* Unmute the headphones/line outputs using the OP_STAGE_MUTE register (0x31) */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_OP_STAGE_MUTE, 0x0A);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_OP_STAGE_MUTE) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* Set up Serial Ports and ASRCs */

  /* Set up serial ports using SAI0 (0x32) and SAI1 (0x33) */

  /* Stereo I2S mode, 8 kHz by default - sample freq needs to be configurable from user */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_SAI0, 0x01);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_SAI0) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  /* 32 BCLKs/channel/cycle, slave mode, data change on falling edge, MSB first */
  rc = adau1372_write_reg(dev_cfg, ADAU1372_REG_SAI1, 0x00);
  if (rc) {
    printk("%s: adau1372_write_reg(ADAU1372_REG_SAI1) failed: %d\n",
	   __FUNCTION__, rc);
    return rc;
  }

  return 0;
}

static const struct audio_codec_api codec_driver_api = {
	.configure		= adau1372_configure,
	.start_output		= adau1372_start_output,
	.stop_output		= adau1372_stop_output,
	.set_property		= adau1372_set_property,
	.apply_properties	= adau1372_apply_properties,
};

DEVICE_AND_API_INIT(adau1372, DT_INST_0_ADI_ADAU1372_LABEL, adau1372_initialize,
		    &driver_data, &device_config, POST_KERNEL,
		    CONFIG_AUDIO_CODEC_INIT_PRIORITY, &codec_driver_api);

