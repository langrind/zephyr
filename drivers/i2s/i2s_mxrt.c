/*
 * Copyright (c) 2020 Nik Langrind
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief I2S bus driver for NXP i.MX RT MCU family.
 *
 * Glue for MCUX-generated driver for FlexIO-based I2S including EDMA support
 */

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/dma.h>
#include <drivers/i2s.h>
#include <soc.h>
#include "i2s_mxrt.h"

#define LOG_DOMAIN dev_i2s_mxrt
#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);


static int i2s_mxrt_configure(struct device *dev, enum i2s_dir dir,
			       struct i2s_config *i2s_cfg)
{
  const struct i2s_mxrt_cfg *const cfg = DEV_CFG(dev);
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);

  (void)cfg;
  (void)dev_data;

  if (dir == I2S_DIR_RX) {
    printk("%s: RX\n", __FUNCTION__);
  } else if (dir == I2S_DIR_TX) {
    printk("%s: RX\n", __FUNCTION__);
  } else {
    printk("Either RX or TX direction must be selected\n");
    LOG_ERR("Either RX or TX direction must be selected");
    return -EINVAL;
  }
  return 0;
}

static int i2s_mxrt_read(struct device *dev, void **mem_block, size_t *size)
{
  struct i2s_mxrt_dev_data *const dev_data = DEV_DATA(dev);
  (void)dev_data;
  return 0;
}

static int i2s_mxrt_write(struct device *dev, void *mem_block, size_t size)
{
  struct i2s_mxrt_dev_data *const dev_data = DEV_DATA(dev);
  (void)dev_data;
  return 0;
}

static int i2s_mxrt_trigger(struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
  return 0;
}

static int i2s_mxrt_initialize(struct device *dev)
{
  printk("%s\n", __FUNCTION__);
  return 0;
}

static const struct i2s_driver_api i2s_mxrt_driver_api = {
	.configure = i2s_mxrt_configure,
	.read = i2s_mxrt_read,
	.write = i2s_mxrt_write,
	.trigger = i2s_mxrt_trigger,
};

/* I2S0 */

static struct device DEVICE_NAME_GET(i2s0_mxrt);

static struct device *get_dev_from_dma_channel(u32_t dma_channel)
{
	return &DEVICE_NAME_GET(i2s0_mxrt);
}

static void i2s0_mxrt_irq_config(void)
{
#if 0
	IRQ_CONNECT(MXRT_IRQn, CONFIG_I2S_MXRT_0_IRQ_PRI, i2s_mxrt_isr,
		    DEVICE_GET(i2s0_mxrt), 0);
#endif
}


static const struct i2s_mxrt_cfg i2s0_mxrt_config = {
	.unused = 0,
};

static struct i2s_mxrt_data i2s0_mxrt_data = {
  .unused = 0,
};

/* Support all three instances xxx */
DEVICE_AND_API_INIT(i2s0_mxrt, DT_INST_2_NXP_FLEXIOI2S_LABEL, &i2s_mxrt_initialize,
		    &i2s0_mxrt_data, &i2s0_mxrt_config, POST_KERNEL,
		    CONFIG_I2S_INIT_PRIORITY, &i2s_mxrt_driver_api);
