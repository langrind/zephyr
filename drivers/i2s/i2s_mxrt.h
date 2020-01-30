/*
 * Copyright (c) 2020 Nik Langrind
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <fsl_flexio_i2s.h>
#include <fsl_flexio_i2s_edma.h>
#include <fsl_edma.h>
#include <fsl_dmamux.h>
#include <stdint.h>

#define DEV_CFG(dev)							\
	(const struct i2s_mxrt_cfg * const)((dev)->config->config_info)

#define DEV_DATA(dev)						\
	((struct i2s_mxrt_data *const)(dev)->driver_data)

/* Device constant configuration parameters */
struct i2s_mxrt_cfg {
	FLEXIO_I2S_Type mcux_base;
	// SPI_TypeDef *i2s;
	// struct stm32_pclken pclken;
	// u32_t i2s_clk_sel;
	// void (*irq_config)(struct device *dev);

	uint32_t unused;
};

/* Device run time data */
struct i2s_mxrt_data {
	edma_config_t dmaConfig;
	DMAMUX_Type   *dmamuxBase;
	edma_handle_t txDmaHandle;
	edma_handle_t rxDmaHandle;

	FLEXIO_Type   *flexioBase; /*!< FlexIO base pointer */
};


