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

/* Device constant configuration parameters */
struct i2s_mxrt_cfg {
  FLEXIO_I2S_Type mcux_base;
};

/*
 * Tx and Rx stream
 *
 * in_queue and out_queue are used as follows
 *   transmit stream:
 *      application provided buffer is queued to in_queue until it can be passed
 *      to the MCUX driver layer
 *      when MCUX driver layer can accept it, buffer is retrieved from in_queue,
 *      passed to MCUX layer and queued to out_queue.
 *      when driver layer completes, buffer is retrieved from out_queue and freed.
 *
 *   receive stream:
 *      driver allocates buffer from slab and loads DMA
 *      buffer is queued to in_queue
 *      when DMA completes, buffer is retrieved from in_queue and queued to
 *      out_queue
 *	when application reads, buffer is read (may optionally block) from
 *      out_queue and presented to application.
 */
#define I2S_MXRT_BUF_Q_LEN 2
struct stream {
  enum i2s_state      state;
  struct k_msgq       in_queue;
  void                *in_msgs[I2S_MXRT_BUF_Q_LEN];
  struct k_msgq       out_queue;
  void                *out_msgs[I2S_MXRT_BUF_Q_LEN];
  flexio_i2s_handle_t mcux_handle;
};

/* Device run time data */
struct i2s_mxrt_data {
  struct i2s_config   i2s_cfg;
  uint32_t            n_ints_handled;
  uint32_t            n_tx_callbacks;
  uint32_t            n_rx_callbacks;
  FLEXIO_I2S_Type     mcux_base;
  struct k_mem_slab   *mem_slab; /* I think I want RX and TX memslab */
  struct stream       tx;
  struct stream       rx;
  bool                flexio_inited;
};


