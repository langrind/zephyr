/*
 * Copyright (c) 2020 Nik Langrind
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief I2S bus driver for NXP i.MX RT MCU family.
 *
 * Glue for MCUX-generated driver for FlexIO-based I2S using interrupt-driven transfer
 */

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/i2s.h>
#include <soc.h>
#include "i2s_mxrt.h"

#define LOG_DOMAIN dev_i2s_mxrt
#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);

/* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM)
 * fref * (32 + 100,000,000/130,222,463)
 * fref * 32.76791667
 */
const clock_audio_pll_config_t audioPllConfig = {
  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
  .loopDivider = 32U,
  /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
  .postDivider = 1U,
  /* 30 bit numerator of fractional loop divider.*/
  .numerator   = 0x05F5E100U,
  /* 30 bit denominator of fractional loop divider */
  .denominator = 0x07C3097FU,
};

/*
 * Select Audio PLL (786.43 MHz) as flexio clock source, need to sync with SAI
 * clock, or the codec may not work (XXX that's true for WM8960 probably not
 * for ADAU1372)
 */
#define DEMO_FLEXIO_CLKSRC_SEL (0U)
/* Clock pre divider for flexio clock source */
#define DEMO_FLEXIO_CLKSRC_PRE_DIV (7U)
/* Clock divider for flexio clock source */
#define DEMO_FLEXIO_CLKSRC_DIV (7U)
#define DEMO_FLEXIO_CLK_FREQ \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_FLEXIO_CLKSRC_PRE_DIV + 1U) \
     / (DEMO_FLEXIO_CLKSRC_DIV + 1U))

#define DEV_CFG(dev)  (const struct i2s_mxrt_cfg * const)((dev)->config->config_info)
#define DEV_DATA(dev) ((struct i2s_mxrt_data *const)(dev)->driver_data)

/*
 * Grab a buffer from input queue, pass it to flexio driver layer
 * If flexio can't send it, free the buffer. Otherwise put it in
 * the output queue to be retrieved by a future ISR
 *
 * It is assumed that this is called with interrupts disabled,
 * either because it is in the ISR callback or because the caller
 * has called irq_lock()
 *
 * We could tweak the flexio layer to pass us back the data address,
 * hence eliminating the output queue.

 */
static int i2s_mxrt_tx_stream_transmit(struct i2s_mxrt_data *dev_data)
{
  int ret = 0;
  void *buffer;
  struct stream *strm = &dev_data->tx;

  /* retrieve buffer from input queue */
  ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("No buffer in input queue to start transmission");
    return ret;
  }

  flexio_i2s_transfer_t xfer;
  xfer.dataSize = dev_data->i2s_cfg.block_size;
  xfer.data     = buffer;
  ret = FLEXIO_I2S_TransferSendNonBlocking(&dev_data->mcux_base,
					   &dev_data->tx.mcux_handle, &xfer);

  if (ret != kStatus_Success) {
    LOG_ERR("FLEXIO_I2S_TransferSendNonBlocking failed (%d)", ret);
    k_mem_slab_free(dev_data->i2s_cfg.mem_slab, &buffer);
  }
  else {
    /* put buffer in output queue, for future ISR to retrieve */
    ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
    if (ret != kStatus_Success) {
      LOG_ERR("failed to put buffer in output queue");
    }
  }
  return ret;
}

/*
 * Callback from flexio layer. This is invoked when a single flexio_i2s_transfer_t
 * is complete
 */
static void i2s_mxrt_tx_transfer_callback(FLEXIO_I2S_Type *i2sBase,
					  flexio_i2s_handle_t *handle,
					  status_t status, void *userData)
{
  struct i2s_mxrt_data *const dev_data = userData;
  dev_data->n_tx_callbacks++;

  struct stream *strm = &dev_data->tx;
  void *buffer;

  int ret = k_msgq_get(&strm->out_queue, &buffer, K_NO_WAIT);
  if (ret == 0) {
    /* transmission complete. free the buffer */
    k_mem_slab_free(dev_data->i2s_cfg.mem_slab, &buffer);
  } else {
    LOG_ERR("no buffer in output queue for channel");
  }

  switch (strm->state) {

  case I2S_STATE_RUNNING:
    i2s_mxrt_tx_stream_transmit(dev_data);
    break;
  }
}

void i2s_mxrt_rx_stream_disable(struct i2s_mxrt_data *const dev_data)
{

}

static int i2s_mxrt_rx_stream_receive(struct i2s_mxrt_data *dev_data)
{
  int ret = 0;
  void *buffer;
  struct stream *strm = &dev_data->rx;

  /* allocate new buffer for next audio frame */
  ret = k_mem_slab_alloc(dev_data->i2s_cfg.mem_slab, &buffer, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("buffer alloc from slab %p err %d", dev_data->i2s_cfg.mem_slab, ret);
    i2s_mxrt_rx_stream_disable(dev_data);
    strm->state = I2S_STATE_READY;
  } else {

    /* SOC_DCACHE_INVALIDATE(buffer, dev_data->cfg.block_size); */

    flexio_i2s_transfer_t xfer;
    xfer.dataSize = dev_data->i2s_cfg.block_size;
    xfer.data     = buffer;
    ret = FLEXIO_I2S_TransferReceiveNonBlocking(&dev_data->mcux_base,
						&dev_data->rx.mcux_handle, &xfer);
    if (ret != kStatus_Success) {
      LOG_ERR("FLEXIO_I2S_TransferReceiveNonBlocking failed (%d)", ret);
      k_mem_slab_free(dev_data->i2s_cfg.mem_slab, &buffer);
    }
    else {
      /* put buffer in input queue, we will g */
      ret = k_msgq_put(&strm->in_queue, &buffer, K_NO_WAIT);
      if (ret != 0) {
	LOG_ERR("buffer %p -> in_queue %p err %d", buffer, &strm->in_queue, ret);
	k_mem_slab_free(dev_data->i2s_cfg.mem_slab, &buffer);
      }
    }
  }
  return ret;
}

static int dbg_captured = false;
char dbg_capture_buf[64];
/*
 * Callback from flexio layer. This is invoked when a single flexio_i2s_transfer_t
 * is complete
 */
static void i2s_mxrt_rx_transfer_callback(FLEXIO_I2S_Type *i2sBase,
					  flexio_i2s_handle_t *handle,
					  status_t status, void *userData)
{
  struct i2s_mxrt_data *const dev_data = userData;
  dev_data->n_rx_callbacks++;

  int  ret;
  void *buffer;

  struct stream *strm = &dev_data->rx;
  switch (strm->state) {

  case I2S_STATE_RUNNING:
    /* retrieve buffer from input queue */
    ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
    if (ret != 0) {
      LOG_ERR("get buffer from in_queue %p failed (%d)", &strm->in_queue, ret);
    }
    else {
      if (!dbg_captured) {
	dbg_captured= true;
	memcpy(dbg_capture_buf, buffer, 64);
      }      

      /* put buffer to output queue */
      ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
      if (ret != 0) {
	LOG_ERR("buffer %p -> out_queue %p err %d", buffer, &strm->out_queue, ret);
      }
    }

    /* kick off next receive */
    i2s_mxrt_rx_stream_receive(dev_data);
    break;

  case I2S_STATE_STOPPING:
    i2s_mxrt_rx_stream_disable(dev_data);
    strm->state = I2S_STATE_READY;
    break;
  }
}

static int i2s_mxrt_configure(struct device *dev, enum i2s_dir dir,
			      struct i2s_config *i2s_cfg)
{
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);

  if (dir == I2S_DIR_RX) {
    printk("%s: RX\n", __FUNCTION__);
  } else if (dir == I2S_DIR_TX) {
    printk("%s: TX\n", __FUNCTION__);
  } else {
    printk("Either RX or TX direction must be selected\n");
    LOG_ERR("Either RX or TX direction must be selected");
    return -EINVAL;
  }

  /* stash the mem_slab - I think I want RX and TX separate mem slabs */
  dev_data->mem_slab = i2s_cfg->mem_slab;

#if 0
  /* i2s_cfg has:
  	u8_t word_size;
	u8_t channels;
	i2s_fmt_t format;
	i2s_opt_t options;
	u32_t frame_clk_freq;
	struct k_mem_slab *mem_slab;
	size_t block_size;
	s32_t timeout;
  */
  /* flexio_i2s_config has: */
  /* XXX set rx and tx TimerPolarity tell flexio to use rising edge (per customer requirement)
   * XXX but map i2s_cfg->format and i2s_cfg->options fields, and then let caller decide
   * XXX not dealing with this until I have the transfers working again after current refactor
   */  
  bool enableI2S;                                  /*!< Enable FlexIO I2S */
  flexio_i2s_master_slave_t masterSlave;           /*!< Master or slave */
  flexio_pin_polarity_t txPinPolarity;             /*!< Tx data pin polarity, active high or low */
  flexio_pin_polarity_t rxPinPolarity;             /*!< Rx data pin polarity */
  flexio_pin_polarity_t bclkPinPolarity;           /*!< Bit clock pin polarity */
  flexio_pin_polarity_t fsPinPolarity;             /*!< Frame sync pin polarity */
  flexio_shifter_timer_polarity_t txTimerPolarity; /*!< Tx data valid on bclk rising or falling edge */
  flexio_shifter_timer_polarity_t rxTimerPolarity; /*!< Rx data valid on bclk rising or falling edge */
#endif

  if (dev_data->flexio_inited) {
    printk("%s: Not re-initing flexio\n", __FUNCTION__);
  }
  else {
    dev_data->i2s_cfg = *i2s_cfg;

    flexio_i2s_config_t config;
    FLEXIO_I2S_GetDefaultConfig(&config);
    /* XXX still need to tweak config.xxx based on i2s_cfg */
    /* Default is:
     * config->txTimerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
     * but that causes data to shift on the edge used by the Codec to sample.
     * so change it so we shift on the other edge
     */
    config.txTimerPolarity = kFLEXIO_ShifterTimerPolarityOnNegitive;

    FLEXIO_I2S_Init(&dev_data->mcux_base, &config);
    dev_data->flexio_inited = true;
  }

  /* Configure the audio format */
  flexio_i2s_format_t format;
  format.bitWidth      = i2s_cfg->word_size;
  format.sampleRate_Hz = i2s_cfg->frame_clk_freq;

  if (dir == I2S_DIR_RX) {

    /* Create the handle - handle is context for MCUX driver code */  
    FLEXIO_I2S_TransferRxCreateHandle(&dev_data->mcux_base,
				      &dev_data->rx.mcux_handle,
				      i2s_mxrt_rx_transfer_callback, dev_data);
    /* Set audio format. DEMO_FLEXIO_CLK_FREQ works out to 12.288 Mhz (12,287,968) */
    FLEXIO_I2S_TransferSetFormat(&dev_data->mcux_base, &dev_data->rx.mcux_handle,
				 &format, DEMO_FLEXIO_CLK_FREQ);

  }
  else if (dir == I2S_DIR_TX) {

    /* Create the handle - handle is context for MCUX driver code */  
    FLEXIO_I2S_TransferTxCreateHandle(&dev_data->mcux_base, &dev_data->tx.mcux_handle,
				      i2s_mxrt_tx_transfer_callback, dev_data);
    /* Set audio format. DEMO_FLEXIO_CLK_FREQ works out to 12.288 Mhz (12,287,968) */
    FLEXIO_I2S_TransferSetFormat(&dev_data->mcux_base, &dev_data->tx.mcux_handle,
				 &format, DEMO_FLEXIO_CLK_FREQ);

  }

  return 0;
}

/**
 * @brief Read data from the RX queue.
 *
 * THIS COMMENT IS COPIED FROM i2s.h AND WILL BE UPDATED AS PROGRESS WARRANTS
 * Data received by the I2S interface is stored in the RX queue consisting of
 * memory blocks preallocated by this function from rx_mem_slab (as defined by
 * i2s_configure). Ownership of the RX memory block is passed on to the user
 * application which has to release it.
 *
 * The data is read in chunks equal to the size of the memory block. If the
 * interface is in READY state the number of bytes read can be smaller.
 *
 * If there is no data in the RX queue the function will block waiting for
 * the next RX memory block to fill in. This operation can timeout as defined
 * by i2s_configure. If the timeout value is set to K_NO_WAIT the function
 * is non-blocking.
 *
 * Reading from the RX queue is possible in any state other than NOT_READY.
 * If the interface is in the ERROR state it is still possible to read all the
 * valid data stored in RX queue. Afterwards the function will return -EIO
 * error.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mem_block Pointer to the RX memory block containing received data.
 * @param size Pointer to the variable storing the number of bytes read.
 *
 * @retval 0 If successful.
 * @retval -EIO The interface is in NOT_READY or ERROR state and there are no
 *         more data blocks in the RX queue.
 * @retval -EBUSY Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
static int i2s_mxrt_read(struct device *dev, void **mem_block, size_t *size)
{
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);

  struct stream *strm = &dev_data->rx;
  void          *buffer;
  int           ret = 0;

  if (strm->state == I2S_STATE_NOT_READY) {
    LOG_ERR("invalid state %d", strm->state);
    return -EIO;
  }

  ret = k_msgq_get(&strm->out_queue, &buffer, dev_data->i2s_cfg.timeout);
  if (ret != 0) {
    return -EAGAIN;
  }

  *mem_block = buffer;
  *size = dev_data->i2s_cfg.block_size;
  return 0;
}

/**
 * @brief Write data to the TX queue.
 *
 * THIS COMMENT IS COPIED FROM i2s.h AND WILL BE UPDATED AS PROGRESS WARRANTS
 * Data to be sent by the I2S interface is stored first in the TX queue. TX
 * queue consists of memory blocks preallocated by the user from tx_mem_slab
 * (as defined by i2s_configure). This function takes ownership of the memory
 * block and will release it when all data are transmitted.
 *
 * If there are no free slots in the TX queue the function will block waiting
 * for the next TX memory block to be send and removed from the queue. This
 * operation can timeout as defined by i2s_configure. If the timeout value is
 * set to K_NO_WAIT the function is non-blocking.
 *
 * Writing to the TX queue is only possible if the interface is in READY or
 * RUNNING state.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mem_block Pointer to the TX memory block containing data to be sent.
 * @param size Number of bytes to write. This value has to be equal or smaller
 *        than the size of the memory block.
 *
 * @retval 0 If successful.
 * @retval -EIO The interface is not in READY or RUNNING state.
 * @retval -EBUSY Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
static int i2s_mxrt_write(struct device *dev, void *mem_block, size_t size)
{
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);
  struct stream *strm = &dev_data->tx;
  int ret;

  if (strm->state != I2S_STATE_RUNNING &&
      strm->state != I2S_STATE_READY) {
    LOG_ERR("invalid state (%d)", strm->state);
    return -EIO;
  }

  /* XXX not doing DMA yet, but when we do, will have to take care of this:
   * SOC_DCACHE_FLUSH(mem_block, size);
   */

  ret = k_msgq_put(&strm->in_queue, &mem_block, dev_data->i2s_cfg.timeout);
  if (ret) {
    LOG_ERR("k_msgq_put failed %d", ret);
    return ret;
  }

  return ret;
}

/* Debug hack */
static void i2s_mxrt_print(struct i2s_mxrt_data *const dev_data)
{
  printk("Ints: %d   TX cbs: %d    RX cbs: %d\n", dev_data->n_ints_handled,
	 dev_data->n_tx_callbacks, dev_data->n_rx_callbacks);

  for ( int i = 0; i < 16; i++ ) {
    for (int j = 0; j < 4; j++) {
      int ix = i * 4 + j;
      printk("%02x", dbg_capture_buf[ix]);
    }
    int ix = i*4;
    char * tmp = &dbg_capture_buf[ix];
    int x = (tmp[1] << 0) + (tmp[2] << 8) + (tmp[3] << 16);
    printk(" : %06x", x);
    printk("\n");
  }
}

static int i2s_mxrt_trigger(struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);
  struct stream *strm;
  unsigned int key;
  int ret = 0;

  strm = (dir == I2S_DIR_TX) ? &dev_data->tx : &dev_data->rx;

  key = irq_lock();
  switch (cmd) {
  case I2S_TRIGGER_START:
    if (strm->state != I2S_STATE_READY) {
      LOG_ERR("START trigger: invalid state %u", strm->state);
      ret = -EIO;
      break;
    }

    __ASSERT_NO_MSG(strm->mem_block == NULL);

    if (dir == I2S_DIR_TX) {
      i2s_mxrt_tx_stream_transmit(dev_data);
    }
    else if (dir == I2S_DIR_RX) {
      i2s_mxrt_tx_stream_transmit(dev_data);
      i2s_mxrt_rx_stream_receive(dev_data);
    }
    else {
      printk("INVALID stream direction\n");
      ret = -EINVAL;
      break;
    }
    strm->state = I2S_STATE_RUNNING;      
    break;

  case I2S_TRIGGER_STOP:
  case I2S_TRIGGER_DRAIN:
  case I2S_TRIGGER_DROP:
    i2s_mxrt_print(dev_data);
    if (strm->state != I2S_STATE_RUNNING) {
      LOG_DBG("STOP/DRAIN/DROP trigger: invalid state");
      ret = -EIO;
      break;
    }
    strm->state = I2S_STATE_STOPPING;
    break;

  case I2S_TRIGGER_PREPARE:
    break;

  default:
    LOG_ERR("Unsupported trigger command");
    ret = -EINVAL;
  }

  irq_unlock(key);
  return ret;
}

int i2s_mxrt_test_transfer(struct device *dev, uint8_t *txData, size_t size)
{
	printk("%s\n", __FUNCTION__);

	const struct i2s_mxrt_cfg *const dev_cfg = DEV_CFG(dev);

	FLEXIO_I2S_WriteBlocking(&dev_cfg->mcux_base, 24, txData, size);
	return 0;
}

int i2s_mxrt_test_receive(struct device *dev, uint8_t *rxData, size_t size)
{
	//printk("%s\n", __FUNCTION__);

	const struct i2s_mxrt_cfg *const dev_cfg = DEV_CFG(dev);

	FLEXIO_I2S_ReadBlocking(&dev_cfg->mcux_base, 24, rxData,  size);
	return 0;
}

#if 0
flexio_i2s_handle_t txHandle      = {0};
flexio_i2s_handle_t rxHandle      = {0};

int i2s_mxrt_test_int_transfer(struct device *dev, uint8_t *txData, size_t size)
{
	printk("%s\n", __FUNCTION__);

	const struct i2s_mxrt_cfg *const dev_cfg = DEV_CFG(dev);

	flexio_i2s_transfer_t txXfer;
	txXfer.dataSize = size;
	txXfer.data     = txData;
	FLEXIO_I2S_TransferSendNonBlocking(&dev_cfg->mcux_base, &txHandle, &txXfer);

	return 0;
}
#endif

static void i2s0_mxrt_irq_config(void);

static int i2s_mxrt_initialize(struct device *dev)
{
  printk("%s\n", __FUNCTION__);

  const struct i2s_mxrt_cfg *const dev_cfg = DEV_CFG(dev);
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);

  /* Initialize the buffer queues */
  k_msgq_init(&dev_data->tx.in_queue, (char *)dev_data->tx.in_msgs,
	      sizeof(void *), I2S_MXRT_BUF_Q_LEN);
  k_msgq_init(&dev_data->rx.in_queue, (char *)dev_data->rx.in_msgs,
	      sizeof(void *), I2S_MXRT_BUF_Q_LEN);
  k_msgq_init(&dev_data->tx.out_queue, (char *)dev_data->tx.out_msgs,
	      sizeof(void *), I2S_MXRT_BUF_Q_LEN);
  k_msgq_init(&dev_data->rx.out_queue, (char *)dev_data->rx.out_msgs,
	      sizeof(void *), I2S_MXRT_BUF_Q_LEN);

  dev_data->tx.state = I2S_STATE_NOT_READY;
  dev_data->rx.state = I2S_STATE_NOT_READY;

  i2s0_mxrt_irq_config();

  CLOCK_InitAudioPll(&audioPllConfig);

  /* This clock configuration was cribbed from MCUX SDK example 
   * "evkmimxrt1064_flexio3_i2s_interrupt_transfer". I don't know
   * enough about MXRT to know why this sets up Flexio2Mux instead
   * of flexio3Mux.
   */
  CLOCK_SetMux(kCLOCK_Flexio2Mux, DEMO_FLEXIO_CLKSRC_SEL);
  CLOCK_SetDiv(kCLOCK_Flexio2PreDiv, DEMO_FLEXIO_CLKSRC_PRE_DIV);
  CLOCK_SetDiv(kCLOCK_Flexio2Div, DEMO_FLEXIO_CLKSRC_DIV);

  /* XXX clean up this const confusion */
  memcpy(&dev_data->mcux_base, &dev_cfg->mcux_base, sizeof(dev_cfg->mcux_base));

  dev_data->tx.state = I2S_STATE_READY;
  dev_data->rx.state = I2S_STATE_READY;
  
#if 0
  /* register ISR might want to do it this way to avoid forward decl  */
  dev_cfg->irq_connect();

  LOG_INF("Device %s initialized", DEV_NAME(dev));

#endif
  return 0;
}

static const struct i2s_driver_api i2s_mxrt_driver_api = {
	.configure = i2s_mxrt_configure,
	.read = i2s_mxrt_read,
	.write = i2s_mxrt_write,
	.trigger = i2s_mxrt_trigger,
};

static struct device DEVICE_NAME_GET(i2s0_mxrt);

static void i2s_mxrt_flexio_isr(void *arg)
{
  struct device *dev = arg;
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);

  dev_data->n_ints_handled++;
  FLEXIO_CommonIRQHandler();
}

static void i2s0_mxrt_irq_config(void)
{
  IRQ_CONNECT(DT_INST_2_NXP_FLEXIOI2S_IRQ_0, DT_INST_2_NXP_FLEXIOI2S_IRQ_0_PRIORITY,
	      i2s_mxrt_flexio_isr, DEVICE_GET(i2s0_mxrt), 0);
}

/* The pin definitions should come from the DTS definition
 * when I get around to it
 */
static const struct i2s_mxrt_cfg i2s0_mxrt_config = {
	.mcux_base.flexioBase     = (FLEXIO_Type *)DT_INST_2_NXP_FLEXIOI2S_BASE_ADDRESS,
	.mcux_base.bclkPinIndex   = 11,
	.mcux_base.fsPinIndex     = 10,
	.mcux_base.txPinIndex     = 5,
	.mcux_base.rxPinIndex     = 1,
	.mcux_base.txShifterIndex = 0,
	.mcux_base.rxShifterIndex = 2,
	.mcux_base.bclkTimerIndex = 0,
	.mcux_base.fsTimerIndex   = 1,
};

static struct i2s_mxrt_data i2s0_mxrt_data = { 0 };

/* XXX TBS Support all three instances */
DEVICE_AND_API_INIT(i2s0_mxrt, DT_INST_2_NXP_FLEXIOI2S_LABEL, &i2s_mxrt_initialize,
		    &i2s0_mxrt_data, &i2s0_mxrt_config, POST_KERNEL,
		    CONFIG_I2S_INIT_PRIORITY, &i2s_mxrt_driver_api);

