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

#define EXAMPLE_DMAMUX DMAMUX
#define EXAMPLE_DMA DMA0
#define EXAMPLE_TX_CHANNEL 1U
#define EXAMPLE_RX_CHANNEL 0U
#define EXAMPLE_TX_DMA_SOURCE kDmaRequestMuxFlexIO2Request0Request1
#define EXAMPLE_RX_DMA_SOURCE kDmaRequestMuxFlexIO2Request2Request3

/* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM)
 * fref * (32 + 100,000,000/130,222,463)
 * fref * 32.76791667
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32U,         /*!< PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1U,          /*!< Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 0x05F5E100U, /*!< 30 bit numerator of fractional loop divider.*/
    .denominator = 0x07C3097FU, /*!< 30 bit denominator of fractional loop divider */
};

static int i2s_mxrt_configure(struct device *dev, enum i2s_dir dir,
			      struct i2s_config *i2s_cfg)
{
	const struct i2s_mxrt_cfg *const cfg = DEV_CFG(dev);
	struct i2s_mxrt_data *const data = DEV_DATA(dev);

	(void)cfg;
	(void)data;

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
  struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);
  (void)dev_data;
  return 0;
}

static int i2s_mxrt_write(struct device *dev, void *mem_block, size_t size)
{
  struct i2s_mxrt_data *const data = DEV_DATA(dev);
  (void)data;
  return 0;
}

static int i2s_mxrt_trigger(struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
  return 0;
}

/*
 * This routine is a bunch of DMA init stuff copied from the MCUX I2S DMA
 * example
 */
static int i2s_mxrt_init_dma(const struct i2s_mxrt_cfg *const dev_cfg,
			     struct i2s_mxrt_data *const dev_data)
{
    /* Create EDMA handle */
    /*
     * dmaConfig.enableRoundRobinArbitration = false;
     * dmaConfig.enableHaltOnError = true;
     * dmaConfig.enableContinuousLinkMode = false;
     * dmaConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&dev_data->dmaConfig);
    EDMA_Init(DMA0, &dev_data->dmaConfig);
    EDMA_CreateHandle(&dev_data->txDmaHandle, DMA0, EXAMPLE_TX_CHANNEL);
    EDMA_CreateHandle(&dev_data->rxDmaHandle, DMA0, EXAMPLE_RX_CHANNEL);

    dev_data->dmamuxBase = EXAMPLE_DMAMUX;

    DMAMUX_Init(dev_data->dmamuxBase);
    DMAMUX_SetSource(dev_data->dmamuxBase, EXAMPLE_TX_CHANNEL, EXAMPLE_TX_DMA_SOURCE);
    DMAMUX_EnableChannel(dev_data->dmamuxBase, EXAMPLE_TX_CHANNEL);
    DMAMUX_SetSource(dev_data->dmamuxBase, EXAMPLE_RX_CHANNEL, EXAMPLE_RX_DMA_SOURCE);
    DMAMUX_EnableChannel(dev_data->dmamuxBase, EXAMPLE_RX_CHANNEL);
    return 0;
}

/* Select Audio PLL (786.43 MHz) as flexio clock source, need to sync with sai clock, or the codec may not work */
#define DEMO_FLEXIO_CLKSRC_SEL (0U)
/* Clock pre divider for flexio clock source */
#define DEMO_FLEXIO_CLKSRC_PRE_DIV (7U)
/* Clock divider for flexio clock source */
#define DEMO_FLEXIO_CLKSRC_DIV (7U)
#define DEMO_FLEXIO_CLK_FREQ \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_FLEXIO_CLKSRC_PRE_DIV + 1U) / (DEMO_FLEXIO_CLKSRC_DIV + 1U))

#define OVER_SAMPLE_RATE (384)
#define BUFFER_SIZE (256)
#define BUFFER_NUM (4)
#define PLAY_COUNT (5000 * 2U)
#define ZERO_BUFFER_SIZE (BUFFER_SIZE * 2)
/* demo audio sample rate */
//#define DEMO_AUDIO_SAMPLE_RATE (kFLEXIO_I2S_SampleRate16KHz)
#define DEMO_AUDIO_SAMPLE_RATE (kFLEXIO_I2S_SampleRate44100Hz)
/* demo audio master clock */
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
#define DEMO_AUDIO_MASTER_CLOCK OVER_SAMPLE_RATE *DEMO_AUDIO_SAMPLE_RATE
#else
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ
#endif
/* demo audio data channel */
#define DEMO_AUDIO_DATA_CHANNEL (2U)
/* demo audio bit width - this sets the frame width, not the number of bits in the frame that get used */
#define DEMO_AUDIO_BIT_WIDTH (kFLEXIO_I2S_WordWidth32bits)

flexio_i2s_handle_t txHandle      = {0};
flexio_i2s_handle_t rxHandle      = {0};

static void txCallback(FLEXIO_I2S_Type *i2sBase, flexio_i2s_handle_t *handle, status_t status, void *userData)
{
#if 0
    if ((emptyBlock < BUFFER_NUM) && (!isZeroBuffer))
    {
        emptyBlock++;
        sendCount++;
    }

    if (isZeroBuffer)
    {
        isZeroBuffer = false;
    }

    if (sendCount == beginCount)
    {
        isTxFinished = true;
    }
#endif
}

static void rxCallback(FLEXIO_I2S_Type *i2sBase, flexio_i2s_handle_t *handle, status_t status, void *userData)
{
#if 0
    if (emptyBlock > 0)
    {
        emptyBlock--;
        receiveCount++;
    }

    if (receiveCount == beginCount)
    {
        isRxFinished = true;
    }
#endif
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

AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM], 4);
AT_NONCACHEABLE_SECTION_ALIGN_INIT(static uint8_t zeroBuff[ZERO_BUFFER_SIZE], 4) = {0};

static int i2s_mxrt_initialize(struct device *dev)
{
	printk("%s\n", __FUNCTION__);

	const struct i2s_mxrt_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2s_mxrt_data *const dev_data = DEV_DATA(dev);

	CLOCK_InitAudioPll(&audioPllConfig);

	/* This clock configuration was cribbed from MCUX SDK example 
	 * "evkmimxrt1064_flexio3_i2s_interrupt_transfer". I don't know
	 * enough about MXRT to know why this sets up Flexio2Mux instead
	 * of flexio3Mux.
	 */
	CLOCK_SetMux(kCLOCK_Flexio2Mux, DEMO_FLEXIO_CLKSRC_SEL);
	CLOCK_SetDiv(kCLOCK_Flexio2PreDiv, DEMO_FLEXIO_CLKSRC_PRE_DIV);
	CLOCK_SetDiv(kCLOCK_Flexio2Div, DEMO_FLEXIO_CLKSRC_DIV);

	//uint32_t dbg = DEMO_FLEXIO_CLK_FREQ;
	//printk("%u\n", dbg);

	flexio_i2s_config_t config;
	FLEXIO_I2S_GetDefaultConfig(&config);
	FLEXIO_I2S_Type base = dev_cfg->mcux_base;
	FLEXIO_I2S_Init(&base, &config);
	dev_data->flexioBase = base.flexioBase;

	/* Configure the audio format */
	flexio_i2s_format_t format;
	format.bitWidth      = DEMO_AUDIO_BIT_WIDTH;
	format.sampleRate_Hz = DEMO_AUDIO_SAMPLE_RATE;

	FLEXIO_I2S_TransferTxCreateHandle(&base, &txHandle, txCallback, NULL);
	FLEXIO_I2S_TransferRxCreateHandle(&base, &rxHandle, rxCallback, NULL);

	/* Set audio format for tx and rx. DEMO_FLEXIO_CLK_FREQ works out to about 12.288 Mhz (12,287,968) */
	FLEXIO_I2S_TransferSetFormat(&base, &txHandle, &format, DEMO_FLEXIO_CLK_FREQ);
	FLEXIO_I2S_TransferSetFormat(&base, &rxHandle, &format, DEMO_FLEXIO_CLK_FREQ);
#if 0
	//emptyBlock = BUFFER_NUM;
	//beginCount = PLAY_COUNT;

	memset (zeroBuff, 0xaa, ZERO_BUFFER_SIZE);
	memset (audioBuff, 0xaa, BUFFER_SIZE * BUFFER_NUM);

	/* send zero buffer fistly to make sure RX data is put into TX queue */
	flexio_i2s_transfer_t txXfer, rxXfer;
	txXfer.dataSize = ZERO_BUFFER_SIZE;
	txXfer.data     = zeroBuff;
	//FLEXIO_I2S_TransferSendNonBlocking(&base, &txHandle, &txXfer);

	FLEXIO_I2S_WriteBlocking(&base, 24, zeroBuff, 8);
#endif
#if 0
	int rc = i2s_mxrt_init_dma(dev_cfg, dev_data);
	if (rc) {
		printk("i2s_mxrt_init_dma failed\n");
		return -EINVAL;
	}
	//FLEXIO_I2S_TransferTxCreateHandleEDMA
#endif

#if 0
	data->dev_dma = device_get_binding(CONFIG_I2S_CAVS_DMA_NAME);
	if (!dev_data->dev_dma) {
		LOG_ERR("%s device not found", CONFIG_I2S_CAVS_DMA_NAME);
		return -ENODEV;
	}

	/* Initialize the buffer queues */
	k_msgq_init(&dev_data->tx.in_queue, (char *)dev_data->tx.in_msgs,
			sizeof(void *), I2S_CAVS_BUF_Q_LEN);
	k_msgq_init(&dev_data->rx.in_queue, (char *)dev_data->rx.in_msgs,
			sizeof(void *), I2S_CAVS_BUF_Q_LEN);
	k_msgq_init(&dev_data->tx.out_queue, (char *)dev_data->tx.out_msgs,
			sizeof(void *), I2S_CAVS_BUF_Q_LEN);
	k_msgq_init(&dev_data->rx.out_queue, (char *)dev_data->rx.out_msgs,
			sizeof(void *), I2S_CAVS_BUF_Q_LEN);

	/* register ISR */
	dev_cfg->irq_connect();

	dev_data->tx.state = I2S_STATE_NOT_READY;
	dev_data->rx.state = I2S_STATE_NOT_READY;

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

//FLEXIO_I2S_Type base;


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
