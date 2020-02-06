#include <zephyr.h>
#include <drivers/i2s.h>
#include <audio/codec.h>

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include "str_codec.h"

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(audio_app);

#define I2S_LABEL "FLEXIO3_I2S"

#define AUDIO_SAMPLE_FREQ		(8000)
#define AUDIO_SAMPLES_PER_CH_PER_FRAME	(64)     /* XXX shorten this up! Make it configurable */
#define AUDIO_NUM_CHANNELS		(2)
#define AUDIO_SAMPLES_PER_FRAME		\
	(AUDIO_SAMPLES_PER_CH_PER_FRAME * AUDIO_NUM_CHANNELS)
#define AUDIO_SAMPLE_BYTES		(4)
#define AUDIO_SAMPLE_BIT_WIDTH		(32)
#define AUDIO_FRAME_BUF_BYTES (AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)
#define I2S_PLAY_BUF_COUNT    (6)

//static struct k_mem_slab i2s_mem_slab;
//__attribute__((section(".dma_buffers")))

static char audio_buffers[AUDIO_FRAME_BUF_BYTES+1][I2S_PLAY_BUF_COUNT];

struct str_codec {
  struct i2s_config        i2s_cfg;
  struct audio_codec_cfg   codec_cfg;
  struct k_mem_slab        i2s_mem_slab;
  struct device            *i2s_device;
  struct device            *codec_device;
  bool                     tx_started;
  bool                     rx_started;
};

static struct str_codec s_codec;

int codec_init(str_codec_handle * pHandle)
{
  printk("%s\n", __FUNCTION__);

  memset(&s_codec, 0, sizeof(s_codec));
  *pHandle = NULL; 

  struct str_codec *psc = &s_codec;
 
  char * tmp = &audio_buffers[0][0];
  tmp += 3;
  tmp = (char *)(0xfffffffc & (intptr_t)tmp); 
  int ret = k_mem_slab_init(&psc->i2s_mem_slab, tmp, AUDIO_FRAME_BUF_BYTES, I2S_PLAY_BUF_COUNT);
  if (ret) {
    printk("%s: k_mem_slab_init failed %d\n", __FUNCTION__, ret);
    return -1;
  }
    
  psc->i2s_device = device_get_binding(I2S_LABEL);
  if (!psc->i2s_device) {
    printk("%s: unable to find device %s\n", __FUNCTION__, I2S_LABEL);
    return -1;
  }

  psc->codec_device = device_get_binding(DT_INST_0_ADI_ADAU1372_LABEL);
  if (!psc->codec_device) {
    printk("%s: unable to find device %s\n", __FUNCTION__, DT_INST_0_ADI_ADAU1372_LABEL);
    return -1;
  }

  /* configure i2s for audio playback */
  psc->i2s_cfg.word_size = AUDIO_SAMPLE_BIT_WIDTH;
  psc->i2s_cfg.channels = AUDIO_NUM_CHANNELS;
  psc->i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_CLK_NF_NB;
  psc->i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
  psc->i2s_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;
  psc->i2s_cfg.block_size = AUDIO_FRAME_BUF_BYTES;
  psc->i2s_cfg.mem_slab = &psc->i2s_mem_slab;

  /* make the transmit interface blocking */
  psc->i2s_cfg.timeout = 2 * 6; /* 2 milliseconds - was K_NO_WAIT; */
  ret = i2s_configure(psc->i2s_device, I2S_DIR_TX, &psc->i2s_cfg);
  if (ret != 0) {
    LOG_ERR("i2s_configure(TX) failed with %d error", ret);
    return -1;
  }

  /* make the receive interface non-blocking too XXX they share one timeout need to fix that! */
  //psc->i2s_cfg.timeout = K_NO_WAIT;
  ret = i2s_configure(psc->i2s_device, I2S_DIR_RX, &psc->i2s_cfg);
  if (ret != 0) {
    LOG_ERR("i2s_configure(RX) failed with %d error", ret);
    return -1;
  }

  /* configure codec - this needs to be made more completed */
  struct audio_codec_cfg codec_cfg;
  //codec_cfg.dai_cfg.i2s = psc->i2s_cfg;
  codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
  codec_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
  codec_cfg.dai_cfg.i2s.mem_slab = NULL;
  // should define soc_get_ref_clk_freq() in soc/arm/nxp_imx/rt/soc.c ?
  // codec_cfg.mclk_freq = soc_get_ref_clk_freq();
  codec_cfg.mclk_freq = 12288000;

  int rc = audio_codec_configure(psc->codec_device, &codec_cfg);
  printk("audio_codec_configure returned %d\n", rc);

  return 0;
}

/* Post-init test cases prior to setting up any transfers.
 * Reads and writes should fail with expected error codes.
 * Can't actually run these tests anymore, because states
 * are now = I2S_STATE_READY
 *
 */
int codec_test_case_1()
{
  struct str_codec *psc = &s_codec;
  void *buffer;
  int ret;

  ret = k_mem_slab_alloc(&psc->i2s_mem_slab, &buffer, K_NO_WAIT);
  if (ret) {
    LOG_ERR("buffer alloc failed %d", ret);
    return -1;
  }

  /* fill the buffer with zeros (silence) */
  memset(buffer, 0, AUDIO_FRAME_BUF_BYTES);

  ret = i2s_write(psc->i2s_device, buffer, AUDIO_FRAME_BUF_BYTES);
  if (ret != -EIO) {
    printk("%s: expected error code %d from i2s_write\n", __FUNCTION__, -EIO);
    if (ret) {
      k_mem_slab_free(&psc->i2s_mem_slab, &buffer);
    }
    return -1;
  }

  size_t size;
  void *rbuf = NULL;
  ret = i2s_read(psc->i2s_device, &rbuf, &size);
  if (ret != -EIO) {
    printk("%s: expected error code %d from i2s_read\n", __FUNCTION__, -EIO);
    return -1;
  }
  return 0;
  
}

int codec_test_case_2()
{
  struct str_codec *psc = &s_codec;
  void *buffer;
  int ret;

  ret = k_mem_slab_alloc(&psc->i2s_mem_slab, &buffer, K_NO_WAIT);
  if (ret) {
    LOG_ERR("buffer alloc failed %d", ret);
    return -1;
  }

  /* fill the buffer with zeros (silence) */
  memset(buffer, 0x00, AUDIO_FRAME_BUF_BYTES);

  ret = i2s_write(psc->i2s_device, buffer, AUDIO_FRAME_BUF_BYTES);
  if (ret) {
    printk("%s: expected i2s_write to succeed\n", __FUNCTION__);
    k_mem_slab_free(&psc->i2s_mem_slab, &buffer);
    return -1;
  }
#if 0
  size_t size;
  void *rbuf = NULL;
  ret = i2s_read(psc->i2s_device, &rbuf, &size);
  if (ret != -EAGAIN) {
    printk("%s: expected error code %d from i2s_read got %d\n", __FUNCTION__, -EAGAIN, ret);
    return -1;
  }
#endif
  return 0;  
}

/* Routines to split/merge between the user's separate stereo channel
 * buffers and the I2S driver's unified buffers
 */

/* Read 24 bits out of buffer into top 24 bits of 32 bit word */
#define GET24PUT32(x) ( (*((x) + 2) << 24) + (*((x) + 1) << 16) + (*((x) + 0) << 8))
#define WRITE_SAMPLE(output, off, samp) ( *(uint32_t *)((output) + (off)) = samp )

static void Codec_merge(char *tx_output_buffer, size_t tx_out_nbytes,
			char *tx_left_ptr, char *tx_right_ptr)
{
  size_t n_output = 0;
  while (n_output < tx_out_nbytes) {
    uint32_t sample;
    if (tx_left_ptr) {
      sample = GET24PUT32(tx_left_ptr);
      tx_left_ptr += 3;
    }
    else {
      sample = 0;
    }
    WRITE_SAMPLE(tx_output_buffer, n_output, sample);
    n_output += sizeof(sample);

    if (tx_right_ptr) {
      sample = GET24PUT32(tx_right_ptr);
      tx_right_ptr += 3;
    } 
    else {
      sample = 0;
    }
    WRITE_SAMPLE(tx_output_buffer, n_output, sample);
    n_output += sizeof(sample);
  }
} 

static void Codec_split(char *rx_input_buffer, size_t rx_in_nbytes,
			char *rx_left_ptr, char *rx_right_ptr)
{
  size_t n_read = 0;
  while (n_read < rx_in_nbytes) {
    if (rx_left_ptr) {
      n_read++;
      *rx_left_ptr++ = rx_input_buffer[n_read++];
      *rx_left_ptr++ = rx_input_buffer[n_read++];
      *rx_left_ptr++ = rx_input_buffer[n_read++];
    }
    else {
      n_read += 4;
    }

    if (rx_right_ptr) {
      n_read++;      
      *rx_right_ptr++ = rx_input_buffer[n_read++];
      *rx_right_ptr++ = rx_input_buffer[n_read++];
      *rx_right_ptr++ = rx_input_buffer[n_read++];
    }
    else {
      n_read += 4;
    }
  }
}


/*
 * This call can block
 *
 * First we create a multiplexed buffer that is twice as long
 * as the input TX buffers and then fill it by interleaving
 * the left and right buffers. While doing this, we also insert
 * the dead byte (one of 4 bytes on I2S is dead if we have 24-bit
 * samples)
 *
 * Then we have two things to do (1) dequeue RX buffers and
 * demux into the user's receive buffers (the reverse of the last paragraph)
 * and (2) send the transmit buffer.
 *
 * This routine only supports transmit at the moment
 *
 * The user's buffers are 24-bit samples packed with no padding
 *
 * eventually this needs to grab a semaphore that will cause it to block
 * until the previous queued update cycle id finished
 */
int32_t Codec_update(enum CODEC_UPDATE_TYPE type, ...)
{
  struct str_codec *psc = &s_codec;

  va_list ap;
  va_start(ap, type);

  char *tx_left_ptr = NULL;
  char *tx_right_ptr = NULL;
  char *rx_left_ptr = NULL;
  char *rx_right_ptr = NULL;

  bool txing = false;
  bool rxing = false;

  switch (type) {
  case CODEC_UPDATE_ALL:
    txing = true;
    rxing = true;
    rx_left_ptr = va_arg(ap, void *);
    rx_right_ptr = va_arg(ap, void *);
    tx_left_ptr = va_arg(ap, void *);
    tx_right_ptr = va_arg(ap, void *);
    break;
  case CODEC_UPDATE_TX1:
    txing = true;
    tx_left_ptr = va_arg(ap, void *);
    break;
  case CODEC_UPDATE_RX1:
    rxing = true;
    rx_left_ptr = va_arg(ap, void *);
    break;
  case CODEC_UPDATE_TX2:
    txing = true;
    tx_right_ptr = va_arg(ap, void *);
    break;
  case CODEC_UPDATE_RX2:
    rxing = true;
    rx_right_ptr = va_arg(ap, void *);
    break;
  case CODEC_UPDATE_TX1_RX1:
    rxing = true;
    txing = true;
    rx_left_ptr = va_arg(ap, void *);
    tx_left_ptr = va_arg(ap, void *);
    break;
  case CODEC_UPDATE_TX2_RX2:
    rxing = true;
    txing = true;
    rx_right_ptr = va_arg(ap, void *);
    tx_right_ptr = va_arg(ap, void *);
    break;
  default:
    printk("%s: Invalid type: %d\n", __FUNCTION__, type);
    return -1;
  }

  if (txing) {
    void *tx_output_buffer;
    int ret = k_mem_slab_alloc(&psc->i2s_mem_slab, &tx_output_buffer, K_NO_WAIT);
    if (ret) {
      LOG_ERR("buffer alloc failed %d", ret);
      return -3;
    }

    Codec_merge(tx_output_buffer, AUDIO_FRAME_BUF_BYTES, tx_left_ptr, tx_right_ptr);
		   
    ret = i2s_write(psc->i2s_device, tx_output_buffer, AUDIO_FRAME_BUF_BYTES);
    if (ret) {
      printk("%s: i2s_write failed %d\n", __FUNCTION__, ret);
      k_mem_slab_free(&psc->i2s_mem_slab, &tx_output_buffer);
      return -4;
    }

    if (!psc->tx_started) {
      /* start i2s */
      ret = i2s_trigger(psc->i2s_device, I2S_DIR_TX, I2S_TRIGGER_START);
      if (ret) {
	LOG_ERR("TX start failed. code %d", ret);
      }
      psc->tx_started = true;
    }
  }

  if (rxing) {
    void *in_buf;
    size_t size;
    int ret;

    if (!psc->rx_started) {
      /* start i2s */
      ret = i2s_trigger(psc->i2s_device, I2S_DIR_RX, I2S_TRIGGER_START);
      if (ret) {
	LOG_ERR("RX start failed. code %d", ret);
      }
      psc->rx_started = true;
    }

    /* read from host I2S interface */
    ret = i2s_read(psc->i2s_device, &in_buf, &size);
    if (ret) {
      LOG_ERR("%s: i2s_read failed %d", __FUNCTION__, ret);
      return -3;
    }
    else {
      Codec_split(in_buf, AUDIO_FRAME_BUF_BYTES, rx_left_ptr, rx_right_ptr);
      k_mem_slab_free(&psc->i2s_mem_slab, &in_buf);      
    }
  }
  return 0;
}
