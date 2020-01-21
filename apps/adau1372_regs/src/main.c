
/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <i2s.h>
#include <i2c.h>
#include <audio/codec.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_sample);

#define AUDIO_SAMPLE_FREQ		(48000)
#define AUDIO_SAMPLES_PER_CH_PER_FRAME	(64)
#define AUDIO_NUM_CHANNELS		(2)
#define AUDIO_SAMPLES_PER_FRAME		\
	(AUDIO_SAMPLES_PER_CH_PER_FRAME * AUDIO_NUM_CHANNELS)
#define AUDIO_SAMPLE_BYTES		(4)
#define AUDIO_SAMPLE_BIT_WIDTH		(32)

#define AUDIO_FRAME_BUF_BYTES		\
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)

#define I2S_PLAYBACK_DEV		"I2S_1"
#define I2S_HOST_DEV			"I2S_2"

#define I2S_PLAY_BUF_COUNT		(2)
#define I2S_TX_PRELOAD_BUF_COUNT	(2)

#define BASE_TONE_FREQ_HZ		1046.502 /* Hz */
#define FLOAT_VALUE_OF_2PI		(2 * 3.1415926535897932384626433832795)
#define BASE_TONE_FREQ_RAD		(BASE_TONE_FREQ_HZ * FLOAT_VALUE_OF_2PI)
#define BASE_TONE_PHASE_DELTA		(BASE_TONE_FREQ_RAD / AUDIO_SAMPLE_FREQ)

#define SECONDS_PER_TONE		(1) /* second(s) */
#define TONES_TO_PLAY			\
	{0, 2, 4, 5, 7, 9, 10, 12, 12, 10, 9, 7, 5, 4, 2, 0}

#define AUDIO_FRAMES_PER_SECOND		\
	(AUDIO_SAMPLE_FREQ / AUDIO_SAMPLES_PER_CH_PER_FRAME)
#define AUDIO_FRAMES_PER_TONE_DURATION	\
	(SECONDS_PER_TONE * AUDIO_FRAMES_PER_SECOND)

#define SIGNAL_AMPLITUDE_DBFS		(-36)
#define SIGNAL_AMPLITUDE_BITS		(31 + (SIGNAL_AMPLITUDE_DBFS / 6))
#define SIGNAL_AMPLITUDE_SCALE		(1 << SIGNAL_AMPLITUDE_BITS)

#ifdef AUDIO_PLAY_FROM_HOST
#define APP_MODE_STRING			"host playback"
#else
#define APP_MODE_STRING			"tone playback"
#endif

#define CODEC_I2C_DEV "I2C_3"

static struct k_mem_slab i2s_mem_slab;
__attribute__((section(".dma_buffers")))
static char audio_buffers[AUDIO_FRAME_BUF_BYTES][I2S_PLAY_BUF_COUNT];
static struct device *spk_i2s_dev;
static struct device *host_i2s_dev;
static struct device *codec_device;

static void i2s_audio_init(void)
{
	int    ret;
	struct i2s_config i2s_cfg;
	struct audio_codec_cfg codec_cfg;

	k_mem_slab_init(&i2s_mem_slab, audio_buffers, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);

	spk_i2s_dev = device_get_binding(I2S_PLAYBACK_DEV);

	if (!spk_i2s_dev) {
		LOG_ERR("unable to find " I2S_PLAYBACK_DEV " device");
		return;
	}

	host_i2s_dev = device_get_binding(I2S_HOST_DEV);

	if (!host_i2s_dev) {
		LOG_ERR("unable to find " I2S_HOST_DEV " device");
		return;
	}

	/* In theory I could use "GPIO_5" here and it would at succeed */
#if 0
	codec_device = device_get_binding(DT_INST_0_TI_TLV320DAC_LABEL);
	if (!codec_device) {
		LOG_ERR("unable to find " DT_INST_0_TI_TLV320DAC_LABEL " device");
		return;
	}
#else
	codec_device = device_get_binding("GPIO_5");
	if (!codec_device) {
		LOG_ERR("unable to find GPIO_5 device");
		return;
	}
#endif
	/* configure i2s for audio playback */
	i2s_cfg.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = AUDIO_NUM_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_CLK_NF_NB;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER |
		I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_cfg.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_cfg.mem_slab = &i2s_mem_slab;

	/* make the transmit interface non-blocking */
	i2s_cfg.timeout = K_NO_WAIT;
	ret = i2s_configure(spk_i2s_dev, I2S_DIR_TX, &i2s_cfg);
	if (ret != 0) {
		LOG_ERR("dmic_configure failed with %d error", ret);
		return;
	}

	/* make the receive interface blocking */
	i2s_cfg.timeout = K_FOREVER;
	ret = i2s_configure(host_i2s_dev, I2S_DIR_RX, &i2s_cfg);
	if (ret != 0) {
		LOG_ERR("dmic_configure failed with %d error", ret);
		return;
	}

	/* configure codec */
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S,
	codec_cfg.dai_cfg.i2s = i2s_cfg;
	codec_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_SLAVE |
				I2S_OPT_BIT_CLK_SLAVE;
	codec_cfg.dai_cfg.i2s.mem_slab = NULL;

	/* soc_get_ref_clk_freq() is only provided by the interl_s1000 soc.c code */
	/* codec_cfg.mclk_freq = soc_get_ref_clk_freq(); */
	/* So will need to fix this at some point, using random value for now */
	codec_cfg.mclk_freq = 1000000;

	audio_codec_configure(codec_device, &codec_cfg);
}

static int cmd_i2s_init(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	i2s_audio_init();

	return 0;
}

static int cmd_i2s_params(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "argc = %d", argc);
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "  argv[%d] = %s", cnt, argv[cnt]);
	}

	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2s,
	SHELL_CMD(params, NULL, "Print params command.", cmd_i2s_params),
	SHELL_CMD(init, NULL, "Init command.", cmd_i2s_init),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(i2s, &sub_i2s, "i2s commands", NULL);

static int i2c_init()
{
  struct device *dev = device_get_binding(CODEC_I2C_DEV);
  if ( !dev ) {
    printk("unable to find device %s\n", CODEC_I2C_DEV);
    return -1;
  }

  uint8_t tx_buf[1] = { 1 };

  int rc = i2c_write(dev, tx_buf, sizeof(tx_buf), 0x02 );
  if ( rc) {
    printk("unable to write device %s: %d\n", CODEC_I2C_DEV, rc);
    return -1;
  }
  else {
    printk("i2c write successful\n");
  }
  return 0;
}

static int adi_read(struct device *dev, uint16_t dev_addr,
		    uint16_t start_addr, uint8_t *buf, uint32_t num_bytes)
{
	uint8_t addr_buffer[2];

	addr_buffer[1] = start_addr & 0xFF;
	addr_buffer[0] = start_addr >> 8;
	return i2c_write_read(dev, dev_addr, addr_buffer, sizeof(addr_buffer),
			      buf, num_bytes);
}

static int i2c_read_test_1()
{
  struct device *dev = device_get_binding(CODEC_I2C_DEV);
  if ( !dev ) {
    printk("%s: unable to find device %s\n", __FUNCTION__, CODEC_I2C_DEV);
    return -1;
  }

  uint8_t rd_buf[1] = { 0xff };
  int rc = adi_read(dev, 0x3c, 0, rd_buf, (uint32_t)(sizeof rd_buf));
  if ( rc) {
    printk("unable to read ADI device: %d\n", rc);
    return -1;
  }
  else {
    printk("read %d from ADI reg 0x00\n", (int)rd_buf[0]);
  }
  return 0;
}

static int i2c_read_test_2()
{
  struct device *dev = device_get_binding(CODEC_I2C_DEV);
  if ( !dev ) {
    printk("%s: unable to find device %s\n", __FUNCTION__, CODEC_I2C_DEV);
    return -1;
  }

  uint8_t rd_buf[1] = { 0xff };
  int rc = adi_read(dev, 0x3c, 0x11, rd_buf, (uint32_t)(sizeof rd_buf));
  if ( rc) {
    printk("unable to read ADI device: %d\n", rc);
    return -1;
  }
  else {
    printk("read %d from ADI reg 0x11\n", (int)rd_buf[0]);
  }
  return 0;
}

static int cmd_i2c_init(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = i2c_init();
	if ( rc ) {
	  printk("init of device %s failed\n", CODEC_I2C_DEV);
	}
	else {
	  printk("init of device %s succeeded\n", CODEC_I2C_DEV);
	}
	return 0;
}

static int cmd_i2c_read0(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = i2c_read_test_1();
	if ( rc ) {
	  printk("i2c_read_test_1() device failed\n");
	}
	else {
	  printk("i2c_read_test_1() device succeeded\n");
	}
	return 0;
}
static int cmd_i2c_read11(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = i2c_read_test_2();
	if ( rc ) {
	  printk("i2c_read_test_2() device failed\n");
	}
	else {
	  printk("i2c_read_test_2() device succeeded\n");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2c,
	SHELL_CMD(init, NULL, "Init command.", cmd_i2c_init),
	SHELL_CMD(rd0, NULL, "read reg 0.", cmd_i2c_read0),
	SHELL_CMD(rd11, NULL, "read reg 0x11.", cmd_i2c_read11),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(i2c, &sub_i2c, "i2c commands", NULL);

void main(void)
{
  printk("Hello World! %s\n", CONFIG_BOARD);
}
