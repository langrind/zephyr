/*
 * Copyright (c) 2020 Nik Langrind
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <i2c.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <audio/codec.h>

#define CODEC_LABEL "ADAU1372"
#define I2S_LABEL "FLEXIO3_I2S"

static int cmd_test_codec_binding(const struct shell *shell, size_t argc, char **argv)
{
  struct device *codec_device;
  codec_device = device_get_binding(CODEC_LABEL);
  if (!codec_device) {
    printk("unable to find " CODEC_LABEL " device");
    return -1;
  }

  /* configure codec */
  struct audio_codec_cfg codec_cfg;
  codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
  //codec_cfg.dai_cfg.i2s = i2s_cfg;
  codec_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
  codec_cfg.dai_cfg.i2s.mem_slab = NULL;
  // should define soc_get_ref_clk_freq() in soc/arm/nxp_imx/rt/soc.c ?
  // codec_cfg.mclk_freq = soc_get_ref_clk_freq();
  codec_cfg.mclk_freq = 600000000;

  int rc = audio_codec_configure(codec_device, &codec_cfg);
  printk("audio_codec_configure returned %d\n", rc);
  return 0;
}

static int cmd_test_i2s_binding(const struct shell *shell, size_t argc, char **argv)
{
  struct device *i2s_device;
  i2s_device = device_get_binding(I2S_LABEL);
  if (!i2s_device) {
    printk("unable to find " I2S_LABEL " device");
    return -1;
  }

  return 0;
}
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_binding,
			       SHELL_CMD(1,  NULL, "test codec binding", cmd_test_codec_binding),
			       SHELL_CMD(2,  NULL, "test i2s binding", cmd_test_i2s_binding),
			       SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(bind, &sub_test_binding, "binding test commands", NULL);

