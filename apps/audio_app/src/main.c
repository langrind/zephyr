/*
 * Copyright (c) 2020 Nik Langrind
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <i2s.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>

#include "str_codec.h"

static str_codec_handle s_codec_handle;

#define I2S_LABEL "FLEXIO3_I2S"

static char ping[512];
static char pong[512];

static char dbg_buf[512];
static bool dbg_flag = false;

/* command to invoke codec_init(), for debugging */
static int cmd_codec_init(const struct shell *shell, size_t argc, char **argv)
{
  int rc = codec_init(&s_codec_handle);
  if (rc) {
    printk("codec_init failed device");
    return -1;
  }
  return 0;
}

static int cmd_i2s_print(const struct shell *shell, size_t argc, char **argv)
{
  struct device *i2s_device;
  i2s_device = device_get_binding(I2S_LABEL);
  if (!i2s_device) {
    printk("unable to find device %s\n", I2S_LABEL);
    return -1;
  }
  i2s_trigger(i2s_device, I2S_DIR_RX, I2S_TRIGGER_DROP);

  if (dbg_flag) {
    for (int i = 0; i < 64; i += 3 ) {
      int x = (dbg_buf[i] << 0) + (dbg_buf[i+1] << 8) + (dbg_buf[i+2] << 16);
      printk(" %02x%02x%02x", dbg_buf[i], dbg_buf[i+1], dbg_buf[i+2]);
      printk(" : %06x", x);
      printk("\n");
    }
  }
  return 0;
}

static int cmd_codec_play(const struct shell *shell, size_t argc, char **argv)
{
  int time_secs = 10;
  if ( argc > 1 ) {
    time_secs = atoi(argv[1]);
  }
  int iters = (1000/16) * time_secs;
  for (int i = 0; i < iters; i++) {
    
    int32_t update_ret = Codec_update(CODEC_UPDATE_TX1_RX1, ping, pong);
    if (update_ret) {
      printk("Codec_update(1:%d) failed: %d\n", i, update_ret);
      break;
    }

    if (!dbg_flag) {
      dbg_flag = true;
      memcpy (dbg_buf, ping, sizeof(dbg_buf));
    }

    update_ret = Codec_update(CODEC_UPDATE_TX1_RX1, pong, ping);
    if (update_ret) {
      printk("Codec_update(2:%d) failed: %d\n", i, update_ret);
    }
  }
  return 0;
}

static int cmd_codec_send(const struct shell *shell, size_t argc, char **argv)
{
  for (int i = 0; i < 1000; i++) {
    memset(ping, 0xaa, sizeof(ping));
    int32_t update_ret = Codec_update(CODEC_UPDATE_TX1, ping);
    if (update_ret) {
      printk("Codec_update(1) failed\n");
    }

    memset(pong, 1, sizeof(pong));
    update_ret = Codec_update(CODEC_UPDATE_TX1, pong);
    if (update_ret) {
      printk("Codec_update(2) failed\n");
    }

    memset(pong, 3, sizeof(pong));
    update_ret = Codec_update(CODEC_UPDATE_TX1, pong);
    if (update_ret) {
      printk("Codec_update(3) failed: %d\n", update_ret);
    }

    memset(pong, 7, sizeof(pong));
    update_ret = Codec_update(CODEC_UPDATE_TX1, pong);
    if (update_ret) {
      printk("Codec_update(4) failed: %d\n", update_ret);
    }
  }
  return 0;
}

static int cmd_codec_receive(const struct shell *shell, size_t argc, char **argv)
{
  memset(ping, 0xaa, sizeof(ping));
  int32_t update_ret = Codec_update(CODEC_UPDATE_RX1, ping);
  if (update_ret) {
    printk("Codec_update(initial) failed\n");
  }
  else {
    for (int i = 0; i < 1000; i++) {
      update_ret = Codec_update(CODEC_UPDATE_RX1, ping);
      if (update_ret) {
	printk("Codec_update(%d) failed\n", i);
	break;
      }
    }
  }
  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE
(sub_a,
 SHELL_CMD(print, NULL, "print driver stats", cmd_i2s_print),
 SHELL_CMD(play, NULL, "route rx to tx", cmd_codec_play),
 SHELL_CMD(rx, NULL, "Invoke Codec Receive", cmd_codec_receive),
 SHELL_CMD(tx, NULL, "Invoke Codec Send", cmd_codec_send),
 SHELL_CMD(config, NULL, "Invoke Codec Init", cmd_codec_init),
 SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(a, &sub_a, "audio driver test commands", NULL);

void main(void)
{
  printk("Example Audio App. %s\n", CONFIG_BOARD );

  str_codec_handle codec_handle;
  int rc = codec_init(&codec_handle);
  if (rc) {
    printk("codec_init() failed\n");
    return;
  }

#if 0
  rc = codec_test_case_1();
  if (rc) {
    printk("codec_test_case_1 failed\n"); 
  }
  else {
    printk("codec_test_case_1 passed\n");
  }
#endif

  /* XXX FIX THIS - codec_test_case_2 i2s_writes silence which needs to
   * be done to get data flow initialized in the normal case too.
   * so add that to.... where?
   */
  rc = codec_test_case_2();
  if (rc) {
    printk("codec_test_case_2 failed\n"); 
  }
  else {
    printk("codec_test_case_2 passed\n");
  }
}
