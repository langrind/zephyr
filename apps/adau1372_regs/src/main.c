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

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_sample);

#define I2S_LABEL "FLEXIO3_I2S"
#define CODEC_I2C_DEV "I2C_3"
#define CODEC_I2C_ADDR 0x3c

/* Perform the two byte address write, and N-byte read */
static int adi_read(struct device *dev, uint16_t devAddr, uint16_t startAddr, uint8_t *buf, uint32_t numBytes)
{
  uint8_t addrBuffer[2];

  addrBuffer[1] = startAddr & 0xFF;
  addrBuffer[0] = startAddr >> 8;
  return i2c_write_read(dev, devAddr, addrBuffer, sizeof(addrBuffer), buf, numBytes);
}

int adi_write(struct device *dev, uint16_t devAddr, uint16_t startAddr, uint8_t *buf, uint32_t numDataBytes)
{
  size_t   bufSize = sizeof devAddr + numDataBytes;
  uint8_t *txBuf = alloca(bufSize);

  txBuf[0] = startAddr >> 8;
  txBuf[1] = startAddr & 0xFF;
  memcpy(&txBuf[sizeof devAddr], buf, numDataBytes);

  return i2c_write(dev, txBuf, bufSize, devAddr);
}

static int cmd_i2c_read_byte(const struct shell *shell, size_t argc, char **argv)
{
  if ( argc < 2 ) {
    printk("usage: %s <hex-address>\n", argv[0] );
    return -1;
  }

  char *endptr = NULL;
  uint16_t addr = (uint16_t)strtoul(argv[1], &endptr, 16);
  if (endptr && *endptr != '\0' ) {
    printk("%s (%s) is not a valid register address\n", argv[1], endptr);
    return -1;
  }

  struct device *dev = device_get_binding(CODEC_I2C_DEV);
  if ( !dev ) {
    printk("%s: unable to find device %s\n", __FUNCTION__, CODEC_I2C_DEV);
    return -1;
  }

  uint8_t val = 0xff;

  int rc = adi_read(dev, CODEC_I2C_ADDR, addr, &val, 1);
  if (rc) {
    printk("unable to read ADI device: %d\n", rc);
    return -1;
  }

  printk("%04x: %02x\n", addr, val);
  return 0;
}

static int cmd_i2c_read_bytes(const struct shell *shell, size_t argc, char **argv)
{
  if ( argc < 3 ) {
    printk("usage: %s <hex-address> <count>\n", argv[0] );
    return -1;
  }

  char *endptr = NULL;
  uint16_t addr = (uint16_t)strtoul(argv[1], &endptr, 16);
  if (endptr && *endptr != '\0' ) {
    printk("%s (%s) is not a valid register address\n", argv[1], endptr);
    return -1;
  }

  struct device *dev = device_get_binding(CODEC_I2C_DEV);
  if ( !dev ) {
    printk("%s: unable to find device %s\n", __FUNCTION__, CODEC_I2C_DEV);
    return -1;
  }

  uint8_t len = (uint8_t)strtoul(argv[2], &endptr, 0);
  if (endptr && *endptr != '\0' ) {
    printk("%s (%s) is not a valid value\n", argv[1], endptr);
    return -1;
  }

  uint8_t *rdBuf = alloca(len);
  int rc = adi_read(dev, CODEC_I2C_ADDR, addr, rdBuf, len);
  if (rc) {
    printk("unable to read ADI device: %d\n", rc);
    return -1;
  }

  for (int i = 0; i < (int)len; i++ ) {
    printk("%04x: %02x\n", addr+i, rdBuf[i]);
  }
  return 0;
}

static int cmd_i2c_write_byte(const struct shell *shell, size_t argc, char **argv)
{
  if ( argc < 3 ) {
    printk("usage: %s <hex-address> <decimal-or-hex-value>\n", argv[0] );
    return -1;
  }

  char *endptr = NULL;
  uint16_t reg_addr = (uint16_t)strtoul(argv[1], &endptr, 16);
  if (endptr && *endptr != '\0' ) {
    printk("%s (%s) is not a valid register address\n", argv[1], endptr);
    return -1;
  }

  uint8_t val = (uint8_t)strtoul(argv[2], &endptr, 0);
  if (endptr && *endptr != '\0' ) {
    printk("%s (%s) is not a valid value\n", argv[1], endptr);
    return -1;
  }

  struct device *dev = device_get_binding(CODEC_I2C_DEV);
  if ( !dev ) {
    printk("%s: unable to find device %s\n", __FUNCTION__, CODEC_I2C_DEV);
    return -1;
  }

  int rc = adi_write(dev, CODEC_I2C_ADDR, reg_addr, &val, 1);
  if (rc) {
    printk("write failed\n");
    return -1;
  }
  return 0;
}

/* hack to pull in test routine */
int i2s_mxrt_test_transfer(struct device *dev, uint8_t *txData, size_t size);

static int cmd_i2s_xfer_test(const struct shell *shell, size_t argc, char **argv)
{
  struct device *i2s_device;
  i2s_device = device_get_binding(I2S_LABEL);
  if (!i2s_device) {
    printk("unable to find " I2S_LABEL " device");
    return -1;
  }

#if 1
  /* this results in 0x332211, 0x665544, 0x998877, 0xCCBBAA being sent on I2S
   * so it seems like if you pack 24 bit words, you get what you want.
   */
  uint8_t txData[] = { 0x11, 0x22, 0x33,
		       0x44, 0x55, 0x66,
		       0x77, 0x88, 0x99,
		       0xAA, 0xBB, 0xCC,
  };
#else
  /* this results in 0x001122, 0x003344, 0x005566, 0x007788 being sent on I2S
   * so I need to understand how to configure and use the interace properly
   */
  uint32_t txData[] = { 0x00112200,
			0x00334400,
			0x00556600,
			0x00778800,
  };
#endif

  int rc = i2s_mxrt_test_transfer(i2s_device, txData, sizeof(txData));
  (void)rc;
  return 0;
}

int i2s_mxrt_test_receive(struct device *dev, uint8_t *rxData, size_t size);

static int cmd_i2s_recv_test(const struct shell *shell, size_t argc, char **argv)
{
  struct device *i2s_device;
  i2s_device = device_get_binding(I2S_LABEL);
  if (!i2s_device) {
    printk("unable to find " I2S_LABEL " device");
    return -1;
  }

  uint8_t rxData[16];
  int rc = i2s_mxrt_test_receive(i2s_device, rxData, 12);

#define GET24(x) ( (*((x) + 0) << 16) + (*((x) + 1) << 8) + (*((x) + 2) << 0))
  //#define GET24(x) ( (*((x) + 2) << 16) + (*((x) + 1) << 8) + (*((x) + 0) << 0))

  uint8_t *p = rxData;
  uint32_t sample = GET24(p);
  printk("%x ", sample);

  p += 3;
  sample = GET24(p);
  printk("%x ", sample);

  p += 3;
  sample = GET24(p);
  printk("%x ", sample);

  p = rxData;
  for (int i = 0; i < 12; i++ ) {
    printk ("%02x ", *p);
    p++;
  }
  printk ("\n");
  return 0;
}
SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2ct,
			       SHELL_CMD(rd, NULL, "read byte", cmd_i2c_read_byte),
			       SHELL_CMD(rr, NULL, "read bytes", cmd_i2c_read_bytes),
			       SHELL_CMD(wr, NULL, "write_byte", cmd_i2c_write_byte),
			       SHELL_CMD(xfer, NULL, "test send", cmd_i2s_xfer_test),
			       SHELL_CMD(recv, NULL, "test recv", cmd_i2s_recv_test),
			       SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(i2ct, &sub_i2ct, "i2c test commands", NULL);

void main(void)
{
  printk("ADAU1372 Register Read/Write Utility. %s\n", CONFIG_BOARD);
}
