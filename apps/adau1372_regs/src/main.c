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

SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2ct,
			       SHELL_CMD(rd, NULL, "read byte", cmd_i2c_read_byte),
			       SHELL_CMD(rr, NULL, "read bytes", cmd_i2c_read_bytes),
			       SHELL_CMD(wr, NULL, "write_byte", cmd_i2c_write_byte),
			       SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(i2ct, &sub_i2ct, "i2c test commands", NULL);

void main(void)
{
  printk("ADAU1372 Register Read/Write Utility. %s\n", CONFIG_BOARD);
}
