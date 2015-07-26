/*
 *  Copyright (C) 2014, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include "stm32fwu.h"
#include "stm32_core.h"
#include "stm32_spi.c"

static inline int stm32fwu_wait_for_ack(struct stm32fwu_fw *fw, int retries)
{
	return fw->wait_for_ack(fw, retries);
}

static inline int stm32fwu_send_cmd(struct stm32fwu_fw *fw,
				    struct stm32fwu_cmd *cmd)
{
	return fw->send_cmd(fw, cmd);
}

static inline int stm32fwu_write(struct stm32fwu_fw *fw, const u8 *buf, int len)
{
	return fw->write(fw, buf, len);
}

static inline int stm32fwu_read(struct stm32fwu_fw *fw, u8 *buf, int len)
{
	return fw->read(fw, buf, len);
}

static int send_addr(struct stm32fwu_fw *fw, u32 addr)
{
	int ret;
	u8 *buffer = fw->buffer;

	buffer[0] = (addr >> 24) & 0xFF;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	buffer[4] = buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3];

	ret = stm32fwu_write(fw, buffer, STM32FWU_ADDR_LEN);
	if (ret < 0)
		return ret;

	return stm32fwu_wait_for_ack(fw, STM32FWU_COMMON_COUNT);
}

static int fwu_write(struct stm32fwu_fw *fw, u32 addr, const u8 *buffer,
		     int len)
{
	int ret, i;
	u8 *sbuf = fw->buffer;
	u8 xor = 0;
	struct stm32fwu_cmd cmd = {
		.cmd = STM32FWU_WRITE_MEM_CMD,
		.neg_cmd = ~STM32FWU_WRITE_MEM_CMD,
		.timeout = STM32FWU_CMD_COUNT,
	};

	if (len > STM32FWU_MAX_TRANSFER_SIZE) {
		dev_err(fw->dev, "More than 256 bytes per transfer\n");
		return -EINVAL;
	}

	ret = stm32fwu_send_cmd(fw, &cmd);
	if (ret < 0)
		return ret;

	ret = send_addr(fw, addr);
	if (ret < 0)
		return ret;

	/* the same buffer is used for commands and data so the order really
	 * matters here */
	memcpy(&sbuf[1], buffer, len);

	/* just in case - check if smaller chunks are 16-bit aligned */
	if (len  < STM32FWU_MAX_TRANSFER_SIZE) {
		if (len & 0x1)
			sbuf[++len] = 0xFF;
	}

	sbuf[0] = len - 1;

	/* compute checksum */
	for (i = 0; i < len + 1; ++i)
		xor ^= sbuf[i];
	sbuf[len + 1] = xor;

	/* NOTE AN4286
	 * in some conditions the master has to wait 1 ms here */
	usleep_range(1000, 1100);

	ret = stm32fwu_write(fw, sbuf, len + 2);
	if (ret < 0)
		return ret;

	return stm32fwu_wait_for_ack(fw, STM32FWU_COMMON_COUNT);
}

static int fwu_erase(struct stm32fwu_fw *fw)
{
	int ret;
	u8 *sbuf = fw->buffer;
	struct stm32fwu_cmd cmd = {
		.cmd = STM32FWU_ERASE_CMD,
		.neg_cmd = ~STM32FWU_ERASE_CMD,
		.timeout = STM32FWU_CMD_COUNT,
	};

	ret = stm32fwu_send_cmd(fw, &cmd);
	if (ret < 0)
		return ret;

	/* global erase */
	sbuf[0] = 0xFF;
	sbuf[1] = 0xFF;
	sbuf[2] = 0x00;

	ret = stm32fwu_write(fw, sbuf, STM32FWU_ERASE_CMD_LEN);
	if (ret < 0)
		return ret;

	return stm32fwu_wait_for_ack(fw, STM32FWU_ERASE_COUNT);
}

/**
 * stm32fwu_get() - gets bootloader information frame
 * @fw:		fw object.
 *
 * Read bootloader information:
 *
 * Byte  1 bootloader version (0 < version < 255), example: 0x10 = version 1.0.
 *
 * Byte  2 0x00 (Get command)
 *
 * Byte  3 0x01 (Get Version)
 *
 * Byte  4 0x02 (Get ID)
 *
 * Byte  5 0x11 (Read Memory command)
 *
 * Byte  6 0x21 (Go command)
 *
 * Byte  7 0x31 (Write Memory command)
 *
 * Byte  8 0x44 (Erase command)
 *
 * Byte  9 0x63 (Write Protect command)
 *
 * Byte 10 0x73 (Write Unprotect command)
 *
 * Byte 11 0x82 (Readout Protect command)
 *
 * Byte 12 0x92 (Readout Unprotect command)
 *
 * Return:	read byte count or error code.
 */
int stm32fwu_get(struct stm32fwu_fw *fw)
{
	int ret, count = 0;

	struct stm32fwu_cmd cmd = {
		.cmd = STM32FWU_GET_CMD,
		.neg_cmd = ~STM32FWU_GET_CMD,
		.timeout = STM32FWU_CMD_COUNT,
	};

	ret = stm32fwu_send_cmd(fw, &cmd);
	if (ret < 0)
		return ret;

	ret = stm32fwu_read(fw, &fw->buffer[STM32FWU_MAX_BUFFER_SIZE], 1);
	if (ret < 0)
		return ret;

	count = fw->buffer[STM32FWU_MAX_BUFFER_SIZE];
	if (count >= STM32FWU_MAX_TRANSFER_SIZE)
		return -EINVAL;

	ret = stm32fwu_read(fw, &fw->buffer[STM32FWU_MAX_BUFFER_SIZE], count);
	if (ret < 0)
		return ret;

	ret = stm32fwu_wait_for_ack(fw, STM32FWU_COMMON_COUNT);
	if (ret < 0)
		return ret;

	return count;
}
EXPORT_SYMBOL(stm32fwu_get);

/**
 * stm32fwu_get() - gets bootloader information frame
 * @fw:		fw object.
 *
 * Return: < 0 - error code, any positive value means success.
 */
int stm32fwu_get_version(struct stm32fwu_fw *fw)
{
	int ret;

	struct stm32fwu_cmd cmd = {
		.cmd = STM32FWU_GET_VERSION_CMD,
		.neg_cmd = ~STM32FWU_GET_VERSION_CMD,
		.timeout = STM32FWU_CMD_COUNT,
	};

	ret = stm32fwu_send_cmd(fw, &cmd);
	if (ret < 0)
		return ret;

	ret = stm32fwu_read(fw, &fw->buffer[STM32FWU_MAX_BUFFER_SIZE], 1);
	if (ret < 0)
		return ret;

	ret = stm32fwu_wait_for_ack(fw, STM32FWU_COMMON_COUNT);
	if (ret < 0)
		return ret;

	return fw->buffer[STM32FWU_MAX_BUFFER_SIZE];
}
EXPORT_SYMBOL(stm32fwu_get_version);

/**
 * stm32fwu_send_sync() - sends bootloader synchronization frame
 * @fw:	fw object.
 *
 * It is used during SPI upgrade.
 *
 * Return:	if succeed the number of ack waiting cycles or error code.
 */
int stm32fwu_send_sync(struct stm32fwu_fw *fw)
{
	int ret;

	dev_info(fw->dev, "send sync byte for upgrade\n");

	fw->buffer[0] = STM32FWU_SPI_SOF;

	ret = stm32fwu_write(fw, fw->buffer, 1);
	if (ret < 0)
		return ret;

	return  stm32fwu_wait_for_ack(fw, STM32FWU_CMD_COUNT);
}
EXPORT_SYMBOL(stm32fwu_send_sync);

/**
 * stm32fwu_update() - runs all firmware update work
 * @fw:		fw object.
 *
 * Return:	status code < 0 if error.
 */
int stm32fwu_update(struct stm32fwu_fw *fw)
{
	int ret = 0, remaining;
	u32 pos = 0;
	u32 fw_addr = STM32FWU_APP_ADDR;
	int block = STM32FWU_MAX_TRANSFER_SIZE;
	int count = 0, err_count = 0, retry_count = 0;

	struct stm32fwu_cmd cmd = {
		.cmd = STM32FWU_GO_ADDR_CMD,
		.neg_cmd = ~STM32FWU_GO_ADDR_CMD,
		.timeout = 1000,
	};

	dev_info(fw->dev, "%s start\n", __func__);

	ret = fwu_erase(fw);
	if (ret < 0) {
		dev_err(fw->dev, "%s, fw_erase_stm failed %d\n", __func__, ret);
		return ret;
	}

	remaining = fw->fw_len;

	while (remaining > 0) {
		if (block > remaining)
			block = remaining;

		while (retry_count < 3) {
			ret = fwu_write(fw, fw_addr, fw->fw_data + pos, block);
			if (ret < 0) {
				dev_err(fw->dev,
					"Returned %d writing to addr 0x%08X\n",
					ret, fw_addr);
				retry_count++;
				err_count++;
				continue;
			}
			retry_count = 0;
			break;
		}

		if (ret  < 0) {
			dev_err(fw->dev,
				"Writing MEM failed: %d, retry cont: %d\n",
				ret, err_count);
			return ret;
		}

		remaining -= block;
		pos += block;
		fw_addr += block;
		if (count++ == 20) {
			dev_info(fw->dev, "Updated %u bytes / %u bytes\n",
				 pos, fw->fw_len);
			count = 0;
		}
	}

	dev_info(fw->dev,
		 "Firmware download success.(%d bytes, retry %d)\n", pos,
		 err_count);

	/* STM : GO USER ADDR */
	ret = stm32fwu_send_cmd(fw, &cmd);
	if (ret < 0)
		return ret;

	return send_addr(fw,  STM32FWU_APP_ADDR);
}
EXPORT_SYMBOL(stm32fwu_update);

/**
 * stm32fwu_init() - creates firmware upgrade instance
 * @dev:	Pointer to device.
 * @iface:	Interface type.
 * @data:	Pointer to firmware data.
 * @len:	Length of firmware data in bytes.
 *
 * It allocates the place for fwu structure and transfer buffer so it should be
 * deinitialized by stm32fw_destroy(). Firmware upgrade is simple operation
 * and it was assumed that it will be called form one context so there is no
 * locking for fwu functions.
 *
 * Return:	Pointer to firmware upgrade structure.
 */
struct stm32fwu_fw *stm32fwu_init(struct device *dev,
				  enum stm32fwu_iface iface, const u8 *data,
				  int len)
{
	struct stm32fwu_fw *fw;

	if (iface >= STM32_MAX || iface < 0) {
		dev_err(dev, "wrong iface type\n");
		return NULL;
	}

	fw = devm_kzalloc(dev, sizeof(*fw), GFP_KERNEL);
	if (fw == NULL)
		return NULL;

	/* 2 times for tx and rx buffer */
	fw->buffer = devm_kzalloc(dev, 2 * STM32FWU_MAX_BUFFER_SIZE,
			     GFP_KERNEL | GFP_DMA);
	if (fw->buffer == NULL) {
		kfree(fw);
		return NULL;
	}

	fw->dev = dev;
	fw->fw_len = len;
	fw->fw_data = data;

	switch (iface) {
	case STM32_SPI:
		stm32fwu_spi_init(fw);
		break;
	default:
		pr_err("wrong interface\n");
		goto _err;
	}

	return fw;
_err:
	kfree(fw->buffer);
	kfree(fw);
	return NULL;
}
EXPORT_SYMBOL(stm32fwu_init);

/**
 * stm32fwu_destroy() - cleans up fwu structure and buffer.
 * @fw:		Pointer to fwu structure.
 */
void stm32fwu_destroy(struct stm32fwu_fw *fw)
{
	devm_kfree(fw->dev, fw->buffer);
	devm_kfree(fw->dev, fw);
}
EXPORT_SYMBOL(stm32fwu_destroy);

MODULE_AUTHOR("Karol Wrona <k.wrona@samsung.com>");
MODULE_DESCRIPTION("STM32 upgrade protocol");
MODULE_LICENSE("GPL");



