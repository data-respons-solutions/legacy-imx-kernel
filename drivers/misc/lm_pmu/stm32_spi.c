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
#include "stm32fwu.h"
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include "stm32_core.h"

static int stm32fwu_spi_write(struct stm32fwu_fw *fw, const u8 *buf, int len)
{
	struct spi_device *sdev = to_spi_device(fw->dev);

	struct spi_message m;
	struct spi_transfer t = {
		.len = len,
		.tx_buf = buf,
		.bits_per_word = 8,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(sdev, &m);
}

static int stm32fwu_spi_read(struct stm32fwu_fw *fw, u8 *buf, int len)
{
	struct spi_device *sdev = to_spi_device(fw->dev);

	struct spi_message m;
	struct spi_transfer t = {
		.len = len,
		.rx_buf = buf,
		.bits_per_word = 8,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(sdev, &m);
}

static int stm32fwu_spi_wait_for_ack(struct stm32fwu_fw *fw, int retires)
{
	int ret, i = 0;
	u8 *buf = &fw->buffer[STM32FWU_MAX_BUFFER_SIZE];

	while (i < retires) {
		ret = stm32fwu_spi_read(fw, buf, 1);
		if (ret < 0) {
			dev_err(fw->dev,
				"firmware upgrade wait for ack fail\n");
			return ret;
		}

		if (fw->buffer[STM32FWU_MAX_BUFFER_SIZE] == STM32FWU_ACK ||
		    fw->buffer[STM32FWU_MAX_BUFFER_SIZE] == STM32FWU_ACK2) {
			return i;
		}

		if (fw->buffer[STM32FWU_MAX_BUFFER_SIZE] == STM32FWU_NACK)
			return -EPROTO;

		usleep_range(1000, 1100);
		i++;
	}

	dev_err(fw->dev, "fw ack timeout\n");

	return -ETIMEDOUT;
}

static int stm32fwu_spi_send_cmd(struct stm32fwu_fw *fw,
				 struct stm32fwu_cmd *cmd)
{
	int ret;

	fw->buffer[0] = STM32FWU_SPI_SOF;
	fw->buffer[1] = cmd->cmd;
	fw->buffer[2] = cmd->neg_cmd;

	ret = stm32fwu_spi_write(fw, fw->buffer, 3);
	if (ret < 0) {
		dev_err(fw->dev, "fw cmd write fail\n");
		return ret;
	}

	return stm32fwu_spi_wait_for_ack(fw, cmd->timeout);
}

void stm32fwu_spi_init(struct stm32fwu_fw *fw)
{
	fw->write = stm32fwu_spi_write;
	fw->read = stm32fwu_spi_read;
	fw->wait_for_ack = stm32fwu_spi_wait_for_ack;
	fw->send_cmd = stm32fwu_spi_send_cmd;
}

