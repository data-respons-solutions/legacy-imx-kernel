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

#ifndef _STM32_CORE_H_
#define _STM32_CORE_H_

#define STM32FWU_ADDR_LEN		5
#define STM32FWU_ERASE_CMD_LEN		3

/* Protocol */
#define STM32FWU_SPI_SOF		0x5A
#define STM32FWU_ACK			0x79
#define STM32FWU_ACK2			0xF9
#define STM32FWU_NACK			0x1F

#define STM32FWU_MAX_TRANSFER_SIZE	256
#define STM32FWU_MAX_BUFFER_SIZE	260

#define STM32FWU_APP_ADDR		0x08000000

/* Retries counts */
#define STM32FWU_ERASE_COUNT		7000
#define STM32FWU_CMD_COUNT		30
#define STM32FWU_COMMON_COUNT		20

/* Commands */
#define STM32FWU_WRITE_MEM_CMD		0x31
#define STM32FWU_GO_ADDR_CMD		0x21
#define STM32FWU_ERASE_CMD		0x44
#define STM32FWU_GET_CMD		0x00
#define STM32FWU_GET_VERSION_CMD	0x01

/**
 * struct stm32fwu_cmd - Upgrade command
 * @cmd:	Fwu command.
 * @neg_cmd:	Bit negation of Fwu command used xor'ed in STM.
 * @timeout:	Number of retries.
 */
struct stm32fwu_cmd {
	u8 cmd;
	u8 neg_cmd;
	int timeout;
};

/**
 * struct stm32fwu_fw - Generic representation for STM32 fw upgrade
 * @dev:		Pointer to device.
 * @wait_for_ack:	ACK callback.
 * @send_cmd:		Command callback.
 * @write:		Write callback.
 * @read:		Read callback.
 * @fw_data:		Pointer to firmware binary.
 * @fw_len:		The length of fw bin.
 * @buffer:		Pointer to SPI buffer: should be DMA safe
 */
struct stm32fwu_fw {
	int (*wait_for_ack)(struct stm32fwu_fw *fw, int retries);
	int (*send_cmd)(struct stm32fwu_fw *fw, struct stm32fwu_cmd *cmd);
	int (*write)(struct stm32fwu_fw *fw, const u8 *buf, int len);
	int (*read)(struct stm32fwu_fw *fw, u8 *buf, int len);
	const u8 *fw_data;
	int fw_len;
	u8 *buffer;
	struct device *dev;
};

void stm32fwu_spi_init(struct stm32fwu_fw *fw);

#endif /*_STM32_CORE_H_ */

