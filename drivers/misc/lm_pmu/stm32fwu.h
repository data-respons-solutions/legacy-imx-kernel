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

#ifndef _STM32FWU_H_
#define _STM32FWU_H_

/**
 * enum stm32fwu_iface - upgrade interfaces for STM32 MCU's
 * @STM32_SPI:		SPI interface.
 * @STM32_MAX:		terminator.
 */
enum stm32fwu_iface {
	STM32_SPI,
	/* Generally for future use, i.e. UART upgrade algorithm looks pretty
	 * similar. */
	STM32_MAX
};

struct stm32fwu_fw;

struct stm32fwu_fw *stm32fwu_init(struct device *dev,
				  enum stm32fwu_iface iface, const u8 *data,
				  int len);

void stm32fwu_destroy(struct stm32fwu_fw *fw);

int stm32fwu_send_sync(struct stm32fwu_fw *fw);

int stm32fwu_update(struct stm32fwu_fw *fw);

int stm32fwu_get(struct stm32fwu_fw *fw);

int stm32fwu_get_version(struct stm32fwu_fw *fw);

#endif /* _STM32FWU_H_ */
