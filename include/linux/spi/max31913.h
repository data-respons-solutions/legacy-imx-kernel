/*
 * MAX31913 kernel header
 */

#ifndef _LINUX_SPI_MAX31913_H_
#define _LINUX_SPI_MAX31913_H_

#include <linux/types.h>
#include <linux/device.h>

#define MAX31913_CHANNELS	7

struct max31913 {
	struct spi_device	*spi;
	struct mutex	lock;

	unsigned char		tx_buff[1];
	unsigned char		rx_buff[1];
};


#endif /* _LINUX_SPI_MAX31913_H_ */
