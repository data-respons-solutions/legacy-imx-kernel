/*
  $Id$
  Copyright 2010-2016 DataRespons

  This program is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License version 2
  as published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spi/spi.h>

#include "linux/platform_data/ad7327.h"

#include <linux/of.h>
#include <linux/of_device.h>

static ad7327_platform_data *ad7327_driver_data = NULL;

static int ad7327_spi_transfer (struct spi_device *spi, const u16 command, u16 *reply);
static int ad7327_setup (ad7327_config_t *config);
static int ad7327_probe(struct spi_device *spi);
static int ad7327_remove(struct spi_device *spi);


static const ad7327_config_t ad7327_config[] =
{
	{
		.chip = 0,
		.name = "ad7327",
		.nchan = ARRAY_SIZE(ad7327_channels),
		.channels = ad7327_channels,
		.last = 7,
		.params = {

			.mode 	= ADC_MODE_8_SINGLE_END,
			.pm 	= ADC_PM_AUTO_NORMAL,
			.coding = ADC_CODING_BINARY,
			.ref 	= ADC_REF_INTERNAL,
		},
		.range = {
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
				ADC_RANGE_UNI_10V,
			},
	}
};

/************************************************************************
 *
 *	ad7327_read
 *
 * 	Read next sample from AD7327 chip
 *
 ************************************************************************/
static int ad7327_read (int channel, uint32_t *sample)
{
	ad7327_platform_data *inst = ad7327_driver_data;
	int status = 0;
	int expected_channel;
	uint16_t control = 0;
	uint16_t reply = 0;

	if (inst == NULL)
		return -EINVAL;

	status = mutex_lock_interruptible(&inst->mlock);
	if (status < 0)
	{
	   pr_err ("mutex_lock_interruptible failed with status %d\n", status);
	   return -ERESTARTSYS;
    }

	expected_channel = channel;
	inst->convert_chan = expected_channel;

	control = inst->control | CR_ADD(inst->convert_chan);

	status = ad7327_spi_transfer(inst->spi, control, &reply);
	if (status < 0)
	{
		pr_err ("spi_write_then_read failed with status %d\n", status);
	}
	else
	{
		u16 actual_channel;
		/*
		 * second spi transaction to stop the sequence
		 */
		status = ad7327_spi_transfer(inst->spi, inst->control, &reply);
		if (status < 0)
		{
			pr_err ("spi_write_then_read failed with status %d\n", status);
		}

		actual_channel = reply >> 14;
		*sample = (reply & 0x3FFF) >> 1;

		/*
		 * Using ADC_RANGE_UNI_10V where      0  > sample > 4095 (ADC_RANGE_12BIT_RES)
		 * fix for channel where   			4095 > sample > 8096
		 */
		if(*sample >= ADC_RANGE_12BIT_RES)
		{
			*sample = *sample - ADC_RANGE_12BIT_RES;
		}

		/*pr_err ("-------- line: %d | function :%s | inst->convert_chan = %d | actual_channel = %d | expected_channel = %d | sample: %d\n"
				, __LINE__, __FUNCTION__, inst->convert_chan, actual_channel, expected_channel, *sample);*/
	}
	mutex_unlock(&inst->mlock);

	return status;
}

static int ad7327_read_raw(struct iio_dev *indio_dev,
                           struct iio_chan_spec const *channel, int *val,
                           int *val2, long mask)
{
        int ret;
        uint32_t sample;

        switch (mask) {
        case IIO_CHAN_INFO_RAW:
                ret = ad7327_read(channel->channel, &sample);
                if (ret < 0)
                        return ret;

                *val = sample;	// sample as a return value
                return IIO_VAL_INT;

        default:
                return -EINVAL;
        }

}


/************************************************************************
 *
 *	ad7327_setup
 *
 * 	Set new control and range parameters for AD7327 chip
 *
 ************************************************************************/
static int ad7327_setup (ad7327_config_t *config)
{
	ad7327_platform_data *inst = ad7327_driver_data;
	int err = 0;
	struct adc_params *params = &config->params;
	u16 reply;
	u16 control;

	if (inst == NULL)
		return -EINVAL;

	err = mutex_lock_interruptible(&inst->mlock);
	if (err < 0)
	{
	   pr_err ("mutex_lock_interruptible failed with status %d\n", err);
	   return -ERESTARTSYS;
	}
	err = 0;

#if 0
	pr_err ("ad7327_setup: inst=0x%p\n", inst);
	pr_err ("params: mode=%d pm=%d coding=%d ref=%d\n", params->mode, params->pm, params->coding, params->ref);
	pr_err ("range: %d:%d:%d:%d %d:%d:%d:%d \n", 
			config->range[0], config->range[1], config->range[2], config->range[3],
			config->range[4], config->range[5], config->range[6], config->range[7] );
#endif

	/* Accept only Single Ended mode!! */
	if (params->mode != ADC_MODE_8_SINGLE_END)
		params->mode = ADC_MODE_8_SINGLE_END;

	/* Accept only Normal Mode! */
	if (params->pm != ADC_PM_AUTO_NORMAL)
		params->pm = ADC_PM_AUTO_NORMAL;

#if 0
	/* Accept only 2's Complement Coding! */
	if (params->coding != ADC_CODING_2S_COMPLEMENT)
		params->coding = ADC_CODING_2S_COMPLEMENT;
#else
	/* Accept only straight binary Coding! */
	if (params->coding != ADC_CODING_BINARY)
		params->coding = ADC_CODING_BINARY;
#endif

	inst->control 	= CONTROL_REG | (params->mode << 8) | (params->pm << 6) | (params->coding << 5) | (params->ref << 4) | CR_SEQ_NOT_USED_0 | CR_ZERO;
	inst->sequence 	= SEQUENCE_REG;
	inst->range1 	= RANGE1_REG | RR1_VIN0 (config->range [0]) | RR1_VIN1 (config->range [1]) | RR1_VIN2 (config->range [2]) | RR1_VIN3 (config->range [3]);
	inst->range2 	= RANGE2_REG | RR2_VIN4 (config->range [4]) | RR2_VIN5 (config->range [5]) | RR2_VIN6 (config->range [6]) | RR2_VIN7 (config->range [7]);

//	pr_err ("control: %04X sequence: %04X range1=%04X range2=%04X\n", inst->control, inst->sequence, inst->range1, inst->range2);

	/* Write to Range Register 1 */
	err = ad7327_spi_transfer(inst->spi, inst->range1, &reply);
	if (err < 0)
		return err;

	/* Write to Range Register 2 */
	err = ad7327_spi_transfer(inst->spi, inst->range2, &reply);
	if (err < 0)
		return err;

	/* Write to Sequence Register */
	err = ad7327_spi_transfer(inst->spi, inst->sequence, &reply);
	if (err < 0)
		return err;

	/* Write to Control Register */
	control = inst->control | CR_ADD (inst->convert_chan);
	err = ad7327_spi_transfer(inst->spi, control, &reply);
	if (err < 0)
		return err;

	mutex_unlock(&inst->mlock);

	return err;
}


/* portable code must never pass more than 32 bytes */
#define	SPI_BUFSIZ	max(32,SMP_CACHE_BYTES)

static u8 *spi_buf_prealloc = NULL;


/************************************************************************
 *
 *	ad7327_spi_transfer
 *
 * 	Start new SPI transfer for AD7327 chip
 *
 ************************************************************************/

static int ad7327_spi_transfer (struct spi_device *spi, const u16 command, u16 *reply)
{
	static DEFINE_MUTEX(lock);

	int status = -1;
	struct spi_message message;
	struct spi_transfer	x;
	u8 *local_buf;

	//pr_err ("ad7327_spi_transfer: command=0x%04X\n", command);

	/* Use preallocated DMA-safe buffer.  We can't avoid copying here,
	 * (as a pure convenience thing), but we can keep heap costs
	 * out of the hot path ...
	 */
	spi_message_init(&message);
	memset(&x, 0, sizeof x);
	x.len = 2;

	message.is_dma_mapped = 0;

	spi_message_add_tail(&x, &message);

	/* ... unless someone else is using the pre-allocated buffer */
	if (!mutex_trylock(&lock))
	{
		local_buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
		if (NULL == local_buf)
			return -ENOMEM;
	}
	else
	{
		local_buf = spi_buf_prealloc;
	}

	local_buf [0] = command >> 8;
	local_buf [1] = command;

	x.tx_buf = &local_buf [0];
	x.rx_buf = &local_buf [2];

	/* do the i/o */
	status = spi_sync(spi, &message);
	if (status == 0)
	{
		*reply = (u16)(local_buf [2] << 8) | local_buf [3];
	}

	//pr_err ("ad7327_spi_transfer: DIN=0x%02X%02X DOUT=0x%02X%02X\n", 
	//		local_buf[0], local_buf[1], local_buf[2], local_buf[3]);

	if (x.tx_buf == spi_buf_prealloc)
		mutex_unlock(&lock);
	else
		kfree(local_buf);

	//pr_err ("ad7327_spi_transfer: reply=0x%04X\n", *reply);

	return status;
}

/************************************************************************
 *
 *	ad7327_info
 *
 *  AD7327 iio info
 *
 ************************************************************************/

static const struct iio_info ad7327_info = {
        .read_raw = ad7327_read_raw,
        .driver_module = THIS_MODULE,
};

/************************************************************************
 *
 *	ad7327_probe
 *
 * 	Probe AD7327 chip
 *
 ************************************************************************/

static int ad7327_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	ad7327_platform_data *inst = NULL;
	int err;
	int chip;
	int ret;

	//pr_err ("Inside ad7327_probe (%s)\n", spi->dev.driver->name);

	/* Locate ADC chip from SPI index - there is only one SPI chip -> 0 */
	chip = 0;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*inst));
	if (!indio_dev)
		return -ENOMEM;

	inst = iio_priv(indio_dev);

	/* Define SPI transfer mode.
	 * NOTE: Slave SCLK must be inverted in HW!!
	 */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_2;

	err = spi_setup(spi);
	if (err < 0)
	{
		pr_err ("spi_setup failed with error %d\n", err);
		kfree (spi_buf_prealloc);
		spi_buf_prealloc = NULL;
//		mutex_unlock(&inst->mlock);
		return err;
	}

	spi_set_drvdata(spi, indio_dev);
	inst->spi = spi;

	pr_err ("Found AD7327\n");

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi->dev.driver->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad7327_info;

	indio_dev->channels = ad7327_config[chip].channels;
	indio_dev->num_channels = ad7327_config[chip].nchan;

	spi_buf_prealloc = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
	if (NULL == spi_buf_prealloc)
		return -ENOMEM;

	mutex_init(&inst->mlock);

	/* Start convertion on first channel */
	inst->convert_chan = 0;

	ad7327_driver_data = inst;

	/* Copy default configuration */
	memcpy (&inst->config, ad7327_config, sizeof (ad7327_config_t));
	ad7327_setup (&inst->config);

	ret = iio_device_register(indio_dev);

	return ret;

}

static int ad7327_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	kfree(spi_buf_prealloc);
	spi_buf_prealloc = NULL;
	ad7327_driver_data = NULL;

	return 0;
}

static const struct spi_device_id ad7327_id[] = {
	{"ad7327", 0},
	{ }
};

MODULE_DEVICE_TABLE(spi, ad7327_id);

#ifdef CONFIG_OF
static const struct of_device_id ad7327_of_match[] = {
	{ .compatible = "ti,ad7327" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ad7327_of_match);
#endif

static struct spi_driver ad7327_spi_driver =
{
		.driver = {
			.name		= "ad7327",
			.bus		= &spi_bus_type,
			.owner		= THIS_MODULE,
			.of_match_table = of_match_ptr(ad7327_of_match),
		},

		.probe		= ad7327_probe,
		.remove		= ad7327_remove,
};
module_spi_driver(ad7327_spi_driver);


MODULE_AUTHOR("Stig Bjoergvik <sb@datarespons.no> / Artur Siewierski <asi@datarespons.no>");
MODULE_DESCRIPTION("AD7327 ADC Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ad7327");

