/*
 * AD7327 kernel header
 */

#ifndef _LINUX_PLATFORM_DATA_AD7327_H_
#define _LINUX_PLATFORM_DATA_AD7327_H_

#include <linux/types.h>
#include <linux/device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AD7327_VERSION			"1.0.0"

#define MAX_AD7327_CHANNELS		8
#define MAX_AD7327_NAME_LEN		32

#define AD7327_TX_BUF_SIZE	2
#define AD7327_RX_BUF_SIZE	2

/*==============================================================================================================*/
/* ADDRESSING REGISTERS				 																			*/
/*==============================================================================================================*/
/* A serial transfer on the AD7327 consists of 16 SCLK cycles. The three MSBs on the DIN line during the 16 	*/
/* SCLK transfer are decoded to determine which register is addressed. The three MSBs consist of the write bit,	*/
/* Register Select 1 bit, and Register Select 2 bit. The register select bits are used to determine which of 	*/
/* the four on-board registers is selected. The write bit determines if the data on the DIN line following the	*/
/* register select bits loads into the addressed register. If the write bit is 1, the bits load into the 		*/
/* register addressed by the register select bits. If the write bit is 0, the data on the DIN line does not 	*/
/* load into any register. 																						*/
/*==============================================================================================================*/

#define	IGNORE_DIN		0x0000
/* Data on the DIN line during this serial transfer is ignored. */

#define CONTROL_REG		0x8000
/* This combination selects the control register.The subsequent 12 bits are loaded into the control register. */

#define RANGE1_REG		0xC000
/* This combination selects Range Register 1. The subsequent 8 bits are loaded into Range Register 1. */

#define RANGE2_REG		0xA000
/* This combination selects Range Register 2. The subsequent 8 bits are loaded into Range Register 2. */

#define SEQUENCE_REG	0xE000
/* This combination selects the sequence register. The subsequent 8 bits are loaded into the sequence register.	*/

/*==============================================================================================================*/
/* CONTROL REGISTER								 																*/
/*==============================================================================================================*/
/* The control register is used to select the analog input channel, analog input configuration, reference, 		*/
/* coding, and power mode. The control register is a write-only, 12-bit register. Data loaded on the DIN line	*/
/* corresponds to the AD7327 configuration for the next conversion. If the sequence register is being used, 	*/
/* data should be loaded into the control register after the range registers and the sequence register have 	*/
/* been initialized. The bit functions of the control register are shown in Table 9 (the power-up status of all	*/
/* bits is 0). 																									*/
/*==============================================================================================================*/


/*--------------------------------------------------------------------------------------------------------------*/
/* Analog Input Channel Select 																					*/
/*--------------------------------------------------------------------------------------------------------------*/
#define	CR_ADD(x)		(((x)&0x07) << 10)	
/* These three channel address bits are used to select the analog input channel for the next conversion if the 	*/
/* sequencer is not being used. If the sequencer is being  used, the three channel address bits are used to 	*/
/* select the final channel in a consecutive sequence.															*/

/*--------------------------------------------------------------------------------------------------------------*/
/* Analog Input Mode bits 																						*/
/*--------------------------------------------------------------------------------------------------------------*/
/* These two mode bits are used to select the configuration of the eight analog input pins, VIN0 to VIN7. 		*/
/* These pins are used in conjunction with the channel address bits. On the AD7327, the analog inputs can be 	*/
/* configured as eight single-ended inputs, four fully differential inputs, four pseudo differential inputs, or	*/
/* seven pseudo differential inputs (see Table 10).																*/
/*--------------------------------------------------------------------------------------------------------------*/
#define	CR_MODE_7_PSEUDO_DIFF		(0x03<<8)		/* 7 Pseudo Differential Inputs */
#define	CR_MODE_4_FULLY_DIFF		(0x02<<8)		/* 4 Fully Differential Inputs 	*/
#define	CR_MODE_4_PSEUDO_DIFF		(0x01<<8)		/* 7 Pseudo Differential Inputs */
#define	CR_MODE_8_SINGLE_END		(0x00<<8)		/* 8 Single-Ended Inputs 		*/

/*--------------------------------------------------------------------------------------------------------------*/
/* Power Mode Selection																							*/
/*--------------------------------------------------------------------------------------------------------------*/
/* The power management bits are used to select different power mode options on the AD7327 (see Table 11).		*/

#define	CR_PM_FULL_SHUTDOWN			(0x03<<6)
/* Full Shutdown Mode. In this mode, all internal circuitry on the AD7327 is powered down. Information in the	*/
/* control register is retained when the AD7327 is in full shutdown mode. 										*/

#define	CR_PM_AUTO_SHUTDOWN			(0x02<<6)
/* Autoshutdown Mode. The AD7327 enters autoshutdown on the 15th SCLK rising edge when the control register is	*/
/* updated. All internal circuitry is powered down in autoshutdown. 											*/

#define	CR_PM_AUTO_STANDBY			(0x01<<6)
/* Autostandby Mode. In this mode, all internal circuitry is powered down, excluding the internal reference. 	*/
/* The AD7327 enters autostandby mode on the 15th SCLK rising edge after the control register is updated. 		*/

#define	CR_PM_AUTO_NORMAL			(0x00<<6)
/* Normal Mode. All internal circuitry is powered up at all times.  											*/

/*--------------------------------------------------------------------------------------------------------------*/
/* Output Coding Select																							*/
/*--------------------------------------------------------------------------------------------------------------*/
/* This bit is used to select the type of output coding the AD7327 uses for the next conversion result. 		*/
/* If coding = 0, the output coding is twos complement.  If coding = 1, the output coding is straight binary. 	*/
/* When operating in sequence mode, the output coding for each channel is the value written to the coding bit 	*/
/* during the last write to the control register.																*/
#define	CR_CODING_2S_COMPLEMENT		(0x00<<5)
#define	CR_CODING_BINARY			(0x01<<5)

/*--------------------------------------------------------------------------------------------------------------*/
/* External/Internal Reference Select																							*/
/*--------------------------------------------------------------------------------------------------------------*/
/* The reference bit is used to enable or disable the internal reference. If Ref = 0, the external reference 	*/
/* is enabled and used for the next conversion and the internal reference is disabled. If Ref = 1, the internal	*/
/* reference is used for the next conversion. When operating in sequence mode, the reference used for each 		*/
/* channel is the value written to	 the Ref bit during the last write to the control register. 				*/
#define	CR_REF_EXTERNAL			(0x00<<4)
#define	CR_REF_INTERNAL			(0x01<<4)

/*--------------------------------------------------------------------------------------------------------------*/
/* Sequencer Selection																							*/
/*--------------------------------------------------------------------------------------------------------------*/
/* The Sequence 1 and Sequence 2 bits are used to control the operation of the sequencer (see Table 12). 		*/

#define	CR_SEQ_NOT_USED_3			(0x03<<2)
/* The channel sequencer is not used. The analog channel, selected by programming the ADD2 bit to ADD0 bit in	*/
/* the control register, selects the next channel for conversion. 												*/

#define	CR_SEQ_USE_CTRL_REG			(0x02<<2)
/* Used in conjunction with the channel address bits in the control register. This allows continuous 			*/
/* conversions on a consecutive sequence of channels, from Channel 0 up to and including a final channel 		*/
/* selected by the channel address bits in the control register. The range for each channel defaults to the		*/
/* range previously written into the corresponding range register. 												*/

#define	CR_SEQ_USE_SEQ_REG			(0x01<<2)
/* Uses the sequence of channels previously programmed into the sequence register for conversion. The AD7327 	*/
/* starts converting on the lowest channel in the sequence. The channels are converted in ascending order. 		*/
/* If uninterrupted, the AD7327 keeps converting the sequence. The range for each channel defaults to the range	*/
/* previously written into the corresponding range register. 													*/

#define	CR_SEQ_NOT_USED_0			(0x00<<2)
/* The channel sequencer is not used. The analog channel, selected by programming the ADD2 bit to ADD0 bit in	*/
/* the control register, selects the next channel for conversion. 												*/


/*--------------------------------------------------------------------------------------------------------------*/
/* Zero Bits																									*/
/*--------------------------------------------------------------------------------------------------------------*/
#define	CR_ZERO						(0x00<<1)   	/* A 0 should be written to this bit at all times.			*/


/*==============================================================================================================*/
/* SEQUENCE REGISTER							 																*/
/*==============================================================================================================*/
/* The sequence register on the AD7327 is an 8-bit, write-only register. Each of the eight analog input 		*/
/* channels has one corresponding bit in the sequence register. To select an analog input channel for inclusion	*/
/* in the sequence, set the corresponding channel bit to 1 in the sequence register.							*/
/*==============================================================================================================*/
#define	SR_VIN0					(0x01<<12)	
#define	SR_VIN1					(0x01<<11)	
#define	SR_VIN2					(0x01<<10)	
#define	SR_VIN3					(0x01<<9)	
#define	SR_VIN4					(0x01<<8)	
#define	SR_VIN5					(0x01<<7)	
#define	SR_VIN6					(0x01<<6)	
#define	SR_VIN7					(0x01<<5)	
                                                                                                            
/*==============================================================================================================*/
/* RANGE REGISTERS							 																	*/
/*==============================================================================================================*/
/* The range registers are used to select one analog input range per analog input channel. 						*/
/*==============================================================================================================*/

#define	VIN_BIP_10V			0x03	/* This combination selects the ±10 V input range on VINx. 			*/
#define	VIN_BIP_5V			0x02	/* This combination selects the ± 5 V input range on VINx. 			*/
#define	VIN_BIP_2_5V		0x01	/* This combination selects the ±2.5V input range on VINx. 			*/
#define	VIN_UNI_10V			0x00	/* This combination selects the 0 V to +10 V input range on VINx 	*/

/*==============================================================================================================*/
/* RANGE REGISTERS 1							 																*/
/*==============================================================================================================*/
/* Range Register 1 is used to set the ranges for Channel 0 to Channel 3. It is an 8-bit, write-only register 	*/
/* with two dedicated range bits for each of the analog input channels from Channel 0 to Channel 3. 			*/
/* There are four analog input ranges, ±10 V, ±5 V, ±2.5 V, and 0 V to +10 V. A write to Range Register 1 is 	*/
/* selected by setting the write bit to 1 and the register select bits to 0 and 1. After the initial write to	*/
/* Range Register 1 occurs, each time an analog input is selected, the AD7327 automatically configures the		*/
/* analog input to the appropriate range, as indicated by Range Register 1. The ±10 V input range is selected	*/
/* by default on each analog input channel (see Table 13).														*/
/*==============================================================================================================*/
#define	RR1_VIN0(x)				(((x)&0x03)<<11)
#define	RR1_VIN1(x)				(((x)&0x03)<<9)
#define	RR1_VIN2(x)				(((x)&0x03)<<7)
#define	RR1_VIN3(x)				(((x)&0x03)<<5)


/*==============================================================================================================*/
/* RANGE REGISTERS 2							 																*/
/*==============================================================================================================*/
/* Range Register 2 is used to set the ranges for Channel 4 to Channel 7. It is an 8-bit, write-only register	*/
/* with two dedicated range bits for each of the analog input channels from Channel 4 to Channel 7. 			*/
/* There are four analog input ranges, ±10 V, ±5 V, ±2.5 V, and 0 V to +10 V. After the initial write to Range	*/
/* Register 2 occurs, each time an analog input is selected, the AD7327 automatically configures the analog		*/
/* input to the appropriate range, as indicated by Range Register 2. The ±10 V input range is selected by 		*/
/* default on each analog input channel (see Table 13). 														*/
/*==============================================================================================================*/
#define	RR2_VIN4(x)				(((x)&0x03)<<11)
#define	RR2_VIN5(x)				(((x)&0x03)<<9)
#define	RR2_VIN6(x)				(((x)&0x03)<<7)
#define	RR2_VIN7(x)				(((x)&0x03)<<5)

/*==============================================================================================================*/
/* SAMPLE RESOLUTIONS							 																*/
/*==============================================================================================================*/
/* Decimal value of sample resolution dependable on selected ADC Range										*/
/*==============================================================================================================*/
#define	ADC_RANGE_12BIT_RES					(0x01<<12)		// applies for ADC_RANGE_UNI_10V

/*==============================================================================================================*/


typedef enum _adc_range_
{
	ADC_RANGE_BIP_10V = 0,			/* This combination selects the ±10 V input range */
	ADC_RANGE_BIP_5V,				/* This combination selects the ± 5 V input range */
	ADC_RANGE_BIP_2_5V, 			/* This combination selects the ±2.5V input range */
	ADC_RANGE_UNI_10V				/* This combination selects the 0 V to +10 V input range */
	
} adc_range_et;

typedef enum _adc_ref_
{
	ADC_REF_EXTERNAL=0,				/* Disable internal reference */
	ADC_REF_INTERNAL				/* Enable internal reference */
} adc_ref_et;


typedef enum _adc_mode_
{
	ADC_MODE_8_SINGLE_END=0,		/* 8 Single-Ended Inputs 		*/
	ADC_MODE_4_PSEUDO_DIFF,			/* 4 Pseudo Differential Inputs */
	ADC_MODE_4_FULLY_DIFF,			/* 4 Fully Differential Inputs 	*/
	ADC_MODE_7_PSEUDO_DIFF			/* 7 Pseudo Differential Inputs */
} adc_mode_et;


typedef enum _adc_coding_
{
	ADC_CODING_2S_COMPLEMENT=0,		/* 2's complement  		*/
	ADC_CODING_BINARY				/* Straight Binary	*/
} adc_coding_et;

typedef enum _adc_pm_
{
	ADC_PM_AUTO_NORMAL=0,		/* Normal Mode. */
	ADC_PM_AUTO_STANDBY,		/* Autostandby Mode. */
	ADC_PM_AUTO_SHUTDOWN,		/* Autoshutdown Mode. */
	ADC_PM_FULL_SHUTDOWN,		/* Full Shutdown Mode. */
} adc_pm_et;

struct adc_params
{
	adc_mode_et 	mode;			/* Input Mode */
	adc_pm_et 		pm;				/* Power Mode */
	adc_coding_et 	coding;			/* Output Coding */
	adc_ref_et 		ref;			/* Internal/External Ref */
};

typedef enum _adc_seq_
{
	SEQ_NO_USED_0 = 0,			/* Sequence Not Used */
	SEQ_USE_SEQ_REG,			/* Use sequence of channels previously programmed into the sequence register */
	SEQ_USE_CTRL_REG,			/* Use channel address bits in the control register */
	SEQ_NO_USED_3				/* Sequence Not Used */
	
} adc_seq_et;


#define AD7327_CURRENT_CHANNEL(num)     \
        { \
                .type = IIO_CURRENT, \
                .indexed = 1, \
                .channel = (num), \
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
        }

static const struct iio_chan_spec ad7327_channels[] = {
        AD7327_CURRENT_CHANNEL(0),
        AD7327_CURRENT_CHANNEL(1),
        AD7327_CURRENT_CHANNEL(2),
        AD7327_CURRENT_CHANNEL(3),
        AD7327_CURRENT_CHANNEL(4),
        AD7327_CURRENT_CHANNEL(5),
        //AD7327_CURRENT_CHANNEL(6),
        //AD7327_CURRENT_CHANNEL(7),
};

typedef struct ad7327_config
{
	int					chip;
	char 				name[MAX_AD7327_NAME_LEN];	
	int					nchan;
	int					last;

	adc_range_et 		range [MAX_AD7327_CHANNELS];	/* Range setting for all channels */
	struct adc_params 	params;							/* Chip params */
	const struct iio_chan_spec      *channels;

} ad7327_config_t;


typedef struct ad7327_platform_data
{
	struct spi_device *spi;

	int chip;

	u16 control;
	u16 sequence;
	u16 range1;
	u16 range2;

	int convert_chan;

	atomic_t usage;	
	spinlock_t slock;

	ad7327_config_t config;

	adc_seq_et seqencer;

	struct mutex mlock;

} ad7327_platform_data;


#endif /* _LINUX_PLATFORM_DATA_AD7327_H_ */

