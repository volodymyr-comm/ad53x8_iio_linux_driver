/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * AD53X8 SPI DAC driver
 *
 * Copyright 2010-2011 Analog Devices Inc.
 */
#ifndef SPI_AD53X8_H_
#define SPI_AD53X8_H_


#define AD53X8_DAC_CHANNELS     8

#define AD53X8_LDAC_LOW                 0x0
#define AD53X8_LDAC_HIGH                0x1
#define AD53X8_LDAC_SINGLE_UPDATE       0x2
#define AD53X8_GAIN_VREF                0x0
#define AD53X8_GAIN_2VREF               0x1



/**
 * struct ad53x8_chip_info - chip specific information
 * @channels:		channel spec for the DAC
 * @resolution_bits:	AD53x8 resolution bits
 */

struct ad53x8_chip_info {
	const struct iio_chan_spec	*channels;
	u8 resolution_bits;
};

/**
 * struct ad53x8_state - driver instance specific data
 * @indio_dev:		the industrial I/O device
 * @us:			spi_device
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:		supply regulator
 * @vref1_mv:		actual reference voltage #1 used
 * @vref2_mv:		actual reference voltage #2 used
 * @pwr_down_mask: 	power down mask;
 * @ldac_mask: actual LDAC mask
 * @gain_mask: actual GAIN mask
 * @buf_mask: actual BUF mask
 * @vdd_mask: actual VDD mask
 */

struct ad53x8_state {
	struct spi_device		*us;
	const struct ad53x8_chip_info	*chip_info;
	struct regulator		*reg;
	unsigned short			vref1_mv;
    unsigned short			vref2_mv;
	unsigned short			pwr_down_mask;
    unsigned short			ldac_mask;
    unsigned short			gain_mask;
    unsigned short			buf_mask;
    unsigned short			vdd_mask;
};


enum ad53x8_supported_device_ids {
	ID_AD5308,
    ID_AD5318,
    ID_AD5328,
};

enum ad53x8_command_ids {
	PDOWN,
    LDAC,
    GAIN,
    BUF,
    VDD
};

#endif /* SPI_AD53X8_H_ */
