// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5308, AD5318, AD5328, Digital to analog convertors spi driver
 *
 * TODO: based on...
 *
 *
 * Copyright 2010-2011 Analog Devices Inc.
 */

#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <asm/unaligned.h>

#include "ad53x8.h"

static int ad53x8_spi_write_val(struct spi_device *spi, u8 addr, u16 val, u8 shift)
{
	u16 msg;

	msg = cpu_to_be((0 << 15) | (addr << 14) | (val << shift));

	return spi_write(spi, cpu_to_be16(msg), sizeof(msg));
}

static int ad53x8_spi_write_cmd(struct spi_device *spi,
				enum ad53x8_command_ids cmd,
				u8 chan, u8 val)
{
	u16 msg = (1 << 15);
	struct ad53x8_state *st = iio_priv(indio_dev);

	switch (cmd) {
		case PDOWN:
			// val = !(val == 0)
			// u8 pwr_down_buf_new = ((st->pwr_down_buf & ~(1 << chan)) | (val << chan))
			if (val)
				st->pwr_down_buf |= (1 << chan);
			else
				st->pwr_down_buf &= ~(1 << chan);
			msg = msg | (0b10 << 13) | (st->pwr_down_buf);
			break;

		case LDAC:
			val &= 0x11
			msg = msg | (0b01 << 13) | val
			break;

		case GAIN:
			break;

		case BUF:
			break;

		case VDD:
			break;
		case RESET:
			msg = msg | (0b11 << 13) | (bool(val) << 12);
			break;
	}
	return spi_write(spi, cpu_to_be16(msg), sizeof(msg));
}

static int ad53x8_get_from_buf_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
	\tscale_mv = ad5360_get_channel_vref(st, chan->channel);
	\tif (scale_mv < 0)
		\treturn scale_mv;
	\t*val = scale_mv;
	\t*val2 = chan->scan_type.realbits;
	\treturn IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int ad53x8_write_raw(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan,
				   int val,
				   int val2,
				   long mask)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;
		msg = cpu_to_be((0 << 15) | (chan->address << 14) | (val << scan_type.shift))

		return spi_write(st->us, cpu_to_be16(msg), sizeof(msg))

	// case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val >= 1 || val <= 2)
			return -EINVAL;

		return ad53x8_spi_write_cmd(indio_dev, GAIN, chan->address, val);

	default:
		return -EINVAL;
	}
}

static const char * const ad53x8_ldac_modes[] = {
	"low" = AD53X8_LDAC_LOW,
	"high" = AD53X8_LDAC_HIGH,
	"single_update" = AD53X8_LDAC_SINGLE_UPDATE
};

static const char * const ad53x8_gain_modes[] = {
	"none" = 0,
	"a-d" = 1,
	"e-h" = 2,
	"a-h" = 3,
};

static const char * const ad53x8_reset_modes[] = {
	"data" = 0,
	"data&control" = 1,
};

// static int ad53x8_get_ldac(struct iio_dev *indio_dev,
// 	const struct iio_chan_spec *chan)
// {
// 	struct ad53x8_state *st = iio_priv(indio_dev);
//
// 	return st->ldac;
// }

static int ad53x8_set_ldac(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	st->ldac = mode;

	return 0;
}

static int ad53x8_reset(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	return ad53x8_spi_write_cmd(indio_dev, RESET, chan->address, val);

	return 0;
}


static int ad53x8_get_gain(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	return st->gain;
}

static int ad53x8_set_gain(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	st->gain = mode;

	return 0;
}

static int ad53x8_set_ldac(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	st->ldac = mode;

	return 0;
}


static const struct iio_enum ad53x8_ldac_enum = {
	.items = ad53x8_ldac_modes,
	.num_items = ARRAY_SIZE(ad53x8_ldac_modes),
	// .get = ad53x8_get_ldac,
	.set = ad53x8_set_ldac,
};

static const struct iio_enum ad53x8_reset_enum = {
	.items = ad53x8_reset_modes,
	.num_items = ARRAY_SIZE(ad53x8_reset_modes),
	.set = ad53x8_reset,
};

static ssize_t ad53x8_read_dac_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ad53x8_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n",
			  !!(st->pwr_down_buf & (1 << chan->channel)));
}

static ssize_t ad53x8_write_dac_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	bool pwr_down;
	int ret;

	ret = kstrtobool(buf, &pwr_down);
	if (ret)
		return ret;

	ret = ad53x8_spi_write_cmd(indio_dev, PDOWN, chan, (u8)pwr_down);

	return ret ? ret : len;
}

static int ad53x8_get_channel_vref(struct ad5360_state *st,
	unsigned int channel)
{
	unsigned int i = (channel < 5) ? 0 : 1;

	return regulator_get_voltage(st->vref_reg[i].consumer);
}

static const struct iio_info ad53x8_info = {
	.write_raw = ad53x8_write_raw,
	.read_raw = ad53x8_get_from_buf_raw,
};

static const struct iio_chan_spec_ext_info ad53x8_ext_info[] = {
	{
		.name = "powerdown",
		.read = ad53x8_read_dac_powerdown,
		.write = ad53x8_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("ldac", IIO_SHARED_BY_TYPE, &ad53x8_ldac_enum),
	IIO_ENUM_AVAILABLE("ldac", IIO_SHARED_BY_TYPE, &ad53x8_ldac_enum)
	IIO_ENUM("gain_2vref", IIO_SHARED_BY_TYPE, &ad53x8_gain_enum),
	IIO_ENUM_AVAILABLE("gain_2vref", IIO_SHARED_BY_TYPE, &ad53x8_gain_enum),
	IIO_ENUM("reset", IIO_SHARED_BY_TYPE, &ad53x8_reset_enum),
	IIO_ENUM_AVAILABLE("reset", IIO_SHARED_BY_TYPE, &ad53x8_reset_enum)
	{ },
};

#define AD53x8_CHANNEL(_chan, _bits) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.channel = (_chan), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.address = (_chan), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = (_bits), \
		.storagebits = 12, \
		.shift = 12 - (_bits), \
	}, \
	.ext_info = ad53x8_ext_info, \
}

#define DECLARE_AD53X8_CHANNELS(_name, _bits) \
	const struct iio_chan_spec _name##_channels[] = { \
		AD53x8_CHANNEL(0, _bits), \
		AD53x8_CHANNEL(1, _bits), \
		AD53x8_CHANNEL(2, _bits), \
		AD53x8_CHANNEL(3, _bits), \
		AD53x8_CHANNEL(4, _bits), \
		AD53x8_CHANNEL(5, _bits), \
		AD53x8_CHANNEL(6, _bits), \
		AD53x8_CHANNEL(7, _bits), \
}

static DECLARE_AD53X8_CHANNELS(ad5308, 8);
static DECLARE_AD53X8_CHANNELS(ad5318, 10);
static DECLARE_AD53X8_CHANNELS(ad5328, 12);

static const struct ad53x8_chip_info ad53x8_chip_info_tbl[] = {
	[ID_AD5308] = {
		.channels = ad5308_channels,
	},
	[ID_AD5318] = {
		.channels = ad5318_channels,
	},
	[ID_AD5328] = {
		.channels = ad5328_channels,
	},
};

static int ad53x8_probe(struct spi_device *spi)
{
	struct ad53x8_state *st;
	struct iio_dev *indio_dev;
	int ret = 0;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	// st->reg = devm_regulator_get_optional(&spi->dev, "vref"); // todo get 2 vrefs
	// if (!IS_ERR(st->reg)) {
	// 	ret = regulator_enable(st->reg);
	// 	if (ret)
	// 		return ret;
 //
	// 	ret = regulator_get_voltage(st->reg);
	// 	if (ret < 0)
	// 		goto error_disable_reg;
 //
	// 	st->vref1_mv = ret;
	// } else {
	// 	if (PTR_ERR(st->reg) != -ENODEV)
	// 		return PTR_ERR(st->reg);
	// }

	st->vref_reg[0].supply = "vref0_mv";
	st->vref_reg[1].supply = "vref1_mv";

	ret = devm_regulator_bulk_get(&st->spi->dev, 2, st->vref_reg);
	if (ret) {
		dev_err(&spi->dev, "Failed to request vref regulators: %d\n", ret);
		goto error_free_channels;
	}

	ret = regulator_bulk_enable(2, st->vref_reg);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable vref regulators: %d\n", ret);
		goto error_free_channels;
	}

	spi_set_drvdata(spi, indio_dev);
	st->chip_info = &ad53x8_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	st->us = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad53x8_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = AD53X8_DAC_CHANNELS;

	if (ret)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	regulator_bulk_disable(st->chip_info->num_vrefs, st->vref_reg);;
	return ret;
}

static void ad53x8_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad53x8_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	regulator_bulk_disable(st->chip_info->num_vrefs, st->vref_reg);
}

static const struct spi_device_id ad53x8_id[] = {
	{"ad5308", ID_AD5308},
	{"ad5318", ID_AD5318},
	{"ad5328", ID_AD5328},
	{}
};
MODULE_DEVICE_TABLE(spi, ad53x8_id);

static struct spi_driver ad53x8_driver = {
	.driver = {
		   .name = "ad53x8",
		   },
	.probe = ad53x8_probe,
	.remove = ad53x8_remove,
	.id_table = ad53x8_id,
};
module_spi_driver(ad53x8_driver);

MODULE_AUTHOR("Volodymyr Yarish <volodymyr.comm@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD5308/18/28 DAC spi driver");
MODULE_LICENSE("GPL v2");
