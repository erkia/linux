/*
 * DRM driver for Multi-Inno mi0283aat panels
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/tinydrm/st7789v.h>
#include <drm/tinydrm/mipi-dbi.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

static int mi0283aat_init(struct mipi_dbi *mipi)
{
	struct tinydrm_device *tdev = &mipi->tinydrm;
	struct device *dev = tdev->drm->dev;
	int ret;

	DRM_DEBUG_KMS("\n");

	ret = regulator_enable(mipi->regulator);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to enable regulator %d\n", ret);
		return ret;
	}

	// Avoid flicker by skipping setup if the bootloader has done it
	if (mipi_dbi_display_is_on(mipi))
		return 0;

	mipi_dbi_hw_reset(mipi);
	ret = mipi_dbi_command(mipi, MIPI_DCS_SOFT_RESET);
	if (ret) {
		DRM_DEV_ERROR(dev, "Error sending command %d\n", ret);
		regulator_disable(mipi->regulator);
		return ret;
	}

	msleep(20);

	//------------------------------------LCD SETTING-------------------------------------//
	mipi_dbi_command(mipi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);
	mipi_dbi_command(mipi, MIPI_DCS_SET_ADDRESS_MODE, 0x00);

	//----------------------------ST7789V Frame rate setting------------------------------//
	mipi_dbi_command(mipi, ST7789V_PORCTRL, 0x05, 0x05, 0x00, 0x33, 0x33);
	mipi_dbi_command(mipi, ST7789V_GCTRL, 0x35);

	//------------------------------ST7789V Power setting---------------------------------//
	mipi_dbi_command(mipi, ST7789V_VCOMS, 0x3F);
	mipi_dbi_command(mipi, ST7789V_LCMCTRL, 0x2C);
	mipi_dbi_command(mipi, ST7789V_VDVVRHEN, 0x01);
	mipi_dbi_command(mipi, ST7789V_VRHS, 0x0F);
	mipi_dbi_command(mipi, ST7789V_VDVS, 0x20);
	mipi_dbi_command(mipi, ST7789V_FRCTRL2, 0x11);
	mipi_dbi_command(mipi, ST7789V_PWCTRL1, 0xA4, 0xA1);
	mipi_dbi_command(mipi, ST7789V_PWCTRL2, 0x03);
	mipi_dbi_command(mipi, ST7789V_EQCTRL, 0x09, 0x09, 0x08);

	//--------------------------ST7789V gamma setting----------------------------//
	mipi_dbi_command(mipi, ST7789V_PVGAMCTRL, 0xD0, 0x05, 0x09, 0x09, 0x08, 0x14, 0x28,
										      0x33, 0x3F, 0x07, 0x13, 0x14, 0x28, 0x30);

	mipi_dbi_command(mipi, ST7789V_NVGAMCTRL, 0xD0, 0x05, 0x09, 0x08, 0x03, 0x24, 0x32,
											  0x32, 0x3B, 0x38, 0x14, 0x13, 0x28, 0x2F);

	mipi_dbi_command(mipi, MIPI_DCS_ENTER_INVERT_MODE);

	mipi_dbi_command(mipi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(50);

	mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_ON);

	return 0;
}

static void mi0283aat_fini(void *data)
{
	struct mipi_dbi *mipi = data;

	DRM_DEBUG_KMS("\n");
	regulator_disable(mipi->regulator);
}

static const struct drm_simple_display_pipe_funcs mi0283aat_pipe_funcs = {
	.enable = mipi_dbi_pipe_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = tinydrm_display_pipe_update,
	.prepare_fb = tinydrm_display_pipe_prepare_fb,
};

static const struct drm_display_mode mi0283aat_mode = {
	TINYDRM_MODE(320, 240, 58, 43),
};

DEFINE_DRM_GEM_CMA_FOPS(mi0283aat_fops);

static struct drm_driver mi0283aat_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,
	.fops			= &mi0283aat_fops,
	TINYDRM_GEM_DRIVER_OPS,
	.lastclose		= tinydrm_lastclose,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "mi0283aat",
	.desc			= "Multi-Inno mi0283aat",
	.date			= "20190103",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id mi0283aat_of_match[] = {
	{ .compatible = "multi-inno,mi0283aat" },
	{},
};
MODULE_DEVICE_TABLE(of, mi0283aat_of_match);

static const struct spi_device_id mi0283aat_id[] = {
	{ "mi0283aat", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, mi0283aat_id);

static int mi0283aat_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi *mipi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	mipi = devm_kzalloc(dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;

	mipi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mipi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(mipi->reset);
	}

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	mipi->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(mipi->regulator))
		return PTR_ERR(mipi->regulator);

	mipi->backlight = tinydrm_of_find_backlight(dev);
	if (IS_ERR(mipi->backlight))
		return PTR_ERR(mipi->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, mipi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi_init(&spi->dev, mipi, &mi0283aat_pipe_funcs,
			    &mi0283aat_driver, &mi0283aat_mode, rotation);
	if (ret)
		return ret;

	ret = mi0283aat_init(mipi);
	if (ret)
		return ret;

	/* use devres to fini after drm unregister (drv->remove is before) */
	ret = devm_add_action(dev, mi0283aat_fini, mipi);
	if (ret) {
		mi0283aat_fini(mipi);
		return ret;
	}

	spi_set_drvdata(spi, mipi);

	return devm_tinydrm_register(&mipi->tinydrm);
}

static void mi0283aat_shutdown(struct spi_device *spi)
{
	struct mipi_dbi *mipi = spi_get_drvdata(spi);

	tinydrm_shutdown(&mipi->tinydrm);
}

static int __maybe_unused mi0283aat_pm_suspend(struct device *dev)
{
	struct mipi_dbi *mipi = dev_get_drvdata(dev);
	int ret;

	ret = tinydrm_suspend(&mipi->tinydrm);
	if (ret)
		return ret;

	mi0283aat_fini(mipi);

	return 0;
}

static int __maybe_unused mi0283aat_pm_resume(struct device *dev)
{
	struct mipi_dbi *mipi = dev_get_drvdata(dev);
	int ret;

	ret = mi0283aat_init(mipi);
	if (ret)
		return ret;

	return tinydrm_resume(&mipi->tinydrm);
}

static const struct dev_pm_ops mi0283aat_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mi0283aat_pm_suspend, mi0283aat_pm_resume)
};

static struct spi_driver mi0283aat_spi_driver = {
	.driver = {
		.name = "mi0283aat",
		.owner = THIS_MODULE,
		.of_match_table = mi0283aat_of_match,
		.pm = &mi0283aat_pm_ops,
	},
	.id_table = mi0283aat_id,
	.probe = mi0283aat_probe,
	.shutdown = mi0283aat_shutdown,
};
module_spi_driver(mi0283aat_spi_driver);

MODULE_DESCRIPTION("Multi-Inno mi0283aat DRM driver");
MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
