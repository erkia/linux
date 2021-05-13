/*
 * DRM driver for Multi-Inno MI0283AAT panels
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/tinydrm/st7789v.h>
#include <drm/tinydrm/mipi-dbi.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <video/mipi_display.h>

typedef struct mi0283aat_config {
	u32 lutb;
} mi0283aat_config_t;

#define MI0283AAT_CONFIG 0x1C
#define DRM_IOCTL_MI0283AAT_CONFIG DRM_IOW(DRM_COMMAND_BASE + MI0283AAT_CONFIG, struct mi0283aat_config)

static void mi0283aat_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
	int ret;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(mipi);
	if (ret < 0)
		return;
	if (ret == 1)
		goto out_enable;

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

	mipi_dbi_command(mipi, ST7789V_DGMEN, 0x04);
	mipi_dbi_command(mipi, MIPI_DCS_ENTER_INVERT_MODE);

	mipi_dbi_command(mipi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(50);

	mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_ON);

out_enable:
	mipi_dbi_enable_flush(mipi, crtc_state, plane_state);
}


static int mi0283aat_config_ioctl(struct drm_device *dev, void *data, struct drm_file *filp)
{
	struct tinydrm_device *tdev = dev->dev_private;
	struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
	mi0283aat_config_t *val = data;

	if (val->lutb < 0) {
		val->lutb = 0;
	} else if (val->lutb > 100) {
		val->lutb = 100;
	}

	mipi_dbi_command(mipi, ST7789V_DGMLUTB,
		0x00 * val->lutb / 100, 0x04 * val->lutb / 100, 0x08 * val->lutb / 100, 0x0C * val->lutb / 100, 0x10 * val->lutb / 100, 0x14 * val->lutb / 100, 0x18 * val->lutb / 100, 0x1C * val->lutb / 100,
		0x20 * val->lutb / 100, 0x24 * val->lutb / 100, 0x28 * val->lutb / 100, 0x2C * val->lutb / 100, 0x30 * val->lutb / 100, 0x34 * val->lutb / 100, 0x38 * val->lutb / 100, 0x3C * val->lutb / 100,
		0x40 * val->lutb / 100, 0x44 * val->lutb / 100, 0x48 * val->lutb / 100, 0x4C * val->lutb / 100, 0x50 * val->lutb / 100, 0x54 * val->lutb / 100, 0x58 * val->lutb / 100, 0x5C * val->lutb / 100,
		0x60 * val->lutb / 100, 0x64 * val->lutb / 100, 0x68 * val->lutb / 100, 0x6C * val->lutb / 100, 0x70 * val->lutb / 100, 0x74 * val->lutb / 100, 0x78 * val->lutb / 100, 0x7C * val->lutb / 100,
		0x80 * val->lutb / 100, 0x84 * val->lutb / 100, 0x88 * val->lutb / 100, 0x8C * val->lutb / 100, 0x90 * val->lutb / 100, 0x94 * val->lutb / 100, 0x98 * val->lutb / 100, 0x9C * val->lutb / 100,
		0xA0 * val->lutb / 100, 0xA4 * val->lutb / 100, 0xA8 * val->lutb / 100, 0xAC * val->lutb / 100, 0xB0 * val->lutb / 100, 0xB4 * val->lutb / 100, 0xB8 * val->lutb / 100, 0xBC * val->lutb / 100,
		0xC0 * val->lutb / 100, 0xC4 * val->lutb / 100, 0xC8 * val->lutb / 100, 0xCC * val->lutb / 100, 0xD0 * val->lutb / 100, 0xD4 * val->lutb / 100, 0xD8 * val->lutb / 100, 0xDC * val->lutb / 100,
		0xE0 * val->lutb / 100, 0xE4 * val->lutb / 100, 0xE8 * val->lutb / 100, 0xEC * val->lutb / 100, 0xF0 * val->lutb / 100, 0xF4 * val->lutb / 100, 0xF8 * val->lutb / 100, 0xFC * val->lutb / 100,
	);

	return 0;
}


static const struct drm_simple_display_pipe_funcs mi0283aat_pipe_funcs = {
	.enable = mi0283aat_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = tinydrm_display_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode mi0283aat_mode = {
	TINYDRM_MODE(320, 240, 58, 43),
};

static const struct drm_ioctl_desc mi0283aat_ioctls[] = {
	DRM_IOCTL_DEF_DRV(MI0283AAT_CONFIG, mi0283aat_config_ioctl, DRM_AUTH),
};

DEFINE_DRM_GEM_CMA_FOPS(mi0283aat_fops);

static struct drm_driver mi0283aat_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,
	.fops			= &mi0283aat_fops,
	TINYDRM_GEM_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.ioctls			= mi0283aat_ioctls,
	.num_ioctls		= sizeof(mi0283aat_ioctls) / sizeof(mi0283aat_ioctls[0]),
	.name			= "mi0283aat",
	.desc			= "Multi-Inno MI0283AAT",
	.date			= "20190602",
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

	mipi->backlight = devm_of_find_backlight(dev);
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

	return drm_mode_config_helper_suspend(mipi->tinydrm.drm);
}

static int __maybe_unused mi0283aat_pm_resume(struct device *dev)
{
	struct mipi_dbi *mipi = dev_get_drvdata(dev);

	drm_mode_config_helper_resume(mipi->tinydrm.drm);

	return 0;
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

MODULE_DESCRIPTION("Multi-Inno MI0283AAT DRM driver");
MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
