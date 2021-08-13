// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for Multi-Inno MI0283AAT panels
 *
 * Copyright 2021 Erki Aring
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

#define ST7789V_RAMCTRL     0xB0
#define ST7789V_RGBCTRL     0xB1
#define ST7789V_PORCTRL     0xB2
#define ST7789V_FRCTRL1     0xB3
#define ST7789V_PARCTRL     0xB5
#define ST7789V_GCTRL       0xB7
#define ST7789V_GTADJ       0xB8
#define ST7789V_DGMEN       0xBA
#define ST7789V_VCOMS       0xBB
#define ST7789V_LCMCTRL     0xC0
#define ST7789V_IDSET       0xC1
#define ST7789V_VDVVRHEN    0xC2
#define ST7789V_VRHS        0xC3
#define ST7789V_VDVS        0xC4
#define ST7789V_VCMOFSET    0xC5
#define ST7789V_FRCTRL2     0xC6
#define ST7789V_CABCCTRL    0xC7
#define ST7789V_REGSEL1     0xC8
#define ST7789V_REGSEL2     0xCA
#define ST7789V_PWMFRSEL    0xCC
#define ST7789V_PWCTRL1     0xD0
#define ST7789V_VAPVANEN    0xD2
#define ST7789V_CMD2EN      0xDF
#define ST7789V_PVGAMCTRL   0xE0
#define ST7789V_NVGAMCTRL   0xE1
#define ST7789V_DGMLUTR     0xE2
#define ST7789V_DGMLUTB     0xE3
#define ST7789V_GATECTRL    0xE4
#define ST7789V_SPI2EN      0xE7
#define ST7789V_PWCTRL2     0xE8
#define ST7789V_EQCTRL      0xE9
#define ST7789V_PROMCTRL    0xEC
#define ST7789V_PROMEN      0xFA
#define ST7789V_NVMSET      0xFC
#define ST7789V_PROMACT     0xFE

typedef struct mi0283aat_config {
	u32 lutb;
} mi0283aat_config_t;

#define MI0283AAT_CONFIG 0x1C
#define DRM_IOCTL_MI0283AAT_CONFIG DRM_IOW(DRM_COMMAND_BASE + MI0283AAT_CONFIG, struct mi0283aat_config)

static void mi0283aat_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	//------------------------------------LCD SETTING-------------------------------------//
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, 0x00);

	//----------------------------ST7789V Frame rate setting------------------------------//
	mipi_dbi_command(dbi, ST7789V_PORCTRL, 0x05, 0x05, 0x00, 0x33, 0x33);
	mipi_dbi_command(dbi, ST7789V_GCTRL, 0x35);

	//------------------------------ST7789V Power setting---------------------------------//
	mipi_dbi_command(dbi, ST7789V_VCOMS, 0x3F);
	mipi_dbi_command(dbi, ST7789V_LCMCTRL, 0x2C);
	mipi_dbi_command(dbi, ST7789V_VDVVRHEN, 0x01);
	mipi_dbi_command(dbi, ST7789V_VRHS, 0x0F);
	mipi_dbi_command(dbi, ST7789V_VDVS, 0x20);
	mipi_dbi_command(dbi, ST7789V_FRCTRL2, 0x11);
	mipi_dbi_command(dbi, ST7789V_PWCTRL1, 0xA4, 0xA1);
	mipi_dbi_command(dbi, ST7789V_PWCTRL2, 0x03);
	mipi_dbi_command(dbi, ST7789V_EQCTRL, 0x09, 0x09, 0x08);

	//--------------------------ST7789V gamma setting----------------------------//
	mipi_dbi_command(dbi, ST7789V_PVGAMCTRL, 0xD0, 0x05, 0x09, 0x09, 0x08, 0x14, 0x28,
										      0x33, 0x3F, 0x07, 0x13, 0x14, 0x28, 0x30);

	mipi_dbi_command(dbi, ST7789V_NVGAMCTRL, 0xD0, 0x05, 0x09, 0x08, 0x03, 0x24, 0x32,
											  0x32, 0x3B, 0x38, 0x14, 0x13, 0x28, 0x2F);

	mipi_dbi_command(dbi, ST7789V_DGMEN, 0x04);
	mipi_dbi_command(dbi, MIPI_DCS_ENTER_INVERT_MODE);

	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(50);

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

out_enable:
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static int mi0283aat_config_ioctl(struct drm_device *dev, void *data, struct drm_file *filp)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	mi0283aat_config_t *val = data;
	int idx;

	if (val->lutb < 0) {
		val->lutb = 0;
	} else if (val->lutb > 100) {
		val->lutb = 100;
	}

	if (!drm_dev_enter(dev, &idx)) {
		return -EINVAL;
	}

	mipi_dbi_command(dbi, ST7789V_DGMLUTB,
		0x00 * val->lutb / 100, 0x04 * val->lutb / 100, 0x08 * val->lutb / 100, 0x0C * val->lutb / 100, 0x10 * val->lutb / 100, 0x14 * val->lutb / 100, 0x18 * val->lutb / 100, 0x1C * val->lutb / 100,
		0x20 * val->lutb / 100, 0x24 * val->lutb / 100, 0x28 * val->lutb / 100, 0x2C * val->lutb / 100, 0x30 * val->lutb / 100, 0x34 * val->lutb / 100, 0x38 * val->lutb / 100, 0x3C * val->lutb / 100,
		0x40 * val->lutb / 100, 0x44 * val->lutb / 100, 0x48 * val->lutb / 100, 0x4C * val->lutb / 100, 0x50 * val->lutb / 100, 0x54 * val->lutb / 100, 0x58 * val->lutb / 100, 0x5C * val->lutb / 100,
		0x60 * val->lutb / 100, 0x64 * val->lutb / 100, 0x68 * val->lutb / 100, 0x6C * val->lutb / 100, 0x70 * val->lutb / 100, 0x74 * val->lutb / 100, 0x78 * val->lutb / 100, 0x7C * val->lutb / 100,
		0x80 * val->lutb / 100, 0x84 * val->lutb / 100, 0x88 * val->lutb / 100, 0x8C * val->lutb / 100, 0x90 * val->lutb / 100, 0x94 * val->lutb / 100, 0x98 * val->lutb / 100, 0x9C * val->lutb / 100,
		0xA0 * val->lutb / 100, 0xA4 * val->lutb / 100, 0xA8 * val->lutb / 100, 0xAC * val->lutb / 100, 0xB0 * val->lutb / 100, 0xB4 * val->lutb / 100, 0xB8 * val->lutb / 100, 0xBC * val->lutb / 100,
		0xC0 * val->lutb / 100, 0xC4 * val->lutb / 100, 0xC8 * val->lutb / 100, 0xCC * val->lutb / 100, 0xD0 * val->lutb / 100, 0xD4 * val->lutb / 100, 0xD8 * val->lutb / 100, 0xDC * val->lutb / 100,
		0xE0 * val->lutb / 100, 0xE4 * val->lutb / 100, 0xE8 * val->lutb / 100, 0xEC * val->lutb / 100, 0xF0 * val->lutb / 100, 0xF4 * val->lutb / 100, 0xF8 * val->lutb / 100, 0xFC * val->lutb / 100,
	);

	drm_dev_exit(idx);

	return 0;
}

static const struct drm_simple_display_pipe_funcs mi0283aat_pipe_funcs = {
	.mode_valid = mipi_dbi_pipe_mode_valid,
	.enable = mi0283aat_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
};

static const struct drm_display_mode mi0283aat_mode = {
	DRM_SIMPLE_MODE(320, 240, 58, 43),
};

static const struct drm_ioctl_desc mi0283aat_ioctls[] = {
	DRM_IOCTL_DEF_DRV(MI0283AAT_CONFIG, mi0283aat_config_ioctl, DRM_AUTH),
};

DEFINE_DRM_GEM_DMA_FOPS(mi0283aat_fops);

static struct drm_driver mi0283aat_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &mi0283aat_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.ioctls			= mi0283aat_ioctls,
	.num_ioctls		= sizeof(mi0283aat_ioctls) / sizeof(mi0283aat_ioctls[0]),
	.name			= "mi0283aat",
	.desc			= "Multi-Inno MI0283AAT",
	.date			= "20210812",
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
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &mi0283aat_driver,
					struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	dbidev->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(dbidev->regulator))
		return PTR_ERR(dbidev->regulator);

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi_dev_init(dbidev, &mi0283aat_pipe_funcs, &mi0283aat_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static void mi0283aat_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void mi0283aat_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused mi0283aat_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused mi0283aat_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

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
	.remove = mi0283aat_remove,
	.shutdown = mi0283aat_shutdown,
};
module_spi_driver(mi0283aat_spi_driver);

MODULE_DESCRIPTION("Multi-Inno MI0283AAT DRM driver");
MODULE_AUTHOR("Erki Aring");
MODULE_LICENSE("GPL");
