// SPDX-License-Identifier: GPL-2.0
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/random.h>

#include "fb_internal.h"

struct fb_logo_priv {
    struct device *dev;
    uint8_t rotate;
    struct notifier_block fb_notif;
};


static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_logo_priv *priv = container_of(self, struct fb_logo_priv, fb_notif);
    struct fb_event *evdata = data;
    struct fb_info *info = evdata->info;

    if (fb_prepare_logo(info, priv->rotate)) {
        fb_set_cmap(&info->cmap, info);
        fb_show_logo(info, priv->rotate);
    }

    if (info->fbops->fb_sync) {
        info->fbops->fb_sync(info);
    } else if (info->fbops->fb_copyarea) {
        struct fb_copyarea area = {
            .dx = 0, .dy = 0,
            .sx = 0, .sy = 0,
            .width  = info->var.xres,
            .height = info->var.yres,
        };
        info->fbops->fb_copyarea(info, &area);
    } else {
        dev_err(priv->dev, "cannot sync\n");
    }

    if (info->fbops->fb_set_par) {
        info->fbops->fb_set_par(info);
    }

    return 0;
}

static int fb_logo_probe(struct platform_device *pdev)
{
    struct fb_logo_priv *priv;
    struct device *dev = &pdev->dev;
    u32 rotate;

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        return -ENOMEM;
    }

    priv->dev = dev;

    if (of_property_read_u32(dev->of_node, "rotate", &rotate) == 0) {
        switch (rotate) {
            case 0:
                priv->rotate = FB_ROTATE_UR;
                break;
            case 1:
                priv->rotate = FB_ROTATE_CW;
                break;
            case 2:
                priv->rotate = FB_ROTATE_UD;
                break;
            case 3:
                priv->rotate = FB_ROTATE_CCW;
                break;
            default:
                dev_warn(dev, "invalid rotate value: %d\n", rotate);
                priv->rotate = FB_ROTATE_UR;
                break;
        }
    } else {
        priv->rotate = FB_ROTATE_UR;
    }

    priv->fb_notif.notifier_call = fb_notifier_callback;
    if (fb_register_client(&priv->fb_notif)) {
        return -EINVAL;
    }

    platform_set_drvdata(pdev, priv);

    dev_info(dev, "registered\n");

    return 0;
}

static void fb_logo_remove(struct platform_device *pdev)
{
    struct fb_logo_priv *priv = platform_get_drvdata(pdev);
    if (priv) {
        fb_unregister_client(&priv->fb_notif);
    }
}


static const struct of_device_id fb_logo_dt_ids[] = {
    { .compatible = "fb-boot-logo" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fb_logo_dt_ids);

static struct platform_driver fb_logo_driver = {
    .probe  = fb_logo_probe,
    .remove = fb_logo_remove,
    .driver = {
        .name           = "fb-boot-logo",
        .of_match_table = fb_logo_dt_ids,
    },
};

module_platform_driver(fb_logo_driver);


MODULE_AUTHOR("Erki Aring");
MODULE_DESCRIPTION("Framebuffer boot logo");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
