// SPDX-License-Identifier: GPL-2.0
#include <linux/platform_device.h>
#include <linux/module.h>

#include <linux/of.h>
#include <linux/pwm.h>
#include <linux/delay.h>


struct pwm_effects_priv {
    struct pwm_chip chip;
    struct pwm_device *pwm;
    struct mutex lock;
    struct pwm_state state;
    struct pwm_state next_state;
    struct work_struct fade;
    int worker_running;
    int fade_period_start;
    int fade_period_diff;
    int fade_duty_start;
    int fade_duty_diff;
};


static void pwm_effects_fade(struct work_struct *work)
{
    struct pwm_effects_priv *pdata = container_of(work, struct pwm_effects_priv, fade);
    // struct pwm_chip *chip = &pdata->chip;
    struct pwm_state state;
    int total = 200;
    int count = 0;

    if (!pdata || !pdata->pwm) {
        return;
    }
/*
    dev_info(chip->dev, "pwm_effects_fade start (%d/%d => %d/%d)\n",
        pdata->fade_duty_start,
        pdata->fade_period_start,
        pdata->fade_duty_start + pdata->fade_duty_diff,
        pdata->fade_period_start + pdata->fade_period_diff
    );
*/
    memcpy(&state, &pdata->state, sizeof(struct pwm_state));

    while (pdata->worker_running) {

        if (pdata->fade_duty_diff) {
            state.duty_cycle = pdata->fade_duty_start + (pdata->fade_duty_diff * count / total);
        } else {
            state.duty_cycle = pdata->fade_duty_start;
        }

        if (pdata->fade_period_diff) {
            state.period = pdata->fade_period_start + (pdata->fade_period_diff * count / total);
        } else {
            state.period = pdata->fade_period_start;
        }

        pwm_apply_might_sleep(pdata->pwm, &state);

        count++;
        if (count >= total) {
            break;
        }

        usleep_range (10000, 12000);

    }
/*
    dev_info(chip->dev, "pwm_effects_fade done\n");
*/
    pdata->worker_running = 0;
}


static int pwm_effects_apply(struct pwm_chip *chip, struct pwm_device *pwm, const struct pwm_state *state)
{
    struct pwm_effects_priv *pdata = pwmchip_get_drvdata(chip);

    mutex_lock(&pdata->lock);

    memcpy(&pdata->next_state, state, sizeof(struct pwm_state));

    // Cancel any running effects
    pdata->worker_running = 0;
    cancel_work_sync(&pdata->fade);

    // Make sure current state is up to date
    pwm_get_state(pdata->pwm, &pdata->state);

    // Fade params
    pdata->fade_period_start = pdata->state.period;
    pdata->fade_period_diff = pdata->next_state.period - pdata->state.period;
    pdata->fade_duty_start = pdata->state.duty_cycle;
    pdata->fade_duty_diff = pdata->next_state.duty_cycle - pdata->state.duty_cycle;

    pdata->state.enabled = pdata->next_state.enabled;
    pdata->state.polarity = pdata->next_state.polarity;
    pwm_apply_might_sleep(pdata->pwm, &pdata->state);
    pdata->state.period = pdata->next_state.period;
    pdata->state.duty_cycle = pdata->next_state.duty_cycle;

    // Schedule fade task
    pdata->worker_running = 1;
    schedule_work(&pdata->fade);

    mutex_unlock(&pdata->lock);

    return 0;
}


static int pwm_effects_get_state(struct pwm_chip *chip, struct pwm_device *pwm, struct pwm_state *state)
{
    struct pwm_effects_priv *pdata = pwmchip_get_drvdata(chip);

    mutex_lock(&pdata->lock);
    memcpy(state, &pdata->state, sizeof(struct pwm_state));
    mutex_unlock(&pdata->lock);

    return 0;
}


static const struct pwm_ops pwm_effects_ops = {
    .apply = pwm_effects_apply,
    .get_state = pwm_effects_get_state,
};


static int pwm_effects_init(struct pwm_chip *chip)
{
    struct pwm_effects_priv *pdata;
    struct pwm_state state;

    pdata = pwmchip_get_drvdata(chip);
    if (!pdata || !pdata->pwm) {
        return -EINVAL;
    }

    pwm_get_state(pdata->pwm, &state);

    state.enabled = 1;
    state.duty_cycle = (state.period >> 4) * 10;

    return pwm_effects_apply(chip, NULL, &state);
}


static int pwm_effects_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct pwm_chip *chip;
    struct pwm_effects_priv *pdata;
    int ret;
    int npwm = 1;

    chip = devm_pwmchip_alloc(dev, npwm, sizeof(*pdata));
    if (IS_ERR(chip)) {
        return PTR_ERR(chip);
    }

    chip->ops = &pwm_effects_ops;

    pdata = pwmchip_get_drvdata(chip);

    mutex_init(&pdata->lock);

    pdata->pwm = devm_pwm_get(dev, NULL);
    if (IS_ERR (pdata->pwm)) {
        if (PTR_ERR(pdata->pwm) == -EPROBE_DEFER) {
            goto defer;
        }
        dev_err(dev, "devm_pwm_get() failed: %ld\n", PTR_ERR(pdata->pwm));
        pdata->pwm = NULL;
    }

    ret = pwmchip_add(chip);
    if (ret < 0) {
        dev_err(dev, "failed to add PWM chip %d\n", ret);
        return ret;
    }

    // platform_set_drvdata(pdev, pdata);

    INIT_WORK(&pdata->fade, pwm_effects_fade);

    pwm_effects_init(chip);

    dev_info(dev, "registered\n");

    return 0;

defer:
    return -EPROBE_DEFER;
}


static void pwm_effects_remove(struct platform_device *pdev)
{
}


static const struct of_device_id pwm_effects_dt_ids[] = {
    { .compatible = "pwm-effects" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pwm_effects_dt_ids);

static struct platform_driver pwm_effects_driver = {
    .probe = pwm_effects_probe,
    .remove = pwm_effects_remove,
    .driver = {
        .name = "pwm_effects",
        .of_match_table = of_match_ptr(pwm_effects_dt_ids),
    },
};
module_platform_driver(pwm_effects_driver);


MODULE_AUTHOR("Erki Aring");
MODULE_DESCRIPTION("PWM effects module");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
