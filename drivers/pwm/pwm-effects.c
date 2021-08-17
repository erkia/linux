#include <linux/platform_device.h>
#include <linux/module.h>

#include <linux/of.h>
#include <linux/pwm.h>
#include <linux/delay.h>


struct pwm_effects_drvdata {
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


#ifdef CONFIG_OF
static const struct of_device_id pwm_effects_dt_ids[] = {
    {
        .compatible = "pwm-effects",
    }, {
        /* sentinel */
    }
};
MODULE_DEVICE_TABLE(of, pwm_effects_dt_ids);
#endif


static void pwm_effects_fade(struct work_struct *work)
{
    struct pwm_effects_drvdata *pdata = container_of(work, struct pwm_effects_drvdata, fade);
    struct pwm_state state;
    int total = 200;
    int count = 0;

    if (!pdata || !pdata->pwm) {
        return;
    }
/*
    dev_info(pdata->chip.dev, "pwm_effects_fade start (%d/%d => %d/%d)\n",
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

        pwm_apply_state(pdata->pwm, &state);

        count++;
        if (count >= total) {
            break;
        }

        usleep_range (10000, 12000);

    }
/*
    dev_info(pdata->chip.dev, "pwm_effects_fade done\n");
*/
    pdata->worker_running = 0;
}


static int pwm_effects_apply(struct pwm_chip *chip, struct pwm_device *pwm, const struct pwm_state *state)
{
    struct pwm_effects_drvdata *pdata = container_of(chip, struct pwm_effects_drvdata, chip);

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
    pwm_apply_state(pdata->pwm, &pdata->state);
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
    struct pwm_effects_drvdata *pdata = container_of(chip, struct pwm_effects_drvdata, chip);

    mutex_lock(&pdata->lock);
    memcpy(state, &pdata->state, sizeof(struct pwm_state));
    mutex_unlock(&pdata->lock);

    return 0;
}


static const struct pwm_ops pwm_effects_ops = {
    .apply = pwm_effects_apply,
    .get_state = pwm_effects_get_state,
    .owner = THIS_MODULE,
};


static int pwm_effects_init(struct platform_device *pdev)
{
    struct pwm_effects_drvdata *pdata;
    struct pwm_state state;

    pdata = platform_get_drvdata(pdev);
    if (!pdata || !pdata->pwm) {
        return -EINVAL;
    }

    pwm_get_state(pdata->pwm, &state);

    state.enabled = 1;
    state.duty_cycle = (state.period >> 4) * 10;

    return pwm_effects_apply(&pdata->chip, NULL, &state);
}


static int pwm_effects_probe(struct platform_device *pdev)
{
    struct pwm_effects_drvdata *pdata;
    int ret;

    pdata = platform_get_drvdata(pdev);
    if (!pdata) {
        pdata = devm_kzalloc(&pdev->dev, sizeof(struct pwm_effects_drvdata), GFP_KERNEL);
        if (!pdata) {
            return -ENOMEM;
        }
    }

    mutex_init(&pdata->lock);

    pdata->pwm = devm_pwm_get(&pdev->dev, NULL);
    if (IS_ERR (pdata->pwm)) {
        if (PTR_ERR(pdata->pwm) == -EPROBE_DEFER) {
            goto defer;
        }
        dev_err(&pdev->dev, "devm_pwm_get() failed: %ld\n", PTR_ERR(pdata->pwm));
        pdata->pwm = NULL;
    }

    pdata->chip.dev = &pdev->dev;
    pdata->chip.ops = &pwm_effects_ops;
    pdata->chip.of_xlate = of_pwm_xlate_with_flags;
    pdata->chip.of_pwm_n_cells = 1;
    pdata->chip.base = -1;
    pdata->chip.npwm = 1;

    ret = pwmchip_add(&pdata->chip);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to add PWM chip %d\n", ret);
        return ret;
    }

    platform_set_drvdata(pdev, pdata);

    INIT_WORK(&pdata->fade, pwm_effects_fade);

    pwm_effects_init(pdev);

    dev_info(&pdev->dev, "registered\n");

    return 0;

defer:
    return -EPROBE_DEFER;
}


static int pwm_effects_remove(struct platform_device *pdev)
{
    return 0;
}


static struct platform_driver pwm_effects_driver = {
    .probe = pwm_effects_probe,
    .remove = pwm_effects_remove,
    .driver = {
        .name = "pwm_effects",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(pwm_effects_dt_ids),
    },
};
module_platform_driver(pwm_effects_driver);


MODULE_AUTHOR("Erki Aring");
MODULE_DESCRIPTION("PWM effects module");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
