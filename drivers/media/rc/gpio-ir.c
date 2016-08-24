/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <media/rc-core.h>
#include <media/gpio-ir.h>

#define GPIO_IR_DRIVER_NAME	"gpio-rc"
#define GPIO_IR_DEVICE_NAME	"gpio_ir"

#define DEFAULT_CARRIER_FREQUENCY 38000
#define DEFAULT_DUTY_CYCLE 50

struct gpio_tx_timing {
    u32 frequency;
    u32 duty_cycle;
    u32 period;
    u32 pulse_width;
    u32 space_width;
};

struct gpio_rc_dev {
	struct rc_dev *rcdev;
	int rx_gpio_nr;
	bool rx_active_low;
	int tx_gpio_nr;
	bool tx_active_low;
	struct gpio_tx_timing timing;
};

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static int gpio_ir_get_devtree_pdata(struct device *dev,
				  struct gpio_ir_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags rx_flags, tx_flags;
	int rx_gpio, tx_gpio;
	tx_gpio = -1;

	rx_gpio = of_get_gpio_flags(np, 0, &rx_flags);
	if (rx_gpio < 0) {
		if (rx_gpio != -EPROBE_DEFER)
			dev_err(dev, "Failed to get rx gpio flags (%d)\n", rx_gpio);
		return rx_gpio;
	}

	if(of_gpio_count(np) > 1) {
    	tx_gpio = of_get_gpio_flags(np, 1, &tx_flags);
	}

	pdata->rx_gpio_nr = rx_gpio;
	pdata->rx_active_low = !!(rx_flags & OF_GPIO_ACTIVE_LOW);
    pdata->tx_gpio_nr = tx_gpio;
	pdata->tx_active_low = !!(tx_flags & OF_GPIO_ACTIVE_LOW);
	/* probe() takes care of map_name == NULL or allowed_protos == 0 */
	pdata->map_name = of_get_property(np, "linux,rc-map-name", NULL);
	pdata->allowed_protos = 0;

	return 0;
}

static struct of_device_id gpio_ir_of_match[] = {
	{ .compatible = "gpio-ir", },
    { .compatible = "gpio-ir-receiver", },
    { .compatible = "lirc_gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_ir_of_match);

#else /* !CONFIG_OF */

#define gpio_ir_tx_get_devtree_pdata(dev, pdata)	(-ENOSYS)

#endif


static irqreturn_t gpio_ir_rx_irq(int irq, void *dev_id)
{
	struct gpio_rc_dev *gpio_dev = dev_id;
	int gval;
	int rc = 0;
	enum raw_event_type type = IR_SPACE;

	gval = gpio_get_value_cansleep(gpio_dev->rx_gpio_nr);

	if (gval < 0)
		goto err_get_value;

	if (gpio_dev->rx_active_low)
		gval = !gval;

	if (gval == 1)
		type = IR_PULSE;

	rc = ir_raw_event_store_edge(gpio_dev->rcdev, type);
	if (rc < 0)
		goto err_get_value;

	ir_raw_event_handle(gpio_dev->rcdev);

err_get_value:
	return IRQ_HANDLED;
}

static bool gpio_ir_timing_valid(u32 freq, u32 dc) {
    //returns true if freq and dc are compatible.
    return ((dc * 10000L) > freq && ( (100 - dc) * 10000L) > freq );
}


static void gpio_ir_tx_calc_timing(struct gpio_rc_dev *gpio_dev) {
    if(!gpio_dev->timing.frequency) return;
    //updates the timing values based on the frequency and duty cycle
    gpio_dev->timing.period = 1000000L / gpio_dev->timing.frequency;
	gpio_dev->timing.pulse_width = gpio_dev->timing.period * gpio_dev->timing.duty_cycle / 100;
	gpio_dev->timing.space_width = gpio_dev->timing.period - gpio_dev->timing.pulse_width;
}

static int gpio_ir_tx_set_carrier(struct rc_dev *dev, u32 carrier) {
    struct gpio_rc_dev *gpio_dev = dev->priv;
    if(carrier && !gpio_ir_timing_valid(carrier, gpio_dev->timing.duty_cycle)) {
        return -EINVAL;
    }
    gpio_dev->timing.frequency = carrier;
    gpio_ir_tx_calc_timing(gpio_dev);

    return 0;
}

static int gpio_ir_tx_set_duty_cycle(struct rc_dev *dev, u32 duty_cycle) {
    struct gpio_rc_dev *gpio_dev = dev->priv;
    if(!gpio_ir_timing_valid(gpio_dev->timing.frequency, duty_cycle)) {
        return -EINVAL;
    }
    gpio_dev->timing.duty_cycle = duty_cycle;
    gpio_ir_tx_calc_timing(gpio_dev);

    return 0;
}

static ktime_t gpio_ir_tx_pulse_softcarrier(struct gpio_rc_dev *gpio_dev, u32 length, ktime_t start)
{
	int flag, active_low;
	u32 actual, d;

	actual = 0; flag = 0, active_low = gpio_dev->tx_active_low;
	while (actual + gpio_dev->timing.period < length) {
		if (flag) {
            gpio_set_value(gpio_dev->tx_gpio_nr, active_low ? 1 : 0);
			d = gpio_dev->timing.space_width;
		} else {
            gpio_set_value(gpio_dev->tx_gpio_nr, active_low ? 0 : 1);
			d = gpio_dev->timing.pulse_width;
		}
		if(actual + d > length) {
    		d -= (actual + d - length);
		}
		actual += d;
		flag = !flag;
    	start = ktime_add_us(start, d);
		while(ktime_us_delta(start, ktime_get()) > 0) cpu_relax();
	}
	return start;
}

static ktime_t gpio_ir_tx_pulse(struct gpio_rc_dev *gpio_dev, unsigned long length, ktime_t start)
{
	if (length && gpio_dev->timing.frequency) {
		return gpio_ir_tx_pulse_softcarrier(gpio_dev, length, start);
	} else {
        gpio_set_value(gpio_dev->tx_gpio_nr, gpio_dev->tx_active_low ? 0 : 1);
        start = ktime_add_us(start, length);
        while(ktime_us_delta(start, ktime_get()) > 0) cpu_relax();
		return start;
	}
}

static ktime_t gpio_ir_tx_space(struct gpio_rc_dev *gpio_dev, long length, ktime_t start)
{
    gpio_set_value(gpio_dev->tx_gpio_nr, gpio_dev->tx_active_low ? 1 : 0);
    start = ktime_add_us(start, length);
    while(ktime_us_delta(start, ktime_get()) > 0) cpu_relax();
    return start;
}

static int gpio_ir_tx(struct rc_dev *dev, unsigned *txbuf, unsigned count)
{
	int i;
	ktime_t start;
    struct gpio_rc_dev *gpio_dev = dev->priv;

	if (count % 2 == 0)
		return -EINVAL;

    start = ktime_get();
	for (i = 0; i < count; i++) {
		if (i%2)
			start = gpio_ir_tx_space(gpio_dev, txbuf[i], start);
		else
			start = gpio_ir_tx_pulse(gpio_dev, txbuf[i], start);
	}
    gpio_set_value(gpio_dev->tx_gpio_nr, gpio_dev->tx_active_low ? 1 : 0);

	return count;
}


static int gpio_ir_probe(struct platform_device *pdev)
{
	struct gpio_rc_dev *gpio_dev;
	struct rc_dev *rcdev;
	const struct gpio_ir_platform_data *pdata =
					pdev->dev.platform_data;
	int rc;

	if (pdev->dev.of_node) {
		struct gpio_ir_platform_data *dtpdata =
			devm_kzalloc(&pdev->dev, sizeof(*dtpdata), GFP_KERNEL);
		if (!dtpdata)
			return -ENOMEM;
		rc = gpio_ir_get_devtree_pdata(&pdev->dev, dtpdata);
		if (rc)
			return rc;
		pdata = dtpdata;
	}

	if (!pdata)
		return -EINVAL;

	if (pdata->rx_gpio_nr < 0)
		return -EINVAL;

	gpio_dev = kzalloc(sizeof(struct gpio_rc_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	rcdev = rc_allocate_device();
	if (!rcdev) {
		rc = -ENOMEM;
		goto err_allocate_device;
	}

	rcdev->priv = gpio_dev;
	rcdev->driver_type = RC_DRIVER_IR_RAW;
	rcdev->input_name = GPIO_IR_DEVICE_NAME;
	rcdev->input_phys = GPIO_IR_DEVICE_NAME "/input";
	rcdev->input_id.bustype = BUS_HOST;
	rcdev->input_id.vendor = 0x0001;
	rcdev->input_id.product = 0x0001;
	rcdev->input_id.version = 0x0100;
	rcdev->dev.parent = &pdev->dev;
	rcdev->driver_name = GPIO_IR_DRIVER_NAME;
	if (pdata->allowed_protos)
		rcdev->allowed_protocols = pdata->allowed_protos;
	else
		rcdev->allowed_protocols = RC_BIT_ALL;
	rcdev->map_name = pdata->map_name ?: RC_MAP_EMPTY;

	if(pdata->tx_gpio_nr > 0) {
        rcdev->s_tx_carrier = gpio_ir_tx_set_carrier;
        rcdev->s_tx_duty_cycle = gpio_ir_tx_set_duty_cycle;
        rcdev->tx_ir = gpio_ir_tx;
	}

	gpio_dev->rcdev = rcdev;
	gpio_dev->rx_gpio_nr = pdata->rx_gpio_nr;
	gpio_dev->rx_active_low = pdata->rx_active_low;
    gpio_dev->tx_gpio_nr = pdata->tx_gpio_nr;
	gpio_dev->tx_active_low = pdata->tx_active_low;
	gpio_dev->timing.frequency = DEFAULT_CARRIER_FREQUENCY;
	gpio_dev->timing.duty_cycle = DEFAULT_DUTY_CYCLE;

	rc = gpio_request(pdata->rx_gpio_nr, "gpio-ir-rx");
	if (rc < 0)
		goto err_gpio_request;
	rc  = gpio_direction_input(pdata->rx_gpio_nr);
	if (rc < 0)
		goto err_gpio_direction_input;

    if(pdata->tx_gpio_nr >= 0) {
    	rc = gpio_request(pdata->tx_gpio_nr, "gpio-ir-tx");
    	if (rc < 0)
    		goto err_gpio_request2;
        rc  = gpio_direction_output(pdata->tx_gpio_nr, pdata->tx_active_low ? 1 : 0);
    	if (rc < 0)
    		goto err_gpio_direction_output;
        gpio_ir_tx_calc_timing(gpio_dev);
    }

	rc = rc_register_device(rcdev);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to register rc device\n");
		goto err_register_rc_device;
	}

	platform_set_drvdata(pdev, gpio_dev);

    rc = request_any_context_irq(gpio_to_irq(pdata->rx_gpio_nr),
				gpio_ir_rx_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					"gpio-ir-rx-irq", gpio_dev);
	if (rc < 0)
		goto err_request_irq;

	return 0;

err_request_irq:
	rc_unregister_device(rcdev);
	rcdev = NULL;
err_register_rc_device:
err_gpio_direction_output:
	if(pdata->tx_gpio_nr >= 0) gpio_free(pdata->tx_gpio_nr);
err_gpio_request2:
err_gpio_direction_input:
	gpio_free(pdata->rx_gpio_nr);
err_gpio_request:
	rc_free_device(rcdev);
err_allocate_device:
	kfree(gpio_dev);
	return rc;
}


static int gpio_ir_remove(struct platform_device *pdev)
{
	struct gpio_rc_dev *gpio_dev = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(gpio_dev->rx_gpio_nr), gpio_dev);
	rc_unregister_device(gpio_dev->rcdev);
	gpio_free(gpio_dev->rx_gpio_nr);
	if(gpio_dev->tx_gpio_nr >= 0) gpio_free(gpio_dev->tx_gpio_nr);
	kfree(gpio_dev);
	return 0;
}

#ifdef CONFIG_PM
static int gpio_ir_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_rc_dev *gpio_dev = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev))
		enable_irq_wake(gpio_to_irq(gpio_dev->rx_gpio_nr));
	else
		disable_irq(gpio_to_irq(gpio_dev->rx_gpio_nr));

	return 0;
}

static int gpio_ir_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_rc_dev *gpio_dev = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev))
		disable_irq_wake(gpio_to_irq(gpio_dev->rx_gpio_nr));
	else
		enable_irq(gpio_to_irq(gpio_dev->rx_gpio_nr));

	return 0;
}

static const struct dev_pm_ops gpio_ir_pm_ops = {
	.suspend        = gpio_ir_suspend,
	.resume         = gpio_ir_resume,
};
#endif

static struct platform_driver gpio_ir_driver = {
	.probe  = gpio_ir_probe,
	.remove = gpio_ir_remove,
	.driver = {
		.name   = GPIO_IR_DRIVER_NAME,
		.of_match_table = of_match_ptr(gpio_ir_of_match),
#ifdef CONFIG_PM
		.pm	= &gpio_ir_pm_ops,
#endif
	},
};
module_platform_driver(gpio_ir_driver);

MODULE_DESCRIPTION("GPIO IR Transceive driver");
MODULE_LICENSE("GPL v2");
