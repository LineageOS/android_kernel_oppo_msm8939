/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/wakelock.h>
#include <soc/qcom/scm.h>

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250
#define FPC_TTW_HOLD_TIME 1000

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vdd_io", 1800000UL, 1800000UL, 6000, },
};

struct fpc1020_data {
	struct device *dev;
	int irq_gpio;
	int rst_gpio;
	int irq_num;
	struct mutex lock;
	bool prepared;
	int irq_enabled;

	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct wake_lock ttw_wl;
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
};

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_info(dev, "%s - gpio: %d\n", label, *gpio);
	return 0;
}

static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->vreg); i++) {
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = fpc1020->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				dev_err(dev, "Unable to get  %s\n", name);
				return PTR_ERR(vreg);
			}
		}
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fpc1020->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fpc1020->vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

static DEFINE_SPINLOCK(fpc1020_lock);

static int fpc1020_enable_irq(struct fpc1020_data *fpc1020, bool enable)
{
	spin_lock_irq(&fpc1020_lock);
	if (enable) {
		if (!fpc1020->irq_enabled) {
			enable_irq(gpio_to_irq(fpc1020->irq_gpio));
			enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
			fpc1020->irq_enabled = 1;
			dev_info(fpc1020->dev, "%s: enabled\n", __func__);
		} else {
			dev_info(fpc1020->dev, "%s: no need to enable\n", __func__);
		}
	} else {
		if (fpc1020->irq_enabled) {
			disable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
			disable_irq_nosync(gpio_to_irq(fpc1020->irq_gpio));
			fpc1020->irq_enabled = 0;
			dev_info(fpc1020->dev, "%s: disabled\n", __func__);
		} else {
			dev_info(fpc1020->dev, "%s: no need to disable\n", __func__);
		}
	}
	spin_unlock_irq(&fpc1020_lock);

	return 0;
}

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attribute, const char *buffer, size_t count)
{
	int op = 0;
	bool enable;
	int rc = 0;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (1 == sscanf(buffer, "%d", &op)) {
		if (op == 1)
			enable = true;
		else if (op == 0)
			enable = false;
	} else {
		pr_err("invalid content: '%s', length = %zd\n", buffer, count);
		return -EINVAL;
	}
	rc = vreg_setup(fpc1020, "vdd_io", enable);
	return rc ? rc : count;
}

static ssize_t irq_enable_set(struct device *dev,
	struct device_attribute *attribute, const char *buffer, size_t count)
{
	int op = 0;
	bool enable;
	int rc = 0;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (1 == sscanf(buffer, "%d", &op)) {
		if (op == 1)
			enable = true;
		else if (op == 0)
			enable = false;
	} else {
		pr_err("invalid content: '%s', length = %zd\n", buffer, count);
		return -EINVAL;
	}
	rc = fpc1020_enable_irq(fpc1020,  enable);
	return rc ? rc : count;
}

static ssize_t irq_enable_get(struct device* dev,
			     struct device_attribute* attribute,
			     char* buffer)
{
	struct fpc1020_data* fpc1020 = dev_get_drvdata(dev);
	return scnprintf(buffer, PAGE_SIZE, "%i\n",fpc1020->irq_enabled);
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);
static DEVICE_ATTR(regulator_enable, S_IWUSR, NULL, regulator_enable_set);
static DEVICE_ATTR(irq_enable, S_IWUSR, irq_enable_get, irq_enable_set);

static struct attribute *attributes[] = {
	&dev_attr_irq.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_irq_enable.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	smp_rmb();

	wake_lock_timeout(&fpc1020->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	int irqf;
	struct device_node *np = dev->of_node;

	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,irq-gpio",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;

	rc = gpio_direction_input(fpc1020->irq_gpio);

	if (rc) {
		dev_err(fpc1020->dev,
			"gpio_direction_input (irq) failed.\n");
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,reset-gpio",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}

	disable_irq_nosync(gpio_to_irq(fpc1020->irq_gpio));
	fpc1020->irq_enabled = 0;

	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	rc = gpio_direction_output(fpc1020->rst_gpio, 1);

	if (rc) {
		dev_err(fpc1020->dev,
			"gpio_direction_output (reset) failed.\n");
		goto exit;
	}
	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH1_US);

	gpio_set_value(fpc1020->rst_gpio, 0);
	udelay(FPC1020_RESET_LOW_US);

	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH2_US);

	rc = vreg_setup(fpc1020, "vdd_io", true);
	if (rc)
		dev_err(fpc1020->dev, "vdd_io vreg_setup failed.\n");
exit:
	return rc;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020-qsee4", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name		= "fpc1020",
		.owner		= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe = fpc1020_probe,
};
module_platform_driver(fpc1020_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_AUTHOR("Martin Trulsson <martin.trulsson@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
