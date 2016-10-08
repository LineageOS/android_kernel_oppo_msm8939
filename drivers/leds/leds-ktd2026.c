/************************************************************************************
** File: - android\kernel\drivers\misc\ktd_shineled\ktd2026_driver.c
** CONFIG_MACH_OPPO
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
**
** Description:
**      breath-led KTD2026's driver for oppo
**      can support three kinds of leds if HW allows.
** Version: 1.0
** Date created: 18:35:46,11/11/2014
** Author: Tong.han@Bsp.group.TP&BL
**
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** Tong.han@Bsp.group.TP&BL ,2014-11-11 , Add this driver for breath-LED for project 14051.
************************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h>
#include <linux/cdev.h>
#include <linux/leds.h>

#define KTD_I2C_NAME	"ktd2026"

struct ktd20xx_data;

struct ktd20xx_led {
	int brightness;
	struct ktd20xx_data *data;
	struct led_classdev ldev;
};

struct ktd20xx_data {
	struct i2c_client *client;
	struct ktd20xx_led leds[3];
	int max_brightness_reg_value;
	int total_ms;
	int on_ms;
};

static const char * ktd20xx_led_names[] = {
	"red", "green", "blue"
};

#define ldev_to_led(c) container_of(c, struct ktd20xx_led, ldev)
#define dev_to_data(d) i2c_get_clientdata(to_i2c_client(d))

#define LED1_OUT_ALON  0x1   //always on
#define LED1_OUT_PWM1 0x2
#define LED2_OUT_ALON  0x1<<2
#define LED2_OUT_PWM1 0x2<<2
#define LED3_OUT_ALON  0x1<<4
#define LED3_OUT_PWM1 0x2<<4

#define LED_OUT_ALON  (LED2_OUT_ALON)
#define LED_OUT_PWM1 (LED2_OUT_PWM1)

static void ktd20xx_set_brightness(struct led_classdev *led_cdev,
				   enum led_brightness value)
{
	struct ktd20xx_led *led = ldev_to_led(led_cdev);

	led->brightness = (value * led->data->max_brightness_reg_value) /
		led_cdev->max_brightness;
}

static ssize_t ktd20xx_grppwm_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ktd20xx_data *data = dev_to_data(dev);
	unsigned long value = simple_strtoul(buf, NULL, 10);

	data->on_ms = value * data->total_ms / 255;
	if (data->on_ms > 9000)
		data->on_ms = 9000;

	return count;
}

static ssize_t ktd20xx_grpfreq_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ktd20xx_data *data = dev_to_data(dev);
	unsigned long value = simple_strtoul(buf, NULL, 10);

	data->total_ms = value * 50;
	if (data->total_ms > 16000)
		data->total_ms = 16000;

	return count;
}

static ssize_t ktd20xx_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ktd20xx_data *data = dev_to_data(dev);
	bool value = !!simple_strtoul(buf, NULL, 10);
	int i;

	if (value && data->total_ms && data->on_ms) {
		int total_ms_reg = (data->total_ms - 256) / 128;
		int on_ms_reg = (data->on_ms / 3 / 192) & 0x0F;
		int stable_ms_reg = data->on_ms * 250 / 3 / data->total_ms * 2;

		on_ms_reg = on_ms_reg | (on_ms_reg << 4);

		// initialization LED off
		i2c_smbus_write_byte_data(data->client, 0x00, 0x20);
		i2c_smbus_write_byte_data(data->client, 0x04, 0x00);
		// set drive strengths
		for (i = 0; i < ARRAY_SIZE(data->leds); i++) {
			i2c_smbus_write_byte_data(data->client, 0x06 + i,
				data->leds[i].brightness);
		}
		// raise time
		i2c_smbus_write_byte_data(data->client, 0x05, on_ms_reg);
		// dry flash period
		i2c_smbus_write_byte_data(data->client, 0x01, total_ms_reg);
		//reset internal counter stableMS_reg
		i2c_smbus_write_byte_data(data->client, 0x02, 0x00);
		//allocate led1 to timer1
		i2c_smbus_write_byte_data(data->client, 0x04, LED_OUT_PWM1);
		//led flashing(current ramp-up and down countinuously)
		i2c_smbus_write_byte_data(data->client, 0x02, stable_ms_reg);
	} else {
		int total_brightness = 0;
		for (i = 0; i < ARRAY_SIZE(data->leds); i++) {
			total_brightness += data->leds[i].brightness;
		}
		if (total_brightness == 0) {
			// set current to 0.125mA
			i2c_smbus_write_byte_data(data->client, 0x06, 0x00);
			// turn off leds
			i2c_smbus_write_byte_data(data->client, 0x04, 0x00);
		} else {
			// set drive strengths
			for (i = 0; i < ARRAY_SIZE(data->leds); i++) {
				i2c_smbus_write_byte_data(data->client, 0x06 + i,
					data->leds[i].brightness);
			}
			//turn on all leds
			i2c_smbus_write_byte_data(data->client, 0x04, LED_OUT_ALON);
		}
	}

	return count;
}

static DEVICE_ATTR(grppwm, S_IWUSR | S_IRUGO, NULL, ktd20xx_grppwm_store);
static DEVICE_ATTR(grpfreq, S_IWUSR | S_IRUGO, NULL, ktd20xx_grpfreq_store);
static DEVICE_ATTR(blink, S_IWUSR | S_IRUGO, NULL, ktd20xx_blink_store);

static struct attribute *blink_attributes[] = {
    &dev_attr_grppwm.attr,
    &dev_attr_grpfreq.attr,
    &dev_attr_blink.attr,
    NULL
};

static const struct attribute_group blink_attr_group = {
    .attrs = blink_attributes,
};

static int ktd20xx_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ktd20xx_data *data;
	u32 max_brightness;
	int ret, i;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"%s: check_functionality failed.", __func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(client->dev.of_node,
		"ktd2026,max_brightness", &max_brightness);
	if (ret != 0)
		max_brightness = 255;

	data = devm_kzalloc(&client->dev, sizeof(struct ktd20xx_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->max_brightness_reg_value = max_brightness;
	i2c_set_clientdata(client, data);

	for (i = 0; i < ARRAY_SIZE(data->leds); i++) {
		struct ktd20xx_led *led = &data->leds[i];
		led->data = data;
		led->ldev.name = ktd20xx_led_names[i];
		led->ldev.brightness_set = ktd20xx_set_brightness;

		ret = led_classdev_register(&client->dev, &led->ldev);
		if (ret < 0)
			goto fail_unregister_leds;
	}

	ret = sysfs_create_group(&client->dev.kobj, &blink_attr_group);
	if (ret) {
		dev_err(&client->dev, "%s : sysfs_create_group failed!\n", __func__);
		goto fail_unregister_leds;
	}

	ret = i2c_smbus_write_byte_data(client, 0x06, 0x00); //set current is 0.125mA
	if (ret == 0)
		ret = i2c_smbus_write_byte_data(client, 0x04, 0x00); //turn off leds
	if (ret < 0) {
		dev_err(&client->dev, "Failed initializing KTD2026\n");
		goto fail_sysfs_remove;
	}

	return 0;

fail_sysfs_remove:
	sysfs_remove_group(&client->dev.kobj, &blink_attr_group);
fail_unregister_leds:
	for (i = 0; i < ARRAY_SIZE(ktd20xx_led_names); i++) {
		if (data->leds[i].ldev.name)
			led_classdev_unregister(&data->leds[i].ldev);
	}
	kfree(data);
	i2c_set_clientdata(client, NULL);

	return ret;
}

static int ktd20xx_remove(struct i2c_client *client)
{
	struct ktd20xx_data *data = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < ARRAY_SIZE(data->leds); i++) {
		led_classdev_unregister(&data->leds[i].ldev);
	}

	sysfs_remove_group(&client->dev.kobj, &blink_attr_group);
	kfree(data);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ktd2xx_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};


static struct of_device_id ktd20xx_match_table[] = {
	{ .compatible = "ktd,ktd2026",},
	{ },
};

static struct i2c_driver ktd20xx_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd20xx_match_table,
	},
	.probe = ktd20xx_probe,
	.remove = ktd20xx_remove,
	.id_table = ktd2xx_id,
};

module_i2c_driver(ktd20xx_driver);

MODULE_LICENSE("GPL");

