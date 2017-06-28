/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include <linux/proc_fs.h>


static struct mutex flash_mode_lock;
struct delayed_work led_blink_work;
bool blink_test_status;
extern bool camera_power_status;

#define FLASH_NAME "ti,lm3642"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define LM3642_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LM3642_DBG(fmt, args...)
#endif


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{0x0A, 0x00},
	{0x08, 0x04},
	{0x09, 0x1A},
};

static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	{0x0A, 0x12},
	{0x08, 0x04},
	{0x09, 0x1A},
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	{0x0A, 0x23},
	{0x08, 0x04},
	{0x09, 0x1A},
};


static const struct of_device_id lm3642_i2c_trigger_dt_match[] = {
	{.compatible = "ti,lm3642"},
	{}
};

MODULE_DEVICE_TABLE(of, lm3642_i2c_trigger_dt_match);
static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3642_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

int msm_flash_lm3642_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return 0;
}

int msm_flash_lm3642_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3642_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return 0;
}

int msm_flash_lm3642_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	return rc;
}

int msm_flash_lm3642_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;


	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_lm3642_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
#ifdef CONFIG_MACH_OPPO
/*OPPO 2014-08-01 hufeng add for flash engineer mode test*/
struct regulator *vreg;
int led_test_mode;
static int msm_led_cci_test_init(void)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	//struct regulator *vreg;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

#ifdef CONFIG_MACH_OPPO
/* xianglie.liu 2014-09-05 Add for lm3642 14043 and 14045 use gpio-vio */
/* zhengrong.zhang 2014-11-08 Add for gpio contrl lm3642 */
	if (camera_power_status) {
		LM3642_DBG("%s:%d camera already power up\n", __func__, __LINE__);
		return rc;
	}
	msm_flash_led_init(&fctrl);

	if (power_info->cam_vreg != NULL && power_info->num_vreg>0)
	{
		msm_camera_config_single_vreg(&fctrl.pdev->dev,
			power_info->cam_vreg,
			&vreg,1);
	}
	else
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_VIO],
			GPIO_OUT_HIGH);
	}
#endif
	LM3642_DBG("%s exit\n", __func__);
	return rc;
}
static int msm_led_cci_test_off(void)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);
	if (led_test_mode == 2)
		cancel_delayed_work_sync(&led_blink_work);
	if (fctrl.flash_i2c_client && fctrl.reg_setting)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			fctrl.reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
#ifdef CONFIG_MACH_OPPO
/* xianglie.liu 2014-09-19 Add for fix ftm mode cannot sleep */
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
#endif
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	/*disable power*/
#ifdef CONFIG_MACH_OPPO
/* xianglie.liu 2014-09-18 Add for lm3642 14043 and 14045 use gpio-vio */
/* zhengrong.zhang 2014-11-08 Add for gpio contrl lm3642 */
	if (camera_power_status) {
		LM3642_DBG("%s:%d camera already power up\n", __func__, __LINE__);
		return rc;
	}
	if (power_info->cam_vreg != NULL && power_info->num_vreg>0)
	{
		msm_camera_config_single_vreg(&fctrl.pdev->dev,
			power_info->cam_vreg,
			&vreg,0);
	}
	else
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_VIO],
			GPIO_OUT_LOW);
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		//return rc;
	}

/* xianglie.liu 2014-09-19 Add for fix ftm mode cannot sleep */
	/* CCI deInit */
	if (fctrl.flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl.flash_i2c_client, 1);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
		}
	}
#endif
	return rc;
}
static void msm_led_cci_test_blink_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	if (blink_test_status)
	{
		msm_flash_led_low(&fctrl);
	}
	else
	{
		msm_flash_led_off(&fctrl);
	}
	blink_test_status = !blink_test_status;
	schedule_delayed_work(dwork, msecs_to_jiffies(1100));
}

/* zhengrong.zhang 2014-11-08 Add for flash proc read */
static ssize_t flash_proc_read(struct file *filp, char __user *buff,
				size_t len, loff_t *data)
{
    char value[2] = {0};

    snprintf(value, sizeof(value), "%d", led_test_mode);
    return simple_read_from_buffer(buff, len, data, value,1);
}

static ssize_t flash_proc_write(struct file *filp, const char __user *buff,
				size_t len, loff_t *data)
{
	char buf[8] = {0};
	int new_mode = 0;
	if (len > 8)
		len = 8;
	if (copy_from_user(buf, buff, len))
	{
		pr_err("proc write error.\n");
		return -EFAULT;
	}
	new_mode = simple_strtoul(buf, NULL, 10);
	if (new_mode == led_test_mode)
	{
		pr_err("the same mode as old\n");
		return len;
	}
	switch (new_mode) {
	case 0:
		mutex_lock(&flash_mode_lock);
		if (led_test_mode > 0 && led_test_mode <= 3)
			msm_led_cci_test_off();
		led_test_mode = 0;
		mutex_unlock(&flash_mode_lock);
		break;
	case 1:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_init();
		led_test_mode = 1;
		mutex_unlock(&flash_mode_lock);
		msm_flash_led_low(&fctrl);

		break;
	case 2:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_init();
		led_test_mode = 2;
		mutex_unlock(&flash_mode_lock);

		schedule_delayed_work(&led_blink_work, msecs_to_jiffies(50));
		break;
	case 3:
		mutex_lock(&flash_mode_lock);
		msm_led_cci_test_init();
		led_test_mode = 3;
		mutex_unlock(&flash_mode_lock);
		msm_flash_led_high(&fctrl);
		break;
	default:
#ifdef CONFIG_MACH_OPPO
/*OPPO hufeng 2014-09-02 add for individual flashlight*/
		mutex_lock(&flash_mode_lock);
		led_test_mode = new_mode;
		mutex_unlock(&flash_mode_lock);
#endif
		pr_err("invalid mode %d\n", led_test_mode);
		break;
	}
	return len;
}
static const struct file_operations led_test_fops = {
    .owner		= THIS_MODULE,
    .read		= flash_proc_read,
    .write		= flash_proc_write,
};
static int flash_proc_init(struct msm_led_flash_ctrl_t *flash_ctl)
{
	int ret=0;
	struct proc_dir_entry *proc_entry;

	INIT_DELAYED_WORK(&led_blink_work, msm_led_cci_test_blink_work);
	proc_entry = proc_create_data( "qcom_flash", 0666, NULL,&led_test_fops, (void*)&fctrl);
	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
		pr_err("[%s]: Error! Couldn't create qcom_flash proc entry\n", __func__);
	}
	return ret;
}
#endif /* CONFIG_MACH_OPPO */
static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3642_DBG("%s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_lm3642_i2c_probe: id is NULL");
		id = lm3642_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	if (!rc)
		msm_lm3642_torch_create_classdev(&(client->dev),NULL);
	return rc;
}

static int msm_flash_lm3642_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3642_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return rc;
}


static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = msm_flash_lm3642_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_i2c_trigger_dt_match,
	},
};
#ifdef CONFIG_MACH_OPPO
/*OPPO hufeng 2014-07-24 add for flash cci driver*/
static const struct of_device_id lm3642_trigger_dt_match[] =
{
	{.compatible = FLASH_NAME, .data = &fctrl},
	{}
};
static int msm_flash_lm3642_platform_probe(struct platform_device *pdev)
{
	int rc;
	const struct of_device_id *match;
#ifdef CONFIG_MACH_OPPO
/*OPPO 2014-11-11 zhengrong.zhang add for torch can't use when open subcamera after boot*/
	struct msm_camera_power_ctrl_t *power_info = NULL;
#endif
	LM3642_DBG("%s entry\n", __func__);
	match = of_match_device(lm3642_trigger_dt_match, &pdev->dev);
	if (!match)
	{
		pr_err("%s, of_match_device failed!\n", __func__);
		return -EFAULT;
	}
	LM3642_DBG("%s of_match_device success\n", __func__);
	rc = msm_flash_probe(pdev, match->data);
#ifdef CONFIG_MACH_OPPO
/*OPPO hufeng 2014-09-02 add for individual flashlight*/
	mutex_init(&flash_mode_lock);
/*OPPO 2014-08-01 hufeng add for flash engineer mode test*/
	flash_proc_init(&fctrl);

/*OPPO 2014-11-11 zhengrong.zhang add for torch can't use when open subcamera after boot*/
	power_info = &fctrl.flashdata->power_info;
	rc = msm_camera_request_gpio_table(
	power_info->gpio_conf->cam_gpio_req_tbl,
	power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	}
	rc = msm_camera_request_gpio_table(
	power_info->gpio_conf->cam_gpio_req_tbl,
	power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	}
#endif

	return rc;
}

static struct platform_driver lm3642_platform_driver =
{
	.probe = msm_flash_lm3642_platform_probe,
	.driver =
	{
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};
#endif
static int __init msm_flash_lm3642_init(void)
{
#ifdef CONFIG_MACH_OPPO
/*OPPO hufeng 2014-07-24 add for flash cci probe*/
	int32_t rc = 0;
#endif
	LM3642_DBG("%s entry\n", __func__);
#ifdef CONFIG_MACH_OPPO
/*OPPO hufeng 2014-07-24 add for flash cci probe*/
	rc = platform_driver_register(&lm3642_platform_driver);
	LM3642_DBG("%s after entry\n", __func__);
	if (!rc)
		return rc;
	pr_debug("%s:%d rc %d\n", __func__, __LINE__, rc);
#endif
	return i2c_add_driver(&lm3642_i2c_driver);
}

static void __exit msm_flash_lm3642_exit(void)
{
	LM3642_DBG("%s entry\n", __func__);
#ifdef CONFIG_MACH_OPPO
/*OPPO hufeng 2014-07-24 add for flash cci probe*/
	platform_driver_unregister(&lm3642_platform_driver);
#endif
	i2c_del_driver(&lm3642_i2c_driver);
	return;
}


static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
};

static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
#ifdef CONFIG_MACH_OPPO
/*OPPO 2014-07-24 modify for flash cci driver*/
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
#else
	.flash_led_init = msm_flash_lm3642_led_init,
	.flash_led_release = msm_flash_lm3642_led_release,
	.flash_led_off = msm_flash_lm3642_led_off,
	.flash_led_low = msm_flash_lm3642_led_low,
	.flash_led_high = msm_flash_lm3642_led_high,
#endif
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
};

module_init(msm_flash_lm3642_init);
module_exit(msm_flash_lm3642_exit);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
