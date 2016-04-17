/*
* Simple driver for Texas Instruments LM3630 Backlight driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/platform_data/lm3630_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <linux/mdss_io_util.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/08/27  Add for 14045 LCD */
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/boot_mode.h>
#endif /*VENDOR_EDIT*/

#define REG_CTRL	0x00
#define REG_CONFIG	0x01
#define REG_BRT_A	0x03
#define REG_BRT_B	0x04
#define REG_INT_STATUS	0x09
#define REG_INT_EN	0x0A
#define REG_FAULT	0x0B
#define REG_PWM_OUTLOW	0x12
#define REG_PWM_OUTHIGH	0x13
#define REG_MAX		0x1F
#define CABC_DISABLE_LEVEL_15009 0x28 //0x28
#define CABC_DISABLE_LEVEL_15037 0x14 //0x14



#define REG_MAXCU_A	0x05
#define REG_MAXCU_B	0x06
#define INT_DEBOUNCE_MSEC	10

#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/23  Add for register backlight info */
#define FILTER_STR 0x50
#define RUN_RAMP 0x08
#define REG_REVISION 0x1F
static bool pwm_flag = false;
static int backlight_level;
extern int cabc_mode;
int set_backlight_pwm(int state);
#endif /*VENDOR_EDIT*/
static struct lm3630_chip_data *lm3630_pchip;
#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/04/20  Add for 15009 lcd +-5v START */
static int disp_enp_gpio_15009 = 999;		//GPIO_97
static int disp_enn_gpio_15009 = 1000;		//GPIO_98
/* YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
#ifdef CONFIG_BL_REGISTER
enum lm3630_leds {
	BLED_ALL = 0,
	BLED_1,
	BLED_2
};

static const char * const bled_name[] = {
	[BLED_ALL] = "lm3630_bled",	/*Bank1 controls all string */
	[BLED_1] = "lm3630_bled1",	/*Bank1 controls bled1 */
	[BLED_2] = "lm3630_bled2",	/*Bank1 or 2 controls bled2 */
};
#endif 

struct lm3630_chip_data {
	struct device *dev;
	struct delayed_work work;
	int irq;
	struct workqueue_struct *irqthread;
	struct lm3630_platform_data *pdata;
	struct backlight_device *bled1;
	struct backlight_device *bled2;
	struct regmap *regmap;
};

/* initialize chip */
static int lm3630_chip_init(struct lm3630_chip_data *pchip)
{
	int ret;
	unsigned int reg_val;
	struct lm3630_platform_data *pdata = pchip->pdata;

#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/06/13  Add for add for cabc filter strength START */
	ret = regmap_write(pchip->regmap, FILTER_STR, 0x03);
	if (ret < 0)
		goto out;
#endif /*VENDOR_EDIT*/

	/*pwm control */
	reg_val = ((pdata->pwm_active & 0x01) << 2) | (pdata->pwm_ctrl & 0x03);
	ret = regmap_update_bits(pchip->regmap, REG_CONFIG, 0x07, reg_val);
	if (ret < 0)
		goto out;

	/* bank control */
	reg_val = ((pdata->bank_b_ctrl & 0x01) << 1) |
			(pdata->bank_a_ctrl & 0x07)|0x18;//linear;
    pr_err("%s REG_CTRL:0x%x\n", __func__, reg_val);
	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x1F, reg_val);
	if (ret < 0)
		goto out;

	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out;


		/* set initial current */
		pr_err("%s REG_MAX_A:20\n", __func__);
		ret = regmap_write(pchip->regmap, REG_MAXCU_A, 20);
		if (ret < 0)
			goto out;

		pr_err("%s REG_MAX_B:20\n", __func__);
		ret = regmap_write(pchip->regmap, REG_MAXCU_B, 20);
		if (ret < 0)
			goto out;


	if(is_project(OPPO_15035)){
		ret = regmap_update_bits(lm3630_pchip->regmap, REG_CONFIG, 0x01, 0x19);
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, REG_BRT_A, 0xFF);
		if (ret < 0)
			goto out;			
	}

	
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2015/06/03  Add for silence mode */
	if(MSM_BOOT_MODE__SILENCE == get_boot_mode())
		return 0;
#endif /*VENDOR_EDIT*/
	
	/* set initial brightness */
	if (pdata->bank_a_ctrl != BANK_A_CTRL_DISABLE) {
		ret = regmap_write(pchip->regmap,
				   REG_BRT_A, pdata->init_brt_led1);
		if (ret < 0)
			goto out;
	}

	if (pdata->bank_b_ctrl != BANK_B_CTRL_DISABLE) {
		ret = regmap_write(pchip->regmap,
				   REG_BRT_B, pdata->init_brt_led2);
		if (ret < 0)
			goto out;
	}
	return ret;

out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return ret;
}

/* interrupt handling */
static void lm3630_delayed_func(struct work_struct *work)
{
	int ret;
	unsigned int reg_val;
	struct lm3630_chip_data *pchip;

	pchip = container_of(work, struct lm3630_chip_data, work.work);

	ret = regmap_read(pchip->regmap, REG_INT_STATUS, &reg_val);
	if (ret < 0) {
		dev_err(pchip->dev,
			"i2c failed to access REG_INT_STATUS Register\n");
		return;
	}

	dev_info(pchip->dev, "REG_INT_STATUS Register is 0x%x\n", reg_val);
}

static irqreturn_t lm3630_isr_func(int irq, void *chip)
{
	int ret;
	struct lm3630_chip_data *pchip = chip;
	unsigned long delay = msecs_to_jiffies(INT_DEBOUNCE_MSEC);

	queue_delayed_work(pchip->irqthread, &pchip->work, delay);

	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out;

	return IRQ_HANDLED;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return IRQ_HANDLED;
}

static int lm3630_intr_config(struct lm3630_chip_data *pchip)
{
	INIT_DELAYED_WORK(&pchip->work, lm3630_delayed_func);
	pchip->irqthread = create_singlethread_workqueue("lm3630-irqthd");
	if (!pchip->irqthread) {
		dev_err(pchip->dev, "create irq thread fail...\n");
		return -1;
	}
	if (request_threaded_irq
	    (pchip->irq, NULL, lm3630_isr_func,
	     IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "lm3630_irq", pchip)) {
		dev_err(pchip->dev, "request threaded irq fail..\n");
		return -1;
	}
	return 0;
}


#ifdef CONFIG_BL_REGISTER
static bool
set_intensity(struct backlight_device *bl, struct lm3630_chip_data *pchip)
{
	if (!pchip->pdata->pwm_set_intensity)
		return false;
	pchip->pdata->pwm_set_intensity(bl->props.brightness - 1,
					pchip->pdata->pwm_period);
	return true;
}

/* update and get brightness */
static int lm3630_bank_a_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	/* brightness 0 means disable */
	if (!bl->props.brightness) {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x04, 0x00);
		if (ret < 0)
			goto out;
		return bl->props.brightness;
	}

	/* pwm control */
	if (pwm_ctrl == PWM_CTRL_BANK_A || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		if (!set_intensity(bl, pchip))
			dev_err(pchip->dev, "No pwm control func. in plat-data\n");
	} else {

		/* i2c control */
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_write(pchip->regmap,
				   REG_BRT_A, bl->props.brightness - 1);
		if (ret < 0)
			goto out;
	}
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access REG_CTRL\n");
	return bl->props.brightness;
}

static int lm3630_bank_a_get_brightness(struct backlight_device *bl)
{
	unsigned int reg_val;
	int brightness, ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_A || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		ret = regmap_read(pchip->regmap, REG_PWM_OUTHIGH, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val & 0x01;
		ret = regmap_read(pchip->regmap, REG_PWM_OUTLOW, &reg_val);
		if (ret < 0)
			goto out;
		brightness = ((brightness << 8) | reg_val) + 1;
	} else {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_read(pchip->regmap, REG_BRT_A, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val + 1;
	}
	bl->props.brightness = brightness;
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return 0;
}

static const struct backlight_ops lm3630_bank_a_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630_bank_a_update_status,
	.get_brightness = lm3630_bank_a_get_brightness,
};

static int lm3630_bank_b_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_B || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		if (!set_intensity(bl, pchip))
			dev_err(pchip->dev,
				"no pwm control func. in plat-data\n");
	} else {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_write(pchip->regmap,
				   REG_BRT_B, bl->props.brightness - 1);
	}
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return bl->props.brightness;
}

static int lm3630_bank_b_get_brightness(struct backlight_device *bl)
{
	unsigned int reg_val;
	int brightness, ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_B || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		ret = regmap_read(pchip->regmap, REG_PWM_OUTHIGH, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val & 0x01;
		ret = regmap_read(pchip->regmap, REG_PWM_OUTLOW, &reg_val);
		if (ret < 0)
			goto out;
		brightness = ((brightness << 8) | reg_val) + 1;
	} else {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_read(pchip->regmap, REG_BRT_B, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val + 1;
	}
	bl->props.brightness = brightness;

	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return bl->props.brightness;
}

static const struct backlight_ops lm3630_bank_b_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630_bank_b_update_status,
	.get_brightness = lm3630_bank_b_get_brightness,
};

static int lm3630_backlight_register(struct lm3630_chip_data *pchip,
				     enum lm3630_leds ledno)
{
	const char *name = bled_name[ledno];
	struct backlight_properties props;
	struct lm3630_platform_data *pdata = pchip->pdata;

	props.type = BACKLIGHT_RAW;
	switch (ledno) {
	case BLED_1:
	case BLED_ALL:
		props.brightness = pdata->init_brt_led1;
		props.max_brightness = pdata->max_brt_led1;
		pchip->bled1 =
		    backlight_device_register(name, pchip->dev, pchip,
					      &lm3630_bank_a_ops, &props);
		if (IS_ERR(pchip->bled1))
			return PTR_ERR(pchip->bled1);
		break;
	case BLED_2:
		props.brightness = pdata->init_brt_led2;
		props.max_brightness = pdata->max_brt_led2;
		pchip->bled2 =
		    backlight_device_register(name, pchip->dev, pchip,
					      &lm3630_bank_b_ops, &props);
		if (IS_ERR(pchip->bled2))
			return PTR_ERR(pchip->bled2);
		break;
	}
	return 0;
}

static void lm3630_backlight_unregister(struct lm3630_chip_data *pchip)
{
	if (pchip->bled1)
		backlight_device_unregister(pchip->bled1);
	if (pchip->bled2)
		backlight_device_unregister(pchip->bled2);
}
#else

extern struct regulator *vddf_2v5;
extern void set_bl_brightness(uint bl_level);
/* update and get brightness */
 int lm3630_bank_a_update_status(u32 bl_level)
{
	int ret;
#ifdef VENDOR_EDIT
/*lile@EXP.BasicDrv.LCD, 2015-07-10,  Add for 15069 HD JDI minimum backlight*/
	int temp_bl = 0;
#endif
	struct lm3630_chip_data *pchip = lm3630_pchip;
	pr_debug("%s: bl=%d\n", __func__,bl_level);
	
	if(!pchip){
		dev_err(pchip->dev, "lm3630_bank_a_update_status pchip is null\n");
		return -ENOMEM;
		}

    if (!pchip->regmap || !lm3630_pchip->regmap) {
        pr_err("%s YXQ pchip->regmap is NULL.\n", __func__);
        return bl_level;
    }




	
	/* brightness 0 means disable */
	if (!bl_level) {
		if(!(is_project(OPPO_15035))){
	        ret = regmap_write(lm3630_pchip->regmap, REG_BRT_A, 0);
		}else{
			set_bl_brightness(0);
		}
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x80);
		if (ret < 0)
			goto out;

#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/04/20  Add for 15009 factory mode big current close +-5v START */
		if(MSM_BOOT_MODE__FACTORY == get_boot_mode()&&is_project(OPPO_15009)){
			mdelay(230);
			if(gpio_is_valid(disp_enp_gpio_15009))
				gpio_set_value(disp_enp_gpio_15009, 0);	//GPIO_97
			else
				pr_err("%s: unvalid gpio disp_enp_gpio_15009 \n", __func__);
			mdelay(3);
			if(gpio_is_valid(disp_enn_gpio_15009))
				gpio_set_value(disp_enn_gpio_15009, 0); //GPIO_98
			else
				pr_err("%s: unvalid gpio disp_enn_gpio_15009 \n", __func__);
		}
/* YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/
		
		return bl_level;
	}


	/* pwm control */
	//bl_level=255;
		/* i2c control */
			ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
			if (ret < 0)
				goto out;
			mdelay(1);
#ifndef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/05/30  Add for 15009 disable cabc when brightness below 20 START */
		ret = regmap_write(pchip->regmap, REG_BRT_A, bl_level);
#else /*VENDOR_EDIT*/
		if(is_project(OPPO_15009)){
			if(bl_level>CABC_DISABLE_LEVEL_15009){
				ret = regmap_write(pchip->regmap,
					   REG_BRT_A, bl_level);
			}else if(bl_level<=CABC_DISABLE_LEVEL_15009){
#ifdef VENDOR_EDIT
/*lile@EXP.BasicDrv.LCD, 2015-07-10,  Add for 15069 HD JDI minimum backlight*/
				if((GET_MODEM_VERSION()!=103) && (get_PCB_Version() > 1)){
					temp_bl = (2+(bl_level-1)*29/40 );
					if(temp_bl< 6)
						temp_bl = 6;
					ret = regmap_write(pchip->regmap,
						REG_BRT_A, temp_bl );
				}
				else{
				    ret = regmap_write(pchip->regmap,
						REG_BRT_A, 2+(bl_level-1)*29/40);
				}
#else
				temp_bl = 2+(bl_level-1)*29/40;
				if(temp_bl < 4)
					temp_bl = 4;
				ret = regmap_write(pchip->regmap, REG_BRT_A, temp_bl);
#endif
				}
		}else{
			if(is_project(OPPO_15037)||is_project(OPPO_15035)){
#ifndef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/07/20  Delete for 15037 set 80% brightness  for ct test */
				if(is_project(OPPO_15037)){
				if(bl_level >= 4)
					bl_level = bl_level*8/10;
					pr_debug("%s:15037 bl=%d\n", __func__,bl_level);
				}
#endif /*VENDOR_EDIT*/

				if(bl_level < 4)
					bl_level = 4;
				}
				if(!(is_project(OPPO_15035))){
					ret = regmap_write(pchip->regmap, REG_BRT_A, bl_level);
				}else{
					set_bl_brightness(bl_level);
				}			
		}

		backlight_level =  bl_level;
		if(is_project(OPPO_15009)){
			if(bl_level <= CABC_DISABLE_LEVEL_15009 && pwm_flag==true){
				set_backlight_pwm(0);
			}else if(bl_level > CABC_DISABLE_LEVEL_15009 && pwm_flag==false && cabc_mode>0){
				set_backlight_pwm(1);
			}
		}

		if(is_project(OPPO_15037)){
			if(bl_level <= CABC_DISABLE_LEVEL_15037 && pwm_flag==true){
				set_backlight_pwm(0);
			}else if(bl_level > CABC_DISABLE_LEVEL_15037 && pwm_flag==false && cabc_mode>0){
				set_backlight_pwm(1);
			}
		}
#endif /*VENDOR_EDIT*/

        return bl_level;
out:
	dev_err(pchip->dev, "i2c failed to access REG_CTRL\n");
	return bl_level;
}

#endif
static const struct regmap_config lm3630_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int lm3630_dt(struct device *dev, struct lm3630_platform_data *pdata)
{
	u32 temp_val;
	int rc;
	struct device_node *np = dev->of_node;	
		dev_err(dev, "%s parse device tree\n", __func__);
		
	rc = of_property_read_u32(np, "ti,bank-a-ctrl", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read bank-a-ctrl\n");
		pdata->bank_a_ctrl=BANK_A_CTRL_ALL;
	} else{
		pdata->bank_a_ctrl=temp_val;
		printk("%s: bank_a_ctrl=%d\n", __func__,pdata->bank_a_ctrl);
	}
	rc = of_property_read_u32(np, "ti,init-brt-led1", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to readinit-brt-led1\n");
		pdata->init_brt_led1=200;
	} else{
		pdata->init_brt_led1=temp_val;
        printk("%s init_brt_led1=%d\n", __func__, pdata->init_brt_led1);
	}
	rc = of_property_read_u32(np, "ti,init-brt-led2", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read init-brt-led2\n");
		pdata->init_brt_led2=200;
	} else{
		pdata->init_brt_led2=temp_val;
        printk("%s init_brt_led2=%d\n", __func__, pdata->init_brt_led2);
	}
	rc = of_property_read_u32(np, "ti,max-brt-led1", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read max-brt-led1\n");
		pdata->max_brt_led1=255;
	} else{
		pdata->max_brt_led1=temp_val;
        printk("%s max_brt_led1=%d\n", __func__, pdata->max_brt_led1);
	}
	rc = of_property_read_u32(np, "ti,max-brt-led2", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read max-brt-led2\n");
		pdata->max_brt_led2=255;
	} else{
		pdata->max_brt_led2=temp_val;
        printk("%s max_brt_led2=%d\n", __func__, pdata->max_brt_led2);
	}
	rc = of_property_read_u32(np, "ti,pwm-active", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read pwm-active\n");
		pdata->pwm_active=PWM_ACTIVE_HIGH;
	} else{
		pdata->pwm_active=temp_val;
        printk("%s pwm_active=%d\n", __func__, pdata->pwm_active);
	}
	rc = of_property_read_u32(np, "ti,pwm-ctrl", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read pwm-ctrl\n");
		pdata->pwm_ctrl=PWM_CTRL_DISABLE;
	} else{
		pdata->pwm_ctrl=temp_val;
        printk("%s pwm_ctrl=%d\n", __func__, pdata->pwm_ctrl);
	}
	rc = of_property_read_u32(np, "ti,pwm-period", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read pwm-period\n");
		pdata->pwm_period=255;
	} else{
		pdata->pwm_period=temp_val;
        printk("%s pwm_period=%d\n", __func__, pdata->pwm_period);
	}

#ifndef VENDOR_EDIT
	    pdata->bl_en_gpio = of_get_named_gpio(np, "ti,bl-enable-gpio", 0);
#else /*VENDOR_EDIT*/
/* YongPeng.Yi@SWDP.MultiMedia, 2015/04/21  Add for 15009 DVT change bl gpio START */
	if(is_project(OPPO_15009)){
		if(get_PCB_Version()>1)
		    pdata->bl_en_gpio = of_get_named_gpio(np, "ti,bl-enable-gpio69", 0);
		else
		    pdata->bl_en_gpio = of_get_named_gpio(np, "ti,bl-enable-gpio", 0);
	}else{
		pdata->bl_en_gpio = of_get_named_gpio(np, "ti,bl-enable-gpio", 0);
	}
/* YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VEDNOR_EDIT*/
    if (!gpio_is_valid(pdata->bl_en_gpio))
		    pr_err("%s:%d, Backlight_en gpio not specified\n", __func__, __LINE__);
	
	
#if 0														
	pdata->bank_b_ctrl=BANK_B_CTRL_DISABLE;
	pdata->init_brt_led1=200;
	pdata->init_brt_led2=200;
	pdata->max_brt_led1=255;
	pdata->max_brt_led2=255;
	pdata->pwm_active=PWM_ACTIVE_HIGH;
	pdata->pwm_ctrl=PWM_CTRL_DISABLE;
	pdata->pwm_period=255;
#endif
	return 0;
}
#define LM3630_ENABLE_GPIO   69

static int enable_bl_i2c_1p8(struct device *dev) {
    int rc = 0;
    struct regulator *vreg;
    enum dss_vreg_type type;
    vreg = regulator_get(dev, "bl_i2c_1v8");
    rc = PTR_RET(vreg);
    if (rc) {
        DEV_ERR("%pS->%s: %s get failed. rc=%d\n",
					 __builtin_return_address(0), __func__,
					 "bl_i2c_1v8", rc);
        vreg = NULL;
    }
    type = (regulator_count_voltages(vreg) > 0)
					? DSS_REG_LDO : DSS_REG_VS;
	if (type == DSS_REG_LDO) {
		rc = regulator_set_voltage(
			vreg,
			1800000,
			1800000);
		if (rc < 0) {
			DEV_ERR("%pS->%s: %s set vltg fail\n",
				__builtin_return_address(0),
				__func__,
				"bl_i2c_1v8");
		}
	}

    rc = regulator_enable(vreg);

    return rc;
}

static int lm3630_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct lm3630_platform_data *pdata = client->dev.platform_data;
	struct lm3630_chip_data *pchip;
	int ret;
#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/23  Add for regist backlight info */
	unsigned int revision;
    static char *temp;
#endif /*VENDOR_EDIT*/
	
	pr_err("%s Enter\n", __func__);
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct lm3630_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = lm3630_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = client->dev.platform_data;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&client->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3630_chip_data),
			     GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	lm3630_pchip=pchip;
	
	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	/* Supply power for I2C start */
	if(!(is_project(OPPO_15009)||is_project(OPPO_15037))){
		enable_bl_i2c_1p8(&client->dev);
	}
    /* Supply power for I2C end */

//HW enable 
    ret = gpio_request(pdata->bl_en_gpio, "backlight_enable");
	if (ret) {
		pr_err("request enable gpio failed, ret=%d\n", ret);
	}
    pr_err("%s YXQ bl_en_gpio=0x%x\n", __func__, pdata->bl_en_gpio);

    if (gpio_is_valid(pdata->bl_en_gpio)){
	    gpio_set_value((pdata->bl_en_gpio), 1);
        gpio_direction_output((pdata->bl_en_gpio), 1);
        pr_err("%s YXQ bl_en_gpio --> 1\n", __func__);
    }

	pchip->regmap = devm_regmap_init_i2c(client, &lm3630_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	/* chip initialize */
	ret = lm3630_chip_init(pchip);
	if (ret < 0) {
		dev_err(&client->dev, "fail : init chip\n");
		goto err_chip_init;
	}

#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2014/09/23  Add for register backlight info */
	regmap_read(pchip->regmap,REG_REVISION,&revision);
    if (revision == 0x02) {
        temp = "02";
    } else {
        temp = "unknown";
    }
    register_device_proc("backlight", temp, "LM3630A");
#endif /*VENDOR_EDIT*/
#ifdef CONFIG_BL_REGISTER
	switch (pdata->bank_a_ctrl) {
	case BANK_A_CTRL_ALL:
		ret = lm3630_backlight_register(pchip, BLED_ALL);
		pdata->bank_b_ctrl = BANK_B_CTRL_DISABLE;
		break;
	case BANK_A_CTRL_LED1:
		ret = lm3630_backlight_register(pchip, BLED_1);
		break;
	case BANK_A_CTRL_LED2:
		ret = lm3630_backlight_register(pchip, BLED_2);
		pdata->bank_b_ctrl = BANK_B_CTRL_DISABLE;
		break;
	default:
		break;
	}

	if (ret < 0)
		goto err_bl_reg;

	if (pdata->bank_b_ctrl && pchip->bled2 == NULL) {
		ret = lm3630_backlight_register(pchip, BLED_2);
		if (ret < 0)
			goto err_bl_reg;
	}
#endif

	/* interrupt enable  : irq 0 is not allowed for lm3630 */
	pchip->irq = client->irq;
	printk("%s:yxq client irq =%d\n", __func__,client->irq);
	pchip->irq=0;
	if (pchip->irq)
		lm3630_intr_config(pchip);
	printk("%s:yxq----\n", __func__);
	dev_err(&client->dev, "LM3630 backlight register OK.\n");

	dev_info(&client->dev, "LM3630 backlight register OK.\n");
	return 0;
#ifdef CONFIG_BL_REGISTER
err_bl_reg:
	dev_err(&client->dev, "fail : backlight register.\n");
	lm3630_backlight_unregister(pchip);
#endif	
err_chip_init:
	return ret;
}

static int lm3630_remove(struct i2c_client *client)
{
	int ret;
	struct lm3630_chip_data *pchip = i2c_get_clientdata(client);

	ret = regmap_write(pchip->regmap, REG_BRT_A, 0);
	if (ret < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

	ret = regmap_write(pchip->regmap, REG_BRT_B, 0);
	if (ret < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");
#ifdef CONFIG_BL_REGISTER
	lm3630_backlight_unregister(pchip);
#endif
	if (gpio_is_valid(LM3630_ENABLE_GPIO))
		gpio_free(LM3630_ENABLE_GPIO);
	
	if (pchip->irq) {
		free_irq(pchip->irq, pchip);
		flush_workqueue(pchip->irqthread);
		destroy_workqueue(pchip->irqthread);
	}
	return 0;
}

static const struct i2c_device_id lm3630_id[] = {
	{LM3630_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3630_id);

static int lm3630_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc ;
	if(is_project(OPPO_15009)||is_project(OPPO_15037)||is_project(OPPO_15035)){
		return 0;
	}
	pr_err("%s: YXQ backlight suspend.\n", __func__);


	
    //rc = regmap_write(lm3630_pchip->regmap, REG_BRT_A, 0);
	rc  = regmap_update_bits(lm3630_pchip->regmap, REG_CTRL, 0x80, 0x80);
	if (rc  < 0)
	{
		pr_err("%s: unable to suspend !!!!!!!!!!!!\n", __func__);
	}
//	rc = gpio_direction_output(LM3630_ENABLE_GPIO, 0);
//	if (rc) {
//		pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
//		return rc;
//	}

	return 0;
}
static int lm3630_resume(struct i2c_client *client)
{
	int rc ;

	pr_err("%s: YXQ backlight resume.\n", __func__);
    //rc = regmap_write(lm3630_pchip->regmap, REG_BRT_A, 0);
	if(is_project(OPPO_15009)||is_project(OPPO_15037)||is_project(OPPO_15035)){
		//Xiaori.yuan add for temp power secquence
		return 0;
		//add end
	 	if(lm3630_pchip!=NULL){
		     if (gpio_is_valid(lm3630_pchip->pdata->i2c_1v8_gpio_14045)){
		 	     gpio_set_value((lm3630_pchip->pdata->i2c_1v8_gpio_14045), 1);
		         gpio_direction_output((lm3630_pchip->pdata->i2c_1v8_gpio_14045), 1);
		         //pr_err("%s yxr i2c_1v8_gpio_14045 --> 1\n", __func__);
		     }else{
		     	 pr_err("%s yxr i2c_1v8_gpio_14045 is not specfied\n", __func__);
		     }
	 	}

	}
	
	rc  = regmap_update_bits(lm3630_pchip->regmap, REG_CTRL, 0x80, 0x00);
	if (rc  < 0)
	{
		pr_err("%s: unable to resume !!!!!!!!!!!!\n", __func__);
	}	
//	rc = gpio_direction_output(LM3630_ENABLE_GPIO, 1);
//	if (rc) {
//		pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
//		return rc;
//	}

	return 0;
}

#ifdef VENDOR_EDIT
/* YongPeng.Yi@SWDP.MultiMedia, 2015/05/19  Add for set cabc START */
int set_backlight_pwm(int state)
{
	int rc = 0;

	if(is_project(OPPO_15035))
		return 0;

	if (get_boot_mode() == MSM_BOOT_MODE__NORMAL) {
		if( state == 1 && backlight_level <= CABC_DISABLE_LEVEL_15009 ) return rc;
		if(state == 1)
		{
			if(pwm_flag == false){
				rc = regmap_update_bits(lm3630_pchip->regmap, REG_CONFIG, 0x01, 0x19);
				pwm_flag = true;
			}
		}
		else
		{
			if(pwm_flag == true){
				rc = regmap_update_bits(lm3630_pchip->regmap, REG_CONFIG, 0x01, 0x18);
				pwm_flag = false;
			}
		}
	}
	return rc;
}
/* YongPeng.Yi@SWDP.MultiMedia END */
#endif /*VENDOR_EDIT*/

static struct i2c_driver lm3630_i2c_driver = {
	.driver = {
		   .name = LM3630_NAME,
		   },
	.probe = lm3630_probe,
	.remove = lm3630_remove,
	.suspend = lm3630_suspend,
	.resume = lm3630_resume,
	.id_table = lm3630_id,
};

module_i2c_driver(lm3630_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Backlight driver for LM3630");
MODULE_AUTHOR("G.Shark Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");
