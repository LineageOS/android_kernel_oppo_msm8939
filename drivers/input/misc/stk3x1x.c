/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : stk3x1x.c
 ** Description : 
 ** Date        : 2014-08-04 19:33
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

/*
 *  stk3x1x.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x
 *  proximity/ambient light sensor
 *
 *  Copyright (c) 2013, The Linux Foundation. All Rights Reserved.
 *  Copyright (C) 2012 Lex Hsieh / sensortek <lex_hsieh@sitronix.com.tw> or
 *   <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Linux Foundation chooses to take subject only to the GPLv2 license
 *  terms, and distributes only under these terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include "linux/stk3x1x.h"
#include <linux/sensors_ftm.h>
#include <soc/oppo/oppo_project.h>

#define DRIVER_VERSION  "3.4.4ts"

#define STK_ALS_CHANGE_THD	20	/* The threshold to trigger ALS interrupt, unit: lux */
#define STK_INT_PS_MODE			1	/* 1, 2, or 3	*/

/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_PSCTRL_REG 			0x01
#define STK_ALSCTRL_REG 		0x02
#define STK_LEDCTRL_REG 		0x03
#define STK_INT_REG 			0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_THDH1_ALS_REG 		0x0A
#define STK_THDH2_ALS_REG 		0x0B
#define STK_THDL1_ALS_REG 		0x0C
#define STK_THDL2_ALS_REG 		0x0D
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_OFFSET_REG 	0x15
#define STK_DATA2_OFFSET_REG 	0x16
#define STK_DATA1_IR_REG 		0x17
#define STK_DATA2_IR_REG 		0x18
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80


/* Define state reg */
#define STK_STATE_EN_IRS_SHIFT  	7
#define STK_STATE_EN_AK_SHIFT  		6
#define STK_STATE_EN_ASO_SHIFT  	5
#define STK_STATE_EN_IRO_SHIFT  	4
#define STK_STATE_EN_WAIT_SHIFT  	2
#define STK_STATE_EN_ALS_SHIFT  	1
#define STK_STATE_EN_PS_SHIFT  		0

#define STK_STATE_EN_IRS_MASK	0x80
#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_ALS_MASK	0x02
#define STK_STATE_EN_PS_MASK	0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT  		6
#define STK_PS_GAIN_SHIFT  		4
#define STK_PS_IT_SHIFT  		0

#define STK_PS_PRS_MASK			0xC0
#define STK_PS_GAIN_MASK		0x30
#define STK_PS_IT_MASK			0x0F

/* Define ALS ctrl reg */
#define STK_ALS_PRS_SHIFT  		6
#define STK_ALS_GAIN_SHIFT  	4
#define STK_ALS_IT_SHIFT  		0

#define STK_ALS_PRS_MASK		0xC0
#define STK_ALS_GAIN_MASK		0x30
#define STK_ALS_IT_MASK			0x0F

/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT  	6
#define STK_LED_DT_SHIFT  		0

#define STK_LED_IRDR_MASK		0xC0
#define STK_LED_DT_MASK			0x3F

/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT  	7
#define STK_INT_OUI_SHIFT  		4
#define STK_INT_ALS_SHIFT  		3
#define STK_INT_PS_SHIFT  		0

#define STK_INT_CTRL_MASK		0x80
#define STK_INT_OUI_MASK		0x10
#define STK_INT_ALS_MASK		0x08
#define STK_INT_PS_MASK			0x07

#define STK_INT_ALS				0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT  		7
#define STK_FLG_PSDR_SHIFT  		6
#define STK_FLG_ALSINT_SHIFT  		5
#define STK_FLG_PSINT_SHIFT  		4
#define STK_FLG_OUI_SHIFT  			2
#define STK_FLG_IR_RDY_SHIFT  		1
#define STK_FLG_NF_SHIFT  			0

#define STK_FLG_ALSDR_MASK		0x80
#define STK_FLG_PSDR_MASK		0x40
#define STK_FLG_ALSINT_MASK		0x20
#define STK_FLG_PSINT_MASK		0x10
#define STK_FLG_OUI_MASK		0x04
#define STK_FLG_IR_RDY_MASK		0x02
#define STK_FLG_NF_MASK			0x01

/* misc define */
#define MIN_ALS_POLL_DELAY_NS	110000000

#define DEVICE_NAME		"stk_ps"
#define ALS_NAME		"light"
#define PS_NAME "proximity"

#define STK3X1X_LOG(format, args...) printk(KERN_ERR DEVICE_NAME " "format,##args)

/* POWER SUPPLY VOLTAGE RANGE */
#define STK3X1X_VDD_MIN_UV	2000000
#define STK3X1X_VDD_MAX_UV	3300000
#define STK3X1X_VIO_MIN_UV	1750000
#define STK3X1X_VIO_MAX_UV	1950000

#define STK_FIR_LEN 4
#define MAX_FIR_LEN 32

#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/08/04, add for dynamic threshold */
#define STK3X1X_ALSPS_DYNAMIC_THRESHOLD
#endif /*VENDOR_EDIT*/

#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD
static int ps_min = 0;
static int ps_adjust_max = 2200;
static int dirty_adjust_low_thd = 200, dirty_adjust_high_thd = 250;
static int ps_thd_low_highlight = 2100, ps_thd_high_highlight = 2150;

static struct delayed_work sample_ps_work;
static DECLARE_WAIT_QUEUE_HEAD(enable_wq);

#endif

static struct stk3x1x_data *g_ps_data = NULL;
static int g_is_resumed = 1;
static int g_low_thd = -1, g_high_thd = -1;

static struct sensors_classdev sensors_light_cdev = {
	.name = "stk3x1x-light",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6500",
	.resolution = "0.0625",
	.sensor_power = "0.09",
	.min_delay = (MIN_ALS_POLL_DELAY_NS / 1000),	/* us */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "stk3x1x-proximity",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct data_filter {
	u16 raw[MAX_FIR_LEN];
	int sum;
	int number;
	int idx;
};

struct stk3x1x_data {
	struct i2c_client *client;
	struct stk3x1x_platform_data *pdata;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	int32_t irq;
	struct work_struct stk_work;
	struct workqueue_struct *stk_wq;
	int		int_pin;
	uint8_t wait_reg;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	struct mutex io_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	struct wake_lock ps_wakelock;
	struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;
	bool als_enabled;
	unsigned int als_enable_state;      /* save sensor enabling state for resume */
	struct hrtimer als_timer;
	ktime_t als_poll_delay;

	struct work_struct stk_als_work;
	struct workqueue_struct *stk_als_wq;

	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
	bool use_fir;
	struct data_filter      fir;
	atomic_t                firlength;
};

static int32_t stk3x1x_enable_ps(struct stk3x1x_data *ps_data, uint8_t enable);
static int32_t stk3x1x_enable_als(struct stk3x1x_data *ps_data, uint8_t enable);
static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l);
static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h);
static int stk3x1x_device_ctl(struct stk3x1x_data *ps_data, bool enable);

/* array: als, gain (base 100 ) */
static uint32_t als_level_len_15011 = 2; 
static int als_level_gain_15011[][2] =
{
	{37, 40},
	{1735, 47},
};
static int als_algo_del(int alscode)
{
	int i = 0;
	int len;
	int (*als_level_gain)[2];
/*huqiao@EXP.BasicDrv.Basic add for clone 15085*/
	if (is_project(OPPO_15011) || is_project(OPPO_15085))
	{
		len = als_level_len_15011;
		als_level_gain = als_level_gain_15011;
	}
	else
	{
		return alscode;
	}

	for (i = 0 ; i < len; i++)
	{
		if (alscode < als_level_gain[i][0])
		{
			//printk(KERN_ERR"%s  alscode:%d  als_level_gain:%d \n", __func__, alscode, als_level_gain[i][1]); 
			return alscode*als_level_gain[i][1]/100;
		}
	}

	return alscode*als_level_gain[len-1][1]/100 > 65535? 65535:alscode*als_level_gain[len-1][1]/100;   
}


static inline uint32_t stk_alscode2lux(struct stk3x1x_data *ps_data, uint32_t alscode)
{
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));
	alscode<<=3;
	alscode/=ps_data->als_transmittance;

	return als_algo_del(alscode);
}

static inline uint32_t stk_lux2alscode(struct stk3x1x_data *ps_data, uint32_t lux)
{
	lux*=ps_data->als_transmittance;
	lux/=1100;
	if (unlikely(lux>=(1<<16)))
		lux = (1<<16) -1;
	return lux;
}

static int32_t stk3x1x_init_all_reg(struct stk3x1x_data *ps_data, struct stk3x1x_platform_data *plat_data)
{
	int32_t ret;
	uint8_t w_reg;

	w_reg = plat_data->state_reg;
	ret = i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	ps_data->ps_thd_h = plat_data->ps_thd_h;
	ps_data->ps_thd_l = plat_data->ps_thd_l;

	w_reg = plat_data->psctrl_reg;
	ret = i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	w_reg = plat_data->alsctrl_reg;
	ret = i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	w_reg = plat_data->ledctrl_reg;
	ret = i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	ps_data->wait_reg = plat_data->wait_reg;

	if(ps_data->wait_reg < 2)
	{
		printk(KERN_WARNING "%s: wait_reg should be larger than 2, force to write 2\n", __func__);
		ps_data->wait_reg = 2;
	}
	else if (ps_data->wait_reg > 0xFF)
	{
		printk(KERN_WARNING "%s: wait_reg should be less than 0xFF, force to write 0xFF\n", __func__);
		ps_data->wait_reg = 0xFF;
	}
	w_reg = plat_data->wait_reg;
	ret = i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

	w_reg = 0;
	w_reg |= STK_INT_PS_MODE;

	ret = i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(ps_data->client, 0x87, 0x60);
	if (ret < 0) {
		dev_err(&ps_data->client->dev,
				"%s: write i2c error\n", __func__);
		return ret;
	}
	return 0;
}

static int32_t stk3x1x_check_pid(struct stk3x1x_data *ps_data)
{
	int32_t err1, err2;

	err1 = i2c_smbus_read_byte_data(ps_data->client,STK_PDT_ID_REG);
	if (err1 < 0)
	{
		printk(KERN_ERR "%s: read i2c error, err=%d\n", __func__, err1);
		return err1;
	}
	printk(KERN_ERR "%s: read ID:0x%x \n", __func__, err1);
	if (err1 == 0x1E)  // stk3311-sa
	{
		//printk(KERN_ERR "%s: the product:stk3311-sa. \n", __func__);
/*huqiao@EXP.BasicDrv.Basic add for clone 15085*/
		if (is_project(OPPO_15011) || is_project(OPPO_15085))
		{
			ps_data->pdata->ledctrl_reg = 0x3F;
			ps_data->pdata->wait_reg = 0x10;
		}
	}

	err2 = i2c_smbus_read_byte_data(ps_data->client,STK_RSRVD_REG);
	if (err2 < 0)
	{
		printk(KERN_ERR "%s: read i2c error, err=%d\n", __func__, err2);
		return -1;
	}
	if(err2 == 0xC0)
		printk(KERN_INFO "%s: RID=0xC0!!!!!!!!!!!!!\n", __func__);

	return 0;
}


static int32_t stk3x1x_software_reset(struct stk3x1x_data *ps_data)
{
	int32_t r;
	uint8_t w_reg;

	w_reg = 0x7F;
	r = i2c_smbus_write_byte_data(ps_data->client,STK_WAIT_REG,w_reg);
	if (r<0)
	{
		printk(KERN_ERR "%s: software reset: write i2c error, ret=%d\n", __func__, r);
		return r;
	}
	r = i2c_smbus_read_byte_data(ps_data->client,STK_WAIT_REG);
	if (w_reg != r)
	{
		printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
		return -1;
	}

	r = i2c_smbus_write_byte_data(ps_data->client,STK_SW_RESET_REG,0);
	if (r<0)
	{
		printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
		return r;
	}
	msleep(1);
	return 0;
}

static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l)
{
	uint8_t temp;
	uint8_t* pSrc = (uint8_t*)&thd_l;
	g_low_thd = thd_l;
	temp = *pSrc;
	*pSrc = *(pSrc+1);
	*(pSrc+1) = temp;
	ps_data->ps_thd_l = thd_l;
	return i2c_smbus_write_word_data(ps_data->client,STK_THDL1_PS_REG,thd_l);
}

static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h)
{
	uint8_t temp;
	uint8_t* pSrc = (uint8_t*)&thd_h;
	g_high_thd = thd_h;
	temp = *pSrc;
	*pSrc = *(pSrc+1);
	*(pSrc+1) = temp;
	ps_data->ps_thd_h = thd_h;
	return i2c_smbus_write_word_data(ps_data->client,STK_THDH1_PS_REG,thd_h);
}

// hight light process . when the light contains lots of IR, the ps value will reduce. 
static int stk_alsprx_prx_val(void)
{
	uint8_t mode;
	int32_t word_data , lii;
	int32_t tmp_word_data_1 = 0;       
	int32_t tmp_word_data_2 = 0;

	tmp_word_data_1 = i2c_smbus_read_word_data(g_ps_data->client,0x20);
	tmp_word_data_2 = i2c_smbus_read_word_data(g_ps_data->client,0x22);
	if(tmp_word_data_1 < 0  || tmp_word_data_2 < 0)
	{
		printk(KERN_ERR "%s fail ", __func__);
		return 0;
	}
	word_data = ((tmp_word_data_1 & 0xFF00) >> 8) | ((tmp_word_data_1 & 0x00FF) << 8) ;   
	word_data += ((tmp_word_data_2 & 0xFF00) >> 8) | ((tmp_word_data_2 & 0x00FF) << 8) ;   

	mode = (g_ps_data->pdata->psctrl_reg) & 0x3F;
	if (mode == 0x30)
		lii = 100;
	else if (mode == 0x31)
		lii = 200;
	else if (mode == 0x32)
		lii = 400;
	else if (mode == 0x33)
		lii = 800;
	else{
		printk( "unsupported PS_IT(0x%x)\n", mode);
		return 0xFF;
	}
	//printk("%s :  word_data=%u, lii=%u\n", __func__,word_data, lii);
	if (word_data > lii*3/4)   //the light contains lots of IR
	{
		return 0xFF;
	}

	return 0;
}


static inline uint32_t stk3x1x_get_ps_reading(struct stk3x1x_data *ps_data)
{
	int32_t word_data, tmp_word_data = 0;

	tmp_word_data = i2c_smbus_read_word_data(ps_data->client,STK_DATA1_PS_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return word_data;
}

static int32_t stk3x1x_set_flag(struct stk3x1x_data *ps_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	w_flag &= (~clr);
	//printk(KERN_INFO "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);
	return i2c_smbus_write_byte_data(ps_data->client,STK_FLAG_REG, w_flag);
}

static int32_t stk3x1x_get_flag(struct stk3x1x_data *ps_data)
{
	return i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);
}

static int32_t stk3x1x_enable_ps(struct stk3x1x_data *ps_data, uint8_t enable)
{
	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;

	uint32_t reading;
	int32_t near_far_state;

	STK3X1X_LOG("%s: enable = %d\n",__func__, enable);
	curr_ps_enable = ps_data->ps_enabled?1:0;
	if(curr_ps_enable == enable)
		return 0;

	if (enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

	ret = i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error, ret=%d\n", __func__, ret);
		return ret;
	}
	w_state_reg = ret;
	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | 0x60);
	if(enable)
	{
		w_state_reg |= STK_STATE_EN_PS_MASK;
		if(!(ps_data->als_enabled))
			w_state_reg |= STK_STATE_EN_WAIT_MASK;
	}
	ret = i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error, ret=%d\n", __func__, ret);
		return ret;
	}

	if(enable)
	{
#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD
		{
			int low_threshold, high_threshold;
			if (ps_min > 1000)
			{
				dirty_adjust_high_thd = 600; 
				dirty_adjust_low_thd = 500;  
			}
			else
			{
				dirty_adjust_high_thd = 250; 
				dirty_adjust_low_thd = 200;                  
			}
			if (ps_min != 0 && ps_min + dirty_adjust_high_thd < ps_adjust_max)
			{
				high_threshold = ps_min + dirty_adjust_high_thd;
				low_threshold = ps_min + dirty_adjust_low_thd;                    
			}
			else
			{
				high_threshold =  ps_thd_high_highlight;  
				low_threshold = ps_thd_low_highlight;   
			}

			stk3x1x_set_ps_thd_h(g_ps_data, high_threshold);
			stk3x1x_set_ps_thd_l(g_ps_data, low_threshold);
		}          
#endif

		ps_data->ps_enabled = true;

		enable_irq(ps_data->irq);
		msleep(1);
		ret = stk3x1x_get_flag(ps_data);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: read i2c error, ret=%d\n", __func__, ret);
			return ret;
		}

		near_far_state = ret & STK_FLG_NF_MASK;
		ps_data->ps_distance_last = near_far_state;
		input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
		input_sync(ps_data->ps_input_dev);
		wake_lock_timeout(&ps_data->ps_wakelock, 2*HZ);
		reading = stk3x1x_get_ps_reading(ps_data);
		STK3X1X_LOG("%s: ps_is_far=%d, cur ps_raw = %d ,  ps_min = %d \n",__func__, near_far_state, reading, ps_min);

#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD
		wake_up(&enable_wq);	
#endif
	}
	else
	{
		disable_irq(ps_data->irq);
		ps_data->ps_enabled = false;

	}
	if (!enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

	return ret;
}

static int32_t stk3x1x_enable_als(struct stk3x1x_data *ps_data, uint8_t enable)
{
	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (ps_data->als_enabled)?1:0;
	STK3X1X_LOG("%s: enable = %d\n",__func__, enable);
	if(curr_als_enable == enable)
		return 0;

	if (enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

	ret = i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	w_state_reg = (uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)));
	if(enable)
		w_state_reg |= STK_STATE_EN_ALS_MASK;
	else if (ps_data->ps_enabled)
		w_state_reg |= STK_STATE_EN_WAIT_MASK;

	ret = i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	if (enable)
	{
		ps_data->als_enabled = true;
		hrtimer_start(&ps_data->als_timer, ns_to_ktime(200000000), HRTIMER_MODE_REL);   //200ms   alsctrl=3a,so time > 0.185*(2^10) ms
	}
	else
	{
		ps_data->als_enabled = false;
		hrtimer_cancel(&ps_data->als_timer);
	}

	if (!enable) 
	{
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

	return ret;
}

static inline int32_t stk3x1x_filter_reading(struct stk3x1x_data *ps_data,int32_t word_data)
{
	int index;
	int firlen = atomic_read(&ps_data->firlength);

	if (ps_data->fir.number < firlen) {
		ps_data->fir.raw[ps_data->fir.number] = word_data;
		ps_data->fir.sum += word_data;
		ps_data->fir.number++;
		ps_data->fir.idx++;
	} else {
		index = ps_data->fir.idx % firlen;
		ps_data->fir.sum -= ps_data->fir.raw[index];
		ps_data->fir.raw[index] = word_data;
		ps_data->fir.sum += word_data;
		ps_data->fir.idx++;
		word_data = ps_data->fir.sum/firlen;
	}
	return word_data;
}

static inline int32_t stk3x1x_get_als_reading(struct stk3x1x_data *ps_data)
{
	int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_byte_data(ps_data->client, STK_FLAG_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	if ((tmp_word_data & 0x80) == 0)   // STK_FLAG_REG[bit7]: FLG_ALSDR
	{
		printk(KERN_ERR "%s fail, als data not ready! \n", __func__);
		return -1;
	}

	tmp_word_data = i2c_smbus_read_word_data(ps_data->client, STK_DATA1_ALS_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	if (ps_data->use_fir)
		word_data = stk3x1x_filter_reading(ps_data, word_data);

	if (word_data < 3)
	{
		word_data = 0;
	}
	else
	{
		word_data -= 2;
	}

	return word_data;
}

static int stk_als_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enabled)
{
	struct stk3x1x_data *als_data = container_of(sensors_cdev,struct stk3x1x_data, als_cdev);
	int err;

	mutex_lock(&als_data->io_lock);
	err = stk3x1x_enable_als(als_data, enabled);
	mutex_unlock(&als_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t stk_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	int32_t enable, ret;

	mutex_lock(&ps_data->io_lock);
	enable = (ps_data->als_enabled)?1:0;
	mutex_unlock(&ps_data->io_lock);
	ret = i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
	ret = (ret & STK_STATE_EN_ALS_MASK)?1:0;

	if(enable != ret)
		printk(KERN_ERR "%s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable ALS : %d\n", __func__, en);
	mutex_lock(&ps_data->io_lock);
	stk3x1x_enable_als(ps_data, en);
	mutex_unlock(&ps_data->io_lock);
	return size;
}

static ssize_t stk_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);
	int32_t als_reading;
	uint32_t als_lux;
	als_reading = stk3x1x_get_als_reading(ps_data);
	mutex_lock(&ps_data->io_lock);
	als_lux = stk_alscode2lux(ps_data, als_reading);
	mutex_unlock(&ps_data->io_lock);
	return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t stk_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",__func__, ret);
		return ret;
	}
	mutex_lock(&ps_data->io_lock);
	ps_data->als_lux_last = value;
	input_report_abs(ps_data->als_input_dev, ABS_MISC, value);
	input_sync(ps_data->als_input_dev);
	mutex_unlock(&ps_data->io_lock);
	dev_dbg(dev, "%s: als input event %ld lux\n", __func__, value);

	return size;
}

static ssize_t stk_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n",(u32)ktime_to_ms(ps_data->als_poll_delay));
}

static inline void stk_als_delay_store_fir(struct stk3x1x_data *ps_data)
{
	ps_data->fir.number = 0;
	ps_data->fir.idx = 0;
	ps_data->fir.sum = 0;
}

static int stk_als_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	struct stk3x1x_data *als_data = container_of(sensors_cdev,
			struct stk3x1x_data, als_cdev);
	uint64_t value = 0;

	value = delay_msec * 1000000;

	if (value < MIN_ALS_POLL_DELAY_NS)
		value = MIN_ALS_POLL_DELAY_NS;

	mutex_lock(&als_data->io_lock);
	if (value != ktime_to_ns(als_data->als_poll_delay))
		als_data->als_poll_delay = ns_to_ktime(value);

	if (als_data->use_fir)
		stk_als_delay_store_fir(als_data);

	mutex_unlock(&als_data->io_lock);

	return 0;
}

static ssize_t stk_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	uint64_t value = 0;
	int ret;
	struct stk3x1x_data *als_data =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0)
	{
		dev_err(dev, "%s:kstrtoull failed, ret=0x%x\n",	__func__, ret);
		return ret;
	}

	ret = stk_als_poll_delay_set(&als_data->als_cdev, value);
	if (ret < 0)
		return ret;
	return size;
}


static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint32_t reading;
	reading = stk3x1x_get_ps_reading(ps_data);
	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static int stk_ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enabled)
{
	struct stk3x1x_data *ps_data = container_of(sensors_cdev,
			struct stk3x1x_data, ps_cdev);
	int err;

	mutex_lock(&ps_data->io_lock);
	err = stk3x1x_enable_ps(ps_data, enabled);
	mutex_unlock(&ps_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t enable, ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);

	mutex_lock(&ps_data->io_lock);
	enable = (ps_data->ps_enabled)?1:0;
	mutex_unlock(&ps_data->io_lock);
	ret = i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
	ret = (ret & STK_STATE_EN_PS_MASK)?1:0;

	if(enable != ret)
		printk(KERN_ERR "%s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable PS : %d\n", __func__, en);
	mutex_lock(&ps_data->io_lock);
	stk3x1x_enable_ps(ps_data, en);
	mutex_unlock(&ps_data->io_lock);
	return size;
}

static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_word_data(ps_data->client, STK_DATA1_OFFSET_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	uint16_t offset;

	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",__func__, ret);
		return ret;
	}
	if(value > 65535)
	{
		printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, value);
		return -EINVAL;
	}

	offset = (uint16_t) ((value&0x00FF) << 8) | ((value&0xFF00) >>8);
	ret = i2c_smbus_write_word_data(ps_data->client,STK_DATA1_OFFSET_REG,offset);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	int32_t dist=1, ret;

	mutex_lock(&ps_data->io_lock);
	ret = stk3x1x_get_flag(ps_data);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: stk3x1x_get_flag failed, ret=0x%x\n", __func__, ret);
		return ret;
	}
	dist = (ret & STK_FLG_NF_MASK)?1:0;

	ps_data->ps_distance_last = dist;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, dist);
	input_sync(ps_data->ps_input_dev);
	mutex_unlock(&ps_data->io_lock);
	wake_lock_timeout(&ps_data->ps_wakelock, 2*HZ);
	dev_dbg(dev, "%s: ps input event %d cm\n", __func__, dist);
	return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",__func__, ret);
		return ret;
	}
	mutex_lock(&ps_data->io_lock);
	ps_data->ps_distance_last = value;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, value);
	input_sync(ps_data->ps_input_dev);
	mutex_unlock(&ps_data->io_lock);
	wake_lock_timeout(&ps_data->ps_wakelock, 2*HZ);
	dev_dbg(dev, "%s: ps input event %ld cm\n", __func__, value);
	return size;
}


static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ps_reg[27];
	uint8_t cnt;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	mutex_lock(&ps_data->io_lock);
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			mutex_unlock(&ps_data->io_lock);
			printk(KERN_ERR "stk_all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		}
		else
		{
			dev_dbg(dev, "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	dev_dbg(dev, "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
	cnt++;
	ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	dev_dbg(dev, "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);
	mutex_unlock(&ps_data->io_lock);

	return scnprintf(buf, PAGE_SIZE, "%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X\n",
			ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8],
			ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17],
			ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
}


static struct device_attribute als_enable_attribute = __ATTR(enable,0664,stk_als_enable_show,stk_als_enable_store);
static struct device_attribute als_lux_attribute = __ATTR(lux,0664,stk_als_lux_show,stk_als_lux_store);
static struct device_attribute als_poll_delay_attribute =__ATTR(poll_delay, 0664, stk_als_delay_show, stk_als_delay_store);

static struct attribute *stk_als_attrs [] =
{
	&als_enable_attribute.attr,
	&als_lux_attribute.attr,
	&als_poll_delay_attribute.attr,
	NULL
};

static struct attribute_group stk_als_attribute_group = {
	.attrs = stk_als_attrs,
};

static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,stk_ps_enable_show,stk_ps_enable_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0664,stk_ps_distance_show, stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset,0664,stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, stk_ps_code_show, NULL);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, stk_all_reg_show, NULL);

static struct attribute *stk_ps_attrs [] =
{
	&ps_enable_attribute.attr,
	&ps_distance_attribute.attr,
	&ps_offset_attribute.attr,
	&ps_code_attribute.attr,
	&all_reg_attribute.attr,
	NULL
};

static struct attribute_group stk_ps_attribute_group = {
	.attrs = stk_ps_attrs,
};

static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_data *ps_data = container_of(timer, struct stk3x1x_data, als_timer);
	queue_work(ps_data->stk_als_wq, &ps_data->stk_als_work);
	hrtimer_forward_now(&ps_data->als_timer, ps_data->als_poll_delay);
	return HRTIMER_RESTART;
}

static void stk_als_work_func(struct work_struct *work)
{
	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_als_work);
	int32_t reading;

	mutex_lock(&ps_data->io_lock);
	reading = stk3x1x_get_als_reading(ps_data);
	if(reading < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		return;       
	}   
	ps_data->als_lux_last = stk_alscode2lux(ps_data, reading);
	input_report_abs(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);
	input_sync(ps_data->als_input_dev);
	mutex_unlock(&ps_data->io_lock);
}

static void stk_work_func(struct work_struct *work)
{
	uint32_t reading;
	int32_t ret;
	uint8_t disable_flag = 0;
	uint8_t org_flag_reg;

	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_work);
	int32_t near_far_state;
	mutex_lock(&ps_data->io_lock);

	/* mode 0x01 or 0x04 */
	org_flag_reg = stk3x1x_get_flag(ps_data);
	if(org_flag_reg < 0)
	{
		printk(KERN_ERR "%s: get_status_reg fail, org_flag_reg=%d", __func__, org_flag_reg);
		goto err_i2c_rw;
	}

	if (org_flag_reg & STK_FLG_PSINT_MASK)
	{
		disable_flag |= STK_FLG_PSINT_MASK;
		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;

		ps_data->ps_distance_last = near_far_state;
		input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
		input_sync(ps_data->ps_input_dev);
		wake_lock_timeout(&ps_data->ps_wakelock, 2*HZ);
		reading = stk3x1x_get_ps_reading(ps_data);
		printk(KERN_INFO "%s: ps input event=%d, ps code = %d ,low_thd:%d high_thd:%d \n",__func__, near_far_state, reading, g_low_thd, g_high_thd);
	}
	ret = stk3x1x_set_flag(ps_data, org_flag_reg, disable_flag);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:reset_int_flag fail, ret=%d\n", __func__, ret);
		goto err_i2c_rw;
	}

	msleep(1);
	enable_irq(ps_data->irq);
	mutex_unlock(&ps_data->io_lock);
	return;

err_i2c_rw:
	mutex_unlock(&ps_data->io_lock);
	msleep(30);
	enable_irq(ps_data->irq);
	return;
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3x1x_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->stk_wq,&pData->stk_work);
	return IRQ_HANDLED;
}

static inline void stk3x1x_init_fir(struct stk3x1x_data *ps_data)
{
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	atomic_set(&ps_data->firlength, STK_FIR_LEN);
}

static int32_t stk3x1x_init_all_setting(struct i2c_client *client, struct stk3x1x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);

	ret = stk3x1x_software_reset(ps_data);
	if(ret < 0)
		return ret;

	stk3x1x_check_pid(ps_data);
	if(ret < 0)
		return ret;

	ret = stk3x1x_init_all_reg(ps_data, plat_data);
	if(ret < 0)
		return ret;

	if (plat_data->use_fir)
		stk3x1x_init_fir(ps_data);

	return 0;
}

static int stk3x1x_setup_irq(struct i2c_client *client)
{
	int irq, err = -EIO;
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);

	irq = gpio_to_irq(ps_data->int_pin);
	if (irq <= 0)
	{
		printk(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;
	err = gpio_request(ps_data->int_pin,"stk-int");
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
		return err;
	}
	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
		return err;
	}

	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ps_data);
	if (err < 0)
	{
		printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);
		goto err_request_any_context_irq;
	}
	disable_irq(irq);

	return 0;

err_request_any_context_irq:
	gpio_free(ps_data->int_pin);
	return err;
}

static int stk3x1x_suspend(struct device *dev)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);
	int err;

	printk("%s enter \n", __func__);
	mutex_lock(&ps_data->io_lock);
	ps_data->als_enable_state = ps_data->als_enabled;
	if(ps_data->als_enable_state)
	{
		stk3x1x_enable_als(ps_data, 0);
	}
	if(ps_data->ps_enabled)
	{
		err = enable_irq_wake(ps_data->irq);
		if (err)
			printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);
	}
	mutex_unlock(&ps_data->io_lock);
	g_is_resumed = 0;
	return 0;
}

static int stk3x1x_resume(struct device *dev)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);

	int err;

	printk("%s enter \n", __func__);
	mutex_lock(&ps_data->io_lock);
	if(ps_data->als_enable_state)
	{   
		stk3x1x_enable_als(ps_data, 1);
	}

	if(ps_data->ps_enabled)
	{
		err = disable_irq_wake(ps_data->irq);
		if (err)
			printk(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);
	}
	mutex_unlock(&ps_data->io_lock);
	g_is_resumed = 1;
	return 0;
}


static int stk3x1x_power_ctl(struct stk3x1x_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {

		printk(KERN_ERR"%s, on = %d\n", __func__, on);

		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,"Regulator vio disable failed ret=%d\n", ret);
			ret = regulator_enable(data->vdd);
			if (ret) {
				dev_err(&data->client->dev,"Regulator vdd enable failed ret=%d\n",ret);
			}
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "stk3x1x_power_ctl on=%d\n",on);
	} else if (on && !data->power_enabled) {

		printk(KERN_ERR"%s, on = %d\n", __func__, on);

		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "stk3x1x_power_ctl on=%d\n",on);

		msleep(20);
	} else {
		dev_warn(&data->client->dev,"Power on=%d. enabled=%d\n",on, data->power_enabled);
	}

	return ret;
}

static int stk3x1x_power_init(struct stk3x1x_data *data, bool on)
{
	int ret;

	printk(KERN_ERR"%s, on = %d\n", __func__, on);

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,0, STK3X1X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,0, STK3X1X_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,STK3X1X_VDD_MIN_UV,STK3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,"Regulator set failed vdd ret=%d\n",ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,STK3X1X_VIO_MIN_UV,STK3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, STK3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int stk3x1x_device_ctl(struct stk3x1x_data *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable && !ps_data->power_enabled) {
		ret = stk3x1x_power_ctl(ps_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		ret = stk3x1x_init_all_setting(ps_data->client, ps_data->pdata);
		if (ret < 0) {
			stk3x1x_power_ctl(ps_data, false);
			dev_err(dev, "Failed to re-init device setting\n");
			goto err_exit;
		}
	} else if (!enable && ps_data->power_enabled) {
		if (!ps_data->als_enabled && !ps_data->ps_enabled) {
			ret = stk3x1x_power_ctl(ps_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
					ps_data->als_enabled, ps_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
				enable, ps_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#ifdef CONFIG_OF
static int stk3x1x_parse_dt(struct device *dev,
		struct stk3x1x_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;
	/* LiuPing@Phone.BSP.Sensor, 2014/09/18, add for set gpio to control ex-ldo for vdd supply in 14005. */
	{
		int vdd_gpio = 0;
		vdd_gpio = of_get_named_gpio(np, "sensor,vdd-gpio", 0);
		if (gpio_is_valid(vdd_gpio)) 
		{
			printk("%s set gpio:%d to high for vdd supply. \n", __func__, vdd_gpio);
			gpio_request(vdd_gpio,"vdd-gpio");
			gpio_direction_output(vdd_gpio, 1);
		}
	}
	pdata->int_pin = of_get_named_gpio_flags(np, "stk,irq-gpio",
			0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}

	rc = of_property_read_u32(np, "stk,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,state-reg", &temp_val);
	if (!rc)
		pdata->state_reg = temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,psctrl-reg", &temp_val);
	if (!rc)
		pdata->psctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read psctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,alsctrl-reg", &temp_val);
	if (!rc)
		pdata->alsctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ledctrl-reg", &temp_val);
	if (!rc)
		pdata->ledctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ledctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,wait-reg", &temp_val);
	if (!rc)
		pdata->wait_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read wait-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	pdata->use_fir = of_property_read_bool(np, "stk,use-fir");

	return 0;
}
#else
static int stk3x1x_parse_dt(struct device *dev,
		struct stk3x1x_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */


#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD

static void sample_work_func(struct work_struct *work)
{
	int i;
	int ret;
	uint16_t ps = 0;
	uint8_t w_state_reg;

	ret = stk3x1x_device_ctl(g_ps_data, 1);
	if (ret)
		return;

	ret = i2c_smbus_read_byte_data(g_ps_data->client, STK_STATE_REG);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error, ret=%d\n", __func__, ret);
		return;
	}

	w_state_reg = ret;
	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | 0x60);

	w_state_reg |= STK_STATE_EN_PS_MASK;
	if(!(g_ps_data->als_enabled))
		w_state_reg |= STK_STATE_EN_WAIT_MASK;

	ret = i2c_smbus_write_byte_data(g_ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		STK3X1X_LOG("%s: write i2c error, ret=%d\n", __func__, ret);
		return;
	}

	msleep(10);
	for (i = 0; i < 10; i++)
	{
		if (stk_alsprx_prx_val() == 0)
		{
			if ((ps = stk3x1x_get_ps_reading(g_ps_data)) <= 0)
			{
				continue;
			}
		}
		if( (ps > 0) &&((ps_min == 0) || (ps_min > ps)))
			ps_min = ps;

		msleep(10);
	}


	if (ps_min > ps_adjust_max)  //ps_adjust_max
		ps_min = ps_adjust_max;


	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | 0x60);
	ret = i2c_smbus_write_byte_data(g_ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		STK3X1X_LOG("%s: write i2c error, ret=%d\n", __func__, ret);
		return;
	}

	ret = stk3x1x_device_ctl(g_ps_data, 0);
	if (ret)
		return;

	STK3X1X_LOG("%s ps:%d  \n", __func__, ps_min);
}

#endif /*STK3X1X_ALSPS_DYNAMIC_THRESHOLD*/

static ssize_t stk3x1x_alsps_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data[2];
	int ret = -EINVAL;
	sscanf(buf, "%x %x", (unsigned int *)&data[0],(unsigned int *)&data[1]);
	switch(data[0])
	{
		case SENSOR_TYPE_LIGHT:
			ret = stk3x1x_enable_als(g_ps_data,(int)data[1]);
			if (ret < 0)
			{
				printk("%s: %s als fail\n",__func__, (data[1] == 1)?"enable":"disable");
			}
			break;

		case SENSOR_TYPE_PROXIMITY:
			ret = stk3x1x_enable_ps(g_ps_data, (int)data[1]);
			if (ret < 0)
			{
				printk("%s: %s prox fail\n",__func__, (data[1] == 1)?"enable":"disable");
			}
			break;

		default:
			ret = -EINVAL;
			printk("%s: DO NOT support this type sensor\n",__func__);
			break;
	}
	if (ret == 0)
		printk("%s: Enable sensor SUCCESS\n",__func__);

	return count;
}
static ssize_t stk3x1x_alsps_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "als:%d  prox:%d\n", g_ps_data->als_enabled, g_ps_data->ps_enabled);
}
static struct kobj_attribute enable = 
{
	.attr = {"enable", 0664},
	.show = stk3x1x_alsps_enable_show,
	.store = stk3x1x_alsps_enable_store,
};
static ssize_t stk3x1x_prox_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	uint32_t reading;
	reading = stk3x1x_get_ps_reading(g_ps_data);
	return snprintf(buf, PAGE_SIZE, "%d\n", reading);
}
static struct kobj_attribute prox_raw = 
{
	.attr = {"prox_raw", 0444},
	.show = stk3x1x_prox_raw_show,
};
static ssize_t stk3x1x_als_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int32_t reading;
	reading = stk3x1x_get_als_reading(g_ps_data);

	return snprintf(buf, PAGE_SIZE, "%d\n", reading);
}
static struct kobj_attribute als_raw = 
{
	.attr = {"als_raw", 0444},
	.show = stk3x1x_als_raw_show,
};


#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD    // Do not modify it as wilful, because it is used for algo.
static ssize_t stk3x1x_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s", DEVICE_NAME);
}
static struct kobj_attribute name = 
{
	.attr = {"name", 0444},
	.show = stk3x1x_name_show,
};
static ssize_t stk3x1x_high_light_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",  (stk_alsprx_prx_val() == 0)? 0:1);
}
static struct kobj_attribute is_high_light = 
{
	.attr = {"is_high_light", 0444},
	.show = stk3x1x_high_light_show,
};
static ssize_t stk3x1x_ps_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", g_ps_data->ps_enabled);
}
static struct kobj_attribute ps_enable = 
{
	.attr = {"ps_enable", 0444},
	.show = stk3x1x_ps_enable_show,
};
static ssize_t stk3x1x_alsps_ps_thd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data[2];
	if (sscanf(buf, "%d %d", (unsigned int *)&data[0],(unsigned int *)&data[1]) == 2)
	{
		//printk("algo set --- low_thd:%5d, high_thd:%5d \n", data[0], data[1]);

		g_low_thd = data[0];
		g_high_thd = data[1];
		stk3x1x_set_ps_thd_h(g_ps_data, data[1]);
		stk3x1x_set_ps_thd_l(g_ps_data, data[0]);
	}
	else
	{
		printk("%s the buf format is error.\n", __func__);
	}
	return count;
}
static struct kobj_attribute ps_thd = 
{
	.attr = {"ps_thd", 0220},
	.store = stk3x1x_alsps_ps_thd_store,
};

static ssize_t stk3x1x_alsps_ps_min_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;
	if (sscanf(buf, "%d", (unsigned int *)&data) == 1)
	{
		printk(KERN_ERR"%s the buf is %s\n", __func__, buf);
		ps_min = data;
	}
	else
	{
		printk("%s the buf format is error.\n", __func__);
	}
	return count;
}
static ssize_t stk3x1x_alsps_ps_min_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ps_min);
}
static struct kobj_attribute ps_min_val = 
{
	.attr = {"ps_min", 0664},
	.show = stk3x1x_alsps_ps_min_show,
	.store = stk3x1x_alsps_ps_min_store,
};

static ssize_t stk3x1x_alsps_algo_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data[5];
	if (sscanf(buf, "%d %d %d %d %d", (unsigned int *)&data[0],(unsigned int *)&data[1], (unsigned int *)&data[2],
				(unsigned int *)&data[3], (unsigned int *)&data[4]) == 5)
	{
		printk(KERN_ERR"%s the buf is %s\n", __func__, buf);
		ps_thd_low_highlight = data[0];
		ps_thd_high_highlight = data[1];
		ps_adjust_max = data[2];
		dirty_adjust_low_thd = data[3];
		dirty_adjust_high_thd = data[4];

	}
	else
	{
		printk("%s the buf format is error.\n", __func__);
	}
	return count;
}
static struct kobj_attribute algo_info = 
{
	.attr = {"algo_info", 0220},
	.store = stk3x1x_alsps_algo_info_store,
};
static ssize_t stk3x1x_algo_wakeup_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	wait_event_interruptible(enable_wq, g_ps_data->ps_enabled);
	printk(KERN_ERR"wait ps enable done\n");
	return snprintf(buf, PAGE_SIZE, "%d\n", g_ps_data->ps_enabled);
}
static struct kobj_attribute algo_wakeup = 
{
	.attr = {"algo_wakeup", 0444},
	.show = stk3x1x_algo_wakeup_show,
};
static ssize_t stk3x1x_far_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",  g_ps_data->ps_distance_last);
}
static struct kobj_attribute far_status = 
{
	.attr = {"far_status", 0444},
	.show = stk3x1x_far_status_show,
};
#endif

static const struct attribute *stk3x1x_ftm_attrs[] = 
{
	&enable.attr,
	&prox_raw.attr,
	&als_raw.attr,
#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD	
	&name.attr,
	&is_high_light.attr,
	&ps_enable.attr,
	&ps_thd.attr,
	&far_status.attr,	
	&ps_min_val.attr,
	&algo_wakeup.attr,
	&algo_info.attr,
#endif
	NULL
};

static struct dev_ftm stk3x1x_ftm;
static int stk3x1x_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int err = -ENODEV;
	struct stk3x1x_data *ps_data;
	struct stk3x1x_platform_data *plat_data;
	printk(KERN_INFO "%s : driver version = %s\n", __func__, DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_BYTE_DATA\n", __func__);
		return -ENODEV;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
	{
		printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_WORD_DATA\n", __func__);
		return -ENODEV;
	}

	ps_data = kzalloc(sizeof(struct stk3x1x_data),GFP_KERNEL);
	if(!ps_data)
	{
		printk(KERN_ERR "%s: failed to allocate stk3x1x_data\n", __func__);
		return -ENOMEM;
	}
	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	mutex_init(&ps_data->io_lock);
	wake_lock_init(&ps_data->ps_wakelock,WAKE_LOCK_SUSPEND, "stk_input_wakelock");

	if (client->dev.of_node) {
		plat_data = devm_kzalloc(&client->dev,sizeof(struct stk3x1x_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = stk3x1x_parse_dt(&client->dev, plat_data);
		dev_err(&client->dev,"%s: stk3x1x_parse_dt ret=%d\n", __func__, err);
		if (err)
			return err;
	} else
		plat_data = client->dev.platform_data;

	if (!plat_data) {
		dev_err(&client->dev,"%s: no stk3x1x platform data!\n", __func__);
		goto err_als_input_allocate;
	}
	ps_data->als_transmittance = plat_data->transmittance;
	ps_data->int_pin = plat_data->int_pin;
	ps_data->use_fir = plat_data->use_fir;
	ps_data->pdata = plat_data;

	if (ps_data->als_transmittance == 0) {
		dev_err(&client->dev,"%s: Please set als_transmittance\n", __func__);
		goto err_als_input_allocate;
	}

	err = stk3x1x_power_init(ps_data, true);
	if (err)
		goto err_power_init;

	err = stk3x1x_power_ctl(ps_data, true);
	if (err)
		goto err_power_on;

	err = stk3x1x_init_all_setting(client, ps_data->pdata);
	if (err)
		goto err_init_fail;

	ps_data->als_input_dev = input_allocate_device();
	if (ps_data->als_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		goto err_init_fail;
	}
	ps_data->ps_input_dev = input_allocate_device();
	if (ps_data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);
		err = -ENOMEM;
		goto err_ps_input_allocate;
	}
	ps_data->als_input_dev->name = ALS_NAME;
	ps_data->ps_input_dev->name = PS_NAME;
	set_bit(EV_ABS, ps_data->als_input_dev->evbit);
	set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
	input_set_abs_params(ps_data->als_input_dev, ABS_MISC, 0, stk_alscode2lux(ps_data, (1<<16)-1), 0, 0);
	input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(ps_data->als_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register als input device\n", __func__);
		goto err_als_input_register;
	}
	err = input_register_device(ps_data->ps_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);
		goto err_ps_input_register;
	}

	err = sysfs_create_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		goto err_als_sysfs_create_group;
	}
	err = sysfs_create_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto err_ps_sysfs_create_group;
	}
	input_set_drvdata(ps_data->als_input_dev, ps_data);
	input_set_drvdata(ps_data->ps_input_dev, ps_data);

	ps_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
	INIT_WORK(&ps_data->stk_als_work, stk_als_work_func);
	hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	ps_data->als_timer.function = stk_als_timer_func;

	ps_data->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&ps_data->stk_work, stk_work_func);
	err = stk3x1x_setup_irq(client);
	if(err < 0)
		goto err_stk3x1x_setup_irq;

	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;

	/* make sure everything is ok before registering the class device */
	ps_data->als_cdev = sensors_light_cdev;
	ps_data->als_cdev.sensors_enable = stk_als_enable_set;
	ps_data->als_cdev.sensors_poll_delay = stk_als_poll_delay_set;
	err = sensors_classdev_register(&ps_data->als_input_dev->dev,&ps_data->als_cdev);
	if (err)
		goto err_class_sysfs_light;

	ps_data->ps_cdev = sensors_proximity_cdev;
	ps_data->ps_cdev.sensors_enable = stk_ps_enable_set;
	err = sensors_classdev_register(&ps_data->ps_input_dev->dev,&ps_data->ps_cdev);
	if (err)
		goto err_class_sysfs_prox;

	/* enable device power only when it is enabled */
	err = stk3x1x_power_ctl(ps_data, false);
	if (err)
		goto err_init_all_setting;

	g_ps_data = ps_data;

	stk3x1x_ftm.name = "als_prox";
	stk3x1x_ftm.i2c_client = ps_data->client;
	stk3x1x_ftm.attrs = stk3x1x_ftm_attrs;
	stk3x1x_ftm.priv_data = ps_data;
	register_single_dev_ftm(&stk3x1x_ftm);       

#ifdef STK3X1X_ALSPS_DYNAMIC_THRESHOLD  

	INIT_DELAYED_WORK(&sample_ps_work, sample_work_func);
	queue_delayed_work(ps_data->stk_wq,&sample_ps_work, msecs_to_jiffies(5000)); 

	init_waitqueue_head(&enable_wq);
#endif

	printk(KERN_ERR"%s: probe successfully", __func__);

	return 0;

err_init_all_setting:
	sensors_classdev_unregister(&ps_data->ps_cdev);
err_class_sysfs_prox:
	sensors_classdev_unregister(&ps_data->als_cdev);
err_class_sysfs_light:
	free_irq(ps_data->irq, ps_data);
	gpio_free(plat_data->int_pin);
err_stk3x1x_setup_irq:
	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->stk_als_wq);
	destroy_workqueue(ps_data->stk_wq);
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
err_ps_sysfs_create_group:
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
err_als_sysfs_create_group:
	input_unregister_device(ps_data->ps_input_dev);
err_ps_input_register:
	input_unregister_device(ps_data->als_input_dev);
err_als_input_register:
	input_free_device(ps_data->ps_input_dev);
err_ps_input_allocate:
	input_free_device(ps_data->als_input_dev);
err_als_input_allocate:
err_init_fail:
	stk3x1x_power_ctl(ps_data, false);
err_power_on:
	stk3x1x_power_init(ps_data, false);
err_power_init:
	wake_lock_destroy(&ps_data->ps_wakelock);
	mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
	return err;
}


static int stk3x1x_remove(struct i2c_client *client)
{
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);

	free_irq(ps_data->irq, ps_data);
	gpio_free(ps_data->int_pin);

	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->stk_als_wq);

	destroy_workqueue(ps_data->stk_wq);

	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
	input_unregister_device(ps_data->ps_input_dev);
	input_unregister_device(ps_data->als_input_dev);
	input_free_device(ps_data->ps_input_dev);
	input_free_device(ps_data->als_input_dev);

	wake_lock_destroy(&ps_data->ps_wakelock);
	mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);

	return 0;
}

static const struct i2c_device_id stk_ps_id[] =
{
	{ "stk_ps", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] = {
	{ .compatible = "stk,stk3x1x", },
	{ },
};

static const struct dev_pm_ops stk3x1x_pm_ops = {
	.suspend	= stk3x1x_suspend,
	.resume 	= stk3x1x_resume,
};

static struct i2c_driver stk_ps_driver =
{
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = stk_match_table,
		.pm = &stk3x1x_pm_ops,
	},
	.probe = stk3x1x_probe,
	.remove = stk3x1x_remove,
	.id_table = stk_ps_id,
};


static int __init stk3x1x_init(void)
{
	int ret;
	ret = i2c_add_driver(&stk_ps_driver);
	if (ret)
		return ret;

	return 0;
}

static void __exit stk3x1x_exit(void)
{
	i2c_del_driver(&stk_ps_driver);
}

module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sitronix.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
