/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : cm36686.c
 ** Description : 
 ** Date        : 2015-03-03 19:14
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

/* drivers/input/misc/cm36686.c - cm36686 optical sensors driver
 *
 * Copyright (C) 2014 Capella Microsystems Inc.
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "cm36686.h"
#include <linux/of_gpio.h>
#include <soc/oppo/oppo_project.h>

#include <asm/uaccess.h>
#include <asm/setup.h>

#include <linux/sensors_ftm.h>

#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02

/* POWER SUPPLY VOLTAGE RANGE */
#define CM36686_VDD_MIN_UV	2700000
#define CM36686_VDD_MAX_UV	3300000
#define CM36686_VI2C_MIN_UV	1750000
#define CM36686_VI2C_MAX_UV	1950000

/* cm36686 polling rate in ms */
#define CM36686_LS_MIN_POLL_DELAY	1
#define CM36686_LS_MAX_POLL_DELAY	1000
#define CM36686_LS_DEFAULT_POLL_DELAY	100

#define CM36686_PS_MIN_POLL_DELAY	1
#define CM36686_PS_MAX_POLL_DELAY	1000
#define CM36686_PS_DEFAULT_POLL_DELAY	100

static struct sensors_classdev sensors_light_cdev = {
	.name = "cm36686-light",
	.vendor = "Capella",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "5240",
	.resolution = "0.01",
	.sensor_power = "0.26",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = CM36686_LS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "cm36686-proximity",
	.vendor = "Capella",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.2",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = CM36686_PS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct cm36686_info *g_ps_data = NULL;

#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/08/04, add for dynamic threshold */
#define CM36686_ALSPS_DYNAMIC_THRESHOLD
#endif /*VENDOR_EDIT*/

#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD
static int ps_min = 0;
static int ps_adjust_max = 2500;
static int dirty_adjust_low_thd = 500, dirty_adjust_high_thd = 550;
static int ps_thd_low_highlight = 2400, ps_thd_high_highlight = 2500;

static struct delayed_work sample_ps_work;
static DECLARE_WAIT_QUEUE_HEAD(enable_wq);

#endif

static const int als_range[] = {
	[CM36686_ALS_IT0] = 5240,
	[CM36686_ALS_IT1] = 2620,
	[CM36686_ALS_IT2] = 1310,
	[CM36686_ALS_IT3] = 655,
};

static const int als_sense[] = {
	[CM36686_ALS_IT0] = 10,
	[CM36686_ALS_IT1] = 20,
	[CM36686_ALS_IT2] = 40,
	[CM36686_ALS_IT3] = 80,
};

static void cm36686_sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, cm36686_sensor_irq_do_work);
static int cm36686_psensor_enable(struct cm36686_info *lpi);
static int cm36686_psensor_disable(struct cm36686_info *lpi);
static void cm36686_esd_handle(struct cm36686_info *lpi, int esd_type);

struct cm36686_info {
	struct class *cm36686_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;
	int ps_enable;
	int als_state_suspend_resume;
	int ps_irq_flag;

	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;

	int ls_calibrate;

	int i2c_err_time;
	int esd_handle_flag;

	int (*power)(int, uint8_t); /* power to the chip */

	int power_enabled;
	uint32_t als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;

	int32_t ps_distance_last;
	struct wake_lock ps_wake_lock;
	uint8_t slave_addr;

	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	int current_level;
	uint16_t current_adc;

	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;

	uint16_t ls_cmd;
	uint8_t record_clear_int_fail;
	bool polling;
	atomic_t ls_poll_delay;
	atomic_t ps_poll_delay;
	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work ldwork;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
};
static struct cm36686_info *lp_info;
static int fLevel=-1;
static struct mutex als_get_adc_mutex;
static struct mutex ps_enable_mutex, ps_disable_mutex, ps_get_adc_mutex;
static struct mutex CM36686_control_mutex;
static struct mutex wq_lock;
static int cm36686_lightsensor_enable(struct cm36686_info *lpi);
static int cm36686_lightsensor_disable(struct cm36686_info *lpi);
static int initial_cm36686(struct cm36686_info *lpi);
static void cm36686_psensor_initial_cmd(struct cm36686_info *lpi);
static int cm36686_power_set(struct cm36686_info *info, bool on);

static int32_t als_kadc;

static int cm36686_als_gain = 100;   // the true: lux = (cm36686_als_gain/100)*lux.

static void cm36686_control_and_report(struct cm36686_info *lpi, uint8_t mode, uint16_t param, int report);

#define SHOW_LOG    1

static int printk_log_level = 0;

#define printk_x(level, ...) \
	do { \
		if (printk_log_level >= (level)) \
		printk(__VA_ARGS__); \
	} while (0) 

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	struct cm36686_info *lpi = lp_info;
	uint8_t subaddr[1];

	struct i2c_msg msgs[] = {
		{
			.addr = slaveAddr,
			.flags = 0,
			.len = 1,
			.buf = subaddr,
		},
		{
			.addr = slaveAddr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},		 
	};

	subaddr[0] = cmd;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		dev_err(&lpi->i2c_client->dev, "%s: I2C error(%d). Retrying.\n",__func__, cmd);
		msleep(10);

		if((lpi->esd_handle_flag == 0)&&((lpi->ps_enable == 1)||(lpi->als_enable == 1)))
			cm36686_esd_handle(lpi, 0); // 0 mean: i2c err
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		dev_err(&lpi->i2c_client->dev, "%s: Retry count exceeds %d.",__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	struct cm36686_info *lpi = lp_info;

	struct i2c_msg msg[] = {
		{
			.addr = slaveAddr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		pr_err("%s: I2C error. Retrying...\n", __func__);
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		dev_err(&lpi->i2c_client->dev, "%s: Retry count exceeds %d.",__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm36686_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err("%s: I2C RxData fail(%d).\n", __func__, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];

	return ret;
}

static int _cm36686_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;

	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	

	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("%s: I2C_TxData failed.\n", __func__);
		return -EIO;
	}

	return ret;
}

static int cm36686_get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm36686_info *lpi = lp_info;
	uint32_t tmp;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_DATA, als_step);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: I2C read word failed.\n",__func__);
		return -EIO;
	}

	if (!lpi->ls_calibrate) {
		tmp = (uint32_t)(*als_step) * lpi->als_gadc / lpi->als_kadc;
		if (tmp > 0xFFFF)
			*als_step = 0xFFFF;
		else
			*als_step = tmp;
	}

	//printk_x(SHOW_LOG,"%s adc_value = %d\n", __func__, *als_step);

	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	if (data == NULL)
		return -EFAULT;	

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, PS_DATA, data);

	if (ret < 0)
		return ret;

	return ret;
}

static void cm36686_esd_handle(struct cm36686_info *lpi, int esd_type)
{
	int als_state =0, ps_state = 0;
	int intflag_err = 0;

	als_state = lpi->als_enable;
	ps_state = lpi->ps_enable;

	if(esd_type == 0)    // i2c err
	{
		lpi->i2c_err_time ++;
		printk(KERN_ERR"%s , i2c err time = %d\n",__func__, lpi->i2c_err_time);
	}        
	else                // intflag err
	{
		intflag_err = 1;    
	}

	if((lpi->i2c_err_time > 2)||(intflag_err == 1))
	{
		lpi->esd_handle_flag = 1;
		lpi->i2c_err_time = 0;

		printk(KERN_ERR"%s , esd err, reinit device, disable sensor.\n",__func__);

		if(lpi->als_enable > 0)
		{
			cm36686_lightsensor_disable(lpi);
		}
		if(lpi->ps_enable > 0)
		{
			cm36686_psensor_disable(lpi);
		}

		printk(KERN_ERR"%s , esd err, reinit device, enable sensor.\n",__func__);

		if(als_state > 0)
		{
			cm36686_lightsensor_enable(lpi);
		}
		if(ps_state > 0)
		{
			cm36686_psensor_enable(lpi);
		}

		printk(KERN_ERR"%s , esd handle over.\n",__func__);

		lpi->esd_handle_flag = 0;
	} 
}

static void cm36686_sensor_irq_do_work(struct work_struct *work)
{
	struct cm36686_info *lpi = lp_info;
	uint16_t intFlag = 0;
	int ret = 0;

	if(lpi->ps_enable > 0)    
	{
		ret = _cm36686_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);
		if(ret < 0)
		{
			pr_err("%s cm36686_I2C_Read_Word err! \n", __func__);
		}

		pr_err("%s intFlag = 0x%x \n", __func__, intFlag);

		if(intFlag == 0)
			cm36686_esd_handle(lpi, 1); // 1 means : intFlag err

		cm36686_control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag, 1);
	}

	enable_irq(lpi->irq);
}

static int get_als_range(void)
{
	uint16_t ls_conf;
	int ret = 0;
	int index = 0;
	struct cm36686_info *lpi = lp_info;

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_CONF, &ls_conf);
	if (ret) {
		dev_err(&lpi->i2c_client->dev, "read ALS_CONF from i2c error. %d\n",ret);
		return -EIO;
	}

	index = (ls_conf & 0xC0) >> 0x06;
	return  als_range[index];
}

static int get_als_sense(void)
{
	uint16_t ls_conf;
	int ret = 0;
	int index = 0;
	struct cm36686_info *lpi = lp_info;

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_CONF, &ls_conf);
	if (ret) {
		dev_err(&lpi->i2c_client->dev, "read ALS_CONF from i2c error. %d\n",ret);
		return -EIO;
	}

	index = (ls_conf & 0xC0) >> 0x06;

	return  als_sense[index];
}

static void cm36686_lsensor_delay_work_handler(struct work_struct *work)
{
	struct cm36686_info *lpi = lp_info;
	uint16_t adc_value = 0;
	int sense;
	int report_lux = 0;

	if(lpi->als_enable == 0)
		return ;

	mutex_lock(&wq_lock);

	cm36686_get_ls_adc_value(&adc_value, 0);
	sense = get_als_sense();

	mutex_unlock(&wq_lock);

	if (sense > 0) {
		lpi->current_adc = adc_value;
		report_lux = cm36686_als_gain*adc_value/sense/100;

		if (is_project(OPPO_15018))
			report_lux = report_lux * 15 / 10;

		printk_x(SHOW_LOG,"%s report_lux = %d\n", __func__, report_lux);

		input_report_abs(lpi->ls_input_dev, ABS_MISC, report_lux); 
		input_sync(lpi->ls_input_dev);
	}
	schedule_delayed_work(&lpi->ldwork,msecs_to_jiffies(atomic_read(&lpi->ls_poll_delay)));
}

static irqreturn_t cm36686_irq_handler(int irq, void *data)
{
	struct cm36686_info *lpi = data;

	pr_err("%s enter \n", __func__);

	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm36686_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int cm36686_sensor_regulator_configure(struct cm36686_info *info, bool on)
{
	int rc;

	printk(KERN_ERR"%s, on = %d\n", __func__, on);

	if (!on) {
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0,CM36686_VDD_MAX_UV);

		regulator_put(info->vdd);

		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0,CM36686_VI2C_MAX_UV);

		regulator_put(info->vio);
	} else {
		info->vdd = regulator_get(&info->i2c_client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->i2c_client->dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,CM36686_VDD_MIN_UV, CM36686_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}

		info->vio = regulator_get(&info->i2c_client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->i2c_client->dev,"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,CM36686_VI2C_MIN_UV, CM36686_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(info->vio);

reg_vdd_set:
	if (regulator_count_voltages(info->vdd) > 0)
		regulator_set_voltage(info->vdd, 0, CM36686_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(info->vdd);
	return rc;
}


static int cm36686_power_set(struct cm36686_info *info, bool on)
{
	int rc;

	printk(KERN_ERR"%s, on = %d, power_enabled = %d\n", __func__, on, info->power_enabled);

	if (on && !info->power_enabled) 
	{
		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,"Regulator vio enable failed rc=%d\n", rc);
			return rc;
		}

		info->power_enabled = true;
		msleep(20);
	} 
	else if(!on && info->power_enabled)
	{
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}

		info->power_enabled = false;
	}

	return 0;
}

static void cm36686_ls_initial_cmd(struct cm36686_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= CM36686_ALS_INT_MASK;
	lpi->ls_cmd |= CM36686_ALS_SD;      // als power down
	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
}

static void cm36686_psensor_initial_cmd(struct cm36686_info *lpi)
{
	uint16_t idReg1, idReg2;

	/*must disable p-sensor interrupt befrore IST create*/
	lpi->ps_conf1_val |= CM36686_PS_SD; // ps power down
	lpi->ps_conf1_val &= CM36686_PS_INT_MASK;
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_away_thd_set);
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDH,  lpi->ps_close_thd_set);

	dev_dbg(&lpi->i2c_client->dev,"%s:send psensor initial command finished\n", __func__);

	_cm36686_I2C_Read_Word(lpi->slave_addr, PS_CONF1, &idReg1);
	_cm36686_I2C_Read_Word(lpi->slave_addr, PS_CONF3, &idReg2);

	printk(KERN_ERR"[PS]%s: PS_CONF1 = 0x%x, PS_CONF3 = 0x%x!\n",__func__, idReg1, idReg2);

}

static int cm36686_psensor_enable(struct cm36686_info *lpi)
{
	int ret = 0;
	unsigned int delay;

	printk(KERN_ERR"%s\n", __func__);

	if ((lpi->ps_enable == 0) &&(lpi->als_enable == 0)) 
	{
		cm36686_power_set(lpi, true);

		initial_cm36686(lpi);
		cm36686_ls_initial_cmd(lpi);
		cm36686_psensor_initial_cmd(lpi);
	}

	mutex_lock(&ps_enable_mutex);

	if (lpi->ps_enable) {
		dev_err(&lpi->i2c_client->dev, "already enabled\n");
		ret = 0;
	} 
	else 
	{
#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD
		int low_threshold, high_threshold;

		if (is_project(OPPO_15018))
		{
			if (ps_min > 500)
			{
				dirty_adjust_high_thd = 850; 
				dirty_adjust_low_thd = 800;  
			}
			else
			{
				dirty_adjust_high_thd =  550; 
				dirty_adjust_low_thd = 500;                  
			}
		}

		printk(KERN_ERR"dirty_adjust_high_thd = %d, dirty_adjust_low_thd = %d, ps_adjust_max = %d\n", dirty_adjust_high_thd, dirty_adjust_low_thd, ps_adjust_max);

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

		lpi->ps_away_thd_set = low_threshold;
		lpi->ps_close_thd_set = high_threshold;
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_away_thd_set);
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDH,  lpi->ps_close_thd_set);

		// when open ps, we need open als too.
		lpi->ls_cmd &= CM36686_ALS_SD_MASK;
		_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
		lpi->ps_conf1_val &= CM36686_PS_SD_MASK;
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);                
#endif

		// enable int.
		lpi->ps_conf1_val |= CM36686_PS_INT_IN_AND_OUT;      
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    

		lpi->ps_enable = 1;  

		msleep(80); // must be more than PS_IT

		cm36686_control_and_report(lpi, CONTROL_PS, 1, 1);
	}

	mutex_unlock(&ps_enable_mutex);

	delay = atomic_read(&lpi->ps_poll_delay);

#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD
	wake_up(&enable_wq);	
#endif

	return ret;
}

static int cm36686_psensor_disable(struct cm36686_info *lpi)
{
	int ret = 0;

	mutex_lock(&ps_disable_mutex);

	printk(KERN_ERR"%s\n", __func__);

	if (lpi->ps_enable == 0) {
		dev_err(&lpi->i2c_client->dev, "already disabled\n");
		ret = 0;
	} 
	else 
	{
		lpi->ps_conf1_val &= CM36686_PS_INT_MASK;   // disable int
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    

		lpi->ps_enable = 0;  
	}

	mutex_unlock(&ps_disable_mutex);

	if (lpi->als_enable == 1)   // if als open , just close ps 
	{
		lpi->ps_conf1_val |= CM36686_PS_SD;
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    	
	}
	else    // if als close , close  als and ps 
	{
		lpi->ls_cmd |= CM36686_ALS_SD;
		_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);

		lpi->ps_conf1_val |= CM36686_PS_SD;
		_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    	        
	}

	if ((lpi->ps_enable == 0) &&(lpi->als_enable == 0)) 
	{
		cm36686_power_set(lpi, false);
	}

	return ret;
}

#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD
static void sample_work_func(struct work_struct *work)
{
	int i;
	int ret;
	uint16_t ps = 0;

	ret = cm36686_psensor_enable(g_ps_data);
	if (ret)
		return;

	msleep(10);
	for (i = 0; i < 10; i++)
	{
		if ((get_ps_adc_value(&ps) < 0) || ps <= 0)
		{
			continue;
		}
		if( (ps > 0) &&((ps_min == 0) || (ps_min > ps)))
			ps_min = ps;

		printk("%s, ps = %d  \n", __func__, ps);

		msleep(70);
	}

	if (ps_min > ps_adjust_max)  //ps_adjust_max
		ps_min = ps_adjust_max;

	ret = cm36686_psensor_disable(g_ps_data);
	if (ret)
		return;

	printk("%s ps_min:%d  \n", __func__, ps_min);
}

#endif /*CM36686_ALSPS_DYNAMIC_THRESHOLD*/

static void lightsensor_set_kvalue(struct cm36686_info *lpi)
{
	if (!lpi) {
		pr_err("%s: ls_info is empty\n", __func__);
		return;
	}

	dev_dbg(&lpi->i2c_client->dev, "%s: ALS calibrated als_kadc=0x%x\n",__func__, als_kadc);

	if (als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		dev_dbg(&lpi->i2c_client->dev, "%s: no ALS calibrated\n",__func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0 && lpi->als_kadc < 0x1000) ?
			lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}
	dev_dbg(&lpi->i2c_client->dev, "%s: als_kadc=0x%x, als_gadc=0x%x\n",__func__, lpi->als_kadc, lpi->als_gadc);
}

static int lightsensor_update_table(struct cm36686_info *lpi)
{
	uint32_t tmp_data[10];
	int i;
	for (i = 0; i < 10; i++) {
		tmp_data[i] = (uint32_t)(*(lpi->adc_table + i))* lpi->als_kadc / lpi->als_gadc;

		if (tmp_data[i] <= 0xFFFF)
			lpi->cali_table[i] = (uint16_t) tmp_data[i];
		else
			lpi->cali_table[i] = 0xFFFF;

		dev_dbg(&lpi->i2c_client->dev, "%s: Calibrated adc_table: data[%d], %x\n",__func__, i, lpi->cali_table[i]);
	}

	return 0;
}


static int cm36686_lightsensor_enable(struct cm36686_info *lpi)
{
	int ret = 0;
	unsigned int delay;

	printk(KERN_ERR"%s\n", __func__);

	if ((lpi->ps_enable == 0) &&(lpi->als_enable == 0)) 
	{
		cm36686_power_set(lpi, true);

		initial_cm36686(lpi);
		cm36686_ls_initial_cmd(lpi);
		cm36686_psensor_initial_cmd(lpi);
	}

	lpi->ls_cmd &= CM36686_ALS_SD_MASK;    
	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);

	lpi->als_enable = 1;

	msleep(50);

	delay = atomic_read(&lpi->ls_poll_delay);

	schedule_delayed_work(&lpi->ldwork,msecs_to_jiffies(delay));

	return ret;
}

static int cm36686_lightsensor_disable(struct cm36686_info *lpi)
{
	int ret = 0;

	printk(KERN_ERR"%s\n", __func__);

	cancel_delayed_work(&lpi->ldwork);

	if ( lpi->als_enable == 0 ) {
		dev_err(&lpi->i2c_client->dev, "already disabled\n");
		ret = 0;
	} else {
		lpi->als_enable = 0;
	}

	if (lpi->ps_enable == 0)   // if ps disable ,  als also close.
	{
		lpi->ls_cmd |= CM36686_ALS_SD;
		_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
	}

	if ((lpi->ps_enable == 0) &&(lpi->als_enable == 0)) 
	{
		cm36686_power_set(lpi, false);
	}

	return ret;
}

static ssize_t ps_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct cm36686_info *lpi = lp_info;
	int intr_val = -1;

	get_ps_adc_value(&value);
	if (gpio_is_valid(lpi->intr_pin))
		intr_val = gpio_get_value(lpi->intr_pin);

	ret = snprintf(buf, PAGE_SIZE, "ADC[0x%04X], ENABLE=%d intr_pin=%d\n",
			value, lpi->ps_enable, intr_val);

	return ret;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct cm36686_info *lpi = container_of(sensors_cdev,
			struct cm36686_info, ps_cdev);
	int ret;

	if (enable)
		ret = cm36686_psensor_enable(lpi);
	else
		ret = cm36686_psensor_disable(lpi);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ps_en;
	struct cm36686_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
			&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	dev_dbg(&lpi->i2c_client->dev, "%s: ps_en=%d\n",__func__, ps_en);

	if (ps_en)
		cm36686_psensor_enable(lpi);
	else
		cm36686_psensor_disable(lpi);

	return count;
}

static ssize_t ps_parameters_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;

	ret = snprintf(buf, PAGE_SIZE,
			"PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x\n",
			lpi->ps_close_thd_set, lpi->ps_away_thd_set);

	return ret;
}

static ssize_t ps_parameters_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	struct cm36686_info *lpi = lp_info;
	char *token[10];
	int i;
	unsigned long tmp;

	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	if (kstrtoul(token[0], 16, &tmp))
		return -EINVAL;
	lpi->ps_close_thd_set = tmp;

	if (kstrtoul(token[1], 16, &tmp))
		return -EINVAL;
	lpi->ps_away_thd_set = tmp;

	dev_dbg(&lpi->i2c_client->dev, "ps_close_thd_set:0x%x\n",lpi->ps_close_thd_set);
	dev_dbg(&lpi->i2c_client->dev, "ps_away_thd_set:0x%x\n",lpi->ps_away_thd_set);

	return count;
}

static ssize_t ps_conf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return sprintf(buf, "PS_CONF1 = 0x%x, PS_CONF3 = 0x%x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}
static ssize_t ps_conf_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int code1, code2;
	struct cm36686_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);
	dev_dbg(&lpi->i2c_client->dev, "PS_CONF1:0x%x PS_CONF3:0x%x\n",code1, code2);

	lpi->ps_conf1_val = code1;
	lpi->ps_conf3_val = code2;

	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);

	return count;
}

static ssize_t ps_thd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;
	ret = sprintf(buf, "%s ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set);
	return ret;	
}
static ssize_t ps_thd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int code;
	struct cm36686_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	lpi->ps_away_thd_set = code &0xFF;
	lpi->ps_close_thd_set = (code & 0xFF00)>>8;

	dev_dbg(&lpi->i2c_client->dev, "ps_away_thd_set:0x%x\n",lpi->ps_away_thd_set);
	dev_dbg(&lpi->i2c_client->dev, "ps_close_thd_set:0x%x\n",lpi->ps_close_thd_set);

	return count;
}

static ssize_t ps_hw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",
			lpi->ps_conf1_val, lpi->ps_conf3_val, lpi->ps_close_thd_set, lpi->ps_away_thd_set);

	return ret;
}
static ssize_t ps_hw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int code;

	sscanf(buf, "0x%x", &code);

	return count;
}

static ssize_t ls_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "ADC[0x%04X] => level %d\n",lpi->current_adc, lpi->current_level);

	return ret;
}

static int ls_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct cm36686_info *lpi = container_of(sensors_cdev,
			struct cm36686_info, als_cdev);
	int ret;

	if (enable)
		ret = cm36686_lightsensor_enable(lpi);
	else
		ret = cm36686_lightsensor_disable(lpi);

	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: set auto light sensor fail\n",__func__);
		return -EIO;
	}

	return 0;
}

static ssize_t ls_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm36686_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		ret = cm36686_lightsensor_enable(lpi);
	} else {
		lpi->ls_calibrate = 0;
		ret = cm36686_lightsensor_disable(lpi);
	}

	dev_dbg(&lpi->i2c_client->dev, "als_enable:0x%x\n",lpi->als_enable);
	dev_dbg(&lpi->i2c_client->dev, "ls_calibrate:0x%x\n",lpi->ls_calibrate);
	dev_dbg(&lpi->i2c_client->dev, "ls_auto:0x%x\n", ls_auto);

	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: set auto light sensor fail\n",__func__);
		return ret;
	}

	return count;
}


static ssize_t ls_kadc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x",lpi->als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);

	mutex_lock(&als_get_adc_mutex);
	if (kadc_temp != 0) {
		lpi->als_kadc = kadc_temp;
		if (lpi->als_gadc != 0) {
			if (lightsensor_update_table(lpi) < 0)
				dev_err(&lpi->i2c_client->dev, "%s: update ls table fail\n",__func__);
			else
				dev_dbg(&lpi->i2c_client->dev, "%s: als_gadc =0x%x wait to be set\n",__func__, lpi->als_gadc);
		}
	} else {
		dev_err(&lpi->i2c_client->dev, "%s: als_kadc can't be set to zero\n",__func__);
	}

	mutex_unlock(&als_get_adc_mutex);
	return count;
}


static ssize_t ls_gadc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "gadc = 0x%x\n", lpi->als_gadc);

	return ret;
}

static ssize_t ls_gadc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int gadc_temp = 0;

	sscanf(buf, "%d", &gadc_temp);

	mutex_lock(&als_get_adc_mutex);
	if (gadc_temp != 0) {
		lpi->als_gadc = gadc_temp;
		if (lpi->als_kadc != 0) {
			if (lightsensor_update_table(lpi) < 0)
				dev_err(&lpi->i2c_client->dev, "%s: update ls table fail\n",__func__);
		} else {
			dev_dbg(&lpi->i2c_client->dev, "als_kadc =0x%x wait to be set\n",lpi->als_kadc);
		}
	} else {
		dev_err(&lpi->i2c_client->dev, "als_gadc can't be set to zero\n");
	}
	mutex_unlock(&als_get_adc_mutex);
	return count;
}


static ssize_t ls_adc_table_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
				"[CM36686]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
				i, *(lp_info->adc_table + i),*(lp_info->adc_table + i),
				i, *(lp_info->cali_table + i),*(lp_info->cali_table + i));
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	struct cm36686_info *lpi = lp_info;
	char *token[10];
	uint16_t tempdata[10];
	int i;

	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		tempdata[i] = simple_strtoul(token[i], NULL, 16);
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			dev_err(&lpi->i2c_client->dev,"adc_table[%d] =  0x%x error\n",i, tempdata[i]);
			return count;
		}
	}
	mutex_lock(&als_get_adc_mutex);
	for (i = 0; i < 10; i++)
		lpi->adc_table[i] = tempdata[i];

	if (lightsensor_update_table(lpi) < 0)
		dev_err(&lpi->i2c_client->dev, "%s: update ls table fail\n",__func__);
	mutex_unlock(&als_get_adc_mutex);

	return count;
}

static ssize_t ls_conf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return sprintf(buf, "ALS_CONF = %x\n", lpi->ls_cmd);
}
static ssize_t ls_conf_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;

	dev_dbg(&lpi->i2c_client->dev, "ALS_CONF:0x%x\n", lpi->ls_cmd);

	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return snprintf(buf, PAGE_SIZE, "%d\n",atomic_read(&lpi->ls_poll_delay));
}

static ssize_t ls_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if ((interval_ms < CM36686_LS_MIN_POLL_DELAY) ||
			(interval_ms > CM36686_LS_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&lpi->ls_poll_delay, (unsigned int) interval_ms);
	return count;
}

static int ls_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	struct cm36686_info *lpi = container_of(sensors_cdev,
			struct cm36686_info, als_cdev);

	if ((delay_msec < CM36686_LS_MIN_POLL_DELAY) ||
			(delay_msec > CM36686_LS_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&lpi->ls_poll_delay, delay_msec);

	return 0;
}

static ssize_t ps_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return snprintf(buf, PAGE_SIZE, "%d\n",atomic_read(&lpi->ps_poll_delay));
}

static ssize_t ps_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if ((interval_ms < CM36686_PS_MIN_POLL_DELAY) ||
			(interval_ms > CM36686_PS_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&lpi->ps_poll_delay, (unsigned int) interval_ms);
	return count;
}

static int ps_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	struct cm36686_info *lpi = container_of(sensors_cdev,struct cm36686_info, als_cdev);

	if ((delay_msec < CM36686_PS_MIN_POLL_DELAY) ||
			(delay_msec > CM36686_PS_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&lpi->ps_poll_delay, delay_msec);
	return 0;
}

static ssize_t ls_fLevel_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fLevel = %d\n", fLevel);
}
static ssize_t ls_fLevel_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int value=0;
	sscanf(buf, "%d", &value);
	(value>=0)?(value=min(value,10)):(value=max(value,-1));
	fLevel=value;
	input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel);
	input_sync(lpi->ls_input_dev);

	msleep(1000);
	fLevel=-1;
	return count;
}

static int lightsensor_setup(struct cm36686_info *lpi)
{
	int ret;
	int range;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err("[LS][CM36686 error]%s: could not allocate ls input device\n",__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "light";
	lpi->ls_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);

	range = get_als_range();
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 65535, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: can not register ls input device\n",__func__);
		goto err_free_ls_input_device;
	}

	return ret;

err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct cm36686_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err("[PS][CM36686 error]%s: could not allocate ps input device\n",__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	lpi->ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: could not register ps input device\n",__func__);
		goto err_free_ps_input_device;
	}

	return ret;

err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}


static int initial_cm36686(struct cm36686_info *lpi)
{
	int val, ret;
	uint16_t idReg;

	val = gpio_get_value(lpi->intr_pin);
	dev_dbg(&lpi->i2c_client->dev, "%s, INTERRUPT GPIO val = %d\n",__func__, val);

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);

	return ret;
}

static int cm36686_setup(struct cm36686_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(20);
	ret = gpio_request(lpi->intr_pin, "gpio_cm36686_intr");
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: gpio %d request failed (%d)\n",__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: fail to set gpio %d as input (%d)\n",__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}


	ret = initial_cm36686(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36686 error]%s: fail to initial cm36686 (%d)\n",__func__, ret);
		goto fail_free_intr_pin;
	}

	/*Default disable P sensor and L sensor*/
	cm36686_ls_initial_cmd(lpi);
	cm36686_psensor_initial_cmd(lpi);

	ret = request_any_context_irq(lpi->irq,cm36686_irq_handler,IRQF_TRIGGER_LOW,"cm36686",lpi);
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: req_irq(%d) fail for gpio %d (%d)\n",__func__, lpi->irq,lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	irq_set_irq_wake(lpi->irq, 1);

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

static int cm36686_parse_dt(struct device *dev,
		struct cm36686_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32	levels[CM36686_LEVELS_SIZE], i;
	u32 temp_val;
	int rc;

	rc = of_get_named_gpio_flags(np, "capella,interrupt-gpio",0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} else {
		pdata->intr = rc;
	}

	rc = of_property_read_u32_array(np, "capella,levels", levels,CM36686_LEVELS_SIZE);
	if (rc) {
		dev_err(dev, "Unable to read levels data\n");
		return rc;
	} else {
		for (i = 0; i < CM36686_LEVELS_SIZE; i++)
			pdata->levels[i] = levels[i];
	}

	rc = of_property_read_u32(np, "capella,ps_close_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd_set\n");
		return rc;
	} else {
		pdata->ps_close_thd_set = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_away_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd_set\n");
		return rc;
	} else {
		pdata->ps_away_thd_set = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ls_cmd", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ls_cmd\n");
		return rc;
	} else {
		pdata->ls_cmd = (u16)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_conf1_val", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_conf1_val\n");
		return rc;
	} else {
		pdata->ps_conf1_val = (u16)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_conf3_val", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_conf3_val\n");
		return rc;
	} else {
		pdata->ps_conf3_val = (u16)temp_val;
	}

	pdata->polling = of_property_read_bool(np, "capella,use-polling");

	rc = of_property_read_u32(np, "capella,als_gain", &temp_val);
	if (rc) {
		cm36686_als_gain = 100;
	} else {
		cm36686_als_gain = temp_val;
	}

	return 0;
}

static int create_sysfs_interfaces(struct device *dev,
		struct device_attribute *attributes, int len)
{
	int i;
	int err;
	for (i = 0; i < len; i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev,
		struct device_attribute *attributes, int len)
{
	int i;
	for (i = 0; i < len; i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static struct device_attribute light_attr[] = {
	__ATTR(ls_adc, 0444, ls_adc_show, NULL),
	__ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store),
	__ATTR(ls_gadc, 0664, ls_gadc_show, ls_gadc_store),
	__ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store),
	__ATTR(ls_adc_table, 0664,ls_adc_table_show, ls_adc_table_store),
	__ATTR(poll_delay, 0664, ls_poll_delay_show,ls_poll_delay_store),
	__ATTR(enable, 0664,ls_enable_show, ls_enable_store),
};

static struct device_attribute proximity_attr[] = {
	__ATTR(enable, 0664, ps_adc_show, ps_enable_store),
	__ATTR(ps_parameters, 0664,ps_parameters_show, ps_parameters_store),
	__ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store),
	__ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store),
	__ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store),
	__ATTR(poll_delay, 0664, ps_poll_delay_show,ps_poll_delay_store),
	__ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store),
};

static ssize_t cm36686_prox_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	uint16_t reading;

	if(lp_info->ps_enable == 0)
		return snprintf(buf, PAGE_SIZE, "%d\n",  0);

	get_ps_adc_value(&reading);

	printk_x(SHOW_LOG,"%s ps_value:%d \n", __func__, reading);

	return snprintf(buf, PAGE_SIZE, "%d\n", reading);
}
static struct kobj_attribute prox_raw = 
{
	.attr = {"prox_raw", 0444},
	.show = cm36686_prox_raw_show,
};

static ssize_t cm36686_log_control_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;

	if (sscanf(buf, "%d", (unsigned int *)&data) == 1)
	{
		printk_log_level = data;

		printk(KERN_ERR"%s log level = %d\n", __func__, printk_log_level);        
	}
	else
	{
		printk("%s error.\n", __func__);
	}

	return count;
}

static struct kobj_attribute log_control = 
{
	.attr = {"log_control", 0220},
	.store = cm36686_log_control_store,
};


static ssize_t cm36686_esd_triger_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;

	if (sscanf(buf, "%d", (unsigned int *)&data) == 1)
	{
		printk(KERN_ERR"%s start\n", __func__);        

		cm36686_esd_handle(lp_info, 1);

		printk(KERN_ERR"%s over\n", __func__);        
	}
	else
	{
		printk("%s error.\n", __func__);
	}

	return count;
}

static struct kobj_attribute esd_triger = 
{
	.attr = {"esd_triger", 0220},
	.store = cm36686_esd_triger_store,
};


#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD    // Do not modify it as wilful, because it is used for algo.
static ssize_t cm36686_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s", "cm36686");
}
static struct kobj_attribute name = 
{
	.attr = {"name", 0444},
	.show = cm36686_name_show,
};
static ssize_t cm36686_high_light_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	uint16_t adc_value = 0;

	if(lp_info->ps_enable == 0)
		return snprintf(buf, PAGE_SIZE, "%d\n",  0);

	cm36686_get_ls_adc_value(&adc_value, 0);

	printk_x(SHOW_LOG,"%s : %d \n", __func__, adc_value);

	return snprintf(buf, PAGE_SIZE, "%d\n",  (adc_value < 8000)? 0:1);
}
static struct kobj_attribute is_high_light = 
{
	.attr = {"is_high_light", 0444},
	.show = cm36686_high_light_show,
};
static ssize_t cm36686_ps_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", g_ps_data->ps_enable);
}
static struct kobj_attribute ps_enable = 
{
	.attr = {"ps_enable", 0444},
	.show = cm36686_ps_enable_show,
};
static ssize_t cm36686_alsps_ps_thd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data[2];
	if (sscanf(buf, "%d %d", (unsigned int *)&data[0],(unsigned int *)&data[1]) == 2)
	{
		//printk("algo set --- low_thd:%5d, high_thd:%5d \n", data[0], data[1]);

		if(lp_info->ps_enable == 0)
			return count;

		g_ps_data->ps_away_thd_set = data[0];
		g_ps_data->ps_close_thd_set = data[1];
		_cm36686_I2C_Write_Word(g_ps_data->slave_addr, PS_THDL,  g_ps_data->ps_away_thd_set);
		_cm36686_I2C_Write_Word(g_ps_data->slave_addr, PS_THDH,  g_ps_data->ps_close_thd_set);
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
	.store = cm36686_alsps_ps_thd_store,
};

static ssize_t cm36686_alsps_ps_min_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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
static ssize_t cm36686_alsps_ps_min_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ps_min);
}
static struct kobj_attribute ps_min_val = 
{
	.attr = {"ps_min", 0664},
	.show = cm36686_alsps_ps_min_show,
	.store = cm36686_alsps_ps_min_store,
};

static ssize_t cm36686_alsps_algo_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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
	.store = cm36686_alsps_algo_info_store,
};
static ssize_t cm36686_algo_wakeup_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	wait_event_interruptible(enable_wq, g_ps_data->ps_enable);
	printk(KERN_ERR"wait ps enable done\n");
	return snprintf(buf, PAGE_SIZE, "%d\n", g_ps_data->ps_enable);
}
static struct kobj_attribute algo_wakeup = 
{
	.attr = {"algo_wakeup", 0444},
	.show = cm36686_algo_wakeup_show,
};
static ssize_t cm36686_far_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",  g_ps_data->ps_distance_last);
}
static struct kobj_attribute far_status = 
{
	.attr = {"far_status", 0444},
	.show = cm36686_far_status_show,
};
#endif


static const struct attribute *cm36686_ftm_attrs[] = 
{
	&prox_raw.attr,
	&log_control.attr,   
	&esd_triger.attr,
#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD	
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
static struct dev_ftm cm36686_ftm;


static int cm36686_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm36686_info *lpi;
	struct cm36686_platform_data *pdata;

	dev_err(&client->dev, "[%s]\n",__func__);

	lpi = kzalloc(sizeof(struct cm36686_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory for pdata\n");
			ret = -ENOMEM;
			goto err_platform_data_null;
		}

		ret = cm36686_parse_dt(&client->dev, pdata);
		pdata->slave_addr = client->addr;
		if (ret) {
			dev_err(&client->dev, "Failed to get pdata from device tree\n");
			goto err_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "%s: Assign platform_data error!!\n",	__func__);
			ret = -EBUSY;
			goto err_platform_data_null;
		}
	}

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);

	lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->power = pdata->power;
	lpi->power_enabled = false;
	lpi->slave_addr = pdata->slave_addr;
	lpi->i2c_err_time = 0;
	lpi->esd_handle_flag = 0;
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	lpi->polling = pdata->polling;
	atomic_set(&lpi->ls_poll_delay,(unsigned int) CM36686_LS_DEFAULT_POLL_DELAY);
	atomic_set(&lpi->ps_poll_delay,(unsigned int) CM36686_PS_DEFAULT_POLL_DELAY);

	lpi->ls_cmd  = pdata->ls_cmd;

	lpi->record_clear_int_fail=0;

	dev_dbg(&lpi->i2c_client->dev, "[PS][CM36686] %s: ls_cmd 0x%x\n", __func__, lpi->ls_cmd);

	if (pdata->ls_cmd == 0) {
		lpi->ls_cmd  = CM36686_ALS_IT_80MS ;
	}

	lp_info = lpi;

	mutex_init(&CM36686_control_mutex);

	mutex_init(&als_get_adc_mutex);

	mutex_init(&ps_enable_mutex);
	mutex_init(&ps_disable_mutex);
	mutex_init(&ps_get_adc_mutex);

	/*
	 * SET LUX STEP FACTOR HERE
	 * if adc raw value one step = 5/100 = 1/20 = 0.05 lux
	 * the following will set the factor 0.05 = 1/20
	 * and lpi->golden_adc = 1;
	 * set als_kadc = (ALS_CALIBRATED << 16) | 20;
	 */

	als_kadc = (ALS_CALIBRATED << 16) | 10;
	lpi->golden_adc = 100;
	lpi->ls_calibrate = 0;

	lightsensor_set_kvalue(lpi);
	ret = lightsensor_update_table(lpi);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: update ls table fail\n",__func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm36686_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][CM36686 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	wake_lock_init(&lpi->ps_wake_lock, WAKE_LOCK_SUSPEND, "cm36686_proximity");

	ret = cm36686_sensor_regulator_configure(lpi, true);
	if (ret){
		pr_err("[PS_ERR][CM36686 error]%s: cm36686_sensor_regulator_configure error!\n", __func__);
		goto err_power_init;
	}

	ret = cm36686_power_set(lpi, true);
	if (ret){
		pr_err("[PS_ERR][CM36686 error]%s: cm36686_power_set error!\n", __func__);
		goto err_power_set;
	}

	ret = cm36686_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36686 error]%s: cm36686_setup error!\n", __func__);
		goto err_cm36686_setup;
	}

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: lightsensor_setup error!!\n",__func__);
		goto err_lightsensor_setup;
	}

	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: psensor_setup error!!\n",__func__);
		goto err_psensor_setup;
	}

	ret = create_sysfs_interfaces(&lpi->ls_input_dev->dev, light_attr,ARRAY_SIZE(light_attr));
	if (ret < 0) {
		dev_err(&client->dev, "failed to create sysfs\n");
		goto err_input_cleanup;
	}

	ret = create_sysfs_interfaces(&lpi->ps_input_dev->dev, proximity_attr,ARRAY_SIZE(proximity_attr));
	if (ret < 0) {
		dev_err(&client->dev, "failed to create sysfs\n");
		goto err_light_sysfs_cleanup;
	}

	lpi->als_cdev = sensors_light_cdev;
	lpi->als_cdev.sensors_enable = ls_enable_set;
	lpi->als_cdev.sensors_poll_delay = ls_poll_delay_set;
	lpi->als_cdev.min_delay = CM36686_LS_MIN_POLL_DELAY * 1000;

	lpi->ps_cdev = sensors_proximity_cdev;
	lpi->ps_cdev.sensors_enable = ps_enable_set;
	lpi->ps_cdev.sensors_poll_delay = ps_poll_delay_set;
	lpi->ps_cdev.min_delay = CM36686_PS_MIN_POLL_DELAY * 1000;

	ret = sensors_classdev_register(&client->dev, &lpi->als_cdev);
	if (ret)
		goto err_proximity_sysfs_cleanup;

	ret = sensors_classdev_register(&client->dev, &lpi->ps_cdev);
	if (ret)
		goto err_create_class_sysfs;

	mutex_init(&wq_lock);
	INIT_DELAYED_WORK(&lpi->ldwork, cm36686_lsensor_delay_work_handler);

#ifdef CM36686_ALSPS_DYNAMIC_THRESHOLD  
	INIT_DELAYED_WORK(&sample_ps_work, sample_work_func);
	queue_delayed_work(lpi->lp_wq,&sample_ps_work, msecs_to_jiffies(5000)); 

	init_waitqueue_head(&enable_wq);
#endif

	cm36686_ftm.name = "als_prox";
	cm36686_ftm.i2c_client = client;
	cm36686_ftm.attrs = cm36686_ftm_attrs;
	cm36686_ftm.priv_data = lpi;
	register_single_dev_ftm(&cm36686_ftm);   

	g_ps_data = lpi;

	ret = cm36686_power_set(lpi, false);        

	dev_err(&lpi->i2c_client->dev, "%s: Probe success!\n", __func__);

	return ret;
err_create_class_sysfs:
	sensors_classdev_unregister(&lpi->als_cdev);
err_proximity_sysfs_cleanup:
	remove_sysfs_interfaces(&lpi->ps_input_dev->dev, proximity_attr,ARRAY_SIZE(proximity_attr));
err_light_sysfs_cleanup:
	remove_sysfs_interfaces(&lpi->ls_input_dev->dev, light_attr,ARRAY_SIZE(light_attr));
err_input_cleanup:
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
err_psensor_setup:
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_lightsensor_setup:
err_cm36686_setup:
	cm36686_power_set(lpi, false);
err_power_set:    
	cm36686_sensor_regulator_configure(lpi, false);
err_power_init:
	wake_lock_destroy(&lpi->ps_wake_lock);
	destroy_workqueue(lpi->lp_wq);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
	mutex_destroy(&CM36686_control_mutex);
	mutex_destroy(&als_get_adc_mutex);
	mutex_destroy(&ps_enable_mutex);
	mutex_destroy(&ps_disable_mutex);
	mutex_destroy(&ps_get_adc_mutex);
err_parse_dt:
	if (client->dev.of_node && (pdata != NULL))
		devm_kfree(&client->dev, pdata);
err_platform_data_null:
	kfree(lpi);
	dev_err(&client->dev, "%s:error exit! ret = %d\n", __func__, ret);

	return ret;
}

static void cm36686_control_and_report(struct cm36686_info *lpi, uint8_t mode, uint16_t param, int report)
{
	uint16_t ps_data = 0;
	int val;

	printk(KERN_ERR"%s mode:%d, param:0x%x, report:%d \n", __func__, mode, param, report);

	mutex_lock(&CM36686_control_mutex);

#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY

	if (report && (lpi->ps_enable)) {
		int ps_status = 0;
		if (mode == CONTROL_PS)
			ps_status = PS_CLOSE_AND_AWAY;
		else if (mode == CONTROL_INT_ISR_REPORT) {
			if (param & INT_FLAG_PS_IF_CLOSE)
				ps_status |= PS_CLOSE;
			if (param & INT_FLAG_PS_IF_AWAY)
				ps_status |= PS_AWAY;
		}

		if (ps_status != 0) {
			get_ps_adc_value(&ps_data);
			switch (ps_status) {
				case PS_CLOSE_AND_AWAY:
					val = (ps_data >= lpi->ps_close_thd_set)? 0 : 1;
					break;
				case PS_AWAY:
					val = 1;
					break;
				case PS_CLOSE:
					val = 0;
					break;
			};
			lpi->ps_distance_last = val;
			pr_err("%s ps_data:%d val:%d low_thd:%d high_thd:%d \n", __func__, ps_data, val, lpi->ps_away_thd_set,lpi->ps_close_thd_set);
			input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
			input_sync(lpi->ps_input_dev);
			wake_lock_timeout(&lpi->ps_wake_lock, 2*HZ);
		}
	}

	mutex_unlock(&CM36686_control_mutex);

	//pr_err("%s out \n", __func__);
}

static int cm36686_suspend(struct device *dev)
{
	struct cm36686_info *lpi = lp_info;

	printk(KERN_ERR"%s enter \n", __func__);

	lpi->als_state_suspend_resume = lpi->als_enable;

	if (lpi->als_state_suspend_resume) {
		if (cm36686_lightsensor_disable(lpi))
			printk(KERN_ERR"%s:failed .\n", __func__);
	}

	if((lpi->ps_enable) && (lpi->irq))
	{
		disable_irq_nosync(lpi->irq);
	}

	return 0;
}

static int cm36686_resume(struct device *dev)
{
	struct cm36686_info *lpi = lp_info;

	printk(KERN_ERR"%s enter \n", __func__);

	if (lpi->als_state_suspend_resume) {
		if (cm36686_lightsensor_enable(lpi))
			printk(KERN_ERR"%s:failed .\n", __func__);
	}

	if (lpi->irq && lpi->ps_enable) 
	{
		enable_irq(lpi->irq);
	}

	return 0;
}

static const struct dev_pm_ops cm36686_pm_ops = {
	.suspend	= cm36686_suspend,
	.resume 	= cm36686_resume,
};

static const struct i2c_device_id cm36686_i2c_id[] = {
	{CM36686_I2C_NAME, 0},
	{}
};

static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "capella,cm36686",},
	{ },
};

static struct i2c_driver cm36686_driver = {
	.id_table = cm36686_i2c_id,
	.probe = cm36686_probe,
	.driver = {
		.name = "cm36686",
		.owner = THIS_MODULE,
		.of_match_table = cm36686_match_table,
		.pm = &cm36686_pm_ops,		
	},
};

static int __init cm36686_init(void)
{
	return i2c_add_driver(&cm36686_driver);
}

static void __exit cm36686_exit(void)
{
	i2c_del_driver(&cm36686_driver);
}

module_init(cm36686_init);
module_exit(cm36686_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36686 Driver");
