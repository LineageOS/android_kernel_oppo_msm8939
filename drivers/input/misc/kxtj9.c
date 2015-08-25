/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : kxtj9.c
 ** Description : 
 ** Date        : 2014-11-04 17:39
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtj9.h>
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include	<linux/workqueue.h>
#include <linux/sensors_ftm.h>

#define ACCEL_INPUT_DEV_NAME	"accelerometer"
#define DEVICE_NAME		"kxtj9"

#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F		3
#define ODR200F		4
#define ODR400F		5
#define ODR800F		6
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3
/* POWER SUPPLY VOLTAGE RANGE */
#define KXTJ9_VDD_MIN_UV	2000000
#define KXTJ9_VDD_MAX_UV	3300000
#define KXTJ9_VIO_MIN_UV	1750000
#define KXTJ9_VIO_MAX_UV	1950000

//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/29, add for gsensor cali */
#define KXTJ9_AXIS_X				0
#define KXTJ9_AXIS_Y				1
#define KXTJ9_AXIS_Z				2
#define KXTJ9_CALIBRATION_FLAG     3
#define KXTJ9_AXES_NUM				3
#define KXTJ9_BUFSIZE				256
//#endif /*VENDOR_EDIT*/

static int g_CTRL_REG1 = 0x1B;
static int g_INT_CTRL1 = 0x1E;

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */

static struct sensors_classdev sensors_cdev = {
	.name = "kxtj9-accel",
	.vendor = "Kionix",
	.version = 1,
	.handle = 0,
	.type = 1,
	.max_range = "19.6",
	.resolution = "0.01",
	.sensor_power = "0.2",
	.min_delay = 2000,	/* microsecond */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,	/* millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtj9_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
	struct input_dev *input_dev;

	unsigned int last_poll_interval;
	bool	enable;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
	bool	power_enabled;
	struct regulator *vdd;
	struct regulator *vio;
	struct sensors_classdev cdev;

	struct delayed_work input_work;

	//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/30, add for gsensor cali */
	s16   cali_sw[3+1];
	//#endif /*VENDOR_EDIT*/
};


static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9,int *xyz, int cali_flag)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;

	err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0)
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");

	x = le16_to_cpu(acc_data[tj9->pdata.axis_map_x]);
	y = le16_to_cpu(acc_data[tj9->pdata.axis_map_y]);
	z = le16_to_cpu(acc_data[tj9->pdata.axis_map_z]);

	/* 8 bits output mode support */
	if (!(tj9->ctrl_reg1 & RES_12BIT)) {
		x <<= 4;
		y <<= 4;
		z <<= 4;
	}

	x >>= tj9->shift;
	y >>= tj9->shift;
	z >>= tj9->shift;

	x = tj9->pdata.negate_x ? -x : x;
	y = tj9->pdata.negate_y ? -y : y;
	z = tj9->pdata.negate_z ? -z : z;

	//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/30, add for gsensor cali */
	//when user calibrat gsensor, need original data from kxtj9 ,can not add cali_sw data
	if(!cali_flag) 
	{
		x += tj9->cali_sw[KXTJ9_AXIS_X];
		y += tj9->cali_sw[KXTJ9_AXIS_Y];
		z += tj9->cali_sw[KXTJ9_AXIS_Z];
	}
#ifdef DEBUG
	dev_dbg(&tj9->client->dev, "%s after cali: the data x=%d, y=%d, z=%d\n",
			__func__, x, y, z);
#endif
	//#endif /*VENDOR_EDIT*/

	xyz[KXTJ9_AXIS_X] = x; 
	xyz[KXTJ9_AXIS_Y] = y; 
	xyz[KXTJ9_AXIS_Z] = z; 

	input_report_abs(tj9->input_dev, ABS_X, x);
	input_report_abs(tj9->input_dev, ABS_Y, y);
	input_report_abs(tj9->input_dev, ABS_Z, z);
	input_sync(tj9->input_dev);
}


//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/29, add for gsensor cali */
static int calculate_gsensor_cali_data(struct i2c_client *client, int data[KXTJ9_AXES_NUM+1])
{
	u8 i = 0;
	int average_offset[3] = {0};
	int read_buff[3] = {0};
	int calibration_buf[KXTJ9_AXES_NUM+1] = {0};
	char buff[KXTJ9_BUFSIZE] = {0};
	struct kxtj9_data *obj = (struct kxtj9_data*)i2c_get_clientdata(client);	

	if (NULL == client)
	{	
		printk("null pointer!!\n");
		return -EINVAL;
	}

	printk("/*****************************************/\n");

	for (i = 0; i < 20; i ++)
	{		
		strcpy(buff,"calibration gsensor");
		kxtj9_report_acceleration_data(obj, read_buff, 1);
		if(read_buff[0] == 0 && read_buff[1] == 0 && read_buff[2] == 0)
		{        
			printk("I2C error: read xyz error \n");
			return -EIO;
		}

		//sscanf(buff, "%x %x %x", &read_buff[KXTJ9_AXIS_X],&read_buff[KXTJ9_AXIS_Y],&read_buff[KXTJ9_AXIS_Z] );

		printk("calculate_gsensor: (%5d %5d %5d)\n",read_buff[KXTJ9_AXIS_X],read_buff[KXTJ9_AXIS_Y],read_buff[KXTJ9_AXIS_Z]);

		average_offset[KXTJ9_AXIS_X] += read_buff[KXTJ9_AXIS_X];
		average_offset[KXTJ9_AXIS_Y] += read_buff[KXTJ9_AXIS_Y];
		average_offset[KXTJ9_AXIS_Z] += read_buff[KXTJ9_AXIS_Z];
		msleep(20);
	}

	average_offset[KXTJ9_AXIS_X] /= 20;
	average_offset[KXTJ9_AXIS_Y] /= 20;
	average_offset[KXTJ9_AXIS_Z] /= 20;

	calibration_buf[KXTJ9_AXIS_X] = 0-average_offset[KXTJ9_AXIS_X];
	calibration_buf[KXTJ9_AXIS_Y] = 0-average_offset[KXTJ9_AXIS_Y] ;
	calibration_buf[KXTJ9_AXIS_Z] = 1024-average_offset[KXTJ9_AXIS_Z] ;

	if(abs(calibration_buf[KXTJ9_AXIS_X]) <= 150
			&& abs(calibration_buf[KXTJ9_AXIS_Y]) <= 150
			&& abs(calibration_buf[KXTJ9_AXIS_Z])<= 300)
	{
		calibration_buf[KXTJ9_CALIBRATION_FLAG] = 1;	// calibration ok

		obj->cali_sw[KXTJ9_AXIS_X] = calibration_buf[KXTJ9_AXIS_X];	// write the calibration data
		obj->cali_sw[KXTJ9_AXIS_Y] = calibration_buf[KXTJ9_AXIS_Y];
		obj->cali_sw[KXTJ9_AXIS_Z] = calibration_buf[KXTJ9_AXIS_Z];	
		obj->cali_sw[KXTJ9_CALIBRATION_FLAG] = calibration_buf[KXTJ9_CALIBRATION_FLAG];
	}
	else   
	{
		calibration_buf[KXTJ9_CALIBRATION_FLAG] = 0;	// calibration failed
	}


	data[KXTJ9_AXIS_X] = calibration_buf[KXTJ9_AXIS_X];
	data[KXTJ9_AXIS_Y] = calibration_buf[KXTJ9_AXIS_Y];
	data[KXTJ9_AXIS_Z] = calibration_buf[KXTJ9_AXIS_Z];
	data[KXTJ9_CALIBRATION_FLAG] = calibration_buf[KXTJ9_CALIBRATION_FLAG];

	printk("\ngsensor offset: (%5d, %5d, %5d)\n\n", data[KXTJ9_AXIS_X], data[KXTJ9_AXIS_Y], data[KXTJ9_AXIS_Z]);	

	printk("/*****************************************/\n");

	return 0;	
}
static ssize_t attr_get_cali(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct kxtj9_data *acc = dev_get_drvdata(dev);
	int cali_buff[KXTJ9_AXES_NUM+1] = {0};

	mutex_lock(&acc->input_dev->mutex);
	calculate_gsensor_cali_data(acc->client, cali_buff);
	mutex_unlock(&acc->input_dev->mutex);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", cali_buff[0], cali_buff[1], cali_buff[2], cali_buff[3]);
}

static ssize_t attr_set_cali(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	int i, ret;
	char *token[10];
	s16 cali_buf[3] = {0};
	struct kxtj9_data *acc = dev_get_drvdata(dev);

	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	for (i = 0; i < 3; i++)
	{
		ret = kstrtol(token[i], 10, (long *)&(cali_buf[i]));
		if (ret < 0) {

			printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",__func__, ret);
			return ret;
		}
	}

	for (i = 0; i < 3; i++)
		acc->cali_sw[i] = cali_buf[i];

	return size;
}

static DEVICE_ATTR(cali, S_IRUGO|S_IWUSR|S_IWGRP,attr_get_cali, attr_set_cali);

static void kxtj9_acc_input_work_func(struct work_struct *work)
{
	struct kxtj9_data *tj9;
	int xyz[3] = { 0 };

	//printk(KERN_EMERG"%s  line:%d \n", __func__, __LINE__); 
	tj9 = container_of((struct delayed_work *)work,struct kxtj9_data,	input_work);

	kxtj9_report_acceleration_data(tj9, xyz, 0);

	schedule_delayed_work(&tj9->input_work, msecs_to_jiffies(tj9->last_poll_interval));
}

static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{
	switch (new_g_range) {
		case KXTJ9_G_2G:
			tj9->shift = 4;
			break;
		case KXTJ9_G_4G:
			tj9->shift = 3;
			break;
		case KXTJ9_G_8G:
			tj9->shift = 2;
			break;
		default:
			return -EINVAL;
	}

	tj9->ctrl_reg1 &= 0xe7;
	tj9->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
		tj9->data_ctrl = kxtj9_odr_table[i].mask;
		if (poll_interval < kxtj9_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(tj9->client, g_CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, g_CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

static int kxtj9_power_on(struct kxtj9_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			if (rc) {
				dev_err(&data->client->dev,"Regulator vdd enable failed rc=%d\n",rc);
			}
		}
		data->power_enabled = false;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
		}
		data->power_enabled = true;
	} else {
		dev_warn(&data->client->dev,"Power on=%d. enabled=%d\n",on, data->power_enabled);
	}

	return rc;
}

static int kxtj9_power_init(struct kxtj9_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, KXTJ9_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, KXTJ9_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, KXTJ9_VDD_MIN_UV,KXTJ9_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio, KXTJ9_VIO_MIN_UV,KXTJ9_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, KXTJ9_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
	int err = 0;
	err = kxtj9_power_on(tj9, true);
	if (err) {
		dev_err(&tj9->client->dev, "power on failed");
		goto err_exit;
	}

	/* Use 80ms as vendor suggested. */
	msleep(80);

err_exit:
	dev_dbg(&tj9->client->dev, "soft power on complete err=%d.\n", err);
	return err;
}

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
	int err;

	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, g_CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		dev_err(&tj9->client->dev, "soft power off failed\n");

	if (tj9->pdata.power_off)
		tj9->pdata.power_off();
	else
		kxtj9_power_on(tj9, false);

	dev_dbg(&tj9->client->dev, "soft power off complete.\n");
	return ;
}

static int kxtj9_enable(struct kxtj9_data *tj9)
{
	int err;

	err = kxtj9_device_power_on(tj9);
	if (err < 0)
		return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, g_CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);
	if (err < 0)
		return err;

	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, g_CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	err = kxtj9_update_odr(tj9, tj9->last_poll_interval);
	if (err < 0)
		return err;

	schedule_delayed_work(&tj9->input_work, msecs_to_jiffies(tj9->last_poll_interval));  
	return 0;
}

static void kxtj9_disable(struct kxtj9_data *tj9)
{
	cancel_delayed_work_sync(&tj9->input_work);
	kxtj9_device_power_off(tj9);
}


static void kxtj9_init_input_device(struct kxtj9_data *tj9,
		struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = ACCEL_INPUT_DEV_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj9->client->dev;
}

static int kxtj9_setup_input_device(struct kxtj9_data *tj9)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj9->input_dev = input_dev;

	input_set_drvdata(input_dev, tj9);

	kxtj9_init_input_device(tj9, input_dev);

	err = input_register_device(tj9->input_dev);
	if (err) {
		dev_err(&tj9->client->dev,"unable to register input polled device %s: %d\n",tj9->input_dev->name, err);
		input_free_device(tj9->input_dev);
		return err;
	}

	return 0;
}

static int kxtj9_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enabled)
{
	struct kxtj9_data *tj9 = container_of(sensors_cdev,struct kxtj9_data, cdev);
	struct input_dev *input_dev = tj9->input_dev;

	printk(KERN_ERR"%s  line:%d \n", __func__, __LINE__); 
	mutex_lock(&input_dev->mutex);

	if (enabled == 0) {
		kxtj9_disable(tj9);
		tj9->enable = false;
	} else if (enabled == 1) {
		if (!kxtj9_enable(tj9)) {
			tj9->enable = true;
		}
	} else {
		dev_err(&tj9->client->dev,"Invalid value of input, input=%d\n", enabled);
		mutex_unlock(&input_dev->mutex);
		return -EINVAL;
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static ssize_t kxtj9_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	return snprintf(buf, 4, "%d\n", tj9->enable);
}

static ssize_t kxtj9_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error < 0)
		return error;

	error = kxtj9_enable_set(&tj9->cdev, data);
	if (error < 0)
		return error;
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		kxtj9_enable_show, kxtj9_enable_store);

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */
static int kxtj9_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	struct kxtj9_data *tj9 = container_of(sensors_cdev,
			struct kxtj9_data, cdev);
	struct input_dev *input_dev = tj9->input_dev;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	tj9->last_poll_interval = max(delay_msec, tj9->pdata.min_interval);

	if (tj9->enable) {
		kxtj9_update_odr(tj9, tj9->last_poll_interval);
	}
	mutex_unlock(&input_dev->mutex);

	return 0;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll_delay(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tj9->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll_delay(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	error = kxtj9_poll_delay_set(&tj9->cdev, interval);
	if (error < 0)
		return error;
	return count;
}

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP,
		kxtj9_get_poll_delay, kxtj9_set_poll_delay);

static struct attribute *kxtj9_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/11/06, add for gsensor cali. */
	&dev_attr_cali.attr,
	//#endif /*VENDOR_EDIT*/
	NULL
};

static struct attribute_group kxtj9_attribute_group = {
	.attrs = kxtj9_attributes
};

static int kxtj9_verify(struct kxtj9_data *tj9)
{
	int retval;

	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&tj9->client->dev, "read err int source\n");
		goto out;
	}
	printk(KERN_ERR"%s  product:0x%x \n", __func__, retval); 
	if (retval == 0x15)     //for kx023
	{
		g_CTRL_REG1 = 0x18;
		g_INT_CTRL1 = 0x1C;
		return 0;
	}
	retval = (retval != 0x05 && retval != 0x07 && retval != 0x08)? -EIO : 0;

out:
	return retval;
}

static int kxtj9_parse_dt(struct device *dev,struct kxtj9_platform_data *kxtj9_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "kionix,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		kxtj9_pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "kionix,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		kxtj9_pdata->init_interval = temp_val;
	}

	rc = of_property_read_u32(np, "kionix,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		kxtj9_pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kionix,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis_map_y\n");
		return rc;
	} else {
		kxtj9_pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kionix,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map-z\n");
		return rc;
	} else {
		kxtj9_pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kionix,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
			case 2:
				kxtj9_pdata->g_range = KXTJ9_G_2G;
				break;
			case 4:
				kxtj9_pdata->g_range = KXTJ9_G_4G;
				break;
			case 8:
				kxtj9_pdata->g_range = KXTJ9_G_8G;
				break;
			default:
				kxtj9_pdata->g_range = KXTJ9_G_2G;
				break;
		}
	}

	kxtj9_pdata->negate_x = of_property_read_bool(np, "kionix,negate-x");

	kxtj9_pdata->negate_y = of_property_read_bool(np, "kionix,negate-y");

	kxtj9_pdata->negate_z = of_property_read_bool(np, "kionix,negate-z");

	if (of_property_read_bool(np, "kionix,res-12bit"))
		kxtj9_pdata->res_ctl = RES_12BIT;
	else
		kxtj9_pdata->res_ctl = RES_8BIT;

	return 0;
}

/*--------------------------------------------------------------------------*/
static struct kxtj9_data *g_kxtj9_data;

static ssize_t kxtj9_acc_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;
	int ret = -EINVAL;
	struct kxtj9_data *acc = g_kxtj9_data;

	sscanf(buf, "%x", &data);
	if (data && acc->enable == 0) {
		ret = kxtj9_enable(acc);
		if (ret < 0) {
			printk("%s: Enable sensor Fail\n",__func__);
		}
	}
	else if (!data && acc->enable == 1)
	{
		kxtj9_disable(acc);
	}
	if (ret == 0)
		printk("%s: Enable sensor SUCCESS\n",__func__);

	return count;
}
static ssize_t kxtj9_acc_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct kxtj9_data *acc = g_kxtj9_data;

	return snprintf(buf, PAGE_SIZE, "accelerator:%d\n", acc->enable);
}
static struct kobj_attribute enable = 
{
	.attr = {"enable", 0664},
	.show = kxtj9_acc_enable_show,
	.store = kxtj9_acc_enable_store,
};
static ssize_t kxtj9_acc_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct kxtj9_data *acc = g_kxtj9_data;

	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;

	err = kxtj9_i2c_read(acc, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0)
		dev_err(&acc->client->dev, "accelerometer data read failed\n");

	x = le16_to_cpu(acc_data[acc->pdata.axis_map_x]);
	y = le16_to_cpu(acc_data[acc->pdata.axis_map_y]);
	z = le16_to_cpu(acc_data[acc->pdata.axis_map_z]);

	/* 8 bits output mode support */
	if (!(acc->ctrl_reg1 & RES_12BIT)) {
		x <<= 4;
		y <<= 4;
		z <<= 4;
	}

	x >>= acc->shift;
	y >>= acc->shift;
	z >>= acc->shift;

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", x, y, z);
}
static struct kobj_attribute accel_raw = 
{
	.attr = {"accel_raw", 0444},
	.show = kxtj9_acc_raw_show,
};

static ssize_t kxtj9_acc_cali_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int i, ret;
	char *token[10];
	s16 cali_buf[3] = {0};
	struct kxtj9_data *acc = g_kxtj9_data;

	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	for (i = 0; i < 3; i++)
	{
		ret = kstrtol(token[i], 10, (long *)&(cali_buf[i]));
		if (ret < 0) {

			printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
					__func__, ret);
			return ret;
		}
	}

	printk(KERN_ERR"%s cali_buf[0]:%d  cali_buf[1]:%d  cali_buf[2]:%d \n", __func__, cali_buf[0], cali_buf[1], cali_buf[2]);
	for (i = 0; i < 3; i++)
		acc->cali_sw[i] = cali_buf[i];

	return count;
}
static ssize_t kxtj9_acc_cali_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct kxtj9_data *acc = g_kxtj9_data;

	int cali_buff[KXTJ9_AXES_NUM+1] = {0};

	calculate_gsensor_cali_data(acc->client, cali_buff);

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", cali_buff[0], cali_buff[1], cali_buff[2], cali_buff[3]);
}
static struct kobj_attribute cali = 
{
	.attr = {"cali", 0664},
	.show = kxtj9_acc_cali_show,
	.store = kxtj9_acc_cali_store,
};

static const struct attribute *kxtj9_ftm_attrs[] = 
{
	&enable.attr,
	&accel_raw.attr,
	&cali.attr,
	NULL
};

static struct dev_ftm kxtj9_ftm;
/*--------------------------------------------------------------------------*/
static int kxtj9_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct kxtj9_data *tj9;
	int err;

	dev_err(&client->dev, "%s: kxtj9_probe start ...\n", __func__); 
	if (!i2c_check_functionality(client->adapter,I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		memset(&tj9->pdata, 0 , sizeof(tj9->pdata));
		err = kxtj9_parse_dt(&client->dev, &tj9->pdata);
		if (err) {
			dev_err(&client->dev,"Unable to parse platfrom data err=%d\n", err);
			return err;
		}
	} else {
		if (client->dev.platform_data)
			tj9->pdata = *(struct kxtj9_platform_data *)client->dev.platform_data;
		else {
			dev_err(&client->dev,"platform data is NULL; exiting\n");
			return -EINVAL;
		}
	}

	tj9->client = client;
	tj9->power_enabled = false;

	if (tj9->pdata.init) {
		err = tj9->pdata.init();
		if (err < 0)
			goto err_free_mem;
	}

	err = kxtj9_power_init(tj9, true);
	if (err < 0) {
		dev_err(&tj9->client->dev, "power init failed! err=%d", err);
		goto err_pdata_exit;
	}

	err = kxtj9_device_power_on(tj9);
	if (err < 0) {
		dev_err(&client->dev, "power on failed! err=%d\n", err);
		goto err_power_deinit;
	}

	err = kxtj9_verify(tj9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_power_off;
	}

	i2c_set_clientdata(client, tj9);

	tj9->ctrl_reg1 = tj9->pdata.res_ctl | tj9->pdata.g_range;
	tj9->last_poll_interval = tj9->pdata.init_interval;

	tj9->cdev = sensors_cdev;

	/* The min_delay is used by userspace and the unit is microsecond. */
	tj9->cdev.min_delay = tj9->pdata.min_interval * 1000;
	tj9->cdev.delay_msec = tj9->pdata.init_interval;
	tj9->cdev.sensors_enable = kxtj9_enable_set;
	tj9->cdev.sensors_poll_delay = kxtj9_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &tj9->cdev);
	if (err) {
		dev_err(&client->dev, "class device create failed: %d\n", err);
		goto err_power_off;
	}

	err = kxtj9_setup_input_device(tj9);
	if (err)
		goto err_power_off;

	INIT_DELAYED_WORK(&tj9->input_work, kxtj9_acc_input_work_func);

	err = sysfs_create_group(&client->dev.kobj, &kxtj9_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
		goto err_destroy_input;
	}

	dev_err(&client->dev, "%s: kxtj9_probe OK.\n", __func__);
	kxtj9_device_power_off(tj9);
	g_kxtj9_data= tj9;
	kxtj9_ftm.name = "accel";
	kxtj9_ftm.i2c_client = tj9->client;
	kxtj9_ftm.priv_data = tj9;
	kxtj9_ftm.attrs = kxtj9_ftm_attrs;
	register_single_dev_ftm(&kxtj9_ftm);

	return 0;

err_destroy_input:
	input_unregister_device(tj9->input_dev);
err_power_off:
	kxtj9_device_power_off(tj9);
err_power_deinit:
	kxtj9_power_init(tj9, false);
err_pdata_exit:
	if (tj9->pdata.exit)
		tj9->pdata.exit();
err_free_mem:
	kfree(tj9);

	dev_err(&client->dev, "%s: kxtj9_probe err=%d\n", __func__, err);
	return err;
}

static int kxtj9_remove(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	kxtj9_device_power_off(tj9);
	kxtj9_power_init(tj9, false);

	if (tj9->pdata.exit)
		tj9->pdata.exit();

	kfree(tj9);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj9_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users && tj9->enable)
		kxtj9_disable(tj9);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int kxtj9_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users && tj9->enable)
		kxtj9_enable(tj9);

	mutex_unlock(&input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

static struct of_device_id kxtj9_match_table[] = {
	{ .compatible = "kionix,kxtj9", },
	{ },
};


MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = kxtj9_match_table,
		.pm	= &kxtj9_pm_ops,
	},
	.probe		= kxtj9_probe,
	.remove		= kxtj9_remove,
	.id_table	= kxtj9_id,
};

module_i2c_driver(kxtj9_driver);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
