/*************************************************************
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : lis3dh_acc.c
 ** Description : 
 ** Date        : 2014-07-19 16:37
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : lis3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *                    : Samuel Huo (samuel.huo@st.com)
 * Version            : V.1.1.0
 * Date               : 07/10/2012
 * Description        : LIS3DH accelerometer sensor driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 05/11/09
 First Release;
 Revision 1.0.3 22/01/2010
 Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
 modified _get_acceleration_data function;
 modified _update_odr function;
 manages 2 interrupts;
 Revision 1.0.6 15/11/2010
 supports sysfs;
 no more support for ioctl;
 Revision 1.0.7 26/11/2010
 checks for availability of interrupts pins
 correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
 corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9 07/25/2011
 Romove several unused functions,add 5ms delay in init,change sysfs attributes.
 Revision 1.1.0 07/10/2012
 To replace some deprecated functions for 3.4 kernel; to pass the checkpatch's formatting requirement;
 To add regulator request;

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/pm.h>
#include	<linux/input/lis3dh.h>
#include	<linux/module.h>
#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>
#include	<linux/sensors.h>
#include <linux/sensors_ftm.h>
#define	DEBUG	1

#define	G_MAX		16000


#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE	0x01
#define LIS3DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS3DH_ACC	0x33	/*	Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/


#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1

#define LIS3DH_ACC_PM_OFF		0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES	0x07


#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */



#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */
#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */


struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
	{    1, ODR1250 },
	{    3, ODR400  },
	{    5, ODR200  },
	{   10, ODR100  },
	{   20, ODR50   },
	{   40, ODR25   },
	{  100, ODR10   },
	{ 1000, ODR1    },
};

struct lis3dh_acc_data {
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;
	struct sensors_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	bool power_on_flag;
	struct regulator *vdd;
	struct regulator *vio;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef DEBUG
	u8 reg_addr;
#endif
	//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/30, add for gsensor cali */
	s16   cali_sw[LIS3DH_AXES_NUM+1];
	//#endif /*VENDOR_EDIT*/
};

static struct sensors_classdev lis3dh_acc_cdev = {
	.name = "lis3dh-accel",
	.vendor = "STMicroelectronics",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 5000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#define LIS3DH_VDD_MIN_UV	2700000
#define LIS3DH_VDD_MAX_UV	3300000
#define LIS3DH_VI2C_MIN_UV	1750000
#define LIS3DH_VI2C_MAX_UV	1950000

static int lis3dh_acc_config_regulator(struct lis3dh_acc_data *data, bool on)
{
	int rc = 0;

	printk(KERN_ERR"%s, on = %d\n", __func__, on);

	if (!on) 
	{
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, LIS3DH_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, LIS3DH_VI2C_MAX_UV);

		regulator_put(data->vio);
	} 
	else 
	{
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, LIS3DH_VDD_MIN_UV, LIS3DH_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev, "Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev, "Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio, LIS3DH_VI2C_MIN_UV, LIS3DH_VI2C_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev, "Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, LIS3DH_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);

	return rc;
}

static int lis3dh_acc_device_power_on(struct lis3dh_acc_data *data, bool on)
{
	int rc = 0;

	printk(KERN_ERR"%s, on = %d, power_on_flag = %d\n", __func__, on, data->power_on_flag);

	if (data->power_on_flag != on) 
	{
		if (!on) 
		{
			rc = regulator_disable(data->vdd);
			if (rc) {
				dev_err(&data->client->dev,"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}

			rc = regulator_disable(data->vio);
			if (rc) 
			{
				dev_err(&data->client->dev, "Regulator vio disable failed rc=%d\n", rc);

				rc = regulator_enable(data->vdd);
				dev_err(&data->client->dev, "Regulator vio re-enabled rc=%d\n", rc);
				if (!rc) 
				{
					rc = -EBUSY;
					return rc;
				}
			}

			data->power_on_flag = on;

			return rc;
		} 
		else 
		{
			rc = regulator_enable(data->vdd);
			if (rc) {
				dev_err(&data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}

			rc = regulator_enable(data->vio);
			if (rc) {
				dev_err(&data->client->dev, "Regulator vio enable failed rc=%d\n", rc);
				regulator_disable(data->vdd);
				return rc;
			}

			data->power_on_flag = on;

			msleep(20);
		}
	}

	return 0;
}

static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc,
		u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_hw_init(struct lis3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	buf[0] = WHO_AM_I;
	{
		int i = 0;
		for (i = 0; i< 3;i++)
		{
			err = lis3dh_acc_i2c_read(acc, buf, 1);
			if (err >= 0)  break;
			msleep(10);
		}
	}
	if (err < 0) {
		dev_warn(&acc->client->dev,"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_LIS3DH_ACC) {
		dev_err(&acc->client->dev,"device unknown. Expected: 0x%x, Replies: 0x%x\n",WHOAMI_LIS3DH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = lis3dh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = lis3dh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],buf[1], err);
	return err;
}

int lis3dh_acc_update_g_range(struct lis3dh_acc_data *acc, u8 new_g_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
		case LIS3DH_ACC_G_2G:

			sensitivity = SENSITIVITY_2G;
			break;
		case LIS3DH_ACC_G_4G:

			sensitivity = SENSITIVITY_4G;
			break;
		case LIS3DH_ACC_G_8G:

			sensitivity = SENSITIVITY_8G;
			break;
		case LIS3DH_ACC_G_16G:

			sensitivity = SENSITIVITY_16G;
			break;
		default:
			dev_err(&acc->client->dev, "invalid g range requested: %u\n",
					new_g_range);
			return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 4,
		 * which contains g range setting */
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return err;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",buf[0], buf[1], err);

	return err;
}

int lis3dh_acc_update_odr(struct lis3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = lis3dh_acc_odr_table[i].mask;

	config[1] |= LIS3DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = lis3dh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",config[0], config[1], err);

	return err;
}


static int lis3dh_acc_register_write(struct lis3dh_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		return err;
	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,int *xyz, int cali_flag)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
			: (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
			: (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
			: (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG
	dev_dbg(&acc->client->dev, "%s read x=%d, y=%d, z=%d\n",LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif

	//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/30, add for gsensor cali */
	//when user calibrat gsensor, need original data from lis3dh ,can not add cali_sw data
	if(!cali_flag) 
	{
		xyz[LIS3DH_AXIS_X] += acc->cali_sw[LIS3DH_AXIS_X];
		xyz[LIS3DH_AXIS_Y] += acc->cali_sw[LIS3DH_AXIS_Y];
		xyz[LIS3DH_AXIS_Z] += acc->cali_sw[LIS3DH_AXIS_Z];
	}
#ifdef DEBUG
	dev_dbg(&acc->client->dev, "%s after cali: the data x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	//#endif /*VENDOR_EDIT*/

	return err;
}

static void lis3dh_acc_report_values(struct lis3dh_acc_data *acc,
		int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int lis3dh_acc_enable(struct lis3dh_acc_data *acc)
{
	int err = -1;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lis3dh_acc_device_power_on(acc, true);
		if (err) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		err = lis3dh_acc_hw_init(acc);
		if (err) {
			lis3dh_acc_device_power_on(acc, false);
			atomic_set(&acc->enabled, 0);
			return err;
		}

		schedule_delayed_work(&acc->input_work,msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lis3dh_acc_disable(struct lis3dh_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		lis3dh_acc_device_power_on(acc, false);
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lis3dh_acc_i2c_read(acc, &data, 1);
	if (err < 0)
		return err;
	ret = snprintf(buf, 4, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis3dh_acc_register_write(acc, x, reg, new_val);
	if (err < 0)
		return err;
	acc->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,struct device_attribute *attr,char *buf)
{
	int val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return snprintf(buf, 8, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	lis3dh_acc_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,struct device_attribute *attr, char *buf)
{
	char val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->g_range;
	switch (val) {
		case LIS3DH_ACC_G_2G:
			range = 2;
			break;
		case LIS3DH_ACC_G_4G:
			range = 4;
			break;
		case LIS3DH_ACC_G_8G:
			range = 8;
			break;
		case LIS3DH_ACC_G_16G:
			range = 16;
			break;
	}
	mutex_unlock(&acc->lock);
	return snprintf(buf, 4, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	lis3dh_acc_update_g_range(acc, val);
	mutex_unlock(&acc->lock);
	return size;
}

//#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2014/07/29, add for gsensor cali */
static int calculate_gsensor_cali_data(struct i2c_client *client, int data[LIS3DH_AXES_NUM+1])
{
	u8 i = 0;
	int average_offset[3] = {0};
	int read_buff[3] = {0};
	int calibration_buf[LIS3DH_AXES_NUM+1] = {0};
	int res = -1;
	char buff[LIS3DH_BUFSIZE] = {0};
	struct lis3dh_acc_data *obj = (struct lis3dh_acc_data*)i2c_get_clientdata(client);	

	if (NULL == client)
	{	
		printk("null pointer!!\n");
		return -EINVAL;
	}

	printk("/*****************************************/\n");

	for (i = 0; i < 20; i ++)
	{		
		strcpy(buff,"calibration gsensor");
		if((res = lis3dh_acc_get_acceleration_data(obj, read_buff, 1)))
		{        
			printk("I2C error: ret value=%d", res);
			return -EIO;
		}

		//sscanf(buff, "%x %x %x", &read_buff[LIS3DH_AXIS_X],&read_buff[LIS3DH_AXIS_Y],&read_buff[LIS3DH_AXIS_Z] );

		printk("calculate_gsensor: (%5d %5d %5d)\n",read_buff[LIS3DH_AXIS_X],read_buff[LIS3DH_AXIS_Y],read_buff[LIS3DH_AXIS_Z]);

		average_offset[LIS3DH_AXIS_X] += read_buff[LIS3DH_AXIS_X];
		average_offset[LIS3DH_AXIS_Y] += read_buff[LIS3DH_AXIS_Y];
		average_offset[LIS3DH_AXIS_Z] += read_buff[LIS3DH_AXIS_Z];
		msleep(20);
	}

	average_offset[LIS3DH_AXIS_X] /= 20;
	average_offset[LIS3DH_AXIS_Y] /= 20;
	average_offset[LIS3DH_AXIS_Z] /= 20;

	calibration_buf[LIS3DH_AXIS_X] = 0-average_offset[LIS3DH_AXIS_X];
	calibration_buf[LIS3DH_AXIS_Y] = 0-average_offset[LIS3DH_AXIS_Y] ;
	calibration_buf[LIS3DH_AXIS_Z] = 1024-average_offset[LIS3DH_AXIS_Z] ;

	if(abs(calibration_buf[LIS3DH_AXIS_X]) <= 150
			&& abs(calibration_buf[LIS3DH_AXIS_Y]) <= 150
			&& abs(calibration_buf[LIS3DH_AXIS_Z])<= 300)
	{
		calibration_buf[LIS3DH_CALIBRATION_FLAG] = 1;	// calibration ok

		obj->cali_sw[LIS3DH_AXIS_X] = calibration_buf[LIS3DH_AXIS_X];	// write the calibration data
		obj->cali_sw[LIS3DH_AXIS_Y] = calibration_buf[LIS3DH_AXIS_Y];
		obj->cali_sw[LIS3DH_AXIS_Z] = calibration_buf[LIS3DH_AXIS_Z];	
		obj->cali_sw[LIS3DH_CALIBRATION_FLAG] = calibration_buf[LIS3DH_CALIBRATION_FLAG];
	}
	else   
	{
		calibration_buf[LIS3DH_CALIBRATION_FLAG] = 0;	// calibration failed
	}


	data[LIS3DH_AXIS_X] = calibration_buf[LIS3DH_AXIS_X];
	data[LIS3DH_AXIS_Y] = calibration_buf[LIS3DH_AXIS_Y];
	data[LIS3DH_AXIS_Z] = calibration_buf[LIS3DH_AXIS_Z];
	data[LIS3DH_CALIBRATION_FLAG] = calibration_buf[LIS3DH_CALIBRATION_FLAG];

	printk("\ngsensor offset: (%5d, %5d, %5d)\n\n", data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);	

	printk("/*****************************************/\n");

	return 0;	
}
static ssize_t attr_get_cali(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int cali_buff[LIS3DH_AXES_NUM+1] = {0};

	mutex_lock(&acc->lock);
	calculate_gsensor_cali_data(acc->client, cali_buff);
	mutex_unlock(&acc->lock);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", cali_buff[0], cali_buff[1], cali_buff[2], cali_buff[3]);
}

static ssize_t attr_set_cali(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	int i, ret;
	char *token[10];
	s16 cali_buf[3] = {0};
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);

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

	mutex_unlock(&acc->lock);

	for (i = 0; i < 3; i++)
		acc->cali_sw[i] = cali_buf[i];

	return size;
}
//#endif /*VENDOR_EDIT*/

static ssize_t attr_get_enable(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis3dh_acc_enable(acc);
	else
		lis3dh_acc_disable(acc);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,struct device_attribute *attr,char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,struct device_attribute *attr,char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int rc;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lis3dh_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,char *buf)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lis3dh_acc_i2c_read(acc, &data, 1);
	/* TODO: error need to be managed */
	ret = snprintf(buf, 8, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(poll_delay, 0664, attr_get_polling_rate,attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(cali, 0664, attr_get_cali, attr_set_cali),	
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim,attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),

#ifdef DEBUG
	__ATTR(reg_value, 0664, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0220, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
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

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int lis3dh_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,struct lis3dh_acc_data, cdev);
	int err;

	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = delay_msec;
	err = lis3dh_acc_update_odr(acc, delay_msec);
	mutex_unlock(&acc->lock);
	return err;
}

static int lis3dh_acc_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enable)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,struct lis3dh_acc_data, cdev);
	int err;

	if (enable)
		err = lis3dh_acc_enable(acc);
	else
		err = lis3dh_acc_disable(acc);
	return err;
}

static void lis3dh_acc_input_work_func(struct work_struct *work)
{
	struct lis3dh_acc_data *acc;

	int xyz[3] = { 0 };
	int err;
	int val;

	acc = container_of((struct delayed_work *)work,struct lis3dh_acc_data,	input_work);

	val = atomic_read(&acc->enabled);
	if(val == 0)
		return ;

	mutex_lock(&acc->lock);
	err = lis3dh_acc_get_acceleration_data(acc, xyz, 0);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lis3dh_acc_report_values(acc, xyz);
	//printk("lis3dh_acc_input_work_func: (%5d %5d %5d)\n",xyz[LIS3DH_AXIS_X],xyz[LIS3DH_AXIS_Y],xyz[LIS3DH_AXIS_Z]);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

int lis3dh_acc_input_open(struct input_dev *input)
{
	struct lis3dh_acc_data *acc = input_get_drvdata(input);

	return lis3dh_acc_enable(acc);
}

void lis3dh_acc_input_close(struct input_dev *dev)
{
	struct lis3dh_acc_data *acc = input_get_drvdata(dev);

	lis3dh_acc_disable(acc);
}

static int lis3dh_acc_validate_pdata(struct lis3dh_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||acc->pdata->axis_map_y > 2 ||acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev,"invalid axis_map value x:%u y:%u z%u\n",
				acc->pdata->axis_map_x,acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev,"invalid negate value x:%u y:%u z:%u\n",
				acc->pdata->negate_x,acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis3dh_acc_input_init(struct lis3dh_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lis3dh_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lis3dh_acc_input_open;
	acc->input_dev->close = lis3dh_acc_input_close;
	acc->input_dev->name = ACCEL_INPUT_DEV_NAME;
	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,"unable to register input device %s\n",acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lis3dh_acc_input_cleanup(struct lis3dh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int lis3dh_parse_dt(struct device *dev,struct lis3dh_acc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "st,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis_map_y\n");
		return rc;
	} else {
		pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map-z\n");
		return rc;
	} else {
		pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
			case 2:
				pdata->g_range = LIS3DH_ACC_G_2G;
				break;
			case 4:
				pdata->g_range = LIS3DH_ACC_G_4G;
				break;
			case 8:
				pdata->g_range = LIS3DH_ACC_G_8G;
				break;
			case 16:
				pdata->g_range = LIS3DH_ACC_G_16G;
				break;
			default:
				pdata->g_range = LIS3DH_ACC_G_2G;
				break;
		}
	}

	pdata->negate_x = of_property_read_bool(np, "st,negate-x");

	pdata->negate_y = of_property_read_bool(np, "st,negate-y");

	pdata->negate_z = of_property_read_bool(np, "st,negate-z");

	pdata->gpio_int1 = of_get_named_gpio_flags(dev->of_node,"st,gpio-int1", 0, NULL);

	pdata->gpio_int2 = of_get_named_gpio_flags(dev->of_node,"st,gpio-int2", 0, NULL);
	return 0;
}

/*--------------------------------------------------------------------------*/
static struct lis3dh_acc_data *g_acc_data;
static ssize_t lis3dh_acc_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;
	int ret = -EINVAL;
	struct lis3dh_acc_data *acc = g_acc_data;

	sscanf(buf, "%x", &data);
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		ret = lis3dh_acc_device_power_on(acc, true);
		if (ret) {
			atomic_set(&acc->enabled, 0);
		}
	}
	if (ret == 0)
		printk("%s: Enable sensor SUCCESS\n",__func__);

	return count;
}
static ssize_t lis3dh_acc_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = g_acc_data;

	return snprintf(buf, PAGE_SIZE, "accelerator:%d\n", atomic_read(&acc->enabled));
}
static struct kobj_attribute enable = 
{
	.attr = {"enable", 0664},
	.show = lis3dh_acc_enable_show,
	.store = lis3dh_acc_enable_store,
};
static ssize_t lis3dh_acc_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = g_acc_data;
	int xyz[3];
	int ret;

	ret = lis3dh_acc_get_acceleration_data(acc, xyz, 0);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", xyz[0], xyz[1], xyz[2]);
}
static struct kobj_attribute accel_raw = 
{
	.attr = {"accel_raw", 0444},
	.show = lis3dh_acc_raw_show,
};

static ssize_t lis3dh_acc_cali_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int i, ret;
	char *token[10];
	s16 cali_buf[3] = {0};
	struct lis3dh_acc_data *acc = g_acc_data;

	mutex_lock(&acc->lock);

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

	mutex_unlock(&acc->lock);
	printk(KERN_ERR"%s cali_buf[0]:%d  cali_buf[1]:%d  cali_buf[2]:%d \n", __func__, cali_buf[0], cali_buf[1], cali_buf[2]);
	for (i = 0; i < 3; i++)
		acc->cali_sw[i] = cali_buf[i];

	return count;
}
static ssize_t lis3dh_acc_cali_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = g_acc_data;

	int cali_buff[LIS3DH_AXES_NUM+1] = {0};

	mutex_lock(&acc->lock);
	calculate_gsensor_cali_data(acc->client, cali_buff);
	mutex_unlock(&acc->lock);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", cali_buff[0], cali_buff[1], cali_buff[2], cali_buff[3]);
}
static struct kobj_attribute cali = 
{
	.attr = {"cali", 0664},
	.show = lis3dh_acc_cali_show,
	.store = lis3dh_acc_cali_store,
};

static const struct attribute *lis3dh_attrs[] = 
{
	&enable.attr,
	&accel_raw.attr,
	&cali.attr,
	NULL
};

static struct dev_ftm lis3dh_ftm;
/*--------------------------------------------------------------------------*/
static int lis3dh_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lis3dh_acc_data *acc;
	int err = -1;

	printk("%s start... \n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	acc = kzalloc(sizeof(struct lis3dh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,"failed to allocate memory for module data: %d\n", err);
		goto exit_check_functionality_failed;
	}

	memset(acc, 0, sizeof(struct lis3dh_acc_data));
	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,"failed to allocate memory for pdata: %d\n",err);
		goto err_mutexunlock;
	}
	memset(acc->pdata, 0 , sizeof(*acc->pdata));

	if (client->dev.of_node) 
	{
		err = lis3dh_parse_dt(&client->dev, acc->pdata);
		if (err) 
		{
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto exit_kfree_pdata;
		}
	} 
	else if (client->dev.platform_data != NULL) 
	{
		memcpy(acc->pdata, client->dev.platform_data,sizeof(*acc->pdata));
	} 
	else 
	{
		dev_err(&client->dev, "No valid platform data. exiting.\n");
		err = -ENODEV;
		goto exit_kfree_pdata;
	}

	err = lis3dh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;

	err = lis3dh_acc_config_regulator(acc, true);
	if (err) {
		dev_err(&acc->client->dev,"power config failed: %d\n", err);
		goto exit_kfree_pdata;
	}

	err = lis3dh_acc_device_power_on(acc, true);
	if (err) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_power_on;
	}

	err = lis3dh_acc_hw_init(acc);
	if (err) {
		dev_err(&client->dev, "hw init failed: %d\n", err);
		goto err_power_off;
	}

	atomic_set(&acc->enabled, 1);

	err = lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "device LIS3DH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	acc->cdev = lis3dh_acc_cdev;
	acc->cdev.sensors_enable = lis3dh_acc_enable_set;
	acc->cdev.sensors_poll_delay = lis3dh_acc_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &acc->cdev);
	if (err) {
		dev_err(&client->dev,"class device create failed: %d\n", err);
		goto err_remove_sysfs_int;
	}

	lis3dh_acc_device_power_on(acc, false);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	mutex_unlock(&acc->lock);
	g_acc_data = acc;
	lis3dh_ftm.name = "accel";
	lis3dh_ftm.i2c_client = acc->client;
	lis3dh_ftm.priv_data = acc;
	lis3dh_ftm.attrs = lis3dh_attrs;
	err = register_single_dev_ftm(&lis3dh_ftm);
	if(err)
	{
		dev_dbg(&client->dev, "register_single_dev_ftm failed \n");       
		goto err_unreg_sensor_class;
	}

	printk("%s ok... \n", __func__);

	return 0;

err_unreg_sensor_class:
	sensors_classdev_unregister(&acc->cdev);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lis3dh_acc_input_cleanup(acc);
err_power_off:
	lis3dh_acc_device_power_on(acc, false);
err_power_on:
	lis3dh_acc_config_regulator(acc, false);        
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	kfree(acc);
exit_check_functionality_failed:
	dev_err(&client->dev, "%s: Driver Init failed\n", LIS3DH_ACC_DEV_NAME);
	return err;
}

static int lis3dh_acc_remove(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	sensors_classdev_unregister(&acc->cdev);
	lis3dh_acc_input_cleanup(acc);
	lis3dh_acc_device_power_on(acc, false);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}


static int lis3dh_acc_resume(struct device *dev)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);

	printk(KERN_ERR"%s\n", __func__);

	if (acc->on_before_suspend)
		return lis3dh_acc_enable(acc);
	return 0;
}

static int lis3dh_acc_suspend(struct device *dev)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);

	printk(KERN_ERR"%s\n", __func__);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return lis3dh_acc_disable(acc);
}

static const struct i2c_device_id lis3dh_acc_id[]
= { { LIS3DH_ACC_DEV_NAME, 0 }, { }, };

static struct of_device_id lis3dh_acc_match_table[] = {
	{ .compatible = "st,lis3dh", },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);

static const struct dev_pm_ops lis3dh_pm_ops = {
	.suspend	= lis3dh_acc_suspend,
	.resume 	= lis3dh_acc_resume,
};

static struct i2c_driver lis3dh_acc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DH_ACC_DEV_NAME,
		.of_match_table = lis3dh_acc_match_table,
		.pm = &lis3dh_pm_ops,
	},
	.probe = lis3dh_acc_probe,
	.remove = lis3dh_acc_remove,
	.id_table = lis3dh_acc_id,
};

static int __init lis3dh_acc_init(void)
{
	return i2c_add_driver(&lis3dh_acc_driver);
}

static void __exit lis3dh_acc_exit(void)
{
	i2c_del_driver(&lis3dh_acc_driver);
	return;
}

module_init(lis3dh_acc_init);
module_exit(lis3dh_acc_exit);

MODULE_DESCRIPTION("lis3dh digital accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Samuel Huo, STMicroelectronics");
MODULE_LICENSE("GPL");

