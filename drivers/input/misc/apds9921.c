/*
 *  apds9921.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2015 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2015 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/sensors_ftm.h>
#include <linux/regulator/consumer.h>
#include <soc/oppo/oppo_project.h>
#include <linux/wakelock.h>

#define APDS9921_DRV_NAME	"apds9921"
#define DRIVER_VERSION		"1.0.0"

#define ABS_LIGHT	0x29	// added to support LIGHT - light sensor

#define CONFIG_SENSOR_APDS9921

#define APDS9921_PS_DETECTION_THRESHOLD		50
#define APDS9921_PS_HSYTERESIS_THRESHOLD	40
#define APDS9921_PS_PULSE_NUMBER			8

#define APDS9921_ALS_THRESHOLD_HSYTERESIS	1	/* 1 = 1% */

#define APDS9921_ALS_CAL_LOOP			1
#define APDS9921_ALS_CAL_LUX			300
#define APDS9921_ALS_CAL_LUX_LOW		((70*APDS9921_ALS_CAL_LUX)/100)		// 70% of 300 lux
#define	APDS9921_ALS_CAL_LUX_HIGH		((130*APDS9921_ALS_CAL_LUX)/100)	// 130% of 300 lux

#define APDS9921_PS_CAL_LOOP			10
#define APDS9921_PS_CAL_CROSSTALK_LOW	0
#define	APDS9921_PS_CAL_CROSSTALK_HIGH	200
#define APDS9921_PS_CROSSTALK_DELTA		20

/* Change History
 *
 * 1.0.0	Fundamental Functions of APDS-9921
 *
 */

#define APDS_IOCTL_PS_ENABLE				1
#define APDS_IOCTL_PS_GET_ENABLE			2
#define APDS_IOCTL_PS_POLL_DELAY			3
#define APDS_IOCTL_ALS_ENABLE				4
#define APDS_IOCTL_ALS_GET_ENABLE			5
#define APDS_IOCTL_ALS_POLL_DELAY			6
#define APDS_IOCTL_PS_GET_PDATA				7	// ps_data
#define APDS_IOCTL_ALS_GET_CLEAR_DATA		8	// clr_data
#define APDS_IOCTL_ALS_GET_ALS_DATA			9	// als_data
#define APDS_IOCTL_ALS_GET_CAL_FACTOR		10	// als calibration factor

#define APDS_DISABLE_PS						0
#define APDS_ENABLE_PS					1

#define APDS_DISABLE_ALS					0
#define APDS_ENABLE_ALS					1

#define APDS_ALS_POLL_SLOW					0	// 2 Hz (500ms)
#define APDS_ALS_POLL_MEDIUM				1	// 10 Hz (100ms)
#define APDS_ALS_POLL_FAST					2	// 40 Hz (25ms)

/*
 * Defines
 */

/* Register Addresses define */
#define APDS9921_DD_MAIN_CTRL_ADDR			0x00
#define APDS9921_DD_PRX_LED_ADDR			0x01
#define APDS9921_DD_PRX_PULSES_ADDR			0x02
#define APDS9921_DD_PRX_MEAS_RATE_ADDR		0x03
#define APDS9921_DD_ALS_MEAS_RATE_ADDR		0x04
#define APDS9921_DD_ALS_GAIN_ADDR			0x05
#define APDS9921_DD_PART_ID_ADDR			0x06
#define APDS9921_DD_MAIN_STATUS_ADDR		0x07
#define APDS9921_DD_PRX_DATA_ADDR			0x08
#define APDS9921_DD_PRX_DATA_0_ADDR			0x08
#define APDS9921_DD_PRX_DATA_1_ADDR			0x09
#define APDS9921_DD_CLEAR_DATA_ADDR			0x0A
#define APDS9921_DD_CLEAR_DATA_0_ADDR		0x0A
#define APDS9921_DD_CLEAR_DATA_1_ADDR		0x0B
#define APDS9921_DD_CLEAR_DATA_2_ADDR		0x0C
#define APDS9921_DD_ALS_DATA_ADDR			0x0D
#define APDS9921_DD_ALS_DATA_0_ADDR			0x0D
#define APDS9921_DD_ALS_DATA_1_ADDR			0x0E
#define APDS9921_DD_ALS_DATA_2_ADDR			0x0F
#define APDS9921_DD_COMP_DATA_ADDR			0x16
#define APDS9921_DD_COMP_DATA_0_ADDR		0x16
#define APDS9921_DD_COMP_DATA_1_ADDR		0x17
#define APDS9921_DD_COMP_DATA_2_ADDR		0x18
#define APDS9921_DD_INT_CFG_ADDR			0x19
#define APDS9921_DD_INT_PERSISTENCE_ADDR	0x1A
#define APDS9921_DD_PRX_THRES_UP_ADDR		0x1B
#define APDS9921_DD_PRX_THRES_UP_0_ADDR		0x1B
#define APDS9921_DD_PRX_THRES_UP_1_ADDR		0x1C
#define APDS9921_DD_PRX_THRES_LOW_ADDR		0x1D
#define APDS9921_DD_PRX_THRES_LOW_0_ADDR	0x1D
#define APDS9921_DD_PRX_THRES_LOW_1_ADDR	0x1E
#define APDS9921_DD_PRX_CAN_ADDR			0x1F
#define APDS9921_DD_PRX_CAN_0_ADDR			0x1F
#define APDS9921_DD_PRX_CAN_1_ADDR			0x20
#define	APDS9921_DD_ALS_THRES_UP_ADDR		0x21
#define	APDS9921_DD_ALS_THRES_UP_0_ADDR		0x21
#define	APDS9921_DD_ALS_THRES_UP_1_ADDR		0x22
#define	APDS9921_DD_ALS_THRES_UP_2_ADDR		0x23
#define	APDS9921_DD_ALS_THRES_LOW_ADDR		0x24
#define	APDS9921_DD_ALS_THRES_LOW_0_ADDR	0x24
#define	APDS9921_DD_ALS_THRES_LOW_1_ADDR	0x25
#define	APDS9921_DD_ALS_THRES_LOW_2_ADDR	0x26
#define	APDS9921_DD_ALS_THRES_VAR_ADDR		0x27
#define	APDS9921_DD_DEVICE_CONFIG_ADDR		0x2F

/* Register Value define : MAIN_CTRL */
#define APDS9921_DD_PRX_EN					0x01
#define APDS9921_DD_ALS_EN					0x02
#define APDS9921_DD_SW_RESET				0x10

/* Register Value define : PS_LED */
#define APDS9921_DD_LED_CURRENT_2_5_MA		0x00  /* 2.5 mA */
#define APDS9921_DD_LED_CURRENT_5_MA		0x01  /* 5 mA */
#define APDS9921_DD_LED_CURRENT_10_MA		0x02  /* 10 mA */
#define APDS9921_DD_LED_CURRENT_25_MA		0x03  /* 25 mA */
#define APDS9921_DD_LED_CURRENT_50_MA		0x04  /* 50 mA */
#define APDS9921_DD_LED_CURRENT_75_MA		0x05  /* 75 mA */
#define APDS9921_DD_LED_CURRENT_100_MA		0x06  /* 100 mA */
#define APDS9921_DD_LED_CURRENT_125_MA		0x07  /* 125 mA */

#define APDS9921_DD_LED_CURRENT_PEAK_ON		0x08

#define APDS9921_DD_LED_FREQ_60_KHZ			0x30  /* LED Pulse frequency = 60KHz */
#define APDS9921_DD_LED_FREQ_70_KHZ			0x40  /* LED Pulse frequency = 70KHz */
#define APDS9921_DD_LED_FREQ_80_KHZ			0x50  /* LED Pulse frequency = 80KHz */
#define APDS9921_DD_LED_FREQ_90_KHZ			0x60  /* LED Pulse frequency = 90KHz */
#define APDS9921_DD_LED_FREQ_100_KHZ		0x70  /* LED Pulse frequency = 100KHz */

/* Register Value define : PS_MEAS_RATE */
#define APDS9921_DD_PRX_MEAS_RATE_6_25_MS	0x01  /* PS Measurement rate = 6.25 ms */
#define APDS9921_DD_PRX_MEAS_RATE_12_5_MS	0x02  /* PS Measurement rate = 12.5 ms */
#define APDS9921_DD_PRX_MEAS_RATE_25_MS		0x03  /* PS Measurement rate = 25 ms */
#define APDS9921_DD_PRX_MEAS_RATE_50_MS		0x04  /* PS Measurement rate = 50 ms */
#define APDS9921_DD_PRX_MEAS_RATE_100_MS	0x05  /* PS Measurement rate = 100 ms */
#define APDS9921_DD_PRX_MEAS_RATE_200_MS	0x06  /* PS Measurement rate = 200 ms */
#define APDS9921_DD_PRX_MEAS_RATE_400_MS	0x07  /* PS Measurement rate = 400 ms */

#define APDS9921_DD_PRX_MEAS_RES_8_BIT		0x00  /* PS resolution 8 bit (full range : 0 ~ 1023) */
#define APDS9921_DD_PRX_MEAS_RES_9_BIT		0x08  /* PS resolution 9 bit (full range : 0 ~ 2047) */
#define APDS9921_DD_PRX_MEAS_RES_10_BIT		0x10  /* PS resolution 10 bit (full range : 0 ~ 3071) */
#define APDS9921_DD_PRX_MEAS_RES_11_BIT		0x18  /* PS resolution 11 bit (full range : 0 ~ 4095) */

/* Register Value define : ALS_MEAS_RATE */
#define APDS9921_DD_ALS_MEAS_RATE_25_MS		0x00  /* ALS Measurement rate = 25 ms */
#define APDS9921_DD_ALS_MEAS_RATE_50_MS		0x01  /* ALS Measurement rate = 50 ms */
#define APDS9921_DD_ALS_MEAS_RATE_100_MS	0x02  /* ALS Measurement rate = 100 ms */
#define APDS9921_DD_ALS_MEAS_RATE_200_MS	0x03  /* ALS Measurement rate = 200 ms */
#define APDS9921_DD_ALS_MEAS_RATE_500_MS	0x04  /* ALS Measurement rate = 500 ms */
#define APDS9921_DD_ALS_MEAS_RATE_1000_MS	0x05  /* ALS Measurement rate = 1000 ms */
#define APDS9921_DD_ALS_MEAS_RATE_2000_MS	0x06  /* ALS Measurement rate = 2000 ms */

#define APDS9921_DD_ALS_MEAS_RES_20_BIT		0x00  /* ALS resolution 20 bit (full range : 0 ~ 1048575) [ADC conversion time = 400ms] */
#define APDS9921_DD_ALS_MEAS_RES_19_BIT		0x10  /* ALS resolution 19 bit (full range : 0 ~ 524287) [ADC conversion time = 200ms]  */
#define APDS9921_DD_ALS_MEAS_RES_18_BIT		0x20  /* ALS resolution 18 bit (full range : 0 ~ 262143) [ADC conversion time = 100ms]  */
#define APDS9921_DD_ALS_MEAS_RES_17_BIT		0x30  /* ALS resolution 17 bit (full range : 0 ~ 131071) [ADC conversion time = 50ms]  */
#define APDS9921_DD_ALS_MEAS_RES_16_BIT		0x40  /* ALS resolution 16 bit (full range : 0 ~ 65535) [ADC conversion time = 25ms]  */

/* Register Value define : ALS_GAIN */
#define APDS9921_DD_ALS_GAIN_1				0x00  /* ALS Gain 1 */
#define APDS9921_DD_ALS_GAIN_3				0x01  /* ALS Gain 3 */
#define APDS9921_DD_ALS_GAIN_6				0x02  /* ALS Gain 6 */
#define APDS9921_DD_ALS_GAIN_9				0x03  /* ALS Gain 9 */
#define APDS9921_DD_ALS_GAIN_18				0x04  /* ALS Gain 18 */

/* Register Value define : MAIN_STATUS */
#define APDS9921_DD_PRX_DATA_STATUS			0x01  /* 1: New data, not read yet (cleared after read) */
#define APDS9921_DD_PRX_INT_STATUS			0x02  /* 1: Interrupt condition fulfilled (cleared after read) */
#define APDS9921_DD_PRX_LOGICAL_STATUS		0x04  /* 1: object is close */
#define APDS9921_DD_ALS_DATA_STATUS			0x08  /* 1: New data, not read yet (cleared after read) */
#define APDS9921_DD_ALS_INT_STATUS			0x10  /* 1: Interrupt condition fulfilled (cleared after read) */
#define APDS9921_DD_POWER_ON_STATUS			0x20  /* 1: Power on cycle */

/* Register Value define : INT_CFG */
#define APDS9921_DD_PRX_INT_EN				0x01  /* 1: PS Interrupt enabled */
#define APDS9921_DD_PRX_LOGIC_MODE			0x02  /* 1: PS Logic Output Mode: INT pad is updated after every measurement and maintains output state between measurements */
#define APDS9921_DD_ALS_INT_EN				0x04  /* 1: ALS Interrupt enabled */
#define APDS9921_DD_ALS_VAR_MODE			0x08  /* 1: ALS variation interrupt mode */
#define APDS9921_DD_ALS_INT_SEL_ALS			0x10  /* ALS channel selected for interrupt */
#define APDS9921_DD_ALS_INT_SEL_CLEAR		0x00  /* Clear channel selected for interrupt */

/* Register Value define : INT_PERSISTENCE */
#define APDS9921_DD_PRX_PERS_1				0x00  /* Every PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_2				0x01  /* 2 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_3				0x02  /* 3 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_4				0x03  /* 4 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_5				0x04  /* 5 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_6				0x05  /* 6 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_7				0x06  /* 7 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_8				0x07  /* 8 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_9				0x08  /* 9 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_10				0x09  /* 10 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_11				0x0A  /* 11 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_12				0x0B  /* 12 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_13				0x0C  /* 13 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_14				0x0D  /* 14 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_15				0x0E  /* 15 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_16				0x0F  /* 16 consecutive PS value out of threshold range */

#define APDS9921_DD_ALS_PERS_1				0x00  /* Every ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_2				0x10  /* 2 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_3				0x20  /* 3 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_4				0x30  /* 4 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_5				0x40  /* 5 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_6				0x50  /* 6 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_7				0x60  /* 7 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_8				0x70  /* 8 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_9				0x80  /* 9 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_10				0x90  /* 10 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_11				0xA0  /* 11 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_12				0xB0  /* 12 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_13				0xC0  /* 13 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_14				0xD0  /* 14 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_15				0xE0  /* 15 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_16				0xF0  /* 16 consecutive ALS value out of threshold range */

/* Register Value define : ALS_THRES_VAR */
#define APDS9921_DD_ALS_VAR_8_COUNT			0x00  /* ALS result varies by 8 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_16_COUNT		0x01  /* ALS result varies by 16 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_32_COUNT		0x02  /* ALS result varies by 32 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_64_COUNT		0x03  /* ALS result varies by 64 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_128_COUNT		0x04  /* ALS result varies by 128 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_256_COUNT		0x05  /* ALS result varies by 256 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_512_COUNT		0x06  /* ALS result varies by 512 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_1024_COUNT		0x07  /* ALS result varies by 1024 counts compared to previous result */

typedef enum
{
	APDS9921_DD_ALS_RES_16BIT = 0,  /* 25ms integration time */
	APDS9921_DD_ALS_RES_17BIT = 1,  /* 50ms integration time */
	APDS9921_DD_ALS_RES_18BIT = 2,  /* 100ms integration time */
	APDS9921_DD_ALS_RES_19BIT = 3,  /* 200ms integration time */
	APDS9921_DD_ALS_RES_20BIT = 4   /* 400ms integration time */
} apds9921_dd_als_res_e;

typedef enum
{
	APDS9921_DD_ALS_GAIN_1X = 0,    /* 1x ALS GAIN */
	APDS9921_DD_ALS_GAIN_3X = 1,    /* 3x ALS GAIN */
	APDS9921_DD_ALS_GAIN_6X = 2,    /* 6x ALS GAIN */
	APDS9921_DD_ALS_GAIN_9X = 3,    /* 9x ALS GAIN */
	APDS9921_DD_ALS_GAIN_18X = 4    /* 18x ALS GAIN */
} apds9921_dd_als_gain_e;

typedef enum
{
	APDS9921_DD_PRX_RES_8BIT = 0,
	APDS9921_DD_PRX_RES_9BIT = 1,
	APDS9921_DD_PRX_RES_10BIT = 2,
	APDS9921_DD_PRX_RES_11BIT = 3
} apds9921_dd_prx_res_e;

#define APDS9921_DD_LUX_FACTOR					30U

#define APDS9921_DD_ALS_DEFAULT_RES				APDS9921_DD_ALS_MEAS_RES_18_BIT
#define APDS9921_DD_ALS_DEFAULT_MEAS_RATE		APDS9921_DD_ALS_MEAS_RATE_100_MS
#define APDS9921_DD_ALS_DEFAULT_GAIN			APDS9921_DD_ALS_GAIN_18

#define	APDS9921_DD_PRX_DEFAULT_PULSE			32	// drop to 16 if crosstalk is too high
#define APDS9921_DD_PRX_DEFAULT_LED_CURRENT		APDS9921_DD_LED_CURRENT_125_MA
#define APDS9921_DD_PRX_DEFAULT_LED_FREQ		APDS9921_DD_LED_FREQ_100_KHZ
#define APDS9921_DD_PRX_DEFAULT_RES				APDS9921_DD_PRX_MEAS_RES_10_BIT
#define APDS9921_DD_PRX_DEFAULT_MEAS_RATE		APDS9921_DD_PRX_MEAS_RATE_50_MS

#define APDS9921_DD_ALS_MEAS_RATE_SET           APDS9921_DD_ALS_MEAS_RATE_100_MS

#define APDS9921_ALSPS_DYNAMIC_THRESHOLD

static int ps_min = 850;
static int ps_adjust_max = 850;
static int dirty_adjust_low_thd = 300, dirty_adjust_high_thd = 350;
static int ps_thd_low_highlight = 700, ps_thd_high_highlight = 750;

static struct delayed_work sample_ps_work;
static DECLARE_WAIT_QUEUE_HEAD(enable_wq);

#define APDS9921_VDD_MIN_UV  2000000
#define APDS9921_VDD_MAX_UV  3300000
#define APDS9921_VIO_MIN_UV  1750000
#define APDS9921_VIO_MAX_UV  1950000

static int apds9921_init_client(struct i2c_client *client);
static int apds9921_enable_ps_sensor(struct i2c_client *client, int val);
static int apds9921_enable_als_sensor(struct i2c_client *client, int val);

/*
 * Structs
 */

struct apds9921_data {
	struct i2c_client *client;

	//struct mutex update_lock;
	struct delayed_work	dwork;		/* for interrupt */
	struct delayed_work	als_dwork;	/* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	struct wake_lock ps_wakelock;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);

	bool power_on_flag;
	struct regulator *vdd;
	struct regulator *vio;

	int irq;
	int als_suspended;
	int ps_suspended;
	unsigned int main_ctrl_suspended_value;	/* suspend_resume usage */

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;
	unsigned int poffset;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;
	unsigned int als_just_enable_flag;
	unsigned int als_state_suspend_resume;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold;	/* always lower than ps_threshold */
	unsigned int prev_ps_detection;
	unsigned int ps_detection;				/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;					/* to store PS data */
	unsigned int ps_overflow;				/* to store PS overflow flag */
	unsigned int ps_poll_delay;				/* needed for proximity sensor polling : ms */
	unsigned int ps_offset;					/* needed if crosstalk under cover glass is big */

	int i2c_err_time;
	int esd_handle_flag;

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */

	unsigned int clr_data;			/* to store CLEAR data */
	unsigned int als_data;			/* to store ALS data */
	int als_prev_lux;				/* to store previous lux value */
	int als_cal_factor;				/* to store the ALS calibration factor */
	int als_cal_loop;				/* loop counter for ALS calibration */

	unsigned int als_gain;			/* needed for Lux calculation */
	unsigned int als_poll_delay;	/* needed for light sensor polling : ms */
	unsigned int als_res_index;		/* storage for als integratiion time */
	unsigned int als_gain_index;	/* storage for als GAIN */
	unsigned int als_reduce;		/* flag indicate ALS 6x reduction */
};

static struct apds9921_data *pdev_data = NULL;

static struct sensors_classdev sensors_light_cdev = {
	.name = "apds9921-light",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "60000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "apds9921-proximity",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 3,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	#ifdef CONFIG_MACH_OPPO
	//xiaohua.tian@EXP.Driver add flags for proximity, 2016-03-01
	.flags = 3,
	#endif
};

#define SHOW_LOG    1

static int printk_log_level = 0;

#define printk_x(level, ...) \
	do { \
		if (printk_log_level >= (level)) \
		printk(__VA_ARGS__); \
	} while (0)


/*
 * Global data
 */
static struct i2c_client *apds9921_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *apds_workqueue;

#ifdef APDS9921_ALSPS_DYNAMIC_THRESHOLD
static void sample_work_func(struct work_struct *work);
#endif

static unsigned short apds9921_als_meas_rate_tb[] = {25, 50, 100, 200, 400};
static unsigned int apds9921_als_res_tb[] = { 65535, 131071, 262143, 524287, 1048575 };
static unsigned char apds9921_als_gain_tb[] = { 1, 3, 6, 9, 18 };
static unsigned char apds9921_als_gain_bit_tb[] = { 0x00, 0x01, 0x02, 0x03, 0x04 };

/*
 * Management functions
 */

static int apds9921_dd_set_main_ctrl(struct i2c_client *client, int main_ctrl)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR, main_ctrl);
}

static int apds9921_dd_set_prx_meas_rate(struct i2c_client *client, int prx_meas)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_PRX_MEAS_RATE_ADDR, prx_meas);
}

static int apds9921_dd_set_als_meas_rate(struct i2c_client *client, int als_meas)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_ALS_MEAS_RATE_ADDR, als_meas);
}

static int apds9921_dd_set_als_gain(struct i2c_client *client, int als_gain)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_ALS_GAIN_ADDR, als_gain);
}

static int apds9921_dd_set_pers(struct i2c_client *client, int pers)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_INT_PERSISTENCE_ADDR, pers);
}

static int apds9921_dd_set_prx_led(struct i2c_client *client, int prx_led)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_PRX_LED_ADDR, prx_led);
}

static int apds9921_dd_set_prx_pulses(struct i2c_client *client, int prx_pulses)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_PRX_PULSES_ADDR, prx_pulses);
}

static int apds9921_dd_set_prx_can(struct i2c_client *client, int prx_can)
{
	return i2c_smbus_write_word_data(client, APDS9921_DD_PRX_CAN_ADDR, prx_can);
}

static int apds9921_dd_set_int_cfg(struct i2c_client *client, int int_cfg)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_INT_CFG_ADDR, int_cfg);
}

static void apds9921_dd_set_prx_thresh(struct i2c_client *client, int thres_low, int thres_up)
{
	struct apds9921_data *data = i2c_get_clientdata(client);

	i2c_smbus_write_word_data(client, APDS9921_DD_PRX_THRES_UP_ADDR, thres_up);
	i2c_smbus_write_word_data(client, APDS9921_DD_PRX_THRES_LOW_ADDR, thres_low);

	data->pilt = thres_low;
	data->piht = thres_up;
}

static inline void apds9921_report_abs_ts(struct input_dev *dev,
					  int code, int value)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	input_report_abs(dev, code, value);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ts.tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ts.tv_nsec);
	input_sync(dev);
}

static int Lux_original(struct i2c_client *client, int als_data)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	unsigned int luxValue=0;

	if (is_project(OPPO_15018))
		als_data = als_data * 5;
	else if (is_project(OPPO_15022))
		als_data = als_data * 3;
	else if (is_project(OPPO_15109))
		als_data = als_data * 3;

	luxValue = ((als_data*APDS9921_DD_LUX_FACTOR*1000))/((apds9921_als_meas_rate_tb[data->als_res_index])*apds9921_als_gain_tb[data->als_gain_index]);

	luxValue = (luxValue*data->als_cal_factor)/100/1000;

	return (int)luxValue;
}

static int LuxCalculation(struct i2c_client *client, int clr_data, int als_data)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	unsigned long long scaled_lux_value, divider;
	unsigned int luxValue=0;
	int status1, status2;

	status1 = i2c_smbus_read_byte_data(client, APDS9921_DD_ALS_GAIN_ADDR);

	status2 = i2c_smbus_read_byte_data(client, APDS9921_DD_ALS_MEAS_RATE_ADDR);

	//printk(KERN_ERR"als_res_index = %d, als_gain_index = %d, gain = %x, rate = %x\n", data->als_res_index, data->als_gain_index, status1, status2);

	if (is_project(OPPO_15018))
		als_data = als_data * 5;
	else if (is_project(OPPO_15022))
		als_data = als_data * 3;
	else if (is_project(OPPO_15109))
		als_data = als_data * 3;

	// Apply als calibration gain
	als_data = als_data * pdev_data->als_gain / 1000;

	scaled_lux_value = (unsigned long long) als_data * APDS9921_DD_LUX_FACTOR * 1000U * data->als_cal_factor;
	divider = (unsigned long long) (apds9921_als_meas_rate_tb[data->als_res_index]*apds9921_als_gain_tb[data->als_gain_index] * 100 * 1000);
	luxValue = (unsigned int) (scaled_lux_value / divider);
	//printk(KERN_ERR"%s,  after %u\n", __func__, luxValue);

	return (int)luxValue;
}

static void apds9921_esd_handle(struct i2c_client *client, int esd_type)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int als_state =0, ps_state = 0;

	als_state = data->enable_als_sensor;
	ps_state = data->enable_ps_sensor;

	if(esd_type == 1)    // intflag err
	{
		data->esd_handle_flag = 1;
		data->i2c_err_time =0;

		printk(KERN_ERR"%s , esd err, reinit device.\n",__func__);

		if(data->enable_als_sensor > 0)
		{
			apds9921_enable_als_sensor(client, 0);
		}
		if(data->enable_ps_sensor > 0)
		{
			apds9921_enable_ps_sensor(client, 0);
		}

		msleep(50);

		if(als_state > 0)
		{
			apds9921_enable_als_sensor(client, 1);
		}
		if(ps_state > 0)
		{
			apds9921_enable_ps_sensor(client, 1);
		}

		data->esd_handle_flag = 0;
	}
}

static int apds9921_check_intr(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int main_status = 0;

	main_status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);

	printk(KERN_ERR"%s , main_status = 0x%x\n",__func__, main_status);

	if((main_status == 0)||(main_status == 0x20))
		apds9921_esd_handle(client,1);  // 1 means : intFlag err

	if((APDS9921_DD_PRX_LOGICAL_STATUS & main_status) == 0)
		data->ps_detection = 0; // far
	else
		data->ps_detection = 1; // near

	printk(KERN_ERR"%s , ps_detection = %d\n",__func__, data->ps_detection);

	return main_status;
}

static void apds9921_change_ps_threshold(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);

	data->ps_data =	i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);
	data->ps_overflow = data->ps_data >> 11;
	data->ps_data = data->ps_data & 0x7FF;

	//printk(KERN_ERR"ps_data=%d overflow=%d [thres_up = %d]\n", data->ps_data, data->ps_overflow, data->ps_threshold);

	if(data->prev_ps_detection != data->ps_detection)
	{
		data->prev_ps_detection = data->ps_detection;

		apds9921_report_abs_ts(data->input_dev_ps, ABS_DISTANCE, data->ps_detection ? 0 : 1);
		wake_lock_timeout(&data->ps_wakelock, 2*HZ);

		printk(KERN_ERR"%s:ps_data = %d, th_l = %d, th_h = %d, ps_interrutp_state is : %d %s\n",
				__func__,data->ps_data, data->pilt, data->piht,data->ps_detection, data->ps_detection?"near":"far");
	}
}

static void apds9921_change_als_threshold(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int clr_data, als_data;
	int luxValue=0;
	unsigned char change_again=0;

	clr_data = i2c_smbus_read_word_data(client, APDS9921_DD_CLEAR_DATA_ADDR);
	als_data = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);

	luxValue = LuxCalculation(client, clr_data, als_data);

	//printk(KERN_ERR"lux=%d clr_data=%d als_data=%d again=%d als_res=%d\n", luxValue, clr_data, als_data, apds9921_als_gain_tb[data->als_gain_index], data->als_res_index);

	data->als_data = als_data;

	data->als_threshold_l = (apds9921_als_res_tb[data->als_res_index] * APDS9921_ALS_THRESHOLD_HSYTERESIS ) /100;
	data->als_threshold_h = (apds9921_als_res_tb[data->als_res_index] * (100-APDS9921_ALS_THRESHOLD_HSYTERESIS) ) /100;

	if (data->als_data >= data->als_threshold_h) {
		// lower AGAIN if possible
		if (data->als_gain_index != APDS9921_DD_ALS_GAIN_1X) {
			data->als_gain_index--;
			change_again = 1;
		}
	}
	else if (data->als_data < data->als_threshold_l) {
		// increase AGAIN if possible
		if (data->als_gain_index != APDS9921_DD_ALS_GAIN_18X) {
			data->als_gain_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		apds9921_dd_set_als_gain(client, apds9921_als_gain_bit_tb[data->als_gain_index]);

		return;
	}

	// report to HAL
	//luxValue = (luxValue>30000) ? 30000 : luxValue;
	data->als_prev_lux = luxValue;

	apds9921_report_abs_ts(data->input_dev_als, ABS_MISC, luxValue); // report the lux level

	printk_x(SHOW_LOG,"apds9921 report to HAL, Lux = %d\n", luxValue);
}

static void apds9921_reschedule_work(struct apds9921_data *data,unsigned long delay)
{
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->dwork);
	queue_delayed_work(apds_workqueue, &data->dwork, delay);
}

/* ALS polling routine */
static void apds9921_als_polling_work_handler(struct work_struct *work)
{
	struct apds9921_data *data = container_of(work, struct apds9921_data, als_dwork.work);
	struct i2c_client *client=data->client;

	if(data->enable_als_sensor == 0)
		return ;

	// the previous als data is not stable after als enable, so we get rid of the first als data .
	apds9921_change_als_threshold(client);

	if(data->als_just_enable_flag == 1)
    {
        apds9921_dd_set_als_meas_rate(client, APDS9921_DD_ALS_DEFAULT_RES|APDS9921_DD_ALS_DEFAULT_MEAS_RATE);
	data->als_just_enable_flag = 0;
    }

	queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));	// restart timer
}

/* Interrupt Service Routine */
static void apds9921_work_handler(struct work_struct *work)
{
	struct apds9921_data *data = container_of(work, struct apds9921_data, dwork.work);
	struct i2c_client *client=data->client;
	int status;

	if(data->enable_ps_sensor > 0)
	{
		status = apds9921_check_intr(client);

		if (status & APDS9921_DD_PRX_INT_STATUS)
		{
			apds9921_change_ps_threshold(client);
		}
	}

	enable_irq(data->irq);
}

/* assume this is ISR */
static irqreturn_t apds9921_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds9921_data *data = i2c_get_clientdata(client);

	disable_irq_nosync(data->irq);
	apds9921_reschedule_work(data, 0);

	return IRQ_HANDLED;
}

static int apds9921_enable_als_sensor(struct i2c_client *client, int val)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int rc;

	printk("%s: ( %d)\n", __func__, val);

	if ((val != APDS_DISABLE_ALS) && (val != APDS_ENABLE_ALS)) {
		printk("%s: enable als sensor=%d\n", __func__, val);
		return -1;
	}

	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0))
	{
		/* Power on and initalize the device */
		if (data->power_on)
			data->power_on(true);

		rc = apds9921_init_client(client);
		if (rc) {
			dev_err(&client->dev, "Failed to init apds993x\n");
			return rc;
		}
	}

	if ((val == APDS_ENABLE_ALS) ) {
		// turn on light  sensor
		if (data->enable_als_sensor==APDS_DISABLE_ALS) {

			data->enable_als_sensor = val;
			if (data->enable_als_sensor == APDS_ENABLE_ALS) {

				printk("%s: enable_als_sensor\n", __func__);

				data->als_just_enable_flag = 1;

                apds9921_dd_set_als_meas_rate(client, APDS9921_DD_ALS_DEFAULT_RES|APDS9921_DD_ALS_MEAS_RATE_50_MS);

				if(data->enable_ps_sensor == 0)     // if ps enable, als also enable.
					apds9921_dd_set_main_ctrl(client, APDS9921_DD_ALS_EN);

				/*
				 * If work is already scheduled then subsequent schedules will not
				 * change the scheduled time that's why we have to cancel it first.
				 */
				cancel_delayed_work(&data->als_dwork);
				flush_delayed_work(&data->als_dwork);

				queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
			}
		}
	}
	else {
		//turn off light sensor
		data->enable_als_sensor = APDS_DISABLE_ALS;

		if(data->enable_ps_sensor == 0)     // if ps disable, als disable.
			apds9921_dd_set_main_ctrl(client, 0);

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->als_dwork);
		flush_delayed_work(&data->als_dwork);
	}

	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) && (data->power_on))
	{
		data->power_on(false);
	}

	return 0;
}

static int apds9921_set_als_poll_delay(struct i2c_client *client, unsigned int val)
{
	struct apds9921_data *data = i2c_get_clientdata(client);

	printk("%s : %d\n", __func__, val);

	data->als_poll_delay = val;

	return 0;
}

static int apds9921_ps_calibration(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int i, rc= 0;
	int ps_data=0;
	int ps_meas_rate;

	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0))
	{
		/* Power on and initalize the device */
		if (data->power_on)
			data->power_on(true);

		rc = apds9921_init_client(client);
		if (rc) {
			dev_err(&client->dev, "Failed to init apds993x\n");
			return -1;
		}
	}

	apds9921_dd_set_prx_can(client, 0);
	msleep(10);

	ps_meas_rate = i2c_smbus_read_byte_data(client, APDS9921_DD_PRX_MEAS_RATE_ADDR);
	if (ps_meas_rate < 0)
		return -4;

	apds9921_dd_set_prx_meas_rate(client, 0x40|APDS9921_DD_PRX_DEFAULT_RES|APDS9921_DD_PRX_DEFAULT_MEAS_RATE);

	for (i=0; i<APDS9921_PS_CAL_LOOP; i++) {
		mdelay(70); // must be greater than prx meas rate
		ps_data += i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);
	}

	apds9921_dd_set_prx_meas_rate(client, ps_meas_rate);

	ps_data = ps_data/i;
	printk(KERN_ERR"APDS9921_PS_CAL pdata = %d\n", ps_data);

	if ((ps_data <= APDS9921_PS_CAL_CROSSTALK_HIGH) &&  // don't adjust if within
			(ps_data >= APDS9921_PS_CAL_CROSSTALK_LOW))
	{
		printk(KERN_ERR"ps_data = %d, no need to adjust.\n", ps_data);

		data->ps_offset = 0;

		return 0;
	}
	else if (ps_data > APDS9921_PS_CAL_CROSSTALK_HIGH)
	{
		if (ps_data <= 900)
		{
			data->ps_offset = ps_data - APDS9921_PS_CAL_CROSSTALK_HIGH;
			apds9921_dd_set_prx_can(client, data->ps_offset);

			printk(KERN_ERR"APDS9921_PS_CAL ps_offset = %d\n", data->ps_offset);
		}
		else
		{
			printk(KERN_ERR"ps_data = %d, something over p_sensor.\n", data->ps_data);

			return -1;
		}
	}

	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) && (data->power_on))
	{
		apds9921_dd_set_main_ctrl(client, 0);
		data->power_on(false);
	}

	return data->ps_offset;
}

static int apds9921_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int rc;
	int init_client_time = 3;

	printk(KERN_ERR"%s: ( %d)\n", __func__, val);

	if ((val != APDS_DISABLE_PS) && (val != APDS_ENABLE_PS)) {
		printk("%s:invalid value=%d\n", __func__, val);
		return -1;
	}

	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0))
	{
		/* Power on and initalize the device */
		if (data->power_on)
			data->power_on(true);

		for(init_client_time = 0; init_client_time < 3; init_client_time ++)
		{
			rc = apds9921_init_client(client);
			if (rc)
			{
				dev_err(&client->dev, "Failed to init apds9921, time = %d\n", init_client_time);
				msleep(50);
			}
			else
			{
				break;
			}
		}
	}

	if(val == APDS_ENABLE_PS) {
		//turn on p sensor
		if (data->enable_ps_sensor==APDS_DISABLE_PS) {

			data->enable_ps_sensor = val;

			apds9921_dd_set_main_ctrl(client, APDS9921_DD_PRX_EN | APDS9921_DD_ALS_EN);     // when open ps, we need open als too.

			apds9921_dd_set_int_cfg(client, APDS9921_DD_PRX_INT_EN);

			if (ps_min != 0 && ps_min + dirty_adjust_high_thd < ps_adjust_max)
			{
				apds9921_dd_set_prx_thresh(client, ps_min + dirty_adjust_low_thd, ps_min + dirty_adjust_high_thd);
			}
			else
			{
				apds9921_dd_set_prx_thresh(client, ps_thd_low_highlight, ps_thd_high_highlight);
			}

			msleep(100); // must be more than APDS9921_DD_PRX_DEFAULT_MEAS_RATE

			data->ps_data = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR) & 0x7FF;

			if (data->ps_data >= data->piht)
				data->ps_detection = 1;
			else
				data->ps_detection = 0;

			printk(KERN_ERR"%s:ps_data = %d, th_l = %d, th_h = %d, ps_original_state is : %d %s\n",
					__func__,data->ps_data, data->pilt, data->piht,data->ps_detection, data->ps_detection?"near":"far");

			apds9921_report_abs_ts(data->input_dev_ps, ABS_DISTANCE, data->ps_detection ? 0 : 1);
			wake_lock_timeout(&data->ps_wakelock, 2*HZ);

			data->prev_ps_detection = data->ps_detection;

#ifdef APDS9921_ALSPS_DYNAMIC_THRESHOLD
			wake_up(&enable_wq);
#endif

		}
	}
	else {

		data->enable_ps_sensor = APDS_DISABLE_PS;

		apds9921_dd_set_int_cfg(client, 0x00);

		if(data->enable_als_sensor == 1)
			apds9921_dd_set_main_ctrl(client, APDS9921_DD_ALS_EN);
		else
			apds9921_dd_set_main_ctrl(client, 0);
	}

	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) && (data->power_on))
	{
		data->power_on(false);
	}

	return 0;
}

/*
 * SysFS support
 */

static ssize_t apds9921_show_clr_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int clr_data;

	clr_data = i2c_smbus_read_word_data(client, APDS9921_DD_CLEAR_DATA_ADDR);

	return sprintf(buf, "%d\n", clr_data);
}

static DEVICE_ATTR(clr_data, S_IRUGO,
		apds9921_show_clr_data, NULL);

static ssize_t apds9921_show_als_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int als_data;

	als_data = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);

	return sprintf(buf, "%d\n", als_data);
}

static DEVICE_ATTR(als_data, S_IRUGO,apds9921_show_als_data, NULL);

static ssize_t apds9921_show_ps_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ps_data;

	ps_data = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR) & 0x7FF;

	return sprintf(buf, "%d\n", ps_data);
}

static DEVICE_ATTR(ps_data, S_IRUGO,
		apds9921_show_ps_data, NULL);

static ssize_t apds9921_show_proximity_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9921_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9921_store_proximity_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	printk("%s: enable ps senosr ( %ld)\n", __func__, val);

	if ((val != APDS_DISABLE_PS) && (val != APDS_ENABLE_PS) ) {
		printk("**%s:store invalid value=%ld\n", __func__, val);
		return count;
	}

	apds9921_enable_ps_sensor(client, val);

	return count;
}

static DEVICE_ATTR(proximity_enable, S_IWUGO | S_IRUGO,
		apds9921_show_proximity_enable, apds9921_store_proximity_enable);

static ssize_t apds9921_show_light_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9921_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds9921_store_light_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	printk("%s: enable als sensor ( %ld)\n", __func__, val);

	if ((val != APDS_DISABLE_ALS) &&
			(val != APDS_ENABLE_ALS)) {

		printk("**%s: store invalid value=%ld\n", __func__, val);
		return count;
	}

	apds9921_enable_als_sensor(client, val);

	return count;
}

static DEVICE_ATTR(light_enable, S_IWUGO | S_IRUGO,
		apds9921_show_light_enable, apds9921_store_light_enable);

static struct attribute *apds9921_attributes[] = {
	&dev_attr_clr_data.attr,
	&dev_attr_als_data.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_proximity_enable.attr,
	&dev_attr_light_enable.attr,
	NULL
};

static const struct attribute_group apds9921_attr_group = {
	.attrs = apds9921_attributes,
};


/*
 * Initialization function
 */

static int apds9921_init_client(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	printk(KERN_ERR"%s.\n", __func__);

#if 0
	{
		int i, i2c_data;

		printk("before\n");

		for (i=0; i<0x28; i++)
		{
			i2c_data = i2c_smbus_read_byte_data(client, i);
			printk("[%x] = %x\n", i, i2c_data);
			mdelay(1);
		}
	}
#endif

	err = apds9921_dd_set_main_ctrl(client, 0);
	if (err < 0)
		return err;

	id = i2c_smbus_read_byte_data(client, APDS9921_DD_PART_ID_ADDR);
	if (id == 0xB1)
	{
		printk(KERN_EMERG"chip APDS-9921.\n");
	}
	else if (id == 0xB3)
	{
		printk(KERN_EMERG"chip APDS-9922.\n");
	}
	else {
		printk(KERN_EMERG"Not chip APDS-9921 or APDS-9922.\n");
		return -EIO;
	}

	/* PS_LED */
	err = apds9921_dd_set_prx_led(client, APDS9921_DD_PRX_DEFAULT_LED_FREQ|APDS9921_DD_PRX_DEFAULT_LED_CURRENT);
	if (err < 0) return err;

	/* PS_PULSES */
	err = apds9921_dd_set_prx_pulses(client, APDS9921_DD_PRX_DEFAULT_PULSE);
	if (err < 0) return err;

	/* PS_MEAS_RATE */
	err = apds9921_dd_set_prx_meas_rate(client, 0x40|APDS9921_DD_PRX_DEFAULT_RES|APDS9921_DD_PRX_DEFAULT_MEAS_RATE);
	if (err < 0) return err;

	/* ALS_MEAS_RATE */
	err = apds9921_dd_set_als_meas_rate(client, APDS9921_DD_ALS_DEFAULT_RES|APDS9921_DD_ALS_DEFAULT_MEAS_RATE);
	if (err < 0) return err;

	/* ALS_GAIN */
	err = apds9921_dd_set_als_gain(client, APDS9921_DD_ALS_DEFAULT_GAIN);
	if (err < 0) return err;

	/* INT_PERSISTENCE */
	err = apds9921_dd_set_pers(client, APDS9921_DD_PRX_PERS_1|APDS9921_DD_ALS_PERS_1);
	if (err < 0) return err;

	/* PS_THRES_UP & PS_THRES_DOWN */
	//	apds9921_dd_set_prx_thresh(client, data->ps_threshold, data->ps_threshold); // init threshold for proximity

	i2c_smbus_write_byte_data(client, APDS9921_DD_ALS_THRES_UP_ADDR, 0);

	/* PS Offset */
	err = apds9921_dd_set_prx_can(client, data->ps_offset);
	if (err < 0) return err;

	//apds9921_check_intr(client);

#if 0
	{
		int i, i2c_data;

		printk("after\n");

		for (i=0; i<0x28; i++)
		{
			i2c_data = i2c_smbus_read_byte_data(client, i);
			printk("[%x] = %x\n", i, i2c_data);
			mdelay(1);
		}
	}
#endif

	return 0;
}

static int apds9921_sensor_regulator_configure(struct apds9921_data *data, bool on)
{
	int rc;

	printk(KERN_ERR"%s, on = %d\n", __func__, on);

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
					APDS9921_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
					APDS9921_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
					"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
					APDS9921_VDD_MIN_UV, APDS9921_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
						"Regulator set failed vdd rc=%d\n",
						rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
					"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
					APDS9921_VIO_MIN_UV, APDS9921_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
						"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, APDS9921_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int sensor_platform_hw_init(void)
{
	struct apds9921_data *data = pdev_data;
	int error;

	error = apds9921_sensor_regulator_configure(data, true);
	if (error < 0) {
		dev_err(&apds9921_i2c_client->dev, "unable to configure regulator\n");
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct apds9921_data *data = pdev_data;

	if (data == NULL)
		return;

	apds9921_sensor_regulator_configure(data, false);
}

static int apds9921_sensor_regulator_power_on(struct apds9921_data *data, bool on)
{
	int rc = 0;

	printk(KERN_ERR"%s, on = %d\n", __func__, on);

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	}

enable_delay:
	msleep(20);
	dev_dbg(&data->client->dev,
			"Sensor regulator power on =%d\n", on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on)
{
	struct apds9921_data *data = pdev_data;
	int err = 0;

	if (data == NULL)
		return -ENODEV;

	if (data->power_on_flag != on) {
		err = apds9921_sensor_regulator_power_on(data, on);
		if (err)
			dev_err(&data->client->dev,
					"Can't configure regulator!\n");
		else
			data->power_on_flag = on;
	}

	return err;
}

static int apds_parse_dt(struct device *dev, struct apds9921_data *data)
{
	struct device_node *np = dev->of_node;

	data->init = sensor_platform_hw_init;
	data->exit = sensor_platform_hw_exit;
	data->power_on = sensor_platform_hw_power_on;

	data->irq = of_get_named_gpio_flags(np, "apds,irq-gpio", 0, NULL);
	if (data->irq < 0)
	{
		dev_err(dev, "Unable to read irq-gpio\n");
		return -1;
	}

	printk("%s, irq = %d\n", __func__, data->irq);

	return 0;
}

#ifdef APDS9921_ALSPS_DYNAMIC_THRESHOLD
static void sample_work_func(struct work_struct *work)
{
	struct apds9921_data *data;
	struct i2c_client *client;
	int i, rc;
	uint16_t ps, als;

	printk("%s ps rboot sample start\n", __func__);

	if (apds9921_i2c_client != NULL)
		client = apds9921_i2c_client;
	else{
		printk(KERN_EMERG"%s:apds9921: sample fail because of global pointer is NULL\n",__func__);
		return;
	}

	data = pdev_data;
	if (data == NULL)
		return;

	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0))
	{
		/* Power on and initalize the device */
		if (data->power_on)
			data->power_on(true);

		rc = apds9921_init_client(client);
		if (rc) {
			dev_err(&client->dev, "Failed to init apds993x\n");
			return ;
		}
	}

	if (data->enable_ps_sensor==0) {
		apds9921_dd_set_main_ctrl(client, 1);
	}

	msleep(10);

	for (i = 0; i < 10; i++){
		als = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);
		if (als < 8000){
			ps = i2c_smbus_read_word_data(client,APDS9921_DD_PRX_DATA_ADDR) & 0x7FF;

			pr_err("%s ps = %d \n", __func__, ps);

			if( (ps != 0) &&((ps_min == 0) || (ps_min > ps)))
				ps_min = ps;
		}
		msleep(80);
	}

	if (ps_min > ps_adjust_max)
		ps_min = 0;

	if (data->enable_ps_sensor==0) {
		apds9921_dd_set_main_ctrl(client, 0);
	}

	if ((data->enable_als_sensor == 0) && (data->enable_ps_sensor == 0) && (data->power_on))
		data->power_on(false);

	pr_err("%s ps_min:%d \n", __func__,  ps_min);
}
#endif

static int apds9921_als_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	struct apds9921_data *data = container_of(sensors_cdev, struct apds9921_data, als_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return apds9921_enable_als_sensor(data->client, enable);
}

static int apds9921_ps_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	struct apds9921_data *data = container_of(sensors_cdev, struct apds9921_data, ps_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return apds9921_enable_ps_sensor(data->client, enable);
}

static int apds9921_als_poll_delay(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	struct apds9921_data *data = container_of(sensors_cdev,struct apds9921_data, als_cdev);
	apds9921_set_als_poll_delay(data->client, delay_msec);
	return 0;
}


static ssize_t apds9921_alsps_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data[2];
	int ret = -EINVAL;
	sscanf(buf, "%x %x", (unsigned int *)&data[0],(unsigned int *)&data[1]);
	switch(data[0])
	{
		case SENSOR_TYPE_LIGHT:
			ret = apds9921_enable_als_sensor(apds9921_i2c_client,(int)data[1]);
			if (ret < 0)
			{
				printk("%s: %s als fail\n",__func__, (data[1] == 1)?"enable":"disable");
			}
			break;

		case SENSOR_TYPE_PROXIMITY:
			ret = apds9921_enable_ps_sensor(apds9921_i2c_client, (int)data[1]);
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
static ssize_t apds9921_alsps_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct apds9921_data *data = i2c_get_clientdata(apds9921_i2c_client);

	return snprintf(buf, PAGE_SIZE, "als:%d  prox:%d\n", data->enable_als_sensor, data->enable_ps_sensor);
}
static struct kobj_attribute enable =
{
	.attr = {"enable", 0664},
	.show = apds9921_alsps_enable_show,
	.store = apds9921_alsps_enable_store,
};
static ssize_t apds9921_prox_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	uint16_t ps_data = 0, als_up = 0;

	static int esd_register_err = 0;

	struct apds9921_data *data = i2c_get_clientdata(apds9921_i2c_client);

	if(pdev_data->enable_ps_sensor == 0)
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);

	ps_data =  i2c_smbus_read_word_data(apds9921_i2c_client, APDS9921_DD_PRX_DATA_ADDR) & 0x7FF;

	als_up = i2c_smbus_read_byte_data(apds9921_i2c_client, APDS9921_DD_ALS_THRES_UP_ADDR);

	printk_x(SHOW_LOG,"%s ps_data = %d, als_up_register = 0x%x, th_l = %d, th_h = %d \n", __func__, ps_data, als_up,data->pilt, data->piht);

	if(als_up == 0xff)
		esd_register_err ++;
	else
		esd_register_err = 0;

	if(esd_register_err == 3)      // esd err
	{
		esd_register_err = 0;

		printk("%s, esd register err\n", __func__);

		apds9921_esd_handle(apds9921_i2c_client,1);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", ps_data);
}
static struct kobj_attribute prox_raw =
{
	.attr = {"prox_raw", 0444},
	.show = apds9921_prox_raw_show,
};


static ssize_t apds9921_als_calibration_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pdev_data->als_gain);
}
static ssize_t apds9921_als_calibration_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;

	if (sscanf(buf, "%d", (unsigned int *)&data) == 1)
	{
		pdev_data->als_gain = data;

		printk(KERN_ERR"%s the calibration als_gain is %d\n", __func__, pdev_data->als_gain);
	}
	else
	{
		printk("%s error.\n", __func__);
	}

	return count;
}

static struct kobj_attribute als_calibration =
{
	.attr = {"als_calibration", 0664},
	.show = apds9921_als_calibration_show,
	.store = apds9921_als_calibration_store,
};

static ssize_t apds9921_als_raw_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int als_data = 0, luxValue = 0;

	als_data = i2c_smbus_read_word_data(apds9921_i2c_client, APDS9921_DD_ALS_DATA_ADDR);

	luxValue = Lux_original(apds9921_i2c_client, als_data);

	return snprintf(buf, PAGE_SIZE, "%d\n", luxValue);
}
static struct kobj_attribute als_raw =
{
	.attr = {"als_raw", 0444},
	.show = apds9921_als_raw_show,
};

static ssize_t apds9921_prox_calibration_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ps_offset = 0;

	printk(KERN_ERR"%s\n", __func__);

	ps_offset = apds9921_ps_calibration(apds9921_i2c_client);

	return snprintf(buf, PAGE_SIZE, "%d\n", ps_offset);
}
static ssize_t apds9921_prox_calibration_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data;

	if (sscanf(buf, "%d", (unsigned int *)&data) == 1)
	{
		pdev_data->ps_offset = data;

		printk(KERN_ERR"%s the calibration data is %d\n", __func__, pdev_data->ps_offset);
	}
	else
	{
		printk("%s error.\n", __func__);
	}

	return count;
}
static struct kobj_attribute calibration =
{
	.attr = {"calibration", 0664},
	.show = apds9921_prox_calibration_show,
	.store = apds9921_prox_calibration_store,
};

static ssize_t apds9921_log_control_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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
	.store = apds9921_log_control_store,
};


static ssize_t apds9921_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i = 0, i2c_data = 0;
	ssize_t num_read_chars = 0;

	for (i = 0; i < 0x28; i++)
	{
		i2c_data = i2c_smbus_read_byte_data(apds9921_i2c_client, i);

		printk("reg[0x%2x] = 0x%x\n", i, i2c_data);

		num_read_chars += sprintf(&(buf[num_read_chars]), "reg[0x%x] = 0x%x\n", i, i2c_data);

		msleep(20);
	}

	return num_read_chars;
}

static struct kobj_attribute reg_show =
{
	.attr = {"reg_show", 0444},
	.show = apds9921_reg_show,
};

#ifdef APDS9921_ALSPS_DYNAMIC_THRESHOLD    // Do not modify it as wilful, because it is used for algo.
static ssize_t apds9921_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s", "apds9921");
}
static struct kobj_attribute name =
{
	.attr = {"name", 0444},
	.show = apds9921_name_show,
};
static ssize_t apds9921_high_light_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ch0data = 0;

	if(pdev_data->enable_ps_sensor == 0)
		return snprintf(buf, PAGE_SIZE, "%d\n",  0);

	ch0data = i2c_smbus_read_word_data(apds9921_i2c_client,APDS9921_DD_ALS_DATA_ADDR);

	printk_x(SHOW_LOG,"apds9921_high_light_show : lux = %5d \n", ch0data);

	return snprintf(buf, PAGE_SIZE, "%d\n",  (ch0data < 8000)? 0:1);
}
static struct kobj_attribute is_high_light =
{
	.attr = {"is_high_light", 0444},
	.show = apds9921_high_light_show,
};
static ssize_t apds9921_ps_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct apds9921_data *data = i2c_get_clientdata(apds9921_i2c_client);
	return snprintf(buf, PAGE_SIZE, "%d", data->enable_ps_sensor);
}
static struct kobj_attribute ps_enable =
{
	.attr = {"ps_enable", 0444},
	.show = apds9921_ps_enable_show,
};
static ssize_t apds9921_alsps_ps_thd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 data[2];
	struct apds9921_data *p_data = i2c_get_clientdata(apds9921_i2c_client);

	if (sscanf(buf, "%d %d", (unsigned int *)&data[0],(unsigned int *)&data[1]) == 2)
	{
		//printk(KERN_ERR"algo set --- low_thd:%5d, high_thd:%5d \n", data[0], data[1]);

		if(pdev_data->enable_ps_sensor == 0)
			return count;

		p_data->piht = data[1];
		p_data->pilt = data[0];

		apds9921_dd_set_prx_thresh(apds9921_i2c_client, p_data->pilt, p_data->piht);
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
	.store = apds9921_alsps_ps_thd_store,
};

static ssize_t apds9921_alsps_ps_min_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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
static ssize_t apds9921_alsps_ps_min_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ps_min);
}
static struct kobj_attribute ps_min_val =
{
	.attr = {"ps_min", 0664},
	.show = apds9921_alsps_ps_min_show,
	.store = apds9921_alsps_ps_min_store,
};

static ssize_t apds9921_alsps_algo_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
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
	.store = apds9921_alsps_algo_info_store,
};
static ssize_t apds9921_algo_wakeup_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct apds9921_data *data = i2c_get_clientdata(apds9921_i2c_client);
	wait_event_interruptible(enable_wq, data->enable_ps_sensor);
	printk(KERN_ERR"wait ps enable done\n");
	return snprintf(buf, PAGE_SIZE, "%d\n", data->enable_ps_sensor);
}
static struct kobj_attribute algo_wakeup =
{
	.attr = {"algo_wakeup", 0444},
	.show = apds9921_algo_wakeup_show,
};
static ssize_t apds9921_far_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct apds9921_data *data = i2c_get_clientdata(apds9921_i2c_client);
	return snprintf(buf, PAGE_SIZE, "%d\n",  data->ps_detection ? 0:1);
}
static struct kobj_attribute far_status =
{
	.attr = {"far_status", 0444},
	.show = apds9921_far_status_show,
};
#endif

static const struct attribute *apds9921_ftm_attrs[] =
{
	&enable.attr,
	&prox_raw.attr,
	&als_calibration.attr,
	&als_raw.attr,
	&reg_show.attr,
	&calibration.attr,
	&log_control.attr,
#ifdef APDS9921_ALSPS_DYNAMIC_THRESHOLD
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
static struct dev_ftm apds9921_ftm;

static struct i2c_driver apds9921_driver;
static int apds9921_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9921_data *data;
	int err = 0, irq = 0;

	printk("%s start.\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9921_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	pdev_data = data;
	data->client = client;
	apds9921_i2c_client = client;

	i2c_set_clientdata(client, data);

	data->ps_threshold = 100;
	data->ps_hysteresis_threshold = APDS9921_PS_HSYTERESIS_THRESHOLD;
	data->prev_ps_detection = 2;
	data->i2c_err_time = 0;
	data->esd_handle_flag= 0;

	data->ps_detection = 0;	/* default to no detection */
	data->ps_poll_delay = 25;	// default to 100ms
	data->ps_offset = 0; // default to 0, should load from nv memory if needed
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 100;	// default to 100ms
	data->als_res_index = APDS9921_DD_ALS_RES_18BIT;	// 100ms conversion time
	data->als_gain_index = APDS9921_DD_ALS_GAIN_18X;	// 3x GAIN
	data->als_gain = 1000;
	data->als_prev_lux = 0;
	data->als_cal_factor = 1050;	// default to 1.0, up scale to 100
	data->als_suspended = 0;
	data->ps_suspended = 0;
	data->main_ctrl_suspended_value = 0;	/* suspend_resume usage */

	//mutex_init(&data->update_lock);
	apds_parse_dt(&client->dev, data);

	wake_lock_init(&data->ps_wakelock,WAKE_LOCK_SUSPEND, "apds_input_wakelock");

	if (data->init)
	{
		err = data->init();
		if(err)
		{
			printk("data->init failed.\n");
			goto exit_data_init;
		}
	}

	if (data->power_on)
	{
		err = data->power_on(true);
		if(err)
		{
			printk("data->power_on failed.\n");
			goto exit_data_power_on;
		}
	}

	/* Initialize the APDS9921 chip */
	err = apds9921_init_client(client);
	if (err)
		goto exit_init_client_fail;

	err = gpio_request(data->irq, "apds_irq");
	if (err)
	{
		printk("Unable to request GPIO.\n");
		goto exit_kfree;
	}

	gpio_direction_input(data->irq);
	irq = gpio_to_irq(data->irq);
	if (irq < 0)
	{
		printk("Unable to request gpio irq. int pin = %d, irq = %d, err=%d\n", data->irq, irq, err);
		gpio_free(data->irq);

		goto exit_kfree;
	}

	if (request_irq(irq, apds9921_interrupt, IRQF_TRIGGER_FALLING, APDS9921_DRV_NAME, (void *)client))
	{
		printk("%s Could not allocate APDS9950_INT !\n", __func__);
		goto exit_kfree;
	}

	irq_set_irq_wake(client->irq, 1);

	// interrupt
	INIT_DELAYED_WORK(&data->dwork, apds9921_work_handler);
	// polling : ALS
	INIT_DELAYED_WORK(&data->als_dwork, apds9921_als_polling_work_handler);

	printk("%s interrupt is hooked\n", __func__);

	/* Register to Input Device */
	data->input_dev_als = devm_input_allocate_device(&client->dev);
	if (!data->input_dev_als) {
		err = -ENOMEM;
		printk("Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = devm_input_allocate_device(&client->dev);
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		printk("Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 30000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 10, 0, 0);

	data->input_dev_als->name = "light";
	data->input_dev_ps->name = "proximity";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("Unable to register input device als: %s\n",
				data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("Unable to register input device ps: %s\n",
				data->input_dev_ps->name);
		goto exit_free_dev_ps;
	}

#ifdef APDS9921_ALSPS_DYNAMIC_THRESHOLD
	INIT_DELAYED_WORK(&sample_ps_work, sample_work_func);

	queue_delayed_work(apds_workqueue,&sample_ps_work, msecs_to_jiffies(5000));

	init_waitqueue_head(&enable_wq);
#endif

	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = apds9921_als_set_enable;
	data->als_cdev.sensors_poll_delay = apds9921_als_poll_delay;
	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = apds9921_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&data->input_dev_als->dev, &data->als_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit_free_dev_ps;
	}

	err = sensors_classdev_register(&data->input_dev_ps->dev, &data->ps_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n", __func__, err);
		goto exit_unregister_als_ioctl;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9921_attr_group);
	if (err)
		goto exit_unregister_ps_ioctl;

	apds9921_ftm.name = "als_prox";
	apds9921_ftm.i2c_client = data->client;
	apds9921_ftm.attrs = apds9921_ftm_attrs;
	apds9921_ftm.priv_data = data;
	register_single_dev_ftm(&apds9921_ftm);

	if (data->power_on)
		err = data->power_on(false);

	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);

	return 0;

exit_unregister_ps_ioctl:
	sensors_classdev_unregister(&data->ps_cdev);
exit_unregister_als_ioctl:
	sensors_classdev_unregister(&data->als_cdev);
exit_free_dev_ps:
exit_free_dev_als:
exit_free_irq:
	free_irq(data->irq, client);
exit_kfree:
exit_init_client_fail:
	data->power_on(false);
exit_data_power_on:
	data->exit();
exit_data_init:
	wake_lock_destroy(&data->ps_wakelock);
	kfree(data);
	pdev_data = NULL;
exit:
	return err;
}

static int apds9921_remove(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);

	/* Power down the device */
	apds9921_dd_set_main_ctrl(client, 0);

	sysfs_remove_group(&client->dev.kobj, &apds9921_attr_group);

	free_irq(data->irq, client);

	wake_lock_destroy(&data->ps_wakelock);

	kfree(data);

	pdev_data = NULL;

	return 0;
}

static int apds9921_suspend(struct device *dev)
{
	struct apds9921_data *data ;
	int rc;

	data = dev_get_drvdata(dev);

	printk(KERN_ERR"%s\n", __func__);

	data->als_state_suspend_resume = data->enable_als_sensor;

	if (data->als_state_suspend_resume) {
		rc = apds9921_enable_als_sensor(data->client, 0);
		if (rc)
			dev_err(&data->client->dev,"Disable light sensor fail! rc=%d\n", rc);
	}

	if (data->irq && data->enable_ps_sensor)
	{
		disable_irq(data->irq);
	}

	return 0;
}

static int apds9921_resume(struct device *dev)
{
	struct apds9921_data *data;
	int rc;

	data = dev_get_drvdata(dev);

	printk(KERN_ERR"%s\n", __func__);

	if (data->als_state_suspend_resume) {
		rc = apds9921_enable_als_sensor(data->client, 1);
		if (rc)
			dev_err(&data->client->dev,"Disable light sensor fail! rc=%d\n", rc);
	}
	if (data->irq && data->enable_ps_sensor)
	{
		enable_irq(data->irq);
	}

	return 0;
}

static const struct i2c_device_id apds9921_id[] = {
	{ "apds9921", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9921_id);


static struct of_device_id apds_match_table[] = {
	{ .compatible = "apds,apds9921", },
	{ },
};

static const struct dev_pm_ops apds9921_pm_ops = {
	.suspend	= apds9921_suspend,
	.resume		= apds9921_resume,
};

static struct i2c_driver apds9921_driver = {
	.driver = {
		.name	= APDS9921_DRV_NAME,
		.of_match_table = apds_match_table,
		.owner	= THIS_MODULE,
		.pm = &apds9921_pm_ops,
	},
	.probe	= apds9921_probe,
	.remove	= apds9921_remove,
	.id_table = apds9921_id,
};

static int __init apds9921_init(void)
{
	apds_workqueue = create_freezable_workqueue("proximity_als");

	if (!apds_workqueue)
		return -ENOMEM;

	return i2c_add_driver(&apds9921_driver);
}

static void __exit apds9921_exit(void)
{
	if (apds_workqueue)
		destroy_workqueue(apds_workqueue);

	apds_workqueue = NULL;

	i2c_del_driver(&apds9921_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9921 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);



module_init(apds9921_init);
module_exit(apds9921_exit);
