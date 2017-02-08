/************************************************************************************
** File: - /android/kernel/drivers/input/touchscreen/synaptic_s4291_13095/synaptics_s4291_13095.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**      touch panel driver for synaptics
**      can change MAX_POINT_NUM value to support multipoint
** Version: 1.0
** Date created: 10:49:46,18/01/2012
** Author: Yixue.Ge@BasicDrv.TP
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
**  chenggang.li@BSP.TP modified for oppo 2014-07-30 14005 tp_driver
************************************************************************************/
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
//#include <mach/device_info.h>
#include <soc/oppo/boot_mode.h>
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/device_info.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>

#ifdef CONFIG_FB
	#include <linux/fb.h>
	#include <linux/notifier.h>
#endif

#include "oppo_tp_devices.h"

#include <linux/input/mt.h>
#include "synaptics_redremote.h"


/*------------------------------------------------Global Define--------------------------------------------*/
#define PAGESIZE 512
#define VKNUMBER 3
#define TPD_USE_EINT
static int LCD_WIDTH ;
static int LCD_HEIGHT ;

#define TPD_DEVICE "synaptics-s4291"

#define IMAGE_AREA_OFFSET 0x100

//#define KEY_USE

//#define SUPPORT_14017
#define SUPPORT_GESTURE
#define RESET_ONESECOND
#define SUPPORT_GLOVES_MODE

#define SUPPORT_TP_SLEEP_MODE
#define TYPE_B_PROTOCOL      //Multi-finger operation

#define TP_FW_NAME_MAX_LEN 64

#define TEST_MAGIC1 0x494D494C
#define TEST_MAGIC2 0x474D4954

struct test_header {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int withCBC;
	unsigned int array_limit_offset;
	unsigned int array_limit_size;
	unsigned int array_limitcbc_offset;
	unsigned int array_limitcbc_size;
};

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
	#define ENABLE_UNICODE  0x40
	#define ENABLE_VEE      0x20
	#define ENABLE_CIRCLE   0x08
	#define ENABLE_SWIPE    0x02
	#define ENABLE_DTAP     0x01

	#define UNICODE_DETECT  0x0b
	#define VEE_DETECT      0x0a
	#define CIRCLE_DETECT   0x08
	#define SWIPE_DETECT    0x07
	#define DTAP_DETECT     0x03
	
	// SUPPORT_GESTURE for s4291  
	#define UNICODE_DETECT_s4291  0x40
	#define VEE_DETECT_s4291      0x20
	#define CIRCLE_DETECT_s4291   0x08
	#define SWIPE_DETECT_s4291    0x02
	#define DTAP_DETECT_s4291     0x01


	#define UnkownGestrue       0
	#define DouTap              1   // double tap
	#define UpVee               2   // V
	#define DownVee             3   // ^
	#define LeftVee             4   // >
	#define RightVee            5   // <
	#define Circle              6   // O
	#define DouSwip             7   // ||
	#define Left2RightSwip      8   // -->
	#define Right2LeftSwip      9   // <--
	#define Up2DownSwip         10  // |v
	#define Down2UpSwip         11  // |^
	#define Mgestrue            12  // M
	#define Wgestrue            13  // W
#endif

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)	
#define TPDTM_DMESG(a, arg...)  printk(TPD_DEVICE ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)	
	
/*---------------------------------------------Global Variable----------------------------------------------*/
static int baseline_ret = 0;
static int TP_FW;
static int tp_dev = 2;
static unsigned int tp_debug = 0;
static unsigned int is_suspend = 0;
static int button_map[3];
static int tx_rx_num[2];
static int16_t Rxdata[30][30];
static int16_t delta_baseline[30][30];	
static int TX_NUM;
static int RX_NUM;
static int report_key_point_y = 0;
static int force_update = 0;
static int tp_probe_ok = 0;
static atomic_t is_touch;

static DEFINE_SEMAPHORE(work_sem);
struct manufacture_info tp_info;
static struct synaptics_ts_data *ts_g;
static struct workqueue_struct *synaptics_wq = NULL;
static struct workqueue_struct *speedup_resume_wq = NULL;
static struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_tpreset = NULL;

#ifdef SUPPORT_TP_SLEEP_MODE
	static atomic_t sleep_enable;
#endif

#ifdef SUPPORT_GLOVES_MODE	
	static atomic_t glove_enable;
#endif

static atomic_t baseline_test;
static atomic_t i2c_device_test;
static struct proc_dir_entry *prEntry_i2c_device_test = NULL;

#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

static uint32_t gesture_upload;
static int gesture_enable = 0;

static atomic_t double_enable;
static int is_gesture_enable = 0;
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
static struct proc_dir_entry *prEntry_double_tap = NULL;

/****point position*****/
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F11_2D_QUERY_BASE;
static int F11_2D_CMD_BASE;
static int F11_2D_CTRL_BASE;
static int F11_2D_DATA_BASE;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;

static int F11_2D_CTRL00;
static int F11_2D_CTRL06;
static int F11_2D_CTRL08;
static int F11_2D_CTRL32;
static int F11_2D_DATA38;
static int F11_2D_DATA39;
static int F11_2D_DATA01;
static int F11_2D_CMD00;

static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_DATA11;

#if TP_TEST_ENABLE
	static int F54_ANALOG_QUERY_BASE;//0x73
	static int F54_ANALOG_COMMAND_BASE;//0x72
	static int F54_ANALOG_CONTROL_BASE;//0x0d
	static int F54_ANALOG_DATA_BASE;//0x00
#endif

/***********for example of key event***********/
#ifdef KEY_USE
static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU, 90, 2050, 180, 100},
	{KEY_HOME, 500, 2050, 180, 100},
	{KEY_BACK, 855, 2050, 180, 100},
};
#endif

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_i2c_suspend(struct device *dev);
static int synaptics_i2c_resume(struct device *dev);
/**************I2C resume && suspend end*********/
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static void speedup_synaptics_resume(struct work_struct *work);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf);
static ssize_t synaptics_rmi4_baseline_show(struct device *dev, char *buf, bool savefile)	;
static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force);


#ifdef TPD_USE_EINT
	static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id);
#endif

#if defined(CONFIG_FB)
	static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif



/*-------------------------------Using Struct----------------------------------*/
struct point_info {
    int x;
    int raw_x;
    int y;
    int raw_y;
    int z;
};

static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct of_device_id synaptics_match_table[] = {
	{ .compatible = TPD_DEVICE ,},
	{ },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_FB
	.suspend = synaptics_i2c_suspend,
	.resume = synaptics_i2c_resume,
#endif
};	

static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,                         
	.id_table	= synaptics_ts_id,
	.driver = {
//		.owner  = THIS_MODULE,
		.name	= TPD_DEVICE,
		.of_match_table =  synaptics_match_table,
		.pm = &synaptic_pm_ops,
	},
};

static int vdd_2v8_exist = 1;
static int vdd_1v8_exist = 1;


struct synaptics_ts_data {
	struct i2c_client *client;
	struct mutex mutex;
	bool glove_mode_enabled;
	bool black_gesture_enabled;
	int irq;
	int irq_gpio;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int enable2v8_gpio;
	int max_num;
	int enable_remote;	
	int boot_mode;
	uint32_t irq_flags;
	uint32_t max_x;
    uint32_t max_y;
	uint32_t max_y_real;
	uint32_t btn_state;
    uint32_t pre_finger_state;
	uint32_t pre_btn_state;
	struct input_dev *kpd;
	struct work_struct  work;
	struct work_struct speed_up_work;
	struct input_dev *input_dev;	
	struct hrtimer timer;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif	
	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;	
	
	/*pinctrl******/
	struct device						*dev;
	struct pinctrl 						*pinctrl;	
	struct pinctrl_state 				*id_pullup;
	struct pinctrl_state 				*id_pulldown;
		
	/*******for FW update*******/
	bool suspended;
	bool loading_fw;
	char fw_name[TP_FW_NAME_MAX_LEN];
	char test_limit_name[TP_FW_NAME_MAX_LEN];
	char fw_id[12];
	char manu_name[12];
};

/*Virtual Keys Setting Start*/
static struct kobject *syna_properties_kobj;
static struct kobj_attribute qrd_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *qrd_properties_attrs[] = {
    &qrd_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group qrd_properties_attr_group = {
    .attrs = qrd_properties_attrs,
};

/*Virtual Keys Setting End*/
static struct device_attribute attrs_oppo[] = {
//	__ATTR(baseline_test, 0664, synaptics_rmi4_baseline_show, NULL),
	__ATTR(vendor_id, 0664, synaptics_rmi4_vendor_id_show, NULL),
 };
 
/*---------------------------------------------Fuction Apply------------------------------------------------*/
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
	return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
        ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
        ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
        "\n",LCD_WIDTH/6,button_map[2],button_map[0],button_map[1],LCD_WIDTH/2,button_map[2],button_map[0],button_map[1],LCD_WIDTH*5/6,button_map[2],button_map[0],button_map[1]);
}

static int synaptics_tpd_button_init(struct synaptics_ts_data *ts)
{
	int ret = 0;
	ts->kpd = input_allocate_device();
    if( ts->kpd == NULL ){
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_tpd_button_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->kpd->name = TPD_DEVICE "-kpd";
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	ts->kpd->id.bustype = BUS_HOST;
    ts->kpd->id.vendor  = 0x0001;
    ts->kpd->id.product = 0x0001;
    ts->kpd->id.version = 0x0100;
	
	if(input_register_device(ts->kpd)){
        TPDTM_DMESG("input_register_device failed.(kpd)\n");	
		input_unregister_device(ts->kpd);
		input_free_device(ts->kpd);
	}
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
    syna_properties_kobj = kobject_create_and_add("board_properties", NULL);
    if( syna_properties_kobj )
        ret = sysfs_create_group(syna_properties_kobj, &qrd_properties_attr_group);
    if( !syna_properties_kobj || ret )
		printk("failed to create board_properties\n");	

	err_input_dev_alloc_failed:		
		return ret;
}

static int Dot_report_down = 0;
static void tpd_down(struct synaptics_ts_data *ts,int raw_x, int raw_y, int x, int y, int p) 
{
    if( ts && ts->input_dev ){
        input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_key(ts->input_dev,	BTN_TOOL_FINGER, 1);
		
		if(ts->boot_mode == MSM_BOOT_MODE__RECOVERY)
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		TPD_DEBUG("Synaptics:Down[%4d %4d %4d]\n", x, y, p);	
		if( Dot_report_down == 150 ){
			TPD_ERR("Synaptics:Down[%4d %4d %4d]\n", x, y, p);		
			Dot_report_down = 0;
		}else{
			Dot_report_down++;
		} 
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(ts->input_dev);
#endif	
    }  
}



static int Dot_report_up = 0;
static void tpd_up(struct synaptics_ts_data *ts, int raw_x, int raw_y, int x, int y, int p) {	
	if( ts && ts->input_dev ){
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		if( Dot_report_up == 150 ){
			TPD_ERR("Up[%4d %4d %4d]\n", x, y, p);	
			Dot_report_up = 0;
		}else{
			Dot_report_up++;
		}
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(ts->input_dev);
#endif
    }  
}



#ifdef KEY_USE
static void tpd_button(struct synaptics_ts_data *ts,
    unsigned int x, unsigned int y, unsigned int down){
    int i;
	if( ts->max_x == 1145 ){
	    if( down ){
	        for(i = 0; i < VKNUMBER; i++){
	            if( x >= tpd_keys[i][1] &&
	                x <= tpd_keys[i][1]+tpd_keys[i][3] &&
	                y >= tpd_keys[i][2]&&
	                y <= tpd_keys[i][2]+tpd_keys[i][4] &&
	                !( ts->btn_state & (1 << i) ) ) {
	                input_report_key( ts->input_dev, tpd_keys[i][0], 1);
	                ts->btn_state |= ( 1<<i );
	            }
	        }
	    }else{
	        for(i=0; i<4; i++){
	            if( ts->btn_state&( 1<<i ) ){
	                input_report_key(ts->input_dev, tpd_keys[i][0], 0);
	            }
	        }
	        ts->btn_state = 0;
	    }
	}
}
#endif

static int tpd_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc;	
	if( vdd_2v8_exist ){
		if( regulator_count_voltages(ts->vdd_2v8) > 0){
			rc = regulator_set_voltage(ts->vdd_2v8, 2850000, 2850000);
			if(rc){
				dev_err(&ts->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
					return rc;
			}
		}
    }
	
	 if( vdd_1v8_exist ){
		if( regulator_count_voltages(ts->vcc_i2c_1v8) > 0 ){
			rc = regulator_set_voltage(ts->vcc_i2c_1v8, 1800000, 1800000);
			if(rc){
				dev_err(&ts->client->dev,
				"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				return rc;
			}
		}	
	} 
	
	/***enable the 2v8 power*****/
	if( vdd_2v8_exist ){
		rc = regulator_enable(ts->vdd_2v8);
		if(rc){
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
		}
	}
	
	if( ts->enable2v8_gpio > 0 ) {	
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 1);		
	}
	
	mdelay(1);
	if( vdd_1v8_exist ){
	rc = regulator_enable( ts->vcc_i2c_1v8 );
		if(rc){
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			regulator_disable(ts->vdd_2v8);
				return rc;
		}
		mdelay(3);
	} 
	
	if( ts->reset_gpio > 0 ) {	
		TPD_DEBUG("synaptics:enable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 1);		
	}
	msleep(100);
	return rc;
}

static int tpd_hw_pwroff(struct synaptics_ts_data *ts)
{ 
	int rc = 0;
	if( ts->reset_gpio > 0 ){
		TPD_ERR("synaptics:disable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 0);
	}
    if( vdd_2v8_exist ){
		rc = regulator_disable(ts->vdd_2v8);
		if (rc) {
			dev_err(&ts->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
    }
	if( ts->enable2v8_gpio > 0 )
	{	
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 0);		
	}
	return rc;
}

static int tpd_power(struct synaptics_ts_data *ts, unsigned int on)
{
	int ret;
	if(on){
		ret = tpd_hw_pwron(ts);	
	}else{
		ret = tpd_hw_pwroff(ts);
	}		
	return ret;
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];   
	int ret;
	memset(buf, 0, sizeof(buf));
   	ret = i2c_smbus_write_byte_data( ts->client, 0xff, 0x0 ); 
	if( ret < 0 ){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xDD, 4, &(buf[0x0]));
	if( ret < 0 ){
		TPD_ERR("failed for page select!\n");
		return -1;
	}
	
	F11_2D_QUERY_BASE = buf[0];
	F11_2D_CMD_BASE = buf[1];
	F11_2D_CTRL_BASE = buf[2]; 
	F11_2D_DATA_BASE = buf[3];

	TPD_ERR("F11_2D_QUERY_BASE = %x \n \
			F11_2D_CMD_BASE  = %x \n\
			F11_2D_CTRL_BASE	= %x \n\
			F11_2D_DATA_BASE	= %x \n\
			",F11_2D_QUERY_BASE,F11_2D_CMD_BASE,F11_2D_CTRL_BASE,F11_2D_DATA_BASE);

	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE3, 4, &(buf[0x0]));    
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2]; 
	F01_RMI_DATA_BASE = buf[3];
    TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
			   F01_RMI_CMD_BASE  = %x \n\
		       F01_RMI_CTRL_BASE	= %x \n\
		       F01_RMI_DATA_BASE	= %x \n\
		       ", F01_RMI_QUERY_BASE, F01_RMI_CMD_BASE, F01_RMI_CTRL_BASE, F01_RMI_DATA_BASE);

	ret = i2c_smbus_read_i2c_block_data( ts->client, 0xE9, 4, &(buf[0x0]) );	  
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2]; 
	F34_FLASH_DATA_BASE = buf[3];
	TPD_DEBUG("F34_FLASH_QUERY_BASE = %x \n\
			  F34_FLASH_CMD_BASE	= %x \n\
				F34_FLASH_CTRL_BASE	= %x \n\
				F34_FLASH_DATA_BASE	= %x \n\
			   ", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);
			   
	F01_RMI_QUERY11 = F11_2D_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1; 
	
	F11_2D_CTRL00 = F11_2D_CTRL_BASE;
	F11_2D_CTRL06 = F11_2D_CTRL_BASE + 6;
	F11_2D_CTRL08 = F11_2D_CTRL_BASE + 8;
	F11_2D_CTRL32 = F11_2D_CTRL_BASE + 15;
	F11_2D_DATA38 = F11_2D_DATA_BASE + 54;
	F11_2D_DATA39 = F11_2D_DATA_BASE + 55;
	F11_2D_DATA01 = F11_2D_DATA_BASE + 2;
	F11_2D_CMD00 = F11_2D_CMD_BASE;	
	
	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;
	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
	if( ret < 0 ){
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0])); 	   
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2]; 
	F51_CUSTOM_DATA_BASE = buf[3];
	F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;
	F51_CUSTOM_DATA11 = F51_CUSTOM_DATA_BASE;
	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			   F51_CUSTOM_CMD_BASE  = %x \n\
			   F51_CUSTOM_CTRL_BASE    = %x \n\
			   F51_CUSTOM_DATA_BASE    = %x \n\
			  ", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);  	
			  
#if TP_TEST_ENABLE	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
	if(ret < 0){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE3, 4, &(buf[0x0])); 
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	TPD_DEBUG("F54_QUERY_BASE = %x \n\
			 F54_CMD_BASE  = %x \n\
			 F54_CTRL_BASE	= %x \n\
			 F54_DATA_BASE	= %x \n\
				   ", F54_ANALOG_QUERY_BASE, F54_ANALOG_COMMAND_BASE , F54_ANALOG_CONTROL_BASE, F54_ANALOG_DATA_BASE);	
#endif
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 	
	return 0;
}

#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t status_int;
	uint8_t abs_status_int;
	
	TPD_DEBUG("%s is called\n", __func__);
	is_gesture_enable = enable;
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		msleep(20);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
		if( ret<0 )
			TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}
	ret = i2c_smbus_read_byte_data(ts->client, F11_2D_CTRL00);
	if(ret < 0) {
		TPD_DEBUG("read reg F11_2D_CTRL00 failed\n");
		return -1;
	}
	if(enable) {		
		status_int = (ret & 0xF8) | 0x04;
		/*enable gpio wake system through intterrupt*/		
		enable_irq_wake(ts->client->irq);
		gesture = UnkownGestrue ;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA01);
		if(ret < 0) {
			TPD_ERR("%s :clear interrupt bits failed\n",__func__);
			return -1;
		}		
	} else {
		status_int = ret & 0xF8;
		/*disable gpio wake system through intterrupt*/	
		disable_irq_wake(ts->client->irq);
	}
	printk("%s:status_int = 0x%x\n", __func__, status_int);

	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, status_int);
	if(ret < 0) {
		TPD_ERR("%s: enable or disable\
		interrupt failed,abs_int =%d\n",__func__,status_int);			
		ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, status_int);
		if(ret <0) {
			TPD_ERR("%s: enable or disable\
				interrupt failed,abs_int =%d second!\n",__func__,status_int);	
			return -1;
		}
	}
	TPD_DEBUG("%s gesture enable = %d\n", __func__,enable);

	if(enable) {
		abs_status_int = 0x3f;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA01);
		if(ret < 0) {
			TPD_DEBUG("%s :clear interrupt bits failed\n",__func__);
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}	

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01, abs_status_int);
	if(ret < 0) {
		TPD_DEBUG("%s: enable or disable abs \
			interrupt failed,abs_int =%d\n",__func__,abs_status_int);
		return -1;
	}		
	return 0;	
}
#endif

#ifdef SUPPORT_GLOVES_MODE
static int synaptics_glove_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	
	TPD_DEBUG("glove mode enable\n");
	/* page select = 0x4 */	
	if(1 == atomic_read(&glove_enable)) {	    
	/*0x00:enable glove mode,0x02:disable glove mode,*/
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
		if (ret < 0) {
			TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00,0x00 ); 
		if (ret < 0) {
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	} else {
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
		if (ret < 0) {
			TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
			goto GLOVE_ENABLE_END;
		}
		printk("glove mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00,0x02 ); 
		if (ret < 0) {
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}		
	}		
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if( ret < 0 ){
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		goto GLOVE_ENABLE_END;
	}
	
GLOVE_ENABLE_END: 
	return ret;
}
#endif

#ifdef SUPPORT_TP_SLEEP_MODE
static int synaptics_sleep_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	if( 1 == atomic_read(&sleep_enable) ){	    
	/*0x00:enable glove mode,0x02:disable glove mode,*/
	    TPDTM_DMESG("sleep mode enable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01 ); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}else{		  
	    TPDTM_DMESG("sleep mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); //chenggang.li@BSP add for oppo TP NO SLEEP set 1
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	
SLEEP_ENABLE_END:
	return ret;
}
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;
	
	memset(buf1, 0 , sizeof(buf1));
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY_BASE+11, 8, &(buf1[0x0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));	
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;
	
	TPD_DEBUG("%s is called!\n",__func__);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/\
    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80);// chenggang.li @BSP change 0x80 to 0x84
	if( ret < 0 ){
		msleep(150);
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	} 
//chenggang.li@BSP add for 14045 will delete
		/*  change Doze Interval to 30ms*/
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00+2, 3);
	if( ret < 0 ){
		msleep(150);
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00+2, 3); 
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	} 
	
    /*enable absolutePosFilter,rezero*/
	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
	if( enable ){
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE+1);
		if( ret < 0 ){
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	}else{
		abs_status_int = 0x0;		
	}	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if( ret < 0 ){
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n", __func__, abs_status_int);
		return -1;
	}
	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL00+1);
	//TPDTM_DMESG("S3202-----------0x5E=%x---------\n", ret);
	return 0;	
}

static void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;
    for(i = 0; i < w_ms; i++){
        for (j = 0; j < 1000; j++){
            udelay(1);
        }
    }
}

static void int_state(struct synaptics_ts_data *ts)
{
    int ret = -1;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	if(ret){
		TPD_ERR("int_state:cannot reset touch panel \n");
		return;
	}
	delay_qt_ms(250);
	
#ifdef SUPPORT_GLOVES_MODE
    synaptics_glove_mode_enable(ts_g);
#endif
	synaptics_init_panel(ts);
	if( ret < 0 ){
		TPD_DEBUG("int_state: control tm1400 to sleep failed\n");
		return;
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_DEBUG("int_state:cannot  enable interrupt \n");
		return;
	}
}

//chenggang.li@BSP.TP modified for oppo 2014-08-05 gesture_judge
/***************start****************/
#ifdef SUPPORT_GESTURE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
    int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11, 8, &(coordinate_buf[0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 8, 8, &(coordinate_buf[8]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 16, 8, &(coordinate_buf[16])); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 24, 1, &(coordinate_buf[24]));	
	for(i = 0; i< 23; i += 2) {
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
    }
	
	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);
	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
		Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
		Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (1280);
		Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
		Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (1280);
		Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
		Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (1280);
		Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
		Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (1280);
		Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
		Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (1280);
		Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
		Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (1280);	
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 : 
                                    (coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
}


static int get_swip_id(struct synaptics_ts_data *ts)
{
	int8_t gesture_buffer[12];
	//uint8_t gesture_buffer[12];
	int ret = 0,regswipe;
	int distance_x, distance_y;
	int ratio;
	
	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 	  
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	
	ret = i2c_smbus_read_i2c_block_data(ts->client,  F11_2D_DATA39, 12, &(gesture_buffer[0]));	
	distance_x = (gesture_buffer[1]<<8) | gesture_buffer[0];
	distance_y = (gesture_buffer[3]<<8) | gesture_buffer[2];
	ratio = distance_y / distance_x;
	if( gesture_buffer[11] == 2 ){
			regswipe = 0x80;
	}
	if( gesture_buffer[11] == 1 ){
		if( (ratio > 1) || (ratio < -1) ){
			if(distance_y < 0){
				regswipe = 0x48;
			}else{
				regswipe = 0x44;
			}
		}else{
			if(distance_x > 0){
				regswipe = 0x41;
			}else{
				regswipe = 0x42;
			}
		}
	}
	return regswipe;
}
static void gesture_judge(struct synaptics_ts_data *ts)
{
	int ret = 0,gesture_sign, regswipe;
    uint8_t gesture_buffer[10]; 
	
	TPD_DEBUG("%s is called!\n",__func__); 
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 	  
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	gesture_sign = i2c_smbus_read_byte_data(ts->client, F11_2D_DATA38);		
	ret = i2c_smbus_read_i2c_block_data(ts->client,  F11_2D_DATA39, 9, &(gesture_buffer[0]));	  
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
	regswipe = get_swip_id(ts);
	TPDTM_DMESG("  gesture_sign = 0x%x ,regswipe = 0x%x ,gesture_buffer[6] = 0x%x ,gesture_buffer[8] = 0x%x\n",gesture_sign,regswipe,gesture_buffer[6],gesture_buffer[8]);	  
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 

	//detect the gesture mode
	switch (gesture_sign) {
	case DTAP_DETECT_s4291:
		gesture = DouTap;
		break;
	case SWIPE_DETECT_s4291:
		gesture =   (regswipe == 0x41) ? Left2RightSwip   :
					(regswipe == 0x42) ? Right2LeftSwip   :
					(regswipe == 0x44) ? Up2DownSwip      :
					(regswipe == 0x48) ? Down2UpSwip      :
					(regswipe == 0x80) ? DouSwip          :
					UnkownGestrue;
		break;
	case CIRCLE_DETECT_s4291:
		gesture = Circle;
		break;
	case VEE_DETECT_s4291:
		gesture =   (gesture_buffer[6] == 0x01) ? DownVee  :
					(gesture_buffer[6] == 0x02) ? UpVee    :
					(gesture_buffer[6] == 0x04) ? RightVee :
					(gesture_buffer[6] == 0x08) ? LeftVee  : 
					UnkownGestrue;
		break;
	case UNICODE_DETECT_s4291:
		gesture =  (gesture_buffer[8] == 0x77) ? Wgestrue :
				   (gesture_buffer[8] == 0x6d) ? Mgestrue :
				 UnkownGestrue;
	 } 
	TPD_DEBUG("detect %s gesture\n", gesture == DouTap ? "double tap" :
													gesture == UpVee ? "up vee" :
													gesture == DownVee ? "down vee" :
													gesture == LeftVee ? "(>)" :
													gesture == RightVee ? "(<)" :
													gesture == Circle ? "circle" :
													gesture == DouSwip ? "(||)" :
													gesture == Left2RightSwip ? "(-->)" :
													gesture == Right2LeftSwip ? "(<--)" :
													gesture == Up2DownSwip ? "up to down |" :
													gesture == Down2UpSwip ? "down to up |" :
													gesture == Mgestrue ? "(M)" :
													gesture == Wgestrue ? "(W)" : "unknown");

   
	synaptics_get_coordinate_point(ts);
	if(gesture != UnkownGestrue ){
		gesture_upload = gesture;
		input_report_key(ts->input_dev, KEY_F4, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_F4, 0);
		input_sync(ts->input_dev);
    }		
	   
} 
#endif
/***************end****************/
static void int_touch_s4291(struct synaptics_ts_data *ts)
{
    int ret= -1,i=0, j = 0;
    uint8_t buf[5];
    uint32_t finger_state = 0;
	uint8_t finger_num = 0;
    struct point_info points;
    memset(buf,0,sizeof(buf));
    ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE, 3, &(buf[0]));
	if (ret < 0) {
		TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
        return;
	}
	points.x=0;
	points.y=0;
	points.z=0;
    finger_state = ((buf[2]&0x0f)<<16)|(buf[1]<<8)|buf[0];	
	for(j = 0;j < ts->max_num;j++){
	    if(finger_state&(0x03<<j*2))
	    finger_num = finger_num+1;
		atomic_set(&is_touch,finger_num);
	}

	if(finger_num > 0) {	
		for(i = 0;i < ts->max_num;i++){
		    ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE + 3 + i*5,
			        5, &(buf[0]));
			if (ret < 0) {
				TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
	        	return;
			}
			points.x = (buf[0]<<4) | (buf[2] & 0x0f);
			points.raw_x = buf[3]&0x0f;
			points.y = (buf[1]<<4) | ((buf[2] & 0xf0)>>4);
			points.raw_y = (buf[3]&0xf0)>>4;
		    points.z = buf[4];

			if(points.z > 0){
#ifdef TYPE_B_PROTOCOL
/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 1);
#endif
			tpd_down(ts,points.raw_x, points.raw_y, points.x, points.y,points.z);
			}
#ifdef TYPE_B_PROTOCOL
			else{
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
			}	
#endif				
		}
	} else {
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
#endif
		tpd_up(ts,points.raw_x, points.raw_y, points.x, points.y,points.z);
	}
			
	input_sync(ts->input_dev);
	ts->pre_finger_state = finger_state;  

#ifdef SUPPORT_GESTURE
	if (is_gesture_enable == 1) {
		gesture_judge(ts);
	}
#endif	
}

static void int_key_report_s3508(struct synaptics_ts_data *ts)
{
    int ret= 0;
	int F1A_0D_DATA00=0x00;
	
	
	TPD_DEBUG("%s is called!\n",__func__);
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x03);
	ret = i2c_smbus_read_byte_data(ts->client, F1A_0D_DATA00);
	if((ret & 0x01) && !(ts->pre_btn_state & 0x01))//menu
	{          
		input_report_key(ts->input_dev, KEY_BACK, 1);//KEY_MENU
		input_sync(ts->input_dev);
	}else if(!(ret & 0x01) && (ts->pre_btn_state & 0x01)){
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_sync(ts->input_dev); 
	}
					  
	if((ret & 0x02) && !(ts->pre_btn_state & 0x02))//home
	{ 
		input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);//KEY_HOMEPAGE
		input_sync(ts->input_dev);	
	}else if(!(ret & 0x02) && (ts->pre_btn_state & 0x02)){
		input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
		input_sync(ts->input_dev);
	}
			
	if((ret & 0x04) && !(ts->pre_btn_state & 0x04))//reback
	{				
		input_report_key(ts->input_dev, KEY_MENU, 1);//KEY_BACK
		input_sync(ts->input_dev);
	}else if(!(ret & 0x04) && (ts->pre_btn_state & 0x04)){
		input_report_key(ts->input_dev, KEY_MENU, 0);
		input_sync(ts->input_dev); 
	}
	ts->pre_btn_state = ret & 0x07;
	input_sync(ts->input_dev);
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
}
extern bool flag_lcd_off;
static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	uint8_t buf[5]; 	
    uint8_t status = 0;
	uint8_t inte = 0;
	uint8_t i2c_err_count = 0;	
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	
	if(flag_lcd_off == true){
		enable_irq(ts_g->client->irq);
		 return;
	}
		
	if(ts->loading_fw  || ts->enable_remote){
		enable_irq(ts_g->client->irq);
		return;
	} 
	
	memset(buf, 0, sizeof(buf));
	down(&work_sem);
	if( is_suspend == 1 )
		goto FREE_IRQ;
	ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
	if( ret < 0 ){
		if( ret != -5 ){
			TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
			goto FREE_IRQ;
		}
	/*for bug :i2c error when wake up system through gesture*/ 
		while( (ret == -5 ) && ( i2c_err_count < 10) ){
			msleep(5);
			ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
			i2c_err_count++;
		}
		TPDTM_DMESG("Synaptic:ret == %d and try %d times\n", ret, i2c_err_count);
	}
	status = ret & 0xff;
	inte = (ret & 0x7f00)>>8;
	if(status)	
		int_state(ts);
	if( inte & 0x40 )
		int_key_report_s3508(ts);
	if( inte & 0x04 )
		int_touch_s4291(ts);
	
FREE_IRQ:
		up(&work_sem);
		enable_irq(ts_g->client->irq);
	return;
}


#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(ts_g->client->irq);
	queue_work(synaptics_wq, &ts_g->work);
	return IRQ_HANDLED;
}
#endif


//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static ssize_t tp_baseline_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	
	char page[PAGESIZE];	
	if(baseline_ret == 0){
		count = synaptics_rmi4_baseline_show(ts_g->dev,page,1);				
		baseline_ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));		
	}else{
		baseline_ret = 0;
	}
	return baseline_ret;
}
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

static ssize_t i2c_device_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_DEBUG("double tap enable is: %d\n", atomic_read(&double_enable));
	ret = sprintf(page, "%d\n", atomic_read(&i2c_device_test));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
	return ret;
}

#ifdef SUPPORT_GESTURE
static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_DEBUG("double tap enable is: %d\n", atomic_read(&double_enable));
	ret = sprintf(page, "%d\n", atomic_read(&double_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
	return ret;
}

static ssize_t tp_double_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{ 
	int ret = 0;
	char buf[10];
	if( count > 2) 
		return count;	
	if( copy_from_user(buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}	
	
	sscanf(buf, "%d", &ret);
	if( (ret == 0 )||(ret == 1) )
			atomic_set(&double_enable, ret);	
		if(gesture_enable == 1)
	{
		switch(ret){
			case 0:
				TPD_ERR("tp_guesture_func will be disable\n");
				ret = synaptics_enable_interrupt_for_gesture(ts_g, 0); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts_g, 0); 
				ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, 0x01); 
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
					return -1;
				}
				break;
			case 1:
				TPD_ERR("tp_guesture_func will be enable\n");
				ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, 0x80); 
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
					return -1;
				}
				ret = synaptics_enable_interrupt_for_gesture(ts_g, 1); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts_g, 1); 
				break;
			default:
				TPD_ERR("Please enter 0 or 1 to open or close the double-tap function\n");
		}
	}
	return count;
}

static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{	
	int ret = 0;
	char page[PAGESIZE];
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
                   Point_start.x, Point_start.y, Point_end.x, Point_end.y,
                   Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
                   Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
                   clockwise);
				 
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;	
}
#endif

//chenggang.li@bsp add 14045
static char buffer_data[128];
static void synaptics_select_regieter(uint16_t address, int count , char *buffer_data)
{
	int i,len,tot=0;
	uint8_t ret;
	struct synaptics_ts_data *ts;
	ts = ts_g;
	TPD_ERR("%s:is called\n",__func__);
	TPD_ERR("the address is 0x%x\n",address);
	if(count>8)
		count = 8;
		
	for(i=0;i<count;i++){
		TPD_ERR("0x0013+%d is 0x%x\n", i,0xF9A3+i);
		ret = i2c_smbus_read_byte_data(ts_g->client,0x00F9A3+i);
		TPD_ERR("ret is -----0x%x-----------\n",ret);
		len = sprintf(buffer_data+tot,"0x%02x  ",ret);
		tot+=len;
	}
	buffer_data[tot]='\0';
	TPD_ERR("buffer_data is %s\n",buffer_data);
}

static ssize_t read_base_register_address(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret;
	char page[PAGESIZE];
	TPD_ERR("%s is called\n",__func__);
	ret = sprintf(page,"%s", buffer_data);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}
static ssize_t write_base_register_address(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[24];
	int data,cnt;
	int off;

	strcpy (buf,buffer);
	cnt = sscanf(buf,"%x %d",&off, &data);
//	TPD_ERR("off is 0x%zx; data is %d\n",off,data);
	synaptics_select_regieter(off,data,buffer_data);
	return count;
}

#ifdef SUPPORT_GLOVES_MODE
static ssize_t tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("glove mode enable is: %d\n", atomic_read(&glove_enable));
	ret = sprintf(page, "%d\n", atomic_read(&glove_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_glove_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts;
	int ret = 0 ;
	char buf[10];
	
	//	down(&work_sem);
	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	ts = ts_g;
	TPDTM_DMESG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){	
		atomic_set(&glove_enable, ret);
		synaptics_glove_mode_enable(ts);
	}	
	switch(ret){	
		case 0:	
			TPDTM_DMESG("tp_glove_func will be disable\n");
			break;
		case 1:	
			TPDTM_DMESG("tp_glove_func will be enable\n");
			break;		
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
//	up(&work_sem);
	return count;
}
#endif


#ifdef SUPPORT_TP_SLEEP_MODE
static ssize_t tp_sleep_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
	char page[PAGESIZE];
	printk("sleep mode enable is: %d\n", atomic_read(&sleep_enable));
	ret = sprintf(page, "%d\n", atomic_read(&sleep_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_sleep_write_func(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    struct synaptics_ts_data *ts; 
    int ret = 0 ;
	char buf[10];
	if( count > 10 ) 
		return count;	
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &ret);	 
	ts = ts_g;
	TPDTM_DMESG("tp_sleep_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){
		atomic_set(&sleep_enable, ret);
		synaptics_sleep_mode_enable(ts);		 
	}
	switch(ret){
		case 0:
			TPDTM_DMESG("tp_sleep_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_sleep_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}
#endif

static ssize_t tp_show(struct device_driver *ddri, char *buf)
{
    uint8_t ret = 0;
	ret = i2c_smbus_read_word_data(ts_g->client, F01_RMI_DATA_BASE);
	if( ret < 0 )
		printk("tp_show read i2c err\n");	
	ret = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_DATA01);
	if( ret < 0 )
		printk("tp_show read i2c err\n");
	return sprintf(buf, "0x13=0x%x\n", ret);
}

static ssize_t store_tp(struct device_driver *ddri, const char *buf, size_t count)
{
	int tmp = 0;
	if( 1 == sscanf(buf, "%d", &tmp) ){
		tp_debug = tmp;
	}
	else {
		TPDTM_DMESG("invalid content: '%s', length = %zd\n", buf, count);
	}	
	return count;
}

#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
	unsigned char buf[4];
	int ret;
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x1); 
	if( ret < 0 ){
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE3, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	return 0;
}

static void checkCMD(void)
{
	int ret;
	int flag_err = 0;
	do {
		delay_qt_ms(30); //wait 10ms
		ret = i2c_smbus_read_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE);
		flag_err++;
    }while( (ret > 0x00) && (flag_err < 30) ); 
	if( ret > 0x00 )
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);	
}
#endif

//chenggang.li@BSP.TP modified for oppo 2014-07-28
static ssize_t tp_baseline_show(struct device_driver *ddri, char *buf)
{
    int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	uint16_t tmp_old = 0;
	uint16_t tmp_new = 0;
	if( is_suspend == 1 )
	   return count;
	memset(delta_baseline, 0, sizeof(delta_baseline));
	/*disable irq when read data from IC*/	
    disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
    down(&work_sem);
	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");
	
	/*step 1:check raw capacitance*/
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_CONTROL_BASE+20, 0x01);
	ret = i2c_smbus_read_byte_data(ts_g->client, F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret & 0xff;
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_CONTROL_BASE+23, (tmp_old & 0xef));
	ret = i2c_smbus_write_word_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+27);
	tmp_new = ret & 0xdf;
	i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
	ret = i2c_smbus_write_word_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update 	
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
	
	ret = i2c_smbus_write_word_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD();
	//TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	//TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){   	
	//printk("\n[%d]", x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h << 8)|tmp_l;
			//printk("%d,", delta_baseline[x][y]);	
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ", delta_baseline[x][y]);
		}
	}
	TPDTM_DMESG("\nread all is oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);
    delay_qt_ms(60);

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts_g);
#endif	
	synaptics_init_panel(ts_g);	
    synaptics_enable_interrupt(ts_g, 1);
	enable_irq(ts_g->client->irq);
	up(&work_sem);
	return num_read_chars;
}

static ssize_t tp_rawdata_show(struct device_driver *ddri, char *buf)
{    
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0, tmp_h = 0; 
	uint16_t count = 0;
	if( is_suspend == 1 )
	   return count;
	memset(delta_baseline, 0, sizeof(delta_baseline));
	/*disable irq when read data from IC*/	
    disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
    down(&work_sem);
	//TPD_DEBUG("\nstep 2:report type2 delta image\n");	
	memset(delta_baseline, 0, sizeof(delta_baseline));
	ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
	ret = i2c_smbus_write_word_data(ts_g->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report
	checkCMD();
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;       		
			//printk("%3d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%3d ", delta_baseline[x][y]);
		}	 
	}
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    synaptics_enable_interrupt(ts_g, 1);
	enable_irq(ts_g->client->irq);
	up(&work_sem);
	return num_read_chars;
}

static ssize_t tp_delta_store(struct device_driver *ddri,
       const char *buf, size_t count)
{
	  TPDTM_DMESG("tp_test_store is not support\n");
	  return count;
}

static ssize_t synaptics_rmi4_baseline_show_s4291(struct device *dev, char *buf, bool savefile)	
{
	ssize_t num_read_chars = 0;		
#if TP_TEST_ENABLE
    int ret = 0;
	uint8_t x,y;
	//int tx_data = 0;
	//int tx_datah;
	//int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0, tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	int error_count = 0;
	int enable_cbc = 0;
	int16_t *baseline_data_test;
	
	int fd = -1;
    struct timespec   now_time;
    struct rtc_time   rtc_now_time;
    uint8_t  data_buf[64];
    mm_segment_t old_fs;
	
	struct synaptics_ts_data *ts_g = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
	ret = request_firmware(&fw, ts_g->test_limit_name, dev);	
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",ts_g->test_limit_name, ret);		
		error_count++;
		num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);
		num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
		return num_read_chars;
	}	
		
	ph = (struct test_header *)(fw->data);	
    down(&work_sem);	
    disable_irq_nosync(ts_g->client->irq);	
	memset(Rxdata,0,sizeof(Rxdata));
	synaptics_read_register_map_page1(ts_g);	//scan_map_page_one
	TPDTM_DMESG("step 1:select report type 0x03\n");
	
	if(savefile) {
        getnstimeofday(&now_time);
        rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
        sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv", 
                (rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
                rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
        if (fd < 0) {
            //print_ts(TS_DEBUG, "Open log file '%s' failed.\n", data_buf);
			TPD_ERR("Open log file '%s' failed.\n", data_buf);
            set_fs(old_fs);
        }
	
    }
	 //step 1:check raw capacitance.
TEST_WITH_CBC_4291:
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}	
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7);
	tmp_old = ret&0xff;
	if(enable_cbc){
		tmp_new = tmp_old | 0x10;
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old, tmp_new);
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7,(tmp_old | 0x10));
		TPDTM_DMESG("Test with cbc\n");
		baseline_data_test = (uint16_t *)(fw->data + ph->array_limitcbc_offset);	
		
	}else{
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7,(tmp_old & 0xef));	
		TPDTM_DMESG("Test without cbc\n");	
		baseline_data_test = (uint16_t *)(fw->data + ph->array_limit_offset);		
	}
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);//force update
	checkCMD();
	TPDTM_DMESG("forbid CBC oK\n");
	
	//Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x02);//Force Cal, F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;	
	

	for(x = 0;x < TX_NUM; x++) {
		for(y = 0; y < RX_NUM; y++)  {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;			
			if (fd >= 0){
                sprintf(data_buf, "%d,", baseline_data);
                sys_write(fd, data_buf, strlen(data_buf));
            }
			if( (y < (RX_NUM-3)) && (x < TX_NUM-1) ){ 				
				if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
						TPD_ERR("Synaptic tpbaseline_fail;count[%d][%d] =%d ;Array_limit[count*2]=%d,Array_limit[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
						num_read_chars += sprintf(&(buf[num_read_chars]), "0 baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
						error_count++;
						goto END;
				    }
			}
			count++;
		}
		if (fd >= 0){
                sys_write(fd, "\n", 1);
        }
	    printk("\n");
   	}	
	if(!enable_cbc){
		enable_cbc = 1;
		TPD_ERR("test cbc baseline again\n");
			goto TEST_WITH_CBC_4291;
	}
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7, tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	

 //step 3 :check rx-to-rx
	TPDTM_DMESG("s4291 step 3 :check rx-to-rx\n" );
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE, 0x07);//select report type 0x07
	//Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);//force F54_ANALOG_CMD00
    TPDTM_DMESG("s4291 1\n" );
	checkCMD();                               //
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x02);//Force Cal,F54_ANALOG_CMD00
    TPDTM_DMESG("s4291 2\n" );
	checkCMD();
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
    TPDTM_DMESG("s4291 3\n" );
	checkCMD();
    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);//read report
	TPDTM_DMESG("F54_ANALOG_CMD00[2]=%d \n",ret);
	TPDTM_DMESG("s4291 111111111111111111\n" );
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);
	for(x = 0;x < TX_NUM; x++) {
		for(y = 0; y < RX_NUM; y++) {
			ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			Rxdata[x][y] = ret&0xffff;
		}
	}
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x7);//select report type 0x17 
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);//force update
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
	for(x = 0;x < RX_NUM - TX_NUM; x++) {
		printk("Rxdata[%d][%d]:",x+TX_NUM,y);   
		for(y = 0; y < RX_NUM; y++) {
			ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			 Rxdata[x + TX_NUM][y] = ret&0xffff;	 
		     printk("%5d",Rxdata[x + TX_NUM][y]);   
		}
		printk("\n");
	}
	TPDTM_DMESG("\nstep 4:check rx-rx short\n"); 
//step 4:check rx-rx short
	/* for(x = 0;x < RX_NUM; x++) {
			for(y = 0; y < RX_NUM; y++) {
				if ((x ==y)&&(x!=0)) {
					printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
					if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit)) {
						num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
						printk("Synaptic check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
						error_count++;
						goto END;
					}	
				}
			}
		} */
	
	num_read_chars += sprintf(buf, "1");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);

END:
	if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }	
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);
	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	TPDTM_DMESG("4 read F54_ANALOG_CTRL07 is: 0x%x\n",ret);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
    msleep(150);

#ifdef SUPPORT_GLOVES_MODE
    synaptics_glove_mode_enable(ts_g);
#endif	

    synaptics_init_panel(ts_g);
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	printk("\n\n syanptics:step5 reset and open irq complete\n");
    up(&work_sem);	
#endif
	return num_read_chars;
}		


static ssize_t tp_baseline_show_with_cbc(struct device_driver *ddri, char *buf)
{ 
    int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint8_t tmp_old, tmp_new;
	uint16_t count = 0;
	if(is_suspend == 1)
	   return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
	/*disable irq when read data from IC*/	
    disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
    down(&work_sem);
	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");
	//step 1:check raw capacitance.

	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7);
	tmp_old = ret&0xff;
	TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
	tmp_new = ret | 0x10;
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+7,tmp_new);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	TPD_DEBUG("forbid CBC oK\n");
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01)	

	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
	checkCMD();
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
		
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {   	
		TPD_DEBUG("\n[%d]",x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
		for(y = 0; y < RX_NUM; y++){
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			TPD_DEBUG("%d,",delta_baseline[x][y]);	
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",delta_baseline[x][y]);
		}
   	}
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	up(&work_sem);
	return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_show(struct device *dev,	char *buf, bool savefile)	
{	
	return synaptics_rmi4_baseline_show_s4291(dev,buf,savefile);	
}


static ssize_t tp_test_store(struct device_driver *ddri,
       const char *buf, size_t count)
{
   TPDTM_DMESG("tp_test_store is not support\n");
   return count;
}


static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	if( (tp_dev == TP_G2Y) || (tp_dev == TP_TPK) )
		return sprintf(buf, "%d\n", TP_TPK); 
	if(tp_dev == TP_TRULY)
		return sprintf(buf, "%d\n", TP_TRULY); 
    if(tp_dev == TP_OFILM)
		return sprintf(buf, "%d\n", TP_OFILM);	
	return sprintf(buf, "%d\n", tp_dev); 
}


static int	synaptics_input_init(struct synaptics_ts_data *ts)
{
	int attr_count = 0;
	int ret = 0;
	
	TPD_DEBUG("%s is called\n",__func__);
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		return -1;
	}
	ts->input_dev->name = TPD_DEVICE;;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume	
	set_bit(KEY_MENU , ts->input_dev->keybit);
	set_bit(KEY_HOMEPAGE , ts->input_dev->keybit);
	set_bit(KEY_BACK , ts->input_dev->keybit);
#endif
	
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);		
#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif				
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	input_set_drvdata(ts->input_dev, ts);
	
	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);			
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);	
		return -1;
	}
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs_oppo); attr_count++) {
		ret = sysfs_create_file(&ts->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
		if (ret < 0) {
			dev_err(&ts->client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			for (attr_count--; attr_count >= 0; attr_count--) {
				sysfs_remove_file(&ts->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
			}
			return -1;
		}
	}		
	return 0;
}
static int synaptics_device_irq_init(struct synaptics_ts_data *ts)
{
	int ret = 0;
	static int function_call_time = 0;
	TPD_DEBUG("%s is called\n",__func__);
	if(function_call_time > 0){
		TPD_ERR("%s is called %d times\n",__func__,function_call_time);
		return -1;
	}
	function_call_time++;	
	/*config tm1429: set report rate, sleep mode */
	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPDTM_DMESG("synaptics_init_panel failed\n");	
	}

	ret = synaptics_enable_interrupt(ts, 1);
	TPD_DEBUG("synaptics_enable_interrupt\n");
	
	if(ret) {
		TPD_ERR("synaptics_ts_probe: failed to enable synaptics  interrupt \n");
		return -1;
	}
#ifdef TPD_USE_EINT
	TPDTM_DMESG("%s: set IRQ GPIO\n",__func__);
/****************
shoud set the irq GPIO
*******************/		
	if (gpio_is_valid(ts->irq_gpio)) {
		/* configure touchscreen irq gpio */
		ret = gpio_request(ts->irq_gpio,"rmi4_irq_gpio");
		if (ret) {
			TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
		}
//		msleep(20);
	}
	
	TPD_DEBUG("synaptic:ts->client->irq is %d\n",ts->client->irq);
	ret = request_irq(ts_g->client->irq, synaptics_ts_irq_handler, ts->irq_flags, TPD_DEVICE, ts_g);
	if(ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
#endif
	return ret;
}

/*********************Added for tp FW update******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
    int ret;
	char fw_id_temp[12];
	uint8_t buf[4];
	static int call_times = 0;
	uint32_t CURRENT_FIRMWARE_ID = 0 ;
	
	TPD_DEBUG("%s is called\n",__func__);
	TPDTM_DMESG("synatpitcs_fw_update: fw_name = %s time = %d\n", ts->fw_name, call_times);
	if(!ts->client){
		TPD_ERR("i2c client point is NULL\n");	
		return 0;
	}
	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
						ts->fw_name, ret);
		return ret;
	}
	ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force); 
	if(ret < 0){
		ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
		if(ret < 0){
			TPD_ERR("FW update failed twice, quit updating process!\n");
			return ret;
		}
	}
	release_firmware(fw);	
	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);	
	ret = i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	sprintf(fw_id_temp,"0x%x",CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	if(force_update == 1){
		ret = synaptics_input_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_input_init failed!\n");			
		}
		ret = synaptics_tpd_button_init(ts);	
		if(ret < 0){
			TPD_ERR("synaptics_tpd_button_init failed!\n");			
		}
		ret = synaptics_device_irq_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_device_irq_init failed!\n");			
		}
	}else{
	#ifdef SUPPORT_GLOVES_MODE
		synaptics_glove_mode_enable(ts);
	#endif	
		synaptics_init_panel(ts);
		synaptics_enable_interrupt(ts,1);	
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_mt_sync(ts->input_dev);	
	input_sync(ts->input_dev);
	call_times++;
	return 0;
}

static ssize_t synaptics_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t synaptics_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)				
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
	int rc;
	TPD_DEBUG("synaptics:start update fw\n");	
	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;
	if(!dev){
		dev_info(dev, "dev is NULL return!!!!!\n");
        return size;		
	}
	if (tp_probe_ok==0) {
		dev_info(dev, "probe not finished return!!!!\n");
		return size;
	}
	if((val == 1)||(force_update==1))
	{
		val = 1;
	}else{
		val = 0;
	}
	if(force_update == 0){
		disable_irq_nosync(ts->client->irq);	
		mutex_lock(&ts->input_dev->mutex);
	}
	TPD_DEBUG("synaptics:loading_fw %x\n",ts->loading_fw);	
	if (!ts->loading_fw) {
		ts->loading_fw = true;
		synatpitcs_fw_update(dev, val);
		ts->loading_fw = false;
	}
	if(force_update == 0){
		mutex_unlock(&ts->input_dev->mutex);	
		enable_irq(ts->client->irq);
	}
	force_update = 0;
	return size;
}
/****************************End*************************************/


static ssize_t synaptics_test_limit_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	int ret = 0;
	uint16_t *prow = NULL; 
	uint16_t *prowcbc = NULL; 
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
	int i = 0;
	int temp = 0;
	static int cat_cbc_change = 0;
	ret = request_firmware(&fw, ts->test_limit_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
						ts->test_limit_name, ret);
		temp = temp + sprintf(&buf[temp],"Request failed,Check the path %d",temp);
		return temp;
	}
		
	ph = (struct test_header *)(fw->data);
	prow = (uint16_t *)(fw->data + ph->array_limit_offset);

	prowcbc = (uint16_t *)(fw->data + ph->array_limitcbc_offset);

	TPD_DEBUG("synaptics_test_limit_show:array_limit_offset = %x array_limitcbc_offset = %x\n",
		ph->array_limit_offset,ph->array_limitcbc_offset);
	
	TPD_DEBUG("test begin:\n");
	if(cat_cbc_change == 0) {
		temp += sprintf(buf, "Without cbc:");
		for(i = 0 ;i < (ph->array_limit_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
			   temp += sprintf(&(buf[temp]), "\n[%d] ",(i/RX_NUM)/2);
			temp += sprintf(&buf[temp],"%d,",prow[i]);
			printk("%d,",prow[i]);						   
		}
		cat_cbc_change = 1;
	}else{
		temp += sprintf(buf, "With cbc:");
		if( ph->withCBC == 0){
		  return temp;
		}
		for(i = 0 ;i < (ph->array_limitcbc_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
			   temp += sprintf(&(buf[temp]), "\n[%d] ",(i/RX_NUM)/2);
			temp += sprintf(&buf[temp],"%d,",prowcbc[i]);
			printk("%d,",prowcbc[i]);
		}
		cat_cbc_change = 0;
	}
	release_firmware(fw);
	return temp;	
}

static ssize_t synaptics_test_limit_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return size;
}

//static DRIVER_ATTR(oppo_tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);
static DEVICE_ATTR(test_limit, 0664, synaptics_test_limit_show, synaptics_test_limit_store);
static DRIVER_ATTR(oppo_tp_baseline_image, 0664, tp_baseline_show, tp_delta_store);
static DRIVER_ATTR(oppo_tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);
static DRIVER_ATTR(oppo_tp_delta_image, 0664, tp_rawdata_show, NULL);
static DRIVER_ATTR(oppo_tp_debug, 0664, tp_show, store_tp);
static DEVICE_ATTR(oppo_tp_fw_update, 0664, synaptics_update_fw_show, synaptics_update_fw_store);


static ssize_t tp_write_func (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	return count;
}

// chenggang.li@BSP.TP modified for oppo 2014-08-08 create node 
/******************************start****************************/
static const struct file_operations tp_double_proc_fops = {
	.write = tp_double_write_func,
	.read =  tp_double_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

//chenggang.li@bsp add for 14045
static const struct file_operations base_register_address= {
	.write = write_base_register_address,
	.read =  read_base_register_address,
	.open = simple_open,
	.owner = THIS_MODULE,
};


//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations i2c_device_test_fops = {	
	.read =  i2c_device_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations tp_baseline_test_proc_fops = {	
	.read =  tp_baseline_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

#ifdef SUPPORT_GLOVES_MODE
static const struct file_operations glove_mode_enable_proc_fops = {
	.write = tp_glove_write_func,
	.read =  tp_glove_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

static const struct file_operations sleep_mode_enable_proc_fops = {
	.write = tp_sleep_write_func,
	.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations tp_reset_proc_fops = {
	.write = tp_write_func,
	//.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int init_synaptics_proc(struct synaptics_ts_data *ts)
{
	int ret = 0;		
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create TP proc entry\n");
	}
    	
#ifdef SUPPORT_GESTURE
	if(ts->black_gesture_enabled) {
		prEntry_double_tap = proc_create( "double_tap_enable", 0666, prEntry_tp, &tp_double_proc_fops);
		if(prEntry_double_tap == NULL){
			ret = -ENOMEM;
			printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		}	
	}
	prEntry_coodinate = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
	if(prEntry_coodinate == NULL){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
#endif	 

#ifdef SUPPORT_TP_SLEEP_MODE
	prEntry_dtap = proc_create("sleep_mode_enable", 0666, prEntry_tp, &sleep_mode_enable_proc_fops);
	if( prEntry_dtap == NULL ){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
#endif	
		
#ifdef RESET_ONESECOND
	prEntry_tpreset = proc_create( "tp_reset", 0666, prEntry_tp, &tp_reset_proc_fops);
	if( prEntry_tpreset == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create tp reset proc entry\n");
	}
#endif
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
	prEntry_double_tap = proc_create( "baseline_test", 0666, prEntry_tp, &tp_baseline_test_proc_fops);
	if(prEntry_double_tap == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\i2c_device_test"  begin
	prEntry_i2c_device_test = proc_create( "i2c_device_test", 0666, prEntry_tp, &i2c_device_test_fops);
	if(prEntry_i2c_device_test == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}	
	
	prEntry_i2c_device_test = proc_create( "synaptics_register_address", 0777, prEntry_tp, &base_register_address);
	if(prEntry_i2c_device_test == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}	
	
	return ret;
}
/******************************end****************************/

/****************************S3310*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT_s4291(struct i2c_client *client)
{
	  uint8_t buf[8];
	  i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	  SynaF34DataBase = buf[3];
	  SynaF34QueryBase = buf[0];
	  i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	  SynaF01DataBase = buf[3];
	  SynaF01CommandBase = buf[1];
	  i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);
	  SynaF34Reflash_BlockNum = SynaF34DataBase;
	  SynaF34Reflash_BlockData = SynaF34DataBase + 2;
	  SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	  TPD_DEBUG("SynaF34ReflashQuery_BootID = 0x%x \n",SynaF34ReflashQuery_BootID);
	  SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 2;
	  TPD_DEBUG("SynaF34ReflashQuery_FlashPropertyQuery = 0x%x \n",SynaF34ReflashQuery_FlashPropertyQuery);
	  SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 3;
	  TPD_DEBUG("SynaF34ReflashQuery_FirmwareBlockSize = 0x%x \n",SynaF34ReflashQuery_FirmwareBlockSize);
	  SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +5;
	  TPD_DEBUG("SynaF34ReflashQuery_FirmwareBlockCount = 0x%x \n",SynaF34ReflashQuery_FirmwareBlockCount);
	  SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	  TPD_DEBUG("SynaF34ReflashQuery_ConfigBlockSize = 0x%x \n",SynaF34ReflashQuery_ConfigBlockSize);
	  SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 7;
	  TPD_DEBUG("SynaF34ReflashQuery_ConfigBlockCount = 0x%x \n",SynaF34ReflashQuery_ConfigBlockCount);
	  i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize,2, buf);
	  SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	  TPDTM_DMESG("SynaFirmwareBlockSize = 0x%x \n",SynaFirmwareBlockSize);
	  SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;
	  TPDTM_DMESG("SynaF34_FlashControl = 0x%x \n",SynaF34_FlashControl);
}

struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" debug checksume is %x", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is %x", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is %x", header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if( header->contains_firmware_id )
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

//chenggang.li@BSP.TP modified for oppo 2014-07-29
static int checkFlashState(void)
{
	int ret ;
	int count = 0;
	int FLSH_STATUS = 0;	
	FLSH_STATUS = SynaF34_FlashControl;	
	//msleep(5);
	ret =  i2c_smbus_read_byte_data(ts_g->client,FLSH_STATUS);
	//TPD_ERR(" ret is %x\n", ret);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		ret =  i2c_smbus_read_byte_data(ts_g->client,FLSH_STATUS);
		count++;
    } 
	if(count == 8)
		return 1;
	else
		return 0;
}



/****************************S3310*****end!!!!!**********************************/

static int synaptics_fw_check(struct synaptics_ts_data *ts )
{
	int ret;
//	uint8_t buf[4];
	uint32_t bootloader_mode;
	int max_y_ic = 0;
	
	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);	
		return -1;
	}
	
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}
	
	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}
   	/*read max_x ,max_y*/
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if (ret < 0) {
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
		if(ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			return -1;
		}
	}
	ret = i2c_smbus_read_word_data(ts->client, F11_2D_CTRL06);
	if(ret > 0)
		ts->max_x = ret&0xffff;       
	ret = i2c_smbus_read_word_data(ts->client, F11_2D_CTRL08);
	if(ret > 0)
		max_y_ic = ret&0xffff;	
	ts->max_y = max_y_ic ;
	TPD_ERR("max_x = %d,max_y = %d\n",ts->max_x,max_y_ic);
	
	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 	
	TPD_DEBUG("afte fw update,program memory self-check bootloader_mode = 0x%x\n",bootloader_mode);	
	
 	if((ts->max_x == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)) {
		TPD_ERR("Something terrible wrong \n Trying Update the Firmware again\n");		
		return -1;
	}
	return 0;
}

static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force)
{
	int ret,j;
	uint8_t buf[8];
	uint8_t bootloder_id[10];
	uint16_t disp_configuration_add;
	uint16_t disp_configuration_size;
	uint16_t bootloader_addr;
	uint16_t bootloader_size;
	uint8_t contains_bootloader = 0;
	const uint8_t *Firmware_Data = NULL;
	const uint8_t *Config_Data = NULL;
	struct image_header_data header;
	uint16_t block,firmware,configuration;
	uint32_t CURRENT_FIRMWARE_ID = 0 , FIRMWARE_ID = 0;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);	
	
	TPD_DEBUG("%s is called\n",__func__);
	if(!client)
		return -1;
		
	parse_header(&header,data);	
	if((header.firmware_size + header.config_size + 0x100) > data_len) {
		TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
		return -1;
	}

	Firmware_Data = data + 0x100;
	Config_Data = Firmware_Data + header.firmware_size;
	disp_configuration_add = (unsigned int)(data[0x40+0]) + 
							 (unsigned int)(data[0x40+1])*0x100 + 
							 (unsigned int)(data[0x40+2])*0x10000 + 
							 (unsigned int)(data[0x40+3])*0x1000000 ;
	TPD_DEBUG("disp_configuration_add is 0x%x \n", disp_configuration_add);
	disp_configuration_size = ((unsigned int)(data[0x44+0])+
								(unsigned int)(data[0x44+1])*0x100 +
								(unsigned int)(data[0x44+2])*0x10000 +
								(unsigned int)(data[0x44+3])*0x1000000)/16;
	TPD_DEBUG("disp_configuration_size is %d \n", disp_configuration_size);
	contains_bootloader = data[0x06] & 0x02;
	TPD_DEBUG("contains_bootloader is %d \n", contains_bootloader);
	if(contains_bootloader){
		bootloader_addr  = 	 (unsigned int)(data[0x20+0])+
							 (unsigned int)(data[0x20+1])*0x100 +
							 (unsigned int)(data[0x20+2])*0x10000 +
							 (unsigned int)(data[0x20+3])*0x1000000;
		TPD_DEBUG("bootloader_addr is 0x%x \n", bootloader_addr);
		bootloader_size = 	 (unsigned int)(data[0x24+0])+
							 (unsigned int)(data[0x24+1])*0x100 +
							 (unsigned int)(data[0x24+2])*0x10000 +
							 (unsigned int)(data[0x24+3])*0x1000000;
		TPD_DEBUG("bootloader_size is 0x%x \n", bootloader_size);			
	}
	ret = i2c_smbus_write_byte_data(client, 0xff, 0x0);	

	ret = i2c_smbus_read_i2c_block_data(client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	FIRMWARE_ID = 0x14045017;
	TPD_ERR("CURRENT_FIRMWARE_ID is %x-----------, FIRMWARE_ID is %x-----------\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);
	TPD_ERR("synaptics force is %d\n", force);	
	if(!force) {
		if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
			return 0;
		}	
	}	
	
	re_scan_PDT_s4291(client);
	
	block = 16;
	TPD_DEBUG("block is %d \n",block);
	firmware = (header.firmware_size)/16;
    TPD_DEBUG("firmware is %d \n",firmware);
    configuration = (header.config_size)/16;
    TPD_DEBUG("configuration is %d \n",configuration);

	ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));  
	TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	TPD_DEBUG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);
		
	i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0F);
	msleep(10);	
	TPD_DEBUG("attn step 4\n");
	ret=checkFlashState();
	if(ret > 0) {
		TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
			return -1;
	}
	ret = i2c_smbus_read_byte_data(client,0x04);
	TPD_DEBUG("The status(device state) is %x\n",ret);
	ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
	TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
	ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
	/********************get into prog end************/
	
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	TPD_DEBUG("ret is %d\n",ret);
	re_scan_PDT_s4291(client);
	i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
	i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
	i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
	msleep(2500);
	ret = i2c_smbus_read_byte_data(client, SynaF34_FlashControl);
	if(ret != 0x00)
		msleep(2000);
		
		
	ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
	TPDTM_DMESG("The status(erase) is %x\n",ret);		

	TPD_ERR("update-----------------update------------------update!\n");
	TPD_DEBUG("cnt %d\n",firmware);
	for(j=0; j<firmware; j++) {
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;		
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16 + bootloader_size]);
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
		ret=checkFlashState();		
		if(ret > 0) {
			TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}		
	}
	//step 7 configure data
	//TPD_ERR("going to flash configuration area\n");
	//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
	//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
	for(j=0;j<configuration;j++) {
		//a)write SynaF34Reflash_BlockNum to access
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
		//b) write data
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16 + header.firmware_size + bootloader_size]);	   
		//c) issue write
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
		//d) wait attn		
		ret = checkFlashState();	
		if(ret > 0) {
			TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}
	
	TPD_DEBUG("Write Display CFG\n");
	TPD_DEBUG("Write bootloader id\n");
	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData,bootloder_id[0]);
	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+1,bootloder_id[1]);
	msleep(10);
	//ERASE DISP CONFIG
	i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0A);
	msleep(2500);
	ret = checkFlashState();	
	if(ret > 0) {
		TPD_ERR("checkFlashState failed\n");
		return -1;
	}
	//Write DISP Configuration
	TPDTM_DMESG("going to flash DISP configuration area\n");
	buf[0]=0;
	buf[1]= (0x03 << 5);//Display configuration area
	i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
	for(j=0;j<disp_configuration_size;j++)
	{
		i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 16, &data[j*16+disp_configuration_add]); 
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
		//d) wait attn
		ret = checkFlashState();	
		if(ret > 0) {
			TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}
	
	//step 1 issue reset
	i2c_smbus_write_byte_data(client,SynaF01CommandBase,0X01);
	//step2 wait ATTN
	//delay_qt_ms(1000);
	mdelay(1500);
    synaptics_read_register_map(ts);
	//FW flash check!
	ret =synaptics_fw_check(ts);
	if(ret < 0 ) {
		TPD_ERR("Firmware self check failed\n");
		return -1;
	}
	TPD_DEBUG("Firmware self check Ok\n");
	return 0;
}
/****************************S3310*****end!!!!!**********************************/

static  void get_tp_id(int TP_ID1,int TP_ID2,int TP_ID3, struct synaptics_ts_data *ts)
{
	int ret,id1 = -1,id2 = -1,id3 = -1;
	if(TP_ID1  >= 0) {
		ret = gpio_request(TP_ID1,"TP_ID1");
	}
	if(TP_ID2  >=  0) {
		ret = gpio_request(TP_ID2,"TP_ID2");
	}	
	if(TP_ID3  >= 0) {
		ret = gpio_request(TP_ID3,"TP_ID3");
	}
//	msleep(80);
	if(TP_ID1  >= 0) {
	    id1=gpio_get_value(TP_ID1);	
	}
	if(TP_ID2  >=  0) {
		id2=gpio_get_value(TP_ID2);		
	}	
	if(TP_ID3  >= 0) {
		id3=gpio_get_value(TP_ID3);
	}
	
	if(id1 == 1){
		TPDTM_DMESG("%s::TP_TRULY\n",__func__);			
		tp_dev=TP_TRULY;
	}else
	{
		TPDTM_DMESG("%s::TP_TPK\n",__func__);
		tp_dev=TP_TPK;
	}	
}
 
static int synaptics_parse_dts(struct device *dev, struct synaptics_ts_data *ts)
{
	int rc;
	struct device_node *np;
	int lcd_size[2];
	
	
	np = dev->of_node;
	ts->glove_mode_enabled = of_property_read_bool(np,
				"synaptics,glove-mode-enabled");
	ts->black_gesture_enabled = of_property_read_bool(np,
				"synaptics,black-gesture-enabled");
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
	if( ts->irq_gpio < 0 ){
		TPD_DEBUG("ts->irq_gpio not specified\n");	
	}
	ts->id1_gpio = of_get_named_gpio(np, "synaptics,id1-gpio", 0);
	if( ts->id1_gpio < 0 ){
		TPD_DEBUG("ts->id1_gpio not specified\n");	
	}
	TPD_DEBUG("synaptic:id1_gpio %d\n",ts->irq_gpio);
	ts->id2_gpio = of_get_named_gpio(np, "synaptics,id2-gpio", 0);
	if( ts->id2_gpio < 0 ){
		TPD_DEBUG("ts->id2_gpio not specified\n");	
	}
	ts->id3_gpio = of_get_named_gpio(np, "synaptics,id3-gpio", 0);	
	if( ts->id3_gpio < 0 ){
		TPD_DEBUG("ts->id3_gpio not specified\n");	
	}
		
	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);	
	if( ts->reset_gpio < 0 ){
		TPD_DEBUG("ts->reset-gpio  not specified\n");	
	}
	ts->enable2v8_gpio = of_get_named_gpio(np, "synaptics,enable2v8-gpio", 0);	
	if( ts->enable2v8_gpio < 0 ){
		TPD_DEBUG("ts->enable2v8_gpio not specified\n");	
	}
	/* ts->enable1v814045_gpio = of_get_named_gpio(np, "synaptics,enable1v814045-gpio", 0);	
	if( ts->enable1v814045_gpio < 0 ){
		TPD_ERR("ts->enable1v814045_gpio not specified\n");	
	} */
	
	rc = of_property_read_u32(np, "synaptics,max-num-support", &ts->max_num);
	if(rc){
		TPD_DEBUG("ts->max_num not specified\n");
		ts->max_num = 10;
	}
	
	rc = of_property_read_u32(np, "synaptics,max-y-point", &ts->max_y);
	if(rc){
		TPD_DEBUG("ts->max_y not specified\n");
		ts->max_num = 10;
	}
	
	rc = of_property_read_u32_array(np, "synaptics,button-map", button_map, 3);
	if(rc){
		TPD_DEBUG("button-map not specified\n");
		button_map[0] = 180;
		button_map[1] = 180;
		button_map[2] = 2021;
	}
	TPD_DEBUG("synaptics:button map readed is %d %d %d\n", button_map[0], button_map[1], button_map[2]);
	
	rc = of_property_read_u32_array(np, "synaptics,tx-rx-num", tx_rx_num,2);
	if(rc){
		TPD_ERR("button-map not specified\n");
		TX_NUM =  13;	
	    RX_NUM =  23;	
	}else{
        TX_NUM =  tx_rx_num[0];	
	    RX_NUM =  tx_rx_num[1];	
	}
	TPD_ERR("synaptics,tx-rx-num is %d %d \n", TX_NUM,RX_NUM);	
	
	rc = of_property_read_u32_array(np, "synaptics,display-coords", lcd_size, 2);
	if(rc){
		TPD_ERR("lcd size not specified\n");
		LCD_WIDTH = 1080;
		LCD_HEIGHT = 1920;
	}else{
		LCD_WIDTH = lcd_size[0];
		LCD_HEIGHT = lcd_size[1];	
	}
	TPDTM_DMESG("synaptic:ts->irq_gpio:%d irq_flags:%u id1_gpio:%d id2_gpio:%d id3_gpio:%d max_num %d\n"
				,ts->irq_gpio, ts->irq_flags ,ts->id1_gpio, ts->id2_gpio, ts->id3_gpio, ts->max_num);
				
/***********power regulator_get****************/
	ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
	if( IS_ERR(ts->vdd_2v8) ){
		rc = PTR_ERR(ts->vdd_2v8);
		TPD_DEBUG("Regulator get failed vdd rc=%d\n", rc);
	   vdd_2v8_exist = 0;		
	}	
	
	ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
	if( IS_ERR(ts->vcc_i2c_1v8) ){
		rc = PTR_ERR(ts->vcc_i2c_1v8);
		TPD_DEBUG("Regulator get failed vcc_i2c rc=%d\n", rc);
		vdd_1v8_exist = 0;
	}
	
	if( ts->reset_gpio > 0){
		if( gpio_is_valid(ts->reset_gpio) ){
			rc = gpio_request(ts->reset_gpio, "rmi4-reset-gpio");
			if(rc){
				TPD_ERR("unable to request gpio [%d]\n", ts->reset_gpio);
			}
		}
	}
	
	if( ts->enable2v8_gpio > 0){
		if( gpio_is_valid(ts->enable2v8_gpio) ){
			rc = gpio_request(ts->enable2v8_gpio, "rmi4-enable2v8-gpio");
			if(rc){
				TPD_ERR("unable to request gpio [%d]\n", ts->enable2v8_gpio);
			}
		}
	}
	
	/* if( ts->enable1v814045_gpio > 0){
		if( gpio_is_valid(ts->enable1v814045_gpio) ){
			rc = gpio_request(ts->enable1v814045_gpio, "rmi4-enable1v814045-gpio");
			if(rc){
				TPD_ERR("unable to request gpio [%d]\n", ts->enable1v814045_gpio);
			}
		}
	} */
	
	return rc;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts = NULL;
	#ifdef CONFIG_SYNAPTIC_RED	
	struct remotepanel_data *premote_data = NULL;
	#endif
	int ret = -1;
	int rc;
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;

	TPD_ERR("%s is called\n",__func__);
	atomic_set(&baseline_test, 0);
	atomic_set(&i2c_device_test, 0);
	atomic_set(&is_touch, 0);	
#ifdef SUPPORT_GESTURE
	atomic_set(&double_enable, 0);
#endif

#ifdef SUPPORT_GLOVES_MODE
	atomic_set(&glove_enable, 0);
#endif

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if( ts == NULL ){
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	memset(ts, 0, sizeof(*ts));	
	ts->boot_mode = get_boot_mode();
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->loading_fw = false;
	ts_g = ts;
	/*********parse DTS***********/
	synaptics_parse_dts(&client->dev, ts);
	get_tp_id(ts->id1_gpio,ts->id2_gpio,ts->id3_gpio,ts);
	if(tp_dev == TP_TRULY) {
		printk("synaptics_ts_probe1 will return for tp_dev == TP_TRULY\n");
		goto err_alloc_data_failed;	
	}
	/***power_init*****/
	
	mutex_init(&ts->mutex);
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
    if( !synaptics_wq ){
        ret = -ENOMEM;
		goto err_alloc_data_failed;	
    }   	
	speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
	if( !speedup_resume_wq ){
         ret = -ENOMEM;
		goto err_alloc_data_failed;	
    }   
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_WORK(&ts->speed_up_work,speedup_synaptics_resume);
	
#ifdef SUPPORT_GESTURE
	is_gesture_enable = 0;
#endif

	ret = tpd_power(ts, 1);	
	if( ret < 0 )
		TPD_ERR("regulator_enable is called\n");
	
	init_synaptics_proc(ts);
	ret = i2c_smbus_read_byte_data(client, 0x13);
	atomic_set(&i2c_device_test, ret);	
	 
	if( (ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN) ){
		TPD_DEBUG("regulator_disable is called\n");
		if( vdd_2v8_exist ){
			rc = regulator_disable(ts_g->vdd_2v8);
			if (rc) {
				TPD_DEBUG("regulator_disable failed\n");
			}
		}
		if( ts->enable2v8_gpio > 0 )
		{	
			TPD_DEBUG("synaptics:disable the enable2v8_gpio\n");
			gpio_direction_output(ts->enable2v8_gpio, 0);				
		}
		
		msleep(5);
		return 0;
		
	}
	/*****power_end*********/
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ){
		TPD_ERR("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}	
	
    ret = i2c_smbus_read_byte_data(client, 0x13);
	if( ret < 0 ){
		ret = i2c_smbus_read_byte_data(client, 0x13);
		if( ret < 0 ){
			tpd_power(ts, 0);
			printk("synaptics is no exist!\n");
			return 0;
		}
	}
	
	synaptics_read_register_map(ts);
	bootloader_mode = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	TPD_DEBUG("synaptics:before fw update,bootloader_mode = 0x%x\n", bootloader_mode);
	
	i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
	TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);	
	TP_FW = CURRENT_FIRMWARE_ID;
		
	sprintf(ts->fw_id,"0x%x",TP_FW);
	tp_info.version = ts->fw_id;
	
	memset(ts_g->fw_name,TP_FW_NAME_MAX_LEN,0);
	memset(ts_g->test_limit_name,TP_FW_NAME_MAX_LEN,0);
	
	tp_info.manufacture = "TPK-0D-NEW";	

	tp_info.manufacture = "TP_TPK";			
	strcpy(ts->fw_name,"tp/14045/14045_FW_S4291_Tpk.img");
	strcpy(ts->test_limit_name,"tp/14045/14045_Limit_Tpk.img");

	TPD_DEBUG("synatpitcs_fw: fw_name = %s \n",ts->fw_name);
	register_device_proc("tp", tp_info.version, tp_info.manufacture);	
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if( ret < 0 ){
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}
	
	ret = synaptics_fw_check(ts);
	if(ret < 0 )
	{	
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");		
	}else{	
		force_update = 0;
		ret = synaptics_input_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_input_init failed!\n");			
		}
		ret = synaptics_tpd_button_init(ts);	
		if(ret < 0){
			TPD_ERR("synaptics_tpd_button_init failed!\n");			
		}
		ret = synaptics_device_irq_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_device_irq_init failed!\n");			
		}
	}		

#ifndef TPD_USE_EINT
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret)
		TPD_ERR("Unable to register fb_notifier: %d\n", ret);
#endif
	
	if (device_create_file(&client->dev, &dev_attr_test_limit)) {            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	TPD_DEBUG("synaptics_ts_probe: going to create files--oppo_tp_fw_update\n");	
	if (device_create_file(&client->dev, &dev_attr_oppo_tp_fw_update)) {            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}  
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_debug) ){            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}	
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_baseline_image_with_cbc)) {           
		TPDTM_DMESG("driver_create_file failt\n");		
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_baseline_image) ){            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_delta_image) ){            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}		

	TPDTM_DMESG("synaptics_ts_probe: normal end\n");
	tp_probe_ok = 1;
	#ifdef CONFIG_SYNAPTIC_RED	
	premote_data = remote_alloc_panel_data();
	if(premote_data)
	{
		premote_data->client 		= client;
		premote_data->input_dev		= ts->input_dev;
		premote_data->kpd			= ts->kpd;
		//premote_data->pmutex		= &ts->mutex;
		premote_data->irq_gpio 		= ts->irq_gpio;
		premote_data->irq			= client->irq;
		premote_data->enable_remote = &(ts->enable_remote);
		register_remote_device(premote_data);

	}
	#endif
	return 0;

exit_init_failed:
err_check_functionality_failed:
	tpd_power(ts, 0);
err_alloc_data_failed:
	kfree(ts);
	printk("synaptics_ts_probe: not normal end\n");
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	int attr_count;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

#if defined(CONFIG_FB)
	if( fb_unregister_client(&ts->fb_notif) )
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif
	
#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif	

	for(attr_count = 0; attr_count < ARRAY_SIZE(attrs_oppo); attr_count++){
		sysfs_remove_file(&ts_g->input_dev->dev.kobj, &attrs_oppo[attr_count].attr);
	}
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
    tpd_hw_pwroff(ts);
	return 0;
}


static int synaptics_ts_suspend(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_ERR("%s: is called\n", __func__);	
	
	if(ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		return -1;
	}
	gesture_enable = 1;
	atomic_set(&is_touch, 0);
/***********report Up key when suspend********/	
	input_report_key(ts->input_dev, KEY_MENU, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_BACK, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#ifndef TYPE_B_PROTOCOL
    input_mt_sync(ts->input_dev);	
#endif
	input_sync(ts->input_dev);

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
	if( ret < 0 ){
		TPD_ERR("write F01_RMI_CTRL00 failed\n");
		return -1;
	}
	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, 0x0c); 
	if( ret < 0 ){
		TPD_ERR("write F11_2D_CTRL00 failed\n");
		return -1;
	}
#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

#ifdef SUPPORT_GLOVES_MODE    
	if( 1 == atomic_read(&glove_enable) ){
		/* page select = 0x4 */
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			return 0;
		}
		//printk("glove mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00, 0x02 ); 	
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			return 0;
		}
	}
#endif

#ifdef SUPPORT_GESTURE	
		if( 1 == atomic_read(&double_enable) ){
			synaptics_enable_interrupt_for_gesture(ts, 1);
			TPD_ERR("synaptics:double_tap end suspend\n");
			return 0;	
		}
#else
    is_suspend = 1;
#endif

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret){
		TPD_DEBUG("%s: cannot disable interrupt\n", __func__);
		return -1;
	}
	ret = cancel_work_sync(&ts->work);
	if(ret) {
		TPD_DEBUG("%s: cannot disable work\n", __func__);
	}
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPD_ERR("synaptics_enable_interrupt failed\n");
	}	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01); 
	if( ret < 0 ){
		TPD_ERR("%s: control tm1400 to sleep failed\n", __func__);
		return -1;
	}
	//TPDTM_DMESG("%s:normal end\n", __func__);
	return 0;
}

static int synaptics_ts_resume(struct device *dev)
{
	struct synaptics_ts_data *ts_g = dev_get_drvdata(dev);
	TPD_DEBUG("%s is called\n", __func__);
	atomic_set(&is_touch, 0);
	if(ts_g->loading_fw)
		return 0;
	queue_work(speedup_resume_wq, &ts_g->speed_up_work);
	return 0;
}

//chenggang.li@BSP add for oppo for 14045 project!
/*****************start********************/
int set_wakeup_gesture_mode(uint8_t value)
{
	int ret;
	TPD_ERR("%s is called\n", __func__);
	ret = i2c_smbus_write_byte_data(ts_g->client, F11_2D_CTRL00, value);
	if( ret < 0 ){
		TPD_ERR("write F11_2D_CTRL00 failed!\n");
	}
	return 0;
}
/*****************end********************/


static void speedup_synaptics_resume(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = ts_g;
	TPD_ERR("%s is called\n", __func__);
/***********report Up key when resume********/	
	if(ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		goto ERR_RESUME;
	}
	gesture_enable = 0;
	input_report_key(ts_g->input_dev, BTN_TOUCH, 0);
#ifndef TYPE_B_PROTOCOL
    input_mt_sync(ts_g->input_dev);	
#endif
	input_sync(ts_g->input_dev);
	ts->pre_btn_state = 0;
	
if( ts_g->reset_gpio > 0 )
	{		
		gpio_direction_output(ts_g->reset_gpio, 1);	
	}
	
	free_irq(ts_g->client->irq, ts_g);
	down(&work_sem);			
#ifndef TPD_USE_EINT
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
	/***Reset TP ******/ 
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPD_ERR("%s: failed for page select try again later\n", __func__);
		msleep(20);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
		if( ret < 0 ){
			TPD_ERR("%s: failed for page select try again later\n", __func__);
		}
	}
	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE);
	/* ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
	msleep(500); */
	/*****Gesture Register********/	
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif	

/*****Normal Init TP********/
    ret = synaptics_init_panel(ts);
	if( ret < 0 ){
		TPD_ERR("%s: TP init failed\n", __func__);
		goto ERR_RESUME;
	}
	ret = request_irq(ts_g->client->irq, synaptics_ts_irq_handler, ts->irq_flags, TPD_DEVICE, ts_g);	
#endif
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_ERR("%s:can't  enable interrupt!\n", __func__);
		goto ERR_RESUME;
	}	
	is_suspend = 0;
    TPD_ERR("%s:normal end!\n", __func__);	
	up(&work_sem);
	return;
	
ERR_RESUME:
	up(&work_sem);
	return;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DEBUG("%s: is called\n", __func__);	
	disable_irq(ts->client->irq);	
	return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DEBUG("%s is called\n", __func__);
	enable_irq(ts->client->irq);
	return 0;
}

#if defined(CONFIG_FB)
static int count_resume = 1;
extern bool te_reset_14045;
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;	
	int ret;
	struct synaptics_ts_data *ts = container_of(self, struct synaptics_ts_data, fb_notif);
	
	TPD_DEBUG("%s is called \n", __func__);
	if( evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client ){
		blank = evdata->data;
		if( *blank == FB_BLANK_UNBLANK ){
			if( count_resume == 0 ){
				TPD_DEBUG("%s going TP resume\n", __func__);				
				synaptics_ts_resume(&ts->client->dev);
				count_resume = 1;
			}		
		}else if( *blank == FB_BLANK_POWERDOWN ){
			if( count_resume == 1){		
				if(te_reset_14045 == 1 ){
					ret = cancel_work_sync(&ts->work);
					if(ret) {
						TPD_DEBUG("%s: cannot disable work\n", __func__);
					}
					disable_irq(ts_g->client->irq);
					count_resume = 0;
					return 0;
				} 			
				TPD_DEBUG("%s : going TP suspend\n", __func__);
				synaptics_ts_suspend(&ts->client->dev);
				count_resume = 0;
			}		
		} 
	}
	return 0;
}
#endif

static int __init tpd_driver_init(void) {
	printk("Synaptics:%s is called\n", __func__);
	 if( i2c_add_driver(&tpd_i2c_driver)!= 0 ){
        TPDTM_DMESG("unable to add i2c driver.\n");
        return -1;
    }	
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void){
 	i2c_del_driver(&tpd_i2c_driver);
	if(synaptics_wq ){
		destroy_workqueue(synaptics_wq);
		synaptics_wq = NULL;
	}	
	return;
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics s4291 Touchscreen Driver");
MODULE_LICENSE("GPL");
