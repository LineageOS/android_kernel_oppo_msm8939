/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd
 ** CONFIG_MACH_OPPO
 ** File        : gt9xx.c
 ** Description :
 ** Date        : 2015-04-21 17:07
 ** Author      : BSP.TP
 **
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

/* drivers/input/touchscreen/gt9xx.c
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.2
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2014/01/14
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 *      V2.2:
 *          1. gt9xx_config for debug
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *                  By Meta, 2014/01/14
 */

#include "gt9xx.h"

#if 0 /* delete it by lauson. */
/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#define GOODIX_PINCTRL_STATE_SLEEP "gt9xx_int_suspend"
#define GOODIX_PINCTRL_STATE_DEFAULT "gt9xx_int_default"
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#endif

//static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
    = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if 0 //#if GTP_DEBUG_ON
static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH};
static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif

#endif

#define TPD_DEVICE "goodix-ts"
static int LCD_WIDTH = 720;
static int LCD_HEIGHT = 1280;
static u32 button_map[MAX_BUTTONS];

#define GOODIX_VTG_MIN_UV   2850000
#define GOODIX_VTG_MAX_UV   2850000
#define GOODIX_I2C_VTG_MIN_UV   1800000
#define GOODIX_I2C_VTG_MAX_UV   1800000
#define GOODIX_VDD_LOAD_MIN_UA  0
#define GOODIX_VDD_LOAD_MAX_UA  10000
#define GOODIX_VIO_LOAD_MIN_UA  0
#define GOODIX_VIO_LOAD_MAX_UA  10000
#define PROP_NAME_SIZE      24
#define GOODIX_COORDS_ARR_SIZE  4

#define OPPO_RAWDATA_INTERFACE

#define GT9XX_SUPPORT_GESTURE
#define GT9XX_SUPPORT_GLOVES_MODE
#define GT9XX_SUPPORT_REPORT_COORDINATE

#define GT9XX_SUPPORT_APK_AUTOTEST   //add the proc for apk , tp autotest.

#define PAGESIZE  512

#ifdef GT9XX_SUPPORT_REPORT_COORDINATE
#include "circle_point.h"
#endif

#ifdef GT9XX_SUPPORT_GESTURE
static atomic_t double_enable;
//static int is_gesture_enable = 0;
static uint32_t gesture;


#define DTAP_DETECT          0xCC
#define UP_VEE_DETECT        0x76
#define DOWN_VEE_DETECT      0x5e
#define LEFT_VEE_DETECT      0x3e
#define RIGHT_VEE_DETECT     0x63
#define CIRCLE_DETECT        0x6f
#define DOUSWIP_DETECT       0x48
#define RIGHT_SLIDE_DETECT   0xAA
#define LEFT_SLIDE_DETECT    0xbb
#define DOWN_SLIDE_DETECT    0xAB
#define UP_SLIDE_DETECT      0xBA
#define M_DETECT             0x6D
#define W_DETECT             0x77




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

#ifdef GT9XX_SUPPORT_REPORT_COORDINATE


struct Coordinate
{
    int x;
    int y;
};
static  Point Point_input[64];
static  Point Point_output[5];
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
static uint32_t clockwise;
#endif


static struct proc_dir_entry *prEntry_tp = NULL;
#ifdef GT9XX_SUPPORT_GLOVES_MODE
static struct proc_dir_entry *prEntry_glove_mode = NULL;
#endif
#ifdef GT9XX_SUPPORT_GESTURE
static struct proc_dir_entry *prEntry_coodinate  = NULL;
static struct proc_dir_entry *prEntry_double_tap = NULL;
#endif
#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static struct proc_dir_entry *prEntry_glove_check = NULL;
#endif

#define GT9XX_UPDATE_FILE_PATH  "/system/etc/firmware/"
char gt9xx_updata_file_path[128] = {0}; // fw patch.
int gt9xx_force_update = 0;
static int tp_probe_ok = 0;

static u8 chip_gt9xxs;  /* true if ic is gt9xxs, like gt915s */

#define GTP_LARGE_AREA_SLEEP            0   /*[BUGFIX] Add by TCTSZ.WH,2014-5-27,Add palm lock function.*/
//static int gtp_is_suspended = 0;  /*[BUGFIX]ADD Begin by TCTSZ-WH,2014-5-20,Enable tp double click to unlock screen.*/
static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(struct goodix_ts_data *ts, s32 ms);
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);

static int gt9xx_free_fingers(struct goodix_ts_data *ts);

static void speedup_goodix_resume(struct work_struct *work);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops =
{
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#ifdef OPPO_RAWDATA_INTERFACE
s32 gtp_sysfs_init(void);
void gtp_sysfs_uninit(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
#endif

//*********** For GT9XXF Start **********//
#if GTP_COMPATIBLE_MODE
extern s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);

void gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);

#endif

#ifdef GT9XX_SUPPORT_GLOVES_MODE
static atomic_t glove_enable;
#endif
#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static int glove_check = 0;
#endif


static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
//********** For GT9XXF End **********//

#ifdef CONFIG_TCT_8X16_POP8LTE
extern u8 g_wakeup_gesture;
#endif

#if GTP_GESTURE_WAKEUP
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
} DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif

//u8 grp_cfg_version = 0;
u8 sensor_id = 0;
static char fw_version[12];

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while (retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {
#if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#if GTP_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
#endif
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
#endif
        {
            gtp_reset_guitar(client, 10);
        }
    }
    return ret;
}



/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while (retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if ((retries >= 5))
    {
#if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#if GTP_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
#endif
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
#endif
        {
            gtp_reset_guitar(client, 10);
        }
    }
    return ret;
}


/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;

    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len + 2))
        {
            memcpy(rxbuf, confirm_buf + 2, len);
            return SUCCESS;
        }
    }
    GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/
static int  gtp_get_sensor_id(int TP_ID1, int TP_ID2, int TP_ID3, struct goodix_ts_platform_data *pdata)
{
    int ret, id1 = -1, id2 = -1, id3 = -1;
    strcpy(gt9xx_updata_file_path, GT9XX_UPDATE_FILE_PATH);
    if (TP_ID1  >= 0)
    {
        ret = gpio_request(TP_ID1, "TP_ID1");
        ret = gpio_direction_input(TP_ID1);
    }
    if (TP_ID2  >=  0)
    {
        ret = gpio_request(TP_ID2, "TP_ID2");
        ret = gpio_direction_input(TP_ID2);
    }
    if (TP_ID3  >= 0)
    {
        ret = gpio_request(TP_ID3, "TP_ID3");
        ret = gpio_direction_input(TP_ID3);
    }
    msleep(20);
    if (TP_ID1  >= 0)
    {
        id1 = gpio_get_value(TP_ID1);
    }
    if (TP_ID2  >=  0)
    {
        id2 = gpio_get_value(TP_ID2);
    }
    if (TP_ID3  >= 0)
    {
        id3 = gpio_get_value(TP_ID3);
    }
    pr_err(KERN_EMERG"%s ID1:%d  ID2:%d  ID3:%d\n", __func__, id1, id2, id3);

    if (is_project(OPPO_15109))
    {
        if ((id1 == 0) && (id2 == 0))
        {
            pr_err("%s::TRULY\n", __func__);
            pdata->tp_info.manufacture = "TRULY";
            sensor_id = TP_TRULY;
            glove_check = 0;
        }
        else if ((id1 == 0) && (id2 == 1))
        {
            pr_err("%s::OFILM \n", __func__);
            pdata->tp_info.manufacture = "OFILM";
            sensor_id = TP_OFILM;
            glove_check = 1;
        }
        else if ((id1 == 1) && (id2 == 0))
        {
            pr_err("%s::BIEL\n", __func__);
            pdata->tp_info.manufacture = "BIEL";
            sensor_id = TP_BIEL;
            glove_check = 4;
        }
        else if ((id1 == 1) && (id2 == 1))
        {
            pr_err("%s::TRULY_E \n", __func__);
            pdata->tp_info.manufacture = "TRULY_E";
            sensor_id = TP_TRULY_E;
            glove_check = 3;
        }
        else
        {
            pr_err("%s::TP_UNKNOWN,default:TP_OFILM \n", __func__);
            sensor_id = TP_TRULY;
            pdata->tp_info.manufacture = "TP_UNKNOWN";
            glove_check = 0;
        }
    }

    if (is_project(OPPO_15109))
    {
        switch(sensor_id)
        {
            case TP_TPK:
                strcat(gt9xx_updata_file_path, "tp/15109/15109_GOODIX_UPDATE_TRULY.bin");
                break;
            case TP_OFILM:
            case TP_TRULY:
            default:
                strcat(gt9xx_updata_file_path, "tp/15109/15109_GOODIX_UPDATE_TRULY.bin");
                break;
        }
    }

    pr_err("%s update_path:%s \n", __func__, gt9xx_updata_file_path);
    return 0;
}

s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->fixed_cfg)
    {
        pr_err("Ic fixed config, no config sent!\n");
        return 0;
    }
    else if (ts->pnl_init_error)
    {
        pr_err("Error occured in init_panel, no config sent\n");
        return 0;
    }

    pr_err("Driver send config.\n");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif
    return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1;
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable)
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static int Dot_report_conut = 0;
static void gtp_touch_down(struct goodix_ts_data* ts, s32 id, s32 x, s32 y, s32 w)
{
#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);

    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_key(ts->input_dev,	BTN_TOOL_FINGER, 1);

    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

    if(ts->boot_mode == MSM_BOOT_MODE__RECOVERY)
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);

    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

    if(ts->boot_mode == MSM_BOOT_MODE__RECOVERY)
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);

    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->input_dev);
#endif
    if ( Dot_report_conut == 50 )
    {
        pr_err("<GTP>touch_down, ID:%d, X:%d, Y:%d, W:%d \n", id, x, y, w);
        Dot_report_conut = 0;
    }
    else
    {
        Dot_report_conut++;
    }

}

/*Virtual Keys Setting Start*/
static struct kobject *gtp_properties_kobj;
static struct kobj_attribute qrd_virtual_keys_attr =
{
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *qrd_properties_attrs[] =
{
    &qrd_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group qrd_properties_attr_group =
{
    .attrs = qrd_properties_attrs,
};

/*Virtual Keys Setting End*/

static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
    return sprintf(buf,
                   __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
                   "\n", LCD_WIDTH / 6, button_map[2], button_map[0], button_map[1], LCD_WIDTH / 2, button_map[2], button_map[0], button_map[1], LCD_WIDTH * 5 / 6, button_map[2], button_map[0], button_map[1]);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
    //GTP_DEBUG("Touch id[%2d] release!", id);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif

}

#if GTP_WITH_PEN

static void gtp_pen_init(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    pr_err("Request input device for pen/stylus.\n");

    ts->pen_dev = input_allocate_device();
    if (ts->pen_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device for pen/stylus.\n");
        return;
    }

    ts->pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

#if GTP_ICS_SLOT_REPORT
    input_mt_init_slots(ts->pen_dev, 16);
#else
    ts->pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

    set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, ts->pen_dev->propbit);
    //set_bit(INPUT_PROP_POINTER, ts->pen_dev->propbit);

#if GTP_PEN_HAVE_BUTTON

    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS);
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS2);
#endif

    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    ts->pen_dev->name = "goodix-pen";
    ts->pen_dev->id.bustype = BUS_I2C;

    ret = input_register_device(ts->pen_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->pen_dev->name);
        return;
    }
}

static void gtp_pen_down(s32 x, s32 y, s32 w, s32 id)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

    input_report_key(ts->pen_dev, BTN_TOOL_PEN, 1);
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->pen_dev, id);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
    input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
#else
    input_report_key(ts->pen_dev, BTN_TOUCH, 1);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
    input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->pen_dev);
#endif
    GTP_DEBUG("(%d)(%d, %d)[%d]", id, x, y, w);
}

static void gtp_pen_up(s32 id)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    input_report_key(ts->pen_dev, BTN_TOOL_PEN, 0);

#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->pen_dev, id);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, -1);
#else
    input_report_key(ts->pen_dev, BTN_TOUCH, 0);
#endif

}
#endif

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static u8 palm_lock_flag = 0;   /*[BUGFIX] Add by TCTSZ.WH,2014-5-27,Add palm lock function.*/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
#if GTP_WITH_PEN
    u8 pen_active = 0;
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;

#if GTP_COMPATIBLE_MODE
    u8 rqst_buf[3] = {0x80, 0x43};  // for GT9XXF
#endif

#if GTP_GESTURE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#ifdef GT9XX_SUPPORT_GESTURE
    u8 coordinate_single[258] = {0x94, 0x20};
    u8 coordinate_size[3] = {0x81, 0x4c, 0x00};
    u8 length = 0;
#endif
#endif

    GTP_DEBUG_FUNC();
    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
#if GTP_GESTURE_WAKEUP
    if (DOZE_ENABLED == doze_status)
    {
        ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
        GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
#ifdef GT9XX_SUPPORT_GESTURE
        if (ret > 0)
        {
            if (doze_buf[2] != 0)
            {

                GTP_DEBUG(" gesture = %d\n", gesture);

                //rendong.shi add gesture
                for (i = 2; i < 66; i++)
                {
                    coordinate_single[i] = 0;
                }
                ret = gtp_i2c_read(i2c_connect_client, coordinate_size, 3);
                GTP_INFO("The length of Gesture Coordinate is %d\n", coordinate_size[2]);
                if (doze_buf[2]  != DTAP_DETECT)
                    ret = gtp_i2c_read(i2c_connect_client, coordinate_single, coordinate_size[2] * 4 + 2);
                length = coordinate_size[2];
                switch (doze_buf[2])
                {
                    case DTAP_DETECT:
                        gesture = DouTap;
                        break;

                    case UP_VEE_DETECT :
                        gesture = UpVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[10]  |  (coordinate_single[11] << 8);
                        Point_end.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case DOWN_VEE_DETECT :
                        gesture = DownVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[10]  |  (coordinate_single[11] << 8);
                        Point_end.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case LEFT_VEE_DETECT:
                        gesture =  LeftVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[10]  |  (coordinate_single[11] << 8);
                        Point_end.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case RIGHT_VEE_DETECT :
                        gesture =  RightVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[18]  |  (coordinate_single[19] << 8);
                        Point_end.y = coordinate_single[20] |  (coordinate_single[21] << 8);
                        Point_1st.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_1st.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        break;

                    case CIRCLE_DETECT  :
                        gesture =  Circle;
#if 0
                        j = 0;
                        for (i = 0; i < length; i++)
                        {
                            Point_input[i].x = coordinate_single[j + 2]  |  (coordinate_single[j + 3] << 8);
                            Point_input[i].y = coordinate_single[j + 3] |  (coordinate_single[j + 4] << 8);
                            j = j + 4;
                            GTP_INFO("Point_input[%d].x = %d,Point_input[%d].y = %d\n", i, Point_input[i].x, i, Point_input[i].y)   ;
                        }
                        clock = cross(Point_input[0], Point_input[1], Point_input[2]);
                        GetCirclePoints(&Point_input[0], length, Point_output);
                        Point_start.x = Point_input[0].x;
                        Point_start.y = Point_input[0].y;
                        Point_end.x = Point_input[length - 1].x;
                        Point_end.y = Point_input[length - 1].y;
                        Point_1st.x = Point_output[0].x;
                        Point_1st.y = Point_output[0].y;
                        Point_2nd.x = Point_output[1].x;
                        Point_2nd.y = Point_output[1].y;
                        Point_3rd.x = Point_output[2].x;
                        Point_3rd.y = Point_output[2].y;
                        Point_4th.x = Point_output[3].x;
                        Point_4th.y = Point_output[3].y;
                        if (clock > 0)
                        {
                            clockwise = 1;
                        }
                        else
                        {
                            clockwise = 0;
                        }
#endif
#if 1
                        {
                            int j = 2;
                            //for (i = 2; i < (length * 4 + 2); i++)
                            //{
                                //coordinate_single[i] = 0;
                            //}
                            //ret = gtp_i2c_read(i2c_connect_client, coordinate_size, 3);
                            //length = coordinate_size[2];
                            //GTP_ERROR("coordinate_size[2] = %d\n ", coordinate_size[2]);
                            //ret = gtp_i2c_read(i2c_connect_client, coordinate_single, coordinate_size[2] * 4 + 2);
                            for (i = 2; i < (length + 2); i++)
                            {
                                Point_input[i].x = coordinate_single[j]  |  (coordinate_single[j + 1] << 8);
                                Point_input[i].y = coordinate_single[j + 2] |  (coordinate_single[j + 3] << 8);
                                j = j + 4;
                                printk("Point_input[%d].x = %d,Point_input[%d].y = %d\n", i, Point_input[i].x, i, Point_input[i].y) ;
                            }
                        }
                        GetCirclePoints(&Point_input[2], length , Point_output);
                        if (cross(Point_input[3], Point_input[4], Point_input[5]) > 0)
                            clockwise = 1;
                        else
                            clockwise = 0;
                        Point_start.x = Point_input[2].x;
                        Point_start.y = Point_input[2].y;
                        Point_end.x = Point_input[length + 1].x;
                        Point_end.y = Point_input[length + 1].y;
                        Point_1st.x = Point_output[0].x;
                        Point_1st.y = Point_output[0].y;
                        Point_2nd.x = Point_output[1].x;
                        Point_2nd.y = Point_output[1].y;
                        Point_3rd.x = Point_output[2].x;
                        Point_3rd.y = Point_output[2].y;
                        Point_4th.x = Point_output[3].x;
                        Point_4th.y = Point_output[3].y;
                        for (i = 0; i < 5; i++)
                        {
                            printk("Point_output[%d].x = %d,Point_output[%d].y = %d\n", i, Point_output[i].x, i, Point_output[i].y);
                        }
#else
                        Point_start.x = 0;
                        Point_start.y = 0;
                        Point_end.x = 0;
                        Point_end.y = 0;
                        Point_1st.x = 0;
                        Point_1st.y = 0;
                        Point_2nd.x = 0;
                        Point_2nd.y = 0;
                        Point_3rd.x = 0;
                        Point_3rd.y = 0;
                        Point_4th.x = 0;
                        Point_4th.y = 0;
#endif
                        break;

                    case DOUSWIP_DETECT  :
                        gesture =  DouSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        Point_1st.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_1st.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_2nd.x = coordinate_single[14]  |  (coordinate_single[15] << 8);
                        Point_2nd.y = coordinate_single[16] |  (coordinate_single[17] << 8);
                        break;

                    case RIGHT_SLIDE_DETECT :
                        gesture =  Left2RightSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case LEFT_SLIDE_DETECT :
                        gesture =  Right2LeftSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case DOWN_SLIDE_DETECT  :
                        gesture =  Up2DownSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case UP_SLIDE_DETECT :
                        gesture =  Down2UpSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case M_DETECT  :
                        gesture =  Mgestrue;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[18]  |  (coordinate_single[19] << 8);
                        Point_end.y = coordinate_single[20] |  (coordinate_single[21] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        Point_2nd.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_2nd.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_3rd.x = coordinate_single[14] |  (coordinate_single[15] << 8);
                        Point_3rd.y = coordinate_single[16] |  (coordinate_single[17] << 8);
                        break;

                    case W_DETECT :
                        gesture =  Wgestrue;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[18]  |  (coordinate_single[19] << 8);
                        Point_end.y = coordinate_single[20] |  (coordinate_single[21] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        Point_2nd.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_2nd.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_3rd.x = coordinate_single[14] |  (coordinate_single[15] << 8);
                        Point_3rd.y = coordinate_single[16] |  (coordinate_single[17] << 8);
                        break;

                    default:
                        gesture =  UnkownGestrue;
                        break;


                }

                GTP_INFO("detect %s gesture\n", gesture == DouTap ? "double tap" :
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
                         gesture == Wgestrue ? "(W)" : "oppo custom gesture");

                 pr_err("%s %d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", __func__,  gesture,
                   Point_start.x, Point_start.y, Point_end.x, Point_end.y,
                   Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
                   Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
                   clockwise);

                //report Key to notify
                if (gesture != UnkownGestrue)
                {


                    GTP_INFO("F4 report!!!!!\n");
                    input_report_key(ts->input_dev, KEY_F4, 1);
                    input_sync(ts->input_dev);
                    input_report_key(ts->input_dev, KEY_F4, 0);
                    input_sync(ts->input_dev);
                    //doze_status = DOZE_WAKEUP;
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_connect_client, doze_buf, 3);
                }
            }
            else
            {
                //gtp_enter_doze(i2c_connect_client);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
                gtp_enter_doze(ts);
            }
        }
#else
        if (ret > 0)
        {
            if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
                (doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') ||
                (doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
                (doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') ||
                (doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
                (doze_buf[2] == 0x5E) /* ^ */
               )
            {
                if (doze_buf[2] != 0x5E)
                {
                    pr_err("Wakeup by gesture(%c), light up the screen!\n", doze_buf[2]);
                }
                else
                {
                    pr_err("Wakeup by gesture(^), light up the screen!\n");
                }
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
            }
            else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
                      (doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
            {
                char *direction[4] = {"Right", "Down", "Up", "Left"};
                u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;

                pr_err("%s slide to light up the screen!\n", direction[type]);
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
            }
            else if (0xC0 == (doze_buf[2] & 0xC0))
            {
                pr_err("Double click to light up the screen!\n");
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
            }
            else
            {
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
                gtp_enter_doze(ts);
            }
        }
#endif
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }
#endif

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }

    finger = point_data[GTP_ADDR_LENGTH];
    /*[BUGFIX] Add Begin by TCTSZ.WH,2014-5-27,Add palm lock function.*/
#if GTP_LARGE_AREA_SLEEP
    /*ADD Begin by TCTSZ-weihong.chen,2014-6-03,Add debug interface for input devices.*/
#ifdef CONFIG_TCT_8X16_POP8LTE
    if ((finger & (1 << 6)) && (0 == palm_lock_flag) && g_wakeup_gesture)
#else
    if ((finger & (1 << 6)) && (0 == palm_lock_flag))
#endif
        /*ADD Begin by TCTSZ-weihong.chen,2014-6-03,Add debug interface for input devices.*/
    {
        pr_err("<2>""large range detected!!!  %s,%d,finger = %x\n", __func__, __LINE__, finger);
        input_report_key(ts->input_dev, KEY_POWER, 1);
        input_report_key(ts->input_dev, KEY_POWER, 0);
        input_sync(ts->input_dev);
        palm_lock_flag = 1;
    }
#endif
    /*[BUGFIX] Add End by TCTSZ.WH,2014-5-27,Add palm lock function.*/
#if GTP_COMPATIBLE_MODE
    // GT9XXF
    if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type))     // request arrived
    {
        ret = gtp_i2c_read(ts->client, rqst_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("Read request status error!");
            goto exit_work_func;
        }

        switch (rqst_buf[2])
        {
            case GTP_RQST_CONFIG:
                pr_err("Request for config.\n");
                ret = gtp_send_cfg(ts->client);
                if (ret < 0)
                {
                    GTP_ERROR("Request for config unresponded!\n");
                }
                else
                {
                    rqst_buf[2] = GTP_RQST_RESPONDED;
                    gtp_i2c_write(ts->client, rqst_buf, 3);
                    pr_err("Request for config responded!\n");
                }
                break;

            case GTP_RQST_BAK_REF:
                pr_err("Request for backup reference.\n");
                ts->rqst_processing = 1;
                ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
                if (SUCCESS == ret)
                {
                    rqst_buf[2] = GTP_RQST_RESPONDED;
                    gtp_i2c_write(ts->client, rqst_buf, 3);
                    ts->rqst_processing = 0;
                    pr_err("Request for backup reference responded!\n");
                }
                else
                {
                    GTP_ERROR("Requeset for backup reference unresponed!\n");
                }
                break;

            case GTP_RQST_RESET:
                pr_err("Request for reset.\n");
                gtp_recovery_reset(ts->client);
                break;

            case GTP_RQST_MAIN_CLOCK:
                pr_err("Request for main clock.\n");
                ts->rqst_processing = 1;
                ret = gtp_main_clk_proc(ts);
                if (FAIL == ret)
                {
                    GTP_ERROR("Request for main clock unresponded!\n");
                }
                else
                {
                    pr_err("Request for main clock responded!\n");
                    rqst_buf[2] = GTP_RQST_RESPONDED;
                    gtp_i2c_write(ts->client, rqst_buf, 3);
                    ts->rqst_processing = 0;
                    ts->clk_chk_fs_times = 0;
                }
                break;

            default:
                pr_err("Undefined request: 0x%02X\n", rqst_buf[2]);
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                break;
        }
    }
#endif
    if (finger == 0x00)
    {
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }

    if ((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
    key_value = point_data[3 + 8 * touch_num];

    if (key_value || pre_key)
    {
#if GTP_PEN_HAVE_BUTTON
        if (key_value == 0x40)
        {
            GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.\n");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else if (key_value == 0x10)
        {
            GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.\n");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            pen_active = 1;
        }
        else if (key_value == 0x20)
        {
            GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.\n");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else
        {
            GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.\n");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            if ( (pre_key == 0x40) || (pre_key == 0x20) ||
                 (pre_key == 0x10)
               )
            {
                pen_active = 1;
            }
        }
        if (pen_active)
        {
            touch_num = 0;      // shield pen point
            //pre_touch = 0;    // clear last pen status
        }
#endif

#if GTP_HAVE_TOUCH_KEY
        if (!pre_touch)
        {
            for (i = 0; i < GTP_MAX_KEY_NUM; i++)
            {
#if 0 //#if GTP_DEBUG_ON
                for (ret = 0; ret < 4; ++ret)
                {
                    if (key_codes[ret] == touch_key_array[i])
                    {
                        GTP_DEBUG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up \n");
                        break;
                    }
                }
#endif
                input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01 << i));
            }
            touch_num = 0;  // shield fingers
        }
#endif
    }
#endif
    pre_key = key_value;

    //GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
    if (pre_pen && (touch_num == 0))
    {
        GTP_DEBUG("Pen touch UP(Slot)!");
        gtp_pen_up(0);
        pen_active = 1;
        pre_pen = 0;
    }
#endif
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;
        u8 report_num = 0;
        coor_data = &point_data[3];

        if (touch_num)
        {
            id = coor_data[pos] & 0x0F;

#if GTP_WITH_PEN
            id = coor_data[pos];
            if ((id & 0x80))
            {
                GTP_DEBUG("Pen touch DOWN(Slot)!");
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                gtp_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pre_touch = 0;
                pen_active = 1;
            }
#endif

            touch_index |= (0x01 << id);
        }

        //GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",id, touch_index,pre_touch);
        for (i = 0; i < GTP_MAX_TOUCH; i++)
        {
#if GTP_WITH_PEN
            if (pre_pen == 1)
            {
                break;
            }
#endif

            if ((touch_index & (0x01 << i)))
            {
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);
//[PLATFORM]-Add-BEGIN by TCTSZ.yaohui.zeng, 2014/04/11, Add for calibratation
#ifdef CONFIG_TCT_8X16_POP8LTE
                input_x = 800 - input_x;
                input_y = 1280 - input_y;
#endif
//[PLATFORM]-Add-END by TCTSZ.yaohui.zeng, 2014/04/11
                gtp_touch_down(ts, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;

                report_num++;
                if (report_num < touch_num)
                {
                    pos += 8;
                    id = coor_data[pos] & 0x0F;
                    touch_index |= (0x01 << id);
                }
            }
            else
            {
                gtp_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }

        if (!pre_touch)   // when all up
        {
            input_report_key(ts->input_dev, BTN_TOUCH, 0);
	     input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
            //pr_err("%s all up.\n", __func__);
        }

    }
#else

    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | (coor_data[2] << 8);
            input_y  = coor_data[3] | (coor_data[4] << 8);
            input_w  = coor_data[5] | (coor_data[6] << 8);

#if GTP_WITH_PEN
            id = coor_data[0];
            if (id & 0x80)
            {
                GTP_DEBUG("Pen touch DOWN!");
                gtp_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pen_active = 1;
                break;
            }
            else
#endif
            {
                gtp_touch_down(ts, id, input_x, input_y, input_w);
            }
        }
    }
    else if (pre_touch)
    {
#if GTP_WITH_PEN
        if (pre_pen == 1)
        {
            GTP_DEBUG("Pen touch UP!");
            gtp_pen_up(0);
            pre_pen = 0;
            pen_active = 1;
        }
        else
#endif
        {
            GTP_DEBUG("Touch Release!");
            gtp_touch_up(ts, 0);
        }
    }

    pre_touch = touch_num;
#endif

#if GTP_WITH_PEN
    if (pen_active)
    {
        pen_active = 0;
        input_sync(ts->pen_dev);
    }
    else
#endif
    {
        input_sync(ts->input_dev);
    }

exit_work_func:
    if (!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            pr_err("I2C write end_cmd error!\n");
        }
    }
    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode.
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();
    gtp_irq_disable(ts);
    queue_work(goodix_wq, &ts->work);
    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(struct goodix_ts_data *ts, s32 ms)
{
    gpio_direction_output(ts->pdata->irq_gpio, 0);
    msleep(ms);
    gpio_direction_input(ts->pdata->irq_gpio);
}


/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
#if 1
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    mutex_lock(&ts->rst_lock);

    pr_err(">>-- %s line:%d \n", __func__, __LINE__);
    /* This reset sequence will selcet I2C slave address */
    gpio_direction_output(ts->pdata->reset_gpio, 0);
    msleep(ms);

    if (ts->client->addr == GTP_I2C_ADDRESS_HIGH)
        gpio_direction_output(ts->pdata->irq_gpio, 1);
    else
        gpio_direction_output(ts->pdata->irq_gpio, 0);

    usleep(RESET_DELAY_T3_US);
    gpio_direction_output(ts->pdata->reset_gpio, 1);
    msleep(RESET_DELAY_T4);

    // gpio_direction_input(ts->pdata->reset_gpio);  // delete by lauson

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        mutex_unlock(&ts->rst_lock);
        return;
    }
#endif
    gtp_int_sync(ts, 50);
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(ts->client);
#endif
    gt9xx_free_fingers(ts);

    mutex_unlock(&ts->rst_lock);
}
#else
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
#if GTP_COMPATIBLE_MODE
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

    GTP_DEBUG_FUNC();
    pr_err("Guitar reset\n");
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   // begin select I2C slave addr
    msleep(ms);                         // T2: > 10ms
    // HIGH: 0x28/0x29, LOW: 0xBA/0xBB
    GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, client->addr == 0x14);

    msleep(2);                          // T3: > 100us
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

    msleep(6);                          // T4: > 5ms

    GTP_GPIO_AS_INPUT(GTP_RST_PORT);    // end select I2C slave addr

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        return;
    }
#endif

    gtp_int_sync(ts, 50);
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(client);
#endif
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

    GTP_DEBUG("Entering gesture mode.\n");
    while (retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GTP_DEBUG("failed to set doze flag into 0x8046, %d\n", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            pr_err("Gesture mode enabled.\n");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send gesture cmd failed.\n");
    return ret;
}

/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
    u8 status_buf[3] = {0x80, 0x44};
#endif

    GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // GT9XXF: host interact with ic
        ret = gtp_i2c_read(ts->client, status_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("failed to get backup-reference status");
        }

        if (status_buf[2] & 0x80)
        {
            ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
            if (FAIL == ret)
            {
                GTP_ERROR("failed to store bak_ref");
            }
        }
    }
#endif

    GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 0);
    msleep(5);

    while (retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            pr_err("GTP enter sleep!");

            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.\n");
    return ret;
}
#endif
/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 opr_buf[3] = {0x41, 0x80};

        GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 1);
        msleep(5);

        for (retry = 0; retry < 10; ++retry)
        {
            // hold ss51 & dsp
            opr_buf[2] = 0x0C;
            ret = gtp_i2c_write(ts->client, opr_buf, 3);
            if (FAIL == ret)
            {
                GTP_ERROR("failed to hold ss51 & dsp!");
                continue;
            }
            opr_buf[2] = 0x00;
            ret = gtp_i2c_read(ts->client, opr_buf, 3);
            if (FAIL == ret)
            {
                GTP_ERROR("failed to get ss51 & dsp status!");
                continue;
            }
            if (0x0C != opr_buf[2])
            {
                GTP_DEBUG("ss51 & dsp not been hold, %d\n", retry + 1);
                continue;
            }
            GTP_DEBUG("ss51 & dsp confirmed hold");

            ret = gtp_fw_startup(ts->client);
            if (FAIL == ret)
            {
                GTP_ERROR("failed to startup GT9XXF, process recovery");
                gtp_esd_recovery(ts->client);
            }
            break;
        }
        if (retry >= 10)
        {
            GTP_ERROR("failed to wakeup, processing esd recovery");
            gtp_esd_recovery(ts->client);
        }
        else
        {
            pr_err("GT9XXF gtp wakeup success");
        }
        return ret;
    }
#endif

#if GTP_POWER_CTRL_SLEEP
    while (retry++ < 5)
    {
        gtp_reset_guitar(ts->client, 20);

        pr_err("GTP wakeup sleep.\n");
        return 1;
    }
#else
    while (retry++ < 10)
    {
#if GTP_GESTURE_WAKEUP
        if (DOZE_WAKEUP != doze_status)
        {
            pr_err("Powerkey wakeup.\n");
        }
        else
        {
            pr_err("Gesture wakeup.\n");
        }
        doze_status = DOZE_DISABLED;
        gtp_irq_disable(ts);
        gtp_reset_guitar(ts->client, 10);
        gtp_irq_enable(ts);

#else
        GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 1);
        msleep(5);
#endif

        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            pr_err("GTP wakeup sleep.\n");

#if (!GTP_GESTURE_WAKEUP)
            {
                gtp_int_sync(ts, 25);
#if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
#endif
            }
#endif

            return ret;
        }
        gtp_reset_guitar(ts->client, 20);
//        gtp_reset_guitar(ts, 20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.\n");
    return ret;
}
#if GTP_DRIVER_SEND_CFG
#if 0 /* delete it by lauson. */
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
    u8 opr_buf[6] = {0};
    s32 ret = 0;

    ts->abs_x_max = GTP_MAX_WIDTH;
    ts->abs_y_max = GTP_MAX_HEIGHT;
    ts->int_trigger_type = GTP_INT_TRIGGER;

    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA + 1) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA + 1) & 0xFF);

    ret = gtp_i2c_read(ts->client, opr_buf, 6);
    if (ret < 0)
    {
        return FAIL;
    }

    ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA + 6) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA + 6) & 0xFF);

    ret = gtp_i2c_read(ts->client, opr_buf, 3);
    if (ret < 0)
    {
        return FAIL;
    }
    ts->int_trigger_type = opr_buf[2] & 0x03;

    pr_err("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
           ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);

    return SUCCESS;
}
#endif
#endif

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
u8 driver_num = 0;
u8 sensor_num = 0;
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    u8 drv_cfg_version;
    u8 flash_cfg_version;

    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 cfg_info_group6[] = CTP_CFG_GROUP6;

    u8 cfg_info1_group1[] = CTP_CFG1_GROUP1;
    u8 cfg_info1_group2[] = CTP_CFG1_GROUP2;
    u8 cfg_info1_group3[] = CTP_CFG1_GROUP3;
    u8 cfg_info1_group4[] = CTP_CFG1_GROUP4;
    u8 cfg_info1_group5[] = CTP_CFG1_GROUP5;
    u8 cfg_info1_group6[] = CTP_CFG1_GROUP6;

    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                          cfg_info_group4, cfg_info_group5, cfg_info_group6
                         };
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5),
                          CFG_GROUP_LEN(cfg_info_group6)
                        };

    if (is_project(OPPO_15109))
    {
			send_cfg_buf[0] = cfg_info1_group1;
			send_cfg_buf[1] = cfg_info1_group2;
			send_cfg_buf[2] = cfg_info1_group3;
			send_cfg_buf[3] = cfg_info1_group4;
			send_cfg_buf[4] = cfg_info1_group5;
			send_cfg_buf[5] = cfg_info1_group6;

			cfg_info_len[0] = CFG_GROUP_LEN(cfg_info1_group1);
			cfg_info_len[1] = CFG_GROUP_LEN(cfg_info1_group2);
			cfg_info_len[2] = CFG_GROUP_LEN(cfg_info1_group3);
			cfg_info_len[3] = CFG_GROUP_LEN(cfg_info1_group4);
			cfg_info_len[4] = CFG_GROUP_LEN(cfg_info1_group5);
			cfg_info_len[5] = CFG_GROUP_LEN(cfg_info1_group6);

    }

    GTP_DEBUG_FUNC();
    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d\n",
              cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
              cfg_info_len[4], cfg_info_len[5]);

    pr_err("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d\n",
           cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
           cfg_info_len[4], cfg_info_len[5]);

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ts->fw_error = 0;
    }
    else
#endif
    {
        ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
        if (SUCCESS == ret)
        {
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                GTP_ERROR("Firmware error, no config sent!");
                return -1;
            }
        }
    }

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
        (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
        (!cfg_info_len[5]))
    {
        sensor_id = 0;
    }
    else
    {
#if GTP_COMPATIBLE_MODE
        msleep(50);
#endif
        if (sensor_id >= 0x06)
        {
            GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
            ts->pnl_init_error = 1;
            return -1;
        }
        pr_err("Sensor_ID: %d\n", sensor_id);
    }
    ts->gtp_cfg_len = cfg_info_len[sensor_id];
    pr_err("CTP_CONFIG_GROUP%d used, config length: %d\n", sensor_id + 1, ts->gtp_cfg_len);
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);

    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id + 1, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }

    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);

    if (ret == SUCCESS)
    {
        GTP_DEBUG("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id + 1,
                  send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);

        flash_cfg_version = opr_buf[0];
        drv_cfg_version = config[GTP_ADDR_LENGTH];

        if ( flash_cfg_version > drv_cfg_version)
        {
            config[GTP_ADDR_LENGTH] = 0x00;
        }
    }
    else
    {
        GTP_ERROR("Failed to get ic config version!No config sent!");
        return -1;
    }



#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH >> 8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT >> 8);

    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe;
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }

#endif // GTP_DRIVER_SEND_CFG

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 sensor_num = 0;
        u8 driver_num = 0;
        u8 have_key = 0;

        have_key = (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);

        if (1 == ts->is_950)
        {
            driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
            if (have_key)
            {
                driver_num--;
            }
            ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
        }
        else
        {
            driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM] & 0x1F);
            if (have_key)
            {
                driver_num--;
            }
            sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
            ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
        }

        pr_err("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
               driver_num, sensor_num, have_key, ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);
        return 0;
    }
    else
#endif
    {
#if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            GTP_ERROR("Send config error.\n");
        }
        // set config version to CTP_CFG_GROUP, for resume to send config
//        config[GTP_ADDR_LENGTH] = grp_cfg_version;
        if ( flash_cfg_version > drv_cfg_version)
        {
            check_sum = 0;
            config[GTP_ADDR_LENGTH] = drv_cfg_version;
            for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
            {
                check_sum += config[i];
            }
            config[ts->gtp_cfg_len] = (~check_sum) + 1;
            msleep(300);
            gtp_send_cfg(ts->client);
        }

#endif
        pr_err("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);
    }
    msleep(10);

    /*[BUGFIX] ADD begin by TCTSZ-WH,2014-5-14,Show TP config version*/
//  ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA,
//          &grp_cfg_version, 1);
//  if(SUCCESS != ret)
//  {
//       GTP_ERROR("Failed to get ic config version!");
//  }
    /*[BUGFIX] ADD begin by TCTSZ-WH,2014-5-14,Show TP config version*/

//  g_tp_cfg_ver = grp_cfg_version;
    pr_err("gtp_init_panel end define = %d\n", GTP_DRIVER_SEND_CFG);
    return 0;
}

#ifdef  GT9XX_SUPPORT_GLOVES_MODE
static s32 gtp_init_glovemode_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    //u8 sensor_id = 0;
	  u8 drv_cfg_version;
	  u8 flash_cfg_version;
	  u8 cfg_info_group0[] = CTP_CFG_GROUP_GLOVE0;
    u8 cfg_info_group1[] = CTP_CFG_GROUP_GLOVE1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP_GLOVE2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP_GLOVE3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP_GLOVE4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP_GLOVE5;

    u8 cfg_info1_group0[] = CTP_CFG1_GROUP_GLOVE0;
    u8 cfg_info1_group1[] = CTP_CFG1_GROUP_GLOVE1;
    u8 cfg_info1_group2[] = CTP_CFG1_GROUP_GLOVE2;
    u8 cfg_info1_group3[] = CTP_CFG1_GROUP_GLOVE3;
    u8 cfg_info1_group4[] = CTP_CFG1_GROUP_GLOVE4;
    u8 cfg_info1_group5[] = CTP_CFG1_GROUP_GLOVE5;

    u8 *send_cfg_buf[] = {cfg_info_group0, cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group0),
						  CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5)};

    if (is_project(OPPO_15109))
    {
        send_cfg_buf[0] = cfg_info1_group0;
        send_cfg_buf[1] = cfg_info1_group1;
        send_cfg_buf[2] = cfg_info1_group2;
        send_cfg_buf[3] = cfg_info1_group3;
        send_cfg_buf[4] = cfg_info1_group4;
        send_cfg_buf[5] = cfg_info1_group5;

        cfg_info_len[0] = CFG_GROUP_LEN(cfg_info1_group0);
        cfg_info_len[1] = CFG_GROUP_LEN(cfg_info1_group1);
        cfg_info_len[2] = CFG_GROUP_LEN(cfg_info1_group2);
        cfg_info_len[3] = CFG_GROUP_LEN(cfg_info1_group3);
        cfg_info_len[4] = CFG_GROUP_LEN(cfg_info1_group4);
        cfg_info_len[5] = CFG_GROUP_LEN(cfg_info1_group5);
    }


    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);
        ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
        if (SUCCESS == ret)
        {
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                GTP_ERROR("Firmware error, no config sent!");
                return -1;
            }
        }

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
        (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
        (!cfg_info_len[5]))
    {
        sensor_id = 0;
    }
    else
    {
        if (sensor_id >= 0x06)
        {
            GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
            ts->pnl_init_error = 1;
            return -1;
        }
     }

    GTP_INFO("Sensor_ID: %d", sensor_id);
	  GTP_DEBUG("Get config data from header file.");

	ts->gtp_cfg_len = cfg_info_len[sensor_id];
	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);
  GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id, ts->gtp_cfg_len);
  if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
  {
        GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
  }
  ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
  if (ret == SUCCESS) {
      GTP_DEBUG("Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X",
                  config[GTP_ADDR_LENGTH], config[GTP_ADDR_LENGTH], opr_buf[0], opr_buf[0]);

	flash_cfg_version = opr_buf[0];
	drv_cfg_version = config[GTP_ADDR_LENGTH];
	if ( flash_cfg_version > drv_cfg_version) {
          config[GTP_ADDR_LENGTH] = 0x00;
      }
    }

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe;
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;
    GTP_INFO("Recalculate checksum is %x\n",config[ts->gtp_cfg_len]);

#else // driver not send config

    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }

#endif // GTP_DRIVER_SEND_CFG

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 sensor_num = 0;
        u8 driver_num = 0;
        u8 have_key = 0;

        have_key = (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);

        if (1 == ts->is_950)
        {
            driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
            if (have_key)
            {
                driver_num--;
            }
            ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
        }
        else
        {
            driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
            if (have_key)
            {
                driver_num--;
            }
            sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
            ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
        }

        GTP_INFO("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
           driver_num, sensor_num, have_key, ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
	return 0;
    }
    else
#endif
    {
#if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            GTP_ERROR("Send config error.");
        }
		if ( flash_cfg_version > drv_cfg_version) {
			check_sum = 0;
	        config[GTP_ADDR_LENGTH] = drv_cfg_version;
			for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
				check_sum += config[i];
			}
			config[ts->gtp_cfg_len] = (~check_sum) + 1;
			msleep(300);
			gtp_send_cfg(ts->client);
		}

#endif
        GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    }
    msleep(10);
	return 0;
}
#endif



static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
    int i;

    if (*ppos)
    {
        return 0;
    }
    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;

    GTP_DEBUG("write count %zd\n", count);

    if (count > GTP_CONFIG_MAX_LENGTH)
    {
        GTP_ERROR("size not match [%d:%zd]\n", GTP_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&config[2], buffer, count))
    {
        GTP_ERROR("copy from user fail\n");
        return -EFAULT;
    }

    ret = gtp_send_cfg(i2c_connect_client);

    if (ret < 0)
    {
        GTP_ERROR("send config failed.\n");
    }

    return count;
}

#ifdef GT9XX_SUPPORT_GLOVES_MODE
static ssize_t tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
    char page[PAGESIZE];
    printk("glove mode enable is: %d\n", atomic_read(&glove_enable));
    ret = sprintf(page, "%d\n", atomic_read(&glove_enable));
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static ssize_t tp_glove_write_func(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    //struct gt9xx_ts_data *ts;
    int ret = 0 ;
    char buf[10] = {0};
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    if (count > 10)
        return count;

    if (ts->gtp_is_suspend == 1)
        return count;
    if (copy_from_user( buf, buffer, count))
    {
        printk(KERN_INFO "%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%d", &ret);

    //ts = ts_g;

    mutex_lock(&ts->io_lock);

    GTP_DEBUG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
    if (atomic_read(&glove_enable) == ret)
    {
        pr_err("%s not set the mode when it is same. \n", __func__);
        mutex_unlock(&ts->io_lock);
        return count;
    }
    if ((ret == 0 ) || (ret == 1) )
    {
        //  printk("tp_glove_write_func called\n");

        atomic_set(&glove_enable, ret);

        //gt9xx_glove_mode_enable(ts);

    }

    switch (ret)
    {
		case 0:
			GTP_DEBUG("glove mode  will be disable\n");
			gtp_init_panel(ts);
			msleep(300);
			break;

		case 1:
			GTP_DEBUG(" glove mode will be enable\n");
			gtp_init_glovemode_panel(ts);
			msleep(300);
			break;
        default:
            GTP_DEBUG("Please enter 0 or 1 to open or close the glove function\n");
    }
    mutex_unlock(&ts->io_lock);
    return count;
}
#endif

#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static ssize_t tp_glove_check_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
    char page[PAGESIZE];
    printk("glove check is: %d\n", glove_check);
    ret = sprintf(page, "%d\n", glove_check);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

#if 0 /* delete it by lauson. */
static int tp_glove_check_write_func(struct file *file, const char *buffer, unsigned long count, void *data)
{
    /*
     int len = 0;
    glove_check =1;
    printk("glove check is: %d\n", glove_check);
    len = sprintf(page, "%d\n", atomic_read(&glove_check));

    return len;
    */
}
#endif

#endif

#ifdef GT9XX_SUPPORT_GESTURE

static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
    char page[PAGESIZE];
    printk("double tap enable is: %d\n", atomic_read(&double_enable));
    ret = sprintf(page, "%d\n", atomic_read(&double_enable));
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static ssize_t tp_double_write_func(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    int ret = 0;

    char buf[10] = {0};
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    if (count > 10)
        return count;

    if (copy_from_user( buf, buffer, count))
    {
        printk(KERN_INFO "%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%d", &ret);

    mutex_lock(&ts->io_lock);

    GTP_DEBUG("tp_double_write_func:buf = %d,ret = %d\n", *buf, ret);
    if (((ret == 0 ) || (ret == 1)) && atomic_read(&double_enable) != ret)
    {
        atomic_set(&double_enable, ret);
    }
    else
    {
        mutex_unlock(&ts->io_lock);
        return count;
    }

    if (ts->gtp_is_suspend)
    {
        switch (ret)
        {
            case 0:
                GTP_DEBUG("tp_guesture_func will be disable\n");
                doze_status = DOZE_DISABLED;
                disable_irq_wake(ts->client->irq);
                gtp_irq_disable(ts);
                gtp_reset_guitar(ts->client, 20);
                gtp_enter_sleep(ts);
                msleep(58);
                break;
            case 1:
                GTP_DEBUG("tp_guesture_func will be enable\n");
                doze_status = DOZE_DISABLED;
                gtp_irq_disable(ts);
                gtp_reset_guitar(ts->client, 20);
                gtp_enter_doze(ts);
                gtp_irq_enable(ts);
                enable_irq_wake(ts->client->irq);
                msleep(58);
                break;
            default:
                GTP_DEBUG("Please enter 0 or 1 to open or close the double-tap function\n");
        }
    }

    mutex_unlock(&ts->io_lock);

    return count;
}

#ifdef GT9XX_SUPPORT_REPORT_COORDINATE
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE];
    pr_err("%s enter.\n", __func__);
    ret =  sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture,
                   Point_start.x, Point_start.y, Point_end.x, Point_end.y,
                   Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
                   Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
                   clockwise);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}
#endif

#endif


#ifdef GT9XX_SUPPORT_GESTURE
static const struct file_operations tp_double_proc_fops =
{
    .write = tp_double_write_func,
    .read =  tp_double_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};

#ifdef GT9XX_SUPPORT_REPORT_COORDINATE
static const struct file_operations coordinate_proc_fops =
{
    .read =  coordinate_proc_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif

#endif

#ifdef GT9XX_SUPPORT_GLOVES_MODE
static const struct file_operations glove_mode_enable_proc_fops =
{
    .write = tp_glove_write_func,
    .read =  tp_glove_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif
#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static const struct file_operations glove_check_proc_fops =
{
    //.write = tp_glove_write_func,
    .read =  tp_glove_check_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif


/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    if (buf[5] == 0x00)
    {
        GTP_DEBUG("IC Version: %c%c%c_%02x%02x\n", buf[2], buf[3], buf[4], buf[7], buf[6]);
        pr_err("IC Version: %c%c%c_%02x%02x\n", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_DEBUG("IC Version: %c%c%c%c_%02x%02x\n", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
        pr_err("IC Version: %c%c%c%c_%02x%02x\n", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
    Read firmware version
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
    0: succeed, otherwise: failed
*******************************************************/
static int gtp_read_fw_version(struct i2c_client *client, u16 *version)
{
    int ret = 0;
    u8 buf[GTP_FW_VERSION_BUFFER_MAXSIZE] =
    {
        GTP_REG_FW_VERSION >> 8, GTP_REG_FW_VERSION & 0xff
    };

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        dev_err(&client->dev, "GTP read version failed.\n");
        return -EIO;
    }

    if (version)
        *version = (buf[3] << 8) | buf[2];

    return ret;
}
/*******************************************************
Function:
    Read and check chip id.
Input:
    client:  i2c device
Output:
    read operation return.
    0: succeed, otherwise: failed
*******************************************************/
static int gtp_check_product_id(struct i2c_client *client)
{
    int ret = 0;
    char product_id[GTP_PRODUCT_ID_MAXSIZE];
    //struct goodix_ts_data *ts = i2c_get_clientdata(client);
    /* 04 bytes are used for the Product-id in the register space.*/
    u8 buf[GTP_PRODUCT_ID_BUFFER_MAXSIZE] =
    {
        GTP_REG_PRODUCT_ID >> 8, GTP_REG_PRODUCT_ID & 0xff
    };

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        dev_err(&client->dev, "GTP read product_id failed.\n");
        return -EIO;
    }

    if (buf[5] == 0x00)
    {
        /* copy (GTP_PRODUCT_ID_MAXSIZE - 1) from buffer. Ex: 915 */
        strlcpy(product_id, &buf[2], GTP_PRODUCT_ID_MAXSIZE - 1);
    }
    else
    {
        if (buf[5] == 'S' || buf[5] == 's')
            chip_gt9xxs = 1;
        /* copy GTP_PRODUCT_ID_MAXSIZE from buffer. Ex: 915s */
        strlcpy(product_id, &buf[2], GTP_PRODUCT_ID_MAXSIZE);
    }

    dev_err(&client->dev, "Goodix Product ID = %s\n", product_id);

#if 0 /* delete it by lauson. */
    ret = strcmp(product_id, ts->pdata->product_id);
    if (ret != 0)
        return -EINVAL;
#endif

    return 0;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

    while (retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.", retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
#if 1
static int gtp_request_io_port(struct goodix_ts_data *ts)
{
    struct i2c_client *client = ts->client;
    struct goodix_ts_platform_data *pdata = ts->pdata;
    int ret;
    if (gpio_is_valid(pdata->irq_gpio))
    {
        ret = gpio_request(pdata->irq_gpio, "goodix_ts_irq_gpio");
        if (ret)
        {
            dev_err(&client->dev, "Unable to request irq gpio [%d]\n",
                    pdata->irq_gpio);
            goto err_pwr_off;
        }
        ret = gpio_direction_input(pdata->irq_gpio);
        if (ret)
        {
            dev_err(&client->dev, "Unable to set direction for irq gpio [%d]\n",
                    pdata->irq_gpio);
            goto err_free_irq_gpio;
        }
    }
    else
    {
        dev_err(&client->dev, "Invalid irq gpio [%d]!\n",
                pdata->irq_gpio);
        ret = -EINVAL;
        goto err_pwr_off;
    }

    if (gpio_is_valid(pdata->reset_gpio))
    {
        ret = gpio_request(pdata->reset_gpio, "goodix_ts_reset_gpio");
        if (ret)
        {
            dev_err(&client->dev, "Unable to request reset gpio [%d]\n",
                    pdata->reset_gpio);
            goto err_free_irq_gpio;
        }
        ret = gpio_direction_output(pdata->reset_gpio, 0);
        if (ret)
        {
            dev_err(&client->dev, "Unable to set direction for reset gpio [%d]\n",
                    pdata->reset_gpio);
            goto err_free_reset_gpio;
        }
    }
    else
    {
        dev_err(&client->dev, "Invalid irq gpio [%d]!\n",
                pdata->reset_gpio);
        ret = -EINVAL;
        goto err_free_irq_gpio;
    }
    /* IRQ GPIO is an input signal, but we are setting it to output
      * direction and pulling it down, to comply with power up timing
      * requirements, mentioned in power up timing section of device
      * datasheet.
      */
    ret = gpio_direction_output(pdata->irq_gpio, 0);
    if (ret)
        dev_warn(&client->dev,
                 "pull down interrupt gpio failed\n");
    ret = gpio_direction_output(pdata->reset_gpio, 0);
    if (ret)
        dev_warn(&client->dev,
                 "pull down reset gpio failed\n");
	//gtp_reset_guitar(client, 20);  // delete by lauson

    return ret;

err_free_reset_gpio:
    if (gpio_is_valid(pdata->reset_gpio))
    {
        gpio_free(pdata->reset_gpio);
    }
err_free_irq_gpio:
    if (gpio_is_valid(pdata->irq_gpio))
    {
        gpio_free(pdata->irq_gpio);
    }
err_pwr_off:
    return ret;
}
#else
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    GTP_DEBUG_FUNC();
    ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
    if (ret < 0)
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d\n", (s32)GTP_INT_PORT, ret);
        ret = -ENODEV;
    }
    else
    {
        GTP_GPIO_AS_INT(GTP_INT_PORT);
        ts->client->irq = GTP_INT_IRQ;
    }

    ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
    if (ret < 0)
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d\n", (s32)GTP_RST_PORT, ret);
        ret = -ENODEV;
    }

    GTP_GPIO_AS_INPUT(GTP_RST_PORT);

    gtp_reset_guitar(ts->client, 20);

    if (ret < 0)
    {
        GTP_GPIO_FREE(GTP_RST_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);
    }

    return ret;
}
#endif
/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    const u8 irq_table[] = GTP_IRQ_TAB;

    GTP_DEBUG_FUNC();
    pr_err("INT trigger type:%x\n", ts->int_trigger_type);

    ret  = request_irq(ts->client->irq,
                       goodix_ts_irq_handler,
//                       irq_table[ts->int_trigger_type],
                       irq_table[ts->int_trigger_type] | IRQF_ONESHOT,
                       ts->client->name,
                       ts);
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        gpio_direction_input(ts->pdata->irq_gpio);
        gpio_free(ts->pdata->irq_gpio);
        ts->use_irq = 0;
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else
    {
        gtp_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif

    GTP_DEBUG_FUNC();

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.\n");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GTP_ICS_SLOT_REPORT
//    input_mt_init_slots(ts->input_dev, 16);     // in case of "out of memory"
    input_mt_init_slots(ts->input_dev, 16, 0);
#else
    input_mt_init_slots(ts->input_dev, 10, 0);
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

    set_bit(BTN_TOUCH, ts->input_dev->keybit);

#if GTP_HAVE_TOUCH_KEY
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);
    }
#endif

#if GTP_GESTURE_WAKEUP
    input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
    set_bit(KEY_F4 , ts->input_dev->keybit);
#endif

#if GTP_CHANGE_X2Y
    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, LCD_WIDTH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, LCD_HEIGHT, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    sprintf(phys, "input/ts");
    ts->input_dev->name = TPD_DEVICE;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;

    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }

#if defined(CONFIG_FB)
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret)
        dev_err(&ts->client->dev,
                "Unable to register fb_notifier: %d\n",
                ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = goodix_ts_early_suspend;
    ts->early_suspend.resume = goodix_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

#if GTP_WITH_PEN
    gtp_pen_init(ts);
#endif

    gtp_properties_kobj = kobject_create_and_add("board_properties", NULL);
    if ( gtp_properties_kobj )
        ret = sysfs_create_group(gtp_properties_kobj, &qrd_properties_attr_group);
    if ( !gtp_properties_kobj || ret )
        pr_err("failed to create board_properties\n");

    return 0;
}

//************** For GT9XXF Start *************//
#if GTP_COMPATIBLE_MODE

s32 gtp_fw_startup(struct i2c_client *client)
{
    u8 opr_buf[4];
    s32 ret = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    //init sw WDT
    opr_buf[0] = 0xAA;
    ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }

    //release SS51 & DSP
    opr_buf[0] = 0x00;
    ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    //int sync
    gtp_int_sync(ts, 25);

    //check fw run status
    ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    if (0xAA == opr_buf[0])
    {
        GTP_ERROR("IC works abnormally,startup failed.\n");
        return FAIL;
    }
    else
    {
        pr_err("IC works normally, Startup success.\n");
        opr_buf[0] = 0xAA;
        i2c_write_bytes(client, 0x8041, opr_buf, 1);
        return SUCCESS;
    }
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
    s32 retry = 0;
    s32 ret = 0;
    struct goodix_ts_data *ts;

    ts = i2c_get_clientdata(client);

    gtp_irq_disable(ts);

    pr_err("GT9XXF esd recovery mode");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
        if (FAIL == ret)
        {
            GTP_ERROR("esd recovery failed %d\n", retry + 1);
            continue;
        }
        ret = gtp_fw_startup(ts->client);
        if (FAIL == ret)
        {
            GTP_ERROR("GT9XXF start up failed %d\n", retry + 1);
            continue;
        }
        break;
    }
    gtp_irq_enable(ts);

    if (retry >= 5)
    {
        GTP_ERROR("failed to esd recovery");
        return FAIL;
    }

    pr_err("Esd recovery successful");
    return SUCCESS;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_OFF);
#endif
    GTP_DEBUG_FUNC();

    gtp_esd_recovery(client);

#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
    s32 ret = 0;
    s32 i = 0;
    s32 j = 0;
    u16 ref_sum = 0;
    u16 learn_cnt = 0;
    u16 chksum = 0;
    s32 ref_seg_len = 0;
    s32 ref_grps = 0;
    struct file *ref_filp = NULL;
    u8 *p_bak_ref;

    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->ref_chk_fs_times++;
        GTP_DEBUG("Ref check /data times/MAX_TIMES: %d / %d\n", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
            pr_err("/data not mounted.\n");
            return FAIL;
        }
        pr_err("check /data mount timeout...\n");
    }
    else
    {
        pr_err("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }

    p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);

    if (NULL == p_bak_ref)
    {
        GTP_ERROR("Allocate memory for p_bak_ref failed!");
        return FAIL;
    }

    if (ts->is_950)
    {
        ref_seg_len = ts->bak_ref_len / 6;
        ref_grps = 6;
    }
    else
    {
        ref_seg_len = ts->bak_ref_len;
        ref_grps = 1;
    }
    ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(ref_filp))
    {
        GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
        if (GTP_BAK_REF_SEND == mode)
        {
            goto bak_ref_default;
        }
        else
        {
            goto bak_ref_exit;
        }
    }

    switch (mode)
    {
        case GTP_BAK_REF_SEND:
            pr_err("Send backup-reference");
            ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
            ret = ref_filp->f_op->read(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
            if (ret < 0)
            {
                GTP_ERROR("failed to read bak_ref info from file, sending defualt bak_ref");
                goto bak_ref_default;
            }
            for (j = 0; j < ref_grps; ++j)
            {
                ref_sum = 0;
                for (i = 0; i < (ref_seg_len); i += 2)
                {
                    ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i + 1 + j * ref_seg_len];
                }
                learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len - 4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len - 3]);
                chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len - 2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len - 1]);
                GTP_DEBUG("learn count = %d\n", learn_cnt);
                GTP_DEBUG("chksum = %d\n", chksum);
                GTP_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);
                // Sum(1~ref_seg_len) == 1
                if (1 != ref_sum)
                {
                    pr_err("wrong chksum for bak_ref, reset to 0x00 bak_ref");
                    memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                    p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
                }
                else
                {
                    if (j == (ref_grps - 1))
                    {
                        pr_err("backup-reference data in %s used", GTP_BAK_REF_PATH);
                    }
                }
            }
            ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
            if (FAIL == ret)
            {
                GTP_ERROR("failed to send bak_ref because of iic comm error");
                goto bak_ref_exit;
            }
            break;

        case GTP_BAK_REF_STORE:
            pr_err("Store backup-reference");
            ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
            if (ret < 0)
            {
                GTP_ERROR("failed to read bak_ref info, sending default back-reference");
                goto bak_ref_default;
            }
            ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
            ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
            break;

        default:
            GTP_ERROR("invalid backup-reference request");
            break;
    }
    ret = SUCCESS;
    goto bak_ref_exit;

bak_ref_default:

    for (j = 0; j < ref_grps; ++j)
    {
        memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
        p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1
    }
    ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
    if (!IS_ERR(ref_filp))
    {
        pr_err("write backup-reference data into %s", GTP_BAK_REF_PATH);
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
    }
    if (ret == FAIL)
    {
        GTP_ERROR("failed to load the default backup reference");
    }

bak_ref_exit:

    if (p_bak_ref)
    {
        kfree(p_bak_ref);
    }
    if (ref_filp && !IS_ERR(ref_filp))
    {
        filp_close(ref_filp, NULL);
    }
    return ret;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
    u8 chksum = 0;
    u8 main_clock = p_main_clk[0];
    s32 i = 0;

    if (main_clock < 50 || main_clock > 120)
    {
        return FAIL;
    }

    for (i = 0; i < 5; ++i)
    {
        if (main_clock != p_main_clk[i])
        {
            return FAIL;
        }
        chksum += p_main_clk[i];
    }
    chksum += p_main_clk[5];
    if ( (chksum) == 0)
    {
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
    s32 ret = 0;
    s32 i = 0;
    s32 clk_chksum = 0;
    struct file *clk_filp = NULL;
    u8 p_main_clk[6] = {0};

    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->clk_chk_fs_times++;
        GTP_DEBUG("Clock check /data times/MAX_TIMES: %d / %d\n", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
            pr_err("/data not mounted.\n");
            return FAIL;
        }
        pr_err("Check /data mount timeout!");
    }
    else
    {
        pr_err("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }

    clk_filp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(clk_filp))
    {
        GTP_ERROR("%s is unavailable, calculate main clock", GTP_MAIN_CLK_PATH);
    }
    else
    {
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);

        ret = gtp_verify_main_clk(p_main_clk);
        if (FAIL == ret)
        {
            // recalculate main clock & rewrite main clock data to file
            GTP_ERROR("main clock data in %s is wrong, recalculate main clock", GTP_MAIN_CLK_PATH);
        }
        else
        {
            pr_err("main clock data in %s used, main clock freq: %d\n", GTP_MAIN_CLK_PATH, p_main_clk[0]);
            filp_close(clk_filp, NULL);
            goto update_main_clk;
        }
    }

#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
    ret = gup_clk_calibration();
    gtp_esd_recovery(ts->client);

#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif

    pr_err("calibrate main clock: %d\n", ret);
    if (ret < 50 || ret > 120)
    {
        GTP_ERROR("wrong main clock: %d\n", ret);
        goto exit_main_clk;
    }

    // Sum{0x8020~0x8025} = 0
    for (i = 0; i < 5; ++i)
    {
        p_main_clk[i] = ret;
        clk_chksum += p_main_clk[i];
    }
    p_main_clk[5] = 0 - clk_chksum;

    if (!IS_ERR(clk_filp))
    {
        GTP_DEBUG("write main clock data into %s", GTP_MAIN_CLK_PATH);
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
        filp_close(clk_filp, NULL);
    }

update_main_clk:
    ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
    if (FAIL == ret)
    {
        GTP_ERROR("update main clock failed!");
        return FAIL;
    }
    return SUCCESS;

exit_main_clk:
    if (!IS_ERR(clk_filp))
    {
        filp_close(clk_filp, NULL);
    }
    return FAIL;
}


s32 gtp_gt9xxf_init(struct i2c_client *client)
{
    s32 ret = 0;
    ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
    if (FAIL == ret)
    {
        return FAIL;
    }
    ret = gtp_fw_startup(client);
    if (FAIL == ret)
    {
        return FAIL;
    }
    return SUCCESS;
}

void gtp_get_chip_type(struct goodix_ts_data *ts)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;

    msleep(10);

    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);

    if (FAIL == ret)
    {
        GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
        ts->chip_type = CHIP_TYPE_GT9;
        return;
    }

    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        ts->chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        ts->chip_type = CHIP_TYPE_GT9F;
    }
    pr_err("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

#endif
//************* For GT9XXF End ************//
static int goodix_power_init(struct goodix_ts_data *ts)
{
    int ret;

#if 0
    ts->avdd = regulator_get(&ts->client->dev, "avdd");
    if (IS_ERR(ts->avdd))
    {
        ret = PTR_ERR(ts->avdd);
        dev_info(&ts->client->dev,
                 "Regulator get failed avdd ret=%d\n", ret);
    }
#endif
    ts->vdd = regulator_get(&ts->client->dev, "vdd");
    if (IS_ERR(ts->vdd))
    {
        ret = PTR_ERR(ts->vdd);
        dev_info(&ts->client->dev, "Regulator get failed vdd ret=%d\n", ret);
    }

    ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
    if (IS_ERR(ts->vcc_i2c))
    {
        ret = PTR_ERR(ts->vcc_i2c);
        dev_info(&ts->client->dev,
                 "Regulator get failed vcc_i2c ret=%d\n", ret);
    }

    return 0;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
    return (regulator_count_voltages(reg) > 0) ?
           regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int goodix_power_on(struct goodix_ts_data *ts)
{
    int ret;

    if (ts->power_on)
    {
        dev_info(&ts->client->dev, "Device already power on\n");
        return 0;
    }
#if 0
    if (!IS_ERR(ts->avdd))
    {
        ret = reg_set_optimum_mode_check(ts->avdd,
                                         GOODIX_VDD_LOAD_MAX_UA);
        if (ret < 0)
        {
            dev_err(&ts->client->dev,
                    "Regulator avdd set_opt failed rc=%d\n", ret);
            goto err_set_opt_avdd;
        }
        ret = regulator_enable(ts->avdd);
        if (ret)
        {
            dev_err(&ts->client->dev,
                    "Regulator avdd enable failed ret=%d\n", ret);
            goto err_enable_avdd;
        }
    }
#endif
    if (!IS_ERR(ts->vdd))
    {
        ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
                                    GOODIX_VTG_MAX_UV);
        if (ret)
        {
            dev_err(&ts->client->dev,
                    "Regulator set_vtg failed vdd ret=%d\n", ret);
            goto err_set_vtg_vdd;
        }
        ret = reg_set_optimum_mode_check(ts->vdd,
                                         GOODIX_VDD_LOAD_MAX_UA);
        if (ret < 0)
        {
            dev_err(&ts->client->dev,
                    "Regulator vdd set_opt failed rc=%d\n", ret);
            goto err_set_opt_vdd;
        }
        ret = regulator_enable(ts->vdd);
        if (ret)
        {
            dev_err(&ts->client->dev,
                    "Regulator vdd enable failed ret=%d\n", ret);
            goto err_enable_vdd;
        }
    }

    if (!IS_ERR(ts->vcc_i2c))
    {
        ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
                                    GOODIX_I2C_VTG_MAX_UV);
        if (ret)
        {
            dev_err(&ts->client->dev,
                    "Regulator set_vtg failed vcc_i2c ret=%d\n",
                    ret);
            goto err_set_vtg_vcc_i2c;
        }
        ret = reg_set_optimum_mode_check(ts->vcc_i2c,
                                         GOODIX_VIO_LOAD_MAX_UA);
        if (ret < 0)
        {
            dev_err(&ts->client->dev,
                    "Regulator vcc_i2c set_opt failed rc=%d\n",
                    ret);
            goto err_set_opt_vcc_i2c;
        }
        ret = regulator_enable(ts->vcc_i2c);
        if (ret)
        {
            dev_err(&ts->client->dev,
                    "Regulator vcc_i2c enable failed ret=%d\n",
                    ret);
            regulator_disable(ts->vdd);
            goto err_enable_vcc_i2c;
        }
    }

    ts->power_on = true;
    return 0;

err_enable_vcc_i2c:
err_set_opt_vcc_i2c:
    if (!IS_ERR(ts->vcc_i2c))
        regulator_set_voltage(ts->vcc_i2c, 0, GOODIX_I2C_VTG_MAX_UV);
err_set_vtg_vcc_i2c:
    if (!IS_ERR(ts->vdd))
        regulator_disable(ts->vdd);
err_enable_vdd:
err_set_opt_vdd:
    if (!IS_ERR(ts->vdd))
        regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
err_set_vtg_vdd:
#if 0
    if (!IS_ERR(ts->avdd))
        regulator_disable(ts->avdd);
err_enable_avdd:
err_set_opt_avdd:
#endif
    ts->power_on = false;
    return ret;
}

/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
#if 0
static int goodix_power_off(struct goodix_ts_data *ts)
{
    int ret;

    if (!ts->power_on)
    {
        dev_info(&ts->client->dev,
                 "Device already power off\n");
        return 0;
    }

    if (!IS_ERR(ts->vcc_i2c))
    {
        ret = regulator_set_voltage(ts->vcc_i2c, 0,
                                    GOODIX_I2C_VTG_MAX_UV);
        if (ret < 0)
            dev_err(&ts->client->dev,
                    "Regulator vcc_i2c set_vtg failed ret=%d\n",
                    ret);
        ret = regulator_disable(ts->vcc_i2c);
        if (ret)
            dev_err(&ts->client->dev,
                    "Regulator vcc_i2c disable failed ret=%d\n",
                    ret);
    }

    if (!IS_ERR(ts->vdd))
    {
        ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
        if (ret < 0)
            dev_err(&ts->client->dev,
                    "Regulator vdd set_vtg failed ret=%d\n", ret);
        ret = regulator_disable(ts->vdd);
        if (ret)
            dev_err(&ts->client->dev,
                    "Regulator vdd disable failed ret=%d\n", ret);
    }
#if 0
    if (!IS_ERR(ts->avdd))
    {
        ret = regulator_disable(ts->avdd);
        if (ret)
            dev_err(&ts->client->dev,
                    "Regulator avdd disable failed ret=%d\n", ret);
    }
#endif
    ts->power_on = false;
    return 0;
}
#endif

static int goodix_ts_get_dt_coords(struct device *dev, char *name,
                                   struct goodix_ts_platform_data *pdata)
{
    struct property *prop;
    struct device_node *np = dev->of_node;
    int rc;
    u32 coords[GOODIX_COORDS_ARR_SIZE];

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    rc = of_property_read_u32_array(np, name, coords,
                                    GOODIX_COORDS_ARR_SIZE);
    if (rc && (rc != -EINVAL))
    {
        dev_err(dev, "Unable to read %s\n", name);
        return rc;
    }

    if (!strcmp(name, "goodix,panel-coords"))
    {
        pdata->panel_minx = coords[0];
        pdata->panel_miny = coords[1];
        pdata->panel_maxx = coords[2];
        pdata->panel_maxy = coords[3];
        LCD_WIDTH = pdata->panel_maxx;
        LCD_HEIGHT = pdata->panel_maxy;
    }
    else if (!strcmp(name, "goodix,display-coords"))
    {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
        LCD_WIDTH = pdata->x_max;
        LCD_HEIGHT = pdata->y_max;
    }
    else
    {
        dev_err(dev, "unsupported property %s\n", name);
        return -EINVAL;
    }

    return 0;
}



static int goodix_parse_dt(struct device *dev,
                           struct goodix_ts_platform_data *pdata)
{
    int rc;
    struct device_node *np = dev->of_node;
    struct property *prop;
    u32 temp_val, num_buttons;
//  u32 button_map[MAX_BUTTONS];
    char prop_name[PROP_NAME_SIZE];
    int i, read_cfg_num;

    rc = goodix_ts_get_dt_coords(dev, "goodix,panel-coords", pdata);
    if (rc && (rc != -EINVAL))
        return rc;

    rc = goodix_ts_get_dt_coords(dev, "goodix,display-coords", pdata);
    if (rc)
        return rc;

    pdata->i2c_pull_up = of_property_read_bool(np,
                         "goodix,i2c-pull-up");

    pdata->force_update = of_property_read_bool(np,
                          "goodix,force-update");

    pdata->enable_power_off = of_property_read_bool(np,
                              "goodix,enable-power-off");

    pdata->have_touch_key = of_property_read_bool(np,
                            "goodix,have-touch-key");

    pdata->driver_send_cfg = of_property_read_bool(np,
                             "goodix,driver-send-cfg");

    pdata->change_x2y = of_property_read_bool(np,
                        "goodix,change-x2y");

    pdata->with_pen = of_property_read_bool(np,
                                            "goodix,with-pen");

    pdata->slide_wakeup = of_property_read_bool(np,
                          "goodix,slide-wakeup");

    pdata->dbl_clk_wakeup = of_property_read_bool(np,
                            "goodix,dbl_clk_wakeup");

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        return pdata->reset_gpio;

    pdata->irq_gpio = of_get_named_gpio_flags(np, "interrupt-gpios",
                      0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        return pdata->irq_gpio;

    pdata->id1_gpio = of_get_named_gpio(np, "goodix,id1-gpio", 0);
    if ( pdata->id1_gpio < 0 )
    {
        dev_err(dev, "ts->id1_gpio not specified\n");
    }
    pdata->id2_gpio = of_get_named_gpio(np, "goodix,id2-gpio", 0);
    if ( pdata->id2_gpio < 0 )
    {
        dev_err(dev, "ts->id2_gpio not specified\n");
    }
    pdata->id3_gpio = of_get_named_gpio(np, "goodix,id3-gpio", 0);
    if ( pdata->id3_gpio < 0 )
    {
        dev_err(dev, "ts->id3_gpio not specified\n");
    }

    rc = of_property_read_string(np, "goodix,product-id",
                                 &pdata->product_id);
    if (rc && (rc != -EINVAL))
    {
        dev_err(dev, "Failed to parse product_id.\n");
        return -EINVAL;
    }

    rc = of_property_read_string(np, "goodix,fw_name",
                                 &pdata->fw_name);
    if (rc && (rc != -EINVAL))
    {
        dev_err(dev, "Failed to parse firmware name.\n");
        return -EINVAL;
    }

    prop = of_find_property(np, "goodix,button-map", NULL);
    if (prop)
    {
        num_buttons = prop->length / sizeof(temp_val);
        if (num_buttons > MAX_BUTTONS)
            return -EINVAL;

        rc = of_property_read_u32_array(np,
                                        "goodix,button-map", button_map,
                                        num_buttons);
        if (rc)
        {
            dev_err(dev, "Unable to read key codes\n");
            return rc;
        }
        pdata->num_button = num_buttons;
        memcpy(pdata->button_map, button_map, pdata->num_button * sizeof(u32));
    }

    read_cfg_num = 0;
    for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++)
    {
        snprintf(prop_name, sizeof(prop_name), "goodix,cfg-data%d\n", i);
        prop = of_find_property(np, prop_name,
                                &pdata->config_data_len[i]);
        if (!prop || !prop->value)
        {
            pdata->config_data_len[i] = 0;
            pdata->config_data[i] = NULL;
            continue;
        }
        pdata->config_data[i] = devm_kzalloc(dev,
                                             GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH,
                                             GFP_KERNEL);
        if (!pdata->config_data[i])
        {
            dev_err(dev,
                    "Not enough memory for panel config data %d\n",
                    i);
            return -ENOMEM;
        }
        pdata->config_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
        pdata->config_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
        memcpy(&pdata->config_data[i][GTP_ADDR_LENGTH],
               prop->value, pdata->config_data_len[i]);
        read_cfg_num++;
    }

    dev_err(dev, "%d config data read from device tree.\n", read_cfg_num);

    return 0;
}

#if 0 /* delete it by lauson. */
/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
struct gtp_pinctrl_info
{
    struct pinctrl *pinctrl;
    struct pinctrl_state *gpio_state_active;
    struct pinctrl_state *gpio_state_suspend;
};

static struct gtp_pinctrl_info gt9xx_pctrl;
static int gtp_pinctrl_init(struct device *dev)
{
    gt9xx_pctrl.pinctrl = devm_pinctrl_get(dev);

    if (IS_ERR_OR_NULL(gt9xx_pctrl.pinctrl))
    {
        pr_err("%s:%d Getting pinctrl handle failed\n",
               __func__, __LINE__);
        return -EINVAL;
    }
    gt9xx_pctrl.gpio_state_active = pinctrl_lookup_state(
                                        gt9xx_pctrl.pinctrl,
                                        GOODIX_PINCTRL_STATE_DEFAULT);

    if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_active))
    {
        pr_err("%s:%d Failed to get the active state pinctrl handle\n",
               __func__, __LINE__);
        return -EINVAL;
    }
    gt9xx_pctrl.gpio_state_suspend = pinctrl_lookup_state(
                                         gt9xx_pctrl.pinctrl,
                                         GOODIX_PINCTRL_STATE_SLEEP);

    if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_suspend))
    {
        pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
               __func__, __LINE__);
        return -EINVAL;
    }
    return 0;
}
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#endif

static int gt9xx_free_fingers(struct goodix_ts_data *ts)
{
	int i = 0;

       if (tp_probe_ok == 0)
           return 0;

	for (i = 0; i < GTP_MAX_TOUCH; i++) {
             gtp_touch_up(ts, i);
	}
       input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
       input_sync(ts->input_dev);

	return 0;
}

static ssize_t gt9xx_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t gt9xx_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
       u8  cfg_version;
	int rc;
	pr_err("gt9xx:start update fw\n");
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
       pr_err("%s fw_version:%s\n", __func__,  fw_version);
	if(val == 1 || val == 0)
       {
            u16 version_info;
            gt9xx_force_update = val;
            if (ts->use_irq)
                gtp_irq_disable(ts);
            gt9xx_free_fingers(ts);
            mutex_lock(&ts->input_dev->mutex);

            // update ......
#if GTP_AUTO_UPDATE
            rc = gup_init_update_proc(ts);
            if (rc < 0)
            {
                pr_err("gt9xx update error.\n");
            }
#endif
            mutex_unlock(&ts->input_dev->mutex);
            rc = gtp_read_fw_version(ts->client, &version_info);
            if (rc != 2)
            {
                dev_err(&ts->client->dev, "GTP firmware version read failed.\n");
            }
            rc = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &cfg_version, 1);
            memset(fw_version, 0, sizeof(fw_version));
            sprintf(fw_version, "%d%x%2x", get_project(), version_info , cfg_version);
              if (ts->use_irq)
                    gtp_irq_enable(ts);
       }
	return size;
}
static DEVICE_ATTR(oppo_tp_fw_update, 0664, gt9xx_update_fw_show, gt9xx_update_fw_store);


static int init_gt9xx_proc(void)
{
    int ret = 0;
    prEntry_tp = proc_mkdir("touchpanel", NULL);
    if ( prEntry_tp == NULL )
    {
        ret = -ENOMEM;
        printk(KERN_INFO"init_gt9xx_proc: Couldn't create TP proc entry\n");
    }

#ifdef GT9XX_SUPPORT_GESTURE
    if (is_project(OPPO_15109))
    {
        prEntry_double_tap = proc_create( "double_tap_enable", 0666, prEntry_tp, &tp_double_proc_fops);
        if (prEntry_double_tap == NULL)
        {
            ret = -ENOMEM;
            printk(KERN_INFO"init_gt9xx_proc: Couldn't create proc entry\n");
        }
    }
    prEntry_coodinate = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
    if (prEntry_coodinate == NULL)
    {
        ret = -ENOMEM;
        printk(KERN_INFO"init_gt9xx_proc: Couldn't create proc entry\n");
    }
#endif

#ifdef GT9XX_SUPPORT_GLOVES_MODE
    prEntry_glove_mode = proc_create( "glove_mode_enable", 0666, prEntry_tp, &glove_mode_enable_proc_fops);
    if (prEntry_glove_mode == NULL)
    {
        ret = -ENOMEM;
        printk(KERN_INFO"init_gt9xx_proc: Couldn't create proc entry\n");
    }
#endif
#ifdef GT9XX_SUPPORT_APK_AUTOTEST
    prEntry_glove_check = proc_create( "glove_mode_check", 0444, prEntry_tp, &glove_check_proc_fops);
    if (prEntry_glove_check == NULL)
    {
        ret = -ENOMEM;
        printk(KERN_INFO"init_gt9xx_proc: Couldn't create proc entry\n");
    }

#endif

	if (device_create_file(&i2c_connect_client->dev, &dev_attr_oppo_tp_fw_update)) {
		pr_err("devices_create_file failt \n");
	}

    return 0;
}

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes.
        0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;

    struct goodix_ts_platform_data *pdata;
    struct goodix_ts_data *ts;
    u16 version_info;
    u8  cfg_version;

    int boot_mode = 0;

    boot_mode = get_boot_mode();

    GTP_DEBUG_FUNC();

    dev_err(&client->dev, "GTP I2C Address: 0x%02x\n", client->addr);
    if (client->dev.of_node)
    {
        pdata = devm_kzalloc(&client->dev,
                             sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
        if (!pdata)
        {
            dev_err(&client->dev,
                    "GTP Failed to allocate memory for pdata\n");
            return -ENOMEM;
        }
        ret = goodix_parse_dt(&client->dev, pdata);
        if (ret)
            return ret;
    }
    else
    {
        pdata = client->dev.platform_data;
    }
    if (!pdata)
    {
        dev_err(&client->dev, "GTP invalid pdata\n");
        return -EINVAL;
    }

    //do NOT remove these logs
    GTP_DEBUG("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    GTP_DEBUG("GTP Driver Built@%s, %s", __TIME__, __DATE__);
    GTP_DEBUG("GTP I2C Address: 0x%02x", client->addr);

    i2c_connect_client = client;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        GTP_ERROR("I2C check functionality failed.\n");
        return -ENODEV;
    }

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
    {
        dev_err(&client->dev, "GTP not enough memory for ts\n");
        return -ENOMEM;
    }

    memset(ts, 0, sizeof(*ts));

#if 0
    pr_err(">>-- %s line:%d \n", __func__, __LINE__);
    ts->client = client;

    ts->pdata = pdata;
    i2c_set_clientdata(client, ts);
    ret = gtp_request_io_port(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.\n");
        kfree(ts);
        return ret;
    }

    pr_err(">>-- %s line:%d \n", __func__, __LINE__);
#endif

    INIT_WORK(&ts->work, goodix_ts_work_func);
    ts->client = client;

    ts->pdata = pdata;

    ts->boot_mode = boot_mode;

    spin_lock_init(&ts->irq_lock);          // 2.6.39 later
    // ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
#if GTP_ESD_PROTECT
    ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
    GTP_DEBUG("Clock ticks for an esd cycle: %d\n", ts->clk_tick_cnt);
    spin_lock_init(&ts->esd_lock);
    // ts->esd_lock = SPIN_LOCK_UNLOCKED;
#endif
    i2c_set_clientdata(client, ts);

    ts->gtp_rawdiff_mode = 0;
    ts->power_on = false;

    mutex_init(&ts->rst_lock);

    ret = gtp_request_io_port(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.\n");
        i2c_set_clientdata(client, NULL);
        return ret;
    }

#if 0 /* delete it by lauson. */
    /*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
    gtp_pinctrl_init(client->dev)
    ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
                               gt9xx_pctrl.gpio_state_active);
    if (ret)
        pr_err("%s:%d cannot set pin to suspend state",
               __func__, __LINE__);
    /*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#endif
    ret = goodix_power_init(ts);
    if (ret)
    {
        dev_err(&client->dev, "GTP power init failed\n");
//      goto exit_free_io_port;
    }

    ret = goodix_power_on(ts);
    if (ret)
    {
        dev_err(&client->dev, "GTP power on failed\n");
//      goto exit_deinit_power;
    }

    gtp_reset_guitar(ts->client, 20);

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
        i2c_set_clientdata(client, NULL);
        return ret;
    }

    /*[FEATURE]Add Begin by TCTSZ-weihong.chen 2014-5-29, do this after  gtp_i2c_test.*/
    //g_tp_device_name = "gt9271";
    /*[FEATURE]Add END by TCTSZ-weihong.chen*/
#if GTP_COMPATIBLE_MODE
    gtp_get_chip_type(ts);
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ret = gtp_gt9xxf_init(ts->client);
        if (FAIL == ret)
        {
            pr_err("Failed to init GT9XXF.\n");
        }
    }
#endif

#if 0
    pr_err(">>-- %s line:%d \n", __func__, __LINE__);
    ret = gtp_i2c_test(client);
    pr_err(">>-- %s line:%d, ret=%d \n", __func__, __LINE__, ret);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
    }
#endif
    ret = gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.\n");
    }
    gtp_get_sensor_id(ts->pdata->id1_gpio, ts->pdata->id2_gpio, ts->pdata->id3_gpio, ts->pdata);
    pr_err("<GTP>sensor_id = %d\n", sensor_id);
    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.\n");
        //    ts->abs_x_max = GTP_MAX_WIDTH;
        //    ts->abs_y_max = GTP_MAX_HEIGHT;
        //    ts->int_trigger_type = GTP_INT_TRIGGER;
    }
    pr_err("gtp_init_panel end");
    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        pr_err("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }

#if GTP_ESD_PROTECT
//#if 0 /* delete it by lauson. */
    gtp_esd_switch(client, SWITCH_ON);
//#endif
#endif
#if 0 /* delete it by lauson. */
#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(ts);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.\n");
    }
#endif
#endif
    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
    }
    input_set_drvdata(ts->input_dev, ts);
    ret = gtp_request_irq(ts);
    if (ret < 0)
    {
        pr_err("GTP works in polling mode.\n");
    }
    else
    {
        pr_err("GTP works in interrupt mode.\n");
    }

    ret = gtp_read_fw_version(client, &version_info);
    if (ret != 2)
    {
        dev_err(&client->dev, "GTP firmware version read failed.\n");
    }
    pr_err("<GTP> firmware version = %x\n ", version_info);
    ret = gtp_check_product_id(client);
    if (ret != 0)
    {
        dev_err(&client->dev, "GTP Product id doesn't match. ret=%d\n", ret);
//      goto exit_free_irq;
        return -1;
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }

#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif

    init_gt9xx_proc();

#ifdef OPPO_RAWDATA_INTERFACE
    gtp_sysfs_init();
#endif

#ifdef GT9XX_SUPPORT_GESTURE
    atomic_set(&double_enable, 0);
#endif

#ifdef GT9XX_SUPPORT_GLOVES_MODE
    atomic_set(&glove_enable, 0);

#endif

    INIT_WORK(&ts->speed_up_work,speedup_goodix_resume);

    mutex_init(&ts->io_lock);

    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &cfg_version, 1);
    //sprintf(ts->pdata->tp_info.version, "150250%x",version_info );
    memset(fw_version, 0, sizeof(fw_version));
    sprintf(fw_version, "%d%x%2x", get_project(), version_info , cfg_version);
    ts->pdata->tp_info.version = fw_version;
    register_device_proc("tp", ts->pdata->tp_info.version, ts->pdata->tp_info.manufacture);
    dev_err(&client->dev, "%s  ok !\n", __func__);
    tp_probe_ok = 1;

    return 0;
}


/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    GTP_DEBUG_FUNC();

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    #ifdef OPPO_RAWDATA_INTERFACE
    gtp_sysfs_uninit();
    #endif

    if (ts)
    {
        if (ts->use_irq)
        {
            GTP_GPIO_AS_INPUT(ts->pdata->irq_gpio);
            GTP_GPIO_FREE(ts->pdata->irq_gpio);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }

    pr_err("GTP driver removing...\n");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int goodix_ts_suspend(struct device *dev)
{
    struct goodix_ts_data *ts = dev_get_drvdata(dev);
    int ret = 0, i;
    pr_err("%s enter \n", __func__);

#if 0 /* delete it by lauson. */
    /*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
    ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
                               gt9xx_pctrl.gpio_state_suspend);
    if (ret)
        pr_err("%s:%d cannot set pin to suspend state",
               __func__, __LINE__);
    /*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#endif
    if (ts->gtp_is_suspend)
    {
        dev_dbg(&ts->client->dev, "Already in suspend state.\n");
        return 0;
    }

   mutex_lock(&ts->io_lock);
#if 0
    if (ts->fw_loading)
    {
        dev_info(&ts->client->dev,
                 "Fw upgrade in progress, can't go to suspend.\n");
//      mutex_unlock(&ts->lock);
        return 0;
    }
#endif

#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_GESTURE_WAKEUP
#ifdef GT9XX_SUPPORT_GESTURE
    if (atomic_read(&double_enable))
#else
    if (ts->pdata->dbl_clk_wakeup)
#endif
    {
        pr_err("%s enable double func \n", __func__);
        /*enable gpio wake system through intterrupt*/
        enable_irq_wake(ts->client->irq);
        ret = gtp_enter_doze(ts);
    }
    else
#endif
    {
        if (ts->use_irq)
            gtp_irq_disable(ts);

        for (i = 0; i < GTP_MAX_TOUCH; i++)
            gtp_touch_up(ts, i);

        input_sync(ts->input_dev);

        ret = gtp_enter_sleep(ts);
        if (ret < 0)
            dev_err(&ts->client->dev, "GTP early suspend failed.\n");
    }
    /* to avoid waking up while not sleeping,
     * delay 48 + 10ms to ensure reliability
     */
    msleep(58);

    ts->gtp_is_suspend = 1;
    palm_lock_flag = 1;     /*Add by TCTSZ-WH,2014-5-27,Add palm lock function, avoid wake up shortly after suspend.*/

   mutex_unlock(&ts->io_lock);

    return ret;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int goodix_ts_resume(struct device *dev)
{
    struct goodix_ts_data *ts = dev_get_drvdata(dev);

    pr_err("%s enter \n", __func__);

    if (!ts->gtp_is_suspend)
    {
        dev_dbg(&ts->client->dev, "Already in awake state.\n");
        return 0;
    }
    queue_work(goodix_wq, &ts->speed_up_work);
    return 0;
}

static void speedup_goodix_resume(struct work_struct *work)
{
    int ret = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    pr_err("%s enter.  cur_mode:%d \n", __func__, ts->gtp_rawdiff_mode);

    gt9xx_free_fingers(ts);

#if 0 /* delete it by lauson. */
    /*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
    ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
                               gt9xx_pctrl.gpio_state_active);
    if (ret)
        pr_err("%s:%d cannot set pin to suspend state",
               __func__, __LINE__);
    /*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#endif

    mutex_lock(&ts->io_lock);
    ts->gtp_rawdiff_mode = false;   //force diff mode  exit.

    ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
#ifdef GT9XX_SUPPORT_GESTURE
    if (atomic_read(&double_enable))
#else
    if (ts->pdata->dbl_clk_wakeup)
#endif
    {
        doze_status = DOZE_DISABLED;
        /*disable gpio wake system through intterrupt*/
        disable_irq_wake(ts->client->irq);
    }
#endif

    if (ret <= 0)
        dev_err(&ts->client->dev, "GTP resume failed.\n");

    if (ts->use_irq)
        gtp_irq_enable(ts);

#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif

    ts->gtp_is_suspend = 0;
    palm_lock_flag = 0;

    mutex_unlock(&ts->io_lock);

    pr_err("%s exit.\n", __func__);
    return ;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct goodix_ts_data *ts =
        container_of(self, struct goodix_ts_data, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
        ts && ts->client)
    {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK)
            goodix_ts_resume(&ts->client->dev);
        else if (*blank == FB_BLANK_POWERDOWN)
            goodix_ts_suspend(&ts->client->dev);
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;
    ts = container_of(h, struct goodix_ts_data, early_suspend);

    GTP_DEBUG_FUNC();

    pr_err("System suspend.\n");

    ts->gtp_is_suspend = 1;
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_GESTURE_WAKEUP
    ret = gtp_enter_doze(ts);
#else
    if (ts->use_irq)
    {
        gtp_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gtp_enter_sleep(ts);
#endif
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.\n");
    }
    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability
    msleep(58);
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;
    ts = container_of(h, struct goodix_ts_data, early_suspend);

    GTP_DEBUG_FUNC();

    pr_err("System resume.\n");

    ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
    doze_status = DOZE_DISABLED;
#endif

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.\n");
    }
#if (GTP_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // do nothing
    }
    else
#endif
    {
        gtp_send_cfg(ts->client);
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif
}
#endif

#endif


#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while (retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
    }
    return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while (retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if ((retries >= 5))
    {
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
    }
    return ret;
}
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_ts_data *ts;

    ts = i2c_get_clientdata(client);
    spin_lock(&ts->esd_lock);

    if (SWITCH_ON == on)     // switch on esd
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
            spin_unlock(&ts->esd_lock);
            pr_err("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            pr_err("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
    GTP_DEBUG("[Esd]Init external watchdog");
    return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    u8 esd_buf[5] = {0x80, 0x40};

    GTP_DEBUG_FUNC();

    ts = i2c_get_clientdata(i2c_connect_client);

    if (ts->gtp_is_suspend)
    {
        pr_err("Esd suspended!");
        return;
    }

    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

        // GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        {
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};

                gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

                GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);

                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = 3;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA;
                gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            if (ts->rqst_processing)
            {
                pr_err("Request processing, no esd recovery");
            }
            else
            {
                GTP_ERROR("IC working abnormally! Process esd recovery.\n");
                esd_buf[0] = 0x42;
                esd_buf[1] = 0x26;
                esd_buf[2] = 0x01;
                esd_buf[3] = 0x01;
                esd_buf[4] = 0x01;
                gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
                msleep(50);
                gtp_esd_recovery(ts->client);
            }
        }
        else
#endif
        {
            GTP_ERROR("IC working abnormally! Process reset guitar.\n");
            esd_buf[0] = 0x42;
            esd_buf[1] = 0x26;
            esd_buf[2] = 0x01;
            esd_buf[3] = 0x01;
            esd_buf[4] = 0x01;
            gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
            msleep(50);
            gtp_reset_guitar(ts->client, 50);
            msleep(50);
            gtp_send_cfg(ts->client);
        }
    }

    if (!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        pr_err("Esd suspended!");
    }
    return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] =
{
    { GTP_I2C_NAME, 0 },
    { }
};
static struct of_device_id goodix_match_table[] =
{
    { .compatible = "goodix,gt9xx", },
    { },
};

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops goodix_ts_dev_pm_ops =
{
    .suspend = goodix_ts_suspend,
    .resume = goodix_ts_resume,
};
#else
static const struct dev_pm_ops goodix_ts_dev_pm_ops =
{
};
#endif

static struct i2c_driver goodix_ts_driver =
{
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_early_suspend,
    .resume     = goodix_ts_late_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = goodix_match_table,
#if CONFIG_PM
        .pm = &goodix_ts_dev_pm_ops,
#endif
    },
};


#ifdef OPPO_RAWDATA_INTERFACE
struct kobject *goodix_debug_kobj;
static ssize_t gtp_sysfs_rawdata_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    u32 index = 0, i = 0;
    u32 len = 0;
    u8  ctrl_buf[3] = {0x80, 0x40, 0x01};
    s32 ret = 0;
    u8 *data = NULL;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG("gtp_sysfs_rawdata_show invoked!driver_num:%d, sensor_num:%d", driver_num, sensor_num);

    if(driver_num*sensor_num == 0)
    {
        GTP_DEBUG("gtp_sysfs_rawdata_show Invalid data length to show!");
        return 0;
    }
    data = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data == NULL)
    {
        GTP_DEBUG("gtp_sysfs_rawdata_show cannot alloc memory!");
        return 0;
    }

    ts->gtp_rawdiff_mode = 1;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        GTP_ERROR("gtp_sysfs_rawdata_show enter rawdata mode failed.");
        kfree(data);
        return 0;
    }

    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;

    GTP_DEBUG("gtp_sysfs_rawdata_show begin wait data ready flag.");
    do {
        gtp_i2c_read(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
        msleep(20);
    }while((ctrl_buf[2] & 0x80) != 0x80);
    GTP_DEBUG("gtp_sysfs_rawdata_show rawdata ready flag is set.");

    data[0] = 0x8B; //0xBB;    //0x8B;
    data[1] = 0x98; //0x10;    //0x98;
    ret = gtp_i2c_read(i2c_connect_client, data, driver_num*sensor_num*2 + 2);
    if(ret < 0)
    {
        GTP_DEBUG("gtp_sysfs_rawdata_show read rawdata failed.");
    }

    ctrl_buf[0] = 0x80;
    ctrl_buf[1] = 0x40;
    ctrl_buf[2] = 0x00;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        GTP_ERROR("gtp_sysfs_rawdata_show exit rawdata mode failed.");
        kfree(data);
        return 0;
    }
    ts->gtp_rawdiff_mode = 0;
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));

    for(index = 0, i = 0; index < driver_num*sensor_num; index++, i += 2)
    {
        if(index && (index % (sensor_num) == 0))
        {
            len += sprintf(&buf[len], "\n");
        }
        len += sprintf(&buf[len], "%04d ", (s16)((((s16)data[2 + i]) << 8) + data[2 + i + 1]));
    }
    GTP_DEBUG("gtp_sysfs_rawdata_show return len:%d", len);

    kfree(data);
    return len;
}

static ssize_t gtp_sysfs_rawdata_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}
static ssize_t gtp_sysfs_referencedata_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    u32 index = 0, i = 0;
    u32 len = 0;
    u8  ctrl_buf[3] = {0x80, 0x40, 0x01};
    s32 ret = 0;
    u8 *data = NULL;

    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG("gtp_sysfs_rawdata_show invoked!driver_num:%d, sensor_num:%d", driver_num, sensor_num);

    if(driver_num*sensor_num == 0)
    {
        GTP_DEBUG("gtp_sysfs_rawdata_show Invalid data length to show!");
        return 0;
    }
    data = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data == NULL)
    {
        GTP_DEBUG("gtp_sysfs_rawdata_show cannot alloc memory!");
        return 0;
    }

    ts->gtp_rawdiff_mode = 1;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        GTP_ERROR("gtp_sysfs_rawdata_show enter rawdata mode failed.");
        kfree(data);
        return 0;
    }

    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;

    GTP_DEBUG("gtp_sysfs_rawdata_show begin wait data ready flag.");
    do {
        gtp_i2c_read(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
        msleep(20);
    }while((ctrl_buf[2] & 0x80) != 0x80);
    GTP_DEBUG("gtp_sysfs_rawdata_show rawdata ready flag is set.");

    data[0] = 0x81; //0xBB;    //0x8B;
    data[1] = 0xC0; //0x10;    //0x98;
    ret = gtp_i2c_read(i2c_connect_client, data, driver_num*sensor_num*2 + 2);
    if(ret < 0)
    {
        GTP_DEBUG("gtp_sysfs_rawdata_show read rawdata failed.");
    }

    ctrl_buf[0] = 0x80;
    ctrl_buf[1] = 0x40;
    ctrl_buf[2] = 0x00;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        GTP_ERROR("gtp_sysfs_rawdata_show exit rawdata mode failed.");
        kfree(data);
        return 0;
    }
    ts->gtp_rawdiff_mode = 0;
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));

    for(index = 0, i = 0; index < driver_num*sensor_num; index++, i += 2)
    {
        if(index && (index % (sensor_num) == 0))
        {
            len += sprintf(&buf[len], "\n");
        }
        len += sprintf(&buf[len], "%04d ", (s16)((((s16)data[2 + i]) << 8) + data[2 + i + 1]));
    }
    GTP_DEBUG("gtp_sysfs_rawdata_show return len:%d", len);

    kfree(data);
    return len;
}

static ssize_t gtp_sysfs_referencedata_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t gtp_sysfs_diffdata_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    u32 index = 0, i = 0;
    u32 len = 0;
    u8  ctrl_buf[3] = {0x80, 0x40, 0x01};
    s32 ret = 0;
    u8 *data = NULL;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG("gtp_sysfs_diffdata_show invoked!driver_num:%d, sensor_num:%d", driver_num, sensor_num);

    if(driver_num*sensor_num == 0)
    {
        GTP_DEBUG("gtp_sysfs_diffdata_show Invalid data length to show!");
        return 0;
    }
    data = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data == NULL)
    {
        GTP_DEBUG("gtp_sysfs_diffdata_show cannot alloc memory!");
        return 0;
    }

    ts->gtp_rawdiff_mode = 1;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        GTP_ERROR("gtp_sysfs_diffdata_show enter rawdata mode failed.");
        kfree(data);
        return 0;
    }

    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;

    GTP_DEBUG("gtp_sysfs_diffdata_show begin wait data ready flag.");
    do {
        gtp_i2c_read(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
        msleep(20);
    }while((ctrl_buf[2] & 0x80) != 0x80);
    GTP_DEBUG("gtp_sysfs_diffdata_show rawdata ready flag is set.");

    data[0] = 0xBB;    //0x8B;
    data[1] = 0x10;    //0x98;
    ret = gtp_i2c_read(i2c_connect_client, data, driver_num*sensor_num*2 + 2);
    if(ret < 0)
    {
        GTP_DEBUG("gtp_sysfs_diffdata_show read rawdata failed.");
    }

    ctrl_buf[0] = 0x80;
    ctrl_buf[1] = 0x40;
    ctrl_buf[2] = 0x00;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        GTP_ERROR("gtp_sysfs_diffdata_show exit rawdata mode failed.");
        kfree(data);
        return 0;
    }
    ts->gtp_rawdiff_mode = 0;
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));

    for(index = 0, i = 0; index < driver_num*sensor_num; index++, i += 2)
    {
        if(index && (index % (sensor_num) == 0))
        {
            len += sprintf(&buf[len], "\n");
        }
        len += sprintf(&buf[len], "%04d ", (s16)((((s16)data[2 + i]) << 8) + data[2 + i + 1]));
    }
    GTP_DEBUG("gtp_sysfs_diffdata_show return len:%d", len);

    kfree(data);
    return len;
}

static ssize_t gtp_sysfs_diffdata_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static DEVICE_ATTR(rawdata, S_IRUGO|S_IWUSR, gtp_sysfs_rawdata_show, gtp_sysfs_rawdata_store);
static DEVICE_ATTR(diffdata, S_IRUGO|S_IWUSR, gtp_sysfs_diffdata_show, gtp_sysfs_diffdata_store);
static DEVICE_ATTR(referencedata, S_IRUGO|S_IWUSR, gtp_sysfs_referencedata_show, gtp_sysfs_referencedata_store);

s32 gtp_sysfs_init(void)
{
    s32 ret ;

 driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
 sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);

    goodix_debug_kobj = kobject_create_and_add("gtp", NULL) ;
    //SET_INFO_LINE_INFO("Starting initlizing gtp_debug_sysfs");
    if (goodix_debug_kobj == NULL)
    {
        GTP_ERROR("%s: subsystem_register failed\n", __func__);
        return -ENOMEM;
    }

    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_rawdata.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs_create_rawdata_file failed\n", __func__);
        return ret;
    }

    ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_diffdata.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs_create_diffdata_file failed\n", __func__);
        return ret;
    }
     ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_referencedata.attr);
    if (ret)
    {
        GTP_ERROR("%s: sysfs_create_diffdata_file failed\n", __func__);
        return ret;
    }

    GTP_INFO("Goodix debug sysfs create success!\n");
    return 0 ;
}

void gtp_sysfs_uninit(void)
{
    if (goodix_debug_kobj != NULL)
    {
        sysfs_remove_file(goodix_debug_kobj, &dev_attr_rawdata.attr);
        sysfs_remove_file(goodix_debug_kobj, &dev_attr_diffdata.attr);
        sysfs_remove_file(goodix_debug_kobj, &dev_attr_referencedata.attr);
        kobject_del(goodix_debug_kobj);
    }
}
#endif


/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int __init goodix_ts_init(void)
{
    s32 ret;

    GTP_DEBUG_FUNC();
#if 1
    if (1)
    {
        pr_err("GTP driver installing...\n");
        goodix_wq = create_singlethread_workqueue("goodix_wq");
        if (!goodix_wq)
        {
            GTP_ERROR("Creat workqueue failed.\n");
            return -ENOMEM;
        }
#if GTP_ESD_PROTECT
        INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
        gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
    }
#endif
    ret = i2c_add_driver(&goodix_ts_driver);

    return ret;
}

/*******************************************************
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    pr_err("GTP driver exited.\n");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }

}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
