/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
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
 * Version: 1.2   
 * Release Date:  2015/04/20
 */
 
#include <linux/irq.h>
#include "gt1x.h"
#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#include "circle_point.h"
#include <soc/oppo/boot_mode.h>


static DEFINE_SEMAPHORE(suspend_sem);//after suspend sucsess  , can start resume

static struct work_struct gt1x_work;
static struct input_dev *input_dev;
static struct workqueue_struct *gt1x_wq;
static const char *gt1x_ts_name = "goodix-ts";
static const char *input_dev_phys = "input/ts";
#ifdef GTP_CONFIG_OF
int gt1x_rst_gpio;
int gt1x_int_gpio;
int gt1x_enable2v8_gpio;
#endif
DEFINE_MUTEX(i2c_access);

//add  for gesture
#define SUPPORT_GESTURE
#define SUPPORT_REPORT_COORDINATE
struct Coordinate {
    int x;
    int y;
};
Point Point_input[64];
Point Point_output[4];
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
static uint32_t clockwise=1;

#ifdef SUPPORT_GESTURE
#define DTAP_DETECT          0xCC
#define UP_VEE_DETECT        0x76 
#define DOWN_VEE_DETECT      0x5e
#define LEFT_VEE_DETECT      0x3e 
#define RIGHT_VEE_DETECT     0x63
#define CIRCLE_DETECT        0x6f
#define DOUSWIP_DETECT       0x48 
#define DOUUPSWIP_DETECT     0x4E
#define RIGHT_SLIDE_DETECT   0xAA
#define LEFT_SLIDE_DETECT    0xbb
#define DOWN_SLIDE_DETECT    0xAB
#define UP_SLIDE_DETECT      0xBA
#define M_DETECT			 0x6D
#define W_DETECT			 0x77


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
#define CustomGestrue       14  //Custom

//static atomic_t double_enable;
atomic_t glove_mode_enable;
atomic_t is_in_suspend;
static uint32_t gesture;
extern int gesture_enabled;
extern bool is_gt1x_tp_charger;
#endif
#ifdef SUPPORT_GESTURE
struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
/*
static int init_goodix_proc(void);
static int tp_double_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
static int tp_double_write_func(struct file *file, const char *buffer, unsigned long count,void *data);
static int coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static int glove_mode_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
static int glove_mode_write_func(struct file *file,const char *buffer, unsigned long count,void *data);
static const struct file_operations gt1x_gesture = {
	.owner = THIS_MODULE,
	.read  = tp_double_read_func,
	.write = tp_double_write_func,
};
static const struct file_operations gt1x_gesture_coor = {
	.owner = THIS_MODULE,
	.read  = coordinate_proc_read_func,
};
static const struct file_operations gt1x_gesture_glove_mode= {
	.owner = THIS_MODULE,
	.read  = glove_mode_read_func,
	.write= glove_mode_write_func,
};
*/
#endif

static int gt1x_register_powermanger(void);
static int gt1x_unregister_powermanger(void);

/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

static spinlock_t irq_lock;
static s32 irq_is_disable = 0;

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(gt1x_i2c_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_enable - disable irq function.
 *
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

#ifndef GTP_CONFIG_OF
int gt1x_power_switch(s32 state)
{
    return 0;
}
#endif

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#if GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
//#error Need to get charger status of your platform.
	if(is_gt1x_tp_charger)
		{	
			//GTP_ERROR("gt1x_get_charger_status =1.");
			return 1;
		}
	else
		{
			//GTP_ERROR("gt1x_get_charger_status =0.");
			return 0;
	}
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_HANDLED: interrupt handled successfully
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	GTP_DEBUG_FUNC();
    gt1x_irq_disable();
	queue_work(gt1x_wq, &gt1x_work);
	return IRQ_HANDLED;
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(input_dev, ABS_MT_PRESSURE, size);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_func - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */

static void gt1x_ts_work_func(struct work_struct *work)
{
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };
	u8 coordinate_single[260];
	u8 coordinate_size;
	//u8 length = 0;
	u8 doze_buf[3];
	u8 clear_buf[1];
	int i = 0;
	int j = 0;
	//double clock=0;
    if (update_info.status) {
        GTP_DEBUG("Ignore interrupts during fw update.");
        return;
    }
    
#if GTP_GESTURE_WAKEUP
	if (DOZE_ENABLED == gesture_doze_status)
		{
        
				ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 2);
				GTP_DEBUG("0x814C = 0x%02X,ret=%d\n", doze_buf[0],ret);  
				if (ret == 0 )
				{  
				   
					if(doze_buf[0] != 0)
					{
					    memset(coordinate_single, 0, 260);
					    coordinate_size=doze_buf[1];
				    	ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE_DETAIL, coordinate_single, coordinate_size*4);
							//rendong.shi add gesture
							GTP_INFO("original  gesture is %d \n",doze_buf[0]);
							switch (doze_buf[0]) 
							{
								case DTAP_DETECT:
								gesture = DouTap;		
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x =  0; 
								Point_1st.y =  0; 
								clockwise = 0 ;
								break;

								case UP_VEE_DETECT :
								gesture = UpVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case DOWN_VEE_DETECT :
								gesture = DownVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case LEFT_VEE_DETECT:
								gesture =  LeftVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case RIGHT_VEE_DETECT :
								gesture =  RightVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;	
								
								case CIRCLE_DETECT  :
								gesture =  Circle;
								j = 0; 
								for(i = 0; i < coordinate_size;i++)
								{
									Point_input[i].x = coordinate_single[j]  |  (coordinate_single[j+1] << 8);
									Point_input[i].y = coordinate_single[j+2] |  (coordinate_single[j+3] << 8);
									j = j+4;
									GTP_INFO("Point_input[%d].x = %d,Point_input[%d].y = %d\n",i,Point_input[i].x,i,Point_input[i].y)	;					
								}

								//clockwise = cross(Point_input[0],Point_input[1],Point_input[2]);
								if(((Point_input[1].x - Point_input[0].x)*(Point_input[2].y - Point_input[1].y)-(Point_input[1].y - Point_input[0].y)*(Point_input[2].x)-Point_input[1].x)>0)
									clockwise =1;
								else
									clockwise =0;
								GetCirclePoints(&Point_input[0], coordinate_size,Point_output);
								Point_start.x = Point_input[0].x;
								Point_start.y = Point_input[0].y;

								Point_end.x = Point_input[coordinate_size-1].x;
								Point_end.y = Point_input[coordinate_size-1].y;

								Point_1st.x = Point_output[0].x;
								Point_1st.y = Point_output[0].y;
						
								Point_2nd.x = Point_output[1].x;
								Point_2nd.y = Point_output[1].y;
						
								Point_3rd.x = Point_output[2].x;
								Point_3rd.y = Point_output[2].y;
						
								Point_4th.x = Point_output[3].x;
								Point_4th.y = Point_output[3].y;
								break;
								
								case DOUSWIP_DETECT  :
								gesture =  DouSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;
							
								case DOUUPSWIP_DETECT:
								gesture =  DouSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;
								
								case RIGHT_SLIDE_DETECT :
								gesture =  Left2RightSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							    break;
								
								case LEFT_SLIDE_DETECT :
								gesture =  Right2LeftSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case DOWN_SLIDE_DETECT  :
								gesture =  Up2DownSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							    break;
								
								case UP_SLIDE_DETECT :
							    gesture =  Down2UpSwip;
							    Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;	
								
								case M_DETECT  :
								gesture =  Mgestrue;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
							    break;
								
								case W_DETECT :
								gesture =  Wgestrue;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;	
								
								default:
								if((doze_buf[0]>=1)&&(doze_buf[0]<=15))
							{
								gesture =  doze_buf[0] + OPPO_CUSTOM_GESTURE_ID_BASE;  //user custom gesture 
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_2nd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								Point_3rd.x = coordinate_single[16] |  (coordinate_single[17] << 8);
								Point_3rd.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_3rd.x = coordinate_single[20] |  (coordinate_single[21] << 8);
								Point_3rd.y = coordinate_single[22] |  (coordinate_single[23] << 8); 
							}
							else
							{
								gesture =  doze_buf[0];
							}
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

					//		if( (gesture==LeftVee) || (gesture==RightVee) || (gesture==DouSwip)  )
					//		{
					//			GTP_INFO("gesture=LeftVeetp  %d go to doze\n",gesture);
					//			gesture_enter_doze();
					//		} 

							if(gesture > OPPO_CUSTOM_GESTURE_ID_BASE )
							{
								ret = gesture_event_handler(input_dev);
								if (ret >= 0) 
								{
									gt1x_irq_enable();
									mutex_unlock(&i2c_access);
									return;
								}
							}
							else
							{
								GTP_INFO("F4 report!!!!!\n");
								input_report_key(input_dev, KEY_F4, 1);
								input_sync(input_dev);
								input_report_key(input_dev, KEY_F4, 0);
								input_sync(input_dev);
								clear_buf[0] = 0x00;
								gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);

							}

					}
					else       
					{
					        GTP_DEBUG(" gesture code =%d \n",doze_buf[0]);
					     	//clear_buf[0] = 0x00;
							//gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);	
							//gesture_enter_doze();
					}	
				}
             gt1x_irq_enable();
		     mutex_unlock(&i2c_access);
            return;
       }
		/*
	ret = gesture_event_handler(input_dev);
	if (ret >= 0) {
		goto exit_work_func;
	}
	*/
#endif

	if (gt1x_halt) {
		GTP_DEBUG("Ignore interrupts after suspend...");
        return;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
		gt1x_power_reset();
#endif
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00) {
		gt1x_request_event_handler();
	}

	if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			//GTP_ERROR("buffer not ready:0x%02x", finger);
			goto exit_eint;
		}
	}
#if HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret) {
		goto exit_work_func;
	}
#endif

#if GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0) {
		goto exit_work_func;
	}
#endif

#if GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0) {
			GTP_ERROR("I2C write end_cmd  error!");
		}
	}
exit_eint:
    gt1x_irq_enable();
    
}

/* 
 * Devices Tree support, 
*/
#ifdef GTP_CONFIG_OF
/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static void gt1x_parse_dt(struct device *dev)
{
	struct device_node *np;
	int rc =0;
    if (!dev) {
        return;
    }

    np = dev->of_node;
	gt1x_int_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	gt1x_rst_gpio = of_get_named_gpio(np, "goodix,rst-gpio", 0);
	gt1x_enable2v8_gpio = of_get_named_gpio(np, "goodix,enable2v8-gpio", 0);
		if( gt1x_enable2v8_gpio < 0 ){
		GTP_DEBUG("gt1x_enable2v8_gpio not specified\n");
			}
		else
			GTP_DEBUG("gt1x_enable2v8_gpio find\n");
		if( gt1x_enable2v8_gpio > 0){
		if( gpio_is_valid(gt1x_enable2v8_gpio) ){
			rc = gpio_request(gt1x_enable2v8_gpio, "goodix-enable2v8-gpio");
			if(rc){
				GTP_DEBUG("unable to request gpio [%d]\n", gt1x_enable2v8_gpio);
			}
		}
	}
		/*
		if( gt1x_rst_gpio > 0){
		if( gpio_is_valid(gt1x_rst_gpio) ){
			rc = gpio_request(gt1x_rst_gpio, "goodix-rst-gpio");
			if(rc){
				GTP_DEBUG("unable to request gpio [%d]\n", gt1x_rst_gpio);
			}
		}
	}
		if( gt1x_int_gpio > 0){
		if( gpio_is_valid(gt1x_int_gpio) ){
			rc = gpio_request(gt1x_int_gpio, "goodix-irq-gpio");
			if(rc){
				GTP_DEBUG("unable to request gpio [%d]\n", gt1x_int_gpio);
			}
		}
	}
		*/
}

/**
 * gt1x_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int gt1x_power_switch(int on)
{
	//static struct regulator *vdd_ana;
	static struct regulator *vcc_i2c;
	int ret;
	struct i2c_client *client = gt1x_i2c_client;

    if (!client)
        return -1;
/*	
	if (!vdd_ana) {
		vdd_ana = regulator_get(&client->dev, "vdd_ana");
		if (IS_ERR(vdd_ana)) {
			GTP_ERROR("regulator get of vdd_ana failed");
			ret = PTR_ERR(vdd_ana);
			vdd_ana = NULL;
			return ret;
		}
	}
	*/

	if (!vcc_i2c) {
		vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(vcc_i2c)) {
			GTP_ERROR("regulator get of vcc_i2c failed");
			ret = PTR_ERR(vcc_i2c);
			vcc_i2c = NULL;
			goto ERR_GET_VCC;
		}
	}

	


	if (on) {
		GTP_DEBUG("GTP power on.");
		//ret = regulator_enable(vdd_ana);
		//GTP_GPIO_OUTPUT(gt1x_enable2v8_gpio, 1);
		gpio_direction_output(gt1x_enable2v8_gpio, 1);
		udelay(2);
		ret = regulator_enable(vcc_i2c);
	} else {
		GTP_DEBUG("GTP power off.");
		ret = regulator_disable(vcc_i2c);
		udelay(2);
		//GTP_GPIO_OUTPUT(gt1x_enable2v8_gpio, 0);
		gpio_direction_output(gt1x_enable2v8_gpio, 0);
		//ret = regulator_disable(vdd_ana);
	}
	return ret;
	
ERR_GET_VCC:
	//regulator_put(vdd_ana);
	return ret;
	
}
#endif

/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = GTP_INT_IRQ;
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	if (ret < 0) {
		gpio_free(GTP_RST_PORT);
		gpio_free(GTP_INT_PORT);
	}

	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);

	ret = request_irq(gt1x_i2c_client->irq, gt1x_ts_irq_handler, irq_table[gt1x_int_type], gt1x_i2c_client->name, gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		gpio_free(GTP_INT_PORT);

		return -1;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
    input_mt_init_slots(input_dev, 16, INPUT_MT_DIRECT);
#else
    input_mt_init_slots(input_dev, 16); 
#endif
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_GES_REGULAR);
    input_set_capability(input_dev, EV_KEY, KEY_GES_CUSTOM);
#endif

#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = gt1x_ts_name;
	input_dev->phys = input_dev_phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}

/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
 
s32 gt1x_init_glovemode_panel(void)
{
	s32 ret = 0;
	u8 cfg_len = 0;
    //u8 buf;
#if GTP_DRIVER_SEND_CFG
	u8 sensor_id = 0;
	 u8 cfg_grp0[] = GTP_CFG_GROUP0_GLOVE;
	 u8 cfg_grp1[] = GTP_CFG_GROUP1_GLOVE;
	 u8 cfg_grp2[] = GTP_CFG_GROUP2_GLOVE;
	 u8 cfg_grp3[] = GTP_CFG_GROUP3_GLOVE;
	 u8 cfg_grp4[] = GTP_CFG_GROUP4_GLOVE;
	 u8 cfg_grp5[] = GTP_CFG_GROUP5_GLOVE;
	 u8 *cfgs[] = {
		cfg_grp0, cfg_grp1, cfg_grp2,
		cfg_grp3, cfg_grp4, cfg_grp5
	};
	u8 cfg_lens[] = {
		CFG_GROUP_LEN(cfg_grp0),
		CFG_GROUP_LEN(cfg_grp1),
		CFG_GROUP_LEN(cfg_grp2),
		CFG_GROUP_LEN(cfg_grp3),
		CFG_GROUP_LEN(cfg_grp4),
		CFG_GROUP_LEN(cfg_grp5)
	};

#if GTP_CHARGER_SWITCH
	 u8 cfg_grp0_charger[] = GTP_CHARGER_CFG_GROUP0;
	 u8 cfg_grp1_charger[] = GTP_CHARGER_CFG_GROUP1;
	 u8 cfg_grp2_charger[] = GTP_CHARGER_CFG_GROUP2;
	 u8 cfg_grp3_charger[] = GTP_CHARGER_CFG_GROUP3;
	 u8 cfg_grp4_charger[] = GTP_CHARGER_CFG_GROUP4;
	 u8 cfg_grp5_charger[] = GTP_CHARGER_CFG_GROUP5;
	 u8 *cfgs_charger[] = {
		cfg_grp0_charger, cfg_grp1_charger, cfg_grp2_charger,
		cfg_grp3_charger, cfg_grp4_charger, cfg_grp5_charger
	};
	u8 cfg_lens_charger[] = {
		CFG_GROUP_LEN(cfg_grp0_charger),
		CFG_GROUP_LEN(cfg_grp1_charger),
		CFG_GROUP_LEN(cfg_grp2_charger),
		CFG_GROUP_LEN(cfg_grp3_charger),
		CFG_GROUP_LEN(cfg_grp4_charger),
		CFG_GROUP_LEN(cfg_grp5_charger)
	};
#endif /* end  GTP_CHARGER_SWITCH */
	GTP_DEBUG("Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens[0], cfg_lens[1], cfg_lens[2], cfg_lens[3], cfg_lens[4], cfg_lens[5]);
	sensor_id = gt1x_version.sensor_id;
	if (sensor_id >= 6 || cfg_lens[sensor_id] < GTP_CONFIG_MIN_LENGTH || cfg_lens[sensor_id] > GTP_CONFIG_MAX_LENGTH) {
		sensor_id = 0;
	}
	cfg_len = cfg_lens[sensor_id];
   
	if (cfg_len < GTP_CONFIG_MIN_LENGTH || cfg_len > GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id + 1);
		return -1;
	}

	gt1x_irq_disable();
	memset(gt1x_config, 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(gt1x_config, cfgs[sensor_id], cfg_len);
     
	/* clear the flag, avoid failure when send the_config of driver. */
	gt1x_config[0] &= 0x7F;

#if GTP_CUSTOM_CFG
	gt1x_config[RESOLUTION_LOC] = (u8) GTP_MAX_WIDTH;
	gt1x_config[RESOLUTION_LOC + 1] = (u8) (GTP_MAX_WIDTH >> 8);
	gt1x_config[RESOLUTION_LOC + 2] = (u8) GTP_MAX_HEIGHT;
	gt1x_config[RESOLUTION_LOC + 3] = (u8) (GTP_MAX_HEIGHT >> 8);

	GTP_INFO("Res: %d * %d, trigger: %d", GTP_MAX_WIDTH, GTP_MAX_HEIGHT, GTP_INT_TRIGGER);

	if (GTP_INT_TRIGGER == 0) {	/* RISING  */
		gt1x_config[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		gt1x_config[TRIGGER_LOC] |= 0x01;
	}
#endif /* END GTP_CUSTOM_CFG */

#if GTP_CHARGER_SWITCH
	GTP_DEBUG("Charger Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens_charger[0],
			cfg_lens_charger[1], cfg_lens_charger[2], cfg_lens_charger[3], cfg_lens_charger[4], cfg_lens_charger[5]);
	memset(gt1x_config_charger, 0, sizeof(gt1x_config_charger));
	if (cfg_lens_charger[sensor_id] == cfg_len) {
		memcpy(gt1x_config_charger, cfgs_charger[sensor_id], cfg_len);
	}
	gt1x_config_charger[0] &= 0x7F;
#if GTP_CUSTOM_CFG
	gt1x_config_charger[RESOLUTION_LOC] = (u8) GTP_MAX_WIDTH;
	gt1x_config_charger[RESOLUTION_LOC + 1] = (u8) (GTP_MAX_WIDTH >> 8);
	gt1x_config_charger[RESOLUTION_LOC + 2] = (u8) GTP_MAX_HEIGHT;
	gt1x_config_charger[RESOLUTION_LOC + 3] = (u8) (GTP_MAX_HEIGHT >> 8);
	if (GTP_INT_TRIGGER == 0) {	/* RISING  */
		gt1x_config_charger[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		gt1x_config_charger[TRIGGER_LOC] |= 0x01;
	}
#endif /* END GTP_CUSTOM_CFG */
	if (cfg_lens_charger[sensor_id] != cfg_len) {
		memset(gt1x_config_charger, 0, sizeof(gt1x_config_charger));
	}
#endif /* END GTP_CHARGER_SWITCH */

#else /* DRIVER NOT SEND CONFIG */
	cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gt1x_i2c_read(GTP_REG_CONFIG_DATA, gt1x_config, cfg_len);
	if (ret < 0) {
		gt1x_irq_enable();	
		return ret;
	}
#endif /* END GTP_DRIVER_SEND_CFG */

	//GTP_DEBUG_FUNC();
	/* match resolution when gt1x_abs_x_max & gt1x_abs_y_max have been set already */
	if ((gt1x_abs_x_max == 0) && (gt1x_abs_y_max == 0)) {
		gt1x_abs_x_max = (gt1x_config[RESOLUTION_LOC + 1] << 8) + gt1x_config[RESOLUTION_LOC];
		gt1x_abs_y_max = (gt1x_config[RESOLUTION_LOC + 3] << 8) + gt1x_config[RESOLUTION_LOC + 2];
		gt1x_int_type = (gt1x_config[TRIGGER_LOC]) & 0x03;
//VENDOR_EDIT delete for suspend current 		gt1x_wakeup_level = !(gt1x_config[MODULE_SWITCH3_LOC] & 0x20);
	} else {
		gt1x_config[RESOLUTION_LOC] = (u8) gt1x_abs_x_max;
		gt1x_config[RESOLUTION_LOC + 1] = (u8) (gt1x_abs_x_max >> 8);
		gt1x_config[RESOLUTION_LOC + 2] = (u8) gt1x_abs_y_max;
		gt1x_config[RESOLUTION_LOC + 3] = (u8) (gt1x_abs_y_max >> 8);
		set_reg_bit(gt1x_config[MODULE_SWITCH3_LOC], 5, !gt1x_wakeup_level);
		gt1x_config[TRIGGER_LOC] = (gt1x_config[TRIGGER_LOC] & 0xFC) | gt1x_int_type;
#if GTP_CHARGER_SWITCH
		gt1x_config_charger[RESOLUTION_LOC] = (u8) gt1x_abs_x_max;
		gt1x_config_charger[RESOLUTION_LOC + 1] = (u8) (gt1x_abs_x_max >> 8);
		gt1x_config_charger[RESOLUTION_LOC + 2] = (u8) gt1x_abs_y_max;
		gt1x_config_charger[RESOLUTION_LOC + 3] = (u8) (gt1x_abs_y_max >> 8);
		set_reg_bit(gt1x_config[MODULE_SWITCH3_LOC], 5, !gt1x_wakeup_level);
		gt1x_config[TRIGGER_LOC] = (gt1x_config[TRIGGER_LOC] & 0xFC) | gt1x_int_type;
#endif
	}
    
	//GTP_INFO("X_MAX=%d,Y_MAX=%d,TRIGGER=0x%02x,WAKEUP_LEVEL=%d", gt1x_abs_x_max, gt1x_abs_y_max, gt1x_int_type, gt1x_wakeup_level);

	gt1x_cfg_length = cfg_len;
	ret = gt1x_send_cfg(gt1x_config, gt1x_cfg_length);

#ifdef VENDOR_EDIT 
	//mingqiang.guo add for TP FW  config ID 
	ret = gt1x_i2c_read(GTP_REG_CONFIG_DATA, &cfg_len,1);
	//TP_FW_CONFIG_ID = cfg_len ;
	//GTP_DEBUG("TP_FW = 0x%x ,  TP_FW_CONFIG_ID = 0x%x \n",TP_FW, TP_FW_CONFIG_ID);
#endif //VENDOR_EDIT  
    gt1x_irq_enable();	
	return ret;
}
#ifdef SUPPORT_GESTURE

static ssize_t tp_double_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{                           
    char pagesize[512];
    int len = 0;
	GTP_INFO("double tap enable is: %d\n", gesture_enabled);
	len = sprintf(pagesize, "%d\n", gesture_enabled);
	len = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize)); 
	return len; 
}

static ssize_t tp_double_write_func(struct file *file,const char *buffer, size_t count, loff_t *data)
{
	int ret = 0;

	char buf[10] = {0};
	static int in_suspend_gustrue_status;
	
	if (count > 10) 
		return count;

	if (copy_from_user( buf, buffer, count)) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	 
	sscanf(buf,"%d",&ret);

	down(&suspend_sem);
	GTP_INFO("double_write_func  %d\n",ret);
	if( atomic_read(&is_in_suspend) )
	{
		if(in_suspend_gustrue_status == ret)
		{
			GTP_INFO("do not need operate when gesture status is same\n");
			up(&suspend_sem);
			return count;
		}

		in_suspend_gustrue_status = ret; 
		switch(ret)
		{
			case 0:
				gt1x_wakeup_sleep_gesture();
				gt1x_enter_sleep_gesture();
				break;
			case 1:
				gt1x_wakeup_sleep_gesture();
				gesture_enter_doze_gesture();
				break;


			default:
				GTP_INFO("Please enter 0 or 1 to open or close the double-tap function\n");
		}
		up(&suspend_sem);
		return count;//can not save double_enable flag when is_in_suspend
		
	}
	else
	{
		switch(ret)
		{	
			case 0:
				GTP_DEBUG("tp_guesture_func will be disable\n");
				break;
			case 1:
				GTP_DEBUG("tp_guesture_func will be enable\n");
				break;
			default:
				GTP_DEBUG("Please enter 0 or 1 to open or close the double-tap function\n");
		}
	}
	up(&suspend_sem);

	if((ret == 0 )||(ret == 1))
	{
		//atomic_set(&gesture_enabled,ret);
		gesture_enabled =ret;
		in_suspend_gustrue_status = ret;
	}

	return count;
}

#ifdef SUPPORT_REPORT_COORDINATE
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char page[512];

	len =sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	GTP_INFO("%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);
   printk("return ret=%d len=%d\n",ret,len);

	return ret;
}
#endif

#endif
static ssize_t glove_mode_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{                           
    char pagesize[512];
    int len = 0;
	GTP_INFO("glove_mode enable is: %d\n", atomic_read(&glove_mode_enable));
	len = sprintf(pagesize, "%d\n", atomic_read(&glove_mode_enable));
	len = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize)); 
	return len; 
}

static ssize_t glove_mode_write_func(struct file *file,const char *buffer, size_t count,loff_t *data)
{

	int ret = 0;

	char buf[10] = {0};

	if (count > 10) 
		return count;

	if (copy_from_user( buf, buffer, count)) {
		GTP_INFO(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf,"%d",&ret);
	GTP_DEBUG("%s :buf = %d,ret = %d\n",__func__,*buf,ret);
	if( ret ==  atomic_read(&glove_mode_enable) )
	{
		GTP_INFO("glove status is same , not need operate\n");
		return count;
	}
	
	if((ret == 0 )||(ret == 1))
	{
		atomic_set(&glove_mode_enable,ret);
		if(  1 ==  atomic_read(&is_in_suspend) )
		{
			GTP_DEBUG("when in suspend , operate glove mode after resume\n");
			return count;
		}
	}

    down(&suspend_sem); 
	switch(ret)
	{	
		case 0:
			GTP_INFO("glove mode  will be disable\n");
			gt1x_init_panel();
			msleep(150);
			break;

		case 1:
			GTP_INFO(" glove mode will be enable\n");
			gt1x_init_glovemode_panel();
			msleep(150);
			break;

		default:
			GTP_INFO("Please enter 0 or 1 to open or close the double-tap function\n");
	}
    up(&suspend_sem); 

	return count;
}
static const struct file_operations gt1x_gesture = {
	//.owner = THIS_MODULE,
	.read  = tp_double_read_func,
	.write = tp_double_write_func,
};
static const struct file_operations gt1x_gesture_coor = {
	//.owner = THIS_MODULE,
	.read  = coordinate_proc_read_func,
};
static const struct file_operations gt1x_gesture_glove_mode= {
	//.owner = THIS_MODULE,
	.read  = glove_mode_read_func,
	.write= glove_mode_write_func,
};

static int init_goodix_proc(void)
{
	int ret = 0;
	
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create TP proc entry\n");
		return ret;
	}
	
#ifdef SUPPORT_GESTURE
	prEntry_dtap = proc_create("double_tap_enable", 0777, prEntry_tp, &gt1x_gesture);
	if(prEntry_dtap == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		return ret;
	}
	GTP_INFO("create gt1x gesture proc success\n");
	prEntry_coodinate =  proc_create("glove_mode_enable", 0777, prEntry_tp, &gt1x_gesture_glove_mode);
    if(prEntry_coodinate == NULL)
    {	   
		ret = -ENOMEM;	   
		GTP_ERROR(KERN_INFO"glove_mode_enable : Couldn't create proc entry\n");
		return ret;
    }
	GTP_INFO("create  glove_mode_enable  proc success\n");
#ifdef SUPPORT_REPORT_COORDINATE
	prEntry_coodinate =  proc_create("coordinate", 0777, prEntry_tp, &gt1x_gesture_coor);
    if(prEntry_coodinate == NULL)
    {	   
		ret = -ENOMEM;	   
		GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create proc entry\n");
		return ret;
    }
	GTP_INFO("create gt1x gesture_coor proc success\n");
    
#endif

#endif
	return ret;
}


static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	int boot_mode = 0;
#if GTP_AUTO_UPDATE
	struct task_struct *thread = NULL;
#endif
	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

    /*chaoying.chen@EXP.BaseDrv.TP,2015/07/14 modify  ftm for 15085 at sleep */
    boot_mode = get_boot_mode();
    	if( (boot_mode == MSM_BOOT_MODE__FACTORY || boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) ){
		GTP_ERROR("regulator_disable is called\n");
		
		return 0; 
	}
	gt1x_i2c_client = client;
	init_goodix_proc();
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

#ifdef GTP_CONFIG_OF	/* device tree support */
	if (client->dev.of_node) {
		gt1x_parse_dt(&client->dev);
	}
#endif

	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		return ret;
	}

	gt1x_init();
	//if( (boot_mode == MSM_BOOT_MODE__FACTORY || boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) ){
	//	GTP_ERROR("regulator_disable is called\n");
	//	gt1x_deinit();
   //     gt1x_power_switch(SWITCH_OFF);/*chaoying.chen@EXP.BaseDrv.TP,2015/07/14 modify  ftm for 15085 at sleep */
	//	return 0; /*chaoying.chen@EXP.BaseDrv.TP,2015/07/14 modify  ftm for 15085 at sleep */
	//}

	INIT_WORK(&gt1x_work, gt1x_ts_work_func);

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
	}

	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

#if GTP_GESTURE_WAKEUP
	enable_irq_wake(client->irq);
#endif

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	// must before auto update
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		GTP_ERROR("Failed to create auto-update thread: %d.", ret);
	}
#endif
	gt1x_register_powermanger();
	return 0;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver removing...");
	gt1x_unregister_powermanger();

#if GTP_GESTURE_WAKEUP
	disable_irq_wake(client->irq);
#endif
    gt1x_deinit();
	input_unregister_device(input_dev);

    return 0;
}

#if   defined(CONFIG_FB)	
/* frame buffer notifier block control the suspend/resume procedure */
static struct notifier_block gt1x_fb_notifier;

static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;
	
	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			gt1x_resume();
				
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspend by fb notifier.");
			gt1x_suspend();
		}
	}

	return 0;
}
#elif defined(CONFIG_PM)
/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_suspend(struct device *dev)
{
    return gt1x_suspend();
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_resume(struct device *dev)
{
	return gt1x_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gt1x_ts_pm_ops = {
	.suspend = gt1x_pm_suspend,
	.resume = gt1x_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_suspend();
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_resume();
}

static struct early_suspend gt1x_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = gt1x_ts_early_suspend,
	.resume = gt1x_ts_late_resume,
};
#endif


static int gt1x_register_powermanger(void)
{
#if   defined(CONFIG_FB)
	gt1x_fb_notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&gt1x_fb_notifier);
	
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&gt1x_early_suspend);
#endif	
	return 0;
}

static int gt1x_unregister_powermanger(void)
{
#if   defined(CONFIG_FB)
	fb_unregister_client(&gt1x_fb_notifier);
		
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

#ifdef GTP_CONFIG_OF
static const struct of_device_id gt1x_match_table[] = {
		{.compatible = "goodix,gt1x",},
		{ },
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
#ifdef GTP_CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		   .pm = &gt1x_ts_pm_ops,
#endif
		   },
};

/**   
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int __init gt1x_ts_init(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	gt1x_wq = create_singlethread_workqueue("gt1x_wq");
	if (!gt1x_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}

	return i2c_add_driver(&gt1x_ts_driver);
}

/**   
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
	if (gt1x_wq) {
		destroy_workqueue(gt1x_wq);
	}
}

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
