/*******************************************************************************
* Copyright (c)  2014- 2014  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* CONFIG_MACH_OPPO
* Description: Source file for CBufferList.
*           To allocate and free memory block safely.
* Version   : 0.0
* Date      : 2014-07-30
* Author    : Lijiada @Bsp.charge
* ---------------------------------- Revision History: -------------------------
* <version>           <date>          < author >              <desc>
* Revision 0.0        2014-07-30      Lijiada @Bsp.charge
* Modified to be suitable to the new coding rules in all functions.
*******************************************************************************/

#ifndef _OPPO_DEF_H_
#define _OPPO_DEF_H_

enum {
    OPCHG_CHG_TEMP_PRESENT = 0,
    OPCHG_CHG_TEMP_COLD,			//	t< -3
    OPCHG_CHG_TEMP_COOL,			//	-3< t <0
    OPCHG_CHG_TEMP_PRE_COOL1,		//	0< t <5
    OPCHG_CHG_TEMP_PRE_COOL,		//	5< t <12
    OPCHG_CHG_TEMP_PRE_NORMAL,		//	12< t <22
    OPCHG_CHG_TEMP_NORMAL,			//	22< t <45
    OPCHG_CHG_TEMP_WARM,			//	45< t <53
    OPCHG_CHG_TEMP_HOT,				//	>53
};

// batterynotify is int type ,so define max bit(14)
#define 	Notify_Charger_Over_Vol                   	BIT(1)// 1 // charger voltage high
#define	Notify_Charger_Low_Vol                    	BIT(2)// 2 // charger voltage low
#define	Notify_Bat_Over_Temp                      	BIT(3)// 3 // battery temp high
#define	Notify_Bat_Low_Temp                       	BIT(4)// 4 // battery temp low
#define	Notify_Bat_Not_Connect                    	BIT(5)// 5 // battery disconnect
#define	Notify_Bat_Over_Vol                       	BIT(6)// 6 // battery voltage high
#define	Notify_Bat_Full                           	BIT(7)// 7 // normal charger full
#define	Notify_Chging_Current                     	BIT(8)// 8
#define	Notify_Chging_OverTime					  	BIT(9)// 9 // time out charegr full
#define	Notify_Bat_Full_High_Temp			  			BIT(10)// 10// high_temp charger full
#define	Notify_Bat_Full_Low_Temp			  			BIT(11)// 11// low_temp charger full
#define	Notify_Bat_Full_THIRD_BATTERY					BIT(14)// 14   // no_thirdbat charger full
#define	Notify_Bat_MAX								BIT(14)// 14

typedef enum
{
	/*! Battery is absent               */
    CV_BATTERY_TEMP_REGION__ABSENT,
    /*! Battery is cold               */
    CV_BATTERY_TEMP_REGION__COLD,		//	t< -10
    /*! Battery is little cold        */
    CV_BATTERY_TEMP_REGION__LITTLE_COLD,//	-10< t <0
    /*! Battery is cool              */
    CV_BATTERY_TEMP_REGION__COOL,		//	0< t <10
    /*! Battery is cool               */
    CV_BATTERY_TEMP_REGION__LITTLE_COOL,	//	10< t <20
    /*! Battery is normal             */
    CV_BATTERY_TEMP_REGION__NORMAL,		//	20< t <45
    /*! Battery is warm               */
    CV_BATTERY_TEMP_REGION__WARM,		//	45< t <55
    /*! Battery is hot                */
    CV_BATTERY_TEMP_REGION__HOT,		//	>55
    /*! Invalid battery temp region   */
    CV_BATTERY_TEMP_REGION__INVALID,
} chg_cv_battery_temp_region_type;

enum {
    INPUT_CURRENT_MIN,
    INPUT_CURRENT_BY_POWER,
    INPUT_CURRENT_BY_FASTER_2PHASE,
    INPUT_CURRENT_BY_VOOC,
    INPUT_CURRENT_LCD,
    INPUT_CURRENT_CAMERA,
    INPUT_CURRENT_CMCC,
    INPUT_CURRENT_MAX
};

enum {
    FAST_CURRENT_MIN,
    FAST_CURRENT_TEMP,
    FAST_CURRENT_2CHARGER,
    FAST_CURRENT_LCD,
    FAST_CURRENT_CAMERA,
    FAST_CURRENT_COOL_TEMP,
    FAST_CURRENT_CMCC,
    FAST_CURRENT_MAX
};

enum {
    TERM_VOL_MIN,
    TERM_VOL_TEMP,
    TERM_VOL_FULL,
    TERM_VOL_BAT,
    TERM_VOL_TAPER_PHASE,
    TERM_VOL_CUSTOM1,
    TERM_VOL_CUSTOM2,
    TERM_VOL_MAX
};

enum {
    TERM_CURRENT_MIN,
    TERM_CURRENT_NORMAL,
    TERM_CURRENT_TAPER_PHASE,
    TERM_CURRENT_MAX
};

enum {
    USER_DISABLE        = BIT(0),
    THERMAL_DISABLE     = BIT(1),
    FACTORY_MODE_DISABLE     = BIT(2),//CURRENT_DISABLE     = BIT(2),
    CHAGER_ERR_DISABLE  = BIT(3),
    CHAGER_OUT_DISABLE  = BIT(4),
    CHAGER_OTG_DISABLE  = BIT(5),
    CHAGER_VOOC_DISABLE = BIT(6),
    CHAGER_TIMEOUT_DISABLE = BIT(7),
    CHAGER_RECHARGER_DISABLE = BIT(8),
};

enum {
    CHARGER_RESET_BY_TEMP_COOL      = BIT(0),
	CHARGER_RESET_BY_TEMP_PRE_COOL1  = BIT(1),
    CHARGER_RESET_BY_TEMP_PRE_COOL  = BIT(2),
    CHARGER_RESET_BY_TEMP_PRE_NORMAL = BIT(3),
    CHARGER_RESET_BY_TEMP_NORMAL    = BIT(4),
    CHARGER_RESET_BY_TEMP_WARM      = BIT(5),
    CHARGER_RESET_BY_VOOC           = BIT(6),
};

enum {
    FACTORY_ENABLE      = BIT(0),
    CHAGER_ERR_ENABLE   = BIT(1),
};

enum {
	OVERTIME_AC = 0,
	OVERTIME_USB,
	OVERTIME_DISABLED,
};

enum {
    PRE_PHASE,
    CC_PHASE,
    CV_PHASE,
    TERM_PHASE,
    RECHARGE_PHASE,
    ERRO_PHASE,
};

enum {
    FASTER_NONE,
    FASTER_1PHASE,
    FASTER_2PHASE,
};

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC 	= BIT(3),
};

#define CONFIG_GAUGE_BQ27411						1
#define OPCHG_THREAD_INTERVAL                   5000//5S
#define OPCHG_THREAD_INTERVAL_INIT              1000//1S

/* constants */
#define USB2_MIN_CURRENT_MA                     100
#define USB2_MAX_CURRENT_MA                     500
#define USB3_MAX_CURRENT_MA                     900

#define OPCHG_FAST_CHG_MAX_MA                   2000
#define OPCHG_IMPUT_CURRENT_LIMIT_MAX_MA        2000


struct opchg_regulator {
    struct regulator_desc           rdesc;
    struct regulator_dev            *rdev;
};

struct opchg_charger {
    struct i2c_client               *client;
    struct device                   *dev;
    struct mutex                    read_write_lock;
    bool                            charger_inhibit_disabled;
	bool							recharge_disabled;
 //   int                             recharge_mv;
    int                             float_compensation_mv;
    bool                            iterm_disabled;
 //   int                             iterm_ma;
 //   int                             vfloat_mv;
    int                             chg_valid_gpio;
    int                             chg_valid_act_low;
    int                             chg_present;
	bool							chg_present_real;
    int                             fake_battery_soc;
    bool                            chg_autonomous_mode;
    bool                            disable_apsd;
    bool                            battery_missing;
    const char                      *bms_psy_name;
	bool							bms_controlled_charging;
	bool							using_pmic_therm;
	int								charging_disabled_status;

    /* status tracking */
	bool                            batt_ovp;
    bool                            batt_pre_full;
	bool							batt_pre_full_smb358;
    bool                            batt_full;
    bool                            batt_hot;
    bool                            batt_cold;
    bool                            batt_warm;
	bool                            batt_normal;
    bool                            batt_cool;
    bool                            charge_voltage_over;
    bool                            batt_voltage_over;
	int								batterynotify;
#ifdef CONFIG_MACH_OPPO
    bool                            multiple_test;
#endif
    bool                            suspending;
    #if 0
    bool                            action_pending;
    #endif
    bool                            otg_enable_pending;
    bool                            otg_disable_pending;

    int                             charging_disabled;
 //   int                             fastchg_current_max_ma;
    int                             fastchg_current_ma;
 //   int                             limit_current_max_ma;
    int                             faster_normal_limit_current_max_ma;
    bool                            charging_time_out;
    int                             charging_total_time;
    int                             charging_opchg_temp_statu;
    int                             temp_vfloat_mv;
    int                             workaround_flags;

	struct power_supply				dc_psy;
    struct power_supply             *usb_psy;
    struct power_supply             *bms_psy;
    struct power_supply             batt_psy;

    struct delayed_work             update_opchg_thread_work;
    struct delayed_work             opchg_delayed_wakeup_work;
	struct work_struct				opchg_modify_tp_param_work;
	struct work_struct				bq24196_usbin_valid_work;
	struct work_struct				bq24157_usbin_valid_work;
	struct work_struct				bq24188_usbin_valid_work;
    struct wakeup_source            source;

    struct opchg_regulator         otg_vreg;

    struct dentry                   *debug_root;
    u32                             peek_poke_address;

    struct qpnp_vadc_chip           *vadc_dev;
    struct qpnp_adc_tm_chip         *adc_tm_dev;
    struct qpnp_adc_tm_btm_param    adc_param;

	int								fast_charge_project;

    int                             fastchg_current_max_ma;
    int                             limit_current_max_ma;
    int                             iterm_ma;
    int                             vfloat_mv;
    int                             recharge_mv;

    int                             hot_bat_decidegc;
    int                             temp_hot_vfloat_mv;
    int                             temp_hot_fastchg_current_ma;

    int                             warm_bat_decidegc;
    int                             temp_warm_vfloat_mv;
    int                             temp_warm_fastchg_current_ma;

	int                             pre_normal_bat_decidegc;
    int                             temp_pre_normal_vfloat_mv;
    int                             temp_pre_normal_fastchg_current_ma;

    int                             pre_cool_bat_decidegc;
    int                             temp_pre_cool_vfloat_mv;
    int                             temp_pre_cool_fastchg_current_ma;
    int                             pre_cool1_bat_decidegc;
    int                             temp_pre_cool1_vfloat_mv;
    int                             temp_pre_cool1_fastchg_current_ma;

    int                             cool_bat_decidegc;
    int                             temp_cool_vfloat_mv;
    int                             temp_cool_fastchg_current_ma;

    int                             cold_bat_decidegc;
    int                             bat_present_decidegc;
	int								non_standard_vfloat_mv;
	int								non_standard_fastchg_current_ma;

	int								check_term_voltage_count;
	int                             pre_full_term_vfloat_mv;
	int								vfloat_new;


    struct regulator*               vcc_i2c;
    int                             irq_gpio;
	int								usbin_switch_gpio;
	int								batt_id_gpio;
	bool							batt_authen;
    //int                             usbphy_on_gpio;
    int                             fastcharger;
	//int								fast_charge_project;
    int                             driver_id;
    bool                            g_is_changed;
    int                             g_is_vooc_changed;
 //   int                             vooc_start_step;
	bool								opchg_earphone_enable;
    int                             g_is_reset_changed;
    int                             max_input_current[INPUT_CURRENT_MAX+1];;
    int                             max_fast_current[FAST_CURRENT_MAX+1];
    int                             max_term_current[TERM_CURRENT_MAX+1];
    int                             min_term_voltage[TERM_VOL_MAX+1];
    int                             disabled_status;
    int                             reseted_status;
    int                            	suspend_status;
    int                             overtime_status;
    int                             charging_phase;
    int                             fastcharger_type;
    int                             fastcharger_status;
    int                             pre_fastcharger_status;
    int                             taper_vfloat_mv;
    int                             fast_taper_fastchg_current_ma;
    int                             charger_ov_status;
    int                             g_is_wakeup;
    int                             g_chg_in;
    u8                              bat_temp_status;
    int                             temperature;
    int                             bat_instant_vol;
    int                             charging_current;
    int                             charger_vol;
	int								battery_vol;
	u8								soc_bms;
    u8                              bat_volt_check_point;
	int								ocv_uv;
    bool                            is_charging;
    bool                            bat_exist;
    u8                              bat_status;
    u8                              bat_charging_state;
    u8                              battery_request_poweroff;
    bool                            is_charger_det;

	int                             charger_type;
	int								battery_low_vol;
	int								boot_mode;
#ifdef OPPO_USE_FAST_CHARGER_RESET_MCU
	int 							   fast_charger_reset_count;
	int 							   fast_charger_reset_sign;
	int 							   fast_charger_disable_sign;
#endif
	bool                            is_lcd_on;
	bool                            is_camera_on;
	bool                            is_factory_mode;
	//Liao Fuchun add for bq24196 wdt enable/disable
	bool							wdt_enable;
	#if 0
	//Liao Fuchun add for batt temp
	int								little_cool_bat_decidegc;
	int								normal_bat_decidegc;
	int								temp_cold_vfloat_mv;
	int								temp_cold_fastchg_current_ma;
	int								temp_little_cool_vfloat_mv;
	int								temp_little_cool_fastchg_current_ma;
	int								temp_normal_vfloat_mv;
	int								temp_normal_fastchg_current_ma;
	int								mBatteryTempRegion;
	int								mBatteryTempBoundT0;
	int								mBatteryTempBoundT1;
	int								mBatteryTempBoundT2;
	int								mBatteryTempBoundT3;
	int								mBatteryTempBoundT4;
	int								mBatteryTempBoundT5;
	int								mBatteryTempBoundT6;
	#endif
	int								aicl_current;
	bool							aicl_working;
	int								aicl_delay_count;

	int 							vindpm_vol;
	int								vindpm_level;
	int								sw_aicl_point;
	int								sw_eoc_count;

	atomic_t						bms_suspended;
	unsigned long					soc_update_time;
	unsigned long					soc_update_pre_time;
	bool							check_stat_again;
	bool							power_off;
    struct mutex                    usbin_lock; /*chaoying.chen@EXP.BaseDrv.charge,2015/08/10 add for USB recognition */
	bool							updating_fw_flag;
};

struct opchg_gpio_control {
	int								opchg_swtich1_gpio;
	int								opchg_swtich2_gpio;
	int								opchg_reset_gpio;
	int								opchg_clock_gpio;
	int								opchg_data_gpio;
	int                             opchg_fastcharger;
};

struct opchg_fast_charger {
    struct i2c_client               	*client;
    struct device                   	*dev;

    struct mutex                    	fast_read_write_lock;
    struct delayed_work             	update_opfastchg_thread_work;
    struct delayed_work             	opfastchg_delayed_wakeup_work;

    struct regulator*               	vcc_i2c;
    int                             	opchg_fast_driver_id;
    int                             	g_fast_charging_wakeup;
};

struct opchg_bms_charger;
struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct opchg_bms_charger *di);
};

#ifdef CONFIG_GAUGE_BQ27411
struct cmd_address {
//bq27411 standard cmds
	u8	reg_cntl;
	u8	reg_temp;
	u8 	reg_volt;
	u8	reg_flags;
	u8	reg_nac;
	u8 	reg_fac;
	u8	reg_rm;
	u8	reg_fcc;
	u8	reg_ai;
	u8	reg_si;
	u8	reg_mli;
	u8	reg_ap;
	u8	reg_soc;
	u8	reg_inttemp;
	u8	reg_soh;
	u16	flag_dsc;
	u16	flag_fc;
	u16	cs_dlogen;
	u16 cs_ss;
//bq27541 external standard cmds
	u8	reg_ar;
	u8	reg_artte;
	u8	reg_tte;
	u8	reg_ttf;
	u8	reg_stte;
	u8	reg_mltte;
	u8	reg_ae;
	u8	reg_ttecp;
	u8	reg_cc;
	u8	reg_nic;
	u8	reg_icr;
	u8	reg_logidx;
	u8	reg_logbuf;
	u8	reg_dod0;
//bq27411 sub cmds
	u16 subcmd_cntl_status;
	u16 subcmd_device_type;
	u16 subcmd_fw_ver;
	u16 subcmd_dm_code;
	u16 subcmd_prev_macw;
	u16 subcmd_chem_id;
	u16 subcmd_set_hib;
	u16 subcmd_clr_hib;
	u16 subcmd_set_cfg;
	u16 subcmd_sealed;
	u16 subcmd_reset;
	u16 subcmd_softreset;
	u16 subcmd_exit_cfg;
	u16 subcmd_enable_dlog;
	u16 subcmd_disable_dlog;
	u16 subcmd_enable_it;
	u16 subcmd_disable_it;
//bq27541 external sub cmds
	u16 subcmd_hw_ver;
	u16 subcmd_df_csum;
	u16 subcmd_bd_offset;
	u16 subcmd_int_offset;
	u16 subcmd_cc_ver;
	u16 subcmd_ocv;
	u16 subcmd_bat_ins;
	u16 subcmd_bat_rem;
	u16 subcmd_set_slp;
	u16 subcmd_clr_slp;
	u16 subcmd_fct_res;
	u16 subcmd_cal_mode;
};
#endif

//struct bq27541_device_info {
struct opchg_bms_charger {
	struct i2c_client					*client;
	struct device						*dev;

	struct bq27541_access_methods			*bus;

	int				id;


	struct work_struct		counter;
	/* 300ms delay is needed after bq27541 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work		hw_config;
	int soc_pre;
	int 								fcc_pre;//full_charge_soc;
	int 								soh_pre;
	int 								cc_pre;
	int 								fac_pre;
	int 								rm_pre;
	int 								pchg_pre;
	int 								dod0_pre;
	int 								flags_pre;
	int temp_pre;
	int batt_vol_pre;
	int current_pre;
	int saltate_counter;
	bool is_authenticated;	//wangjc add for authentication
	bool fast_chg_started;
	bool fast_switch_to_normal;
	bool fast_normal_to_warm;	//lfc add for fastchg over temp
	int battery_type;			//lfc add for battery type
	struct power_supply		*batt_psy;
	int irq;
	struct work_struct fastcg_work;
	bool alow_reading;
	// chang mod timer to delay tims for dengnw in 20141221
	struct timer_list watchdog;
	struct delayed_work             watchdog_delayed_work;
	struct mutex					work_irq_lock;

	struct wake_lock fastchg_wake_lock;
	bool fast_chg_allow;
	bool fast_low_temp_full;
	int retry_count;
	unsigned long rtc_resume_time;
	unsigned long rtc_suspend_time;
	unsigned long rtc_refresh_time;
	atomic_t suspended;
	bool fast_chg_ing;


	int								opchg_swtich1_gpio;
	int								opchg_swtich2_gpio;
	int								opchg_reset_gpio;
	int								opchg_clock_gpio;
	int								opchg_data_gpio;

	struct pinctrl 						*pinctrl;
	struct pinctrl_state 					*gpio_switch1_act_switch2_act;
	struct pinctrl_state 					*gpio_switch1_sleep_switch2_sleep;
	struct pinctrl_state 					*gpio_switch1_act_switch2_sleep;
	struct pinctrl_state 					*gpio_switch1_sleep_switch2_act;

	struct pinctrl_state 					*gpio_clock_active;
	struct pinctrl_state 					*gpio_clock_sleep;
	struct pinctrl_state 					*gpio_data_active;
	struct pinctrl_state 					*gpio_data_sleep;
	struct pinctrl_state 					*gpio_reset_active;
	struct pinctrl_state 					*gpio_reset_sleep;
#ifdef CONFIG_GAUGE_BQ27411
	int device_type;
	struct cmd_address						cmd_addr;
#endif

};


#endif /*_OPPO_DEF_H_*/
