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

#ifndef _OPPO_BATTERY_H_
#define _OPPO_BATTERY_H_

#ifdef OPPO_BATTERY_PAR
#define OPPO_BATTERY_EXT
#else
#define OPPO_BATTERY_EXT extern
#endif

#define HYS_NONE										0
#define HYSTERISIS_DECIDEGC                     		20

#define LCD_ON_CHARGING_INPUT_CURRENT_15011				1200
#define LCD_OFF_CHARGING_INPUT_CURRENT_15011				2000
#define LCD_ON_CHARGING_FAST_CURRENT_15011				1500
#define LCD_OFF_CHARGING_FAST_CURRENT_15011				2000

#define LCD_ON_CHARGING_FAST_CURRENT_15009				650
#define LCD_OFF_CHARGING_FAST_CURRENT_15009				1000

#define LCD_ON_CHARGING_INPUT_CURRENT_15018				1200
#define LCD_OFF_CHARGING_INPUT_CURRENT_15018			2000

#define LCD_ON_CHARGING_INPUT_CURRENT_15109				1000
#define LCD_OFF_CHARGING_INPUT_CURRENT_15109			1500

#define CAMERA_ON_CHARGING_INPUT_CURRENT					1000
#define CAMERA_OFF_CHARGING_INPUT_CURRENT					1500
#define CAMERA_ON_CHARGING_FAST_CURRENT					600
#define CAMERA_OFF_CHARGING_FAST_CURRENT					1500

#define CMCC_CHARGER_FULL_4250MV						4240000
#define CMCC_CHARGER_FULL_4208MV						4208000
#define CMCC_FULL_CHARGING_INPUT_CURRENT					150
#define CMCC_CHARGING_INPUT_CURRENT						1500

#define SMB358_VFLOAT_STEP_MV							20
#define BQ24196_VFLOAT_STEP_MV							16

#define TREM_FULL_VOLTAGE								4320
#define TREM_FULL_4350MV								4350000
#define TREM_FULL_4340MV								4340000
#define TREM_4180MV										4180000
#define TREM_4000MV										4000000

#define SMB358_PRE_TREM_FULL_VOLTAGE						(TREM_FULL_VOLTAGE - VFLOAT_STEP_MV)
#define SMB358_PRE_2TREM_FULL_VOLTAGE					(TREM_FULL_VOLTAGE - 2*VFLOAT_STEP_MV)
#define BQ24196_PRE_TREM_FULL_VOLTAGE					(TREM_FULL_VOLTAGE  - BQ24196_VFLOAT_STEP_MV)
#define BQ24196_PRE_2TREM_FULL_VOLTAGE					(TREM_FULL_VOLTAGE  - 2*BQ24196_VFLOAT_STEP_MV)

#if 0
#ifdef OPPO_BATTERY_PAR
struct opchg_charger *chip_opchg = NULL;
#else
extern struct opchg_charger *chip_opchg;
#endif
#endif
OPPO_BATTERY_EXT int oppo_power_type;

OPPO_BATTERY_EXT int opchg_inout_charge_parameters(struct opchg_charger *chip);
OPPO_BATTERY_EXT int opchg_init_charge_parameters(struct opchg_charger *chip);
#ifdef OPPO_USE_ADC_TM_IRQ
OPPO_BATTERY_EXT void opchg_get_adc_notification(enum qpnp_tm_state state, void *ctx);
#else
OPPO_BATTERY_EXT void opchg_get_adc_notification(struct opchg_charger *chip);
#endif
OPPO_BATTERY_EXT void opchg_get_charger_ov_status(struct opchg_charger *chip);
OPPO_BATTERY_EXT void opchg_get_battery_ov_status(struct opchg_charger *chip);
OPPO_BATTERY_EXT void opchg_config_reset_charger(struct opchg_charger *chip, int reason, int reset);
OPPO_BATTERY_EXT void opchg_config_input_chg_current(struct opchg_charger *chip, int reason, int mA);
OPPO_BATTERY_EXT void opchg_config_over_time(struct opchg_charger *chip, int mA);
OPPO_BATTERY_EXT void opchg_config_fast_current(struct opchg_charger *chip, int reason, int mA);
OPPO_BATTERY_EXT void opchg_config_term_voltage(struct opchg_charger *chip, int reason, int mV);
OPPO_BATTERY_EXT void opchg_config_term_current(struct opchg_charger *chip, int reason, int mA);
OPPO_BATTERY_EXT void opchg_config_charging_disable(struct opchg_charger *chip, int reason, int disable);
OPPO_BATTERY_EXT void opchg_config_suspend_enable(struct opchg_charger *chip, int reason, bool suspend);
OPPO_BATTERY_EXT void opchg_config_charging_phase(struct opchg_charger *chip,int phase);
#ifdef OPPO_USE_2CHARGER
OPPO_BATTERY_EXT void opchg_get_prop_fastcharger_status(struct opchg_charger *chip);
#endif
OPPO_BATTERY_EXT void opchg_check_status(struct opchg_charger *chip);
OPPO_BATTERY_EXT void opchg_set_status(struct opchg_charger *chip, bool input_curr_set);
OPPO_BATTERY_EXT void opchg_update_thread(struct work_struct *work);
OPPO_BATTERY_EXT void opchg_delayed_wakeup_thread(struct work_struct *work);
OPPO_BATTERY_EXT void opchg_works_init(struct opchg_charger *chip);
#if 0
OPPO_BATTERY_EXT int  opchg_battery_temp_region_get(struct opchg_charger *chip);
OPPO_BATTERY_EXT void opchg_battery_temp_region_set(struct opchg_charger *chip,int batt_temp_region);
#endif

#endif /*_OPPO_BATTERY_H_*/
