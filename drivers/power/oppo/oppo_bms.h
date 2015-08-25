/*******************************************************************************
* Copyright (c)  2014- 2014  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* VENDOR_EDIT
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

#ifndef _OPPO_BMS_H_
#define _OPPO_BMS_H_

#ifdef OPPO_BMS_PAR
#define OPPO_BMS_EXT
#else
#define OPPO_BMS_EXT extern
#endif

#define OPCHG_DEFAULT_BATT_CAPACITY		50
#define OPCHG_SOC_CHANGE_20S			3
#define OPCHG_SOC_CHANGE_40S			8
#define OPCHG_SOC_CHANGE_60S			12
#define OPCHG_SOC_CHANGE_35S			7
#define OPCHG_SOC_CHANGE_150S			30
#define OPCHG_SOC_CHANGE_300S			66
#define EIGHT_MINUTES					480

OPPO_BMS_EXT	int opchg_get_prop_fast_chg_started(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_prop_fast_chg_allow(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_set_fast_chg_allow(struct opchg_charger *chip,int enable);
OPPO_BMS_EXT	int opchg_get_fast_low_temp_full(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_prop_fast_switch_to_normal(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_prop_fast_normal_to_warm(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_fast_chg_ing(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_set_fast_normal_to_warm_false(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_set_fast_switch_to_normal_false(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_fast_normal_to_warm(struct opchg_charger *chip);	
OPPO_BMS_EXT	int opchg_get_prop_authenticate(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_prop_batt_capacity(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_prop_current_now(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_get_prop_batt_health(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_battery_notify_check(struct opchg_charger *chip);
OPPO_BMS_EXT	bool opchg_get_prop_batt_present(struct opchg_charger *chip);
OPPO_BMS_EXT	int opchg_backup_ocv_soc(int soc);
OPPO_BMS_EXT	int msmrtc_alarm_read_time(struct rtc_time *tm);

#endif /*_OPPO_BMS_H_*/
