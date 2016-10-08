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

#define OPPO_BMS_PAR
#include "oppo_inc.h"

int opchg_get_prop_fast_chg_started(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->fast_chg_started)
		return qpnp_batt_gauge->fast_chg_started();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_get_prop_fast_chg_allow(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->get_fast_chg_allow)
		return qpnp_batt_gauge->get_fast_chg_allow();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_set_fast_chg_allow(struct opchg_charger *chip,int enable)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->set_fast_chg_allow)
		return qpnp_batt_gauge->set_fast_chg_allow(enable);
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_set_fast_switch_to_normal_false(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->set_switch_to_noraml_false)
		return qpnp_batt_gauge->set_switch_to_noraml_false();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_set_fast_normal_to_warm_false(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->set_normal_to_warm_false)
		return qpnp_batt_gauge->set_normal_to_warm_false();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_get_fast_normal_to_warm(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->fast_normal_to_warm)
		return qpnp_batt_gauge->fast_normal_to_warm();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_get_fast_low_temp_full(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->get_fast_low_temp_full)
		return qpnp_batt_gauge->get_fast_low_temp_full();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}
int opchg_get_prop_fast_switch_to_normal(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->fast_switch_to_normal)
		return qpnp_batt_gauge->fast_switch_to_normal();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}
int opchg_get_prop_fast_normal_to_warm(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->fast_normal_to_warm)
		return qpnp_batt_gauge->fast_normal_to_warm();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_get_fast_chg_ing(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->get_fast_chg_ing)
		return qpnp_batt_gauge->get_fast_chg_ing();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

int opchg_get_prop_authenticate(struct opchg_charger *chip)
{
	if (qpnp_batt_gauge && qpnp_batt_gauge->is_battery_authenticated)
		return qpnp_batt_gauge->is_battery_authenticated();
	else {
		//pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

static bool opchg_soc_reduce_slowly_when_1(struct opchg_charger *chip)
{
	static int reduce_count = 0;
	int batt_vol = 0;

	if(opchg_get_prop_batt_present(chip) == false)
		return 0;

	batt_vol = opchg_get_prop_battery_voltage_now(chip)/1000;
	if(batt_vol < 3410)
		reduce_count++;
	else
		reduce_count = 0;
	pr_err("%s batt_vol:%d,reduce_count:%d\n",__func__,batt_vol,reduce_count);
	if(reduce_count < 5){
		return 0;
	} else {
		reduce_count = 5;
		return 1;
	}
}

int opchg_get_prop_batt_capacity_from_bms(struct opchg_charger *chip)
{
	static char is_pon_on = 0;
	static char sync_up_count = 0;
	static char sync_down_count = 0;
	static char sync_down_limit = 0;
	union power_supply_propval ret = {0, };
	struct rtc_time	soc_update_rtc_time;
	int rc = 0;

	if(!chip->bms_psy){
		return OPCHG_DEFAULT_BATT_CAPACITY;
	} else if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,POWER_SUPPLY_PROP_CAPACITY, &ret);
	}
	chip->soc_bms = ret.intval;

	/************************************
	*  bms soc is init
	************************************/
	if(is_pon_on == 0){
		is_pon_on = 1;
		chip->bat_volt_check_point = chip->soc_bms;
		pr_err("soc is %d after pon\n",chip->bat_volt_check_point);
		return chip->bat_volt_check_point;
	}

	/************************************
	*  charging full but soc is not 100%
	************************************/
	if((chip->chg_present) && (chip->batt_full) && (chip->batt_authen) && (opchg_get_prop_batt_present(chip) == 1)
		&& ((chip->charging_opchg_temp_statu >= OPCHG_CHG_TEMP_PRE_COOL1)&& (chip->charging_opchg_temp_statu  <= OPCHG_CHG_TEMP_NORMAL)))
	{
		sync_down_count = 0;
		if(sync_up_count >= OPCHG_SOC_CHANGE_60S){
			sync_up_count = 0;
			chip->bat_volt_check_point++;
		} else {
			sync_up_count++;
		}

		if(chip->bat_volt_check_point >= 100){
			chip->bat_volt_check_point = 100;
			chip->batt_pre_full = 1;
		}

		chip->ocv_uv = opchg_backup_ocv_soc(chip->bat_volt_check_point);
		//pr_debug("full,soc_bms:%d,soc_cal:%d,sync_up_count:%d\n",chip->soc_bms,chip->bat_volt_check_point,sync_up_count);
		return chip->bat_volt_check_point;
	} else if((chip->chg_present) && (opchg_get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_CHARGING) && (opchg_get_prop_batt_present(chip) == 1)){
		sync_down_count = 0;
		if(chip->soc_bms == chip->bat_volt_check_point){
			//pr_debug("charing,soc_bms:%d,soc_cal:%d\n",chip->soc_bms,chip->bat_volt_check_point);
		} else if (chip->soc_bms > chip->bat_volt_check_point){
			if(sync_up_count >= OPCHG_SOC_CHANGE_35S){
				sync_up_count = 0;
				chip->bat_volt_check_point++;
			} else {
				sync_up_count++;
			}
			//pr_debug("charging,soc_bms:%d,soc_cal:%d,sync_up_count:%d\n",chip->soc_bms,chip->bat_volt_check_point,sync_up_count);
		}
	} else {
		sync_up_count = 0;
		if ((chip->soc_bms < chip->bat_volt_check_point) ||
				((opchg_get_prop_battery_voltage_now(chip) < 3300 * 1000) && (opchg_get_prop_batt_present(chip) == true))){
			if(atomic_read(&chip->bms_suspended) == 1){
				rc = msmrtc_alarm_read_time(&soc_update_rtc_time);
				if (rc < 0) {
					pr_err("%s: failed to read soc update time\n", __func__);
				}
				rtc_tm_to_time(&soc_update_rtc_time, &chip->soc_update_time);
				if((chip->soc_update_time - chip->soc_update_pre_time) >= EIGHT_MINUTES)		//if soc don't update for 8min
					chip->bat_volt_check_point--;
			}

			if(chip->bat_volt_check_point == 100)
				sync_down_limit = OPCHG_SOC_CHANGE_300S;
			else if(chip->bat_volt_check_point >= 95)
				sync_down_limit = OPCHG_SOC_CHANGE_150S;
			else if(chip->bat_volt_check_point >= 60)
				sync_down_limit = OPCHG_SOC_CHANGE_60S;
			else
				sync_down_limit = OPCHG_SOC_CHANGE_40S;

			if((opchg_get_prop_battery_voltage_now(chip) < 3300 * 1000) && (opchg_get_prop_batt_present(chip) == true)){
				sync_down_limit = OPCHG_SOC_CHANGE_20S;
			}

			sync_down_count++;
			if(sync_down_count >= sync_down_limit)
			{
				if(chip->bat_volt_check_point > 1)
				{
					chip->bat_volt_check_point--;
				}
				sync_down_count = 0;
			}
		}
		//pr_debug("discharging soc_bms:%d,soc_cal:%d,sync_down_count:%d\n",chip->soc_bms,chip->bat_volt_check_point,sync_down_count);
	}

	if(chip->bat_volt_check_point >= 100){
		chip->bat_volt_check_point = 100;
	} else if(chip->bat_volt_check_point < 2){
		if(opchg_soc_reduce_slowly_when_1(chip))
		{
			chip->bat_volt_check_point = 0;
		}
	}

	if(chip->bat_volt_check_point <= 2){
		chip->ocv_uv = opchg_backup_ocv_soc(2);
	} else {
		chip->ocv_uv = opchg_backup_ocv_soc(chip->bat_volt_check_point);
	}

	rc = msmrtc_alarm_read_time(&soc_update_rtc_time);
	if (rc < 0) {
		pr_err("%s: failed to read soc update pre time\n", __func__);
	} else {
		rtc_tm_to_time(&soc_update_rtc_time, &chip->soc_update_pre_time);
	}

	return chip->bat_volt_check_point;
}

int opchg_get_prop_batt_capacity_from_bms_bq24157(struct opchg_charger *chip)
{
	static char is_pon_on = 0;
	static char sync_up_count = 0;
	static char sync_down_count = 0;
	static char sync_down_limit = 0;
	union power_supply_propval ret = {0, };
	struct rtc_time	soc_update_rtc_time;
	int rc = 0;

	if(!chip->bms_psy){
		return OPCHG_DEFAULT_BATT_CAPACITY;
	} else if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,POWER_SUPPLY_PROP_CAPACITY, &ret);
	}
	chip->soc_bms = ret.intval;

	if(is_pon_on == 0){
		is_pon_on = 1;
		chip->bat_volt_check_point = chip->soc_bms;
		pr_err("soc is %d after pon\n",chip->bat_volt_check_point);
		return chip->bat_volt_check_point;
	}

	if((chip->chg_present) && ((chip->batt_full) && (chip->soc_bms > 84)) && (chip->batt_authen) && (opchg_get_prop_batt_present(chip) == 1)
		&& ((chip->charging_opchg_temp_statu >= OPCHG_CHG_TEMP_PRE_COOL1)&& (chip->charging_opchg_temp_statu  <= OPCHG_CHG_TEMP_NORMAL)))
	{
		sync_down_count = 0;
		if(sync_up_count >= OPCHG_SOC_CHANGE_60S){
			sync_up_count = 0;
			chip->bat_volt_check_point++;
		} else {
			sync_up_count++;
		}

		if(chip->bat_volt_check_point >= 100){
			chip->bat_volt_check_point = 100;
			chip->batt_pre_full = 1;
		}
		chip->ocv_uv = opchg_backup_ocv_soc(chip->bat_volt_check_point);
		//pr_debug("full,soc_bms:%d,soc_cal:%d,sync_up_count:%d\n",chip->soc_bms,chip->bat_volt_check_point,sync_up_count);
		return chip->bat_volt_check_point;
	} else if((chip->chg_present) && (opchg_get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_CHARGING) && (opchg_get_prop_batt_present(chip) == 1)
				&& ((chip->soc_bms >= chip->bat_volt_check_point) || ((chip->bat_volt_check_point == 100) && (chip->soc_bms > 90)))){
		sync_down_count = 0;
		if(chip->soc_bms == chip->bat_volt_check_point){
			//pr_debug("charing,soc_bms:%d,soc_cal:%d\n",chip->soc_bms,chip->bat_volt_check_point);
		} else if (chip->soc_bms > chip->bat_volt_check_point){
			if(sync_up_count >= OPCHG_SOC_CHANGE_35S){
				sync_up_count = 0;
				chip->bat_volt_check_point++;
			} else {
				sync_up_count++;
			}
			//pr_debug("charging,soc_bms:%d,soc_cal:%d,sync_up_count:%d\n",chip->soc_bms,chip->bat_volt_check_point,sync_up_count);
		}
	} else {
		sync_up_count = 0;
		if ((chip->soc_bms < chip->bat_volt_check_point) ||
				((opchg_get_prop_battery_voltage_now(chip) < 3300 * 1000) && (opchg_get_prop_batt_present(chip) == true))){
			if(atomic_read(&chip->bms_suspended) == 1){
				rc = msmrtc_alarm_read_time(&soc_update_rtc_time);
				if (rc < 0) {
					pr_err("%s: failed to read soc update time\n", __func__);
				}
				rtc_tm_to_time(&soc_update_rtc_time, &chip->soc_update_time);
				if((chip->soc_update_time - chip->soc_update_pre_time) >= EIGHT_MINUTES)		//if soc don't update for 8min
					chip->bat_volt_check_point--;
			}

			if(chip->bat_volt_check_point == 100)
				sync_down_limit = OPCHG_SOC_CHANGE_300S;
			else if(chip->bat_volt_check_point >= 95)
				sync_down_limit = OPCHG_SOC_CHANGE_150S;
			else if(chip->bat_volt_check_point >= 60)
				sync_down_limit = OPCHG_SOC_CHANGE_60S;
			else
				sync_down_limit = OPCHG_SOC_CHANGE_40S;

			if((opchg_get_prop_battery_voltage_now(chip) < 3300 * 1000) && (opchg_get_prop_batt_present(chip) == true)){
				sync_down_limit = OPCHG_SOC_CHANGE_20S;
			}

			if(sync_down_count >= sync_down_limit){
				sync_down_count = 0;
				if(chip->bat_volt_check_point > 1)
					chip->bat_volt_check_point--;
			} else {
				sync_down_count++;
			}
		}
		//pr_debug("discharging soc_bms:%d,soc_cal:%d,sync_down_count:%d\n",chip->soc_bms,chip->bat_volt_check_point,sync_down_count);
	}

	if(chip->bat_volt_check_point >= 100){
		chip->bat_volt_check_point = 100;
	} else if(chip->bat_volt_check_point < 2){
		if(opchg_soc_reduce_slowly_when_1(chip))
			chip->bat_volt_check_point = 0;
		else
			chip->bat_volt_check_point = 1;
	}

	if(chip->bat_volt_check_point <= 2){
		chip->ocv_uv = opchg_backup_ocv_soc(2);
	} else {
#if 0
		if(chip->bat_volt_check_point == 100 && chip->soc_bms > 90)
			chip->ocv_uv = opchg_backup_ocv_soc(100);
		else
			chip->ocv_uv = opchg_backup_ocv_soc(chip->bat_volt_check_point - 1);
#else
		chip->ocv_uv = opchg_backup_ocv_soc(chip->bat_volt_check_point);
#endif
	}

	rc = msmrtc_alarm_read_time(&soc_update_rtc_time);
	if (rc < 0) {
		pr_err("%s: failed to read soc update pre time\n", __func__);
	} else {
		rtc_tm_to_time(&soc_update_rtc_time, &chip->soc_update_pre_time);
	}

	return chip->bat_volt_check_point;
}

int opchg_get_prop_batt_capacity(struct opchg_charger *chip)
{
	int soc;

	if(is_project(OPPO_15109)){
		soc = opchg_get_prop_batt_capacity_from_bms(chip);
	} else {
		if (qpnp_batt_gauge && qpnp_batt_gauge->get_battery_soc){
			soc = qpnp_batt_gauge->get_battery_soc();
			if (soc < 2) {
				if (opchg_soc_reduce_slowly_when_1(chip))
					soc = 0;
				else
					soc = 1;
			}
		} else {
			pr_err("qpnp-charger no batt gauge assuming 50percent\n");
			soc = 50;
		}
	}

	return soc;
}

int opchg_get_prop_current_now(struct opchg_charger *chip)
{
	int chg_current = 0;

	if(is_project(OPPO_15109)){
		if(!chip->chg_present){
			chg_current = 0;
		} else {
			chg_current = -450;
		}
	} else {
		if (qpnp_batt_gauge && qpnp_batt_gauge->get_average_current)
		{
			chg_current = qpnp_batt_gauge->get_average_current();
		}
		else {
			pr_err("qpnp-charger no batt gauge assuming 0mA\n");
			chg_current = 0;
		}
	}
	return chg_current;
}

int opchg_get_prop_batt_health(struct opchg_charger *chip)
{
    union power_supply_propval ret = {0, };

	if (chip->battery_missing) {
        ret.intval = POWER_SUPPLY_HEALTH_UNKNOWN;
    }
    else if (chip->batt_hot) {
        ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    else if (chip->batt_cold) {
        ret.intval = POWER_SUPPLY_HEALTH_COLD;
    }
#if 0
	else if (chip->batt_warm) {
        ret.intval = POWER_SUPPLY_HEALTH_WARM;
    }
    else if (chip->batt_cool) {
        ret.intval = POWER_SUPPLY_HEALTH_COOL;
    }
#endif
    else {
        ret.intval = POWER_SUPPLY_HEALTH_GOOD;
    }
    return ret.intval;
}


int opchg_battery_notify_check(struct opchg_charger *chip)
{
	int batterynotify= 0x0000;

	// step1: Check  the battery is connected good
	if(chip->battery_missing == true)
	{
		batterynotify |= Notify_Bat_Not_Connect;
	}

	// step 2: Check the battery voltage is safe
	if(chip->batt_voltage_over == true)
	{
		batterynotify |= Notify_Bat_Over_Vol;
	}

	// step 3: Check the charging status
	if(chip->chg_present == true)
	{
		// step 3.1: Check the charger voltage is over
		if(chip->charger_ov_status == true)
		{
			batterynotify |= Notify_Charger_Over_Vol;
		}

		// step 3.2: Check the charging time is over
		if( chip->charging_time_out  == true)
		{
			batterynotify |= Notify_Chging_OverTime;
		}

		// step 3.3: Check the charging full status of five kinds of cues
		if(chip->batt_hot == true)
		{
			batterynotify |= Notify_Bat_Over_Temp;
		}
		if(chip->batt_cold == true)
		{
			batterynotify |= Notify_Bat_Low_Temp;
		}

		//if(chip->batt_full == true)
		if(chip->bat_status == POWER_SUPPLY_STATUS_FULL)
		{
			if(chip->batt_warm == true)
			{
		       batterynotify |= Notify_Bat_Full_High_Temp;
		    }
			if(chip->batt_normal == true)
			{
				if(chip->batt_authen == true)
				{
					batterynotify |= Notify_Bat_Full;
				}
				else
				{
					batterynotify |= Notify_Bat_Full_THIRD_BATTERY;
				}
			}
			if(chip->batt_cool== true)
			{
				batterynotify |= Notify_Bat_Full_Low_Temp;
		    }
		}
	}
	#ifdef OPCHARGER_DEBUG_ENABLE
	pr_err(" opchg_battery_notify_check chip->battery_missing=%d,chip->batt_voltage_over=%d,chip->charger_ov_status=%d,chip->charging_time_out=%d,chip->batt_hot=%d,chip->batt_cold =%d,chip->batt_warm=%d,chip->batt_normal=%d,chip->batt_cool=%d,chip->batt_authen=%d,batterynotify = 0X%x\n",
		chip->battery_missing,chip->batt_voltage_over,chip->charger_ov_status,chip->charging_time_out,chip->batt_hot,chip->batt_cold,chip->batt_warm,chip->batt_normal,chip->batt_cool,chip->batt_authen,batterynotify);
	#endif
	return batterynotify;
}

bool opchg_get_prop_batt_present(struct opchg_charger *chip)
{
	chip->bat_exist = !chip->battery_missing;
	return chip->bat_exist;
}
