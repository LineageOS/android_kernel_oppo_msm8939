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

#define OPPO_ADC_PAR
#include <oppo_inc.h>

int opchg_get_prop_charger_voltage_now(struct opchg_charger *chip)
{
    int rc = 0;
    int V_charger = 0;
    struct qpnp_vadc_result results;

	switch (chip->driver_id) {
	  case OPCHG_SMB358_ID:
			{
				// board version_B
			    rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
			    if (rc) {
			        pr_err("Unable to read vchg rc=%d\n", rc);
			        return 0;
			    }
			    V_charger = (int)results.physical/1000;
				break;
			}
	   case OPCHG_SMB1357_ID:
	   		{
				// board version_B
			    rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
			    if (rc) {
			        pr_err("Unable to read vchg rc=%d\n", rc);
			        return 0;
			    }
			    V_charger = (int)results.physical/1000;
				break;
			}
	   case OPCHG_BQ24196_ID:
	   		{
				// board version_B
			    rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
			    if (rc) {
			        pr_err("Unable to read vchg rc=%d\n", rc);
			        return 0;
			    }
			    V_charger = (int)results.physical/1000;
				break;
			}
	   
	   case OPCHG_BQ24157_ID:
	   		{
				// board version_B
			    rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
			    if (rc) {
			        pr_err("Unable to read vchg rc=%d\n", rc);
			        return 0;
			    }
			    V_charger = (int)results.physical/1000;
				break;
			}

	   case OPCHG_BQ24188_ID:
	   		{
				rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
			    if (rc) {
			        pr_err("Unable to read vchg rc=%d\n", rc);
			        return 0;
			    }
			    V_charger = (int)results.physical/1000;
				V_charger = V_charger * 2;
				break;
			}
	   
		default:
        	break;
	}
    return V_charger;//return (int)results.physical/1000;
}

int opchg_get_prop_low_battery_voltage(struct opchg_charger *chip)
{
	int rc = 0;
	int64_t mpp_uV= 0;
	int V_low_battery = 0;
	struct qpnp_vadc_result results;

	switch (chip->driver_id) {
	  case OPCHG_SMB358_ID:
			if(is_project(OPPO_14043) || is_project(OPPO_15005) || is_project(OPPO_15025)){
				V_low_battery = 0;	
			}
			break;
	   case OPCHG_SMB1357_ID:
			rc = qpnp_vadc_read(chip->vadc_dev, P_MUX4_1_1, &results);
			if (rc) {
				pr_err("Unable to read vbattery rc=%d\n", rc);
				return 0;
			}
			mpp_uV =results.physical*34;
			V_low_battery = (int)mpp_uV/10000;	
			break;
	   case OPCHG_BQ24196_ID:

	   /*huqiao@EXP.BasicDrv.Basic add for clone 15085*/
	   		if(is_project(OPPO_14037) || is_project(OPPO_14051) || is_project(OPPO_15057) || is_project(OPPO_15009) || is_project(OPPO_15037)|| is_project(OPPO_15085))
			{
				return 0;
			}
			else
			{
				rc = qpnp_vadc_read(chip->vadc_dev, P_MUX4_1_1, &results);
				if (rc) {
					pr_err("Unable to read vbattery rc=%d\n", rc);
					return 0;
				}
				if(is_project(OPPO_14005)||is_project(OPPO_14023))
				{
					if(get_PCB_Version()== HW_VERSION__10)
					{
						mpp_uV =results.physical*34;
						V_low_battery = (int)mpp_uV/10000;
					}
					else
					{
						mpp_uV =results.physical*31;
						V_low_battery = (int)mpp_uV/1000;
					}
				}
				else if(is_project(OPPO_14045))	
				{
					if(get_PCB_Version()== HW_VERSION__10)
					{
						mpp_uV =results.physical*34;
						V_low_battery = (int)mpp_uV/10000;
					}
					else
					{
						mpp_uV =results.physical*4;
						V_low_battery = (int)mpp_uV/1000;
					}
				}
				else if(is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022)) /*#hanqing.wang@EXP.BasicDrv.Audio add for clone 15089 and add the macor MSM_15062 and OPPO_15011 = OPPO_15018*/	
				{
					mpp_uV =results.physical*4;
					V_low_battery = (int)mpp_uV/1000;
				}
				else
				{
					mpp_uV =results.physical*4;
					V_low_battery = (int)mpp_uV/1000;
				}
			}
			break;
		default:
        	break;
	}
	return V_low_battery;
}

int opchg_get_prop_battery_voltage_now(struct opchg_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	int V_battery = 0;
/*huqiao@EXP.BasicDrv.Basic add for clone 15085*/
	if(is_project(OPPO_14043) || is_project(OPPO_14037) || is_project(OPPO_14051) 
		|| is_project(OPPO_15005) || is_project(OPPO_15057) || is_project(OPPO_15009) || is_project(OPPO_15037)|| is_project(OPPO_15085)){

		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
		if (rc) {
			pr_err("Unable to read vbat rc=%d\n", rc);
			return 0;
		}
		V_battery =(int)results.physical;
	} 

	else if(is_project(OPPO_14005) || is_project(OPPO_14023)|| is_project(OPPO_14045)|| is_project(OPPO_15011)|| is_project(OPPO_15018) || is_project(OPPO_15022)) /*#hanqing.wang@EXP.BasicDrv.Audio add for clone 15089 and add the macor MSM_15062 and OPPO_15011 = OPPO_15018*/

	{
		if (qpnp_batt_gauge && qpnp_batt_gauge->get_battery_mvolts)
			V_battery =qpnp_batt_gauge->get_battery_mvolts();
		else {
			pr_err("qpnp-charger no batt gauge assuming 3.5V\n");
			V_battery =3500*1000;
		}
	}	
	else {
		if (qpnp_batt_gauge && qpnp_batt_gauge->get_battery_mvolts)
			V_battery =qpnp_batt_gauge->get_battery_mvolts();
		else {
			pr_err("qpnp-charger no batt gauge assuming 3.5V\n");
			V_battery =3500*1000;
		}
	}

	return V_battery;
}

int opchg_get_prop_batt_temp(struct opchg_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	int T_battery = 0;
    
/*huqiao@EXP.BasicDrv.Basic add for clone 15085*/
	if(is_project(OPPO_14043) || is_project(OPPO_14037) ||is_project(OPPO_14051) ||
		is_project(OPPO_15005) || is_project(OPPO_15057) || is_project(OPPO_15025)||
		is_project(OPPO_15009)|| is_project(OPPO_15037)|| is_project(OPPO_15035)
        || is_project(OPPO_15085) ){
		rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
		if (rc) {
			pr_err("Unable to read batt temperature rc=%d\n", rc);
			return 0;
		}
		T_battery = (int)results.physical;
	}
	/*#hanqing.wang@EXP.BasicDrv.Audio add for clone 15089 and add the macor MSM_15062 and OPPO_15011 = OPPO_15018*/
	else if(is_project(OPPO_14005) || is_project(OPPO_14023)|| is_project(OPPO_14045)|| is_project(OPPO_15011)|| is_project(OPPO_15018) || is_project(OPPO_15022)) 
	{
		if (qpnp_batt_gauge && qpnp_batt_gauge->get_battery_temperature) {
			T_battery =qpnp_batt_gauge->get_battery_temperature();
			#ifdef OPCHG_DEBUG
			if (T_battery  > 600)
				T_battery  = 600;
			#endif
		} else {
			pr_err("qpnp-charger no batt gauge assuming 35 deg G\n");
			T_battery = -400;
		}
	}	
	else {
		if (qpnp_batt_gauge && qpnp_batt_gauge->get_battery_temperature) {
			T_battery =qpnp_batt_gauge->get_battery_temperature();
			#ifdef OPCHG_DEBUG
			if (T_battery  > 600)
				T_battery  = 600;
			#endif
		} else {
			pr_err("qpnp-charger no batt gauge assuming 35 deg G\n");
			T_battery = -400;
		}
	}
	return T_battery;
}
