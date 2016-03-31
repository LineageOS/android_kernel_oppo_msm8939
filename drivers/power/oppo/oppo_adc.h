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

#ifndef _OPPO_ADC_H_
#define _OPPO_ADC_H_

#ifdef OPPO_ADC_PAR
#define OPPO_ADC_EXT
#else
#define OPPO_ADC_EXT extern
#endif

OPPO_ADC_EXT int opchg_get_prop_charger_voltage_now(struct opchg_charger *chip);
OPPO_ADC_EXT int opchg_get_prop_battery_voltage_now(struct opchg_charger *chip);
OPPO_ADC_EXT int opchg_get_prop_batt_temp(struct opchg_charger *chip);
OPPO_ADC_EXT int opchg_get_prop_low_battery_voltage(struct opchg_charger *chip);
OPPO_ADC_EXT int opchg_get_prop_battery_id_voltage(struct opchg_charger *chip);

#endif /*_OPPO_ADC_H_*/
