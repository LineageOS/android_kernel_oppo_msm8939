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

#ifndef _OPPO_UPPER_H_
#define _OPPO_UPPER_H_

#ifdef OPPO_UPPER_PAR
#define OPPO_UPPER_EXT
#else
#define OPPO_UPPER_EXT extern
#endif

OPPO_UPPER_EXT void opchg_property_config(struct opchg_charger *chip);

OPPO_UPPER_EXT void opchg_dc_property_config(struct opchg_charger *chip);

OPPO_UPPER_EXT void opchg_debugfs_create(struct opchg_charger *chip);

OPPO_UPPER_EXT int opchg_batt_property_is_writeable(struct power_supply *psy,
                             enum power_supply_property psp);
OPPO_UPPER_EXT int opchg_battery_set_property(struct power_supply *psy,
                            enum power_supply_property prop,
                            const union power_supply_propval *val);
OPPO_UPPER_EXT int opchg_battery_get_property(struct power_supply *psy,
                            enum power_supply_property prop,
                            union power_supply_propval *val);
OPPO_UPPER_EXT int qpnp_power_get_property_mains(struct power_supply *psy,
					  enum power_supply_property prop,
					  union power_supply_propval *val);
OPPO_UPPER_EXT int qpnp_dc_power_set_property(struct power_supply *psy,
					  enum power_supply_property prop,
					  const union power_supply_propval *val);
OPPO_UPPER_EXT int qpnp_dc_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp);

#endif /*_OPPO_UPPER_H_*/
