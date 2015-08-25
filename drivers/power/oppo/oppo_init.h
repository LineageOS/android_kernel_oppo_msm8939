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

#ifndef _OPPO_INIT_H_
#define _OPPO_INIT_H_

#ifdef OPPO_INIT_PAR
#define OPPO_INIT_EXT
#else
#define OPPO_INIT_EXT extern
#endif

#define OPCHARGER_I2C_VTG_MIN_UV                      1800000
#define OPCHARGER_I2C_VTG_MAX_UV                      1800000

OPPO_INIT_EXT int opchg_parse_dt(struct opchg_charger *chip);
#ifdef OPPO_INIT_PAR
struct qpnp_battery_gauge *qpnp_batt_gauge = NULL;
struct opchg_charger *opchg_chip = NULL;
struct oppo_qpnp_chg_chip *opchg_pimic_chip = NULL;
#else
extern struct qpnp_battery_gauge *qpnp_batt_gauge;
extern struct opchg_charger *opchg_chip;
extern struct qpnp_lbc_chip *the_chip;
extern struct oppo_qpnp_chg_chip *opchg_pimic_chip;
#endif

OPPO_INIT_EXT void qpnp_battery_gauge_register(struct qpnp_battery_gauge *batt_gauge);
OPPO_INIT_EXT void qpnp_battery_gauge_unregister(struct qpnp_battery_gauge *batt_gauge);

#endif /*_OPPO_INIT_H_*/
