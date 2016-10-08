/*******************************************************************************
* Copyright (c)  2014- 2014  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* CONFIG_MACH_OPPO
* Description: Source file for CBufferList.
*           To allocate and free memory block safely.
* Version   : 0.0
* Date      : 2014-07-30
* Author    : Dengnanwei @Bsp.charge
*             Lijiada @Bsp.charge
* ---------------------------------- Revision History: -------------------------
* <version>           <date>          < author >              <desc>
* Revision 0.0        2014-07-30      Dengnanwei @Bsp.charge
*                                     Lijiada @Bsp.charge
* Modified to be suitable to the new coding rules in all functions.
*******************************************************************************/

#ifndef _OPPO_BQ24157_H_
#define _OPPO_BQ24157_H_

#ifdef OPPO_BQ24157_PAR
#define OPPO_BQ24157_EXT
#else
#define OPPO_BQ24157_EXT extern
#endif

#define BQ24157_FIRST_REG					0x00
#define BQ24157_LAST_REG					0x06
#define BQ24157_STATUS_CTRL_REG				0x00
#define BQ24157_CTRL_REG					0x01
#define BQ24157_BATTERY_VOL_REG				0x02
#define BQ24157_VENDOR_PART_REV_REG			0x03
#define BQ24157_TERM_FAST_CURRENT_REG		0x04
#define BQ24157_SPECIAL_CHARGER_VOL_REG		0x05
#define BQ24157_SAFETY_LIMIT_REG			0x06

#define BQ24157_CHARGE_STATUS_MASK			(BIT(4) | BIT(5))
#define BQ24157_CHARGE_IN_PROGRESS			BIT(4)
#define BQ24157_CHARGE_TERM					BIT(5)

#define BQ24157_FAULT_STATUS_MASK			(BIT(2) | BIT(1) | BIT(0))
#define BQ24157_BATTERY_OVP					BIT(2)

#define BQ24157_CHARGE_ENABLE_MASK			BIT(2)
#define BQ24157_CHARGE_ENABLE				0x00
#define BQ24157_CHARGE_DISABLE				BIT(2)

#define BQ24157_OTG_MASK					BIT(0)
#define BQ24157_OTG_ENABLE					BIT(0)
#define BQ24157_OTG_DISABLE					0x00

#define BQ24157_STATUS_OTG_MODE				BIT(3)

#define BQ24157_RESET_MASK					BIT(7)
#define BQ24157_RESET_ENABLE				BIT(7)
#define BQ24157_RESET_NONE					0x00

#define BQ24157_SUSPEND_MASK				BIT(1)
#define BQ24157_SUSPEND_ENABLE				BIT(1)
#define BQ24157_SUSPEND_DISABLE				0x00

#define BQ24157_FAST_CURRENT_MASK			(BIT(4) | BIT(5) | BIT(6))
#define BQ24157_LOWCHG_CURRENT_MASK			BIT(5)
#define BQ24157_LOWCHG_CURRENT_ENABLE		BIT(5)
#define BQ24157_LOWCHG_CURRENT_DISABLE		0x00

#define BQ24157_FAST_CURRENT_MAX_MA			1250
#define BQ24157_FAST_CURRENT_OFFSET_MA		550
#define BQ24157_FAST_CURRENT_STEP_MA		100
#define BQ24157_FAST_CURRENT_LOWCHG_MA		325

#define BQ24157_TERM_CURRENT_MASK			(BIT(0) | BIT(1) | BIT(2))
#define BQ24157_TERM_CURRENT_MIN_MA			50
#define BQ24157_TERM_CURRENT_STEP_MA		50

#define BQ24157_IUSBMAX_MASK				(BIT(7) | BIT(6))
#define BQ24157_IUSBMAX_SHIFT				6
#define BQ24157_IUSBMAX_100MA				0x00
#define BQ24157_IUSBMAX_500MA				BIT(6)
#define BQ24157_IUSBMAX_800MA				BIT(7)
#define BQ24157_IUSBMAX_NOLIMIT				(BIT(7) | BIT(6))

#define BQ24157_CURRENT_NOLIMIT				1500

#define BQ24157_FLOAT_VOLTAGE_MASK			(BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define BQ24157_FLOAT_VOLTAGE_MIN_MV		3500
#define BQ24157_FLOAT_VOLLTAGE_STEP_MV		20
#define BQ24157_FLOAT_VOLTAGE_SHIFT			2

#define BQ24157_VINDPM_MASK					(BIT(2) | BIT(1) | BIT(0))
#define BQ24157_VINDPM_OFFSET_MV			4200
#define BQ24157_VINDPM_STEP_MV				80

#define BQ24157_VFLOAT_STEP_MV              20

#define BQ24157_MAX_VOLT_CURRENT_MASK				0xFF
#define BQ24157_MAX_FAST_CURRENT_STEP_MA			100
#define BQ24157_MAX_FAST_CURRENT_MAX_MA				1550
#define BQ24157_MAX_FAST_CURRENT_OFFSET_MA			550
#define BQ24157_MAX_FAST_CURRENT_SHIFT				4

#define BQ24157_MAX_BATT_VOL_MAX_MV					4440
#define BQ24157_MAX_BATT_VOL_MIN_MV					4200
#define BQ24157_MAX_BATT_VOL_STEP_MV				20

#define BQ24157_TERM_ENABLE_MASK			BIT(3)
#define BQ24157_TERM_ENABLE					BIT(3)
#define BQ24157_WEAK_BATTERY_MASK			(BIT(5) |  BIT(4))
#define BQ24157_WEAK_BATTERY_MAX_MV			3700
#define BQ24157_WEAK_BATTERY_MIN_MV			3400
#define BQ24157_WEAK_BATTERY_STEP_MV		100
#define BQ24157_WEAK_BATTERY_SHIFT			4

OPPO_BQ24157_EXT int bq24157_set_otg_enable(void);
OPPO_BQ24157_EXT int bq24157_set_otg_disable(void);
OPPO_BQ24157_EXT int bq24157_get_otg_enable(void);
OPPO_BQ24157_EXT int bq24157_set_otg_regulator_enable(struct regulator_dev *rdev);
OPPO_BQ24157_EXT int  bq24157_set_otg_regulator_disable(struct regulator_dev *rdev);
OPPO_BQ24157_EXT int bq24157_get_otg_regulator_is_enable(struct regulator_dev *rdev);
OPPO_BQ24157_EXT int  bq24157_get_prop_charge_type(struct opchg_charger *chip);
OPPO_BQ24157_EXT int  bq24157_get_charging_status(struct opchg_charger *chip);
OPPO_BQ24157_EXT  int bq24157_get_prop_batt_status(struct opchg_charger *chip);
OPPO_BQ24157_EXT int  bq24157_set_reset_charger(struct opchg_charger *chip, bool reset);
OPPO_BQ24157_EXT int  bq24157_set_input_chg_current(struct opchg_charger *chip, int iusbin_mA, bool aicl_enable);
OPPO_BQ24157_EXT int  bq24157_set_fastchg_current(struct opchg_charger *chip, int ifast_mA);
OPPO_BQ24157_EXT int  bq24157_set_float_voltage(struct opchg_charger *chip, int vfloat_mv);
OPPO_BQ24157_EXT int bq24157_set_vindpm_vol(struct opchg_charger *chip, int vol);
OPPO_BQ24157_EXT int  bq24157_set_charging_disable(struct opchg_charger *chip, bool disable);
OPPO_BQ24157_EXT int  bq24157_set_suspend_enable(struct opchg_charger *chip, bool enable);
OPPO_BQ24157_EXT int bq24157_hw_init(struct opchg_charger *chip);
OPPO_BQ24157_EXT int bq24157_check_battovp(struct opchg_charger *chip);
OPPO_BQ24157_EXT int bq24157_check_charging_pre_full(struct opchg_charger *chip);
OPPO_BQ24157_EXT void bq24157_usbin_valid_work(struct work_struct *work);
OPPO_BQ24157_EXT void  bq24157_usbin_valid_irq_handler(struct opchg_charger *chip);
OPPO_BQ24157_EXT void  bq24157_dump_regs(struct opchg_charger *chip);
OPPO_BQ24157_EXT int bq24157_get_initial_state(struct opchg_charger *chip);

#endif /*_OPPO_BQ24196_H_*/
