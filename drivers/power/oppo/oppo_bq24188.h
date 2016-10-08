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

#ifndef _OPPO_BQ24188_H_
#define _OPPO_BQ24188_H_

#ifdef OPPO_BQ24188_PAR
#define OPPO_BQ24188_EXT
#else
#define OPPO_BQ24188_EXT extern
#endif

#define BQ24188_STATUS_CTRL_REG				0x00
#define BQ24188_CTRL_REG					0x01
#define BQ24188_BATT_VOL_CTRL				0x02
#define BQ24188_VENDOR_PART_REG				0x03
#define BQ24188_CHG_CURRENT_CTRL			0x04
#define BQ24188_VINDPM_STATUS_REG			0x05
#define BQ24188_TIMEOUT_NTC_REG				0x06
#define BQ24188_FIRST_REG					0x00
#define BQ24188_LAST_REG					0x06

#define BQ24188_CHARGE_STATUS_MASK			0x30
#define BQ24188_CHARGE_READY				0x00
#define BQ24188_CHARGE_IN_PROGRESS			0x10
#define BQ24188_CHARGE_DONE					0x20
#define BQ24188_OTG_ENABLE_MASK				0x40
#define BQ24188_OTG_ENABLE					0x40
#define BQ24188_OTG_DISABLE					0x00
#define BQ24188_RESET_MASK					0x80
#define BQ24188_REG_RESET					0x80
#define BQ24188_REG_NOT_RESET				0x00
#define BQ24188_CHARGE_EN_MASK				0x02
#define BQ24188_CHARGE_ENABLE				0x00
#define BQ24188_CHARGE_DISABLE				0x02
#define BQ24188_SUSPEND_MASK				0x01
#define BQ24188_SUSPEND_ENABLE				0x01
#define BQ24188_SUSPEND_DISABLE				0x00
#define BQ24188_IBATMAX_MASK				0xF8
#define BQ24188_IBATMAX_MIN_MA				500
#define BQ24188_IBATMAX_STEP_MA				100

#define BQ24188_LOWCHG_ENABLE_MASK			0x20
#define BQ24188_LOWCHG_ENABLE				0x20
#define BQ24188_LOWCHG_DISABLE				0x00
#define BQ24188_TERM_CURRENT_MASK			0x07
#define BQ24188_TERM_CURRENT_MIN_MA			50
#define BQ24188_TERM_CURRENT_STEP_MA		50

#define BQ24188_IUSBMAX_MIN					100
#define BQ24188_IUSBMAX_MAX					2500
#define BQ24188_IUSBMAX_MASK				0x70
#define BQ24188_IUSBMAX_USB2D0_100MA		0x00
#define BQ24188_IUSBMAX_USB3D0_150MA		0x10
#define BQ24188_IUSBMAX_USB2D0_500MA		0x20
#define BQ24188_IUSBMAX_USB3D0_DCP_900MA	0x30
#define BQ24188_IUSBMAX_DCP_1500MA			0x40
#define BQ24188_IUSBMAX_DCP_1950MA			0x50
#define BQ24188_IUSBMAX_DCP_2500MA			0x60
#define BQ24188_IUSBMAX_DCP_2000MA			0x70

#define BQ24188_VDDMAX_MASK					0xFC
#define BQ24188_VDDMAX_MIN_MV				3500
#define BQ24188_VDDMAX_STEP_MV				20
#define BQ24188_VDDMAX_SHIFT				0x02

#define BQ24188_TIMEOUT_MASK				0x60
#define BQ24188_TIMEOUT_1D25MIN				0x00
#define BQ24188_TIMEOUT_6HOURS				0x20
#define BQ24188_TIMEOUT_9HOURS				0x40
#define BQ24188_TIMEOUT_DISABLED			0x60

#define BQ24188_WDT_KICK_MASK				0x80
#define BQ24188_WDT_KICK					0x80

#define BQ24188_FAULT_STATUS_MASK			0x07
#define BQ24188_POWER_GOOD					0x00
#define BQ24188_BATTERY_OVP					0x06
#define BQ24188_CHARGER_OVP					0x01

#define BQ24188_STAT_FUNC_MASK				0x08
#define BQ24188_STAT_FUNC_ENABLE			0x08
#define BQ24188_STAT_FUNC_DISABLE			0x00

#define BQ24188_TS_ENABLE_MASK				0x08
#define BQ24188_TS_ENABLE					0x08
#define BQ24188_TS_DISABLE					0x00

#define BQ24188_VINDPM_OFFSET				4200
#define BQ24188_VINDPM_STEP_MV				84
#define BQ24188_VINDPM_MASK					(BIT(2) | BIT(1) | BIT(0))
#define BQ24188_VINDPM_ACTIVE				BIT(6)
#define BQ24188_VINDPM_STATUS_MASK			BIT(6)

#define BQ24188_VFLOAT_STEP_MV				20

#define BQ24188_BOOST_ILIM_MASK				BIT(4)
#define BQ24188_BOOST_ILIM_500MA			0x00
#define BQ24188_BOOST_ILIM_1000MA			BIT(4)


OPPO_BQ24188_EXT int bq24188_get_prop_charge_type(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_get_prop_batt_status(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_get_charging_status(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_set_otg_regulator_enable(struct regulator_dev *rdev);
OPPO_BQ24188_EXT int bq24188_set_otg_regulator_disable(struct regulator_dev *rdev);
OPPO_BQ24188_EXT int bq24188_get_otg_regulator_is_enable(struct regulator_dev *rdev);
#if 1
OPPO_BQ24188_EXT int bq24188_set_otg_enable(void);
OPPO_BQ24188_EXT int bq24188_set_otg_disable(void);
OPPO_BQ24188_EXT int bq24188_get_otg_enable(void);
#endif
OPPO_BQ24188_EXT int bq24188_set_reset_charger(struct opchg_charger *chip, bool reset);
OPPO_BQ24188_EXT int bq24188_set_charging_disable(struct opchg_charger *chip, bool disable);
OPPO_BQ24188_EXT int bq24188_set_suspend_enable(struct opchg_charger *chip, bool enable);
OPPO_BQ24188_EXT int bq24188_set_enable_volatile_writes(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_set_prechg_current(struct opchg_charger *chip, int ipre_mA);
OPPO_BQ24188_EXT int bq24188_set_fastchg_current(struct opchg_charger *chip, int ifast_mA);
OPPO_BQ24188_EXT int bq24188_set_temrchg_current(struct opchg_charger *chip, int iterm_current);
OPPO_BQ24188_EXT int bq24188_set_float_voltage(struct opchg_charger *chip, int vfloat_mv);
OPPO_BQ24188_EXT int bq24188_set_input_chg_current(struct opchg_charger *chip, int iusbin_mA, bool aicl);
OPPO_BQ24188_EXT int bq24188_set_complete_charge_timeout(struct opchg_charger *chip, int val);
OPPO_BQ24188_EXT int bq24188_set_wdt_timer(struct opchg_charger *chip, bool enable);
OPPO_BQ24188_EXT int bq24188_set_wdt_reset(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_check_charging_pre_full(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_check_battovp(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_check_chargerovp(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_set_vindpm_vol(struct opchg_charger *chip, int vol);
OPPO_BQ24188_EXT int bq24188_hw_init(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_get_initial_state(struct opchg_charger *chip);
OPPO_BQ24188_EXT void bq24188_dump_regs(struct opchg_charger *chip);
OPPO_BQ24188_EXT int bq24188_chg_uv(struct opchg_charger *chip, u8 status);
OPPO_BQ24188_EXT int bq24188_fast_chg(struct opchg_charger *chip, u8 status);
OPPO_BQ24188_EXT int bq24188_chg_term(struct opchg_charger *chip, u8 status);
OPPO_BQ24188_EXT int bq24188_safety_timeout(struct opchg_charger *chip, u8 status);
OPPO_BQ24188_EXT void bq24188_usbin_valid_work(struct work_struct *work);
OPPO_BQ24188_EXT void bq24188_usbin_valid_irq_handler(struct opchg_charger *chip);

#endif /*_OPPO_BQ24196_H_*/
