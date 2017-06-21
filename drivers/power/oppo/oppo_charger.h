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

#ifndef _OPPO_CHARGER_H_
#define _OPPO_CHARGER_H_

#ifdef OPPO_CHARGER_PAR
#define OPPO_CHARGER_EXT
#else
#define OPPO_CHARGER_EXT extern
#endif

enum {
OPCHG_UNKOWN_ID=0,
OPCHG_SMB358_ID,
OPCHG_SMB1357_ID,
OPCHG_BQ24196_ID,
OPCHG_BQ24157_ID,
OPCHG_BQ24188_ID,
OPCHG_MAX_ID,
};
#define DEVICE_CHARGER_IC_VERSION		"1.0"
#define DEVICE_CHARGER_IC_TYPE_BQ24196	"BQ24196"
#define DEVICE_CHARGER_IC_TYPE_BQ24188	"BQ24188"
#define DEVICE_CHARGER_IC_TYPE_BQ24157	"BQ24157"
#define DEVICE_CHARGER_IC_TYPE_SMB358	"SMB358"
#define DEVICE_CHARGER_IC_TYPE_SMB1357	"SMB1357"
#define DEVICE_CHARGER_IC_TYPE_UNKOWN	"UNKOWN"

enum {
OPCHG_BMS_UNKOWN_ID=0,
OPCHG_BMS_BQ27541_ID = OPCHG_BMS_UNKOWN_ID,
OPCHG_BMS_BQ27411_ID,
OPCHG_BMS_MAX_ID,
};
#define DEVICE_BMS_IC_VERSION		"1.0"
#define DEVICE_BMS_IC_TYPE_BQ27541	"BQ27541"
#define DEVICE_BMS_IC_TYPE_BQ27411	"BQ27411"
#define DEVICE_BMS_IC_TYPE_UNKOWN	"UNKOWN"


enum {
OPCHG_VOOC_UNKOWN_ID=0,
OPCHG_VOOC_PIC16F_ID,
OPCHG_VOOC_STM8S_ID,
OPCHG_VOOC_MAX_ID,
};
#define DEVICE_FASTCHARGER_MCU_VERSION		"1.0"
#define DEVICE_FASTCHARGER_MCU_TYPE_PIC16F	"PIC16F"
#define DEVICE_FASTCHARGER_MCU_TYPE_STM8S	"STM8S"
#define DEVICE_FASTCHARGER_MCU_TYPE_UNKOWN	"UNKOWN"

#if 0
enum {
OPCHG_VOOC_WATCHDOG_OUT,			// fast charging is  watchdog delay
OPCHG_VOOC_FAST_OUT,				// fast charging is drop out delay
OPCHG_VOOC_FAST_OUT_DELAY, 		// fast charging is drop out delay
OPCHG_VOOC_TO_STANDARD,		// fast charging is to Standard charging
OPCHG_VOOC_TO_FAST,			// fast charging is to fast charging
OPCHG_VOOC_IN_FAST,			// fast charging is in fast charging
};
#else
#define  OPCHG_VOOC_WATCHDOG_OUT		0		// fast charging is  watchdog delay
#define  OPCHG_VOOC_TO_STANDARD		1		// fast charging is to Standard charging
#define  OPCHG_VOOC_TO_FAST			2		// fast charging is to fast charging
#define  OPCHG_VOOC_IN_FAST			3		// fast charging is in fast charging
#endif

#define CURRENT_500MA				500
#define CURRENT_800MA				800
#define CURRENT_900MA				900
#define CURRENT_1200MA				1200
#define CURRENT_1500MA				1500
#define CURRENT_1950MA				1950
#define CURRENT_2000MA				2000
#define CURRENT_2500MA				2500

struct smb_irq_info {
    const char                      *name;
    int                             (*smb_irq)(struct opchg_charger *chip,u8 rt_stat);
    int                             high;
    int                             low;
};

struct irq_handler_info {
    u8                              stat_reg;
    u8                              val;
    u8                              prev_val;
    struct smb_irq_info             irq_info[4];
};

OPPO_CHARGER_EXT int opchg_read_reg(struct opchg_charger *chip, int reg, u8 *val);
OPPO_CHARGER_EXT int opchg_write_reg(struct opchg_charger *chip, int reg, u8 val);
OPPO_CHARGER_EXT int opchg_masked_write(struct opchg_charger *chip, int reg, u8 mask, u8 val);
OPPO_CHARGER_EXT int opchg_regulator_init(struct opchg_charger *chip);
OPPO_CHARGER_EXT int opchg_check_i2c_status(struct opchg_charger *chip);

#ifdef OPPO_USE_2CHARGER
OPPO_CHARGER_EXT void opchg_get_prop_fastcharger_type(struct opchg_charger *chip);
#endif
OPPO_CHARGER_EXT void opchg_get_prop_charge_type(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_get_charging_status(struct opchg_charger *chip);
OPPO_CHARGER_EXT int opchg_get_prop_batt_status(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_set_enable_volatile_writes(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_set_complete_charge_timeout(struct opchg_charger *chip, int val);
OPPO_CHARGER_EXT void opchg_set_reset_charger(struct opchg_charger *chip, bool reset);
OPPO_CHARGER_EXT void opchg_set_precharger_voltage(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_set_recharger_voltage(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_set_prechg_current(struct opchg_charger *chip, int ipre_mA);
OPPO_CHARGER_EXT void opchg_set_input_chg_current(struct opchg_charger *chip, int mA, bool aicl);
OPPO_CHARGER_EXT void opchg_set_fast_chg_current(struct opchg_charger *chip, int mA);
OPPO_CHARGER_EXT void opchg_set_float_voltage(struct opchg_charger *chip, int mV);
OPPO_CHARGER_EXT void opchg_set_charging_disable(struct opchg_charger *chip, bool disable);
OPPO_CHARGER_EXT void opchg_set_suspend_enable(struct opchg_charger *chip, bool enable);
OPPO_CHARGER_EXT int opchg_hw_init(struct opchg_charger *chip);
OPPO_CHARGER_EXT int opchg_get_initial_state(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_set_wdt_reset(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_set_wdt_timer(struct opchg_charger *chip, bool enable);
OPPO_CHARGER_EXT int opchg_check_charging_pre_full(struct opchg_charger *chip);
OPPO_CHARGER_EXT int opchg_check_battovp(struct opchg_charger *chip);
OPPO_CHARGER_EXT int opchg_check_chargerovp(struct opchg_charger *chip);
OPPO_CHARGER_EXT irqreturn_t opchg_chg_irq_handler(int irq, void *dev_id);
OPPO_CHARGER_EXT void opchg_dump_regs(struct opchg_charger *chip);
OPPO_CHARGER_EXT int qpnp_charger_type_get(struct opchg_charger *chip);
OPPO_CHARGER_EXT void opchg_switch_to_usbin(struct opchg_charger *chip,bool enable);
OPPO_CHARGER_EXT int opchg_get_otg_regulator_is_enable(struct regulator_dev *rdev);
#if 1
OPPO_CHARGER_EXT int opchg_set_otg_enable(void);
OPPO_CHARGER_EXT int opchg_set_otg_disable(void);
OPPO_CHARGER_EXT int opchg_get_otg_enable(void);
#endif
OPPO_CHARGER_EXT void opchg_set_vindpm_vol(struct opchg_charger *chip, int mV);

extern int opchg_get_charger_inout(void);
extern int opchg_get_charger_inout_cblpwr(void);
#endif /*_OPPO_CHARGER_H_*/
