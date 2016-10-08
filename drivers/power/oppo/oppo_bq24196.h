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

#ifndef _OPPO_BQ24196_H_
#define _OPPO_BQ24196_H_

#ifdef OPPO_BQ24196_PAR
#define OPPO_BQ24196_EXT
#else
#define OPPO_BQ24196_EXT extern
#endif


//#define BQ24192

/* Address:00h */
#define REG00_BQ24196_ADDRESS                                   0x00

#define REG00_BQ24196_SUSPEND_MODE_MASK                         BIT(7)
#define REG00_BQ24196_SUSPEND_MODE_DISABLE                      0x00
#define REG00_BQ24196_SUSPEND_MODE_ENABLE                       BIT(7)

#define REG00_BQ24196_INPUT_VOL_LIMIT_MASK                      (BIT(6) | BIT(5) | BIT(4) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_3880MV                    0x00
#define REG00_BQ24196_INPUT_VOL_LIMIT_3960MV                    BIT(3)
#define REG00_BQ24196_INPUT_VOL_LIMIT_4040MV                    BIT(4)
#define REG00_BQ24196_INPUT_VOL_LIMIT_4120MV                    (BIT(4) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4200MV                    BIT(5)
#define REG00_BQ24196_INPUT_VOL_LIMIT_4280MV                    (BIT(5) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4360MV                    (BIT(5) | BIT(4))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4440MV                    (BIT(5) | BIT(4) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4520MV                    BIT(6)
#define REG00_BQ24196_INPUT_VOL_LIMIT_4600MV                    (BIT(6) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4680MV                    (BIT(6) | BIT(4))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4760MV                    (BIT(6) | BIT(4) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4840MV                    (BIT(6) | BIT(5))
#define REG00_BQ24196_INPUT_VOL_LIMIT_4920MV                    (BIT(6) | BIT(5) | BIT(3))
#define REG00_BQ24196_INPUT_VOL_LIMIT_5000MV                    (BIT(6) | BIT(5) | BIT(4))
#define REG00_BQ24196_INPUT_VOL_LIMIT_5080MV                    (BIT(6) | BIT(5) | BIT(4) | BIT(3))

#define REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK                  (BIT(2) | BIT(1) | BIT(0))
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT                 0
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_100MA                 0x00
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_150MA                 BIT(0)
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_500MA                 BIT(1)
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_900MA                 (BIT(1) | BIT(0))
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_1200MA                BIT(2)
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_1500MA                (BIT(2) | BIT(0))
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_2000MA                (BIT(2) | BIT(1))
#define REG00_BQ24196_INPUT_CURRENT_LIMIT_3000MA                (BIT(2) | BIT(1) | BIT(0))


/* Address:01h */
#define REG01_BQ24196_ADDRESS                                   0x01

#define REG01_BQ24196_REGISTER_RESET_MASK                       BIT(7)
#define REG01_BQ24196_REGISTER_KEEP                             0x00
#define REG01_BQ24196_REGISTER_RESET                            BIT(7)

#define REG01_BQ24196_WDT_TIMER_RESET_MASK                      BIT(6)
#define REG01_BQ24196_WDT_TIMER_NORMAL                          0x00
#define REG01_BQ24196_WDT_TIMER_RESET                           BIT(6)

#define REG01_BQ24196_CHARGING_MASK                             BIT(4)
#define REG01_BQ24196_CHARGING_DISABLE                          0x00
#define REG01_BQ24196_CHARGING_ENABLE                           BIT(4)

#define REG01_BQ24196_OTG_MASK                                  BIT(5)
#define REG01_BQ24196_OTG_DISABLE                               0x00
#define REG01_BQ24196_OTG_ENABLE                                BIT(5)

#define REG01_BQ24196_SYS_VOL_LIMIT_MASK                        (BIT(3) | BIT(2) | BIT(1))
#define REG01_BQ24196_SYS_VOL_LIMIT_3000MV                      0x00
#define REG01_BQ24196_SYS_VOL_LIMIT_3100MV                      BIT(1)
#define REG01_BQ24196_SYS_VOL_LIMIT_3200MV                      BIT(2)
#define REG01_BQ24196_SYS_VOL_LIMIT_3300MV                      (BIT(2) | BIT(1))
#define REG01_BQ24196_SYS_VOL_LIMIT_3400MV                      BIT(3)
#define REG01_BQ24196_SYS_VOL_LIMIT_3500MV                      (BIT(3) | BIT(1))
#define REG01_BQ24196_SYS_VOL_LIMIT_3600MV                      (BIT(3) | BIT(2))
#define REG01_BQ24196_SYS_VOL_LIMIT_3700MV                      (BIT(3) | BIT(2) | BIT(1))

#define REG01_BQ24196_BOOST_MODE_CURRENT_LIMIT_MASK             BIT(0)
#define REG01_BQ24196_BOOST_MODE_CURRENT_LIMIT_500MA            0x00
#define REG01_BQ24196_BOOST_MODE_CURRENT_LIMIT_1300MA           BIT(0)


/* Address:02h */
#define REG02_BQ24196_ADDRESS                                   0x02

#define REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_MASK          (BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT         2
#define BQ24196_MIN_FAST_CURRENT_MA                             512
#define BQ24196_MAX_FAST_CURRENT_MA                             2048
#define BQ24196_MIN_FAST_CURRENT_MA_ALLOWED						1088
#define BQ24196_MAX_FAST_CURRENT_MA_20_PERCENT					908
#define BQ24196_FAST_CURRENT_STEP_MA                            64

#define REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_MASK     BIT(0)
#define REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_DISABLE  0x00
#define REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_ENABLE   BIT(0)


/* Address:03h */
#define REG03_BQ24196_ADDRESS                                   0x03

#define REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_MASK           (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_SHIFT          4
#define BQ24196_MIN_PRE_CURRENT_MA                              128
#define BQ24196_MAX_PRE_CURRENT_MA                              2048
#define BQ24196_PRE_CURRENT_STEP_MA                             128

#define REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_MASK          (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_SHIFT         0
#define BQ24196_MIN_TERM_CURRENT_MA                              128
#define BQ24196_MAX_TERM_CURRENT_MA                              2048
#define BQ24196_TERM_CURRENT_STEP_MA                             128


/* Address:04h */
#define REG04_BQ24196_ADDRESS                                   0x04

#define REG04_BQ24196_CHARGING_VOL_LIMIT_MASK                   (BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define REG04_BQ24196_CHARGING_VOL_LIMIT_SHIFT                  2
#define BQ24196_MIN_FLOAT_MV                                    3504
#define BQ24196_MAX_FLOAT_MV                                    4400
#define BQ24196_VFLOAT_STEP_MV                                  16

#define REG04_BQ24196_PRE_TO_FAST_CHARGING_VOL_MASK             BIT(1)
#define REG04_BQ24196_PRE_TO_FAST_CHARGING_VOL_2800MV           0x00
#define REG04_BQ24196_PRE_TO_FAST_CHARGING_VOL_3000MV           BIT(1)

#define REG04_BQ24196_RECHARGING_THRESHOLD_VOL_MASK             BIT(0)
#define REG04_BQ24196_RECHARGING_THRESHOLD_VOL_100MV            0x00
#define REG04_BQ24196_RECHARGING_THRESHOLD_VOL_300MV            BIT(0)


/* Address:05h */
#define REG05_BQ24196_ADDRESS                                   0x05

#define REG05_BQ24196_TERMINATION_MASK                          BIT(7)
#define REG05_BQ24196_TERMINATION_DISABLE                       0x00
#define REG05_BQ24196_TERMINATION_ENABLE                        BIT(7)

#define REG05_BQ24196_TERMINATION_STAT_MASK                     BIT(6)
#define REG05_BQ24196_TERMINATION_STAT_DISABLE                  0x00
#define REG05_BQ24196_TERMINATION_STAT_ENABLE                   BIT(6)

#define REG05_BQ24196_I2C_WATCHDOG_TIME_MASK                    (BIT(5) | BIT(4))
#define REG05_BQ24196_I2C_WATCHDOG_TIME_DISABLE                 0x00
#define REG05_BQ24196_I2C_WATCHDOG_TIME_40S                     BIT(4)
#define REG05_BQ24196_I2C_WATCHDOG_TIME_80S                     BIT(5)
#define REG05_BQ24196_I2C_WATCHDOG_TIME_160S                    (BIT(5) | BIT(4))

#define REG05_BQ24196_CHARGING_SAFETY_TIME_MASK                 BIT(3)
#define REG05_BQ24196_CHARGING_SAFETY_TIME_DISABLE              0x00
#define REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE               BIT(3)

#define REG05_BQ24196_FAST_CHARGING_TIMEOUT_MASK                (BIT(2) | BIT(1))
#define REG05_BQ24196_FAST_CHARGING_TIMEOUT_5H                  0x00
#define REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H                  BIT(1)
#define REG05_BQ24196_FAST_CHARGING_TIMEOUT_12H                 BIT(2)
#define REG05_BQ24196_FAST_CHARGING_TIMEOUT_20H                 (BIT(2) | BIT(1)


/* Address:06h */
#define REG06_BQ24196_ADDRESS                                   0x06

#define REG06_BQ24196_COMPENSATION_RESISTOR_MASK                (BIT(7) | BIT(6) | BIT(5))
#define REG06_BQ24196_COMPENSATION_RESISTOR_0MO                 0x00
#define REG06_BQ24196_COMPENSATION_RESISTOR_10MO                BIT(5)
#define REG06_BQ24196_COMPENSATION_RESISTOR_20MO                BIT(6)
#define REG06_BQ24196_COMPENSATION_RESISTOR_30MO                (BIT(6) | BIT(5))
#define REG06_BQ24196_COMPENSATION_RESISTOR_40MO                BIT(7)
#define REG06_BQ24196_COMPENSATION_RESISTOR_50MO                (BIT(7) | BIT(5))
#define REG06_BQ24196_COMPENSATION_RESISTOR_60MO                (BIT(7) | BIT(6))
#define REG06_BQ24196_COMPENSATION_RESISTOR_70MO                (BIT(7) | BIT(6) | BIT(5))

#define REG06_BQ24196_COMPENSATION_VOLTAGE_MASK                (BIT(4) | BIT(3) | BIT(2))
#define REG06_BQ24196_COMPENSATION_VOLTAGE_0MV                 0x00
#define REG06_BQ24196_COMPENSATION_VOLTAGE_16MV                BIT(2)
#define REG06_BQ24196_COMPENSATION_VOLTAGE_32MV                BIT(3)
#define REG06_BQ24196_COMPENSATION_VOLTAGE_48MV                (BIT(3) | BIT(2))
#define REG06_BQ24196_COMPENSATION_VOLTAGE_64MV                BIT(4)
#define REG06_BQ24196_COMPENSATION_VOLTAGE_80MV                (BIT(4) | BIT(2))
#define REG06_BQ24196_COMPENSATION_VOLTAGE_96MV                (BIT(4) | BIT(3))
#define REG06_BQ24196_COMPENSATION_VOLTAGE_112MV                (BIT(4) | BIT(3) | BIT(2))

#define REG06_BQ24196_TEMP_THRESHOLD_MASK                       (BIT(1) | BIT(0))
#define REG06_BQ24196_TEMP_THRESHOLD_60C                        0x00
#define REG06_BQ24196_TEMP_THRESHOLD_80C                        BIT(0)
#define REG06_BQ24196_TEMP_THRESHOLD_100C                       BIT(1)
#define REG06_BQ24196_TEMP_THRESHOLD_120C                       (BIT(1) | BIT(0))


/* Address:07h */
#define REG07_BQ24196_ADDRESS                                   0x07
#define REG07_BQ24196_BATFET_MASK							BIT(5)
#define REG07_BQ24196_BATFET_OFF								BIT(5)


/* Address:08h */
#define REG08_BQ24196_ADDRESS                                   0x08

#define REG08_BQ24196_VBUS_STAT_MASK                            (BIT(7) | BIT(6))
#define REG08_BQ24196_VBUS_STAT_UNKNOWN                         0x00
#define REG08_BQ24196_VBUS_STAT_USB_HOST_MODE                   BIT(6)
#define REG08_BQ24196_VBUS_STAT_AC_HOST_MODE                    BIT(7)
#define REG08_BQ24196_VBUS_STAT_OTG_MODE                        (BIT(7) | BIT(6))

#define REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK             (BIT(5) | BIT(4))
#define REG08_BQ24196_CHARGING_STATUS_NO_CHARGING               0x00
#define REG08_BQ24196_CHARGING_STATUS_PRE_CHARGING              BIT(4)
#define REG08_BQ24196_CHARGING_STATUS_FAST_CHARGING             BIT(5)
#define REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING             (BIT(5) | BIT(4))

#define REG08_BQ24196_CHARGING_STATUS_DPM_MASK                  BIT(3)
#define REG08_BQ24196_CHARGING_STATUS_DPM_OUT                   0x00
#define REG08_BQ24196_CHARGING_STATUS_DPM_IN                    BIT(3)

#define REG08_BQ24196_CHARGING_STATUS_POWER_MASK                BIT(2)
#define REG08_BQ24196_CHARGING_STATUS_POWER_OUT                 0x00
#define REG08_BQ24196_CHARGING_STATUS_POWER_IN                  BIT(2)


/* Address:09h */
#define REG09_BQ24196_ADDRESS                                   0x09

#define REG09_BQ24196_WDT_FAULT_MASK                            BIT(7)
#define REG09_BQ24196_WDT_FAULT_NORMAL                          0x00
#define REG09_BQ24196_WDT_FAULT_EXPIRATION                      BIT(7)

#define REG09_BQ24196_CHARGING_MASK                             (BIT(5) | BIT(4))
#define REG09_BQ24196_CHARGING_NORMAL                           0x00
#define REG09_BQ24196_CHARGING_INPUT_ERROR                      BIT(4)
#define REG09_BQ24196_CHARGING_THER_SHUTDOWN                    BIT(5)
#define REG09_BQ24196_CHARGING_TIMEOUT_ERROR                    (BIT(5) | BIT(4))

#define REG09_BQ24196_BATTERY_VOLATGE_MASK                      BIT(3)
#define REG09_BQ24196_BATTERY_VOLATGE_HIGH_ERROR                BIT(3)

#define REG09_BQ24196_BATTERY_TEMP_MASK                         (BIT(2) | BIT(1) | BIT(0))
#define REG09_BQ24196_BATTERY_TEMP_GOOD                         0x00
#define REG09_BQ24196_BATTERY_TEMP_LOW_ERROR                    (BIT(2) | BIT(0))
#define REG09_BQ24196_BATTERY_TEMP_HIGH_ERROR                   (BIT(2) | BIT(1))


/* Address:0Ah */
#define REG0A_BQ24196_ADDRESS                                   0x0A

// config register
#define BQ24196_LAST_CNFG_REG                                   0x06
// status register
#define BQ24196_FIRST_STATUS_REG                                0x07
#define BQ24196_LAST_STATUS_REG                                 0x0A

#define BQ24196_VINDPM_MASK										(BIT(6) | BIT(5) | BIT(4) | BIT(3))
#define BQ24196_VINDPM_STEP_MV									80
#define BQ24196_VINDPM_OFFSET									3880
#define BQ24196_VINDPM_SHIFT									3


#ifdef OPPO_BQ24196_PAR
u8 reg08_val = 0, reg09_val = 0;
u8 pre_reg08_val = 0, pre_reg09_val = 0;
#else
extern u8 reg08_val, reg09_val;
extern u8 pre_reg08_val, pre_reg09_val;
#endif

OPPO_BQ24196_EXT int bq24196_get_prop_charge_type(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_get_prop_batt_status(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_get_charging_status(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_set_otg_regulator_enable(struct regulator_dev *rdev);
OPPO_BQ24196_EXT int bq24196_set_otg_regulator_disable(struct regulator_dev *rdev);
OPPO_BQ24196_EXT int bq24196_get_otg_regulator_is_enable(struct regulator_dev *rdev);
#if 1
OPPO_BQ24196_EXT int bq24196_set_otg_enable(void);
OPPO_BQ24196_EXT int bq24196_set_otg_disable(void);
OPPO_BQ24196_EXT int bq24196_get_otg_enable(void);
#endif
OPPO_BQ24196_EXT int bq24196_set_reset_charger(struct opchg_charger *chip, bool reset);
OPPO_BQ24196_EXT int bq24196_set_charging_disable(struct opchg_charger *chip, bool disable);
OPPO_BQ24196_EXT int bq24196_set_suspend_enable(struct opchg_charger *chip, bool enable);
OPPO_BQ24196_EXT int bq24196_set_enable_volatile_writes(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_set_precharger_voltage(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_set_recharger_voltage(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_set_prechg_current(struct opchg_charger *chip, int ipre_mA);
OPPO_BQ24196_EXT int bq24196_set_fastchg_current(struct opchg_charger *chip, int ifast_mA);
OPPO_BQ24196_EXT int bq24196_set_temrchg_current(struct opchg_charger *chip, int iterm_current);
OPPO_BQ24196_EXT int bq24196_set_float_voltage(struct opchg_charger *chip, int vfloat_mv);
OPPO_BQ24196_EXT int bq24196_set_input_chg_current(struct opchg_charger *chip, int iusbin_mA, bool aicl);
OPPO_BQ24196_EXT int bq24196_set_complete_charge_timeout(struct opchg_charger *chip, int val);
OPPO_BQ24196_EXT int bq24196_set_wdt_timer(struct opchg_charger *chip, bool enable);
OPPO_BQ24196_EXT void bq24196_set_batfet_off(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_set_wdt_reset(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_check_charging_pre_full(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_check_battovp(struct opchg_charger *chip);
#ifdef BQ24192
OPPO_BQ24196_EXT int bq24196_set_compenstion_resistor(struct opchg_charger *chip, int reg);
OPPO_BQ24196_EXT int bq24196_set_compenstion_voltage(struct opchg_charger *chip, int reg);
#endif
OPPO_BQ24196_EXT int bq24196_hw_init(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_get_initial_state(struct opchg_charger *chip);
OPPO_BQ24196_EXT void bq24196_dump_regs(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_chg_uv(struct opchg_charger *chip, u8 status);
OPPO_BQ24196_EXT int bq24196_chg_ov(struct opchg_charger *chip, u8 status);
OPPO_BQ24196_EXT int bq24196_fast_chg(struct opchg_charger *chip, u8 status);
OPPO_BQ24196_EXT int bq24196_chg_term(struct opchg_charger *chip, u8 status);
OPPO_BQ24196_EXT int bq24196_safety_timeout(struct opchg_charger *chip, u8 status);
OPPO_BQ24196_EXT void bq24196_chg_irq_handler(int irq, struct opchg_charger *chip);
OPPO_BQ24196_EXT void bq24196_usbin_valid_work(struct work_struct *work);
OPPO_BQ24196_EXT void bq24196_usbin_valid_irq_handler(struct opchg_charger *chip);
OPPO_BQ24196_EXT int bq24196_set_vindpm_vol(struct opchg_charger *chip, int vol);

#endif /*_OPPO_BQ24196_H_*/
