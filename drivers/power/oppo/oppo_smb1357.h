/*******************************************************************************
* Copyright (c)  2014- 2014  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* VENDOR_EDIT
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

#ifndef _OPPO_SMB1357_H_
#define _OPPO_SMB1357_H_

#ifdef OPPO_SMB1357_PAR
#define OPPO_SMB1357_EXT
#else
#define OPPO_SMB1357_EXT extern
#endif

enum {
    SMB1357_FAST_TIME_192MIN = 0,
    SMB1357_FAST_TIME_384MIN,
    SMB1357_FAST_TIME_768MIN,
    SMB1357_FAST_TIME_1536MIN,
};

enum {
    SMB1357_PRE_TIME_24MIN = 0,
    SMB1357_PRE_TIME_48MIN,
    SMB1357_PRE_TIME_96MIN,
    SMB1357_PRE_TIME_192MIN,
};


/* Address:C02h */
#define REG02_SMB1357_ADDRESS                                   0x02

#define REG02_SMB1357_THERM_A_SWITCHING_FREQ_MASK               BIT(4)
#define REG02_SMB1357_THERM_A_SWITCHING_FREQ_2_0MHZ             BIT(4)
#define REG02_SMB1357_THERM_A_SWITCHING_FREQ_3_0MHZ             0x00


/* Address:C03h */
#define REG03_SMB1357_ADDRESS                                   0x03

#define REG03_SMB1357_TEMP_THERMAL_LOOPS_MASK                   (BIT(7) | BIT(6))
#define REG03_SMB1357_TEMP_THERMAL_LOOPS_100                    0x00
#define REG03_SMB1357_TEMP_THERMAL_LOOPS_110                    BIT(6)
#define REG03_SMB1357_TEMP_THERMAL_LOOPS_120                    BIT(7)
#define REG03_SMB1357_TEMP_THERMAL_LOOPS_130                    (BIT(7) | BIT(6))

#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_MASK            (BIT(5) | BIT(4) | BIT(3))
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_SHIFT           3
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_300MA           0x00
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_50MA            BIT(3)
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_100MA           BIT(4)
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_150MA           (BIT(4) | BIT(3))
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_200MA           BIT(5)
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_250MA           (BIT(5) | BIT(3))
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_500MA           (BIT(5) | BIT(4))
#define REG03_SMB1357_CURRENT_TERMINATION_LEVEL_600MA           (BIT(5) | BIT(4) | BIT(3))

#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_MASK      (BIT(2) | BIT(1) | BIT(0))
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_SHIFT     0
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_DISABLE   0x00
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_25MV      BIT(0)
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_50MV      BIT(1)
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_75MV      (BIT(1) | BIT(0))
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_100MV     BIT(2)
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_125MV     (BIT(2) | BIT(0))
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_150MV     (BIT(2) | BIT(1))
#define REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_175MV     (BIT(2) | BIT(1) | BIT(0))


/* Address:C04h */
#define REG04_SMB1357_ADDRESS                                   0x04

#define REG04_SMB1357_CHARGER_INHIBIT_VFLT_MASK                 (BIT(7) | BIT(6))
#define REG04_SMB1357_CHARGER_INHIBIT_VFLT_50MV                                 0x00
#define REG04_SMB1357_CHARGER_INHIBIT_VFLT_100MV                BIT(6)
#define REG04_SMB1357_CHARGER_INHIBIT_VFLT_200MV                BIT(7)
#define REG04_SMB1357_CHARGER_INHIBIT_VFLT_300MV                (BIT(7) | BIT(6))

#define REG04_SMB1357_CHARGER_VSYS_VOLTAGE_MASK                 (BIT(5) | BIT(4))
#define REG04_SMB1357_CHARGER_VSYS_VOLTAGE_3150MV               0x00
#define REG04_SMB1357_CHARGER_VSYS_VOLTAGE_3450MV               BIT(4)
#define REG04_SMB1357_CHARGER_VSYS_VOLTAGE_3600MV               BIT(5)

#define REG04_SMB1357_OTG_THRESHOLD_VOLTAGE_MASK                (BIT(3) | BIT(2))
#define REG04_SMB1357_OTG_THRESHOLD_VOLTAGE_2700MV              0x00
#define REG04_SMB1357_OTG_THRESHOLD_VOLTAGE_2800MV              BIT(2)
#define REG04_SMB1357_OTG_THRESHOLD_VOLTAGE_3100MV              BIT(3)
#define REG04_SMB1357_OTG_THRESHOLD_VOLTAGE_3300MV              (BIT(3) | BIT(2))

#define REG04_SMB1357_PRE_TO_FAST_THRESHOLD_VOLTAGE_MASK        (BIT(1) | BIT(0))
#define REG04_SMB1357_PRE_TO_FAST_THRESHOLD_VOLTAGE_2400MV      0x00
#define REG04_SMB1357_PRE_TO_FAST_THRESHOLD_VOLTAGE_2600MV      BIT(0)
#define REG04_SMB1357_PRE_TO_FAST_THRESHOLD_VOLTAGE_2800MV      BIT(1)
#define REG04_SMB1357_PRE_TO_FAST_THRESHOLD_VOLTAGE_3000MV      (BIT(1) | BIT(0))


/* Address:C05h */
#define REG05_SMB1357_ADDRESS                                   0x05

#define REG05_SMB1357_MAX_SYSTEM_VOLTAGE_MASK                   BIT(4)
#define REG05_SMB1357_MAX_SYSTEM_VOLTAGE_VFLT100MV              0x00
#define REG05_SMB1357_MAX_SYSTEM_VOLTAGE_VFLT200MV              BIT(4)

#define REG05_SMB1357_RECHARGER_VOL_THRESHOLD_MASK              BIT(2)
#define REG05_SMB1357_RECHARGER_VOL_THRESHOLD_100MV             0x00
#define REG05_SMB1357_RECHARGER_VOL_THRESHOLD_200MV             BIT(2)


/* Address:C06h */
#define REG06_SMB1357_ADDRESS                                   0x06

#define REG06_SMB1357_AFVC_IN_TAPER_MODE_MASK                   BIT(3)
#define REG06_SMB1357_AFVC_IN_TAPER_MODE_ENABLE                 0x00
#define REG06_SMB1357_AFVC_IN_TAPER_MODE_DISABLE                BIT(3)

#define REG06_SMB1357_DIGITAL_THERMAL_LOOP_MASK                 BIT(0)
#define REG06_SMB1357_DIGITAL_THERMAL_LOOP_ENABLE               0x00
#define REG06_SMB1357_DIGITAL_THERMAL_LOOP_DISABLE              BIT(0)


/* Address:C07h */
#define REG07_SMB1357_ADDRESS                                   0x07

#define REG07_SMB1357_IRQ_MASK                                  0xFF
#define REG07_SMB1357_HARD_LIMIT_IRQ                            BIT(7)
#define REG07_SMB1357_SOFT_LIMIT_IRQ                            BIT(6)
#define REG07_SMB1357_OTG_FAIL_IRQ                              BIT(5)
#define REG07_SMB1357_OTG_CURRENT_OVER_IRQ                      BIT(4)
#define REG07_SMB1357_USBIN_OV_IRQ                              BIT(3)
#define REG07_SMB1357_USBIN_UV_IRQ                              BIT(2)
#define REG07_SMB1357_AICL_DONE_IRQ                             BIT(1)
#define REG07_SMB1357_INTERNAL_THEP_SHUTDOWN_IRQ                BIT(0)


/* Address:C08h */
#define REG08_SMB1357_ADDRESS                                   0x08

#define REG08_SMB1357_IRQ_MASK                                  0xFF
#define REG08_SMB1357_SAFETY_TIME_IRQ                           BIT(7)
#define REG08_SMB1357_CHARGE_ERROR_IRQ                          BIT(6)
#define REG08_SMB1357_BATTERY_OV_IRQ                            BIT(5)
#define REG08_SMB1357_CHARGE_PHASE_IRQ                          BIT(4)
#define REG08_SMB1357_CHARGER_INHIBIT_IRQ                       BIT(3)
#define REG08_SMB1357_POWER_OK_IRQ                              BIT(2)
#define REG08_SMB1357_BATTERY_MISSING_IRQ                       BIT(1)
#define REG08_SMB1357_LOW_BATTERY_IRQ                           BIT(0)


/* Address:C09h */
#define REG09_SMB1357_ADDRESS                                   0x09

#define REG09_SMB1357_IRQ_MASK                                  0x0F
#define REG09_SMB1357_WATCHDOG_OVERTIME_IRQ                     BIT(3)
#define REG09_SMB1357_SOURCE_DETECTION_STATUS_IRQ               BIT(2)
#define REG09_SMB1357_DCIN_OV_IRQ                               BIT(1)
#define REG09_SMB1357_DCIN_UV_IRQ                               BIT(0)


/* Address:C0Ah */
#define REG0A_SMB1357_ADDRESS                                   0x0A

#define REG0A_SMB1357_DCIN_VOLTAGE_MASK                         (BIT(7) | BIT(6) | BIT(5))
#define REG0A_SMB1357_DCIN_VOLTAGE_ONLY_5V                      0x00
#define REG0A_SMB1357_DCIN_VOLTAGE_5V_OR_9V                     BIT(5)
#define REG0A_SMB1357_DCIN_VOLTAGE_5V_TO_9V                     BIT(6)
#define REG0A_SMB1357_DCIN_VOLTAGE_ONLY_9V                      (BIT(6) | BIT(5))

#define REG0A_SMB1357_DCIN_INPUT_CURRENT_MASK                   (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_SHIFT                  0
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_300MA                  0x00
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_400MA                  BIT(0)
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_450MA                  BIT(1)
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_475MA                  (BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_500MA                  BIT(2)
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_550MA                  (BIT(2) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_600MA                  (BIT(2) | BIT(1))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_650MA                  (BIT(2) | BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_700MA                  BIT(3)
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_900MA                  (BIT(3) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_950MA                  (BIT(3) | BIT(1))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1000MA                 (BIT(3) | BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1100MA                 (BIT(3) | BIT(2))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1200MA                 (BIT(3) | BIT(2) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1400MA                 (BIT(3) | BIT(2) | BIT(1))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1450MA                 (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1500MA                 BIT(4)
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1600MA                 (BIT(4) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1800MA                 (BIT(4) | BIT(1))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1850MA                 (BIT(4) | BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1880MA                 (BIT(4) | BIT(2))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1910MA                 (BIT(4) | BIT(2) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1930MA                 (BIT(4) | BIT(2) | BIT(1))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1950MA                 (BIT(4) | BIT(2) | BIT(1) | BIT(0))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_1970MA                 (BIT(4) | BIT(3))
#define REG0A_SMB1357_DCIN_INPUT_CURRENT_2000MA                 (BIT(4) | BIT(3) | BIT(0))


/* Address:C0Ch */
#define REG0C_SMB1357_ADDRESS                                   0x0C

#define REG0C_SMB1357_USBIN_VOLTAGE_ONLY_MASK                   (BIT(7) | BIT(6) | BIT(5))
#define REG0C_SMB1357_USBIN_VOLTAGE_ONLY_5V                     0x00
#define REG0C_SMB1357_USBIN_VOLTAGE_5V_OR_9V                    BIT(5)
#define REG0C_SMB1357_USBIN_VOLTAGE_5V_TO_9V                    BIT(6)
#define REG0C_SMB1357_USBIN_VOLTAGE_ONLY_9V                     (BIT(6) | BIT(5))

#define REG0C_SMB1357_USBIN_INPUT_CURRENT_MASK                  (BIT(4)|BIT(3)|	BIT(2)|	BIT(1)|BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_SHIFT                 0
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_300MA                 0x00
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_400MA                 BIT(0)
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_450MA                 BIT(1)
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_475MA                 (BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_500MA                 BIT(2)
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_550MA                 (BIT(2) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_600MA                 (BIT(2) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_650MA                 (BIT(2) | BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_700MA                 BIT(3)
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_900MA                 (BIT(3) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_950MA                 (BIT(3) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1000MA                (BIT(3) | BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1100MA                (BIT(3) | BIT(2))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1200MA                (BIT(3) | BIT(2) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1400MA                (BIT(3) | BIT(2) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1450MA                (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1500MA                BIT(4)
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1600MA                (BIT(4) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1800MA                (BIT(4) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1850MA                (BIT(4) | BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1880MA                (BIT(4) | BIT(2))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1910MA                (BIT(4) | BIT(2) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1930MA                (BIT(4) | BIT(2) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1950MA                (BIT(4) | BIT(2) | BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_1970MA                (BIT(4) | BIT(3))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_2000MA                (BIT(4) | BIT(3) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_2050MA                (BIT(4) | BIT(3) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_2100MA                (BIT(4) | BIT(3) | BIT(1) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_2300MA                (BIT(4) | BIT(3) | BIT(2))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_2400MA                (BIT(4) | BIT(3) | BIT(2) | BIT(0))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_2500MA                (BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define REG0C_SMB1357_USBIN_INPUT_CURRENT_3000MA                (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))


/* Address:C10h */
#define REG10_SMB1357_ADDRESS                                   0x10

#define REG10_SMB1357_WATCHDOG_TIMEOUT_MASK                     (BIT(6) | BIT(5))
#define REG10_SMB1357_WATCHDOG_TIMEOUT_18S                      0x00
#define REG10_SMB1357_WATCHDOG_TIMEOUT_36S                      BIT(5)
#define REG10_SMB1357_WATCHDOG_TIMEOUT_72S                      BIT(6)

#define REG10_SMB1357_SAFETY_TIMER_AFTER_WATCHDOG_IRQ_MASK      (BIT(4) | BIT(3))
#define REG10_SMB1357_SAFETY_TIMER_AFTER_WATCHDOG_IRQ_12MIN     0x00
#define REG10_SMB1357_SAFETY_TIMER_AFTER_WATCHDOG_IRQ_24MIN     BIT(3)
#define REG10_SMB1357_SAFETY_TIMER_AFTER_WATCHDOG_IRQ_48MIN     BIT(4)
#define REG10_SMB1357_SAFETY_TIMER_AFTER_WATCHDOG_IRQ_96MIN     (BIT(4) | BIT(3))

#define REG10_SMB1357_WATCHDOG_IRQ_MASK                         BIT(2)
#define REG10_SMB1357_WATCHDOG_IRQ_DISABLE                      0x00
#define REG10_SMB1357_WATCHDOG_IRQ_ENABLE                       BIT(2)

#define REG10_SMB1357_WATCHDOG_OPTION_MASK                      BIT(1)
#define REG10_SMB1357_WATCHDOG_OPTION_AFTER_ACK                 0x00
#define REG10_SMB1357_WATCHDOG_OPTION_ALWAYS                    BIT(1)

#define REG10_SMB1357_WATCHDOG_TIMER_MASK                       BIT(0)
#define REG10_SMB1357_WATCHDOG_TIMER_DISABLE                    0x00
#define REG10_SMB1357_WATCHDOG_TIMER_ENABLE                     BIT(0)


/* Address:C11h */
#define REG11_SMB1357_ADDRESS                                   0x11

#define REG11_SMB1357_ENTER_SDP_OR_SUSPEND_MODE_MASK            BIT(4)
#define REG11_SMB1357_ENTER_SDP_MODE                            0x00
#define REG11_SMB1357_ENTER_SUSPEND_MODE                        BIT(4)

#define REG11_SMB1357_APSD_MASK                                 BIT(0)
#define REG11_SMB1357_APSD_DISABLE                              0x00
#define REG11_SMB1357_APSD_ENABLE                               BIT(0)


/* Address:C12h */
#define REG12_SMB1357_ADDRESS                                   0x12

#define REG12_SMB1357_OTG_CURRENT_LIMIT_MASK                    (BIT(3) | BIT(2))
#define REG12_SMB1357_OTG_CURRENT_LIMIT_AT_USB_IN_250MA         0x00
#define REG12_SMB1357_OTG_CURRENT_LIMIT_AT_USB_IN_600MA         BIT(2)
#define REG12_SMB1357_OTG_CURRENT_LIMIT_AT_USB_IN_750MA         BIT(3)
#define REG12_SMB1357_OTG_CURRENT_LIMIT_AT_USB_IN_1000MA        (BIT(3) | BIT(2))


/* Address:C13h */
#define REG13_SMB1357_ADDRESS                                   0x13

#define REG13_SMB1357_LOW_BATTERY_THESHOLD_MASK                 (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG13_SMB1357_LOW_BATTERY_THESHOLD_DISABLE              0x00
#define REG13_SMB1357_LOW_BATTERY_THESHOLD_2500MV               BIT(0)
#define REG13_SMB1357_LOW_BATTERY_THESHOLD_3580MV               (BIT(3) | BIT(2) | BIT(1) | BIT(0))


/* Address:C14h */
#define REG14_SMB1357_ADDRESS                                   0x14

#define REG14_SMB1357_CHG_ENABLE_SOURCE_MASK                    BIT(7)
#define REG14_SMB1357_CHG_ENABLE_FOR_COMMAND_REGISTER           0x00
#define REG14_SMB1357_CHG_ENABLE_FOR_ENABLE_PIN                 BIT(7) 

#define REG14_SMB1357_CHG_ENABLE_POLARITY_MASK                  BIT(6)
#define REG14_SMB1357_CHG_ENABLE_FOR_ACTIVE_HIGH                0x00
#define REG14_SMB1357_CHG_ENABLE_FOR_ACTIVE_LOW                 BIT(6) 

#define REG14_SMB1357_PRE_TO_FAST_CHARGING_MODE_MASK            BIT(5)
#define REG14_SMB1357_PRE_TO_FAST_CHARGING_FOR_AUTO             0x00
#define REG14_SMB1357_PRE_TO_FAST_CHARGING_FOR_COMMAND          BIT(5) 

#define REG14_SMB1357_CURRENT_TERMINATION_MASK                  BIT(3)
#define REG14_SMB1357_CURRENT_TERMINATION_ENABLE                0x00
#define REG14_SMB1357_CURRENT_TERMINATION_DISABLE               BIT(3) 

#define REG14_SMB1357_AUTO_RECHARGE_MASK                        BIT(2)
#define REG14_SMB1357_AUTO_RECHARGE_ENABLE                      0x00
#define REG14_SMB1357_AUTO_RECHARGE_DISABLE                     BIT(2) 

#define REG14_SMB1357_HOLD_OFF_CHARGING_TIME_MASK               BIT(1)
#define REG14_SMB1357_HOLD_OFF_CHARGING_FOR_700US               0x00
#define REG14_SMB1357_HOLD_OFF_CHARGING_FOR_350MS               BIT(1) 

#define REG14_SMB1357_CHARGER_INHIBIT_MASK                      BIT(0)	
#define REG14_SMB1357_CHARGER_INHIBIT_ENABLE                    BIT(0) 	
#define REG14_SMB1357_CHARGER_INHIBIT_DISABLE                   0x00


/* Address:C16h */
#define REG16_SMB1357_ADDRESS                                   0x16

#define REG16_SMB1357_CHARGER_SAFETY_TIMER_ENABLE_MASK          (BIT(5)|BIT(4))
#define REG16_SMB1357_CHARGER_SAFETY_TIMER_ENABLE_PRE_TOTAL     0x00
#define REG16_SMB1357_CHARGER_SAFETY_TIMER_ENABLE_TOTAL         BIT(4)
#define REG16_SMB1357_CHARGER_SAFETY_TIMER_ENABLE_NONE          BIT(5)

#define REG16_SMB1357_CHARGER_TOTAL_TIME_MASK                   (BIT(3)|BIT(2))
#define REG16_SMB1357_CHARGER_TOTAL_TIME_192MIN                 0x00
#define REG16_SMB1357_CHARGER_TOTAL_TIME_384MIN                 BIT(2)
#define REG16_SMB1357_CHARGER_TOTAL_TIME_768MIN                 BIT(3)
#define REG16_SMB1357_CHARGER_TOTAL_TIME_1536MIN                (BIT(3)|BIT(2))

#define REG16_SMB1357_PRE_CHARGER_TOTAL_TIME_MASK               (BIT(1)|BIT(0))
#define REG16_SMB1357_PRE_CHARGER_TOTAL_TIME_24MIN              0x00
#define REG16_SMB1357_PRE_CHARGER_TOTAL_TIME_48MIN              BIT(0)
#define REG16_SMB1357_PRE_CHARGER_TOTAL_TIME_96MIN              BIT(1)
#define REG16_SMB1357_PRE_CHARGER_TOTAL_TIME_192MIN             (BIT(1)|BIT(0))


/* Address:C17h */
#define REG17_SMB1357_ADDRESS                                   0x17

#define REG17_SMB1357_STAT_PIN_CONFIG_MASK                      BIT(4)
#define REG17_SMB1357_STAT_PIN_CONFIG_PULSE                     0x00
#define REG17_SMB1357_STAT_PIN_CONFIG_STATIC                    BIT(4)

#define REG17_SMB1357_STAT_ACTIVE_MASK                          BIT(1)
#define REG17_SMB1357_STAT_LOW_ACTIVE                           0x00
#define REG17_SMB1357_STAT_HIGH_ACTIVE                          BIT(1)

#define REG17_SMB1357_CMD_A_STAT_DISABLE_MASK                   BIT(0)
#define REG17_SMB1357_CMD_A_STAT_ENABLE_BIT                     0x00
#define REG17_SMB1357_CMD_A_STAT_DISABLE_BIT                    BIT(0)


/* Address:C19h */
#define REG19_SMB1357_ADDRESS                                   0x19

#define REG19_SMB1357_BATTERY_MISSING_CHECK_PLUG_IN_MASK        BIT(4)
#define REG19_SMB1357_BATTERY_MISSING_CHECK_PLUG_IN_DISABLE     0x00
#define REG19_SMB1357_BATTERY_MISSING_CHECK_PLUG_IN_ENABLE      BIT(4)

#define REG19_SMB1357_BATTERY_MISSING_CHECK_POLLING_MASK        BIT(3)
#define REG19_SMB1357_BATTERY_MISSING_CHECK_POLLING_DISABLE     0x00
#define REG19_SMB1357_BATTERY_MISSING_CHECK_POLLING_ENABLE      BIT(3)

#define REG19_SMB1357_BATTERY_MISSING_ALGORITHM_MASK            BIT(2)
#define REG19_SMB1357_BATTERY_MISSING_ALGORITHM_ENABLE          BIT(2)
#define REG19_SMB1357_BATTERY_MISSING_ALGORITHM_DISABLE         0x00

#define REG19_SMB1357_BATTERY_MISSING_PIN_MASK                  (BIT(1) | BIT(0))
#define REG19_SMB1357_BATTERY_MISSING_BMD_PIN_ENABLE            BIT(0)
#define REG19_SMB1357_BATTERY_MISSING_THERM_PIN_ENABLE          BIT(1)


/* Address:C1Ch */
#define REG1C_SMB1357_ADDRESS                                   0x1C

#define REG1C_SMB1357_PRE_CHARGER_CURRENT_MASK                  (BIT(7) | BIT(6) | BIT(5))
#define REG1C_SMB1357_PRE_CHARGER_CURRENT_SHIFT                 5
#define REG1C_SMB1357_PRE_CHARGER_CURRENT_100MA                 0x00
#define REG1C_SMB1357_PRE_CHARGER_CURRENT_150MA                 BIT(5)
#define REG1C_SMB1357_PRE_CHARGER_CURRENT_200MA                 BIT(6)
#define REG1C_SMB1357_PRE_CHARGER_CURRENT_250MA                 (BIT(6) | BIT(5))
#define REG1C_SMB1357_PRE_CHARGER_CURRENT_550MA                 BIT(7)

#define REG1C_SMB1357_FAST_CHARGER_CURRENT_MASK                 (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_SHIFT                0
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_300MA                0x00
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_400MA                BIT(0)
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_450MA                BIT(1)
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_475MA                (BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_500MA                BIT(2)
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_550MA                (BIT(2) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_600MA                (BIT(2) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_650MA                (BIT(2) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_700MA                BIT(3)
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_900MA                (BIT(3) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_950MA                (BIT(3) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1000MA               (BIT(3) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1100MA               (BIT(3) | BIT(2))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1200MA               (BIT(3) | BIT(2) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1400MA               (BIT(3) | BIT(2) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2700MA               (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1500MA               BIT(4)
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1600MA               (BIT(4) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1800MA               (BIT(4) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1850MA               (BIT(4) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1880MA               (BIT(4) | BIT(2))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1910MA	            (BIT(4) | BIT(2) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2800MA               (BIT(4) | BIT(2) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1950MA               (BIT(4) | BIT(2) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_1970MA               (BIT(4) | BIT(3))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2000MA               (BIT(4) | BIT(3) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2050MA               (BIT(4) | BIT(3) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2100MA               (BIT(4) | BIT(3) | BIT(1) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2300MA               (BIT(4) | BIT(3) | BIT(2))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2400MA               (BIT(4) | BIT(3) | BIT(2) | BIT(0))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_2500MA               (BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define REG1C_SMB1357_FAST_CHARGER_CURRENT_3000MA               (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))


/* Address:C1Eh */
#define REG1E_SMB1357_ADDRESS                                   0x1E

#define REG1E_SMB1357_FLOAT_VOLTAGE_MASK                        (BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG1E_SMB1357_FLOAT_VOLTAGE_3600MV                      (BIT(2) | BIT(0))
#define REG1E_SMB1357_FLOAT_VOLTAGE_4320MV                      (BIT(5) | BIT(3))
#define REG1E_SMB1357_FLOAT_VOLTAGE_4330MV                      (BIT(5) | BIT(3) | BIT(0))
#define REG1E_SMB1357_FLOAT_VOLTAGE_4340MV                      (BIT(5) | BIT(3) | BIT(1))
#define REG1E_SMB1357_FLOAT_VOLTAGE_4350MV	                    (BIT(5) | BIT(3) | BIT(1) | BIT(0))
#define SMB1357_MIN_FLOAT_MV                            		3600
#define SMB1357_MAX_FLOAT_MV                            		4500
#define SMB1357_VFLOAT_STEP_MV                          		20
#define SMB1357_VFLOAT_4350MV                           		4350


/* Address:32h */
#define REG32_SMB1357_ADDRESS                                   0x32
#define REG32_SMB1357_PRODUCT_ID_MASK                           (BIT(1) | BIT(0))
#define REG32_SMB1357_PRODUCT_ID_SMB1359                        0x00
#define REG32_SMB1357_PRODUCT_ID_SMB1358                        BIT(0)
#define REG32_SMB1357_PRODUCT_ID_SMB1357                        BIT(1)

/* Address:34h */
#define REG34_SMB1357_ADDRESS                                   0x34
#define REG34_SMB1357_PRODUCT_ID_A_MASK                         BIT(7)
#define REG34_SMB1357_PRODUCT_ID_A_SMB1357                      0x00
#define REG34_SMB1357_PRODUCT_ID_A_SMB1356                      BIT(7)


/* Address:40h */
#define REG40_SMB1357_ADDRESS									0x40
#define REG40_SMB1357_BQ_CONFIG_MASK							BIT(6)
#define REG40_SMB1357_BQ_CONFIG_DISABLE							0x00
#define REG40_SMB1357_BQ_CONFIG_ENABLE							BIT(6)


/* Address:41h */
#define REG41_SMB1357_ADDRESS									0x41

#define REG41_SMB1357_TURN_OFF_BATTERY_FET_MASK					BIT(7)
#define REG41_SMB1357_TURN_OFF_BATTERY_FET_ENABLE				BIT(7)

#define REG41_SMB1357_USBIN_COMMAND_SHUTDOWN_MASK				BIT(6)
#define REG41_SMB1357_USBIN_COMMAND_SHUTDOWN_ENABLE				BIT(6)

#define REG41_SMB1357_DCIN_COMMAND_SHUTDOWN_MASK				BIT(5)
#define REG41_SMB1357_DCIN_COMMAND_SHUTDOWN_ENABLE				BIT(5)

#define REG41_SMB1357_INPUT_CURRENT_MODE_MASK					BIT(2)
#define REG41_SMB1357_INPUT_CURRENT_MODE_FOR_APSD				0x00
#define REG41_SMB1357_INPUT_CURRENT_MODE_FOR_COMMAND			BIT(2)

#define REG41_SMB1357_USB_CONTROL_MODE_MASK 					(BIT(1) | BIT(0))
#define REG41_SMB1357_USB_100_MODE 								0x00
#define REG41_SMB1357_USB_500_MODE 								BIT(1)
#define REG41_SMB1357_USB_AC_MODE 								BIT(0)


/* Address:42h */
#define REG42_SMB1357_ADDRESS									0x42

#define REG42_SMB1357_CHARGER_OK_COMMAND_MASK            		BIT(5)
#define REG42_SMB1357_CHARGER_OK_COMMAND_ENABLE					BIT(5)

#define REG42_SMB1357_THERM_A_THERM_MONITOR_MASK           		BIT(4)
#define REG42_SMB1357_THERM_A_THERM_MONITOR_ENABLE            	0x00
#define REG42_SMB1357_THERM_A_THERM_MONITOR_DISABLE            	BIT(4)

#define REG42_SMB1357_TURN_OFF_STAT_PIN_MASK            		BIT(3)
#define REG42_SMB1357_TURN_OFF_STAT_PIN_DISABLE					0x00
#define REG42_SMB1357_TURN_OFF_STAT_PIN_ENABLE					BIT(3)

#define REG42_SMB1357_PRE_TO_FAST_CHARGING_MASK            		BIT(2)
#define REG42_SMB1357_PRE_TO_FAST_CHARGING_DISABLE				0x00
#define REG42_SMB1357_PRE_TO_FAST_CHARGING_ENABLE				BIT(2)

#define REG42_SMB1357_CHARGING_MASK								BIT(1)
#define REG42_SMB1357_CHARGING_DISABLE                    		0x00
#define REG42_SMB1357_CHARGING_ENABLE                    		BIT(1)

#define REG42_SMB1357_OTG_MASK                   				BIT(0)
#define REG42_SMB1357_OTG_DISABLE			                	0x00
#define REG42_SMB1357_OTG_ENABLE			                	BIT(0)


/* Address:47h */
#define REG47_SMB1357_ADDRESS									0x47

#define REG47_SMB1357_CMD_A_USBIN_SUSPEND_MASK                  BIT(3)
#define REG47_SMB1357_CMD_A_USBIN_SUSPEND_ENABLE                BIT(3)

#define REG47_SMB1357_CMD_A_DCIN_SUSPEND_MASK                  	BIT(2)
#define REG47_SMB1357_CMD_A_DCIN_SUSPEND_ENABLE                 BIT(2)


/* Address:4Ah */
#define REG4A_SMB1357_ADDRESS									0x4A

#define REG4A_SMB1357_STATUS_C_CHG_HOLD_OFF_BIT               	BIT(3)

#define REG4A_SMB1357_STATUS_C_CHARGING_MASK					(BIT(2) | BIT(1))
#define REG4A_SMB1357_STATUS_C_NO_CHARGING						0x00
#define REG4A_SMB1357_STATUS_C_PRE_CHARGING						BIT(1)
#define REG4A_SMB1357_STATUS_C_FAST_CHARGING					BIT(2)
#define REG4A_SMB1357_STATUS_C_TAPER_CHARGING					(BIT(2) | BIT(1))

#define REG4A_SMB1357_STATUS_C_CHG_MASK							BIT(0)
#define REG4A_SMB1357_STATUS_C_CHG_DISABLE_STATUS_BIT			0x00
#define REG4A_SMB1357_STATUS_C_CHG_ENABLE_STATUS_BIT			BIT(0)


/* Address:4Dh */
#define REG4D_SMB1357_ADDRESS									0x4D

#define REG4D_SMB1357_STATUS_IDEV_HVDCP_SEL_A_BIT               (BIT(4) | BIT(3))


/* Address:4Dh */
#define REG4E_SMB1357_ADDRESS									0x4E

#define REG4E_SMB1357_STATUS_DCIN_HV_INPUT_BIT                  BIT(2)


/* Address:50h */
#define REG50_SMB1357_ADDRESS									0x50

#define REG50_SMB1357_IRQ_A_HOT_HARD_LIMIT_STATUS				BIT(6)

#define REG50_SMB1357_IRQ_A_COLD_HARD_LIMIT_STATUS				BIT(4)

#define REG50_SMB1357_IRQ_A_HOT_SOFT_LIMIT_STATUS				BIT(2)

#define REG50_SMB1357_IRQ_A_COLD_SOFT_LIMIT_STATUS				BIT(0)


/* Address:51h */
#define REG51_SMB1357_ADDRESS									0x51

#define REG51_SMB1357_IRQ_B_BATTERY_TERM_STATUS					BIT(6)

#define REG51_SMB1357_IRQ_B_BATTERY_MISSING_STATUS				BIT(4)

#define REG51_SMB1357_IRQ_B_BATTERY_LOW_VOL_STATUS				BIT(2)

#define REG51_SMB1357_IRQ_B_INTERNAL_TEMP_STATUS				BIT(0)


/* Address:52h */
#define REG52_SMB1357_ADDRESS									0x52

#define REG52_SMB1357_IRQ_C_PRE_TO_FAST_VOL_STATUS				BIT(6)

#define REG52_SMB1357_IRQ_C_RECHARGE_VOL_STATUS					BIT(4)

#define REG52_SMB1357_IRQ_C_TAPER_CHARGING_VOL_STATUS			BIT(2)

#define REG52_SMB1357_IRQ_C_TERM_CHARGING_VOL_STATUS			BIT(0)


/* Address:53h */
#define REG53_SMB1357_ADDRESS									0x53

#define REG53_SMB1357_IRQ_D_BATTERY_VOL_HIG_STATUS				BIT(6)

#define REG53_SMB1357_IRQ_D_AICL_DONE_STATUS					BIT(4)

#define REG53_SMB1357_IRQ_D_CHARGING_TIMEOUT_STATUS				BIT(2)

#define REG53_SMB1357_IRQ_D_PRE_CHARGING_TIMEOUT_STATUS			BIT(0)


/* Address:54h */
#define REG54_SMB1357_ADDRESS									0x54

#define REG54_SMB1357_IRQ_E_DC_CHARGER_VOL_OV_STATUS			BIT(6)

#define REG54_SMB1357_IRQ_E_DC_CHARGER_VOL_UV_STATUS			BIT(4)

#define REG54_SMB1357_IRQ_E_USB_CHARGER_VOL_OV_STATUS			BIT(2)

#define REG54_SMB1357_IRQ_E_USB_CHARGER_VOL_UV_STATUS			BIT(0)


/* Address:55h */
#define REG55_SMB1357_ADDRESS									0x55

#define REG55_SMB1357_IRQ_F_OTG_CURRENT_OV_STATUS				BIT(6)

#define REG55_SMB1357_IRQ_F_OTG_ERROR_STATUS					BIT(4)

#define REG55_SMB1357_IRQ_F_CHARGING_OK_STATUS					BIT(0)


/* Address:56h */
#define REG56_SMB1357_ADDRESS									0x56

#define REG56_SMB1357_IRQ_G_SOURCE_DETECT_STATUS				BIT(6)

#define REG56_SMB1357_IRQ_G_WATCHDOG_TIMEOUT_STATUS				BIT(4)

#define REG56_SMB1357_IRQ_G_CHARGING_ERROR_STATUS				BIT(2)

#define REG56_SMB1357_IRQ_G_CHARGE_INHIBIT_STATUS				BIT(0)


#define IRQ_A_REG_SMB1357   	                                REG50_SMB1357_ADDRESS	
#define IRQ_B_REG_SMB1357   	                                REG51_SMB1357_ADDRESS	
#define IRQ_C_REG_SMB1357   	                                REG52_SMB1357_ADDRESS	
#define IRQ_D_REG_SMB1357   	                                REG53_SMB1357_ADDRESS	
#define IRQ_E_REG_SMB1357   	                                REG54_SMB1357_ADDRESS	
#define IRQ_F_REG_SMB1357   	                                REG55_SMB1357_ADDRESS	
#define IRQ_G_REG_SMB1357   	                                REG56_SMB1357_ADDRESS	


// config register
#define SMB1357_LAST_CNFG_REG                           		0x1F
// command register
#define SMB1357_FIRST_CMD_REG                           		0x40
#define SMB1357_LAST_CMD_REG                            		0x42
// status register
#define SMB1357_FIRST_STATUS_REG                        		0x46
#define SMB1357_LAST_STATUS_REG                         		0x56


OPPO_SMB1357_EXT int smb1357_get_prop_fastcharger_type(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_get_prop_charge_type(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_get_prop_batt_status(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_get_charging_status(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_set_otg_regulator_enable(struct regulator_dev *rdev);
OPPO_SMB1357_EXT int smb1357_set_otg_regulator_disable(struct regulator_dev *rdev);
OPPO_SMB1357_EXT int smb1357_get_otg_regulator_is_enable(struct regulator_dev *rdev);
OPPO_SMB1357_EXT int smb1357_set_charging_disable(struct opchg_charger *chip, bool disable);
OPPO_SMB1357_EXT int smb1357_set_suspend_enable(struct opchg_charger *chip, bool enable);
OPPO_SMB1357_EXT int smb1357_set_enable_volatile_writes(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_set_prechg_current(struct opchg_charger *chip, int ipre_mA);
OPPO_SMB1357_EXT int smb1357_set_termchg_current(struct opchg_charger *chip, int iterm_current);
OPPO_SMB1357_EXT int smb1357_set_fastchg_current(struct opchg_charger *chip, int ifast_mA);
OPPO_SMB1357_EXT int smb1357_set_float_voltage(struct opchg_charger *chip, int vfloat_mv);
OPPO_SMB1357_EXT int smb1357_set_input_chg_current(struct opchg_charger *chip, int current_ma, bool aicl);
OPPO_SMB1357_EXT int smb1357_set_complete_charge_timeout(struct opchg_charger *chip, int val);
OPPO_SMB1357_EXT int smb1357_set_float_compensation_voltage(struct opchg_charger *chip, int comp_voltage);
OPPO_SMB1357_EXT int smb1357_set_fastcharger_dectect(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_hw_init(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_get_initial_state(struct opchg_charger *chip);
OPPO_SMB1357_EXT void smb1357_dump_regs(struct opchg_charger *chip);
OPPO_SMB1357_EXT int smb1357_chg_uv(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT int smb1357_chg_ov(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT int smb1357_fast_chg(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT int smb1357_recharge_chg(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT int smb1357_taper_chg(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT int smb1357_chg_term(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT int smb1357_safety_timeout(struct opchg_charger *chip, u8 status);
OPPO_SMB1357_EXT void smb1357_chg_irq_handler(int irq, struct opchg_charger *chip);

#endif /*_OPPO_SMB1357_H_*/
