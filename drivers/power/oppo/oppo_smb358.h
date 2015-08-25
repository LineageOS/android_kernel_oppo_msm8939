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

#ifndef _OPPO_SMB358_H_
#define _OPPO_SMB358_H_

#ifdef OPPO_SMB358_PAR
#define OPPO_SMB358_EXT
#else
#define OPPO_SMB358_EXT extern
#endif

enum {
    TIME_382MIN = 0,
    TIME_764MIN,
    TIME_1527MIN,
    TIME_DISABLED,
};

enum {
    SMB358_FAST_TIME_382MIN = 0,
    SMB358_FAST_TIME_764MIN,
    SMB358_FAST_TIME_1527MIN,
    SMB358_FAST_TIME_DISABLED,
};

#ifdef OPPO_SMB358_PAR
static int input_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};
#endif

#define MIN_FLOAT_MV                            3500
#define MAX_FLOAT_MV                            4500
#define VFLOAT_STEP_MV                          20
#define VFLOAT_4350MV                           4350

#define IRQ_LATCHED_MASK                        0x02
#define IRQ_STATUS_MASK                         0x01
#define BITS_PER_IRQ                            2

#define SMB358_IRQ_REG_COUNT                    6
#define SMB358_FAST_CHG_MIN_MA                  200
#define SMB358_FAST_CHG_MAX_MA                  2000
#define SMB358_FAST_CHG_SHIFT                   5
#define SMB358_INPUT_CURRENT_LIMIT_SHIFT        4
#define SMB358_IMPUT_CURRENT_LIMIT_MAX_MA       2000

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG                    0x00
#define SMB_FAST_CHG_CURRENT_MASK               (BIT(7) | BIT(6) | BIT(5))//0xE0
#define CHG_PRE_MASK                            (BIT(4) | BIT(3))//0x18
#define REG00_SMB358_PRE_CHARGING_CURRENT_150MA        0x00
#define REG00_SMB358_PRE_CHARGING_CURRENT_250MA        BIT(3)
#define REG00_SMB358_PRE_CHARGING_CURRENT_350MA        BIT(4)
#define REG00_SMB358_PRE_CHARGING_CURRENT_450MA       (BIT(4) | BIT(3))//0x18
#define CHG_ITERM_MASK                          (BIT(2) | BIT(1) | BIT(0))//0x07

#define INPUT_CURRENT_LIMIT_REG                 0x01
#define AC_CHG_CURRENT_MASK                     (BIT(7) | BIT(6) | BIT(5) | BIT(4))//0xF0
#define VFLT_MASK                               (BIT(3) | BIT(2))//0x0C
#define VFLT_300MV                              (BIT(3) | BIT(2))//0x0C
#define VFLT_200MV                              BIT(3)//0x08
#define VFLT_100MV                              BIT(2)//00x04
#define VFLT_50MV                               0x00
#define CHG_INHI_EN_MASK                        BIT(1)
#define CHG_INHI_EN_BIT                         BIT(1)

#define VARIOUS_FUNC_REG                        0x02
#define VARIOUS_FUNC_USB_SUSP_MASK              BIT(6)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT        BIT(6)
#define AICL_EN_MASK							BIT(4)
#define AICL_EN_BIT								BIT(4)
#define AICL_DISEN_BIT							0x00

#define VFLOAT_REG                              0x03
#define REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_MASK            (BIT(7) | BIT(6))
#define REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_2300MV           0x00
#define REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_2500MV           BIT(6)
#define REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_2800MV           BIT(7)
#define REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_3000MV          (BIT(7) | BIT(6))
#define VFLOAT_MASK                             (BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))//0x3F

#define CHG_CTRL_REG                            0x04
#define CHG_CTRL_AUTO_RECHARGE_MASK             BIT(7)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT       0x0
#define CHG_AUTO_RECHARGE_DIS_BIT				BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK             BIT(6)
#define CHG_CTRL_CURR_TERM_END_CHG_BIT          0x0
#define CHG_CTRL_BATT_MISSING_DET_MASK          (BIT(5) | BIT(4))
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO      (BIT(5) | BIT(4))
#define CHG_CTRL_APSD_EN_MASK                   BIT(2)
#define CHG_CTRL_APSD_EN_BIT                    BIT(2)

#define STAT_AND_TIMER_CTRL_REG                 0x05
#define TIMER_CTRL_REG_MASK                     (BIT(3) | BIT(2))
#define TIMER_CTRL_REG_SHIFT                    2

#define CHG_PIN_EN_CTRL_REG                     0x06
#define CHG_PIN_CTRL_CHG_EN_MASK                (BIT(6) | BIT(5))//(BIT(5) | BIT(6))
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT         (BIT(6) | BIT(5))//(BIT(5) | BIT(6))
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT         0x0
#define CHG_PIN_CTRL_USBCS_REG_MASK             BIT(4)
#define CHG_PIN_CTRL_USBCS_REG_BIT              0x0
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK           BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT            BIT(2)
#define CHG_PIN_CTRL_APSD_IRQ_MASK              BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_BIT               BIT(1)

#define THERM_A_CTRL_REG                        0x07
#define THERM_A_SWITCHING_FREQ_MASK             BIT(7)
#define THERM_A_SWITCHING_FREQ_1_5MHZ           BIT(7)
#define THERM_A_THERM_MONITOR_EN_MASK           BIT(4)
#define THERM_A_THERM_MONITOR_EN_BIT            0x0

#define SYSOK_AND_USB3_REG                      0x08
#define USB3_ENABLE_MASK                        BIT(5)
#define USB3_ENABLE_BIT                         BIT(5)

#define OTHER_CTRL_REG                  		0x09
#define CHG_LOW_BATT_THRESHOLD_MASK             (BIT(3) | BIT(2) | BIT(1) | BIT(0))//0x0F
#define SMB358_BATT_GOOD_THRE_2P5 				0x1

#define OTG_TLIM_THERM_REG                      0x0A
#define OTG_CURRENT_LIMIT_MASK                  (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_BIT                   (BIT(2) | BIT(3))

#define FAULT_INT_REG                           0x0C
#define FAULT_INT_HOT_COLD_HARD_BIT             BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT             BIT(6)
#define FAULT_INT_INPUT_OV_BIT                  BIT(3)
#define FAULT_INT_INPUT_UV_BIT                  BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT             BIT(1)

#define STATUS_INT_REG                          0x0D
#define STATUS_INT_CHG_TIMEOUT_BIT              BIT(7)
#define STATUS_INT_OTG_DETECT_BIT               BIT(6)
#define STATUS_INT_BATT_OV_BIT                  BIT(5)
#define STATUS_INT_CHGING_BIT                   BIT(4)
#define STATUS_INT_CHG_INHI_BIT                 BIT(3)
#define STATUS_INT_INOK_BIT                     BIT(2)
#define STATUS_INT_MISSING_BATT_BIT             BIT(1)
#define STATUS_INT_LOW_BATT_BIT                 BIT(0)


/* Command registers */
#define CMD_A_REG                               0x30
#define CMD_A_VOLATILE_W_PERM_BIT               BIT(7)
#define CMD_A_FAST_CHARGING_SET_MASK            BIT(6)
#define CMD_A_FAST_CHARGING_SET_BIT             BIT(6)
#define CMD_A_OTG_ENABLE_MASK                   BIT(4)
#define CMD_A_OTG_ENABLE_BIT                    BIT(4)
#define CMD_A_CHG_SUSP_EN_MASK                  BIT(2)
#define CMD_A_CHG_SUSP_EN_BIT                   BIT(2)
#define CMD_A_CHG_ENABLE_MASK                   BIT(1)
#define CMD_A_CHG_ENABLE_BIT                    BIT(1)
#define CMD_A_STAT_DISABLE_MASK                 BIT(0)
#define CMD_A_STAT_DISABLE_BIT                  BIT(0)

#define CMD_B_REG                               0x31
#define CMD_B_CHG_USB_500_900_ENABLE_BIT        BIT(1)
#define CMD_B_CHG_HC_ENABLE_BIT                 BIT(0)

#define CHG_REVISION_REG                        0x34

/* IRQ status registers */
#define IRQ_A_REG                               0x35
#define IRQ_A_HOT_HARD_BIT                      BIT(6)
#define IRQ_A_COLD_HARD_BIT                     BIT(4)
#define IRQ_A_HOT_SOFT_BIT                      BIT(2)
#define IRQ_A_COLD_SOFT_BIT                     BIT(0)

#define IRQ_B_REG                               0x36
#define IRQ_B_BATT_OV_BIT                       BIT(6)
#define IRQ_B_BATT_MISSING_BIT                  BIT(4)
#define IRQ_B_BATT_LOW_BIT                      BIT(2)
#define IRQ_B_PRE_FAST_CHG_BIT                  BIT(0)

#define IRQ_C_REG                               0x37
#define IRQ_C_INT_OVER_TEMP_BIT                 BIT(6)
#define IRQ_C_TAPER_CHG_BIT                     BIT(2)
#define IRQ_C_TERM_BIT                          BIT(0)

#define IRQ_D_REG                               0x38
#define IRQ_D_APSD_COMPLETE                     BIT(6)
#define IRQ_D_AICL_DONE_BIT                     BIT(4)
#define IRQ_D_CHG_TIMEOUT_BIT                   (BIT(2) | BIT(0))//(BIT(0) | BIT(2))

#define IRQ_E_REG                               0x39
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_E_INPUT_OV_BIT                      BIT(2)
#define IRQ_E_INPUT_UV_BIT                      BIT(0)

#define IRQ_F_REG                               0x3A
#define IRQ_F_OTG_OC_BIT                        BIT(6)
#define IRQ_F_OTG_BATT_FAIL_BIT                 BIT(4)
#define IRQ_F_OTG_VALID_BIT                     BIT(2)
#define IRQ_F_POWER_OK                          BIT(0)

/* Status registers */
#define STATUS_C_REG                            0x3D
#define STATUS_C_CHG_ERR_STATUS_BIT             BIT(6)
#define STATUS_C_CHG_HOLD_OFF_BIT               BIT(3)
#define STATUS_C_CHARGING_MASK                  (BIT(2) | BIT(1))//(BIT(1) | BIT(2))
#define STATUS_C_PRE_CHARGING                   BIT(1)
#define STATUS_C_FAST_CHARGING                  BIT(2)
#define STATUS_C_TAPER_CHARGING                 (BIT(2) | BIT(1))
#define STATUS_C_CHG_ENABLE_STATUS_BIT          BIT(0)

#define STATUS_D_REG                            0x3E
#define STATUS_D_PORT_ACA_DOCK                  BIT(7)
#define STATUS_D_PORT_ACA_C                     BIT(6)
#define STATUS_D_PORT_ACA_B                     BIT(5)
#define STATUS_D_PORT_ACA_A                     BIT(4)
#define STATUS_D_PORT_CDP                       BIT(3)
#define STATUS_D_PORT_DCP                       BIT(2)
#define STATUS_D_PORT_SDP                       BIT(1)
#define STATUS_D_PORT_OTHER                     BIT(0)

#define STATUS_E_REG                            0x3F


#define CHG_PRE_150MA                           0x00
#define CHG_PRE_250MA                           0x08
#define CHG_PRE_350MA                           0x10
#define CHG_PRE_450MA                           0x18

#define CHG_ITERM_30MA                          0x00
#define CHG_ITERM_40MA                          0x01
#define CHG_ITERM_60MA                          0x02
#define CHG_ITERM_80MA                          0x03
#define CHG_ITERM_100MA                         0x04
#define CHG_ITERM_125MA                         0x05
#define CHG_ITERM_150MA                         0x06
#define CHG_ITERM_200MA                         0x07


#define LAST_CNFG_REG                           0x0D

#define FIRST_CMD_REG                           0x30
#define LAST_CMD_REG                            0x33

#define FIRST_STATUS_REG                        0x35
#define LAST_STATUS_REG                         0x3F

// config register
#define SMB358_LAST_CNFG_REG                    0x0D
// command register
#define SMB358_FIRST_CMD_REG                   	0x30
#define SMB358_LAST_CMD_REG                     0x33
// status register
#define SMB358_FIRST_STATUS_REG                 0x35
#define SMB358_LAST_STATUS_REG                  0x3F
#define STATUS_FAST_CHARGING					BIT(6)



OPPO_SMB358_EXT int smb358_get_prop_charge_type(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_get_prop_batt_status(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_get_charging_status(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_set_otg_regulator_enable(struct regulator_dev *rdev);
OPPO_SMB358_EXT int smb358_set_otg_regulator_disable(struct regulator_dev *rdev);
OPPO_SMB358_EXT int smb358_get_otg_regulator_is_enable(struct regulator_dev *rdev);
#if 1
OPPO_SMB358_EXT int smb358_set_otg_enable(void);
OPPO_SMB358_EXT int smb358_set_otg_disable(void);
OPPO_SMB358_EXT int smb358_get_otg_enable(void);
#endif
OPPO_SMB358_EXT int smb358_set_charging_disable(struct opchg_charger *chip, bool disable);
OPPO_SMB358_EXT int smb358_set_suspend_enable(struct opchg_charger *chip, bool enable);
OPPO_SMB358_EXT int smb358_set_enable_volatile_writes(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_set_fastchg_current(struct opchg_charger *chip, int ifast_mA);
OPPO_SMB358_EXT int smb358_set_float_voltage(struct opchg_charger *chip, int vfloat_mv);
OPPO_SMB358_EXT int smb358_set_precharger_voltage(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_set_recharger_voltage(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_set_prechg_current(struct opchg_charger *chip, int ipre_mA);
OPPO_SMB358_EXT int smb358_set_input_chg_current(struct opchg_charger *chip, int current_ma, bool aicl);
OPPO_SMB358_EXT int smb358_set_complete_charge_timeout(struct opchg_charger *chip, int val);
OPPO_SMB358_EXT int smb358_hw_init(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_get_initial_state(struct opchg_charger *chip);
OPPO_SMB358_EXT void smb358_dump_regs(struct opchg_charger *chip);
OPPO_SMB358_EXT int smb358_chg_uv(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT int smb358_chg_ov(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT int smb358_fast_chg(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT int smb358_recharge_chg(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT int smb358_taper_chg(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT int smb358_chg_term(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT int smb358_safety_timeout(struct opchg_charger *chip, u8 status);
OPPO_SMB358_EXT void smb358_chg_irq_handler(int irq, struct opchg_charger *chip);
OPPO_SMB358_EXT	int smb358_get_prop_batt_present(struct opchg_charger *chip);
OPPO_SMB358_EXT	int smb358_charging_disable(struct opchg_charger *chip,int reason, int disable);
OPPO_SMB358_EXT	int smb358_get_prop_batt_capacity(struct opchg_charger * chip);

#endif /*_OPPO_SMB358_H_*/
