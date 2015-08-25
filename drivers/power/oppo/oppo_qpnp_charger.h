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

#ifndef _OPPO_QPNP_CHARGER_H_
#define _OPPO_QPNP_CHARGER_H_

#ifdef OPPO_QPNP_CHARGER_PAR
#define OPPO_QPNP_CHARGER_EXT
#else
#define OPPO_QPNP_CHARGER_EXT extern
#endif

struct oppo_qpnp_chg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				misc_base;
	spinlock_t			hw_access_lock;
	struct delayed_work	usbin_irq_work;//sjc add for no-24196-INT
	int				usbin_irq;//sjc add for no-24196-INT
	struct wakeup_source	usbin_ws;//sjc add for no-24196-INT
};

/*pmic soc memory reg address*/
#define PMIC_SOC_STORAGE_REG			0xB0
#define PMIC_IAVG_STORAGE_REG			0xB1
#define BMS_VM_BMS_DATA_REG_0			0x40B0
#define BMS_VM_BMS_DATA_REG_1			0x40B1
#define BMS_VM_BMS_DATA_REG_2			0x40B2



OPPO_QPNP_CHARGER_EXT int oppo_qpnp_chg_read(struct oppo_qpnp_chg_chip *chip, u16 base,u8 *val, int count);
OPPO_QPNP_CHARGER_EXT int oppo_qpnp_chg_write(struct oppo_qpnp_chg_chip *chip, u16 base,u8 *val, int count);
OPPO_QPNP_CHARGER_EXT void opchg_set_pmic_soc_memory(int soc);
OPPO_QPNP_CHARGER_EXT int opchg_get_pmic_soc_memory(void);

#endif /*_OPPO_CHARGER_H_*/
