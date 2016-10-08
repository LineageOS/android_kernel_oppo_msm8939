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

#ifndef _OPPO_INC_H_
#define _OPPO_INC_H_

#define OPCHG_DEBUG
#define OPPO_USE_FAST_CHARGER
#define OPPO_USE_TIMEOVER_BY_AP
#define OPPO_USE_TP_ANTI_INTERFERENCE_14005		// check 14005 TP  anti-interference
#define OPPO_USE_FAST_CHARGER_RESET_MCU		// check fast charger no data reset mcu

//#define OPPO_USE_ADC_TM_IRQ

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/qpnp-charger.h>
#include <linux/idr.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

// move form smb1360_charger_fg.c 20140720
#include <linux/bitops.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/random.h>
#include <linux/rtc.h>
#include <linux/err.h>

// move form qpnp-vm-bms.c 20140720
#include <linux/types.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/spmi.h>
#include <linux/wakelock.h>
#include <linux/qpnp/power-on.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/qpnp-revid.h>
#include <uapi/linux/vm_bms.h>

// move form qpnp-bms.c 20140720
#include <linux/types.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/sched.h>
#include <linux/of_batterydata.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include "oppo_def.h"
#include "oppo_adc.h"
#include "oppo_battery.h"
#include "oppo_bms.h"
#include "oppo_charger.h"
#include "oppo_init.h"
#include "oppo_upper.h"
#include "oppo_smb358.h"
#include "oppo_smb1357.h"
#include "oppo_bq24196.h"
#include "oppo_bq24157.h"
#include "oppo_bq24188.h"
#include "oppo_bq27541.h"
#include "oppo_bq2022a.h"
#include "oppo_vooc.h"
#include "oppo_vooc_stm8s.h"

#include <soc/oppo/boot_mode.h>
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/device_info.h>

#undef pr_debug
#define pr_debug(fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
//#define pr_err(fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)

#undef dev_dbg
#define dev_dbg(dev, format, arg...) dev_printk(KERN_ERR, dev, format, ##arg)

#ifndef OPCHARGER_DEBUG_ENABLE
//#define OPCHARGER_DEBUG_ENABLE
#endif

#ifndef OPCHARGER_DEBUG_FOR_FAST_CHARGER
//#define OPCHARGER_DEBUG_FOR_FAST_CHARGER
#endif

#ifndef OPCHARGER_DEBUG_FOR_SOC
//#define OPCHARGER_DEBUG_FOR_SOC
#endif

#ifndef OPCHG_VOOC_WATCHDOG
#define OPCHG_VOOC_WATCHDOG
#endif

#endif /*_OPPO_INC_H_*/
