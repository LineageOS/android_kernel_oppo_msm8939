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

#ifndef _OPPO_BQ27541_H_
#define _OPPO_BQ27541_H_

#ifdef OPPO_BQ27541_PAR
#define OPPO_BQ27541_EXT
#else
#define OPPO_BQ27541_EXT extern
#endif


#define DRIVER_VERSION			"1.1.0"
/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_INTTEMP		0x28
#define BQ27541_REG_CC			0x2a
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e//#define BQ27541_REG_SOH		0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34//#define BQ27541_REG_PCHG		0x34
#define BQ27541_REG_DOD0		0x36

#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS		    BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CNTL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVICE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041

#ifdef CONFIG_GAUGE_BQ27411
/* Bq27411 standard data commands */
#define BQ27411_REG_CNTL				0x00
#define BQ27411_REG_TEMP				0x02
#define BQ27411_REG_VOLT				0x04
#define BQ27411_REG_FLAGS				0x06
#define BQ27411_REG_NAC					0x08
#define BQ27411_REG_FAC					0x0a
#define BQ27411_REG_RM					0x0c
#define BQ27411_REG_FCC					0x2c//0x0e
#define BQ27411_REG_AI					0x10
#define BQ27411_REG_SI					0x12
#define BQ27411_REG_MLI					0x14
#define BQ27411_REG_AP					0x18
#define BQ27411_REG_SOC					0x1c
#define BQ27411_REG_INTTEMP				0x1e
#define BQ27411_REG_SOH					0x20
#define BQ27411_FLAG_DSC				BIT(0)
#define BQ27411_FLAG_FC					BIT(9)
#define BQ27411_CS_DLOGEN				BIT(15)
#define BQ27411_CS_SS		    		BIT(13)
/* Bq27411 sub commands */
#define BQ27411_SUBCMD_CNTL_STATUS  			0x0000
#define BQ27411_SUBCMD_DEVICE_TYPE  			0x0001
#define BQ27411_SUBCMD_FW_VER  					0x0002
#define BQ27411_SUBCMD_DM_CODE  				0x0004
#define BQ27411_SUBCMD_PREV_MACW   				0x0007
#define BQ27411_SUBCMD_CHEM_ID   				0x0008
#define BQ27411_SUBCMD_SET_HIB   				0x0011
#define BQ27411_SUBCMD_CLR_HIB   				0x0012
#define BQ27411_SUBCMD_SET_CFG	   				0x0013
#define BQ27411_SUBCMD_SEALED   				0x0020
#define BQ27411_SUBCMD_RESET   					0x0041
#define BQ27411_SUBCMD_SOFTRESET				0x0042
#define BQ27411_SUBCMD_EXIT_CFG					0x0043
#define BQ27411_SUBCMD_ENABLE_DLOG  			0x0018
#define BQ27411_SUBCMD_DISABLE_DLOG 			0x0019
#define BQ27411_SUBCMD_ENABLE_IT    			0x0021
#define BQ27411_SUBCMD_DISABLE_IT   			0x0023
#define BQ27541_BQ27411_CMD_INVALID				0xFF
#endif

#define BQ27541_BQ27411_REG_CNTL				0x00
#define BQ27541_BQ27411_CS_DLOGEN				BIT(15)
#define BQ27541_BQ27411_CS_SS		    		BIT(13)
#define BQ27541_BQ27411_SUBCMD_CNTL_STATUS		0x0000
#define BQ27541_BQ27411_SUBCMD_ENABLE_IT		0x0021
#define BQ27541_BQ27411_SUBCMD_ENABLE_DLOG		0x0018
#define BQ27541_BQ27411_SUBCMD_DEVICE_TYPE		0x0001
#define BQ27541_BQ27411_SUBCMD_FW_VER			0x0002
#define BQ27541_BQ27411_SUBCMD_DISABLE_DLOG		0x0019
#define BQ27541_BQ27411_SUBCMD_DISABLE_IT		0x0023

#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)

#define CAPACITY_SALTATE_COUNTER 4
#define CAPACITY_SALTATE__FAST_COUNTER_20S 		4				// 5s*4
#define CAPACITY_SALTATE__AC_COUNTER_40S 		8				// 5s*10
#define CAPACITY_SALTATE__USB_COUNTER_1MIN 		12				// 5s*12
#define CAPACITY_SALTATE_COUNTER_CHARGING_TERM_30S	6			// 5s*6
#define CAPACITY_SALTATE_COUNTER_CHARGING_TERM_60S	12			// 5s*6

#define CAPACITY_SALTATE_COUNTER_NOT_CHARGING		12//20			// 5s*20
#define CAPACITY_SALTATE_COUNTER_10S				2 // 10
#define CAPACITY_SALTATE_COUNTER_20S				4 // 10
#define CAPACITY_SALTATE_COUNTER_40S				8 //40			// 5s*16
#define CAPACITY_SALTATE_COUNTER_60_60S			12//40			// 5s*20
#define CAPACITY_SALTATE_COUNTER_95_150S			30//150			// 5s*30
#define CAPACITY_SALTATE_COUNTER_FULL_250S		50//250			// 5s*50

#define LOW_POWER_VOLTAGE_3600MV				3600000
#define LOW_POWER_VOLTAGE_3500MV				3500000
#define LOW_POWER_VOLTAGE_3400MV				3400000
#define LOW_POWER_VOLTAGE_3300MV				3250000
#define LOW_POWER_VOLTAGE_2500MV				2500000
#define SOC_SHUTDOWN_VALID_LIMITS				20
#define TEN_MINUTES							600
#define CAPACITY_SALTATE_COUNTER_LOW_VOLTAGE_30S	6			// 5s*6
#define CAPACITY_SALTATE_COUNTER_LOW_VOLTAGE_15S	3			// 5s*3


#ifdef OPPO_BQ27541_PAR
struct opchg_bms_charger *bq27541_di = NULL;
struct opchg_gpio_control opchg_gpio;
int vooc_start_step=OPCHG_VOOC_TO_STANDARD;
#else
extern struct opchg_bms_charger *bq27541_di;
extern struct opchg_gpio_control opchg_gpio;
extern int vooc_start_step;
#endif

#define wait_us(n) udelay(n)
#define wait_ms(n) mdelay(n)
OPPO_BQ27541_EXT int msmrtc_alarm_read_time(struct rtc_time *tm);

OPPO_BQ27541_EXT int opchg_set_gpio_val(int gpio , u8 val);
OPPO_BQ27541_EXT int opchg_get_gpio_val(int gpio);
OPPO_BQ27541_EXT int opchg_set_gpio_dir_output(int gpio , u8 val);
OPPO_BQ27541_EXT int opchg_set_gpio_dir_intput(int gpio);

#ifdef OPPO_USE_FAST_CHARGER
OPPO_BQ27541_EXT int opchg_bq27541_gpio_pinctrl_init(struct opchg_bms_charger *di);
OPPO_BQ27541_EXT int opchg_bq27541_parse_dt(struct opchg_bms_charger *di);

OPPO_BQ27541_EXT int opchg_set_clock_active(struct opchg_bms_charger *di);
OPPO_BQ27541_EXT int opchg_set_clock_sleep(struct opchg_bms_charger *di);
OPPO_BQ27541_EXT int opchg_set_data_active(struct opchg_bms_charger *di);
OPPO_BQ27541_EXT int opchg_set_data_sleep(struct opchg_bms_charger *di);

OPPO_BQ27541_EXT int opchg_set_switch_fast_charger(void);
OPPO_BQ27541_EXT int opchg_set_switch_normal_charger(void);
OPPO_BQ27541_EXT int opchg_set_switch_earphone(void);

OPPO_BQ27541_EXT int opchg_set_reset_active(struct opchg_bms_charger  *di);
OPPO_BQ27541_EXT int opchg_set_switch_mode(u8 mode);
OPPO_BQ27541_EXT void opchg_set_pmic_soc_memory(int soc);
OPPO_BQ27541_EXT int opchg_get_pmic_soc_memory(void);
#endif

#endif /*_OPPO_BQ27541_H_*/
