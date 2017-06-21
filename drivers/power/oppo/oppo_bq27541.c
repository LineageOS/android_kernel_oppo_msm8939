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

#define OPPO_BQ27541_PAR
#include "oppo_inc.h"

#ifdef CONFIG_MACH_OPPO
extern char *BQ27541_HMACSHA1_authenticate(char *Message,char *Key,char *result);
#endif //CONFIG_MACH_OPPO

/* OPPO 2013-12-20 liaofuchun add for fastchg firmware update */
#ifdef OPPO_USE_FAST_CHARGER
/*
extern unsigned char Pic16F_firmware_data_14005[];
extern int pic_fw_ver_count_14005;

extern unsigned char Pic16F_firmware_data_15011[];
extern int pic_fw_ver_count_15011;

extern unsigned char Pic16F_firmware_data_15018[];
extern int pic_fw_ver_count_15018;

extern unsigned char Pic16F_firmware_data_15022[];
extern int pic_fw_ver_count_15022;

extern unsigned char stm8s_firmware_data_15022[];
extern int st_fw_ver_count_15022;

extern unsigned char Pic16F_firmware_data[];
extern int pic_fw_ver_count;
*/
int vooc_need_to_up_fw = 0;
int vooc_have_updated = 0;
int vooc_fw_ver_count = 0;
unsigned char *vooc_firmware_data = NULL;
struct opchg_fast_charger *opchg_fast_charger_chip = NULL;
int (*vooc_fw_update) (struct opchg_fast_charger *chip, bool enable) = NULL;
#endif
/* OPPO 2013-12-20 liaofuchun add end */


static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

//static struct opchg_bms_charger *bq27541_di;
static int coulomb_counter;
static spinlock_t lock; /* protect access to coulomb_counter */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct opchg_bms_charger *di);

#ifdef OPPO_USE_FAST_CHARGER
int opchg_bq27541_gpio_pinctrl_init(struct opchg_bms_charger *di);
int opchg_bq27541_parse_dt(struct opchg_bms_charger *di);

int opchg_set_gpio_val(int gpio , u8 val);
int opchg_get_gpio_val(int gpio);
int opchg_set_gpio_dir_output(int gpio , u8 val);
int opchg_set_gpio_dir_intput(int gpio);

int opchg_set_clock_active(struct opchg_bms_charger *di);
int opchg_set_clock_sleep(struct opchg_bms_charger *di);
int opchg_set_data_active(struct opchg_bms_charger *di);
int opchg_set_data_sleep(struct opchg_bms_charger *di);

int opchg_set_switch_fast_charger(void);
int opchg_set_switch_normal_charger(void);
int opchg_set_switch_earphone(void);

int opchg_set_reset_active(struct opchg_bms_charger  *di);
#endif
static int bq27541_battery_temperature(struct opchg_bms_charger *di);
static int bq27541_remaining_capacity(struct opchg_bms_charger *di);
static int bq27541_battery_voltage(struct opchg_bms_charger *di);

static int bq27541_read(u8 reg, int *rt_value, int b_single,
			struct opchg_bms_charger *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct opchg_bms_charger *di)
{
	int ret;
	int temp = 0;
	static int count = 0;

	if(atomic_read(&di->suspended) == 1) {
		return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}

	if(di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_temp, &temp, 0, di);
		if (ret) {
			count++;
			dev_err(di->dev, "error reading temperature\n");
			if(count > 1) {
				count = 0;
				/* jingchun.wang@Onlinerd.Driver, 2014/01/22  Add for it report bad status when plug out battery */
				di->temp_pre = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
				return -400;
			} else {
				return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
			}
		}
		count = 0;
	} else {
		return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}

	di->temp_pre = temp;

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

/* OPPO 2013-08-24 wangjc Add begin for add adc interface. */
static int bq27541_battery_cc(struct opchg_bms_charger *di)//sjc20150105
{
	int ret;
	int cc = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->cc_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_cc, &cc, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading cc.\n");
			return ret;
		}
	} else {
		return di->cc_pre;
	}

	di->cc_pre = cc;
	return cc;
}

static int bq27541_battery_fcc(struct opchg_bms_charger *di)//sjc20150105
{
	int ret;
	int fcc = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->fcc_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_fcc, &fcc, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->fcc_pre;
	}

	di->fcc_pre = fcc;
	return fcc;
}

static int bq27541_battery_soh(struct opchg_bms_charger *di)//sjc20150105
{
	int ret;
	int soh = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->soh_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_soh, &soh, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->soh_pre;
	}

	di->soh_pre = soh;
	return soh;
}


static int bq27541_remaining_capacity(struct opchg_bms_charger *di)
{
	int ret;
	int cap = 0;

	if(di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_rm, &cap, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading capacity.\n");
			return ret;
		}
	}

	return cap;
}

#if 0
static int bq27541_battery_rm(struct opchg_bms_charger *di)
{
	int ret;
	int rm = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->rm_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_RM, &rm, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading capacity.\n");
			return ret;
		}
	} else {
		return di->rm_pre;
	}

	di->rm_pre = rm;
	return rm;

}

static int bq27541_battery_fac(struct opchg_bms_charger *di)//sjc20150105
{
	int ret;
	int fac = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->fac_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_FAC, &fac, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->fac_pre;
	}

	di->fac_pre = fac;
	return fac;
}

static int bq27541_battery_pchg(struct opchg_bms_charger *di)
{
	int ret;
	int pchg = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->pchg_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_LOGBUF, &pchg, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->pchg_pre;
	}

	di->pchg_pre = pchg;
	return pchg;

}

static int bq27541_battery_dod0(struct opchg_bms_charger *di)
{
	int ret;
	int dod0 = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->dod0_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_DOD0, &dod0, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->dod0_pre;
	}

	di->dod0_pre = dod0;
	return dod0;

}
static int bq27541_battery_flags(struct opchg_bms_charger *di)
{
	int ret;
	int flags = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->flags_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_FLAGS, &flags, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->flags_pre;
	}

	di->flags_pre = flags;
	return flags;

}
#endif
static int bq27541_soc_calibrate(struct opchg_bms_charger *di, int soc)
{
	union power_supply_propval ret = {0,};
	unsigned int soc_calib;
	int counter_temp = 0;
	int soc_load= 0;
	int soc_calib_temp= 0;
	static int counter_debug = 0;
	static int chg_present_pre = 0;//sjc20150529
	static int soc_temp = 0;

	//stap1: get init soc
	if(!di->batt_psy){
		di->batt_psy = power_supply_get_by_name("battery");

		// get pmic soc
		soc_load = opchg_get_pmic_soc_memory();
		if(soc_load == 0xFF)
		{
			di->soc_pre = soc;
		}
		else if((soc_load != 0) && ( abs(soc_load-soc) < SOC_SHUTDOWN_VALID_LIMITS ))
		{
			//compare the soc and the last soc
			if(soc_load > soc) {
				di->soc_pre = soc_load -1;
			} else {
				di->soc_pre = soc_load;
			}
		}
		else
		{
			di->soc_pre = soc;
		}
		pr_debug("oppo_debug soc_load=%d, soc=%d, di->soc_pre=%d\n",soc_load,soc,di->soc_pre);
		// set pmic soc
		opchg_set_pmic_soc_memory(di->soc_pre);
	}
	if(opchg_chip == NULL)
	{
		pr_debug("oppo_debug opchg_chip is not probe \n");
		if(di->soc_pre > 100)
		{
			di->soc_pre = 100;
		}
		return di->soc_pre;
	}

	//stap2: soc Smoothing
	soc_calib_temp= di->soc_pre;
	soc_calib= di->soc_pre;
	if(di->batt_psy){
		di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_STATUS, &ret);

		if (opchg_chip->chg_present ^ chg_present_pre) {//sjc20150529
			chg_present_pre = opchg_chip->chg_present;
			di->saltate_counter = 0;
		}
		// charing
		if(opchg_chip->chg_present)
		{
			// when charging full and soc is not 100
			//if((opchg_chip->batt_pre_full)&&((opchg_chip->temperature >= opchg_chip->pre_cool_bat_decidegc) && (opchg_chip->temperature < opchg_chip->warm_bat_decidegc)))
			if ((opchg_chip->batt_pre_full)&&
				(opchg_chip->charging_opchg_temp_statu >= OPCHG_CHG_TEMP_PRE_COOL1 && opchg_chip->charging_opchg_temp_statu <= OPCHG_CHG_TEMP_NORMAL))
			{
				di->saltate_counter++;
				if(di->saltate_counter >= CAPACITY_SALTATE_COUNTER_CHARGING_TERM_60S)
				{
					if(di->soc_pre < 100)
					{
						soc_calib = di->soc_pre + 1;
					}
					else
					{
						soc_calib = di->soc_pre;
					}
					di->saltate_counter = 0;
				}
				else{
					soc_calib = di->soc_pre;
				}
			}
			// charging and charging full
			else if((ret.intval == POWER_SUPPLY_STATUS_CHARGING)||(ret.intval == POWER_SUPPLY_STATUS_FULL))
			{
				if(abs(soc - di->soc_pre) > 0)
				{
					di->saltate_counter++;
					//	counter_temp = CAPACITY_SALTATE__FAST_COUNTER_20S;//1min
					if(di->saltate_counter < CAPACITY_SALTATE__FAST_COUNTER_20S)
					{
						if(di->soc_pre > 100)
						{
							di->soc_pre = 100;
						}
						return di->soc_pre;
					}
					else
						di->saltate_counter = 0;
				}
				else
				{
					di->saltate_counter = 0;
				}

				// SOC Smoothing
				// when di->saltate_counter == counter_temp
				if(soc > di->soc_pre) {
					soc_calib = di->soc_pre + 1;
				} else if(soc < (di->soc_pre - 1)) {
				/* jingchun.wang@Onlinerd.Driver, 2013/04/14  Add for allow soc fail when charging. */
					soc_calib = di->soc_pre - 1;
				} else {
					soc_calib = di->soc_pre;
				}
			}
			// discharging when temp > 55 or timeout
			else
			{
				if ((abs(soc - di->soc_pre) > 0) || (di->batt_vol_pre <= LOW_POWER_VOLTAGE_3300MV && di->batt_vol_pre > LOW_POWER_VOLTAGE_2500MV)) {//sjc1118 add for batt_vol is too low but soc is not jumping
					di->saltate_counter++;

					if(di->soc_pre == 100) {
						counter_temp = CAPACITY_SALTATE_COUNTER_FULL_250S;//6
					} else if (di->soc_pre > 95) {
						counter_temp = CAPACITY_SALTATE_COUNTER_95_150S;///3
					} else if (di->soc_pre > 60) {
						counter_temp = CAPACITY_SALTATE_COUNTER_60_60S;///2
					} else{
						counter_temp = CAPACITY_SALTATE_COUNTER_40S;///1.5
					}

					if(di->batt_vol_pre <= LOW_POWER_VOLTAGE_3300MV && di->batt_vol_pre > LOW_POWER_VOLTAGE_2500MV)
					{
						if ( (bq27541_battery_voltage(di) <= LOW_POWER_VOLTAGE_3300MV)
							&& (bq27541_battery_voltage(di) > LOW_POWER_VOLTAGE_2500MV))
							counter_temp = CAPACITY_SALTATE_COUNTER_10S;///1.5
					}

					if(di->saltate_counter < counter_temp)
					{
						if(di->soc_pre > 100)
						{
							di->soc_pre = 100;
						}
						return di->soc_pre;
					}
					else
						di->saltate_counter = 0;
				}
				else
				{
					di->saltate_counter = 0;
				}

				// SOC Smoothing
				// whendi->saltate_counter == counter_temp ;
				if(soc < di->soc_pre)
				{
					soc_calib = di->soc_pre - 1;
				}
				else if (di->batt_vol_pre <= LOW_POWER_VOLTAGE_3300MV && di->batt_vol_pre >LOW_POWER_VOLTAGE_2500MV && di->soc_pre > 0)//sjc1118 add for batt_vol is too low but soc is not jumping
				{
					soc_calib = di->soc_pre - 1;
				}
				else
				{
					soc_calib = di->soc_pre;
				}
			}
		}
		// discharing
		else
		{
			if ((abs(soc - di->soc_pre) > 0) || (di->batt_vol_pre <= LOW_POWER_VOLTAGE_3300MV && di->batt_vol_pre > LOW_POWER_VOLTAGE_2500MV)) {//sjc1118 add for batt_vol is too low but soc is not jumping
				di->saltate_counter++;

				if(di->soc_pre == 100) {
					counter_temp = CAPACITY_SALTATE_COUNTER_FULL_250S;//6
				} else if (di->soc_pre > 95) {
					counter_temp = CAPACITY_SALTATE_COUNTER_95_150S;///3
				} else if (di->soc_pre > 60) {
					counter_temp = CAPACITY_SALTATE_COUNTER_60_60S;///2
				} else{
					counter_temp = CAPACITY_SALTATE_COUNTER_40S;///1.5
				}

				if(di->batt_vol_pre <= LOW_POWER_VOLTAGE_3300MV && di->batt_vol_pre > LOW_POWER_VOLTAGE_2500MV)
				{
					if ( (bq27541_battery_voltage(di) <= LOW_POWER_VOLTAGE_3300MV)
						&& (bq27541_battery_voltage(di) > LOW_POWER_VOLTAGE_2500MV))
						counter_temp = CAPACITY_SALTATE_COUNTER_10S;///1.5
				}

				if(di->saltate_counter < counter_temp)
				{
					if(di->soc_pre > 100)
					{
						di->soc_pre = 100;
					}
					return di->soc_pre;
				}
				else
					di->saltate_counter = 0;
			}
			else
			{
				di->saltate_counter = 0;
			}

			// SOC Smoothing
			// whendi->saltate_counter == counter_temp ;
			if(soc < di->soc_pre)
			{
				soc_calib = di->soc_pre - 1;
			}
			else if (di->batt_vol_pre <= LOW_POWER_VOLTAGE_3300MV && di->batt_vol_pre >LOW_POWER_VOLTAGE_2500MV && di->soc_pre > 0)//sjc1118 add for batt_vol is too low but soc is not jumping
			{
				soc_calib = di->soc_pre - 1;
			}
			else
			{
				soc_calib = di->soc_pre;
			}
		}
	}
	else{
		soc_calib= di->soc_pre;
	}

	//stap3: soc fault-tolerant processing
	if(soc_calib > 100)
		soc_calib = 100;

	//stap4: soc display
	di->soc_pre = soc_calib;
	opchg_set_pmic_soc_memory(soc_calib);

	//stap5: debug_log
	counter_debug++;
	if((counter_debug >= 9)||(soc_calib_temp != soc_calib) || (soc_temp != soc))
	{
		pr_debug("soc:%d, soc_calib:%d,counter_temp:%d,ret.intval=%d,opchg_chip->batt_pre_full:%d,opchg_chip->batt_full:%d,di->saltate_counter:%d,di->batt_vol_pre:%d\n",
			soc, soc_calib,counter_temp,ret.intval,opchg_chip->batt_pre_full,opchg_chip->batt_full,di->saltate_counter,di->batt_vol_pre);
		counter_debug = 0;
		soc_temp = soc;
	}
	return soc_calib;
}

static int bq27541_battery_soc(struct opchg_bms_charger *di, int time)
{
	int ret;
	int soc = 0;
	int soc_delt = 0;

	// step1: suspended is not get soc
	if(atomic_read(&di->suspended) == 1) {
		return di->soc_pre;
	}

	// step2: get soc
	if(di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_soc, &soc, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading soc.ret:%d\n",ret);
			goto read_soc_err;
		}
	} else {
		if(di->soc_pre)
			return di->soc_pre;
		else
			return 0;
	}

	ret = bq27541_battery_cc(di);
	ret = bq27541_battery_fcc(di);
	ret = bq27541_battery_soh(di);

	// step3:  get right soc when sleep long time
	if(time != 0) {
		if(soc < di->soc_pre) {
			soc_delt  =  di->soc_pre - soc;
			//allow capacity decrease 1% every 10minutes when sleep
			time = time/TEN_MINUTES;
			if(time <  soc_delt) {
				di->soc_pre  -=  time;
				//when time =0,Prevent the system is frequently wake up,  power show is not change
				if(di->soc_pre - soc >= 5)
				{
					di->soc_pre  -= 1;
				}
			} else  {
			di->soc_pre = soc +1;
			}
		}
	}
	#ifdef OPCHARGER_DEBUG_FOR_SOC
	pr_err("oppo_check_soc_check_value soc_check=%d,di->cc_pre=%d,di->fcc_pre=%d,di->soh_pre=%d\n",soc,di->cc_pre,di->fcc_pre,di->soh_pre);
    #endif
	soc = bq27541_soc_calibrate(di,soc);
	return soc;

read_soc_err:
	if(di->soc_pre)
		return di->soc_pre;
	else
		return 0;
}

static int bq27541_average_current(struct opchg_bms_charger *di)
{
	int ret;
	int curr = 0;

	if(atomic_read(&di->suspended) == 1) {
		return -di->current_pre;
	}

	if(di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_ai, &curr, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading current.\n");
			return ret;
		}
	} else {
		return -di->current_pre;
	}
	// negative current
	if(curr&0x8000)
		curr = -((~(curr-1))&0xFFFF);
	di->current_pre = curr;
	return -curr;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct opchg_bms_charger *di)
{
	int ret;
	int volt = 0;

	if(atomic_read(&di->suspended) == 1) {
		return di->batt_vol_pre;
	}

	if(di->alow_reading == true) {
		ret = bq27541_read(di->cmd_addr.reg_volt, &volt, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading voltage,ret:%d\n",ret);
			return ret;
		}
	} else {
		return di->batt_vol_pre;
	}

	di->batt_vol_pre = volt * 1000;

	return volt * 1000;
}

static void bq27541_cntl_cmd(struct opchg_bms_charger *di,
				int subcmd)
{
	bq27541_i2c_txsubcmd(di->cmd_addr.reg_cntl, subcmd, di);
}

/*
 * i2c specific code
 */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct opchg_bms_charger *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

#ifdef CONFIG_GAUGE_BQ27411
	if(reg == BQ27541_BQ27411_CMD_INVALID)
		return 0;
#endif
	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	return 0;
}

static int bq27541_chip_config(struct opchg_bms_charger *di)
{
	int flags = 0, ret = 0;

	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_CNTL_STATUS);
	udelay(66);
	ret = bq27541_read(BQ27541_BQ27411_REG_CNTL, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading register %02x ret = %d\n",
			 BQ27541_BQ27411_REG_CNTL, ret);
		return ret;
	}
	udelay(66);

	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27541_BQ27411_CS_DLOGEN)) {
		bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static void bq27541_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0;
	struct opchg_bms_charger *di;
	unsigned long flags;
	int count = 0;

	di = container_of(work, struct opchg_bms_charger, counter);

	/* retrieve 30 values from FIFO of coulomb data logging buffer
	 * and average over time
	 */
	do {
		ret = bq27541_read(di->cmd_addr.reg_logbuf, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}
		/* delay 66uS, waiting time between continuous reading
		 * results
		 */
		udelay(66);
		ret = bq27541_read(di->cmd_addr.reg_logidx, &index, 0, di);
		if (ret < 0)
			break;
		udelay(66);
	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		dev_err(di->dev, "Error reading datalog register\n");
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}


static int bq27541_get_battery_mvolts(void)
{
	return bq27541_battery_voltage(bq27541_di);
}

static int bq27541_get_battery_temperature(void)
{
	return bq27541_battery_temperature(bq27541_di);
}
static int bq27541_is_battery_present(void)
{
	return 1;
}
static int bq27541_is_battery_temp_within_range(void)
{
	return 1;
}
static int bq27541_is_battery_id_valid(void)
{
	return 1;
}

/* OPPO 2013-08-24 wangjc Add begin for add adc interface. */
#ifdef CONFIG_MACH_OPPO
#if 0
static int bq27541_get_batt_cc(void)//sjc20150105
{
	return bq27541_battery_cc(bq27541_di);
}
static int bq27541_get_batt_fcc(void)//sjc20150105
{
	return bq27541_battery_fcc(bq27541_di);
}
static int bq27541_get_batt_soh(void)//sjc20150105
{
	return bq27541_battery_soh(bq27541_di);
}
static int bq27541_get_batt_rm(void)//sjc20150105
{
	return bq27541_battery_rm(bq27541_di);
}
static int bq27541_get_batt_fac(void)//sjc20150105
{
	return bq27541_battery_fac(bq27541_di);
}
static int bq27541_get_batt_pchg(void)//sjc20150105
{
	return bq27541_battery_pchg(bq27541_di);
}
static int bq27541_get_batt_dod0(void)//sjc20150105
{
	return bq27541_battery_dod0(bq27541_di);
}
static int bq27541_get_batt_flags(void)//sjc20150105
{
	return bq27541_battery_flags(bq27541_di);
}
#endif
static int bq27541_get_batt_remaining_capacity(void)
{
	return bq27541_remaining_capacity(bq27541_di);
}

static int bq27541_get_battery_soc(void)
{
	return bq27541_battery_soc(bq27541_di, false);
}


static int bq27541_get_average_current(void)
{
	return bq27541_average_current(bq27541_di);
}

//wangjc add for authentication
static int bq27541_is_battery_authenticated(void)
{
	if(bq27541_di) {
		return bq27541_di->is_authenticated;
	}
	return false;
}

static int bq27541_fast_chg_started(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_chg_started;
	}
	return false;
}

static int bq27541_fast_switch_to_normal(void)
{
	if(bq27541_di) {
		//pr_err("%s fast_switch_to_normal:%d\n",__func__,bq27541_di->fast_switch_to_normal);
		return bq27541_di->fast_switch_to_normal;
	}
	return false;
}

static int bq27541_set_switch_to_noraml_false(void)
{
	if(bq27541_di) {
		bq27541_di->fast_switch_to_normal = false;
	}

	return 0;
}

static int bq27541_get_fast_low_temp_full(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_low_temp_full;
	}
	return false;
}

static int bq27541_set_fast_low_temp_full_false(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_low_temp_full = false;
	}
	return 0;
}
#endif
/* OPPO 2013-08-24 wangjc Add end */

/* OPPO 2013-12-12 liaofuchun add for set/get fastchg allow begin*/
static int bq27541_fast_normal_to_warm(void)
{
	if(bq27541_di) {
		//pr_err("%s fast_switch_to_normal:%d\n",__func__,bq27541_di->fast_switch_to_normal);
		return bq27541_di->fast_normal_to_warm;
	}
	return 0;
}

static int bq27541_set_fast_normal_to_warm_false(void)
{
	if(bq27541_di) {
		bq27541_di->fast_normal_to_warm = false;
	}

	return 0;
}

static int bq27541_set_fast_chg_allow(int enable)
{
	if(bq27541_di) {
		bq27541_di->fast_chg_allow = enable;
	}
	return 0;
}

static int bq27541_get_fast_chg_allow(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_chg_allow;
	}
	return 0;
}

static int bq27541_get_fast_chg_ing(void)
{
	if(bq27541_di) {
			return bq27541_di->fast_chg_ing;
		}
	return 0;
}

/* OPPO 2013-12-12 liaofuchun add for set/get fastchg allow end */

static struct qpnp_battery_gauge bq27541_batt_gauge = {
	.get_battery_mvolts		= bq27541_get_battery_mvolts,
	.get_battery_temperature	= bq27541_get_battery_temperature,
	.is_battery_present		= bq27541_is_battery_present,
	.is_battery_temp_within_range	= bq27541_is_battery_temp_within_range,
	.is_battery_id_valid		= bq27541_is_battery_id_valid,
/* OPPO 2013-09-30 wangjc Add begin for add new interface */
#ifdef CONFIG_MACH_OPPO
#if 0
	.get_batt_cc				= bq27541_get_batt_cc,
	.get_batt_fcc				= bq27541_get_batt_fcc,
	.get_batt_soh				= bq27541_get_batt_soh,
	.get_batt_rm				= bq27541_get_batt_rm,
	.get_batt_fac				= bq27541_get_batt_fac,
	.get_batt_pchg				= bq27541_get_batt_pchg,
	.get_batt_dod0				= bq27541_get_batt_dod0,
	.get_batt_flags				= bq27541_get_batt_flags,
#endif
	.get_batt_remaining_capacity = bq27541_get_batt_remaining_capacity,
	.get_battery_soc			= bq27541_get_battery_soc,
	.get_average_current		= bq27541_get_average_current,
	//wangjc add for authentication
	.is_battery_authenticated	= bq27541_is_battery_authenticated,
	.fast_chg_started			= bq27541_fast_chg_started,
	.fast_switch_to_normal		= bq27541_fast_switch_to_normal,
	.set_switch_to_noraml_false	= bq27541_set_switch_to_noraml_false,
	.set_fast_chg_allow			= bq27541_set_fast_chg_allow,
	.get_fast_chg_allow			= bq27541_get_fast_chg_allow,
	.fast_normal_to_warm		= bq27541_fast_normal_to_warm,
	.set_normal_to_warm_false	= bq27541_set_fast_normal_to_warm_false,
	.get_fast_chg_ing			= bq27541_get_fast_chg_ing,
	.get_fast_low_temp_full		= bq27541_get_fast_low_temp_full,
	.set_low_temp_full_false	= bq27541_set_fast_low_temp_full_false,
#endif
/* OPPO 2013-09-30 wangjc Add end */
};

static bool bq27541_authenticate(struct i2c_client *client);
static int bq27541_batt_type_detect(struct i2c_client *client);

#ifdef CONFIG_GAUGE_BQ27411
#define DEVICE_TYPE_BQ27541			0x0541
#define DEVICE_TYPE_BQ27411			0x0421
#define DEVICE_BQ27541				0
#define DEVICE_BQ27411				1
static void gauge_set_cmd_addr(struct opchg_bms_charger *di,int device_type)
{
	if(device_type == DEVICE_BQ27541){
		di->cmd_addr.reg_cntl = BQ27541_REG_CNTL;
		di->cmd_addr.reg_temp = BQ27541_REG_TEMP;
		di->cmd_addr.reg_volt = BQ27541_REG_VOLT;
		di->cmd_addr.reg_flags = BQ27541_REG_FLAGS;
		di->cmd_addr.reg_nac = BQ27541_REG_NAC;
		di->cmd_addr.reg_fac = BQ27541_REG_FAC;
		di->cmd_addr.reg_rm = BQ27541_REG_RM;
		di->cmd_addr.reg_fcc = BQ27541_REG_FCC;
		di->cmd_addr.reg_ai = BQ27541_REG_AI;
		di->cmd_addr.reg_si = BQ27541_REG_SI;
		di->cmd_addr.reg_mli = BQ27541_REG_MLI;
		di->cmd_addr.reg_ap = BQ27541_REG_AP;
		di->cmd_addr.reg_soc = BQ27541_REG_SOC;
		di->cmd_addr.reg_inttemp = BQ27541_REG_INTTEMP;
		di->cmd_addr.reg_soh = BQ27541_REG_SOH;
		di->cmd_addr.flag_dsc = BQ27541_FLAG_DSC;
		di->cmd_addr.flag_fc = BQ27541_FLAG_FC;
		di->cmd_addr.cs_dlogen = BQ27541_CS_DLOGEN;
		di->cmd_addr.cs_ss = BQ27541_CS_SS;
		di->cmd_addr.reg_ar = BQ27541_REG_AR;
		di->cmd_addr.reg_artte = BQ27541_REG_ARTTE;
		di->cmd_addr.reg_tte = BQ27541_REG_TTE;
		di->cmd_addr.reg_ttf = BQ27541_REG_TTF;
		di->cmd_addr.reg_stte = BQ27541_REG_STTE;
		di->cmd_addr.reg_mltte = BQ27541_REG_MLTTE;
		di->cmd_addr.reg_ae = BQ27541_REG_AE;
		di->cmd_addr.reg_ttecp = BQ27541_REG_TTECP;
		di->cmd_addr.reg_cc = BQ27541_REG_CC;
		di->cmd_addr.reg_nic = BQ27541_REG_NIC;
		di->cmd_addr.reg_icr = BQ27541_REG_ICR;
		di->cmd_addr.reg_logidx = BQ27541_REG_LOGIDX;
		di->cmd_addr.reg_logbuf = BQ27541_REG_LOGBUF;
		di->cmd_addr.reg_dod0 = BQ27541_REG_DOD0;
		di->cmd_addr.subcmd_cntl_status = BQ27541_SUBCMD_CNTL_STATUS;
		di->cmd_addr.subcmd_device_type = BQ27541_SUBCMD_DEVICE_TYPE;
		di->cmd_addr.subcmd_fw_ver = BQ27541_SUBCMD_FW_VER;
		di->cmd_addr.subcmd_dm_code = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_prev_macw = BQ27541_SUBCMD_PREV_MACW;
		di->cmd_addr.subcmd_chem_id = BQ27541_SUBCMD_CHEM_ID;
		di->cmd_addr.subcmd_set_hib = BQ27541_SUBCMD_SET_HIB;
		di->cmd_addr.subcmd_clr_hib = BQ27541_SUBCMD_CLR_HIB;
		di->cmd_addr.subcmd_set_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_sealed = BQ27541_SUBCMD_SEALED;
		di->cmd_addr.subcmd_reset = BQ27541_SUBCMD_RESET;
		di->cmd_addr.subcmd_softreset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_exit_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_enable_dlog = BQ27541_SUBCMD_ENABLE_DLOG;
		di->cmd_addr.subcmd_disable_dlog = BQ27541_SUBCMD_DISABLE_DLOG;
		di->cmd_addr.subcmd_enable_it = BQ27541_SUBCMD_ENABLE_IT;
		di->cmd_addr.subcmd_disable_it = BQ27541_SUBCMD_DISABLE_IT;
		di->cmd_addr.subcmd_hw_ver = BQ27541_SUBCMD_HW_VER;
		di->cmd_addr.subcmd_df_csum = BQ27541_SUBCMD_DF_CSUM;
		di->cmd_addr.subcmd_bd_offset = BQ27541_SUBCMD_BD_OFFSET;
		di->cmd_addr.subcmd_int_offset = BQ27541_SUBCMD_INT_OFFSET;
		di->cmd_addr.subcmd_cc_ver = BQ27541_SUBCMD_CC_VER;
		di->cmd_addr.subcmd_ocv = BQ27541_SUBCMD_OCV;
		di->cmd_addr.subcmd_bat_ins = BQ27541_SUBCMD_BAT_INS;
		di->cmd_addr.subcmd_bat_rem = BQ27541_SUBCMD_BAT_REM;
		di->cmd_addr.subcmd_set_slp = BQ27541_SUBCMD_SET_SLP;
		di->cmd_addr.subcmd_clr_slp = BQ27541_SUBCMD_CLR_SLP;
		di->cmd_addr.subcmd_fct_res = BQ27541_SUBCMD_FCT_RES;
		di->cmd_addr.subcmd_cal_mode = BQ27541_SUBCMD_CAL_MODE;
	} else {		//device_bq27411
		di->cmd_addr.reg_cntl = BQ27411_REG_CNTL;
		di->cmd_addr.reg_temp = BQ27411_REG_TEMP;
		di->cmd_addr.reg_volt = BQ27411_REG_VOLT;
		di->cmd_addr.reg_flags = BQ27411_REG_FLAGS;
		di->cmd_addr.reg_nac = BQ27411_REG_NAC;
		di->cmd_addr.reg_fac = BQ27411_REG_FAC;
		di->cmd_addr.reg_rm = BQ27411_REG_RM;
		di->cmd_addr.reg_fcc = BQ27411_REG_FCC;
		di->cmd_addr.reg_ai = BQ27411_REG_AI;
		di->cmd_addr.reg_si = BQ27411_REG_SI;
		di->cmd_addr.reg_mli = BQ27411_REG_MLI;
		di->cmd_addr.reg_ap = BQ27411_REG_AP;
		di->cmd_addr.reg_soc = BQ27411_REG_SOC;
		di->cmd_addr.reg_inttemp = BQ27411_REG_INTTEMP;
		di->cmd_addr.reg_soh = BQ27411_REG_SOH;
		di->cmd_addr.flag_dsc = BQ27411_FLAG_DSC;
		di->cmd_addr.flag_fc = BQ27411_FLAG_FC;
		di->cmd_addr.cs_dlogen = BQ27411_CS_DLOGEN;
		di->cmd_addr.cs_ss = BQ27411_CS_SS;
		/*bq27541 external standard cmds*/
		di->cmd_addr.reg_ar = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_artte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_tte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_ttf = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_stte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_mltte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_ae = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_ttecp = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_cc = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_nic = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_icr = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_logidx = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_logbuf = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_dod0 = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_cntl_status = BQ27411_SUBCMD_CNTL_STATUS;
		di->cmd_addr.subcmd_device_type = BQ27411_SUBCMD_DEVICE_TYPE;
		di->cmd_addr.subcmd_fw_ver = BQ27411_SUBCMD_FW_VER;
		di->cmd_addr.subcmd_dm_code = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_prev_macw = BQ27411_SUBCMD_PREV_MACW;
		di->cmd_addr.subcmd_chem_id = BQ27411_SUBCMD_CHEM_ID;
		di->cmd_addr.subcmd_set_hib = BQ27411_SUBCMD_SET_HIB;
		di->cmd_addr.subcmd_clr_hib = BQ27411_SUBCMD_CLR_HIB;
		di->cmd_addr.subcmd_set_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_sealed = BQ27411_SUBCMD_SEALED;
		di->cmd_addr.subcmd_reset = BQ27411_SUBCMD_RESET;
		di->cmd_addr.subcmd_softreset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_exit_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_enable_dlog = BQ27411_SUBCMD_ENABLE_DLOG;
		di->cmd_addr.subcmd_disable_dlog = BQ27411_SUBCMD_DISABLE_DLOG;
		di->cmd_addr.subcmd_enable_it = BQ27411_SUBCMD_ENABLE_IT;
		di->cmd_addr.subcmd_disable_it = BQ27411_SUBCMD_DISABLE_IT;
		/*bq27541 external sub cmds*/
		di->cmd_addr.subcmd_hw_ver = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_df_csum = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_bd_offset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_int_offset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_cc_ver = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_ocv = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_bat_ins = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_bat_rem = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_set_slp = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_clr_slp = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_fct_res = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_cal_mode = BQ27541_BQ27411_CMD_INVALID;
	}
}
#endif
static void bq27541_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0;
	struct opchg_bms_charger *di;

	di  = container_of(work, struct opchg_bms_charger, hw_config.work);
	ret = bq27541_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27541\n");
		di->retry_count--;
		if(di->retry_count > 0) {
			schedule_delayed_work(&di->hw_config, HZ);
		}
		return;
	}

	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_CNTL_STATUS);
	udelay(66);
	bq27541_read(BQ27541_BQ27411_REG_CNTL, &flags, 0, di);
	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_DEVICE_TYPE);
	udelay(66);
	bq27541_read(BQ27541_BQ27411_REG_CNTL, &type, 0, di);
	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read(BQ27541_BQ27411_REG_CNTL, &fw_ver, 0, di);

#ifdef CONFIG_GAUGE_BQ27411
	if(type == DEVICE_TYPE_BQ27411)
	{
		di->device_type = DEVICE_BQ27411;
		register_device_proc("bms_ic", DEVICE_BMS_IC_VERSION, DEVICE_BMS_IC_TYPE_BQ27411);
	}
	else if(type == DEVICE_TYPE_BQ27541)
	{
		di->device_type = DEVICE_BQ27541;
		register_device_proc("bms_ic", DEVICE_BMS_IC_VERSION, DEVICE_BMS_IC_TYPE_BQ27541);
	}
	else
	{
		register_device_proc("bms_ic", DEVICE_BMS_IC_VERSION, DEVICE_BMS_IC_TYPE_UNKOWN);
	}
	gauge_set_cmd_addr(di,di->device_type);
#endif
	qpnp_battery_gauge_register(&bq27541_batt_gauge);

	di->is_authenticated = bq27541_authenticate(di->client);
	di->battery_type = bq27541_batt_type_detect(di->client);
	dev_err(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n",
			type, fw_ver);
	dev_info(di->dev, "Complete bq27541 configuration 0x%02X\n", flags);
}

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct opchg_bms_charger *di)
{
	struct i2c_client *client = di->client;
/* OPPO 2013-12-09 wangjc Modify begin for use standard i2c interface */
#ifndef CONFIG_MACH_OPPO
	struct i2c_msg msg[1];
#else
	struct i2c_msg msg[2];
#endif
/* OPPO 2013-12-09 wangjc Modify end */
	unsigned char data[2];
	int err;

#ifdef CONFIG_GAUGE_BQ27411
	if(reg == BQ27541_BQ27411_CMD_INVALID)
		return 0;
#endif

	if (!client->adapter)
		return -ENODEV;

	mutex_lock(&battery_mutex);
/* OPPO 2013-12-09 wangjc Modify begin for use standard i2c interface */
#ifndef CONFIG_MACH_OPPO
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			mutex_unlock(&battery_mutex);

			return 0;
		}
	}
#else
	/* Write register */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	data[0] = reg;

	/* Read data */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	if (!b_single)
		msg[1].len = 2;
	else
		msg[1].len = 1;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err >= 0) {
		if (!b_single)
			*rt_value = get_unaligned_le16(data);
		else
			*rt_value = data[0];

		mutex_unlock(&battery_mutex);

		return 0;
	}
#endif
/* OPPO 2013-12-09 wangjc Modify end */
	mutex_unlock(&battery_mutex);

	return err;
}

#ifdef CONFIG_BQ27541_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27541_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct opchg_bms_charger *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= di->cmd_addr.reg_icr && reg > 0x00) {
		ret = bq27541_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	return ret;
}

static ssize_t bq27541_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct opchg_bms_charger *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == di->cmd_addr.subcmd_device_type ||
		 subcmd == di->cmd_addr.subcmd_fw_ver ||
		 subcmd == di->cmd_addr.subcmd_hw_ver ||
		 subcmd == di->cmd_addr.subcmd_chem_id) {

		bq27541_cntl_cmd(di, subcmd); /* Retrieve Chip status */
		udelay(66);
		ret = bq27541_read(di->cmd_addr.reg_cntl, &temp, 0, di);

		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27541_read_stdcmd,
	bq27541_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27541_read_subcmd,
	bq27541_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name			= "bq27541-test",
	.id			= -1,
	.dev.platform_data	= NULL,
};
#endif

#ifdef CONFIG_MACH_OPPO
/*OPPO 2013-09-18 liaofuchun add begin for bq27541 authenticate */
#define BLOCKDATACTRL	0X61
#define DATAFLASHBLOCK	0X3F
#define AUTHENDATA		0X40
#define AUTHENCHECKSUM	0X54
#define MESSAGE_LEN		20
#define KEY_LEN			16

/* OPPO 2014-02-25 sjc Modify begin for FIND7OP not use authenticate */
#ifndef CONFIG_OPPO_DEVICE_FIND7
static bool bq27541_authenticate(struct i2c_client *client)
{
	return true;
}
#else
static bool bq27541_authenticate(struct i2c_client *client)
{
	char recv_buf[MESSAGE_LEN]={0x0};
	char send_buf[MESSAGE_LEN]={0x0};
	char result[MESSAGE_LEN]={0x0};
	char Key[KEY_LEN]={0x77,0x30,0xa1,0x28,0x0a,0xa1,0x13,0x20,0xef,0xcd,0xab,0x89,0x67,0x45,0x23,0x01};
	char checksum_buf[1] ={0x0};
	char authen_cmd_buf[1] = {0x00};
	int i,rc;
	pr_info("%s Enter\n",__func__);

	// step 0: produce 20 bytes random data and checksum
	get_random_bytes(send_buf,20);
	for(i = 0;i < 20;i++){
		checksum_buf[0] = checksum_buf[0] + send_buf[i];
	}
	checksum_buf[0] = 0xff - (checksum_buf[0]&0xff);

	/* step 1: unseal mode->write 0x01 to blockdatactrl
	authen_cmd_buf[0] = 0x01;
	rc = i2c_smbus_write_i2c_block_data(client,BLOCKDATACTRL,1,&authen_cmd_buf[0]);
	}	*/

	// step 1: seal mode->write 0x00 to dataflashblock
	rc = i2c_smbus_write_i2c_block_data(client,DATAFLASHBLOCK,1,&authen_cmd_buf[0]);
	if( rc < 0 ){
		pr_info("%s i2c write error\n",__func__);
		return false;
	}
	// step 2: write 20 bytes to authendata_reg
	i2c_smbus_write_i2c_block_data(client,AUTHENDATA,MESSAGE_LEN,&send_buf[0]);
	msleep(1);
	// step 3: write checksum to authenchecksum_reg for compute
	i2c_smbus_write_i2c_block_data(client,AUTHENCHECKSUM,1,&checksum_buf[0]);
	msleep(50);
	// step 4: read authendata
	i2c_smbus_read_i2c_block_data(client,AUTHENDATA,MESSAGE_LEN,&recv_buf[0]);
	// step 5: phone do hmac(sha1-generic) algorithm
	BQ27541_HMACSHA1_authenticate(send_buf,Key,result);
	// step 6: compare recv_buf from bq27541 and result by phone
	rc = strncmp(recv_buf,result,MESSAGE_LEN);
	if(rc == 0){
		pr_info("bq27541_authenticate success\n");
		return true;
	}
	pr_info("bq27541_authenticate error,dump buf:\n");
	for(i = 0;i < 20;i++){
		pr_info("BQ27541 send_buf[%d]:0x%x,recv_buf[%d]:0x%x = result[%d]:0x%x\n",i,send_buf[i],i,recv_buf[i],i,result[i]);
	}
	return false;
}
#endif //CONFIG_OPPO_DEVICE_FIND7OP
/* OPPO 2014-02-25 sjc Modify end */
#endif //CONFIG_MACH_OPPO

#ifdef CONFIG_MACH_OPPO
//Fuchun.Liao@EXP.Driver,2014/01/10 add for check battery type
#define BATTERY_2700MA		0
#define BATTERY_3000MA		1
#define TYPE_INFO_LEN		8

#ifndef CONFIG_OPPO_DEVICE_FIND7OP
/* jingchun.wang@Onlinerd.Driver, 2014/03/10  Modify for 14001 */
static int bq27541_batt_type_detect(struct i2c_client *client)
{
	char blockA_cmd_buf[1] = {0x01};
	char rc = 0;
	char recv_buf[TYPE_INFO_LEN] = {0x0};
	int i = 0;

	rc = i2c_smbus_write_i2c_block_data(client,DATAFLASHBLOCK,1,&blockA_cmd_buf[0]);
	if( rc < 0 ){
		pr_info("%s i2c write error\n",__func__);
		return 0;
	}
	msleep(30);	//it is needed
	i2c_smbus_read_i2c_block_data(client,AUTHENDATA,TYPE_INFO_LEN,&recv_buf[0]);
	if((recv_buf[0] == 0x01) && (recv_buf[1] == 0x09) && (recv_buf[2] == 0x08) && (recv_buf[3] == 0x06))
		rc = BATTERY_2700MA;
	else if((recv_buf[0] == 0x02) && (recv_buf[1] == 0x00) && (recv_buf[2] == 0x01) && (recv_buf[3] == 0x03))
		rc = BATTERY_3000MA;
	else {
		for(i = 0;i < TYPE_INFO_LEN;i++)
			pr_info("%s error,recv_buf[%d]:0x%x\n",__func__,i,recv_buf[i]);
		rc =  BATTERY_2700MA;
	}
	pr_info("%s battery_type:%d\n",__func__,rc);
	return rc;
}
#else /*CONFIG_OPPO_DEVICE_FIND7OP*/
static int bq27541_batt_type_detect(struct i2c_client *client)
{
	return BATTERY_3000MA;
}
#endif /*CONFIG_OPPO_DEVICE_FIND7OP*/
#endif


int opchg_set_gpio_val(int gpio , u8 val)
{
	int rc=0;

	if (!gpio_is_valid(gpio))//sjc
		return rc;
	gpio_set_value(gpio,val);
	return rc;
}

int opchg_get_gpio_val(int gpio)
{
	int rc=0;

	if (!gpio_is_valid(gpio))//sjc
		return rc;
	rc=gpio_get_value(gpio);
	return rc;
}

int opchg_set_gpio_dir_output(int gpio , u8 val)
{
	int rc=0;

	if (!gpio_is_valid(gpio))//sjc
		return rc;
	rc=gpio_direction_output(gpio,val);
	return rc;
}
int opchg_set_gpio_dir_intput(int gpio)
{
	int rc=0;

	if (!gpio_is_valid(gpio))//sjc
		return rc;
	rc=gpio_direction_input(gpio);
	return rc;
}

/* OPPO 2013-12-12 liaofuchun add for fastchg */
#ifdef OPPO_USE_FAST_CHARGER

static irqreturn_t irq_rx_handler(int irq, void *dev_id)
{
	struct opchg_bms_charger *di = dev_id;
	//pr_info("%s\n", __func__);

	schedule_work(&di->fastcg_work);
	return IRQ_HANDLED;
}

int vooc_get_mcu_hw_type(void)
{
	if (is_project(OPPO_15022) && get_PCB_Version() == HW_VERSION__16) {
		return OPCHG_VOOC_STM8S_ID;
	} else if (is_project(OPPO_14005) || is_project(OPPO_15011) ||
			is_project(OPPO_15018) || is_project(OPPO_15022)) {
		return OPCHG_VOOC_PIC16F_ID;
	} else {
		return OPCHG_VOOC_UNKOWN_ID;
	}
}

static void fastcg_work_func(struct work_struct *work)
{
	int data = 0;
	int i;
	int bit = 0;
	int retval = 0;
	int ret_info = 0;
	static int fw_ver_info = 0;
	int remain_cap = 0;
	static bool isnot_power_on = 0;
	int rc =0	;


	//pr_err("%s is start\n",__func__);
	if(is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		usleep_range(2000,2000);
		if(opchg_get_gpio_val(bq27541_di->opchg_data_gpio) != 1)
		{
			pr_err("%s  Shield fastchg irq. opchg_data_gpio=%d\n", __func__,opchg_get_gpio_val(bq27541_di->opchg_data_gpio));
			return;
		}
		else
		{
		//	pr_err("%s  Shield fastchg irq. opchg_data_gpio=%d\n", __func__,opchg_get_gpio_val(bq27541_di->opchg_data_gpio));
		}
	}

	free_irq(bq27541_di->irq, bq27541_di);

	// step1: get MCU transfer Data
	for(i = 0; i < 7; i++)
	{
		if(is_project(OPPO_14005) || is_project(OPPO_15011)|| is_project(OPPO_15018) || is_project(OPPO_15022))
		{
			rc =opchg_set_clock_active(bq27541_di);
			usleep_range(1000,1000);
			rc =opchg_set_clock_sleep(bq27541_di);
			usleep_range(19000,19000);
			bit = opchg_get_gpio_val(bq27541_di->opchg_data_gpio);
		}

		data |= bit<<(6-i);
		if((i == 2) && (data != 0x50) && (!fw_ver_info)){	//data recvd not start from "101"
			pr_err("%s data err:0x%x\n",__func__,data);

			if(is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				#ifdef OPPO_USE_FAST_CHARGER_RESET_MCU
				if(opchg_chip != NULL)
			{
					opchg_chip->fast_charger_reset_count=0;
				}
				#endif

			}

			if(bq27541_di->fast_chg_started == true) {
				bq27541_di->alow_reading = true;
				bq27541_di->fast_chg_started = false;
				#if 0
				if (opchg_chip->vooc_start_step > 0) {
				    opchg_chip->vooc_start_step = 1;
				}
				#else
				if (vooc_start_step > 0) {
				    //vooc_start_step = 1;
				    vooc_start_step = OPCHG_VOOC_TO_STANDARD;
				}
				#endif
				bq27541_di->fast_chg_allow = false;
				bq27541_di->fast_switch_to_normal = false;
				bq27541_di->fast_normal_to_warm = false;
				bq27541_di->fast_chg_ing = false;

				rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
				#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
				pr_err("%s data err:0x%x fastchg handshake fail\n", __func__,data);
				#endif

				power_supply_changed(bq27541_di->batt_psy);
			}
			goto out;
		}
	}
	pr_err("%s recv data:0x%x\n", __func__, data);

	// step2: Fast charge connected is success
	if(data == VOOC_NOTIFY_FAST_PRESENT) {
		//request fast charging
		wake_lock(&bq27541_di->fastchg_wake_lock);
		vooc_need_to_up_fw = 0;
		fw_ver_info = 0;
		bq27541_di->alow_reading = false;
		bq27541_di->fast_chg_started = true;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_normal_to_warm = false;

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		mod_timer(&bq27541_di->watchdog,
		  jiffies + msecs_to_jiffies(15000));
		#else
		schedule_delayed_work(&bq27541_di->watchdog_delayed_work,
                            jiffies + msecs_to_jiffies(15000));
		#endif

		if(!isnot_power_on){
			isnot_power_on = 1;
			ret_info = 0x1;
		} else {
			ret_info = 0x2;
		}

		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s fastchg handshake Success\n", __func__);
		#endif
	}
	// step3: Fast charge connected is unexpectly stop
	else if(data == VOOC_NOTIFY_FAST_ABSENT)
	{
		// add for MMI test
		if((opchg_chip != NULL)&&(the_chip != NULL))
		{
			if(opchg_chip->is_factory_mode == false)
			{
				msleep(1000);
			}
		}

		//fast charge stopped
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		#if 0
		if (opchg_chip->vooc_start_step > 0) {
		    opchg_chip->vooc_start_step = 1;
		}
		#else
		if (vooc_start_step > 0) {
		    //vooc_start_step = 1;
		    vooc_start_step = OPCHG_VOOC_TO_STANDARD;
		}
		#endif
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_switch_to_normal = false;
		bq27541_di->fast_normal_to_warm = false;
		bq27541_di->fast_chg_ing = false;
		//switch off fast chg
		rc = opchg_set_switch_mode(NORMAL_CHARGER_MODE);
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s fastchg stop unexpectly,switch off fastchg\n", __func__);
		#endif

		#ifdef OPPO_USE_FAST_CHARGER
		if( is_project(OPPO_14005) || is_project(OPPO_15011)||
			is_project(OPPO_15018) || is_project(OPPO_15022)) {
			if((opchg_chip != NULL)&&(the_chip != NULL))
			{
				// add for MMI test
				if((opchg_get_charger_inout() == 0)&&(opchg_chip->is_factory_mode))
				{
					if (opchg_chip->chg_present == true) {
						opchg_chip->chg_present = false;
						pr_debug("oppo_debug check fastcharger is out\n");
						rc = bq24196_chg_uv(opchg_chip, 1);
					}
				}
			}
		}
		#endif

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		del_timer(&bq27541_di->watchdog);
		#else
		cancel_delayed_work(&bq27541_di->watchdog_delayed_work);
		#endif
		ret_info = 0x2;
	}
	// step4: ap with mcu handshake normal,fast charging  is success
	else if(data == VOOC_NOTIFY_ALLOW_READING_IIC)
	{
		//ap with mcu handshake normal,enable fast charging
		// allow read i2c
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_ing = true;

		if(opchg_chip != NULL)
		{
		opchg_chip->bat_instant_vol=bq27541_get_battery_mvolts();
		opchg_chip->temperature= bq27541_get_battery_temperature();
		remain_cap = bq27541_get_batt_remaining_capacity();
		opchg_chip->bat_volt_check_point= bq27541_get_battery_soc();
		opchg_chip->charging_current= bq27541_get_average_current();
		//opchg_chip->charger_type = qpnp_charger_type_get(opchg_chip);
		pr_err("%s volt:%d,temp:%d,remain_cap:%d,soc:%d,current:%d,charger_type:%d\n",__func__,opchg_chip->bat_instant_vol,
			opchg_chip->temperature,remain_cap,opchg_chip->bat_volt_check_point,opchg_chip->charging_current,opchg_chip->charger_type);
		}
		//don't read
		bq27541_di->alow_reading = false;
		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		mod_timer(&bq27541_di->watchdog,
		  jiffies + msecs_to_jiffies(15000));
		#else
		schedule_delayed_work(&bq27541_di->watchdog_delayed_work,
			  jiffies + msecs_to_jiffies(15000));
		#endif

		ret_info = 0x2;
	}
	// step5: ap with mcu handshake normal,fast charging  is normal full
	else if(data == VOOC_NOTIFY_NORMAL_TEMP_FULL)
	{
		//fastchg full,vbatt > 4350
		rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s fast charging full in normal temperature\n", __func__);
		#endif

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		del_timer(&bq27541_di->watchdog);
		#else
		cancel_delayed_work(&bq27541_di->watchdog_delayed_work);
		#endif

		ret_info = 0x2;
	}
	// step6: ap with mcu handshake normal,fast charging  is low  temperature full
	else if(data == VOOC_NOTIFY_LOW_TEMP_FULL)
	{
		//the  fast charging full in low temperature
		if (bq27541_di->battery_type == BATTERY_3000MA){	//13097 ATL battery
			//if temp:10~20 decigec,vddmax = 4250mv
			//switch off fast chg
			pr_info("%s fastchg low temp full,switch off fastchg,set GPIO96 0\n", __func__);

			rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
		}
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s bq27541_di->battery_type = %d,fastchg low temp full\n", __func__,bq27541_di->battery_type);
		#endif

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		del_timer(&bq27541_di->watchdog);
		#else
		cancel_delayed_work(&bq27541_di->watchdog_delayed_work);
		#endif

		ret_info = 0x2;
	}
	// step7: battery is not connect  stop charging
	else if(data == VOOC_NOTIFY_BAD_CONNECTED)
	{
		//usb bad connected,stop fastchg
		rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);

		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s the battery is disconnect  stop fast cherging\n", __func__);
		#endif

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		del_timer(&bq27541_di->watchdog);
		#else
		cancel_delayed_work(&bq27541_di->watchdog_delayed_work);
		#endif

		ret_info = 0x2;
	}
	// step8: temp is not up or dowm stop charging
	else if(data == VOOC_NOTIFY_TEMP_OVER)
	{
		//fastchg temp over 45 or under 20
		rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s fastchg temp > 49 or < 15,stop fast charging\n", __func__);
		#endif

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		del_timer(&bq27541_di->watchdog);
		#else
		cancel_delayed_work(&bq27541_di->watchdog_delayed_work);
		#endif

		ret_info = 0x2;
	}
	// step9: up date firmware
	else if(data == VOOC_NOTIFY_FIRMWARE_UPDATE)
	{
		//ready to get fw_ver
		fw_ver_info = 1;
		ret_info = 0x2;
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s is update the mcu firmware\n", __func__);
		#endif
	}
	else if(fw_ver_info)
	{
#if 0
		//fw in local is large than mcu1503_fw_ver
		if(is_project(OPPO_14005))
		{
			if((!pic_have_updated) && (Pic16F_firmware_data_14005[pic_fw_ver_count_14005 - 4] > data)){
				ret_info = 0x2;
				pic_need_to_up_fw = 1;	//need to update fw
			}else{
				ret_info = 0x1;
				pic_need_to_up_fw = 0;	//fw is already new,needn't to up
			}
		}
		else if(is_project(OPPO_15011))
		{
			if((!pic_have_updated) && (Pic16F_firmware_data_15011[pic_fw_ver_count_15011 - 4] > data)){
				ret_info = 0x2;
				pic_need_to_up_fw = 1;	//need to update fw
			}else{
				ret_info = 0x1;
				pic_need_to_up_fw = 0;	//fw is already new,needn't to up
			}
		}
		else if(is_project(OPPO_15018))
		{
			if((!pic_have_updated) && (Pic16F_firmware_data_15018[pic_fw_ver_count_15018 - 4] > data)){
				ret_info = 0x2;
				pic_need_to_up_fw = 1;	//need to update fw
			}else{
				ret_info = 0x1;
				pic_need_to_up_fw = 0;	//fw is already new,needn't to up
			}
		}
		else if (is_project(OPPO_15022))
		{
			if((!pic_have_updated) && (Pic16F_firmware_data_15022[pic_fw_ver_count_15022 - 4] > data)){
				ret_info = 0x2;
				pic_need_to_up_fw = 1;	//need to update fw
			}else{
				ret_info = 0x1;
				pic_need_to_up_fw = 0;	//fw is already new,needn't to up
			}
		}
		else
		{
			if((!pic_have_updated) && (Pic16F_firmware_data[pic_fw_ver_count - 4] > data)){
				ret_info = 0x2;
				pic_need_to_up_fw = 1;	//need to update fw
			}else{
				ret_info = 0x1;
				pic_need_to_up_fw = 0;	//fw is already new,needn't to up
			}
		}

		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		if(is_project(OPPO_14005))
		{
			pr_err("local_fw:0x%x,need_to_up_fw:%d\n",Pic16F_firmware_data_14005[pic_fw_ver_count_14005 - 4],pic_need_to_up_fw);
		}
		else if(is_project(OPPO_15011))
		{
			pr_err("local_fw:0x%x,need_to_up_fw:%d\n",Pic16F_firmware_data_15011[pic_fw_ver_count_15011 - 4],pic_need_to_up_fw);
		}
		else if(is_project(OPPO_15018))
		{
			pr_err("local_fw:0x%x,need_to_up_fw:%d\n",Pic16F_firmware_data_15018[pic_fw_ver_count_15018 - 4],pic_need_to_up_fw);
		}
		else if(is_project(OPPO_15022))
		{
			pr_err("local_fw:0x%x,need_to_up_fw:%d\n",Pic16F_firmware_data_15022[pic_fw_ver_count_15022 - 4],pic_need_to_up_fw);
		}
		else
		{
			pr_err("local_fw:0x%x,need_to_up_fw:%d\n",Pic16F_firmware_data[pic_fw_ver_count - 4],pic_need_to_up_fw);
		}
		#endif
#endif
		if ((!vooc_have_updated) && (vooc_firmware_data[vooc_fw_ver_count - 4] > data)){
			ret_info = 0x2;
			vooc_need_to_up_fw = 1;	//need to update fw
		} else {
			ret_info = 0x1;
			vooc_need_to_up_fw = 0;	//fw is already new,needn't to up
		}
		//#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("local_fw:0x%x,need_to_up_fw:%d\n",vooc_firmware_data[vooc_fw_ver_count - 4],vooc_need_to_up_fw);
		//#endif
		fw_ver_info = 0;
	}
	else
	{
		rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
		msleep(500);	//avoid i2c conflict
		//data err
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		#if 0
		if (opchg_chip->vooc_start_step > 0) {
		    opchg_chip->vooc_start_step = 1;
		}
		#else
		if (vooc_start_step > 0) {
		    //vooc_start_step = 1;
		    vooc_start_step = OPCHG_VOOC_TO_STANDARD;
		}
		#endif
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_switch_to_normal = false;
		bq27541_di->fast_normal_to_warm = false;
		bq27541_di->fast_chg_ing = false;
		//data err
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s data=0x%x is data err(101xxxx),stop fast charging\n", __func__,data);
		#endif
		power_supply_changed(bq27541_di->batt_psy);
		goto out;
	}
	msleep(2);

	// set fast charging clock and data
	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc =opchg_set_data_sleep(bq27541_di);
		for(i = 0; i < 3; i++) {
			if(i == 0){	//tell mcu1503 battery_type
				opchg_set_gpio_val(bq27541_di->opchg_data_gpio, ret_info >> 1);
			} else if(i == 1){
				opchg_set_gpio_val(bq27541_di->opchg_data_gpio, ret_info & 0x1);
			} else {
				//opchg_set_gpio_val(bq27541_di->opchg_data_gpio,bq27541_di->battery_type);
				opchg_set_gpio_val(bq27541_di->opchg_data_gpio,bq27541_di->device_type);
			}

			rc =opchg_set_clock_active(bq27541_di);
			usleep_range(1000,1000);
			rc =opchg_set_clock_sleep(bq27541_di);
			usleep_range(19000,19000);
		}
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("%s data=0x%x is data success, set 100 to mcu  \n", __func__,data);
		#endif
	}
out:
	// Setting the feedback signal
	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc =opchg_set_data_active(bq27541_di);
		if(is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022)){ //add for int trigger mistakenly
			rc =opchg_set_clock_active(bq27541_di);
			usleep_range(10000,10000);
			rc =opchg_set_clock_sleep(bq27541_di);
			usleep_range(10000,10000);
			usleep_range(15000,15000);
		}
	}

	//lfc add for it is faster than usb_plugged_out irq to send 0x5a(fast_chg full and usb bad connected) to AP
	if(data == VOOC_NOTIFY_NORMAL_TEMP_FULL || data == VOOC_NOTIFY_BAD_CONNECTED){
		usleep_range(180000,180000);
		bq27541_di->fast_switch_to_normal = true;
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		#if 0
		if (opchg_chip->vooc_start_step > 0) {
		    opchg_chip->vooc_start_step = 1;
		}
		#else
		if (vooc_start_step > 0) {
		    //vooc_start_step = 1;
		    vooc_start_step = OPCHG_VOOC_TO_STANDARD;
		}
		#endif
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_chg_ing = false;
	}
	//fastchg temp over( > 45 or < 20)

	//lfc add to set fastchg vddmax = 4250mv during 10 ~ 20 decigec for ATL 3000mAH battery
	if(data == VOOC_NOTIFY_LOW_TEMP_FULL){
		if(bq27541_di->battery_type == BATTERY_3000MA){	//13097 ATL battery
			usleep_range(180000,180000);
			bq27541_di->fast_low_temp_full = true;
			bq27541_di->alow_reading = true;
			bq27541_di->fast_chg_started = false;
			#if 0
			if (opchg_chip->vooc_start_step > 0) {
			    opchg_chip->vooc_start_step = 1;
			}
			#else
			if (vooc_start_step > 0) {
			    //vooc_start_step = 1;
			    vooc_start_step = OPCHG_VOOC_TO_STANDARD;
			}
			#endif
			bq27541_di->fast_chg_allow = false;
			bq27541_di->fast_chg_ing = false;
		}
	}
	//lfc add to set fastchg vddmax = 4250mv end

	if(data == VOOC_NOTIFY_TEMP_OVER){
		usleep_range(180000,180000);
		bq27541_di->fast_normal_to_warm = true;
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		#if 0
		if (opchg_chip->vooc_start_step > 0) {
		    opchg_chip->vooc_start_step = 1;
		}
		#else
		if (vooc_start_step > 0) {
		    //vooc_start_step = 1;
		    vooc_start_step = OPCHG_VOOC_TO_STANDARD;
		}
		#endif
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_chg_ing = false;
	}

	#ifdef OPPO_USE_FAST_CHARGER
	if(vooc_need_to_up_fw){
		msleep(500);

		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		del_timer(&bq27541_di->watchdog);
		#else
		cancel_delayed_work(&bq27541_di->watchdog_delayed_work);
		#endif
		if (vooc_fw_update != NULL && opchg_fast_charger_chip != NULL)
			rc = vooc_fw_update(opchg_fast_charger_chip,false);
		vooc_need_to_up_fw = 0;
		// chang mod timer to delay tims for dengnw in 20141221
		#ifdef OPCHG_VOOC_WATCHDOG
		mod_timer(&bq27541_di->watchdog,
		  jiffies + msecs_to_jiffies(15000));
		#else
		schedule_delayed_work(&bq27541_di->watchdog_delayed_work,
                jiffies + msecs_to_jiffies(15000));
		#endif
	}
	#endif

	//add delay 3ms for data irq
	//usleep_range(3000,3000);
	retval = request_irq(bq27541_di->irq, irq_rx_handler, IRQF_TRIGGER_RISING, "mcu_data", bq27541_di);	//0X01:rising edge,0x02:falling edge
	if(retval < 0) {
		pr_err("%s request ap rx irq failed.\n", __func__);
	}
	else
	{
		pr_err("%s request ap rx irq succeeded.\n", __func__);
	}

	if((data == VOOC_NOTIFY_FAST_PRESENT) || (data == VOOC_NOTIFY_ALLOW_READING_IIC)){
		power_supply_changed(bq27541_di->batt_psy);
	}

	if(data == VOOC_NOTIFY_LOW_TEMP_FULL){
		if(bq27541_di->battery_type == BATTERY_3000MA){
			power_supply_changed(bq27541_di->batt_psy);
			wake_unlock(&bq27541_di->fastchg_wake_lock);
		}
	}

	if((data == VOOC_NOTIFY_FAST_ABSENT) || (data == VOOC_NOTIFY_NORMAL_TEMP_FULL)
					|| (data == VOOC_NOTIFY_BAD_CONNECTED) || (data == VOOC_NOTIFY_TEMP_OVER)){
		power_supply_changed(bq27541_di->batt_psy);
		wake_unlock(&bq27541_di->fastchg_wake_lock);
	}
}


#ifdef OPCHG_VOOC_WATCHDOG
void di_watchdog(unsigned long data)
{
	pr_err("di_watchdog can't receive mcu data\n");
	bq27541_di->alow_reading = true;
	bq27541_di->fast_chg_started = false;
	#if 0
	if (opchg_chip->vooc_start_step > 0) {
	    opchg_chip->vooc_start_step = 0;
	}
	#else
	if (vooc_start_step > 0) {
	    vooc_start_step = OPCHG_VOOC_WATCHDOG_OUT;
	}
	#endif
	bq27541_di->fast_switch_to_normal = false;
	bq27541_di->fast_low_temp_full = false;
	bq27541_di->fast_chg_allow = false;
	bq27541_di->fast_normal_to_warm = false;
	bq27541_di->fast_chg_ing = false;
	//switch off fast chg
	pr_info("%s switch off fastchg\n", __func__);

	opchg_set_switch_mode(NORMAL_CHARGER_MODE);
	wake_unlock(&bq27541_di->fastchg_wake_lock);
}
#else
void di_watchdog(struct work_struct *work)
{
	pr_err("di_watchdog can't receive mcu data\n");
	bq27541_di->alow_reading = true;
	bq27541_di->fast_chg_started = false;
	#if 0
	if (opchg_chip->vooc_start_step > 0) {
	    opchg_chip->vooc_start_step = 0;
	}
	#else
	if (vooc_start_step > 0) {
	    vooc_start_step = OPCHG_VOOC_WATCHDOG_OUT;
	}
	#endif
	bq27541_di->fast_switch_to_normal = false;
	bq27541_di->fast_low_temp_full = false;
	bq27541_di->fast_chg_allow = false;
	bq27541_di->fast_normal_to_warm = false;
	bq27541_di->fast_chg_ing = false;
	//switch off fast chg
	pr_info("%s switch off fastchg\n", __func__);

	opchg_set_switch_mode(NORMAL_CHARGER_MODE);
	wake_unlock(&bq27541_di->fastchg_wake_lock);
}
#endif
#endif
/* OPPO 2013-12-12 liaofuchun add for fastchg */

int opchg_bq27541_parse_dt(struct opchg_bms_charger *di)
{
#ifdef OPPO_USE_FAST_CHARGER
    int rc=0;
	struct device_node *node = di->dev->of_node;

	// Parsing gpio swutch1
	di->opchg_swtich1_gpio = of_get_named_gpio(node, "qcom,charging_swtich1-gpio", 0);
	if(di->opchg_swtich1_gpio < 0 ){
		pr_err("chip->opchg_swtich1_gpio not specified\n");
	}
	else
	{
		if( gpio_is_valid(di->opchg_swtich1_gpio) ){
			rc = gpio_request(di->opchg_swtich1_gpio, "charging-switch1-gpio");
			if(rc){
				pr_err("unable to request gpio [%d]\n", di->opchg_swtich1_gpio);
			}
		}
		pr_err("chip->opchg_swtich1_gpio =%d\n",di->opchg_swtich1_gpio);
	}

	// Parsing gpio swutch2
	if(get_PCB_Version()== HW_VERSION__10)
	{
		di->opchg_swtich2_gpio = of_get_named_gpio(node, "qcom,charging_swtich3-gpio", 0);
		if(di->opchg_swtich2_gpio < 0 ){
			pr_err("chip->opchg_swtich2_gpio not specified\n");
		}
		else
		{
			if( gpio_is_valid(di->opchg_swtich2_gpio) ){
				rc = gpio_request(di->opchg_swtich2_gpio, "charging-switch3-gpio");
				if(rc){
					pr_err("unable to request gpio [%d]\n", di->opchg_swtich2_gpio);
				}
			}
			pr_err("chip->opchg_swtich2_gpio =%d\n",di->opchg_swtich2_gpio);
		}
	}
	else
	{
		di->opchg_swtich2_gpio = of_get_named_gpio(node, "qcom,charging_swtich2-gpio", 0);
		if(di->opchg_swtich2_gpio < 0 ){
			pr_err("chip->opchg_swtich2_gpio not specified\n");
		}
		else
		{
			if( gpio_is_valid(di->opchg_swtich2_gpio) ){
				rc = gpio_request(di->opchg_swtich2_gpio, "charging-switch2-gpio");
				if(rc){
					pr_err("unable to request gpio [%d]\n", di->opchg_swtich2_gpio);
				}
			}
			pr_err("chip->opchg_swtich2_gpio =%d\n",di->opchg_swtich2_gpio);
		}
	}
	// Parsing gpio reset
	di->opchg_reset_gpio = of_get_named_gpio(node, "qcom,charging_reset-gpio", 0);
	if(di->opchg_reset_gpio < 0 ){
		pr_err("chip->opchg_reset_gpio not specified\n");
	}
	else
	{
		if( gpio_is_valid(di->opchg_reset_gpio) ){
			rc = gpio_request(di->opchg_reset_gpio, "charging-reset-gpio");
			if(rc){
				pr_err("unable to request gpio [%d]\n", di->opchg_reset_gpio);
			}
		}
		pr_err("chip->opchg_reset_gpio =%d\n",di->opchg_reset_gpio);
	}

	// Parsing gpio clock
	di->opchg_clock_gpio = of_get_named_gpio(node, "qcom,charging_clock-gpio", 0);
	if(di->opchg_clock_gpio < 0 ){
		pr_err("chip->opchg_clock_gpio not specified\n");
	}
	else
	{
		if( gpio_is_valid(di->opchg_clock_gpio) ){
			rc = gpio_request(di->opchg_clock_gpio, "charging-clock-gpio");
			if(rc){
				pr_err("unable to request gpio [%d]\n", di->opchg_clock_gpio);
			}
		}
		pr_err("chip->opchg_clock_gpio =%d\n",di->opchg_clock_gpio);
	}

	// Parsing gpio data
	di->opchg_data_gpio = of_get_named_gpio(node, "qcom,charging_data-gpio", 0);
	if(di->opchg_data_gpio < 0 ){
		pr_err("chip->opchg_data_gpio not specified\n");
	}
	else
	{
		if( gpio_is_valid(di->opchg_data_gpio) ){
			rc = gpio_request(di->opchg_data_gpio, "charging-data-gpio");
			if(rc){
				pr_err("unable to request gpio [%d]\n", di->opchg_data_gpio);
			}
		}
		pr_err("chip->opchg_data_gpio =%d\n",di->opchg_data_gpio);
	}

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		opchg_gpio.opchg_swtich1_gpio=di->opchg_swtich1_gpio;
		if(di->opchg_swtich2_gpio > 0 )
		{
		opchg_gpio.opchg_swtich2_gpio=di->opchg_swtich2_gpio;
		}
		opchg_gpio.opchg_reset_gpio=di->opchg_reset_gpio;
		opchg_gpio.opchg_clock_gpio=di->opchg_clock_gpio;
		opchg_gpio.opchg_data_gpio=di->opchg_data_gpio;

		rc =opchg_bq27541_gpio_pinctrl_init(di);
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}
int opchg_bq27541_gpio_pinctrl_init(struct opchg_bms_charger *di)
{
#ifdef OPPO_USE_FAST_CHARGER
    di->pinctrl = devm_pinctrl_get(di->dev);
    if (IS_ERR_OR_NULL(di->pinctrl)) {
            pr_err("%s:%d Getting pinctrl handle failed\n",
            __func__, __LINE__);
         return -EINVAL;
	}

	// set switch1 is active and switch2 is active
	if(get_PCB_Version()== HW_VERSION__10)
	{
	    di->gpio_switch1_act_switch2_act =
	        pinctrl_lookup_state(di->pinctrl, "switch1_act_switch3_act");
	    if (IS_ERR_OR_NULL(di->gpio_switch1_act_switch2_act)) {
	            pr_err("%s:%d Failed to get the active state pinctrl handle\n",
	            __func__, __LINE__);
	        return -EINVAL;
	    }

		// set switch1 is sleep and switch2 is sleep
	    di->gpio_switch1_sleep_switch2_sleep =
	        pinctrl_lookup_state(di->pinctrl, "switch1_sleep_switch3_sleep");
	    if (IS_ERR_OR_NULL(di->gpio_switch1_sleep_switch2_sleep)) {
	            pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
	            __func__, __LINE__);
	        return -EINVAL;
	    }
	}
	else
	{
		di->gpio_switch1_act_switch2_act =
	        pinctrl_lookup_state(di->pinctrl, "switch1_act_switch2_act");
	    if (IS_ERR_OR_NULL(di->gpio_switch1_act_switch2_act)) {
	            pr_err("%s:%d Failed to get the active state pinctrl handle\n",
	            __func__, __LINE__);
	        return -EINVAL;
	    }

		// set switch1 is sleep and switch2 is sleep
	    di->gpio_switch1_sleep_switch2_sleep =
	        pinctrl_lookup_state(di->pinctrl, "switch1_sleep_switch2_sleep");
	    if (IS_ERR_OR_NULL(di->gpio_switch1_sleep_switch2_sleep)) {
	            pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
	            __func__, __LINE__);
	        return -EINVAL;
	    }
	}
	// set switch1 is active and switch2 is sleep
    di->gpio_switch1_act_switch2_sleep =
        pinctrl_lookup_state(di->pinctrl, "switch1_act_switch2_sleep");
    if (IS_ERR_OR_NULL(di->gpio_switch1_act_switch2_sleep)) {
            pr_err("%s:%d Failed to get the state 2 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }

	// set switch1 is sleep and switch2 is active
    di->gpio_switch1_sleep_switch2_act =
        pinctrl_lookup_state(di->pinctrl, "switch1_sleep_switch2_act");
    if (IS_ERR_OR_NULL(di->gpio_switch1_sleep_switch2_act)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }

	// set clock is active
	di->gpio_clock_active =
        pinctrl_lookup_state(di->pinctrl, "clock_active");
    if (IS_ERR_OR_NULL(di->gpio_clock_active)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }

	// set clock is sleep
	di->gpio_clock_sleep =
        pinctrl_lookup_state(di->pinctrl, "clock_sleep");
    if (IS_ERR_OR_NULL(di->gpio_clock_sleep)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }

	// set clock is active
	di->gpio_data_active =
        pinctrl_lookup_state(di->pinctrl, "data_active");
    if (IS_ERR_OR_NULL(di->gpio_data_active)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }

	// set clock is sleep
	di->gpio_data_sleep =
        pinctrl_lookup_state(di->pinctrl, "data_sleep");
    if (IS_ERR_OR_NULL(di->gpio_data_sleep)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }
	// set reset is atcive
	di->gpio_reset_active =
        pinctrl_lookup_state(di->pinctrl, "reset_active");
    if (IS_ERR_OR_NULL(di->gpio_reset_active)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }
	// set reset is sleep
	di->gpio_reset_sleep =
        pinctrl_lookup_state(di->pinctrl, "reset_sleep");
    if (IS_ERR_OR_NULL(di->gpio_reset_sleep)) {
            pr_err("%s:%d Failed to get the state 3 pinctrl handle\n",
            __func__, __LINE__);
        return -EINVAL;
    }
    return 0;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

int opchg_set_clock_active(struct opchg_bms_charger  *di)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc=opchg_set_gpio_dir_output(di->opchg_clock_gpio,0);	// out 0
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc=pinctrl_select_state(di->pinctrl,di->gpio_clock_sleep);	// PULL_down
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}
int opchg_set_clock_sleep(struct opchg_bms_charger *di)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc=opchg_set_gpio_dir_output(di->opchg_clock_gpio,1);	// out 1
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc=pinctrl_select_state(di->pinctrl,di->gpio_clock_active);// PULL_up
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

int opchg_set_data_active(struct opchg_bms_charger *di)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc=opchg_set_gpio_dir_intput(di->opchg_data_gpio);	// in
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc=pinctrl_select_state(di->pinctrl,di->gpio_data_active);	// no_PULL
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
	}
	//rc=opchg_set_gpio_val(di->opchg_data_gpio,1);	// in
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

int opchg_set_data_sleep(struct opchg_bms_charger *di)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc=pinctrl_select_state(di->pinctrl,di->gpio_data_sleep);// PULL_down
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
		rc=opchg_set_gpio_dir_output(di->opchg_data_gpio,0);	// out 1
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}
int opchg_set_reset_active(struct opchg_bms_charger  *di)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if (opchg_chip->updating_fw_flag) {
		printk(KERN_ERR "%s: updating_fw, return.\n", __func__);
		return rc;
	}

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		printk(KERN_ERR "%s: reset MCU\n", __func__);
		rc=opchg_set_gpio_dir_output(di->opchg_reset_gpio,1);	// out 1
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc=pinctrl_select_state(di->pinctrl,di->gpio_reset_active);	// PULL_up
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
		rc=opchg_set_gpio_val(di->opchg_reset_gpio,1);
		wait_ms(10);
		rc=opchg_set_gpio_val(di->opchg_reset_gpio,0);
		//wait_ms(10);
		msleep(200);
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

int opchg_set_switch_fast_charger(void)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc = opchg_set_gpio_dir_output(opchg_gpio.opchg_swtich1_gpio,1);	// out 1
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		rc = opchg_set_gpio_dir_output(opchg_gpio.opchg_swtich2_gpio,1);	// out 1
		}
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		if (!IS_ERR_OR_NULL(bq27541_di->pinctrl) && !IS_ERR_OR_NULL(bq27541_di->gpio_switch1_act_switch2_act))
		rc = pinctrl_select_state(bq27541_di->pinctrl,bq27541_di->gpio_switch1_act_switch2_act);	// PULL_up
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
		rc = opchg_set_gpio_val(opchg_gpio.opchg_swtich1_gpio,1);
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		rc = opchg_set_gpio_val(opchg_gpio.opchg_swtich2_gpio,1);
		}
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

int opchg_set_switch_normal_charger(void)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc = opchg_set_gpio_dir_output(opchg_gpio.opchg_swtich1_gpio,0);	// in 0
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		rc = opchg_set_gpio_dir_output(opchg_gpio.opchg_swtich2_gpio,1);	// out 1
		}
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc = pinctrl_select_state(bq27541_di->pinctrl,bq27541_di->gpio_switch1_sleep_switch2_sleep);	// PULL_down
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
		rc = opchg_set_gpio_val(opchg_gpio.opchg_swtich1_gpio,0);
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		rc = opchg_set_gpio_val(opchg_gpio.opchg_swtich2_gpio,1);
		}
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}
int opchg_set_switch_earphone(void)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		rc = opchg_set_gpio_dir_output(opchg_gpio.opchg_swtich1_gpio,1);
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		rc = opchg_set_gpio_dir_output(opchg_gpio.opchg_swtich2_gpio,0);
		}
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_lock(&bq27541_di->work_irq_lock);
		#endif
		rc = pinctrl_select_state(bq27541_di->pinctrl,bq27541_di->gpio_switch1_sleep_switch2_sleep);	// PULL_down
		#ifndef OPCHG_VOOC_WATCHDOG
		mutex_unlock(&bq27541_di->work_irq_lock);
		#endif
		rc = opchg_set_gpio_val(opchg_gpio.opchg_swtich1_gpio,1);
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		rc = opchg_set_gpio_val(opchg_gpio.opchg_swtich2_gpio,0);
		}
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

int opchg_set_switch_mode(u8 mode)
{
#ifdef OPPO_USE_FAST_CHARGER
	int rc=0;

	// check GPIO23 and GPIO38 is  undeclared, prevent the  Invalid use for earphone
	//if(opchg_pinctrl_chip == NULL)
	if((bq27541_di == NULL)||(opchg_chip == NULL))
	{
		pr_err("%s GPIO23 and GPIO38 is no probe\n",__func__);
		return -1;
	}

	// GPIO23 and GPIO38 is  declared
    switch(mode) {
        case VOOC_CHARGER_MODE:	//11
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				if(opchg_chip->opchg_earphone_enable ==false)
				{
					rc=opchg_set_switch_fast_charger();
				}
			}
			break;

        case HEADPHONE_MODE:		//10
		if(is_project(OPPO_14005))
			{
				rc=opchg_set_switch_earphone();
            }
			break;

        case NORMAL_CHARGER_MODE:	//01
        default:
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				if(opchg_chip->opchg_earphone_enable ==false)
				{
					rc=opchg_set_switch_normal_charger();
				}
            }
			break;
    }

	if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		if(opchg_gpio.opchg_swtich2_gpio > 0 )
		{
		pr_err("%s charge_mode,rc:%d,GPIO%d:%d,GPIO%d:%d\n",__func__,rc,opchg_gpio.opchg_swtich1_gpio,opchg_get_gpio_val(opchg_gpio.opchg_swtich1_gpio),opchg_gpio.opchg_swtich2_gpio,opchg_get_gpio_val(opchg_gpio.opchg_swtich2_gpio));
		}
		else
		{
		pr_err("%s charge_mode,rc:%d,GPIO%d:%d\n",__func__,rc,opchg_gpio.opchg_swtich1_gpio,opchg_get_gpio_val(opchg_gpio.opchg_swtich1_gpio));
		}
	}
	return rc;
#else
	int rc=0;

	return rc;
#endif/*#ifdef OPPO_USE_FAST_CHARGER*/
}

#define MAX_RETRY_COUNT	5
static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct opchg_bms_charger *di;
	struct bq27541_access_methods *bus;
	int retval = 0;

	char *name;
	int num;

	pr_debug("opcharger bq27541 start==================================\n");
	/**/
    /* i2c pull up Regulator configuration */
	#if 0
	chip->vcc_i2c = regulator_get(&client->dev, "vcc_i2c_opfastcharger");
	if (IS_ERR(chip->vcc_i2c)) {
	        dev_err(&client->dev, "%s: Failed to get i2c regulator\n", __func__);
	        retval = PTR_ERR(chip->vcc_i2c);
	        return -1;//retval;
	}
	if (regulator_count_voltages(chip->vcc_i2c) > 0) {
	        retval = regulator_set_voltage(chip->vcc_i2c, OPCHARGER_I2C_VTG_MIN_UV, OPCHARGER_I2C_VTG_MAX_UV);
	        if (retval) {
	            dev_err(&client->dev, "reg set i2c vtg failed retval =%d\n", retval);
	            goto err_set_vtg_i2c;
	        }
	}

	retval = regulator_enable(chip->vcc_i2c);
	if (retval) {
	        dev_err(&client->dev,"Regulator vcc_i2c enable failed " "rc=%d\n", retval);
	        return retval;
	}
	#endif

	/**/
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s pic16F_probe,i2c_func error\n",__func__);
		return -ENODEV;
	}

	/**/
	/*
	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}
	di->client = client;	//pic16F_client = client;
	di->dev = &client->dev;
	*/

	/* Get new ID for the new battery device */
	#if 0
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	#else
	retval =__idr_pre_get(&battery_id, GFP_KERNEL);
	#endif

	if (retval == 0)
		return -ENOMEM;

	mutex_lock(&battery_mutex);
	#if 0
	retval = idr_get_new(&battery_id, client, &num);
	#else
	retval = __idr_get_new_above(&battery_id, client,0,&num);
	//retval = oppo_idr_get_new(&battery_id, client, &num);
	#endif

	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}


	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	// chang mod timer to delay tims for dengnw in 20141221
	#ifndef OPCHG_VOOC_WATCHDOG
	mutex_init(&di->work_irq_lock);
	#endif

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	di->bus = bus;
	di->client = client;
	di->temp_pre = 0;
	di->rtc_refresh_time = 0;
	di->alow_reading = true;
	di->fast_chg_ing = false;
	di->fast_low_temp_full = false;
	di->retry_count = MAX_RETRY_COUNT;
	atomic_set(&di->suspended, 0);
	//opchg_pinctrl_chip =di;
	bq27541_di = di;


	/**/
#ifdef OPPO_USE_FAST_CHARGER
	retval=opchg_bq27541_parse_dt(di);
	retval =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
#endif

#ifdef CONFIG_BQ27541_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
			 &fs_attr_group);
		if (retval)
			goto batt_failed_4;
	} else
		goto batt_failed_4;
#endif

	if (retval) {
		dev_err(&client->dev, "failed to setup bq27541\n");
		goto batt_failed_4;
	}

	if (retval) {
		dev_err(&client->dev, "failed to powerup bq27541\n");
		goto batt_failed_4;
	}

	spin_lock_init(&lock);

	INIT_WORK(&di->counter, bq27541_coulomb_counter_work);
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
	schedule_delayed_work(&di->hw_config, 0);

	/* OPPO 2013-12-22 wangjc add for fastchg*/
#ifdef OPPO_USE_FAST_CHARGER
if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
{
	// chang mod timer to delay tims for dengnw in 20141221
	#ifdef OPCHG_VOOC_WATCHDOG
	init_timer(&di->watchdog);
	di->watchdog.data = (unsigned long)di;
	di->watchdog.function = di_watchdog;
	#else
	INIT_DELAYED_WORK(&di->watchdog_delayed_work, di_watchdog);
	#endif

	wake_lock_init(&di->fastchg_wake_lock,
		WAKE_LOCK_SUSPEND, "fastcg_wake_lock");
	INIT_WORK(&di->fastcg_work,fastcg_work_func);

	retval= opchg_set_data_active(di);
	di->irq = gpio_to_irq(di->opchg_data_gpio);
	retval = request_irq(di->irq, irq_rx_handler, IRQF_TRIGGER_RISING, "mcu_data", di);	//0X01:rising edge,0x02:falling edge
	if(retval < 0) {
		pr_err("%s request ap rx irq failed.\n", __func__);
	}
}
#endif
	/* OPPO 2013-12-22 wangjc add end*/
	pr_debug("opcharger bq27541 end==================================\n");

	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct opchg_bms_charger *di = i2c_get_clientdata(client);

	qpnp_battery_gauge_unregister(&bq27541_batt_gauge);
	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_BQ27411_SUBCMD_DISABLE_IT);

	// chang mod timer to delay tims for dengnw in 20141221
	#ifdef OPCHG_VOOC_WATCHDOG
	cancel_delayed_work_sync(&di->hw_config);
	#else
	cancel_delayed_work(&di->hw_config);
	#endif
	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

/* OPPO 2013-11-19 wangjingchun Add use to get rtc times for other driver */
int msmrtc_alarm_read_time(struct rtc_time *tm)
{
	struct rtc_device *alarm_rtc_dev;
	int ret=0;

#ifndef CONFIG_MACH_OPPO
/* jingchun.wang@Onlinerd.Driver, 2014/02/28  Delete for sovle alarm can't sleep */
	wake_lock(&alarm_rtc_wake_lock);
#endif /*CCONFIG_MACH_OPPO*/

	alarm_rtc_dev = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (alarm_rtc_dev == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	ret = rtc_read_time(alarm_rtc_dev, tm);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		goto err;
	}

#ifndef CONFIG_MACH_OPPO
/* jingchun.wang@Onlinerd.Driver, 2014/02/28  Delete for sovle alarm can't sleep */
	wake_unlock(&alarm_rtc_wake_lock);
#endif /*CONFIG_MACH_OPPO*/
	return 0;
err:
	pr_err("%s: rtc alarm will lost!", __func__);
#ifndef CONFIG_MACH_OPPO
/* jingchun.wang@Onlinerd.Driver, 2014/02/28  Delete for sovle alarm can't sleep */
	wake_unlock(&alarm_rtc_wake_lock);
#endif /*CONFIG_MACH_OPPO*/
	return -1;

}

static int bq27541_battery_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret=0;
	struct rtc_time	rtc_suspend_rtc_time;
	struct opchg_bms_charger *di = i2c_get_clientdata(client);

	atomic_set(&di->suspended, 1);
	ret = msmrtc_alarm_read_time(&rtc_suspend_rtc_time);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		return 0;
	}
	rtc_tm_to_time(&rtc_suspend_rtc_time, &di->rtc_suspend_time);

	return 0;
}

/*1 minute*/
#define RESUME_TIME  1*60
#define REFRESH_TIME  10*60
static int bq27541_battery_resume(struct i2c_client *client)
{
	int ret=0;
	struct rtc_time	rtc_resume_rtc_time;
	struct opchg_bms_charger *di = i2c_get_clientdata(client);
	int suspend_time;

	atomic_set(&di->suspended, 0);
	ret = msmrtc_alarm_read_time(&rtc_resume_rtc_time);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		return 0;
	}
	rtc_tm_to_time(&rtc_resume_rtc_time, &di->rtc_resume_time);

	suspend_time = di->rtc_resume_time - di->rtc_suspend_time;
	/*update pre capacity when sleep time more than 1minutes or refresh time more than 10min*/
	if(opchg_chip != NULL && qpnp_batt_gauge && qpnp_batt_gauge->get_battery_soc)
	{
		pr_debug("oppo_check_rtc_time suspend_time=%d\n",suspend_time);
		opchg_chip->bat_volt_check_point = bq27541_battery_soc(bq27541_di, suspend_time);
		if (opchg_chip->bat_volt_check_point == 0 && bq27541_battery_voltage(bq27541_di) > LOW_POWER_VOLTAGE_3400MV)
			opchg_chip->bat_volt_check_point = 1;
		power_supply_changed(&opchg_chip->batt_psy);
	}

	return 0;
}

#if 0
static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);
#endif

/* OPPO 2014-11-18 sjc Add begin for 14021 */
#define CONTROL_CMD				0x00
#define CONTROL_STATUS				0x00
#define SEAL_POLLING_RETRY_LIMIT	100
#define BQ27541_UNSEAL_KEY			0x11151986
#define BQ27411_UNSEAL_KEY			0x80008000
#define BQ27541_RESET_SUBCMD				0x0041
#define BQ27411_RESET_SUBCMD				0x0042
#define SEAL_SUBCMD					0x0020

static void control_cmd_write(struct opchg_bms_charger *di, u16 cmd)
{
	int value;

	//dev_dbg(di->dev, "%s: %04x\n", __FUNCTION__, cmd);
	if (di->device_type == DEVICE_BQ27541)
	{
		bq27541_cntl_cmd(di, BQ27541_RESET_SUBCMD);
		msleep(10);
		bq27541_read(CONTROL_STATUS, &value, 0, di);
	}
	else if (di->device_type == DEVICE_BQ27411)
	{
		bq27541_cntl_cmd(di, BQ27411_RESET_SUBCMD);
		msleep(10);
		bq27541_read(CONTROL_STATUS, &value, 0, di);
	}
	printk(KERN_ERR "bq27541 CONTROL_STATUS: 0x%x\n", value);
}
static int sealed(struct opchg_bms_charger *di)
{
	//return control_cmd_read(di, CONTROL_STATUS) & (1 << 13);
	int value = 0;

	bq27541_cntl_cmd(di,CONTROL_STATUS);
	msleep(10);
	bq27541_read(CONTROL_STATUS, &value, 0, di);
	pr_err("%s REG_CNTL: 0x%x\n", __func__, value);

	if (di->device_type == DEVICE_BQ27541)
		return value & BIT(14);
	else if (di->device_type == DEVICE_BQ27411)
		return value & BIT(13);
	else
		return 1;
}

static int seal(struct opchg_bms_charger *di)
{
	int i = 0;

	if(sealed(di)){
		pr_err("bq27541/27411 sealed,return\n");
		return 1;
	}
	bq27541_cntl_cmd(di,SEAL_SUBCMD);
	msleep(10);
	for(i = 0;i < SEAL_POLLING_RETRY_LIMIT;i++){
		if (sealed(di))
			return 1;
		msleep(10);
	}
	return 0;
}
static int unseal(struct opchg_bms_charger *di, u32 key)
{
	int i = 0;

	if (!sealed(di))
		goto out;

	if(di->device_type == DEVICE_BQ27541){
		//bq27541_write(CONTROL_CMD, key & 0xFFFF, false, di);
		bq27541_cntl_cmd(di, 0x1115);
		msleep(10);
		//bq27541_write(CONTROL_CMD, (key & 0xFFFF0000) >> 16, false, di);
		bq27541_cntl_cmd(di, 0x1986);
		msleep(10);
	} else if(di->device_type == DEVICE_BQ27411){
		//bq27541_write(CONTROL_CMD, key & 0xFFFF, false, di);
		bq27541_cntl_cmd(di, 0x8000);
		msleep(10);
		//bq27541_write(CONTROL_CMD, (key & 0xFFFF0000) >> 16, false, di);
		bq27541_cntl_cmd(di, 0x8000);
		msleep(10);
	}
	bq27541_cntl_cmd(di, 0xffff);
	msleep(10);
	bq27541_cntl_cmd(di, 0xffff);
	msleep(10);

	while (i < SEAL_POLLING_RETRY_LIMIT) {
		i++;
		if (!sealed(di))
			break;
		msleep(10);
	}

out:
	pr_err("bq27541 %s: i=%d\n", __func__, i);

	if ( i == SEAL_POLLING_RETRY_LIMIT) {
		pr_err("bq27541 %s failed\n", __func__);
		return 0;
	} else {
		return 1;
	}
}

static void bq27541_reset(struct i2c_client *client)
{
	struct opchg_bms_charger *di = i2c_get_clientdata(client);

	if (bq27541_get_battery_mvolts() <= 3250 * 1000
			&& bq27541_get_battery_mvolts() > 2500 * 1000
			&& bq27541_get_battery_soc() == 0
			&& bq27541_get_battery_temperature() > 150) {
		if (!unseal(di, BQ27541_UNSEAL_KEY)) {
			pr_err( "bq27541 unseal fail !\n");
			return;
		}
		pr_err( "bq27541 unseal OK !\n");

		if(di->device_type == DEVICE_BQ27541){
			control_cmd_write(di, BQ27541_RESET_SUBCMD);
		}
		else if(di->device_type == DEVICE_BQ27411)
		{
			control_cmd_write(di, BQ27411_RESET_SUBCMD);
		}

		if (di->device_type == DEVICE_BQ27411){
			if (!seal(di))
				pr_err("bq27411 seal fail\n");
		}
	}
	return;
}

static const struct of_device_id bq27541_match[] = {
	{ .compatible = "ti,bq27541-battery" },
	{ },
};

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27541_id);


static struct i2c_driver bq27541_battery_driver = {
	.driver		= {
		.name = "bq27541-battery",
		.owner	= THIS_MODULE,
		.of_match_table = bq27541_match,
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
	.shutdown	= bq27541_reset,
	.suspend	= bq27541_battery_suspend ,
	.resume		= bq27541_battery_resume,
	.id_table	= bq27541_id,
};
module_i2c_driver(bq27541_battery_driver);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
