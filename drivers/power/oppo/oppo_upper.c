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

#define OPPO_UPPER_PAR
#include "oppo_inc.h"


/* add supplied to "bms" function */
static char *pm_batt_supplied_to[] = {
    "bms",
};
static char *pm_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property opchg_battery_properties[] = {
	POWER_SUPPLY_PROP_AUTHENTICATE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,

	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_STATU,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
	POWER_SUPPLY_PROP_HEALTH,

	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_MODEL_NAME,

	POWER_SUPPLY_PROP_FAST_CHARGE,//wangjc add for fast charger
	POWER_SUPPLY_PROP_FAST_CHARGE_PROJECT,//wangjc add for fast charger project sign
	POWER_SUPPLY_PROP_BATTERY_FCC,			//dengnw add for battery fcc
	POWER_SUPPLY_PROP_BATTERY_SOH,			//dengnw add for battery soh
	POWER_SUPPLY_PROP_BATTERY_CC,			//dengnw add for battery cc
	POWER_SUPPLY_PROP_BATTERY_CHARGER_ENABLE,//dengnw add for charger enable for MMI test
	POWER_SUPPLY_PROP_BATTERY_NOTIFY,		//dengnw add for charging status
	POWER_SUPPLY_PROP_POWER_OFF,			//lfc add for power_off when vbatt is low
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static void opchg_external_power_changed(struct power_supply *psy)
{
    struct opchg_charger *chip = container_of(psy, struct opchg_charger, batt_psy);
    union power_supply_propval prop = {0,};
    int rc, current_limit = 0, online = 0;

    if (chip->bms_psy_name) {
        chip->bms_psy = power_supply_get_by_name((char *)chip->bms_psy_name);
    }

    rc = chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_ONLINE, &prop);
    if (rc) {
        dev_err(chip->dev, "Couldn't read USB online property, rc=%d\n", rc);
    }
	else {
        online = prop.intval;
    }

    rc = chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
    if (rc) {
        dev_err(chip->dev, "Couldn't read USB current_max property, rc=%d\n", rc);
    }
	else {
        current_limit = prop.intval / 1000;
    }

    if(current_limit > chip->limit_current_max_ma) {
        current_limit = chip->limit_current_max_ma;
    }

	//lfc add for charger_ovp
	chip->charger_vol = opchg_get_prop_charger_voltage_now(chip);
	opchg_get_charger_ov_status(chip);
	if(chip->charger_ov_status == true){
		dev_err(chip->dev,"%s charger-ovp,return \n",__func__);
		return ;
	}

    opchg_set_enable_volatile_writes(chip);
	opchg_config_input_chg_current(chip, INPUT_CURRENT_LCD, chip->limit_current_max_ma);
	opchg_config_input_chg_current(chip, INPUT_CURRENT_CAMERA, chip->limit_current_max_ma);
    opchg_config_input_chg_current(chip, INPUT_CURRENT_BY_POWER, current_limit);

	if(is_project(OPPO_15109))
		opchg_set_input_chg_current(chip, chip->max_input_current[INPUT_CURRENT_BY_POWER], false);
	else
	opchg_set_input_chg_current(chip, chip->max_input_current[INPUT_CURRENT_MIN], false);

	if ((chip->multiple_test == 1) && (current_limit >= 500)) {
		opchg_config_suspend_enable(chip, FACTORY_ENABLE, 1);
	}

    opchg_config_over_time(chip, current_limit);//opchg_set_complete_charge_timeout(chip);
	dev_dbg(chip->dev, "%s set charger input current=%d,online = %d, current_limit = %d\n", __func__,chip->max_input_current[INPUT_CURRENT_MIN], online, current_limit);

    opchg_check_status(chip);
	opchg_set_status(chip, true);//opchg_set_status(chip, false);
	if(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_NORMAL)
		opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);

    power_supply_changed(&chip->batt_psy);
}

void opchg_property_config(struct opchg_charger *chip)
{
    chip->batt_psy.name = "battery";
    chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
    chip->batt_psy.get_property = opchg_battery_get_property;
    chip->batt_psy.set_property = opchg_battery_set_property;
    chip->batt_psy.property_is_writeable = opchg_batt_property_is_writeable;
    chip->batt_psy.properties = opchg_battery_properties;
    chip->batt_psy.num_properties = ARRAY_SIZE(opchg_battery_properties);
    chip->batt_psy.external_power_changed = opchg_external_power_changed;
    chip->batt_psy.supplied_to = pm_batt_supplied_to;
    chip->batt_psy.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);
}
void opchg_dc_property_config(struct opchg_charger *chip)
{
	chip->dc_psy.name = "qpnp-dc";
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->dc_psy.get_property = qpnp_power_get_property_mains;
	chip->dc_psy.set_property = qpnp_dc_power_set_property;
	chip->dc_psy.property_is_writeable =qpnp_dc_property_is_writeable;
	chip->dc_psy.properties = pm_power_props_mains;
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
	chip->dc_psy.supplied_to = pm_power_supplied_to;
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);

}

static int show_cnfg_regs(struct seq_file *m, void *data)
{
    struct opchg_charger *chip = m->private;
    int rc;
    u8 reg;
    u8 addr;

    for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
        rc = opchg_read_reg(chip, addr, &reg);
        if (!rc) {
            seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
        }
    }

    return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
    struct opchg_charger *chip = inode->i_private;

    return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
    .owner		= THIS_MODULE,
    .open		= cnfg_debugfs_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release	= single_release,
};

static int show_cmd_regs(struct seq_file *m, void *data)
{
    struct opchg_charger *chip = m->private;
    int rc;
    u8 reg;
    u8 addr;

    for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
        rc = opchg_read_reg(chip, addr, &reg);
        if (!rc) {
            seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
        }
    }

    return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
    struct opchg_charger *chip = inode->i_private;

    return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
    .owner		= THIS_MODULE,
    .open		= cmd_debugfs_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release	= single_release,
};

static int show_status_regs(struct seq_file *m, void *data)
{
    struct opchg_charger *chip = m->private;
    int rc;
    u8 reg;
    u8 addr;

    for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
        rc = opchg_read_reg(chip, addr, &reg);
        if (!rc) {
        seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
        }
    }

    return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
    struct opchg_charger *chip = inode->i_private;

    return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
    .owner		= THIS_MODULE,
    .open		= status_debugfs_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
    struct opchg_charger *chip = data;
    int rc;
    u8 temp;

    rc = opchg_read_reg(chip, chip->peek_poke_address, &temp);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't read reg %x rc = %d\n", chip->peek_poke_address, rc);
        return -EAGAIN;
    }
    *val = temp;

    return 0;
}

static int set_reg(void *data, u64 val)
{
    struct opchg_charger *chip = data;
    int rc;
    u8 temp;

    temp = (u8) val;
    rc = opchg_write_reg(chip, chip->peek_poke_address, temp);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't write 0x%02x to 0x%02x rc= %d\n", chip->peek_poke_address, temp, rc);
        return -EAGAIN;
    }

    return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

void opchg_debugfs_create(struct opchg_charger *chip)
{
    int rc = 0;

    chip->debug_root = debugfs_create_dir("opchg", NULL);//debugfs_create_dir("smb358", NULL);
    if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
    }

    if (chip->debug_root) {
        struct dentry *ent;

        ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
                            chip->debug_root, chip,
                            &cnfg_debugfs_ops);
        if (!ent) {
            dev_err(chip->dev, "Couldn't create cnfg debug file rc = %d\n", rc);
        }

        ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
                            chip->debug_root, chip,
                            &status_debugfs_ops);
        if (!ent) {
            dev_err(chip->dev, "Couldn't create status debug file rc = %d\n", rc);
        }

        ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
                            chip->debug_root, chip,
                            &cmd_debugfs_ops);
        if (!ent) {
            dev_err(chip->dev, "Couldn't create cmd debug file rc = %d\n", rc);
        }

        ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
                            chip->debug_root,
                            &(chip->peek_poke_address));
        if (!ent) {
            dev_err(chip->dev, "Couldn't create address debug file rc = %d\n", rc);
        }

        ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
                            chip->debug_root, chip,
                            &poke_poke_debug_ops);
        if (!ent) {
			dev_err(chip->dev, "Couldn't create data debug file rc = %d\n", rc);
        }
	}
}

int opchg_batt_property_is_writeable(struct power_supply *psy,
                            enum power_supply_property psp)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
    case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_BATTERY_CHARGER_ENABLE:
        return 1;
    default:
        break;
    }

    return 0;
}

int opchg_battery_set_property(struct power_supply *psy,
                            enum power_supply_property prop,
                            const union power_supply_propval *val)
{
	int rc;
    struct opchg_charger *chip = container_of(psy,
                            struct opchg_charger, batt_psy);

    switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
				return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
				rc = smb358_charging_disable(chip, SOC, true);
				if (rc < 0) {
					dev_err(chip->dev,"Couldn't set charging disable rc = %d\n",rc);
				} else {
					chip->batt_full = true;
					dev_dbg(chip->dev, "status = FULL, batt_full = %d\n",chip->batt_full);
				}
				break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
				chip->batt_full = false;
				power_supply_changed(&chip->batt_psy);
				dev_dbg(chip->dev, "status = DISCHARGING, batt_full = %d\n",chip->batt_full);
				break;
		case POWER_SUPPLY_STATUS_CHARGING:
				rc = smb358_charging_disable(chip, SOC, false);
				if (rc < 0) {
					dev_err(chip->dev,"Couldn't set charging disable rc = %d\n",rc);
				} else {
					chip->batt_full = false;
					dev_dbg(chip->dev, "status = CHARGING, batt_full = %d\n",chip->batt_full);
				}
				break;
		default:
				return -EINVAL;
		}
		break;
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        opchg_config_charging_disable(chip, USER_DISABLE, !val->intval);//smb358_charging(chip, val->intval);
        break;

    case POWER_SUPPLY_PROP_CAPACITY:
        chip->fake_battery_soc = val->intval;
        power_supply_changed(&chip->batt_psy);
        break;

    case POWER_SUPPLY_PROP_BATTERY_CHARGER_ENABLE:
		chip->is_factory_mode= val->intval;
		if(chip->is_factory_mode == true)
		{
				opchg_config_charging_disable(chip, FACTORY_MODE_DISABLE, 0);
		}
		else
		{
			if(is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				opchg_set_switch_mode(NORMAL_CHARGER_MODE);
			}
			opchg_config_charging_disable(chip, FACTORY_MODE_DISABLE, 1);
		}
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

int opchg_battery_get_property(struct power_supply *psy,
                            enum power_supply_property prop,
                            union power_supply_propval *val)
{
    struct opchg_charger *chip = container_of(psy,
                        struct opchg_charger, batt_psy);

    switch (prop) {
		case POWER_SUPPLY_PROP_AUTHENTICATE:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->bat_exist;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
	        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	        break;

		case POWER_SUPPLY_PROP_CHARGE_NOW:
	        val->intval = chip->charger_vol;
			//val->intval = opchg_get_prop_charger_voltage_now(chip);
	        break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	        val->intval = chip->bat_instant_vol;
	        //val->intval = opchg_get_prop_battery_voltage_now(chip);
	        break;
		case POWER_SUPPLY_PROP_TEMP:
	        val->intval = chip->temperature;
	        //val->intval = opchg_get_prop_batt_temp(chip);
	        break;
		case 	POWER_SUPPLY_PROP_TEMP_STATU:
	        val->intval = chip->charging_opchg_temp_statu;
	        break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
	        val->intval = chip->charging_current;
	        //val->intval = opchg_get_prop_current_now(chip);
	        break;
	    case POWER_SUPPLY_PROP_CAPACITY:
			if(chip->bat_volt_check_point > 100)
			{
				val->intval = 100;
			}
			else
			{
				val->intval =chip->bat_volt_check_point;
			}
	        break;

	    case POWER_SUPPLY_PROP_CHARGE_TIMEOUT:
	        val->intval = (int)chip->charging_time_out;
	        break;
	    case POWER_SUPPLY_PROP_HEALTH:
			#if 0
			if(is_project(OPPO_14005)) {
				if(chip->bat_temp_status ==POWER_SUPPLY_HEALTH_WARM)
				{
					val->intval = POWER_SUPPLY_HEALTH_GOOD;
				}
				else
				{
					val->intval = chip->bat_temp_status;
				}
			}
			else
			{
		        val->intval = chip->bat_temp_status;
			}
			#else
			val->intval = chip->bat_temp_status;
			#endif
	        break;

        case POWER_SUPPLY_PROP_ONLINE:
			// set usb charging sign
			if((chip->chg_present) && (qpnp_charger_type_get(chip) == POWER_SUPPLY_TYPE_USB_DCP))
			{
				val->intval = 0;
			}
			else
			{
			val->intval = chip->chg_present;
			}
			#ifdef OPCHARGER_DEBUG_ENABLE
			pr_debug("oppo_debug usb charging sign = %d\n",val->intval);
			#endif
	        break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	        if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022)) {
	            if(opchg_get_prop_fast_chg_started(chip) == true) {
	                val->intval = 1;
	            }
	            else {
	                val->intval = chip->is_charging;
	            }
	        }
	        else {
	            val->intval = chip->is_charging;
	        }
	        break;
	    case POWER_SUPPLY_PROP_STATUS:
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022)) {
				if(opchg_get_prop_fast_chg_started(chip) == true) {
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				}
				else {
					if((opchg_get_prop_fast_switch_to_normal(chip) == true)&&(chip->g_is_vooc_changed == false))
					{
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
					}
					else
					{
						val->intval = chip->bat_status;
					}
				}
			}
			else {
				val->intval = chip->bat_status;
			}
			#ifdef OPCHARGER_DEBUG_ENABLE
			pr_debug("oppo_debug  bat_status= %d vooc_start_step= %d\r\n",val->intval,vooc_start_step);
			#endif
	        break;
	    case POWER_SUPPLY_PROP_CHARGE_TYPE:
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022)) {
			    if(opchg_get_prop_fast_chg_started(chip) == true) {	// in fast charging
			        val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			    }
			    else {										// in normal  charging
				    val->intval = chip->bat_charging_state;
				}
			}
			else {
			    val->intval = chip->bat_charging_state;
			}
	        break;

	    case POWER_SUPPLY_PROP_MODEL_NAME:
	        val->strval = "OPCHARGER";//"SMB358";
	        break;


		case POWER_SUPPLY_PROP_FAST_CHARGE://wangjc add for fast charger
			#ifdef OPPO_USE_FAST_CHARGER /* OPPO 2013-12-22 wangjc add for fastchg*/
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				chip->fastcharger =opchg_get_prop_fast_chg_started(chip);
				if(chip->fastcharger == 0) {
					//if(chip->batt_full == true)
					if((chip->bat_status == POWER_SUPPLY_STATUS_FULL)|| (chip->batt_hot == true)|| (chip->batt_cold == true)
						|| (chip->batt_voltage_over == true)||(chip->charger_ov_status == true) || (chip->charging_time_out == true))
					{
						chip->fastcharger = 0;
					}
					else if((opchg_get_prop_fast_switch_to_normal(chip) == true) ||
						(opchg_get_prop_fast_normal_to_warm(chip) == true) ||
						(opchg_get_fast_low_temp_full(chip) == true)) {
						chip->fastcharger = 1;
					}
				}
			}
			else
			{
				chip->fastcharger =0;
			}
			#else
			chip->fastcharger =0;
			#endif
			val->intval = chip->fastcharger;
			break;
		case POWER_SUPPLY_PROP_FAST_CHARGE_PROJECT://wangjc add for fast charger project sign
			val->intval = chip->fast_charge_project;
			break;

		case POWER_SUPPLY_PROP_BATTERY_FCC:			//dengnw add for battery fcc
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				if(bq27541_di == NULL)
				{
					val->intval = 100;
				}
				else
				{
					//val->intval = bq27541_di->full_charge_soc;
					val->intval = bq27541_di->fcc_pre;
				}
			}
			else
			{
				val->intval = 100;
			}
			break;
		case POWER_SUPPLY_PROP_BATTERY_SOH:			//dengnw add for battery soh
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				if(bq27541_di == NULL)
				{
					val->intval = 100;
				}
				else
				{
					//val->intval = bq27541_di->state_of_health;
					val->intval = bq27541_di->soh_pre;
				}
			}
			else
			{
				val->intval = 100;
			}
			break;
		case POWER_SUPPLY_PROP_BATTERY_CC:			//dengnw add for battery cc
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
			{
				if(bq27541_di == NULL)
				{
					val->intval = 100;
				}
				else
				{
					//val->intval = bq27541_di->state_of_health;
					val->intval = bq27541_di->cc_pre;
				}
			}
			else
			{
				val->intval = 100;
			}
			break;
		case 	POWER_SUPPLY_PROP_BATTERY_CHARGER_ENABLE:
			val->intval = chip->is_factory_mode;
			break;

		case POWER_SUPPLY_PROP_BATTERY_NOTIFY:
			val->intval = chip->batterynotify;
			#ifdef OPCHARGER_DEBUG_ENABLE
			pr_debug("oppo_debug  batterynotify = 0x%x\r\n",chip->batterynotify);
			#endif
			break;

		case POWER_SUPPLY_PROP_POWER_OFF:
			val->intval = 0;
			break;

	    default:
	        return -EINVAL;
	}
	return 0;
}

int qpnp_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property prop,
				  union power_supply_propval *val)
{
	struct opchg_charger *chip = container_of(psy, struct opchg_charger,
								dc_psy);

	switch (prop)
	{

	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:

		if (chip->charging_disabled)
			return 0;
		if( is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022))
		{
			if((opchg_get_prop_fast_chg_started(chip) == true)
				|| (opchg_get_prop_fast_switch_to_normal(chip) == true)
				|| (opchg_get_fast_normal_to_warm(chip) == true)) {
				val->intval = 1;
			} else {
				if(qpnp_charger_type_get(chip) == POWER_SUPPLY_TYPE_USB_DCP)
					val->intval = 1;
				else
					val->intval = 0;
			}
		}
		else
		{
			/* jingchun.wang@Onlinerd.Driver, 2014/02/11  Modify for when no battery gauge present */
			if(qpnp_charger_type_get(chip) == POWER_SUPPLY_TYPE_USB_DCP)
				val->intval = 1;
			else
				val->intval = 0;
		}
		#ifdef OPCHARGER_DEBUG_ENABLE
		pr_debug("oppo_debug dc charging and fast charging sign = %d\n",val->intval);
		#endif
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
#ifdef OPPO_USE_FAST_CHARGER
		if (opchg_get_prop_fast_chg_started(chip)) {
			// report a hardcoded value of 5A, as that's the theoretical VOOC limit
			val->intval = 5000000;
		} else
#endif
		{
			val->intval = chip->max_input_current[INPUT_CURRENT_MIN] * 1000;
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}


int qpnp_dc_power_set_property(struct power_supply *psy,
				  enum power_supply_property prop,
				  const union power_supply_propval *val)
{
	//struct opchg_charger *chip = container_of(psy, struct opchg_charger,dc_psy);
	int rc = 0;

	switch (prop)
	{
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (!val->intval)
			break;

		#if 0
		rc = qpnp_chg_idcmax_set(chip, val->intval / 1000);
		if (rc) {
			pr_err("Error setting idcmax property %d\n", rc);
			return rc;
		}
		chip->maxinput_dc_ma = (val->intval / 1000);
		#endif
		break;

	default:
		return -EINVAL;
	}

	pr_debug("psy changed dc_psy\n");
	return rc;
}

int qpnp_dc_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}
