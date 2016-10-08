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

#define OPPO_SMB1357_PAR
#include "oppo_inc.h"


static int smb1357_per_chg_current[] = {
    100,    150,    200,    250,
    550,
};

static int smb1357_fast_chg_current[] = {
    300,    400,    450,    475,
    500,    550,    600,    650,
    700,    900,    950,    1000,
    1100,   1200,   1400,   1500,
    1600,   1800,   1850,   1880,
    1930,   1950,   1970,   2000,
    2050,   2100,   2300,   2400,
    2500,   2700,   2800,   3000,
};

static int smb1357_termin_chg_current[] = {
    50,     100,    150,    200,
    250,    300,    500,    600,
};

static int smb1357_usbin_input_current_limit[] = {
    300,    400,    450,    475,
    500,    550,    600,    650,
    700,    900,    950,    1000,
    1100,   1200,   1400,   1450,
    1500,   1600,   1800,   1850,
    1880,   1910,   1930,   1950,
    1970,   2000,   2050,   2100,
    2300,   2400,   2500,   3000,
};

static int smb1357_float_compensation_voltage[] = {
    0,      25,     50,     75,
    100,    125,    150,    175,
};

int smb1357_get_prop_fastcharger_type(struct opchg_charger *chip)
{
    int rc = 0;
    u8 reg = 0;

    rc = opchg_read_reg(chip, REG4D_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read REG4D_SMB1357_ADDRESS rc = %d\n", rc);
        return false;
    }

    if (reg & REG4D_SMB1357_STATUS_IDEV_HVDCP_SEL_A_BIT) {
        return true;
    }
    else {
        return false;
    }
}

int smb1357_get_prop_charge_type(struct opchg_charger *chip)
{
    int rc = 0;
    u8 reg = 0;

    rc = opchg_read_reg(chip, REG4A_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read REG4A_SMB1357_ADDRESS rc = %d\n", rc);
        return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
    }

    reg &= REG4A_SMB1357_STATUS_C_CHARGING_MASK;

	if ((reg == REG4A_SMB1357_STATUS_C_FAST_CHARGING) || (reg == REG4A_SMB1357_STATUS_C_TAPER_CHARGING)) {
        return POWER_SUPPLY_CHARGE_TYPE_FAST;
    }
    else if (reg == REG4A_SMB1357_STATUS_C_PRE_CHARGING) {
        return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
    }
    else {
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }
}

int smb1357_get_prop_batt_status(struct opchg_charger *chip)
{
    int rc = 0;
    u8 reg = 0;

    rc = opchg_read_reg(chip, REG4A_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read REG4A_SMB1357_ADDRESS rc = %d\n", rc);
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

    if (reg & REG4A_SMB1357_STATUS_C_CHG_HOLD_OFF_BIT) {
        return POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    if ((reg & REG4A_SMB1357_STATUS_C_CHARGING_MASK)) {
        return POWER_SUPPLY_STATUS_CHARGING;
    }
    return POWER_SUPPLY_STATUS_DISCHARGING;
}

int smb1357_get_charging_status(struct opchg_charger *chip)
{
    int rc = 0;
    u8 reg = 0;

    rc = opchg_read_reg(chip, REG4A_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read REG4A_SMB1357_ADDRESS rc = %d\n", rc);
        return 0;
    }

    return (reg & REG4A_SMB1357_STATUS_C_CHG_ENABLE_STATUS_BIT) ? 1 : 0;
}

int smb1357_set_otg_regulator_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_masked_write(chip, REG42_SMB1357_ADDRESS, REG42_SMB1357_OTG_MASK, REG42_SMB1357_OTG_ENABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
    }

	return rc;

}

int smb1357_set_otg_regulator_disable(struct regulator_dev *rdev)
{

    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

	return rc;
    rc = opchg_masked_write(chip, REG42_SMB1357_ADDRESS, REG42_SMB1357_OTG_MASK, REG42_SMB1357_OTG_DISABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	}

    return rc;
}

int smb1357_get_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    u8 reg = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_read_reg(chip, REG42_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read OTG enable bit rc=%d\n", rc);
        return rc;
    }

    return (reg & REG42_SMB1357_OTG_ENABLE) ? 1 : 0;

}

int smb1357_set_charging_disable(struct opchg_charger *chip, bool disable)
{
    int rc = 0;

	rc = opchg_masked_write(chip, REG42_SMB1357_ADDRESS, REG42_SMB1357_CHARGING_MASK,
                            disable ? 0 : REG42_SMB1357_CHARGING_ENABLE);
    if (rc < 0) {
		pr_err("Couldn't set CHG_ENABLE_BIT disable = %d, rc = %d\n", disable, rc);
	}

	return rc;
}

int smb1357_set_suspend_enable(struct opchg_charger *chip, bool enable)
{
    int rc = 0;

    rc = opchg_masked_write(chip, REG11_SMB1357_ADDRESS, REG11_SMB1357_ENTER_SDP_OR_SUSPEND_MODE_MASK,
                            enable ? REG11_SMB1357_ENTER_SUSPEND_MODE : 0);
    if (rc < 0) {
		pr_err("Couldn't set CHG_SUSP_EN_BIT enable = %d, rc = %d\n", enable, rc);
	}

	return rc;

}

int smb1357_set_enable_volatile_writes(struct opchg_charger *chip)
{
    int rc = 0;

    rc = opchg_masked_write(chip, REG40_SMB1357_ADDRESS,REG40_SMB1357_BQ_CONFIG_MASK,
                            REG40_SMB1357_BQ_CONFIG_ENABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n", rc);
	}

    return rc;
}

int smb1357_set_prechg_current(struct opchg_charger *chip, int ipre_mA)
{
	u8 i = 0;

	for (i = ARRAY_SIZE(smb1357_per_chg_current) - 1; i >= 0; i--) {
		if (smb1357_per_chg_current[i] <= ipre_mA) {
			break;
		}
	}

	i = i << REG1C_SMB1357_PRE_CHARGER_CURRENT_SHIFT;

    return opchg_masked_write(chip, REG1C_SMB1357_ADDRESS, REG1C_SMB1357_PRE_CHARGER_CURRENT_MASK, i);
}

int smb1357_set_fastchg_current(struct opchg_charger *chip, int ifast_mA)
{
    u8 i=0;

    for (i = ARRAY_SIZE(smb1357_fast_chg_current) - 1; i >= 0; i--) {
        if (smb1357_fast_chg_current[i] <= ifast_mA) {
            break;
		}
    }

    if (i == 29) {
		i = REG1C_SMB1357_FAST_CHARGER_CURRENT_2700MA;
	}
	else if (i == 30) {
		i = REG1C_SMB1357_FAST_CHARGER_CURRENT_2800MA;
	}
	else if (i == 31) {
		i = REG1C_SMB1357_FAST_CHARGER_CURRENT_3000MA;
	}
	else if (i < 15) {
	    i = i<< REG1C_SMB1357_FAST_CHARGER_CURRENT_SHIFT;
	}
	else if (i < 22){
        i = i+1;
	    i = i<< REG1C_SMB1357_FAST_CHARGER_CURRENT_SHIFT;
	}
	else {
        i = i+2;
	    i = i<< REG1C_SMB1357_FAST_CHARGER_CURRENT_SHIFT;
	}

    i = i << REG1C_SMB1357_FAST_CHARGER_CURRENT_SHIFT;
    dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n", ifast_mA, i);

    return opchg_masked_write(chip, REG1C_SMB1357_ADDRESS, REG1C_SMB1357_FAST_CHARGER_CURRENT_MASK, i);
}

int smb1357_set_termchg_current(struct opchg_charger *chip, int iterm_current)
{
	u8 i= 0;

    for (i = ARRAY_SIZE(smb1357_termin_chg_current) - 1; i >= 0; i--) {
        if (smb1357_termin_chg_current[i] <= iterm_current) {
			break;
		}
	}

	if (i == 5) {
		i = 0;// 300mA
	}
	else if (i > 5) {
	    i = i<< REG03_SMB1357_CURRENT_TERMINATION_LEVEL_SHIFT;// 500mA,600mA
	}
	else {
        i = i+1;
	    i = i<< REG03_SMB1357_CURRENT_TERMINATION_LEVEL_SHIFT;
	}

    dev_dbg(chip->dev, "Termination limit=%d setting %02x\n", iterm_current, i);
    return opchg_masked_write(chip, REG03_SMB1357_ADDRESS, REG03_SMB1357_CURRENT_TERMINATION_LEVEL_MASK, i);
}

int smb1357_set_float_voltage(struct opchg_charger *chip, int vfloat_mv)
{
    u8 temp;

	//check float voltage <3600mv or >4500mv
    if (vfloat_mv < SMB1357_MIN_FLOAT_MV) {
        vfloat_mv = SMB1357_MIN_FLOAT_MV;
    }
    else if (vfloat_mv > SMB1357_MAX_FLOAT_MV) {
        vfloat_mv = SMB1357_MAX_FLOAT_MV;
    }

	// set float voltage
    if (vfloat_mv== SMB1357_VFLOAT_4350MV) {
        temp = REG1E_SMB1357_FLOAT_VOLTAGE_4350MV;
	}
    else if (vfloat_mv > SMB1357_VFLOAT_4350MV) {
        temp = (vfloat_mv - SMB1357_MIN_FLOAT_MV) / SMB1357_VFLOAT_STEP_MV + 1;
		temp += REG1E_SMB1357_FLOAT_VOLTAGE_3600MV;
    }
    else {
        temp = (vfloat_mv - SMB1357_MIN_FLOAT_MV) / SMB1357_VFLOAT_STEP_MV;
		temp += REG1E_SMB1357_FLOAT_VOLTAGE_3600MV;
    }

    return opchg_masked_write(chip, REG1E_SMB1357_ADDRESS, REG1E_SMB1357_FLOAT_VOLTAGE_MASK, temp);
}

int smb1357_set_input_chg_current(struct opchg_charger *chip, int current_ma, bool aicl)
{
	u8 i=0;

	for (i = ARRAY_SIZE(smb1357_usbin_input_current_limit) - 1; i >= 0; i--) {
		if (smb1357_usbin_input_current_limit[i] <= current_ma) {
			break;
		}
	}

	i=i << REG0C_SMB1357_USBIN_INPUT_CURRENT_SHIFT;

    dev_dbg(chip->dev, "usb input max current limit=%d setting %02x\n", current_ma, i);
    return opchg_masked_write(chip, REG0C_SMB1357_ADDRESS, REG0C_SMB1357_USBIN_INPUT_CURRENT_MASK, i);
}

int smb1357_set_complete_charge_timeout(struct opchg_charger *chip, int val)
{
    int rc = 0;
    int mask = 0;

    if (val == OVERTIME_AC) {
        val = SMB1357_FAST_TIME_384MIN | SMB1357_PRE_TIME_24MIN;
    }
    else if (val == OVERTIME_USB) {
        val = SMB1357_FAST_TIME_768MIN | SMB1357_PRE_TIME_48MIN;
    }
    else {
        val = SMB1357_FAST_TIME_768MIN | SMB1357_PRE_TIME_48MIN;
    }
	mask= REG16_SMB1357_CHARGER_TOTAL_TIME_MASK | REG16_SMB1357_PRE_CHARGER_TOTAL_TIME_MASK;
    rc = opchg_masked_write(chip, REG16_SMB1357_ADDRESS, mask, val);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't complete charge timeout rc = %d\n", rc);
    }

    return rc;
}

int smb1357_set_float_compensation_voltage(struct opchg_charger *chip, int comp_voltage)
{
	u8 i= 0;

	for (i = ARRAY_SIZE(smb1357_float_compensation_voltage) - 1; i >= 0; i--) {
        if (smb1357_float_compensation_voltage[i] <= comp_voltage) {
            break;
        }
    }

	i = i<< REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_SHIFT;

    dev_dbg(chip->dev, "compensation_voltage=%d setting %02x\n", comp_voltage, i);
    return opchg_masked_write(chip, REG03_SMB1357_ADDRESS, REG03_SMB1357_AUTO_FLOAT_VOLTAGE_COMPENSATION_MASK, i);
}

int smb1357_set_fastcharger_dectect(struct opchg_charger *chip)
{
	u8 reg = 0,mask = 0;
	int rc = 0;

	//====================================================================================
	// step:Allow volatile writes to CONFIG registers
	// set  registers40= 0x40 (Allow volatile access )
	// set  registers11= 0xA1 (Allow SDP mode and  Auto source detect enable APSD)
	// set  registers0C= 0x64 (USB IN 9V only  / Input Current Limit 500mA )
	// set  registers0C= 0x72 (USB IN 9V only  / Input Current Limit 1800mA )
	// set  registers0C= 0x5A (USB IN 5-9V Continuous  /Input Current Limit 2050mA )
	// set  registers1C= 0x9A (Fast Charge Current Limit 2050mA )
	// set  registers03= 0xCF (Automatic Float Voltage Compensation  )
	//====================================================================================

	reg = REG11_SMB1357_ENTER_SDP_MODE | REG11_SMB1357_APSD_ENABLE;
	mask = REG11_SMB1357_ENTER_SDP_OR_SUSPEND_MODE_MASK | REG11_SMB1357_APSD_MASK;
    rc = opchg_masked_write(chip, REG11_SMB1357_ADDRESS, mask,reg);

	reg = REG0C_SMB1357_USBIN_VOLTAGE_ONLY_9V;
	mask = REG0C_SMB1357_USBIN_VOLTAGE_ONLY_MASK;
    rc = opchg_masked_write(chip, REG0C_SMB1357_ADDRESS, mask,reg);

    mdelay(10);

	reg = REG0C_SMB1357_USBIN_VOLTAGE_5V_OR_9V;
	mask = REG0C_SMB1357_USBIN_VOLTAGE_ONLY_MASK;
    rc = opchg_masked_write(chip, REG0C_SMB1357_ADDRESS, mask,reg);

	return rc;
}

int smb1357_hw_init(struct opchg_charger *chip)
{
    int rc = 0;
	//u8 i=0;
    u8 reg = 0, mask = 0;

	rc = smb1357_set_enable_volatile_writes(chip);
    if (rc) {
        dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n", rc);
        return rc;
	}

	reg = REG07_SMB1357_HARD_LIMIT_IRQ | REG07_SMB1357_SOFT_LIMIT_IRQ | REG07_SMB1357_OTG_FAIL_IRQ |
			REG07_SMB1357_OTG_CURRENT_OVER_IRQ | REG07_SMB1357_USBIN_OV_IRQ | REG07_SMB1357_USBIN_UV_IRQ |
			REG07_SMB1357_AICL_DONE_IRQ | REG07_SMB1357_INTERNAL_THEP_SHUTDOWN_IRQ;
	mask = REG07_SMB1357_IRQ_MASK;
	rc = opchg_masked_write(chip, REG07_SMB1357_ADDRESS, mask, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set REG07_SMB1357_ADDRESS rc=%d\n", rc);
        return rc;
    }

	reg = REG08_SMB1357_SAFETY_TIME_IRQ | REG08_SMB1357_CHARGE_ERROR_IRQ | REG08_SMB1357_BATTERY_OV_IRQ |
			REG08_SMB1357_CHARGE_PHASE_IRQ | REG08_SMB1357_CHARGER_INHIBIT_IRQ | REG08_SMB1357_POWER_OK_IRQ |
			REG08_SMB1357_BATTERY_MISSING_IRQ | REG08_SMB1357_LOW_BATTERY_IRQ;
	mask = REG08_SMB1357_IRQ_MASK;
	rc = opchg_masked_write(chip, REG08_SMB1357_ADDRESS, mask, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set REG08_SMB1357_ADDRESS rc=%d\n", rc);
        return rc;
    }
	reg = REG09_SMB1357_WATCHDOG_OVERTIME_IRQ | REG09_SMB1357_SOURCE_DETECTION_STATUS_IRQ |
			REG09_SMB1357_DCIN_OV_IRQ | REG09_SMB1357_DCIN_UV_IRQ;
	mask = REG09_SMB1357_IRQ_MASK;
	rc = opchg_masked_write(chip, REG09_SMB1357_ADDRESS, mask, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set REG09_SMB1357_ADDRESS rc=%d\n", rc);
        return rc;
    }

	rc = smb1357_set_fastcharger_dectect(chip);

    /* setup THERM Monitor */
    rc = opchg_masked_write(chip, REG42_SMB1357_ADDRESS,
                            REG42_SMB1357_THERM_A_THERM_MONITOR_MASK, REG42_SMB1357_THERM_A_THERM_MONITOR_DISABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n", rc);
        return rc;
    }

	/* setup otg current limit */
    rc = opchg_masked_write(chip, REG12_SMB1357_ADDRESS, REG12_SMB1357_OTG_CURRENT_LIMIT_MASK, REG12_SMB1357_OTG_CURRENT_LIMIT_AT_USB_IN_1000MA);
    if (rc) {
        dev_err(chip->dev, "Couldn't set OTG_TLIM_THERM_REG rc=%d\n", rc);
        return rc;
    }

    /* set the input charge current limit */
	smb1357_set_input_chg_current(chip, chip->max_input_current[INPUT_CURRENT_MIN], false);

	/* set the fast charge current limit */
	opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);

	/* set the float voltage */
	opchg_set_float_voltage(chip, chip->min_term_voltage[TERM_VOL_MIN]);
    /* set pre-charging current */

	/* set pre-charging current */
	smb1357_set_prechg_current(chip, 250);

    /* set iterm */
    rc = smb1357_set_termchg_current(chip, chip->max_term_current[TERM_CURRENT_MAX]);//chip->iterm_ma);
	if (rc) {
        dev_err(chip->dev, "Couldn't set iterm rc = %d\n", rc);
        return rc;
    }
    rc = opchg_masked_write(chip, REG14_SMB1357_ADDRESS, REG14_SMB1357_CURRENT_TERMINATION_MASK, REG14_SMB1357_CURRENT_TERMINATION_ENABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't enable iterm rc = %d\n", rc);
        return rc;
    }

	//Charge enable source
	rc = opchg_masked_write(chip, REG14_SMB1357_ADDRESS, REG14_SMB1357_CHG_ENABLE_SOURCE_MASK, REG14_SMB1357_CHG_ENABLE_FOR_COMMAND_REGISTER);
	// set Charge enable command or pin polarity
	rc = opchg_masked_write(chip, REG14_SMB1357_ADDRESS, REG14_SMB1357_CHG_ENABLE_POLARITY_MASK, REG14_SMB1357_CHG_ENABLE_FOR_ACTIVE_HIGH);

    /* TODO: set inhibit threshold */
    if (chip->charger_inhibit_disabled) {
        rc = opchg_masked_write(chip, REG14_SMB1357_ADDRESS, REG14_SMB1357_CHARGER_INHIBIT_MASK, REG14_SMB1357_CHARGER_INHIBIT_DISABLE);
	}
	else {
        rc = opchg_masked_write(chip, REG14_SMB1357_ADDRESS, REG14_SMB1357_CHARGER_INHIBIT_MASK, REG14_SMB1357_CHARGER_INHIBIT_ENABLE);
    }

    /* set recharge voltage*/
    if (chip->recharge_mv >= 200) {
        reg = REG05_SMB1357_RECHARGER_VOL_THRESHOLD_200MV;
    }
	else
	{
        reg = REG05_SMB1357_RECHARGER_VOL_THRESHOLD_100MV;
    }
    rc = opchg_masked_write(chip, REG05_SMB1357_ADDRESS, REG05_SMB1357_RECHARGER_VOL_THRESHOLD_MASK, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set recharge voltage rc = %d\n", rc);
        return rc;
    }

    /* set float compensation voltage*/
    smb1357_set_float_compensation_voltage(chip, chip->float_compensation_mv);

	/* enable/disable stat output */
    rc = opchg_masked_write(chip, REG17_SMB1357_ADDRESS, REG17_SMB1357_CMD_A_STAT_DISABLE_MASK, REG17_SMB1357_CMD_A_STAT_DISABLE_BIT);
    if (rc) {
        dev_err(chip->dev, "Unable to %s stat pin. rc=%d\n",
                            REG17_SMB1357_CMD_A_STAT_DISABLE_BIT ? "disable" : "enable", rc);
    }

    rc = opchg_masked_write(chip, REG17_SMB1357_ADDRESS, REG17_SMB1357_STAT_PIN_CONFIG_MASK, REG17_SMB1357_STAT_PIN_CONFIG_PULSE);
    if (rc) {
        dev_err(chip->dev, "Unable to %s stat pin config. rc=%d\n",
                            REG17_SMB1357_STAT_PIN_CONFIG_PULSE ? "PULSE" : "static", rc);
    }

	/* enable/disable charging */
    //opchg_config_charging_disable(chip, USER_DISABLE, !!chip->disabled_status);
	opchg_config_charging_disable(chip, USER_DISABLE, 0);
	//rc = opchg_masked_write(chip, REG42_SMB1357_ADDRESS, REG42_SMB1357_CHARGING_MASK,
	//				   REG42_SMB1357_CHARGING_ENABLE);


	/* enable/disable fast charging setting */
    rc = opchg_masked_write(chip, REG14_SMB1357_ADDRESS, REG14_SMB1357_PRE_TO_FAST_CHARGING_MODE_MASK, REG14_SMB1357_PRE_TO_FAST_CHARGING_FOR_AUTO);
    if (rc) {
        dev_err(chip->dev, "Unable to %s fast charging set. rc=%d\n",
                            REG14_SMB1357_PRE_TO_FAST_CHARGING_FOR_AUTO ? "disable" : "enable", rc);
    }

    return rc;
}

int smb1357_get_initial_state(struct opchg_charger *chip)
{
    int rc = 0;
    u8 reg = 0;


    /* BATTERY ERROR */
    rc = opchg_read_reg(chip, REG56_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
        goto fail_init_status;
    }
	else
	{
		dev_err(chip->dev, "read IRQ_G_reg56  = %d\n", reg);
	}

    /* Use PMIC BTM way to detect battery exist */
    rc = opchg_read_reg(chip, REG52_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
        goto fail_init_status;
    }
	else
	{
		dev_err(chip->dev, "read IRQ_B_reg52  = %d\n", reg);
	}
    chip->batt_full = (reg & REG52_SMB1357_IRQ_C_TERM_CHARGING_VOL_STATUS) ? true : false;

	/* usb_in*/
    rc = opchg_read_reg(chip, REG54_SMB1357_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
        goto fail_init_status;
    }
	else
	{
		dev_err(chip->dev, "read IRQ_E_reg54  = %d\n", reg);
	}
    if (reg & REG54_SMB1357_IRQ_E_USB_CHARGER_VOL_UV_STATUS) {
        smb1357_chg_uv(chip, 0);
	}
	else {
        smb1357_chg_uv(chip, 1);
    }

    return 0;

fail_init_status:
    dev_err(chip->dev, "Couldn't determine intial status\n");
    return rc;
}

void smb1357_dump_regs(struct opchg_charger *chip)
{
    int rc;
    u8 reg, addr;

	// read config register
	for (addr = 0; addr <= SMB1357_LAST_CNFG_REG; addr++) {
		rc = opchg_read_reg(chip, addr, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
		else {
			pr_debug("smb1357_read_reg 0x%02x = 0x%02x\n", addr, reg);
		}
	}
	// read status register
	for (addr = SMB1357_FIRST_STATUS_REG; addr <= SMB1357_LAST_STATUS_REG; addr++) {
		rc = opchg_read_reg(chip, addr, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
		else {
			pr_debug("smb1357_read_reg 0x%02x = 0x%02x\n", addr, reg);
		}
	}
	// read command register
	for (addr = SMB1357_FIRST_CMD_REG; addr <= SMB1357_LAST_CMD_REG; addr++) {
		rc = opchg_read_reg(chip, addr, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
		else {
			pr_debug("smb1357_read_reg 0x%02x = 0x%02x\n", addr, reg);
		}
	}
}

int smb1357_chg_uv(struct opchg_charger *chip, u8 status)
{
    opchg_inout_charge_parameters(chip);

    if (status == 0) {
        chip->g_chg_in = 1;
		if(chip->g_is_wakeup == 0){ //if awake not be lock,lock it here else do nothing
            __pm_stay_awake(&chip->source);
            chip->g_is_wakeup= 1;
        }
    }
    else {
        chip->g_chg_in = 0;
        schedule_delayed_work(&chip->opchg_delayed_wakeup_work,
                            round_jiffies_relative(msecs_to_jiffies(2000)));
    }

    /* use this to detect USB insertion only if !apsd */
    if (chip->disable_apsd && status == 0) {
        chip->chg_present = true;
        dev_dbg(chip->dev, "%s updating usb_psy present=%d", __func__, chip->chg_present);
        power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
        power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

    if (status != 0) {
        chip->chg_present = false;
        dev_dbg(chip->dev, "%s updating usb_psy present=%d", __func__, chip->chg_present);
        /* we can't set usb_psy as UNKNOWN so early, it'll lead USERSPACE issue */
        power_supply_set_present(chip->usb_psy, chip->chg_present);
    }

    power_supply_changed(chip->usb_psy);
	//power_supply_changed(&chip->dc_psy);

    dev_dbg(chip->dev, "chip->chg_present = %d,chip->g_is_wakeup = %d\n", chip->chg_present ,chip->g_is_wakeup);
    return 0;
}

int smb1357_chg_ov(struct opchg_charger *chip, u8 status)
{
    u8 psy_health_sts;
    if (status) {
        psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        opchg_config_charging_disable(chip, CHAGER_ERR_DISABLE, 1);//smb358_charging(chip, false);
	}
	else {
        psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
        opchg_config_charging_disable(chip, CHAGER_ERR_DISABLE, 0);//smb358_charging(chip, true);
    }
    power_supply_set_health_state(chip->usb_psy, psy_health_sts);
    power_supply_changed(chip->usb_psy);
    //power_supply_changed(&chip->dc_psy);
    return 0;
}

int smb1357_fast_chg(struct opchg_charger *chip, u8 status)
{
    if (!!status) {
        opchg_config_charging_phase(chip,CC_PHASE);
    }

    power_supply_changed(&chip->batt_psy);
    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int smb1357_recharge_chg(struct opchg_charger *chip, u8 status)
{
    if (!!status) {
        opchg_config_charging_phase(chip,RECHARGE_PHASE);
    }

    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int smb1357_taper_chg(struct opchg_charger *chip, u8 status)
{
    if (!!status) {
        opchg_config_charging_phase(chip,CV_PHASE);
    }

    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int smb1357_chg_term(struct opchg_charger *chip, u8 status)
{
    if (!!status) {
        opchg_config_charging_phase(chip,TERM_PHASE);
    }

    chip->batt_pre_full = !!status;//chip->batt_full = !!status;
    //power_supply_changed(&chip->batt_psy);

    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int smb1357_safety_timeout(struct opchg_charger *chip, u8 status)
{
    if (status) {
		chip->charging_time_out = true;
	}
	else {
        chip->charging_time_out = false;
    }

    return 0;
}

static struct irq_handler_info handlers[] = {
    [0] = {
        .stat_reg   = IRQ_A_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "cold_soft",
            },
            {
                .name       = "hot_soft",
            },
            {
                .name       = "cold_hard",
            },
            {
                .name       = "hot_hard",
            },
        },
    },
    [1] = {
        .stat_reg   = IRQ_B_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "chg_hot",
            },
            {
                .name       = "vbat_low",
            },
            {
                .name       = "battery_missing",
            },
            {
                .name       = "battery_ov",//"battery_terminal",
            },
        },
    },
    [2] = {
        .stat_reg   = IRQ_C_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "chg_term",
                .smb_irq    = smb1357_chg_term,
            },
            {
                .name       = "taper",
                .smb_irq    = smb358_taper_chg,
            },
            {
                .name       = "recharge",
                .smb_irq    = smb358_recharge_chg,
            },
            {
                .name       = "fast_chg",
                .smb_irq    = smb1357_fast_chg,
            },
        },
    },
    [3] = {
        .stat_reg   = IRQ_D_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "prechg_timeout",
            },
            {
                .name       = "safety_timeout",
                .smb_irq    = smb1357_safety_timeout,
            },
            {
                .name       = "aicl_complete",
            },
            {
                .name       = "src_detect",//"battery_ov",
            },
        },
    },
    [4] = {
        .stat_reg   = IRQ_E_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "usbin_uv",
                .smb_irq    = smb1357_chg_uv,
            },
            {
                .name       = "usbin_ov",
                .smb_irq    = smb1357_chg_ov,
            },
            {
                .name       = "unknown",
            },
            {
                .name       = "unknown",
            },
        },
    },
    [5] = {
        .stat_reg   = IRQ_F_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "power_ok",
            },
            {
                .name       = "otg_det",
            },
            {
                .name       = "otg_batt_uv",
            },
            {
                .name       = "otg_oc",
            },
        },
    },
    [6] = {
        .stat_reg   = IRQ_G_REG_SMB1357,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "Charge_inhibit",
            },
            {
                .name       = "Charge_error",
            },
            {
                .name       = "Watchdog_timeout",
            },
            {
                .name       = "src_detect",
            },
        },
    },
};

void smb1357_chg_irq_handler(int irq, struct opchg_charger *chip)
{

    //struct opchg_charger *chip = dev_id;
    int i, j;
    //u8 reg = 0;
    u8 triggered;
    u8 changed;
    u8 rt_stat, prev_rt_stat;
    int rc = 0;
    int handler_count = 0;

    for (i = 0; i < ARRAY_SIZE(handlers); i++) {
        rc = opchg_read_reg(chip, handlers[i].stat_reg,&handlers[i].val);
        if (rc < 0) {
            dev_err(chip->dev, "Couldn't read %d rc = %d\n",handlers[i].stat_reg, rc);
            continue;
        }

        for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
            triggered = handlers[i].val & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
            rt_stat = handlers[i].val & (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
            prev_rt_stat = handlers[i].prev_val & (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
            changed = prev_rt_stat ^ rt_stat;

            if (triggered || changed) {
                pr_debug("irq %s: triggered = 0x%02x, rt_stat = 0x%02x, prev_rt_stat = 0x%02x\n",
                            handlers[i].irq_info[j].name, triggered,rt_stat, prev_rt_stat);

                rt_stat ? handlers[i].irq_info[j].high++ : handlers[i].irq_info[j].low++;
            }

            if ((triggered || changed) && handlers[i].irq_info[j].smb_irq != NULL) {
                handler_count++;

                rc = handlers[i].irq_info[j].smb_irq(chip, rt_stat);
                if (rc < 0) {
					dev_err(chip->dev, "Couldn't handle %d irq for reg 0x%02x rc = %d\n",
                            j, handlers[i].stat_reg, rc);
                }
            }
        }
        handlers[i].prev_val = handlers[i].val;
    }

	pr_debug("oppo handler count = %d\n", handler_count);
	if (handler_count) {
        pr_debug("batt psy changed\n");
        power_supply_changed(&chip->batt_psy);
    }
    opchg_dump_regs(chip);

	//return IRQ_HANDLED;
}
