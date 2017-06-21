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

#define OPPO_SMB358_PAR
#include "oppo_inc.h"


int smb358_get_prop_charge_type(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, STATUS_C_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
    }

    reg &= STATUS_C_CHARGING_MASK;

   if (reg == STATUS_C_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

int smb358_get_prop_batt_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, STATUS_C_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

    if (reg & STATUS_C_CHG_HOLD_OFF_BIT) {
        return POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    if ((reg & STATUS_C_CHARGING_MASK) && !(reg & STATUS_C_CHG_ERR_STATUS_BIT)) {
        return POWER_SUPPLY_STATUS_CHARGING;
    }

    return POWER_SUPPLY_STATUS_DISCHARGING;
}

int smb358_get_charging_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, STATUS_C_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }

    return (reg & STATUS_C_CHG_ENABLE_STATUS_BIT) ? 1 : 0;
}

int smb358_set_otg_regulator_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, CMD_A_OTG_ENABLE_BIT);
    if (rc) {
        dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
    }

	return rc;
}

int smb358_set_otg_regulator_disable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
    if (rc) {
        dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	}

    return rc;
}

int smb358_get_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    u8 reg = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_read_reg(chip, CMD_A_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read OTG enable bit rc=%d\n", rc);
        return rc;
    }

    return (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}
#if 1
int smb358_set_otg_enable(void)
{
    int rc = 0;

	if(opchg_chip == NULL)
	{
		return rc;
	}

    rc = opchg_masked_write(opchg_chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, CMD_A_OTG_ENABLE_BIT);
	if (rc) {
        pr_debug("Couldn't enable  OTG mode rc=%d\n", rc);
    }
	else
	{
		pr_debug("smb358_set_otg_enable\n");
	}

	return rc;
}

int smb358_set_otg_disable(void)
{
    int rc = 0;

    if(opchg_chip == NULL)
	{
		return rc;
	}
    rc = opchg_masked_write(opchg_chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);

	if (rc) {
        pr_debug("Couldn't disable OTG mode rc=%d\n", rc);
	}
	else
	{
		pr_debug("smb358_set_otg_disable\n");
	}

    return rc;
}

int smb358_get_otg_enable(void)
{
    int rc = 0;
    u8 reg = 0;

	if(opchg_chip == NULL)
	{
		return rc;
	}
	rc = opchg_read_reg(opchg_chip, CMD_A_REG, &reg);
    if (rc) {
        pr_debug("Couldn't read OTG enable bit rc=%d\n", rc);
    }
	else
	{
		pr_debug("smb358_get_otg_enable read OTG enable bit =%d\n", reg);
    }
	return rc;
}
#endif

int smb358_set_charging_disable(struct opchg_charger *chip, bool disable)
{
    int rc;

    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_MASK,
                            disable ? 0 : CMD_A_CHG_ENABLE_BIT);
    if (rc < 0) {
		pr_err("Couldn't set CHG_ENABLE_BIT disable = %d, rc = %d\n", disable, rc);
	}

	return rc;
}

int smb358_charging_disable(struct opchg_charger *chip,
						int reason, int disable)
{
	int rc = 0;
	int disabled;

	disabled = chip->charging_disabled_status;

	pr_debug("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;

	rc = smb358_set_charging_disable(chip, !!disabled);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	} else {
	/* will not modify online status in this condition */
		power_supply_changed(&chip->batt_psy);
	}

skip:
	chip->charging_disabled_status = disabled;
	return rc;
}

int smb358_set_suspend_enable(struct opchg_charger *chip, bool enable)
{
    int rc;

    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK,
                            enable ? CMD_A_CHG_SUSP_EN_BIT : 0);
    if (rc < 0) {
		pr_err("Couldn't set CHG_SUSP_EN_BIT enable = %d, rc = %d\n", enable, rc);
	}

	return rc;
}

int smb358_set_enable_volatile_writes(struct opchg_charger *chip)
{
    int rc;

    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
                            CMD_A_VOLATILE_W_PERM_BIT);
    if (rc) {
        dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n", rc);
	}

    return rc;
}

int smb358_set_fastchg_current(struct opchg_charger *chip, int ifast_mA)
{
    u8 i;

    if ((ifast_mA < SMB358_FAST_CHG_MIN_MA) || (ifast_mA >  SMB358_FAST_CHG_MAX_MA)) {
        dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n", ifast_mA);
        return -EINVAL;
    }

	if((!chip->batt_authen) && (ifast_mA > chip->non_standard_fastchg_current_ma)){
		ifast_mA = chip->non_standard_fastchg_current_ma;
	}

    for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
        if (fast_chg_current[i] <= ifast_mA) {
            break;
		}
    }

    if (i < 0) {
        dev_err(chip->dev, "Cannot find %dmA\n", ifast_mA);
        i = 0;
    }

    i = i << SMB358_FAST_CHG_SHIFT;
    dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n", ifast_mA, i);

    return opchg_masked_write(chip, CHG_CURRENT_CTRL_REG, SMB_FAST_CHG_CURRENT_MASK, i);
}

int smb358_set_float_voltage(struct opchg_charger *chip, int vfloat_mv)
{
    u8 temp;

    if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
        dev_err(chip->dev, "bad float voltage mv =%d asked to set\n", vfloat_mv);
        return -EINVAL;
    }

	#if 0
	if((!chip->batt_authen) && (opchg_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__NORMAL))
	#else
	if((!chip->batt_authen) && (chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_NORMAL))
	#endif
	{
		vfloat_mv = chip->non_standard_vfloat_mv;
	}

    if (VFLOAT_4350MV == vfloat_mv) {
        temp = 0x2B;
	}else if (vfloat_mv > VFLOAT_4350MV) {
        temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV + 1;
    }else {
        temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;
    }

    return opchg_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

static int smb358_set_term_current(struct opchg_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled)
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = opchg_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = opchg_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		rc = opchg_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int smb358_set_recharge_and_inhibit(struct opchg_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->recharge_disabled)
		rc = opchg_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_AUTO_RECHARGE_MASK, CHG_AUTO_RECHARGE_DIS_BIT);
	else
		rc = opchg_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_AUTO_RECHARGE_MASK, 0x0);
	if (rc) {
		dev_err(chip->dev,"Couldn't set auto recharge en reg rc = %d\n", rc);
	}

	if (chip->charger_inhibit_disabled)
		rc = opchg_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = opchg_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit en reg rc = %d\n", rc);
		return rc;
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		rc = opchg_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}
int smb358_set_precharger_voltage(struct opchg_charger *chip)
{
	u8 reg;
	int rc=0;

	/* set precharge voltage*/
	reg = REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_3000MV;
	rc = opchg_masked_write(chip, VFLOAT_REG, REG03_SMB358_PRE_TO_FAST_CHARGING_VOL_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set recharging threshold rc = %d\n", rc);
		return rc;
	}
    return rc;
}

int smb358_set_recharger_voltage(struct opchg_charger *chip)
{
	u8 reg;
	int rc=0;

	/* set recharge voltage*/
	if(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_COOL)
	{
		reg = VFLT_300MV;
	}
	else
	{
		reg = VFLT_100MV;
	}
	rc = opchg_masked_write(chip, INPUT_CURRENT_LIMIT_REG,VFLT_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set recharging threshold rc = %d\n", rc);
		return rc;
	}
    return rc;
}
int smb358_set_prechg_current(struct opchg_charger *chip, int ipre_mA)
{
    u8 value;
    if(ipre_mA <= 150)
    {
	value =REG00_SMB358_PRE_CHARGING_CURRENT_150MA;
    }
	else if(ipre_mA <= 250)
	{
		value =REG00_SMB358_PRE_CHARGING_CURRENT_150MA;
	}
	else if(ipre_mA <= 350)
	{
		value =REG00_SMB358_PRE_CHARGING_CURRENT_150MA;
	}
	else
	{
		value =REG00_SMB358_PRE_CHARGING_CURRENT_150MA;
	}
	return opchg_masked_write(chip, CHG_CURRENT_CTRL_REG, CHG_PRE_MASK, value);
}

int smb358_aicl_enable(struct opchg_charger *chip, bool enable)
{
	int rc;

	pr_err("%s enable:%d\n",__func__,enable);
	rc = opchg_masked_write(chip, VARIOUS_FUNC_REG,AICL_EN_MASK,
			enable ? AICL_EN_BIT : AICL_DISEN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set aicl enable rc = %d\n", rc);
		return rc;
	}
	return 0;
}

int smb358_set_input_chg_current(struct opchg_charger *chip, int current_ma, bool aicl)
{
    int i, rc = 0;
    u8 reg1 = 0, reg2 = 0, mask = 0;
    //u8 val = 0;

    dev_dbg(chip->dev, "%s: USB current_ma = %d\n", __func__, current_ma);

    if (chip->chg_autonomous_mode) {
        dev_dbg(chip->dev, "%s: Charger in autonmous mode\n", __func__);
        return 0;
    }
	smb358_aicl_enable(chip, false);
	smb358_aicl_enable(chip, true);
    #if 0
    /* Only set suspend bit when chg present and current_ma = 2 */
    if (current_ma == 2 && chip->chg_present) {
        rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK, CMD_A_CHG_SUSP_EN_BIT);
        if (rc < 0) {
			dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);
		}

        return rc;
    }
	#endif

	if (current_ma <= 2)
        current_ma = USB2_MIN_CURRENT_MA;
    else if (current_ma <= USB2_MIN_CURRENT_MA)
        current_ma = USB2_MAX_CURRENT_MA;

    if (current_ma == USB2_MIN_CURRENT_MA) {
        /* USB 2.0 - 100mA */
        reg1 &= ~USB3_ENABLE_BIT;
        reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	}
	else if (current_ma == USB2_MAX_CURRENT_MA) {
        /* USB 2.0 - 500mA */
        reg1 &= ~USB3_ENABLE_BIT;
        reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	}
	else if (current_ma == USB3_MAX_CURRENT_MA) {
        /* USB 3.0 - 900mA */
        reg1 |= USB3_ENABLE_BIT;
        reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	}
	else if (current_ma > USB2_MAX_CURRENT_MA) {
        /* HC mode  - if none of the above */
        reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

        for (i = ARRAY_SIZE(input_current) - 1; i >= 0; i--) {
            if (input_current[i] <= current_ma) {
                break;
            }
        }
        if (i < 0) {
            dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
            i = 0;
        }

        i = i << SMB358_INPUT_CURRENT_LIMIT_SHIFT;
        rc = opchg_masked_write(chip, INPUT_CURRENT_LIMIT_REG, AC_CHG_CURRENT_MASK, i);
        if (rc) {
            dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
        }
    }

    mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
    rc = opchg_masked_write(chip, CMD_B_REG, mask, reg2);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);
    }

    mask = USB3_ENABLE_MASK;
    rc = opchg_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);
    if (rc < 0) {
		dev_err(chip->dev, "Couldn't set USB3 mode rc = %d\n", rc);
    }

    return rc;
}

int smb358_set_complete_charge_timeout(struct opchg_charger *chip, int val)
{
    int rc = 0;

    if (val == OVERTIME_AC){
        val = TIME_382MIN;
    }
    else if (val == OVERTIME_USB){
        val = TIME_764MIN;
    }
    else {
        val = TIME_DISABLED;
    }
    val = val << TIMER_CTRL_REG_SHIFT;
    rc = opchg_masked_write(chip, STAT_AND_TIMER_CTRL_REG, TIMER_CTRL_REG_MASK, val);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't complete charge timeout rc = %d\n", rc);
    }

    return rc;
}

int smb358_hw_init(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additonal settings
	 */
    if (chip->chg_autonomous_mode) {
        dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
        return 0;
    }

    rc = opchg_read_reg(chip, CHG_REVISION_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read CHG_REVISION_REG rc=%d\n", rc);
        return rc;
    }

    rc = smb358_set_enable_volatile_writes(chip);
    if (rc) {
        dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n", rc);
        return rc;
	}

    /* setup defaults for CHG_CNTRL_REG */
    reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
    mask = CHG_CTRL_BATT_MISSING_DET_MASK;
    rc = opchg_masked_write(chip, CHG_CTRL_REG, mask, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
        return rc;
    }

    /* setup defaults for PIN_CTRL_REG */
    reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
                            CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
    mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
                            CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
    rc = opchg_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n", rc);
        return rc;
    }

    /* setup USB suspend and APSD  */
    rc = opchg_masked_write(chip, VARIOUS_FUNC_REG,
                            VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
    if (rc) {
        dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n", rc);
        return rc;
    }

    if (!chip->disable_apsd) {
        reg = CHG_CTRL_APSD_EN_BIT;
    }
	else {
        reg = 0;
    }
    rc = opchg_masked_write(chip, CHG_CTRL_REG, CHG_CTRL_APSD_EN_MASK, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
        return rc;
    }

    /* Fault and Status IRQ configuration */
    reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT | FAULT_INT_INPUT_UV_BIT
                            | FAULT_INT_AICL_COMPLETE_BIT | FAULT_INT_INPUT_OV_BIT;
    rc = opchg_write_reg(chip, FAULT_INT_REG, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
        return rc;
    }

    reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT | STATUS_INT_BATT_OV_BIT
                            | STATUS_INT_CHGING_BIT | STATUS_INT_CHG_INHI_BIT
                            | STATUS_INT_INOK_BIT | STATUS_INT_LOW_BATT_BIT
                            | STATUS_INT_MISSING_BATT_BIT;
    rc = opchg_write_reg(chip, STATUS_INT_REG, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
        return rc;
    }

    /* setup THERM Monitor */
    rc = opchg_masked_write(chip, THERM_A_CTRL_REG,
                            THERM_A_THERM_MONITOR_EN_MASK, THERM_A_THERM_MONITOR_EN_MASK);
    if (rc) {
        dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n", rc);
        return rc;
    }

    /* setup switching frequency */
    rc = opchg_masked_write(chip, THERM_A_CTRL_REG,
                            THERM_A_SWITCHING_FREQ_MASK, THERM_A_SWITCHING_FREQ_1_5MHZ);
    if (rc) {
        dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n", rc);
        return rc;
    }

    /* setup otg current limit */
    rc = opchg_masked_write(chip, OTG_TLIM_THERM_REG, OTG_CURRENT_LIMIT_MASK, OTG_CURRENT_LIMIT_BIT);
    if (rc) {
        dev_err(chip->dev, "Couldn't set OTG_TLIM_THERM_REG rc=%d\n", rc);
        return rc;
    }

    /* set the fast charge current limit */
	opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);

	/* set the float voltage */
	opchg_set_float_voltage(chip, chip->min_term_voltage[TERM_VOL_MIN]);

    #if 0
    /* set low_battery voltage threshold */
    reg = 0x0f;//3.58V
    rc = opchg_masked_write(chip, LOW_BATT_THRESHOLD_REG, CHG_LOW_BATT_THRESHOLD_MASK, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set LOW_BATT_THRESHOLD_REG rc=%d\n", rc);
        return rc;
    }
    #endif

    /* set pre-charging current */
    reg = CHG_PRE_450MA;
    rc = opchg_masked_write(chip, CHG_CURRENT_CTRL_REG, CHG_PRE_MASK, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set CHG_CURRENT_CTRL_REG rc=%d\n", rc);
        return rc;
    }

	/* set iterm */
	rc = smb358_set_term_current(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set term current rc=%d\n", rc);

	/* set recharge */
	rc = smb358_set_recharge_and_inhibit(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set recharge para rc=%d\n", rc);

    rc = opchg_masked_write(chip, INPUT_CURRENT_LIMIT_REG, VFLT_MASK, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set inhibit threshold rc = %d\n", rc);
        return rc;
    }

    /* enable/disable stat output */
    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_STAT_DISABLE_MASK, CMD_A_STAT_DISABLE_BIT);
    if (rc) {
        dev_err(chip->dev, "Unable to %s stat pin. rc=%d\n",
                            CMD_A_STAT_DISABLE_BIT ? "disable" : "enable", rc);
    }

    /* enable/disable charging */
    //opchg_config_charging_disable(chip, USER_DISABLE, !!chip->disabled_status);
    /* enable/disable charging */
	if (chip->charging_disabled) {
		rc = smb358_charging_disable(chip, USER, 1);
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
								chip->charging_disabled ? "disable" : "enable", rc);
	} else {
			/*
			* Enable charging explictly,
			* because not sure the default behavior.
			*/
			rc = smb358_set_charging_disable(chip, 0);
			if (rc)
				dev_err(chip->dev, "Couldn't enable charging\n");
	}

    /* enable/disable fast charging setting */
    rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_FAST_CHARGING_SET_MASK, CMD_A_FAST_CHARGING_SET_BIT);
    if (rc) {
        dev_err(chip->dev, "Unable to %s fast charging set. rc=%d\n",
                            CMD_A_FAST_CHARGING_SET_BIT ? "disable" : "enable", rc);
    }

	/*
	* Workaround for recharge frequent issue: When battery is
	* greater than 4.2v, and charging is disabled, charger
	* stops switching. In such a case, system load is provided
	* by battery rather than input, even though input is still
	* there. Make reg09[0:3] to be a non-zero value which can
	* keep the switcher active
	*/
	rc = opchg_masked_write(chip, OTHER_CTRL_REG, CHG_LOW_BATT_THRESHOLD_MASK,
											SMB358_BATT_GOOD_THRE_2P5);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTHER_CTRL_REG, rc = %d\n",rc);

    return rc;
}

int smb358_get_initial_state(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, IRQ_B_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
        goto fail_init_status;
    }

    /* Use PMIC BTM way to detect battery exist */
    rc = opchg_read_reg(chip, IRQ_C_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
        goto fail_init_status;
    }
    chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

    rc = opchg_read_reg(chip, IRQ_E_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
        goto fail_init_status;
    }
    if (reg & IRQ_E_INPUT_UV_BIT) {
        smb358_chg_uv(chip, 1);
	}
	else {
        smb358_chg_uv(chip, 0);
    }

    return 0;

fail_init_status:
    dev_err(chip->dev, "Couldn't determine intial status\n");
    return rc;
}

void smb358_dump_regs(struct opchg_charger *chip)
{
    int rc;
    u8 reg, addr;

	// read config register
    for (addr = 0; addr <= SMB358_LAST_CNFG_REG; addr++) {
        rc = opchg_read_reg(chip, addr, &reg);
        if (rc) {
            dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
        else {
            pr_debug("smb358_read_reg 0x%02x = 0x%02x\n", addr, reg);
        }
    }
	// read status register
    for (addr = SMB358_FIRST_STATUS_REG; addr <= SMB358_LAST_STATUS_REG; addr++) {
        rc = opchg_read_reg(chip, addr, &reg);
        if (rc) {
            dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
        else {
            pr_debug("smb358_read_reg 0x%02x = 0x%02x\n", addr, reg);
        }
    }
	// read command register
    for (addr = SMB358_FIRST_CMD_REG; addr <= SMB358_LAST_CMD_REG; addr++) {
        rc = opchg_read_reg(chip, addr, &reg);
        if (rc) {
            dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
        else {
            pr_debug("smb358_read_reg 0x%02x = 0x%02x\n", addr, reg);
        }
    }
}

int smb358_chg_uv(struct opchg_charger *chip, u8 status)
{
	int rc = 0;

	if(chip->chg_present && (status == 0)){
		pr_err("%s chg has plugged in,return\n",__func__);
		return 0;
	}

	opchg_inout_charge_parameters(chip);
	//opchg_switch_to_usbin(chip,!status);

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

		if (chip->bms_controlled_charging){
			/*
			* Disable SOC based USB suspend to enable charging on
			* USB insertion.
			*/
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc < 0)
				dev_err(chip->dev,"Couldn't disable usb suspend rc = %d\n",rc);
		}
    }

    //chip->BMT_status.charger_exist = chip->chg_present;

    power_supply_changed(chip->usb_psy);

    dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

    return 0;
}

int smb358_chg_ov(struct opchg_charger *chip, u8 status)
{
    u8 psy_health_sts;
    if (status) {
        psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        //opchg_config_charging_disable(chip, CHAGER_ERR_DISABLE, 1);//smb358_charging(chip, false);
		smb358_set_charging_disable(chip,true);
	}
	else {
        psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
        //opchg_config_charging_disable(chip, CHAGER_ERR_DISABLE, 0);//smb358_charging(chip, true);
		smb358_set_charging_disable(chip,false);
	}
    power_supply_set_health_state(chip->usb_psy, psy_health_sts);
    power_supply_changed(chip->usb_psy);

    return 0;
}

int smb358_fast_chg(struct opchg_charger *chip, u8 status)
{
    opchg_config_charging_phase(chip,CC_PHASE);

    power_supply_changed(&chip->batt_psy);
    dev_dbg(chip->dev, "%s\n", __func__);
	//if(status & STATUS_FAST_CHARGING)
	//		chip->batt_full = false;
    return 0;
}

int smb358_recharge_chg(struct opchg_charger *chip, u8 status)
{
    opchg_config_charging_phase(chip,RECHARGE_PHASE);

    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int smb358_taper_chg(struct opchg_charger *chip, u8 status)
{
    opchg_config_charging_phase(chip,CV_PHASE);

    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int smb358_chg_term(struct opchg_charger *chip, u8 status)
{
	bool term_status = 0;

    opchg_config_charging_phase(chip,TERM_PHASE);
    term_status = !!status;
	if(term_status)
	chip->batt_pre_full = term_status;//chip->batt_full = !!status;
    //power_supply_changed(&chip->batt_psy);
    chip->batt_pre_full_smb358 = term_status;
    dev_dbg(chip->dev, "%s status:%d\n", __func__, status);
    return 0;
}

int smb358_battery_ov(struct opchg_charger *chip, u8 status)
{
	bool ov_status = 0;

    opchg_config_charging_phase(chip,TERM_PHASE);
    ov_status = !!status;
	if(ov_status)
	chip->batt_pre_full = ov_status;

    dev_dbg(chip->dev, "%s status:%d\n", __func__, status);
    return 0;
}

int smb358_safety_timeout(struct opchg_charger *chip, u8 status)
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
        .stat_reg   = IRQ_A_REG,
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
        .stat_reg   = IRQ_B_REG,
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
                .name       = "battery_ov",
				.smb_irq    = smb358_battery_ov,
            },
        },
    },
    [2] = {
        .stat_reg   = IRQ_C_REG,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "chg_term",
                .smb_irq    = smb358_chg_term,
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
                .smb_irq    = smb358_fast_chg,
            },
        },
    },
    [3] = {
        .stat_reg   = IRQ_D_REG,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "prechg_timeout",
            },
            {
                .name       = "safety_timeout",
                .smb_irq    = smb358_safety_timeout,
            },
            {
                .name       = "aicl_complete",
            },
            {
                .name       = "src_detect",
            },
        },
    },
    [4] = {
        .stat_reg   = IRQ_E_REG,
        .val        = 0,
        .prev_val   = 0,
        .irq_info   = {
            {
                .name       = "usbin_uv",
                .smb_irq    = smb358_chg_uv,
            },
            {
                .name       = "usbin_ov",
                .smb_irq    = smb358_chg_ov,
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
        .stat_reg   = IRQ_F_REG,
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
};

void smb358_chg_irq_handler(int irq, struct opchg_charger *chip)
{
    //struct opchg_charger *chip = dev_id;
    int i, j;
    u8 reg = 0;
    u8 triggered;
    u8 changed;
    u8 rt_stat, prev_rt_stat;
    int rc;
    int handler_count = 0;

    rc = opchg_read_reg(chip, FAULT_INT_REG, &reg);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't read %d rc = %d\n", FAULT_INT_REG, rc);
    }

	rc = opchg_read_reg(chip, STATUS_INT_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n", STATUS_INT_REG, rc);
	}

	rc = opchg_read_reg(chip, STATUS_D_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n", STATUS_D_REG, rc);
	}

	rc = opchg_read_reg(chip, STATUS_E_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n", STATUS_E_REG, rc);
	}

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

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
        pr_debug("batt psy changed\n");
        power_supply_changed(&chip->batt_psy);
    }

	//return IRQ_HANDLED;
}
