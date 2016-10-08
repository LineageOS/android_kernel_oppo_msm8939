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

#define OPPO_BQ24157_PAR
#include "oppo_inc.h"

int bq24157_get_prop_charge_type(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, BQ24157_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
    }

    reg &= BQ24157_CHARGE_STATUS_MASK;

    if (reg == BQ24157_CHARGE_IN_PROGRESS) {
        return POWER_SUPPLY_CHARGE_TYPE_FAST;
    } else {
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }
}

int bq24157_get_prop_batt_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;
	static char vbus_low_count = 0;

	//because bq24157 dpm_vol is 4360mv,so vbus don't below 4300mv normally
	if(opchg_get_prop_charger_voltage_now(chip) < 4200){
		vbus_low_count++;
		if(vbus_low_count > 3){
			//dev_err(chip->dev, "vbus_vol lower than 4200\n,return discharging");
			return POWER_SUPPLY_STATUS_DISCHARGING;
		}
	} else {
		vbus_low_count = 0;
	}

    rc = opchg_read_reg(chip, BQ24157_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

	reg &= BQ24157_CHARGE_STATUS_MASK;
	if (reg == BQ24157_CHARGE_IN_PROGRESS || reg == BQ24157_CHARGE_TERM) {
        return POWER_SUPPLY_STATUS_CHARGING;
    } else {
        return POWER_SUPPLY_STATUS_DISCHARGING;
    }
}

int bq24157_get_charging_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, BQ24157_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }
	if(reg & BQ24157_CHARGE_ENABLE_MASK)
		return 0;
	else
		return 1;
}

int bq24157_set_otg_regulator_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    opchg_config_charging_disable(chip, CHAGER_OTG_DISABLE, 1);
    opchg_set_charging_disable(chip, !!chip->disabled_status);

    rc = opchg_masked_write(chip, BQ24157_BATTERY_VOL_REG, BQ24157_OTG_MASK, BQ24157_OTG_ENABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
    }
	else
	{
		dev_err(chip->dev, "bq24157_set_otg_regulator_enable\n");
	}

	return rc;
}

int bq24157_set_otg_regulator_disable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    opchg_config_charging_disable(chip, CHAGER_OTG_DISABLE, 0);
    opchg_set_charging_disable(chip, !!chip->disabled_status);

    rc = opchg_masked_write(chip, BQ24157_BATTERY_VOL_REG, BQ24157_OTG_MASK, BQ24157_OTG_DISABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	}
	else
	{
		dev_err(chip->dev, "bq24157_set_otg_regulator_disable\n");
	}

    return rc;
}

int bq24157_get_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    u8 reg = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_read_reg(chip, BQ24157_BATTERY_VOL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read OTG enable bit rc=%d\n", rc);
        return rc;
    }
	else
	{
		dev_err(chip->dev, "bq24157_get_otg_regulator_is_enable read OTG enable bit =%d\n", reg);
    }

    return (reg & BQ24157_OTG_MASK) ? 1 : 0;
}

#if 1
int bq24157_set_otg_enable(void)
{
    int rc = 0;

	if(opchg_chip == NULL)
	{
		return rc;
	}
	rc = opchg_masked_write(opchg_chip, BQ24157_BATTERY_VOL_REG, BQ24157_OTG_MASK, BQ24157_OTG_ENABLE);
    if (rc) {
        pr_debug("Couldn't enable  OTG mode rc=%d\n", rc);
    }
	else
	{
		pr_debug("bq24157_set_otg_enable\n");
	}

	return rc;
}

int bq24157_set_otg_disable(void)
{
    int rc = 0;

    if(opchg_chip == NULL)
	{
		return rc;
	}
	rc = opchg_masked_write(opchg_chip, BQ24157_BATTERY_VOL_REG, BQ24157_OTG_MASK, BQ24157_OTG_DISABLE);
    if (rc) {
        pr_debug("Couldn't disable OTG mode rc=%d\n", rc);
	}
	else
	{
		pr_debug("bq24157_set_otg_disable\n");
	}

    return rc;
}

int bq24157_get_otg_enable(void)
{
    int rc = 0;
    u8 reg = 0;

	if(opchg_chip == NULL)
	{
		return rc;
	}
    rc = opchg_read_reg(opchg_chip, BQ24157_BATTERY_VOL_REG, &reg);
    if (rc) {
        pr_debug("Couldn't read OTG enable bit rc=%d\n", rc);
		return 0;
    } else {
		pr_debug("bq24157_get_otg_enable read OTG enable bit =%d\n", reg);
		return (reg & BQ24157_OTG_MASK) ? 1 : 0;
    }
}

#endif

int bq24157_set_reset_charger(struct opchg_charger *chip, bool reset)
{
    int rc;

	chip->batt_pre_full = 0;
    chip->batt_full = 0;

    rc = opchg_masked_write(chip, BQ24157_TERM_FAST_CURRENT_REG, BQ24157_RESET_MASK,
                            reset ? BQ24157_RESET_ENABLE : BQ24157_RESET_NONE);
    if (rc < 0) {
		pr_err("Couldn't set REGISTER_RESET = %d, rc = %d\n", reset, rc);
	}

	return rc;
}

int bq24157_set_charging_disable(struct opchg_charger *chip, bool disable)
{
    int rc;

    rc = opchg_masked_write(chip, BQ24157_CTRL_REG, BQ24157_CHARGE_ENABLE_MASK,
                            disable ? BQ24157_CHARGE_DISABLE : BQ24157_CHARGE_ENABLE);
    if (rc < 0) {
		pr_err("Couldn't set CHG_ENABLE_BIT disable = %d, rc = %d\n", disable, rc);
	}
	else
	{
		pr_err(" set CHG_ENABLE_BIT disable = %d\n", disable);
	}

	return rc;
}

int bq24157_set_suspend_enable(struct opchg_charger *chip, bool enable)
{
    int rc;

    rc = opchg_masked_write(chip, BQ24157_CTRL_REG, BQ24157_SUSPEND_MASK,
                            enable ? BQ24157_SUSPEND_ENABLE : BQ24157_SUSPEND_DISABLE);
    if (rc < 0) {
		pr_err("Couldn't set CHG_SUSP_EN_BIT enable = %d, rc = %d\n", enable, rc);
	}

	return rc;
}

int bq24157_set_enable_volatile_writes(struct opchg_charger *chip)
{
    int rc=0;

	//need do nothing

    return rc;
}

int bq24157_set_fastchg_current(struct opchg_charger *chip, int ifast_mA)
{
	int rc;
    u8 value;

	pr_err("%s ibatmax:%d\n",__func__,ifast_mA);
	if(ifast_mA < BQ24157_FAST_CURRENT_OFFSET_MA){
		value = BQ24157_LOWCHG_CURRENT_ENABLE;
		rc = opchg_masked_write(chip, BQ24157_SPECIAL_CHARGER_VOL_REG,
			BQ24157_LOWCHG_CURRENT_MASK, value);
		return rc;
	}

	opchg_masked_write(chip, BQ24157_SPECIAL_CHARGER_VOL_REG,
			BQ24157_LOWCHG_CURRENT_MASK, BQ24157_LOWCHG_CURRENT_DISABLE);

	if(ifast_mA > BQ24157_FAST_CURRENT_MAX_MA) {
		ifast_mA = BQ24157_FAST_CURRENT_MAX_MA;
	}
	value = ((ifast_mA - BQ24157_FAST_CURRENT_OFFSET_MA) / BQ24157_FAST_CURRENT_STEP_MA) << 4;

	rc = opchg_masked_write(chip, BQ24157_TERM_FAST_CURRENT_REG, BQ24157_FAST_CURRENT_MASK, value);
    if (rc < 0) {
		pr_err("%s error\n",__func__);
	}

	return rc;
}

int bq24157_set_termchg_current(struct opchg_charger *chip, int iterm_current)
{
    u8 value;

	value = (iterm_current - BQ24157_TERM_CURRENT_MIN_MA)/BQ24157_TERM_CURRENT_STEP_MA;
	dev_dbg(chip->dev, "bq24157_set_termchg_current value=%d\n", value);

	return opchg_masked_write(chip, BQ24157_TERM_FAST_CURRENT_REG, BQ24157_TERM_CURRENT_MASK, value);
}

#define AICL_MAX_COUNT			30
int bq24157_iusbmax_set_noaicl(struct opchg_charger *chip, int iusbin_mA)
{
    u8 value;

	pr_err("%s iusbin_mA:%d\n",__func__,iusbin_mA);
	if(iusbin_mA < 500)
		value = BQ24157_IUSBMAX_100MA;
	else if(iusbin_mA < 800)
		value = BQ24157_IUSBMAX_500MA;
	else if(iusbin_mA == 800)
		value = BQ24157_IUSBMAX_800MA;
	else
		value = BQ24157_IUSBMAX_NOLIMIT;

	return opchg_masked_write(chip, BQ24157_CTRL_REG, BQ24157_IUSBMAX_MASK, value);
}

void bq24157_usb_plugout_check_whenaicl(struct opchg_charger *chip)
{
#if 0
	if(chip->chg_present && !chip->chg_present_real){
		msleep(500);
		if(chip->chg_present && !chip->chg_present_real){
			pr_err("%s handle\n",__func__);
			chip->chg_present = false;
			bq24157_usbin_valid_irq_handler(chip);
		}
	}
#endif
}

int bq24157_set_input_chg_current(struct opchg_charger *chip, int iusbin_mA, bool aicl_enable)
{
	u8 aicl_count = 0;
	int chg_vol=0,rc=0;

    chip->is_charger_det = 1;

	if (iusbin_mA <= 2)
        iusbin_mA = USB2_MIN_CURRENT_MA;
    else if (iusbin_mA <= USB2_MIN_CURRENT_MA)
        iusbin_mA = USB2_MAX_CURRENT_MA;
	pr_err("%s iusbin_mA:%d,aicl_current:%d,fast_current:%d,aicl_enable:%d new\n",__func__,
		iusbin_mA,chip->aicl_current,chip->max_fast_current[FAST_CURRENT_MIN],aicl_enable);

	if(iusbin_mA <= CURRENT_500MA){
		rc = bq24157_iusbmax_set_noaicl(chip, iusbin_mA);
		chip->is_charger_det = 0;
		return 0;
	}
	if((chip->aicl_current > 0) && (aicl_enable == false)){
		if(chip->aicl_current > iusbin_mA)
		{
			//when chip->aicl_current =2000; iusbin_mA = 1500
			rc = bq24157_iusbmax_set_noaicl(chip, iusbin_mA);
		}
		else
		{
			//when chip->aicl_current =1500; iusbin_mA = 2000 or iusbin_mA = 1500
			rc = bq24157_iusbmax_set_noaicl(chip, chip->aicl_current);
		}
		chip->is_charger_det = 0;
		return 0;
	}
	if((chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_COLD) ||
		(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_HOT)){
		pr_err("%s cold/hot,iusbin_mA:%d\n",__func__,iusbin_mA);
		rc = bq24157_iusbmax_set_noaicl(chip, iusbin_mA);
		chip->is_charger_det = 0;
		return 0;
	}

	rc = bq24157_iusbmax_set_noaicl(chip, CURRENT_500MA);
	msleep(20);
	opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);

	chip->aicl_working = true;
	rc = bq24157_iusbmax_set_noaicl(chip, CURRENT_500MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_500MA;
			rc = bq24157_iusbmax_set_noaicl(chip, chip->aicl_current);
			chip->aicl_working = false;
			pr_err("aicl end 500ma_1,chg_vol:%d\n",chg_vol);
			bq24157_usb_plugout_check_whenaicl(chip);
			chip->is_charger_det = 0;
			return 0;
		}
	}

	if(iusbin_mA < CURRENT_800MA){
		chip->aicl_current = CURRENT_500MA;
		rc = bq24157_iusbmax_set_noaicl(chip, chip->aicl_current);
		chip->aicl_working = false;
		chip->is_charger_det = 0;
		return 0;
	}

	rc = bq24157_iusbmax_set_noaicl(chip, CURRENT_800MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_500MA;
			rc = bq24157_iusbmax_set_noaicl(chip, chip->aicl_current);
			pr_err("aicl end 500ma_2,chg_vol:%d\n",chg_vol);
			bq24157_usb_plugout_check_whenaicl(chip);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}

	rc = bq24157_iusbmax_set_noaicl(chip, BQ24157_CURRENT_NOLIMIT);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_800MA;
			rc = bq24157_iusbmax_set_noaicl(chip, chip->aicl_current);
			pr_err("aicl end 500ma_3,chg_vol:%d\n",chg_vol);
			bq24157_usb_plugout_check_whenaicl(chip);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}
	chip->aicl_current = BQ24157_CURRENT_NOLIMIT;
	rc = bq24157_iusbmax_set_noaicl(chip, chip->aicl_current);
	chip->aicl_working = false;
    chip->is_charger_det = 0;
    return 0;
}

int bq24157_set_float_voltage(struct opchg_charger *chip, int vfloat_mv)
{
    u8 value;

	if(vfloat_mv < BQ24157_FLOAT_VOLTAGE_MIN_MV)
		vfloat_mv = BQ24157_FLOAT_VOLTAGE_MIN_MV;

	pr_err("%s vfloat_mv:%d\n",__func__,vfloat_mv);
	value = (vfloat_mv - BQ24157_FLOAT_VOLTAGE_MIN_MV) / BQ24157_FLOAT_VOLLTAGE_STEP_MV;
	value <<= BQ24157_FLOAT_VOLTAGE_SHIFT;

	return opchg_masked_write(chip, BQ24157_BATTERY_VOL_REG, BQ24157_FLOAT_VOLTAGE_MASK, value);
}

int bq24157_set_wdt_timer(struct opchg_charger *chip, bool enable)
{

    return 0;
}

int bq24157_set_vindpm_vol(struct opchg_charger *chip, int vol)
{
	int rc;
	u8 value = 0;

	value = (vol - BQ24157_VINDPM_OFFSET_MV) / BQ24157_VINDPM_STEP_MV;
    rc = opchg_masked_write(chip, BQ24157_SPECIAL_CHARGER_VOL_REG, BQ24157_VINDPM_MASK, value);

	return rc;
}

int bq24157_max_volt_current_set(struct opchg_charger *chip, int max_batt_vol, int max_current)
{
	int rc;
	u8 value_low = 0,value = 0;

	if(max_batt_vol > BQ24157_MAX_BATT_VOL_MAX_MV)
		max_batt_vol = BQ24157_MAX_BATT_VOL_MAX_MV;
	else if(max_batt_vol < BQ24157_MAX_BATT_VOL_MIN_MV)
		max_batt_vol = BQ24157_MAX_BATT_VOL_MIN_MV;

	value_low = (max_batt_vol - BQ24157_MAX_BATT_VOL_MIN_MV) / BQ24157_MAX_BATT_VOL_STEP_MV;
	value_low = value_low & 0x0F;
	if(max_current > BQ24157_MAX_FAST_CURRENT_MAX_MA)
		max_current = BQ24157_MAX_FAST_CURRENT_MAX_MA;
	value = ((max_current - BQ24157_MAX_FAST_CURRENT_OFFSET_MA) / BQ24157_MAX_FAST_CURRENT_STEP_MA)
				<< BQ24157_MAX_FAST_CURRENT_SHIFT;
	value = value |  value_low;
    rc = opchg_masked_write(chip, BQ24157_SAFETY_LIMIT_REG, BQ24157_MAX_VOLT_CURRENT_MASK, value);

	return rc;
}

int bq24157_set_wdt_reset(struct opchg_charger *chip)
{
    int rc = 0;


    return rc;
}

int bq24157_chg_term_enable(struct opchg_charger *chip, bool enable)
{
    int rc = 0;

    rc = opchg_masked_write(chip, BQ24157_CTRL_REG, BQ24157_TERM_ENABLE_MASK,
			enable ? BQ24157_TERM_ENABLE : 0x00);

    return rc;
}

int bq24157_weak_battery_vol_set(struct opchg_charger *chip, int weak_mv)
{
    int rc = 0;
	u8 value = 0;

	if(weak_mv > BQ24157_WEAK_BATTERY_MAX_MV)
		weak_mv = BQ24157_WEAK_BATTERY_MAX_MV;
	else if(weak_mv < BQ24157_WEAK_BATTERY_MIN_MV)
		weak_mv = BQ24157_WEAK_BATTERY_MIN_MV;

	value = (weak_mv - BQ24157_WEAK_BATTERY_MIN_MV) / BQ24157_WEAK_BATTERY_STEP_MV;
	value = value << BQ24157_WEAK_BATTERY_SHIFT;
    rc = opchg_masked_write(chip, BQ24157_CTRL_REG, BQ24157_WEAK_BATTERY_MASK, value);

    return rc;
}

int bq24157_check_battovp(struct opchg_charger *chip)
{
    int rc = 0;
	u8 value;

    if(chip->chg_present == true) {
		rc = opchg_read_reg(chip, BQ24157_STATUS_CTRL_REG,&value);
        if ((value & BQ24157_FAULT_STATUS_MASK) == BQ24157_BATTERY_OVP){
			chip->batt_ovp = 1;
        }
        else {
            chip->batt_ovp = 0;
        }
    }
    else {
        chip->batt_ovp = 0;
    }

    return rc;
}

int bq24157_check_charging_pre_full(struct opchg_charger *chip)
{
    int rc = 0;
	u8 value = 0;

    if(chip->chg_present == true) {
        rc = opchg_read_reg(chip, BQ24157_STATUS_CTRL_REG,&value);
        if ((value & BQ24157_CHARGE_STATUS_MASK) == BQ24157_CHARGE_TERM) {
            chip->batt_pre_full = 1;
        }
        else {
            chip->batt_pre_full = 0;
        }
    }
    else {
        chip->batt_pre_full = 0;
    }

    return rc;
}

int bq24157_hw_init(struct opchg_charger *chip)
{
	//don't write anything to BQ24157_STATUS_CTRL_REG(0x00) because of safety limit
	//bq24157_stat_func_enable(chip, false);
	bq24157_max_volt_current_set(chip, 4340, BQ24157_MAX_FAST_CURRENT_MAX_MA);	//it must be before set_float_voltage and set_fast_current
	opchg_set_float_voltage(chip, chip->min_term_voltage[TERM_VOL_MIN]);

	/* set the fast charge current limit */
	//opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);
	opchg_set_fast_chg_current(chip, BQ24157_FAST_CURRENT_OFFSET_MA);

    /* set iterm current*/
    bq24157_set_termchg_current(chip, chip->iterm_ma);

    bq24157_set_vindpm_vol(chip, 4440);

	bq24157_chg_term_enable(chip, true);

	bq24157_weak_battery_vol_set(chip, BQ24157_WEAK_BATTERY_MIN_MV);

    //opchg_config_charging_disable(chip, USER_DISABLE, !!chip->disabled_status);
    opchg_config_charging_disable(chip, USER_DISABLE, 0);
	return 0;
}

void bq24157_dump_regs(struct opchg_charger *chip)
{
	int rc;
	u8 reg, addr;

	// read status register
	for (addr = BQ24157_FIRST_REG; addr <= BQ24157_LAST_REG; addr++) {
		rc = opchg_read_reg(chip, addr, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
		else {
			pr_debug("bq24157_read_reg 0x%x = 0x%x\n", addr, reg);
		}
	}
}

int bq24157_chg_uv(struct opchg_charger *chip, u8 status)
{
    opchg_inout_charge_parameters(chip);
    opchg_set_reset_charger(chip, true);
    //msleep(10);
    opchg_hw_init(chip);

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
        dev_dbg(chip->dev, "%s charger is in updating usb_psy present=%d", __func__, chip->chg_present);
        power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
        power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

    if (status != 0) {
        chip->chg_present = false;
        dev_dbg(chip->dev, "%s charger is out updating usb_psy present=%d", __func__, chip->chg_present);
        /* we can't set usb_psy as UNKNOWN so early, it'll lead USERSPACE issue */
        power_supply_set_present(chip->usb_psy, chip->chg_present);
    }
    //chip->BMT_status.charger_exist = chip->chg_present;
    power_supply_changed(chip->usb_psy);

    dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

    return 0;
}

void bq24157_usbin_valid_work(struct work_struct *work)
{
	struct opchg_charger *chip = container_of(work,
                            struct opchg_charger, bq24157_usbin_valid_work);

	if(bq24157_get_otg_enable()){
		pr_err("%s otg enabled,return\n",__func__);
		return ;
	}
	bq24157_chg_uv(chip, !chip->chg_present);
	power_supply_changed(&chip->batt_psy);
}

void bq24157_usbin_valid_irq_handler(struct opchg_charger *chip)
{
	schedule_work(&chip->bq24157_usbin_valid_work);
}

int bq24157_get_initial_state(struct opchg_charger *chip)
{
    int rc = 0;
	u8 reg = 0;


	rc = opchg_read_reg(chip, BQ24157_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read BQ24188_STATUS_CTRL_REG rc = %d\n", rc);
        goto fail_init_status;
    }
	pr_err("%s reg:0x%x\n", __func__,reg);
	reg &= BQ24157_CHARGE_STATUS_MASK;
    if ((reg == BQ24157_CHARGE_IN_PROGRESS) || (reg == BQ24157_CHARGE_TERM)) {
        bq24157_chg_uv(chip, 0);
		dev_err(chip->dev, "oppo_charger_in_init\n");
	} else {
        bq24157_chg_uv(chip, 1);
		dev_err(chip->dev, "oppo_charger_out_init\n");
    }

    return rc;

fail_init_status:
    dev_err(chip->dev, "bq24157 couldn't get intial status\n");
    return rc;
}
