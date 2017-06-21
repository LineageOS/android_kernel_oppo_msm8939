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

#define OPPO_BQ24196_PAR
#include "oppo_inc.h"

void (*enable_aggressive_segmentation_fn)(bool);

static int bq24196_usbin_input_current_limit[] = {
    100,    150,    500,    900,
    1200,   1500,   2000,   3000,
};

int bq24196_get_prop_charge_type(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, REG08_BQ24196_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
    }

    reg &= REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK;

    if (reg == REG08_BQ24196_CHARGING_STATUS_FAST_CHARGING) {
        return POWER_SUPPLY_CHARGE_TYPE_FAST;
    }
    else if (reg == REG08_BQ24196_CHARGING_STATUS_PRE_CHARGING) {
        return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
    }
    else {
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }
}

int bq24196_get_prop_batt_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;
	static char vbus_low_count = 0;

	//because bq24196 dpm_vol is 4360mv,so vbus don't below 4300mv normally
	if(opchg_get_prop_charger_voltage_now(chip) < 4200){
		vbus_low_count++;
		if(vbus_low_count > 3){
			//dev_err(chip->dev, "vbus_vol lower than 4200\n,return discharging");
			return POWER_SUPPLY_STATUS_DISCHARGING;
		}
	} else {
		vbus_low_count = 0;
	}

    rc = opchg_read_reg(chip, REG08_BQ24196_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

	reg &= REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK;
	if ((reg == REG08_BQ24196_CHARGING_STATUS_FAST_CHARGING) || (reg == REG08_BQ24196_CHARGING_STATUS_PRE_CHARGING))
	{
        return POWER_SUPPLY_STATUS_CHARGING;
    }

    return POWER_SUPPLY_STATUS_DISCHARGING;
}

int bq24196_get_charging_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, REG01_BQ24196_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }
	else
	{
		if((reg & 0x20)&&chip->chg_present)
		{
			dev_err(chip->dev, "otg is disable vbus,read REG01_BQ24196_ADDRESS = %d\n", reg);
		}
	}

    return (reg & REG01_BQ24196_CHARGING_ENABLE) ? 1 : 0;
}

int bq24196_set_otg_regulator_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    opchg_config_charging_disable(chip, CHAGER_OTG_DISABLE, 1);
    opchg_set_charging_disable(chip, !!chip->disabled_status);

    rc = opchg_masked_write(chip, REG01_BQ24196_ADDRESS, REG01_BQ24196_OTG_MASK, REG01_BQ24196_OTG_ENABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
    }
	else
	{
		dev_err(chip->dev, "bq24196_set_otg_regulator_enable\n");
	}

	return rc;
}

int bq24196_set_otg_regulator_disable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    opchg_config_charging_disable(chip, CHAGER_OTG_DISABLE, 0);
    opchg_set_charging_disable(chip, !!chip->disabled_status);

    rc = opchg_masked_write(chip, REG01_BQ24196_ADDRESS, REG01_BQ24196_OTG_MASK, REG01_BQ24196_OTG_DISABLE);
    if (rc) {
        dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	}
	else
	{
		dev_err(chip->dev, "bq24196_set_otg_regulator_disable\n");
	}

    return rc;
}

int bq24196_get_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    u8 reg = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_read_reg(chip, REG01_BQ24196_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read OTG enable bit rc=%d\n", rc);
        return rc;
    }
	else
	{
		dev_err(chip->dev, "bq24196_get_otg_regulator_is_enable read OTG enable bit =%d\n", reg);
    }

    return (reg & REG01_BQ24196_OTG_MASK) ? 1 : 0;
}

#if 1
int bq24196_set_otg_enable(void)
{
    int rc = 0;

	if(opchg_chip == NULL)
	{
		return rc;
	}
    rc = opchg_masked_write(opchg_chip, REG01_BQ24196_ADDRESS, REG01_BQ24196_OTG_MASK, REG01_BQ24196_OTG_ENABLE);
    if (rc) {
        pr_debug("Couldn't enable  OTG mode rc=%d\n", rc);
    }
	else
	{
		pr_debug("bq24196_set_otg_enable\n");
	}

	return rc;
}

int bq24196_set_otg_disable(void)
{
    int rc = 0;

    if(opchg_chip == NULL)
	{
		return rc;
	}
    rc = opchg_masked_write(opchg_chip, REG01_BQ24196_ADDRESS, REG01_BQ24196_OTG_MASK, REG01_BQ24196_OTG_DISABLE);
    if (rc) {
        pr_debug("Couldn't disable OTG mode rc=%d\n", rc);
	}
	else
	{
		pr_debug("bq24196_set_otg_disable\n");
	}

    return rc;
}

int bq24196_get_otg_enable(void)
{
    int rc = 0;
    u8 reg = 0;

	if(opchg_chip == NULL)
	{
		return rc;
	}
    rc = opchg_read_reg(opchg_chip, REG01_BQ24196_ADDRESS, &reg);
    if (rc) {
        pr_debug("Couldn't read OTG enable bit rc=%d\n", rc);
    }
	else
	{
		pr_debug("bq24196_get_otg_enable read OTG enable bit =%d\n", reg);
    }

	return (reg & REG01_BQ24196_OTG_MASK) ? 1 : 0;
}
#endif

int bq24196_set_reset_charger(struct opchg_charger *chip, bool reset)
{
    int rc;

	// Resolve exceptions charging full 20141028
	chip->batt_pre_full = 0;
    chip->batt_full = 0;

    rc = opchg_masked_write(chip, REG01_BQ24196_ADDRESS, REG01_BQ24196_REGISTER_RESET_MASK,
                            reset ? REG01_BQ24196_REGISTER_RESET : REG01_BQ24196_REGISTER_KEEP);
    if (rc < 0) {
		pr_err("Couldn't set REGISTER_RESET = %d, rc = %d\n", reset, rc);
	}

	return rc;
}

int bq24196_set_charging_disable(struct opchg_charger *chip, bool disable)
{
    int rc;

    rc = opchg_masked_write(chip, REG01_BQ24196_ADDRESS, REG01_BQ24196_CHARGING_MASK,
                            disable ? REG01_BQ24196_CHARGING_DISABLE : REG01_BQ24196_CHARGING_ENABLE);
    if (rc < 0) {
		pr_err("Couldn't set CHG_ENABLE_BIT disable = %d, rc = %d\n", disable, rc);
	}
	else
	{
		pr_err(" set CHG_ENABLE_BIT disable = %d\n", disable);
	}

	return rc;
}

int bq24196_set_suspend_enable(struct opchg_charger *chip, bool enable)
{
    int rc;

    rc = opchg_masked_write(chip, REG00_BQ24196_ADDRESS, REG00_BQ24196_SUSPEND_MODE_MASK,
                            enable ? REG00_BQ24196_SUSPEND_MODE_ENABLE : 0);
    if (rc < 0) {
		pr_err("Couldn't set CHG_SUSP_EN_BIT enable = %d, rc = %d\n", enable, rc);
	}

	return rc;
}

int bq24196_set_enable_volatile_writes(struct opchg_charger *chip)
{
    int rc=0;

	//need do nothing

    return rc;
}

int bq24196_set_precharger_voltage(struct opchg_charger *chip)
{
	u8 reg;
	int rc=0;

	/* set precharge voltage*/
	reg = REG04_BQ24196_PRE_TO_FAST_CHARGING_VOL_3000MV;
	rc = opchg_masked_write(chip, REG04_BQ24196_ADDRESS, REG04_BQ24196_PRE_TO_FAST_CHARGING_VOL_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set recharging threshold rc = %d\n", rc);
		return rc;
	}
    return rc;
}

int bq24196_set_recharger_voltage(struct opchg_charger *chip)
{
	u8 reg;
	int rc=0;

	/* set recharge voltage*/
	if(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_COOL)
	{
		reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_300MV;
	}
	else
	{
		reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_100MV;
	}
	rc = opchg_masked_write(chip, REG04_BQ24196_ADDRESS, REG04_BQ24196_RECHARGING_THRESHOLD_VOL_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set recharging threshold rc = %d\n", rc);
		return rc;
	}

    return rc;
}

int bq24196_set_prechg_current(struct opchg_charger *chip, int ipre_mA)
{
    u8 value;

	value = (ipre_mA - BQ24196_MIN_PRE_CURRENT_MA)/BQ24196_PRE_CURRENT_STEP_MA;
	value <<= REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_SHIFT;

	return opchg_masked_write(chip, REG03_BQ24196_ADDRESS, REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_MASK, value);
}

int bq24196_set_fastchg_current(struct opchg_charger *chip, int ifast_mA)
{
    u8 value;

	if(ifast_mA < BQ24196_MIN_FAST_CURRENT_MA_ALLOWED){
		if(ifast_mA > BQ24196_MAX_FAST_CURRENT_MA_20_PERCENT)
			ifast_mA = BQ24196_MAX_FAST_CURRENT_MA_20_PERCENT;
		ifast_mA = ifast_mA * 5;
	    value = (ifast_mA - BQ24196_MIN_FAST_CURRENT_MA)/BQ24196_FAST_CURRENT_STEP_MA;
	    value <<= REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT;
	    value = value | REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_ENABLE;
	}
	else {
	    value = (ifast_mA - BQ24196_MIN_FAST_CURRENT_MA)/BQ24196_FAST_CURRENT_STEP_MA;
	    value <<= REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT;
	}

	return opchg_masked_write(chip, REG02_BQ24196_ADDRESS, REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_MASK | REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_MASK, value);
}

int bq24196_set_temrchg_current(struct opchg_charger *chip, int iterm_current)
{
    u8 value;

	value = (iterm_current - BQ24196_MIN_TERM_CURRENT_MA)/BQ24196_TERM_CURRENT_STEP_MA;
	value <<= REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_SHIFT;
	dev_dbg(chip->dev, "bq24196_set_temrchg_current value=%d\n", value);

	return opchg_masked_write(chip, REG03_BQ24196_ADDRESS, REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_MASK, value);
}

#define AICL_MAX_COUNT			30
void bq24196_iusbmax_set_noaicl(struct opchg_charger *chip, int iusbin_mA)
{
	int i = 0;

	//pr_err("%s iusbin_mA:%d\n",__func__,iusbin_mA);
	for (i = ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1; i >= 0; i--)
	{
		if (bq24196_usbin_input_current_limit[i] <= iusbin_mA) {
			break;
		}
		else if (i == 0) {
		    break;
		}
	}
	opchg_masked_write(chip, REG00_BQ24196_ADDRESS, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK, i << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT);
}

void bq24196_usb_plugout_check_whenaicl(struct opchg_charger *chip)
{
#if 0
	if(chip->chg_present && !chip->chg_present_real){
		msleep(500);
		if(chip->chg_present && !chip->chg_present_real){
			pr_err("%s handle\n",__func__);
			chip->chg_present = false;
			bq24196_usbin_valid_irq_handler(chip);
		}
	}
#endif
}

int bq24196_set_input_chg_current(struct opchg_charger *chip, int iusbin_mA, bool aicl_enable)
{
	u8 aicl_count = 0;
	int chg_vol;
	int iusbmax = 0;

    chip->is_charger_det = 1;

	if (iusbin_mA <= 2)
        iusbin_mA = USB2_MIN_CURRENT_MA;
    else if (iusbin_mA <= USB2_MIN_CURRENT_MA)
        iusbin_mA = USB2_MAX_CURRENT_MA;

	#ifdef OPCHARGER_DEBUG_ENABLE
	pr_err("%s iusbin_mA:%d,aicl_current:%d,fast_current:%d,max_current:%d,aicl_enable:%d\n",__func__,
		iusbin_mA,chip->aicl_current,chip->max_fast_current[FAST_CURRENT_MIN],chip->limit_current_max_ma,aicl_enable);
	#endif

	// set current < 500mA
	if(iusbin_mA <= CURRENT_500MA){
		bq24196_iusbmax_set_noaicl(chip, iusbin_mA);
		pr_err("%s no aicl 500ma,iusbin_mA:%d set input current is end \n",__func__,iusbin_mA);
		chip->is_charger_det = 0;
		return 0;
	}

	// The situation does not require adaptation
	if((chip->aicl_current > 0) && (aicl_enable == false)){
		if(chip->aicl_current > iusbin_mA)
		{
			//when chip->aicl_current =2000; iusbin_mA = 1500
			bq24196_iusbmax_set_noaicl(chip, iusbin_mA);
		}
		else
		{
			//when chip->aicl_current =1500; iusbin_mA = 2000 or iusbin_mA = 1500
			bq24196_iusbmax_set_noaicl(chip, chip->aicl_current);
		}
		chip->is_charger_det = 0;
		return 0;
	}

	bq24196_iusbmax_set_noaicl(chip, CURRENT_500MA);
	msleep(20);
	opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);
	chip->aicl_working = true;

	// set current  500mA
	bq24196_iusbmax_set_noaicl(chip, CURRENT_500MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_500MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24196_iusbmax_set_noaicl(chip, iusbmax);
			chip->aicl_working = false;
			pr_err("%s aicl end 500ma_1,chg_vol:%d,iusbmax:%d set input current is end\n",
					__func__,chg_vol,iusbmax);
			bq24196_usb_plugout_check_whenaicl(chip);
			chip->is_charger_det = 0;
			return 0;
		}
	}
	if(chip->limit_current_max_ma < CURRENT_900MA){
		chip->aicl_current = CURRENT_500MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24196_iusbmax_set_noaicl(chip, iusbmax);
		chip->aicl_working = false;
		pr_err("%s aicl end 500ma_2,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->is_charger_det = 0;
		return 0;
	}

	// set current < 900mA
	bq24196_iusbmax_set_noaicl(chip, CURRENT_900MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_500MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24196_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 500ma_3,chg_vol:%d,iusbmax:%d set input current is end\n",
					__func__,chg_vol,iusbmax);
			bq24196_usb_plugout_check_whenaicl(chip);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}
	if(chip->limit_current_max_ma < CURRENT_1200MA){
		chip->aicl_current = CURRENT_900MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24196_iusbmax_set_noaicl(chip, iusbmax);
		chip->aicl_working = false;
		pr_err("%s aicl end 900ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->is_charger_det = 0;
		return 0;
	}

	// add for 15109 compatible 1A and 2A charger
	if(is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__10||get_PCB_Version() == HW_VERSION__12||get_PCB_Version() == HW_VERSION__14||get_PCB_Version() == HW_VERSION__15))
	{
		//do nothing
	}
	else
	{
		// set current 1200mA
		bq24196_iusbmax_set_noaicl(chip, CURRENT_1200MA);
		msleep(90);
		for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
			chg_vol = opchg_get_prop_charger_voltage_now(chip);
			if(chg_vol < chip->sw_aicl_point - 50){
				chip->aicl_current = CURRENT_900MA;
				iusbmax = min(iusbin_mA, chip->aicl_current);
				bq24196_iusbmax_set_noaicl(chip, iusbmax);
				pr_err("%s aicl end 900ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",
						__func__,chg_vol,iusbmax);
				chip->aicl_working = false;
				bq24196_usb_plugout_check_whenaicl(chip);
				chip->is_charger_det = 0;
				return 0;
			}
		}

		if(chip->limit_current_max_ma < CURRENT_1500MA){
			chip->aicl_current = CURRENT_1200MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24196_iusbmax_set_noaicl(chip, iusbmax);
			chip->aicl_working = false;
			pr_err("%s aicl end 1200ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
			chip->is_charger_det = 0;
			return 0;
		}
	}

	// set current 1500mA
	bq24196_iusbmax_set_noaicl(chip, CURRENT_1500MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			if(is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__10||get_PCB_Version() == HW_VERSION__12||get_PCB_Version() == HW_VERSION__14||get_PCB_Version() == HW_VERSION__15))
			{
				chip->aicl_current = CURRENT_900MA;
			}
			else
			{
				chip->aicl_current = CURRENT_1200MA;
			}
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24196_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 1200ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",
					__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			bq24196_usb_plugout_check_whenaicl(chip);
			chip->is_charger_det = 0;
			return 0;
		}
	}
	if(chip->limit_current_max_ma < CURRENT_2000MA){
		chip->aicl_current = CURRENT_1500MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24196_iusbmax_set_noaicl(chip, iusbmax);
		chip->aicl_working = false;
		pr_err("%s aicl end 1500ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->is_charger_det = 0;
		return 0;
	}

	// set current 2000mA
	bq24196_iusbmax_set_noaicl(chip, CURRENT_2000MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_1500MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24196_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 1500ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",
					__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			bq24196_usb_plugout_check_whenaicl(chip);
			chip->is_charger_det = 0;
			return 0;
		}
	}
	chip->aicl_current = CURRENT_2000MA;
	iusbmax = min(iusbin_mA, chip->aicl_current);
	bq24196_iusbmax_set_noaicl(chip, iusbmax);
	chip->aicl_working = false;
	pr_err("%s aicl end 2000ma,iusbmax:%d set input current is end\n",__func__,iusbmax);
    chip->is_charger_det = 0;
    return 0;
}

int bq24196_set_float_voltage(struct opchg_charger *chip, int vfloat_mv)
{
    u8 value;

	value = (vfloat_mv - BQ24196_MIN_FLOAT_MV)/BQ24196_VFLOAT_STEP_MV;
	value <<= REG04_BQ24196_CHARGING_VOL_LIMIT_SHIFT;
	dev_dbg(chip->dev, "bq24196_set_float_voltage value=%d\n", value);

	return opchg_masked_write(chip, REG04_BQ24196_ADDRESS, REG04_BQ24196_CHARGING_VOL_LIMIT_MASK, value);
}

int bq24196_set_complete_charge_timeout(struct opchg_charger *chip, int val)
{
    int rc = 0;

    #ifdef OPPO_USE_TIMEOVER_BY_AP
	//bq24196 do not use itself's timeover function
	val = OVERTIME_DISABLED;
	#endif

    if (val == OVERTIME_AC){
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H;
    }
    else if (val == OVERTIME_USB){
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_12H;
    }
    else {
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_DISABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H;
    }

    rc = opchg_masked_write(chip, REG05_BQ24196_ADDRESS, REG05_BQ24196_CHARGING_SAFETY_TIME_MASK | REG05_BQ24196_FAST_CHARGING_TIMEOUT_MASK, val);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't complete charge timeout rc = %d\n", rc);
    }
	else
	{
		dev_err(chip->dev, "oppo set charger_timeout val = %d\n", val);
	}

    return rc;
}

int bq24196_set_wdt_timer(struct opchg_charger *chip, bool enable)
{
    int rc = 0;
	int value = 0;

    if(enable == true) {
		value = REG05_BQ24196_I2C_WATCHDOG_TIME_40S;
	}
	else
	{
		value=REG05_BQ24196_I2C_WATCHDOG_TIME_DISABLE;
	}
    rc = opchg_masked_write(chip, REG05_BQ24196_ADDRESS,REG05_BQ24196_I2C_WATCHDOG_TIME_MASK,value);

    return rc;
}

void bq24196_set_batfet_off(struct opchg_charger *chip)
{
	if (chip != NULL)
		opchg_masked_write(chip, REG07_BQ24196_ADDRESS, REG07_BQ24196_BATFET_MASK, REG07_BQ24196_BATFET_OFF);
}

int bq24196_set_vindpm_vol(struct opchg_charger *chip, int vol)
{
	int rc;
	u8 value = 0;

	value = (vol - BQ24196_VINDPM_OFFSET) / BQ24196_VINDPM_STEP_MV;
    rc = opchg_masked_write(chip, REG00_BQ24196_ADDRESS, BQ24196_VINDPM_MASK, value << BQ24196_VINDPM_SHIFT);
	return rc;
}

int bq24196_set_wdt_reset(struct opchg_charger *chip)
{
    int rc = 0;

    rc = opchg_masked_write(chip, REG01_BQ24196_ADDRESS,REG01_BQ24196_WDT_TIMER_RESET_MASK,REG01_BQ24196_WDT_TIMER_RESET);

    return rc;
}

int bq24196_check_charging_pre_full(struct opchg_charger *chip)
{
    int rc = 0;

    if(chip->chg_present == true) {
        rc = opchg_read_reg(chip, REG08_BQ24196_ADDRESS,&reg08_val);
        if ((reg08_val & REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK) == REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING) {
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

int bq24196_check_battovp(struct opchg_charger *chip)
{
    int rc = 0;

    if(chip->chg_present == true) {
		rc = opchg_read_reg(chip, REG09_BQ24196_ADDRESS,&reg09_val);
        if ((reg09_val & REG09_BQ24196_BATTERY_VOLATGE_MASK) == REG09_BQ24196_BATTERY_VOLATGE_HIGH_ERROR){
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

#ifdef BQ24192
int bq24196_set_compenstion_resistor(struct opchg_charger *chip, int reg)
{
    int rc = 0;

    rc = opchg_masked_write(chip, REG06_BQ24196_ADDRESS,REG06_BQ24196_COMPENSATION_RESISTOR_MASK,reg);

    return rc;
}

int bq24196_set_compenstion_voltage(struct opchg_charger *chip, int reg)
{
    int rc = 0;

    rc = opchg_masked_write(chip, REG06_BQ24196_ADDRESS,REG06_BQ24196_COMPENSATION_VOLTAGE_MASK,reg);

    return rc;
}
#endif

int bq24196_hw_init(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

	/* set the float voltage */
	opchg_set_float_voltage(chip, chip->min_term_voltage[TERM_VOL_MIN]);

	/* set the fast charge current limit */
	opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);

    /* set pre-charging current */
    rc = bq24196_set_prechg_current(chip, 300);

    /* set iterm current*/
    rc =bq24196_set_temrchg_current(chip, chip->iterm_ma);

    /* set recharge voltage*/
    if (chip->recharge_mv >= 300) {
        reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_300MV;
    }
	else
	{
        reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_100MV;
    }
    rc = opchg_masked_write(chip, REG04_BQ24196_ADDRESS, REG04_BQ24196_RECHARGING_THRESHOLD_VOL_MASK, reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't set recharging threshold rc = %d\n", rc);
        return rc;
    }
    bq24196_set_vindpm_vol(chip, 4440);
    /* enable/disable charging */
    //opchg_config_charging_disable(chip, USER_DISABLE, !!chip->disabled_status);
    opchg_config_charging_disable(chip, USER_DISABLE, 0);

    /* wdt timeout setting */ /*defuat 40s*/
    //rc = bq24196_set_wdt_timer(chip,REG05_BQ24196_I2C_WATCHDOG_TIME_DISABLE);

	#ifdef BQ24192
	/* compenstion resistor setting */
	rc = bq24196_set_compenstion_resistor(chip, REG06_BQ24196_COMPENSATION_RESISTOR_70MO);

	/* compenstion voltage setting */
	rc = bq24196_set_compenstion_voltage(chip, REG06_BQ24196_COMPENSATION_VOLTAGE_112MV);
	#endif

    return rc;
}

int bq24196_get_initial_state(struct opchg_charger *chip)
{
    int rc = 0;
	u8 reg = 0;

    rc = opchg_read_reg(chip, REG08_BQ24196_ADDRESS, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read REG08_BQ24196_ADDRESS rc = %d\n", rc);
        goto fail_init_status;
    }
    if ((reg & REG08_BQ24196_CHARGING_STATUS_POWER_MASK) == REG08_BQ24196_CHARGING_STATUS_POWER_IN) {
        bq24196_chg_uv(chip, 0);
		dev_err(chip->dev, "oppo_charger_in_init\n");
	}
	else {
        bq24196_chg_uv(chip, 1);
		dev_err(chip->dev, "oppo_charger_out_init\n");
    }
    pre_reg08_val = reg;

    return rc;

fail_init_status:
    dev_err(chip->dev, "bq24196 couldn't get intial status\n");
    return rc;
}

void bq24196_dump_regs(struct opchg_charger *chip)
{
	int rc;
	u8 reg, addr;

	// read config register
	for (addr = 0; addr <= BQ24196_LAST_CNFG_REG; addr++) {
		rc = opchg_read_reg(chip, addr, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
		else {
			pr_debug("bq24196_read_reg 0x%x = 0x%x\n", addr, reg);
		}
	}
	// read status register
	for (addr = BQ24196_FIRST_STATUS_REG; addr <= BQ24196_LAST_STATUS_REG; addr++) {
		rc = opchg_read_reg(chip, addr, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
		else {
			pr_debug("bq24196_read_reg 0x%x = 0x%x\n", addr, reg);
		}
	}
}

#ifdef OPPO_USE_FAST_CHARGER
static void reset_fastchg_after_usbout(struct opchg_charger *chip)
{
	int rc =0;

	if(opchg_get_prop_fast_chg_started(chip) == false) {
		pr_err("%s switch off fastchg\n", __func__);
			//opchg_set_switch_sleep(opchg_pinctrl_chip);
			rc =opchg_set_switch_mode(NORMAL_CHARGER_MODE);
	}
	rc =opchg_set_fast_switch_to_normal_false(chip);
	rc =opchg_set_fast_normal_to_warm_false(chip);
	//whenever charger gone, mask allo fast chg.
	rc =opchg_set_fast_chg_allow(chip,false);
	//vooc_start_step = 1;
	vooc_start_step = OPCHG_VOOC_TO_STANDARD;

	rc =opchg_get_fast_low_temp_full(chip);
}
#endif

int bq24196_chg_uv(struct opchg_charger *chip, u8 status)
{
	int chg_vol;
	bool chg_present_status = false;

	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;
		chg_present_status = true;
	}
	if (status != 0) {
		chip->chg_present = false;
		chg_present_status = false;
	}


    mutex_lock(&chip->usbin_lock);
	chg_vol = opchg_get_prop_charger_voltage_now(chip);
    opchg_inout_charge_parameters(chip);
    opchg_set_reset_charger(chip, true);
    //msleep(10);
    opchg_hw_init(chip);
    //opchg_config_input_chg_current(chip, INPUT_CURRENT_BY_POWER, 0);
    //opchg_set_input_chg_current(chip, chip->max_input_current[INPUT_CURRENT_MIN], true);

	#ifdef OPPO_USE_FAST_CHARGER
	if(is_project(OPPO_14005) || is_project(OPPO_15011)|| is_project(OPPO_15018) || is_project(OPPO_15022))
	{
		opchg_set_switch_mode(NORMAL_CHARGER_MODE);
	}
	#endif
	/*****************************************
	* External OVP circuit sets enable switch
	*****************************************/
	if((is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__10||get_PCB_Version() == HW_VERSION__12||get_PCB_Version() == HW_VERSION__14||get_PCB_Version() == HW_VERSION__15)))
	{
		opchg_switch_to_usbin(chip,!status);
	}

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
        //chip->chg_present = true;

#ifdef OPPO_USE_TP_ANTI_INTERFERENCE_14005
		if(is_project(OPPO_14005)){
			if(enable_aggressive_segmentation_fn)
				enable_aggressive_segmentation_fn(true);
		}
#endif

        dev_dbg(chip->dev, "%s charger is in updating usb_psy present=%d", __func__, chg_present_status);
        power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
        power_supply_set_present(chip->usb_psy, chg_present_status);
	}

    if (status != 0) {
        //chip->chg_present = false;

#ifdef OPPO_USE_TP_ANTI_INTERFERENCE_14005
		if(is_project(OPPO_14005)){
			if(enable_aggressive_segmentation_fn)
				enable_aggressive_segmentation_fn(false);
		}
#endif

#ifdef OPPO_USE_FAST_CHARGER
		if(is_project(OPPO_14005) || is_project(OPPO_15011)|| is_project(OPPO_15018) || is_project(OPPO_15022))
		{
			reset_fastchg_after_usbout(chip);
			opchg_config_charging_disable(chip, CHAGER_VOOC_DISABLE, 0);
		}
#endif
        dev_dbg(chip->dev, "%s charger is out updating usb_psy present=%d,chg_vol=%d", __func__, chg_present_status, chg_vol);
        /* we can't set usb_psy as UNKNOWN so early, it'll lead USERSPACE issue */
        power_supply_set_present(chip->usb_psy, chg_present_status);
    }
    mutex_unlock(&chip->usbin_lock);/*chaoying.chen@EXP.BaseDrv.charge,2015/08/10 add for usb */
    //chip->BMT_status.charger_exist = chip->chg_present;
    power_supply_changed(chip->usb_psy);

	if(is_project(OPPO_15018) || is_project(OPPO_15011) ||is_project(OPPO_15022)){
		schedule_work(&chip->opchg_modify_tp_param_work);
	}
    //dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

    return 0;
}

int bq24196_chg_ov(struct opchg_charger *chip, u8 status)
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

    return 0;
}

int bq24196_fast_chg(struct opchg_charger *chip, u8 status)
{
    power_supply_changed(&chip->batt_psy);
    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int bq24196_chg_term(struct opchg_charger *chip, u8 status)
{
    chip->batt_pre_full = !!status;//chip->batt_full = !!status;
    //power_supply_changed(&chip->batt_psy);
    if(status == 1)
	dev_dbg(chip->dev, "%s is charging full\n", __func__);
	else
		dev_dbg(chip->dev, "%s is charging \n", __func__);
    return 0;
}

int bq24196_safety_timeout(struct opchg_charger *chip, u8 status)
{
    if (status) {
		chip->charging_time_out = true;
	}
	else {
        chip->charging_time_out = false;
    }

    return 0;
}

#if 1//for 15029
void bq24196_chg_irq_handler(int irq, struct opchg_charger *chip)
{
	int rc;
	u8 changed;
	int handler_count = 0;

	//if (chip->is_charger_det) return;
	if(opchg_get_prop_fast_chg_started(chip) == true)
	{
		#ifdef OPCHARGER_DEBUG_FOR_FAST_CHARGER
		pr_err("oppo_charg is fast charging disenable read bq24196 IRQ\n");
		#endif
		return ;
	}

	rc = opchg_read_reg(chip, REG08_BQ24196_ADDRESS,&reg08_val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",REG08_BQ24196_ADDRESS, rc);
		//continue;
	}
	else {
		pr_debug("oppo check int reg0x%x = 0x%x\n", REG08_BQ24196_ADDRESS,reg08_val);
	}

	#if 0
	changed = ((reg08_val & REG08_BQ24196_CHARGING_STATUS_POWER_MASK) == REG08_BQ24196_CHARGING_STATUS_POWER_IN)
				^ ((pre_reg08_val & REG08_BQ24196_CHARGING_STATUS_POWER_MASK) == REG08_BQ24196_CHARGING_STATUS_POWER_IN);
	if (changed) {
		if((reg08_val & REG08_BQ24196_CHARGING_STATUS_POWER_MASK) == REG08_BQ24196_CHARGING_STATUS_POWER_IN) {
			handler_count++;
			rc = bq24196_chg_uv(chip, 0);
		}
		else {
			handler_count++;
			// if fast_charging not response usb init
		    if(is_project(OPPO_14005)) {
			//if(opchg_gpio.opchg_fastcharger !=1)
			if(opchg_get_prop_fast_chg_started(chip) == false){
			    rc = bq24196_chg_uv(chip, 1);
			}
			} else {
				rc = bq24196_chg_uv(chip, 1);
			}
		}
	}
	#else
	if((reg08_val & REG08_BQ24196_CHARGING_STATUS_POWER_MASK) == REG08_BQ24196_CHARGING_STATUS_POWER_IN) {
		handler_count++;
		if (chip->chg_present == false) {
			rc = bq24196_chg_uv(chip, 0);
		}
	}
	else {
		handler_count++;
		if (chip->chg_present == true) {
			// if fast_charging not response usb init
			if(is_project(OPPO_14005) || is_project(OPPO_15011) || is_project(OPPO_15018) || is_project(OPPO_15022)) {
				if(opchg_get_prop_fast_chg_started(chip) == false){
					rc = bq24196_chg_uv(chip, 1);
				}
			} else {
				rc = bq24196_chg_uv(chip, 1);
			}
		}
	}
	#endif

	/*changed = ((reg08_val & REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK) == REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING)
				^ ((pre_reg08_val & REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK) == REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING);
	if (changed) */
	{
		if ((reg08_val & REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK) == REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING) {
			handler_count++;
			rc = bq24196_chg_term(chip, 1);
		}
		else {
			handler_count++;
			rc = bq24196_chg_term(chip, 0);
		}
	}
	pre_reg08_val = reg08_val;

	rc = opchg_read_reg(chip, REG09_BQ24196_ADDRESS,&reg09_val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",REG09_BQ24196_ADDRESS, rc);
		//continue;
	}
	else {
		pr_debug("oppo check int reg0x%x = 0x%x\n", REG09_BQ24196_ADDRESS,reg09_val);
	}

	changed = ((reg09_val & REG09_BQ24196_CHARGING_MASK) == REG09_BQ24196_CHARGING_TIMEOUT_ERROR)
				^ ((pre_reg09_val & REG09_BQ24196_CHARGING_MASK) == REG09_BQ24196_CHARGING_TIMEOUT_ERROR);
	if (changed) {
		if ((reg09_val & REG09_BQ24196_CHARGING_MASK) == REG09_BQ24196_CHARGING_TIMEOUT_ERROR) {
			handler_count++;
			rc = bq24196_safety_timeout(chip, 1);
		}
		else {
			handler_count++;
			rc = bq24196_safety_timeout(chip, 0);
		}
	}
	pre_reg09_val = reg09_val;

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}
}
#endif

void bq24196_usbin_valid_work(struct work_struct *work)
{
	struct opchg_charger *chip = container_of(work,
                            struct opchg_charger, bq24196_usbin_valid_work);

	if(bq24196_get_otg_enable()){
		pr_err("%s otg enabled,return\n",__func__);
		return ;
	}
	bq24196_chg_uv(chip, !chip->chg_present);
	power_supply_changed(&chip->batt_psy);
}

void bq24196_usbin_valid_irq_handler(struct opchg_charger *chip)
{
	schedule_work(&chip->bq24196_usbin_valid_work);
}
