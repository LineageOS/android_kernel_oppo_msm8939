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

#define OPPO_BQ24188_PAR
#include "oppo_inc.h"

int bq24188_get_prop_charge_type(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

	rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
    }

    reg &= BQ24188_CHARGE_STATUS_MASK;
    if (reg == BQ24188_CHARGE_IN_PROGRESS) {
        return POWER_SUPPLY_CHARGE_TYPE_FAST;
    } else if(reg == BQ24188_CHARGE_DONE){
		return POWER_SUPPLY_CHARGE_TYPE_TERMINATE;
	}

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

int bq24188_get_prop_batt_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

	reg &= BQ24188_CHARGE_STATUS_MASK;
    if((reg == BQ24188_CHARGE_IN_PROGRESS) || (reg == BQ24188_CHARGE_DONE)) {
		return POWER_SUPPLY_STATUS_CHARGING;
	} else {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

int bq24188_get_charging_status(struct opchg_charger *chip)
{
    int rc;
    u8 reg = 0;

    rc = opchg_read_reg(chip, BQ24188_CTRL_REG, &reg);
    if(rc){
        dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }
	reg &= BQ24188_CHARGE_EN_MASK;
	if((reg == BQ24188_CHARGE_ENABLE) && chip->chg_present)
		return 1;
	else
		return 0;
}

int bq24188_set_otg_regulator_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    opchg_config_charging_disable(chip, CHAGER_OTG_DISABLE, 1);
    opchg_set_charging_disable(chip, !!chip->disabled_status);

    rc = opchg_masked_write(chip, BQ24188_STATUS_CTRL_REG, BQ24188_OTG_ENABLE_MASK, BQ24188_OTG_ENABLE);
    if(rc)
        dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		dev_err(chip->dev, "bq24188 set otg enable success\n");

	return rc;
}

int bq24188_set_otg_regulator_disable(struct regulator_dev *rdev)
{
    int rc = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    opchg_config_charging_disable(chip, CHAGER_OTG_DISABLE, 0);
    opchg_set_charging_disable(chip, !!chip->disabled_status);

    rc = opchg_masked_write(chip, BQ24188_STATUS_CTRL_REG, BQ24188_OTG_ENABLE_MASK, BQ24188_OTG_DISABLE);
    if(rc)
        dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		dev_err(chip->dev, "bq24188 set otg disable success\n");

	return rc;
}

int bq24188_get_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int rc = 0;
    u8 reg = 0;
    struct opchg_charger *chip = rdev_get_drvdata(rdev);

    rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read OTG enable bit rc=%d\n", rc);
        return rc;
    } else {
		dev_err(chip->dev, "bq24188_get_otg_regulator_is_enable read OTG enable bit =%d\n", reg);
    }

    return (reg & BQ24188_OTG_ENABLE_MASK) ? 1 : 0;
}

int bq24188_set_otg_enable(void)
{
    int rc = 0;

	if(opchg_chip == NULL)
		return rc;

    rc = opchg_masked_write(opchg_chip, BQ24188_STATUS_CTRL_REG, BQ24188_OTG_ENABLE_MASK, BQ24188_OTG_ENABLE);
    if(rc){
        pr_debug("Couldn't enable  OTG mode rc=%d\n", rc);
    } else {
		pr_debug("%s success\n",__func__);
	}

	return rc;
}

int bq24188_set_otg_disable(void)
{
    int rc = 0;

    if(opchg_chip == NULL)
		return rc;

	rc = opchg_masked_write(opchg_chip, BQ24188_STATUS_CTRL_REG, BQ24188_OTG_ENABLE_MASK, BQ24188_OTG_DISABLE);
    if(rc){
        pr_debug("Couldn't disable OTG mode rc=%d\n", rc);
	} else {
		pr_debug("bq24188_set_otg_disable\n");
	}

    return rc;
}

int bq24188_get_otg_enable(void)
{
    int rc = 0;
    u8 reg = 0;

	if(opchg_chip == NULL)
		return rc;

    rc = opchg_read_reg(opchg_chip, BQ24188_STATUS_CTRL_REG, &reg);
    if(rc){
        pr_debug("Couldn't read OTG enable bit rc=%d\n", rc);
    } else {
		pr_debug("bq24188_get_otg_enable read OTG enable bit =%d\n", reg);
    }

	return (reg & BQ24188_OTG_ENABLE_MASK) ? 1 : 0;
}

int bq24188_set_reset_charger(struct opchg_charger *chip, bool reset)
{
    int rc;

	// Resolve exceptions charging full 20141028
	chip->batt_pre_full = 0;
    chip->batt_full = 0;

    rc = opchg_masked_write(chip, BQ24188_CTRL_REG, BQ24188_RESET_MASK,
							reset ? BQ24188_REG_RESET : BQ24188_REG_NOT_RESET);
    if (rc < 0) {
		pr_err("Couldn't set reg_reset = %d, rc = %d\n", reset, rc);
	}

	return rc;
}

int bq24188_set_charging_disable(struct opchg_charger *chip, bool disable)
{
    int rc=0;

#if 0
	u8 value = 0;

	if(disable){
		opchg_read_reg(chip, BQ24188_VINDPM_STATUS_REG, &value);
		if((value & BQ24188_LOWCHG_ENABLE_MASK) == BQ24188_LOWCHG_ENABLE){
			bq24188_set_fastchg_current(chip, BQ24188_IBATMAX_MIN_MA);
		}
	}
#endif
    rc = opchg_masked_write(chip, BQ24188_CTRL_REG, BQ24188_CHARGE_EN_MASK,
                            disable ? BQ24188_CHARGE_DISABLE : BQ24188_CHARGE_ENABLE);
    if (rc < 0) {
		pr_err("Couldn't set CHG_ENABLE_BIT disable = %d, rc = %d\n", disable, rc);
	}
	else
	{
		pr_err(" set CHG_ENABLE_BIT disable = %d\n", disable);
	}

	return rc;
}

int bq24188_set_suspend_enable(struct opchg_charger *chip, bool enable)
{
    int rc;

    rc = opchg_masked_write(chip, BQ24188_CTRL_REG, BQ24188_SUSPEND_MASK,
                            enable ? BQ24188_SUSPEND_ENABLE : BQ24188_SUSPEND_DISABLE);
    if (rc < 0) {
		pr_err("Couldn't set CHG_SUSP_EN_BIT enable = %d, rc = %d\n", enable, rc);
	}

	return rc;
}

int bq24188_set_enable_volatile_writes(struct opchg_charger *chip)
{
    int rc=0;

	//need do nothing

    return rc;
}

int bq24188_set_prechg_current(struct opchg_charger *chip, int ipre_mA)
{
	return 0;
}

int bq24188_set_fastchg_current(struct opchg_charger *chip, int ifast_mA)
{
    u8 value=0;
	int rc=0;

	if((is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__11||get_PCB_Version() == HW_VERSION__13)))
	{
		if((!chip->batt_authen) && (ifast_mA > chip->non_standard_fastchg_current_ma))
			ifast_mA = chip->non_standard_fastchg_current_ma;
	}
	pr_debug("%s ibatmax:%d\n",__func__,ifast_mA);

	if(ifast_mA < BQ24188_IBATMAX_MIN_MA){
		rc=opchg_masked_write(chip, BQ24188_VINDPM_STATUS_REG, BQ24188_LOWCHG_ENABLE_MASK, BQ24188_LOWCHG_ENABLE);
	} else {
		rc=opchg_masked_write(chip, BQ24188_VINDPM_STATUS_REG, BQ24188_LOWCHG_ENABLE_MASK, BQ24188_LOWCHG_DISABLE);
		value = ((ifast_mA - BQ24188_IBATMAX_MIN_MA) / BQ24188_IBATMAX_STEP_MA) << 3;
		rc=opchg_masked_write(chip, BQ24188_CHG_CURRENT_CTRL, BQ24188_IBATMAX_MASK, value);
	}
	return rc;
}

int bq24188_set_temrchg_current(struct opchg_charger *chip, int iterm_current)
{
    u8 value;

	value = (iterm_current - BQ24188_TERM_CURRENT_MIN_MA) / BQ24188_TERM_CURRENT_STEP_MA;

	return opchg_masked_write(chip, BQ24188_CHG_CURRENT_CTRL, BQ24188_TERM_CURRENT_MASK, value);
}

bool bq24188_is_vindpm_active(struct opchg_charger *chip)
{
	u8 value = 0;

	value = opchg_read_reg(chip, BQ24188_VINDPM_STATUS_REG, &value);
	return (value & BQ24188_VINDPM_STATUS_MASK) ? 1 : 0;
}

int bq24188_iusbmax_set_noaicl(struct opchg_charger *chip, int iusbin_mA)
{
	u8 value = 0;

	//pr_err("%s iusbmax:%d\n",__func__,iusbin_mA);

	if(iusbin_mA >= BQ24188_IUSBMAX_MAX)
		value = BQ24188_IUSBMAX_DCP_2500MA;
	else if(iusbin_mA >= 2000)
		value = BQ24188_IUSBMAX_DCP_2000MA;
	else if(iusbin_mA >= 1950)
		value = BQ24188_IUSBMAX_DCP_1950MA;
	else if(iusbin_mA >= 1500)
		value = BQ24188_IUSBMAX_DCP_1500MA;
	else if(iusbin_mA >= 900)
		value = BQ24188_IUSBMAX_USB3D0_DCP_900MA;
	else if(iusbin_mA >= 500)
		value = BQ24188_IUSBMAX_USB2D0_500MA;
	else if(iusbin_mA >= 150)
		value = BQ24188_IUSBMAX_USB3D0_150MA;
	else
		value = BQ24188_IUSBMAX_USB2D0_100MA;

	return opchg_masked_write(chip, BQ24188_CTRL_REG, BQ24188_IUSBMAX_MASK, value);
}

#define AICL_MAX_COUNT			30

int bq24188_set_input_chg_current_aicl(struct opchg_charger *chip, int iusbin_mA, bool aicl_enable)
{
	int chg_vol;
	u8 aicl_count = 0;
	int iusbmax = 0;

    chip->is_charger_det = 1;

	if (iusbin_mA <= 2)
        iusbin_mA = USB2_MIN_CURRENT_MA;
    else if (iusbin_mA <= USB2_MIN_CURRENT_MA)
        iusbin_mA = USB2_MAX_CURRENT_MA;

	pr_err("%s iusbin_mA:%d,aicl_current:%d,fast_current:%d,max_current:%d,aicl_enable:%d\n",__func__,
		iusbin_mA,chip->aicl_current,chip->max_fast_current[FAST_CURRENT_MIN],chip->limit_current_max_ma,aicl_enable);

	if(iusbin_mA <= CURRENT_500MA){
		bq24188_iusbmax_set_noaicl(chip, iusbin_mA);
		pr_err("%s no aicl 500ma,iusbin_mA:%d set input current is end\n",__func__,iusbin_mA);
		chip->is_charger_det = 0;
		return 0;
	}

	if((chip->aicl_current > 0) && (aicl_enable == false)){
		if(chip->aicl_current > iusbin_mA)
		{
			//when chip->aicl_current =2000; iusbin_mA = 1500
			bq24188_iusbmax_set_noaicl(chip, iusbin_mA);
		}
		else
		{
			//when chip->aicl_current =1500; iusbin_mA = 2000 or iusbin_mA = 1500
			bq24188_iusbmax_set_noaicl(chip, chip->aicl_current);
		}
		chip->is_charger_det = 0;
		return 0;
	}

	bq24188_iusbmax_set_noaicl(chip, CURRENT_500MA);
	msleep(20);
	opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);
	chip->aicl_working = true;

	bq24188_iusbmax_set_noaicl(chip, CURRENT_500MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_500MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24188_iusbmax_set_noaicl(chip, iusbmax);
			chip->aicl_working = false;
			pr_err("%s aicl end 500ma_1,chg_vol:%d,iusbmax:%d set input current is end\n",__func__,chg_vol,iusbmax);
			chip->is_charger_det = 0;
			return 0;
		}
	}
	if(chip->limit_current_max_ma < CURRENT_900MA){
		chip->aicl_current = CURRENT_500MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24188_iusbmax_set_noaicl(chip, iusbmax);
		chip->aicl_working = false;
		chip->is_charger_det = 0;
		pr_err("%s aicl end 500ma_2,iusbmax:%d set input current is end\n",__func__,iusbmax);
		return 0;
	}

	bq24188_iusbmax_set_noaicl(chip, CURRENT_900MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_500MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24188_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 500ma_3,chg_vol:%d,iusbmax:%d set input current is end\n",__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}

	if(chip->limit_current_max_ma < CURRENT_1500MA){
		chip->aicl_current = CURRENT_900MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24188_iusbmax_set_noaicl(chip, iusbmax);
		pr_err("%s aicl end 900ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->aicl_working = false;
		chip->is_charger_det = 0;
		return 0;
	}

	bq24188_iusbmax_set_noaicl(chip, CURRENT_1500MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_900MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24188_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 900ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}
	if(chip->limit_current_max_ma < CURRENT_1950MA){
		chip->aicl_current = CURRENT_1500MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24188_iusbmax_set_noaicl(chip, iusbmax);
		pr_err("%s aicl end 1500ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->aicl_working = false;
		chip->is_charger_det = 0;
		return 0;
	}

	bq24188_iusbmax_set_noaicl(chip, CURRENT_1950MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_1500MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24188_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 1500ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}

	if(chip->limit_current_max_ma < CURRENT_2000MA){
		chip->aicl_current = CURRENT_1950MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24188_iusbmax_set_noaicl(chip, iusbmax);
		pr_err("%s aicl end 1950ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->aicl_working = false;
		chip->is_charger_det = 0;
		return 0;
	}

	bq24188_iusbmax_set_noaicl(chip, CURRENT_2000MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_1950MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24188_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 1950ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}

	if(chip->limit_current_max_ma < CURRENT_2500MA){
		chip->aicl_current = CURRENT_2000MA;
		iusbmax = min(iusbin_mA, chip->aicl_current);
		bq24188_iusbmax_set_noaicl(chip, iusbmax);
		pr_err("%s aicl end 2000ma_1,iusbmax:%d set input current is end\n",__func__,iusbmax);
		chip->aicl_working = false;
		chip->is_charger_det = 0;
		return 0;
	}

	bq24188_iusbmax_set_noaicl(chip, CURRENT_2500MA);
	msleep(90);
	for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++){
		chg_vol = opchg_get_prop_charger_voltage_now(chip);
		if(chg_vol < chip->sw_aicl_point){
			chip->aicl_current = CURRENT_2000MA;
			iusbmax = min(iusbin_mA, chip->aicl_current);
			bq24188_iusbmax_set_noaicl(chip, iusbmax);
			pr_err("%s aicl end 2000ma_2,chg_vol:%d,iusbmax:%d set input current is end\n",__func__,chg_vol,iusbmax);
			chip->aicl_working = false;
			chip->is_charger_det = 0;
			return 0;
		}
	}

	chip->aicl_current = CURRENT_2500MA;
	iusbmax = min(iusbin_mA, chip->aicl_current);
	bq24188_iusbmax_set_noaicl(chip, iusbmax);
	chip->aicl_working = false;
    chip->is_charger_det = 0;
	pr_err("%s aicl end 2500ma,iusbmax:%d set input current is end\n",__func__,iusbmax);
    return 0;
}

int bq24188_set_input_chg_current_noaicl(struct opchg_charger *chip, int iusbin_mA, bool aicl_enable)
{
    chip->is_charger_det = 1;

	if (iusbin_mA <= 2)
        iusbin_mA = USB2_MIN_CURRENT_MA;
    else if (iusbin_mA <= USB2_MIN_CURRENT_MA)
        iusbin_mA = USB2_MAX_CURRENT_MA;

	pr_err("%s iusbin_mA:%d\n",__func__,iusbin_mA);
	bq24188_iusbmax_set_noaicl(chip, iusbin_mA);
	chip->is_charger_det = 0;
	return 0;
}

int bq24188_set_input_chg_current(struct opchg_charger *chip, int iusbin_mA, bool aicl_enable)
{
	int rc =0;
	int aicl_count = 0,chg_vol = 0;


	// Use bq24188 project uses an hardware adaptive , canceled software adaptation
	if(is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__11||get_PCB_Version() == HW_VERSION__13))
	{
		if((chip->aicl_current > 0) && (aicl_enable == false))
		{
			if(chip->aicl_current > iusbin_mA)
			{
				//when chip->aicl_current =2000; iusbin_mA = 1500
				rc = bq24188_set_input_chg_current_noaicl(chip, iusbin_mA, aicl_enable);
			}
			else
			{
				//when chip->aicl_current =1500; iusbin_mA = 2000 or iusbin_mA = 1500
				rc = bq24188_set_input_chg_current_noaicl(chip, chip->aicl_current, aicl_enable);
			}
		}
		else
		{
			if(iusbin_mA > 1000)
			{
				opchg_set_fast_chg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);
				rc =bq24188_iusbmax_set_noaicl(chip, CURRENT_1500MA);
				chip->aicl_current =1500;
				msleep(90);

				for(aicl_count = 0;aicl_count < AICL_MAX_COUNT;aicl_count++)
				{
					chg_vol =opchg_get_prop_charger_voltage_now(chip);
					if(chg_vol <= 4700)
					{
						chip->aicl_current =1000;
						pr_err("%s chg_vol=%d,chip->aicl_current=%d\n",__func__,chg_vol,chip->aicl_current);

						rc =bq24188_set_input_chg_current_noaicl(chip, chip->aicl_current, aicl_enable);
						return 0;
					}
				}
				pr_err("%s  chg_vol=%d,chip->aicl_current=%d\n",__func__,chg_vol,chip->aicl_current);
			}
			else
			{
				chip->aicl_current =iusbin_mA;
				rc =bq24188_set_input_chg_current_noaicl(chip, iusbin_mA, aicl_enable);
			}
		}
	}
	else
	{
		rc =bq24188_set_input_chg_current_aicl(chip, iusbin_mA, aicl_enable);
	}
	return rc;
}

int bq24188_set_float_voltage(struct opchg_charger *chip, int vfloat_mv)
{
    u8 value;

	if((is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__11||get_PCB_Version() == HW_VERSION__13)))
	{
		if((!chip->batt_authen) && (vfloat_mv > chip->non_standard_vfloat_mv))
			vfloat_mv = chip->non_standard_vfloat_mv;
	}
	pr_err("%s vddmax:%d\n",__func__,vfloat_mv);
	value = (vfloat_mv - BQ24188_VDDMAX_MIN_MV)/BQ24188_VDDMAX_STEP_MV;
	value <<= BQ24188_VDDMAX_SHIFT;

	return opchg_masked_write(chip, BQ24188_BATT_VOL_CTRL, BQ24188_VDDMAX_MASK, value);
}

int bq24188_set_vindpm_vol(struct opchg_charger *chip, int vol)
{
	int rc;
	u8 value = 0;

	if(vol < BQ24188_VINDPM_OFFSET)
		value = 0;
	else
		value = (vol - BQ24188_VINDPM_OFFSET) / BQ24188_VINDPM_STEP_MV;
    rc = opchg_masked_write(chip, BQ24188_VINDPM_STATUS_REG, BQ24188_VINDPM_MASK, value);

	return rc;
}

int bq24188_set_complete_charge_timeout(struct opchg_charger *chip, int val)
{
    int rc = 0;
	u8 value = 0;

	if(val == OVERTIME_AC)
		value = BQ24188_TIMEOUT_9HOURS;
	else if(val == OVERTIME_USB)
		value = BQ24188_TIMEOUT_9HOURS;
	else
		value = BQ24188_TIMEOUT_DISABLED;

    rc = opchg_masked_write(chip, BQ24188_TIMEOUT_NTC_REG, BQ24188_TIMEOUT_MASK, value);
    if (rc < 0) {
        dev_err(chip->dev, "Couldn't complete charge timeout rc = %d\n", rc);
    } else {
		dev_err(chip->dev, "oppo set charger_timeout val = %d\n", value);
	}

    return rc;
}

int bq24188_set_wdt_timer(struct opchg_charger *chip, bool enable)
{
    return 0;
}

int bq24188_set_wdt_reset(struct opchg_charger *chip)
{
    int rc = 0;

    rc = opchg_masked_write(chip, BQ24188_STATUS_CTRL_REG, BQ24188_WDT_KICK_MASK,BQ24188_WDT_KICK);

    return rc;
}

int bq24188_ts_enable(struct opchg_charger *chip, bool enable)
{
	int rc = 0;

	rc = opchg_masked_write(chip, BQ24188_TIMEOUT_NTC_REG, BQ24188_TS_ENABLE_MASK,
							enable ? BQ24188_TS_ENABLE : BQ24188_TS_DISABLE);
	return rc;
}

#define COUNT_20S		3
bool bq24188_sw_eoc_check(struct opchg_charger *chip,int vfloat)
{
#if 0
	int max_comp_volt,vbat_mv,ibat_ma;

	max_comp_volt = chip->temp_cool_vfloat_mv - 200;
	vbat_mv = opchg_get_prop_battery_voltage_now(chip) / 1000;
	ibat_ma = opchg_get_prop_current_now(chip);
	pr_err("max_comp_volt:%d,vbat_mv:%d,ibat_ma:%d,sw_eoc_count:%d\n",max_comp_volt,vbat_mv,ibat_ma,chip->sw_eoc_count);
	if((vbat_mv > max_comp_volt) && ((ibat_ma * -1) < chip->iterm_ma)){
		chip->sw_eoc_count++;
		if(chip->sw_eoc_count > 12){
			chip->sw_eoc_count = 13;
			opchg_config_charging_disable(chip, CHAGER_RECHARGER_DISABLE, 1); // disable charging
			return 1;
		} else {
			return 0;
		}
	} else {
		chip->sw_eoc_count = 0;
		return 0;
	}
#else
	int vbat_mv;
	static int check_count = 0;

	if(chip->sw_eoc_count > 2){
		return true;
	}
	check_count++;
	if(check_count > COUNT_20S){
		check_count = 0;
		bq24188_set_charging_disable(chip, true);
		msleep(1000);
		vbat_mv = opchg_get_prop_battery_voltage_now(chip) / 1000;
		pr_err("vbat_mv:%d,sw_eoc_count:%d\n", vbat_mv,chip->sw_eoc_count);

		bq24188_set_charging_disable(chip, false);

		if(vbat_mv > vfloat) {
			chip->sw_eoc_count++;
			if(chip->sw_eoc_count > 2){
				chip->sw_eoc_count = 3;
				opchg_config_charging_disable(chip, CHAGER_RECHARGER_DISABLE, 1); // disable charging
				return true;
			} else {
				return false;
			}
		} else {
			chip->sw_eoc_count = 0;
			return false;
		}
	} else {
		return false;
	}
#endif
}

bool bq24188_hw_eoc_check(struct opchg_charger *chip)
{
	u8 value = 0;

	opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG,&value);
	opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG, &value);
	value &= BQ24188_CHARGE_STATUS_MASK;
	if(value == BQ24188_CHARGE_DONE)
		return 1;
	else
		return 0;
}

int bq24188_check_battovp(struct opchg_charger *chip)
{
    int rc = 0;
	u8 value;

    if(chip->chg_present == true) {
		rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG,&value);
        if ((value & BQ24188_FAULT_STATUS_MASK) == BQ24188_BATTERY_OVP){
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

int bq24188_check_chargerovp(struct opchg_charger *chip)
{
    int rc = 0;
	u8 value;

    if(chip->chg_present == true) {
		rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG,&value);
        if ((value & BQ24188_FAULT_STATUS_MASK) == BQ24188_CHARGER_OVP){
            return 1;
        }
        else {
            return 0;
        }
    }
    else {
        return 0;
    }
}

int bq24188_check_charging_pre_full(struct opchg_charger *chip)
{
    if(chip->chg_present == true) {
		if(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_COOL)
		{
			chip->batt_pre_full = bq24188_sw_eoc_check(chip,3850);
		}
		else if(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_PRE_COOL1)
		{
			chip->batt_pre_full = bq24188_sw_eoc_check(chip,4250);
		}
		else if(chip->charging_opchg_temp_statu == OPCHG_CHG_TEMP_PRE_COOL)
		{
			if(chip->bat_instant_vol > TREM_4180MV)
			{
				chip->batt_pre_full = bq24188_sw_eoc_check(chip,4280);
			}
			else
			{
				chip->batt_pre_full = bq24188_hw_eoc_check(chip);
			}
		}
		else
		{
			chip->batt_pre_full = bq24188_hw_eoc_check(chip);
		}
    }
    else {
        chip->batt_pre_full = 0;
    }

    return 0;
}

int bq24188_stat_func_enable(struct opchg_charger *chip, bool enable)
{
	int rc = 0;

    rc = opchg_masked_write(chip, BQ24188_CTRL_REG,BQ24188_STAT_FUNC_MASK,
							enable ? BQ24188_STAT_FUNC_ENABLE : BQ24188_STAT_FUNC_DISABLE);

    return rc;
}

int bq24188_hw_init(struct opchg_charger *chip)
{
	bq24188_set_wdt_reset(chip);
	bq24188_stat_func_enable(chip, false);
	bq24188_set_vindpm_vol(chip, 4452);
	bq24188_set_float_voltage(chip, chip->min_term_voltage[TERM_VOL_MIN]);
	bq24188_set_fastchg_current(chip, chip->max_fast_current[FAST_CURRENT_MIN]);
	bq24188_set_temrchg_current(chip, chip->iterm_ma);
	bq24188_set_complete_charge_timeout(chip, OVERTIME_DISABLED);
	bq24188_set_charging_disable(chip,false);	//it is must,because bq24188 charging_enable is 0 after reset
	//opchg_config_charging_disable(chip, USER_DISABLE, !!chip->disabled_status);
	opchg_config_charging_disable(chip, USER_DISABLE, 0);
	opchg_config_charging_disable(chip, CHAGER_VOOC_DISABLE, 0);
	bq24188_set_wdt_reset(chip);
	return 0;
}

int bq24188_get_initial_state(struct opchg_charger *chip)
{
    int rc = 0;
	u8 reg = 0;

    rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG, &reg);
	rc = opchg_read_reg(chip, BQ24188_STATUS_CTRL_REG, &reg);
    if (rc) {
        dev_err(chip->dev, "Couldn't read BQ24188_STATUS_CTRL_REG rc = %d\n", rc);
        goto fail_init_status;
    }
	reg &= BQ24188_FAULT_STATUS_MASK;
    if ((reg == BQ24188_POWER_GOOD) || (reg == BQ24188_BATTERY_OVP)) {
        bq24188_chg_uv(chip, 0);
		dev_err(chip->dev, "oppo_charger_in_init\n");
	} else {
        bq24188_chg_uv(chip, 1);
		dev_err(chip->dev, "oppo_charger_out_init\n");
    }

    return rc;

fail_init_status:
    dev_err(chip->dev, "bq24188 couldn't get intial status\n");
    return rc;
}

void bq24188_dump_regs(struct opchg_charger *chip)
{
	int rc;
	u8 addr,value;

	for(addr = BQ24188_FIRST_REG;addr <= BQ24188_LAST_REG;addr++){
		rc = opchg_read_reg(chip, addr, &value);
		if(rc < 0){
			pr_err("%s read_reg fail,rc:%d\n",__func__,rc);
			return ;
		} else {
			pr_err("bq24188 reg[%d]:0x%x\n",addr,value);
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

int bq24188_chg_uv(struct opchg_charger *chip, u8 status)
{

    opchg_inout_charge_parameters(chip);
    opchg_set_reset_charger(chip, true);
    opchg_hw_init(chip);

	#ifdef OPPO_USE_FAST_CHARGER
	if(is_project(OPPO_14005) || is_project(OPPO_15011))
	{
		opchg_set_switch_mode(NORMAL_CHARGER_MODE);
	}
	#endif

	if((is_project(OPPO_15109) && (get_PCB_Version() == HW_VERSION__11||get_PCB_Version() == HW_VERSION__13)))
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
        chip->chg_present = true;

        dev_dbg(chip->dev, "%s charger is in updating usb_psy present=%d", __func__, chip->chg_present);
        power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
        power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

    if (status != 0) {
        chip->chg_present = false;

#ifdef OPPO_USE_FAST_CHARGER
		if(is_project(OPPO_14005) || is_project(OPPO_15011))
		{
			reset_fastchg_after_usbout(chip);
			opchg_config_charging_disable(chip, CHAGER_VOOC_DISABLE, 0);
		}
#endif
        dev_dbg(chip->dev, "%s charger is out updating usb_psy present=%d", __func__, chip->chg_present);
        /* we can't set usb_psy as UNKNOWN so early, it'll lead USERSPACE issue */
        power_supply_set_present(chip->usb_psy, chip->chg_present);
    }
    power_supply_changed(chip->usb_psy);
    dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

    return 0;
}


int bq24188_fast_chg(struct opchg_charger *chip, u8 status)
{
    power_supply_changed(&chip->batt_psy);
    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int bq24188_chg_term(struct opchg_charger *chip, u8 status)
{
    chip->batt_pre_full = !!status;//chip->batt_full = !!status;
    //power_supply_changed(&chip->batt_psy);

    dev_dbg(chip->dev, "%s\n", __func__);
    return 0;
}

int bq24188_safety_timeout(struct opchg_charger *chip, u8 status)
{
    if (status) {
		chip->charging_time_out = true;
	}
	else {
        chip->charging_time_out = false;
    }

    return 0;
}

void bq24188_usbin_valid_work(struct work_struct *work)
{
	struct opchg_charger *chip = container_of(work,
                            struct opchg_charger, bq24188_usbin_valid_work);

	if(bq24188_get_otg_enable()){
		pr_err("%s otg enabled,return\n",__func__);
		return ;
	}
	bq24188_chg_uv(chip, !chip->chg_present);
	power_supply_changed(&chip->batt_psy);
}

void bq24188_usbin_valid_irq_handler(struct opchg_charger *chip)
{
	schedule_work(&chip->bq24188_usbin_valid_work);
}
