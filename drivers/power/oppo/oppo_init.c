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

#define OPPO_INIT_PAR
#include <oppo_inc.h>


int opchg_parse_dt(struct opchg_charger *chip)
{
    #ifdef OPPO_USE_QCOMM
    int rc;
    enum of_gpio_flags gpio_flags;
    struct device_node *node = chip->dev->of_node;
    int batt_cold_degree_negative;
    int batt_present_degree_negative;
    #elif defined(OPPO_USE_MTK)
    int rc;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    if (!node) {
        dev_dbg(chip->dev, "device tree info. missing\n");
        return -EINVAL;
    }
    #endif
    
    #ifdef OPPO_USE_QCOMM
    chip->charging_disabled = of_property_read_bool(node, "qcom,charging-disabled");
    #elif defined(OPPO_USE_MTK)
    chip->charging_disabled = false;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    chip->chg_autonomous_mode = of_property_read_bool(node, "qcom,chg-autonomous-mode");
    #elif defined(OPPO_USE_MTK)
    chip->chg_autonomous_mode = false;
    #endif

	#ifdef OPPO_USE_QCOMM
	chip->bms_controlled_charging = of_property_read_bool(node,
										"qcom,bms-controlled-charging");
	#elif defined(OPPO_USE_MTK)
    chip->bms_controlled_charging = false;
	#endif

	#ifdef OPPO_USE_QCOMM
	chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");
	#elif defined(OPPO_USE_MTK)
    chip->using_pmic_therm = false;
	#endif
	
    #ifdef OPPO_USE_QCOMM
    chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");
    #elif defined(OPPO_USE_MTK)
    chip->disable_apsd = true;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_string(node, "qcom,bms-psy-name", &chip->bms_psy_name);
    if (rc) {
        chip->bms_psy_name = NULL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->bms_psy_name = "bms";
    #endif
    
    #ifdef OPPO_USE_QCOMM
    chip->chg_valid_gpio = of_get_named_gpio_flags(node, "qcom,chg-valid-gpio", 0, &gpio_flags);
    if (!gpio_is_valid(chip->chg_valid_gpio)) {
        dev_dbg(chip->dev, "Invalid chg-valid-gpio");
    }
    else {
        chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;
    }
    #elif defined(OPPO_USE_MTK)
    //
    #endif
    
    #ifdef OPPO_USE_QCOMM
    chip->irq_gpio = of_get_named_gpio_flags(node, "qcom,irq-gpio", 0, &gpio_flags);
    if (!gpio_is_valid(chip->irq_gpio)) {
        dev_dbg(chip->dev, "Invalid irq-gpio");
    }
    #elif defined(OPPO_USE_MTK)
    //
    #endif

	#ifdef OPPO_USE_QCOMM
    #ifdef VENDOR_EDIT
    /*chaoying.chen@EXP.BaseDrv.charge,2015/07/04 modify  for 15085*/
    if(is_project(OPPO_15085)){
         chip->usbin_switch_gpio = of_get_named_gpio_flags(node, "qcom,usbin_switch-gpio", 0, &gpio_flags);
    } else {
         chip->usbin_switch_gpio = of_get_named_gpio_flags(node, "qcom,usbin-switch-gpio", 0, &gpio_flags);
    }
    #endif /*VENDOR_EDIT*/
	pr_err("%s usbin_switch_gpio:%d\n",__func__,chip->usbin_switch_gpio);
    if (!gpio_is_valid(chip->usbin_switch_gpio)) {
        dev_dbg(chip->dev, "Invalid usbin-switch-gpio");
    }
    #elif defined(OPPO_USE_MTK)
    //
    #endif

	#ifdef OPPO_USE_QCOMM
    chip->batt_id_gpio = of_get_named_gpio_flags(node, "qcom,batt-id-gpio", 0, &gpio_flags);
    if (!gpio_is_valid(chip->batt_id_gpio)) {
        dev_dbg(chip->dev, "Invalid batt-id-gpio");
    }
    #elif defined(OPPO_USE_MTK)
    //
    #endif
	
	#ifdef OPPO_USE_QCOMM
    chip->charger_inhibit_disabled = of_property_read_bool(node, "qcom,charger-inhibit-disabled");
	#elif defined(OPPO_USE_MTK)
    chip->charger_inhibit_disabled = true;
	#endif
	
//    #ifdef OPPO_USE_QCOMM
//    chip->usbphy_on_gpio = of_get_named_gpio_flags(node, "qcom,usbphy_on-gpio", 0, &gpio_flags);
//    if (!gpio_is_valid(chip->usbphy_on_gpio)) {
//        dev_dbg(chip->dev, "Invalid usbphy_on-gpio");
//    }
//    #elif defined(OPPO_USE_MTK)
//    //
//    #endif
    
	
#if 1   
    #ifdef OPPO_USE_QCOMM
	rc = of_property_read_u32(node, "qcom,faster-normal-limit-current-max-ma", &chip->faster_normal_limit_current_max_ma);
    if (rc) {
        chip->faster_normal_limit_current_max_ma = OPCHG_IMPUT_CURRENT_LIMIT_MAX_MA;
    }
    #elif defined(OPPO_USE_MTK)
    chip->faster_normal_limit_current_max_ma = OPCHG_IMPUT_CURRENT_LIMIT_MAX_MA;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    chip->iterm_disabled = of_property_read_bool(node, "qcom,iterm-disabled");
    #elif defined(OPPO_USE_MTK)
    chip->iterm_disabled = false;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,taper-float-voltage-mv", &chip->taper_vfloat_mv);
    if (rc < 0) {
        chip->taper_vfloat_mv = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->taper_vfloat_mv = 4320;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,opchg-fast-taper-fastchg-current-ma", &chip->fast_taper_fastchg_current_ma);
    if (rc < 0) {
        chip->fast_taper_fastchg_current_ma = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->fast_taper_fastchg_current_ma = 2100;
    #endif
	
	#ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,float-compensation-mv", &chip->float_compensation_mv);
    if (rc < 0) {
        chip->float_compensation_mv = -EINVAL;
    }
	#elif defined(OPPO_USE_MTK)
    chip->float_compensation_mv = 100;
	#endif
 #endif

 	//======================================================
 	// step1: get charger_ic_chip_id and  bms_ic_chip_id and  fast_charging_sign
 	//======================================================
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,fast-charger-project-sign", &chip->fast_charge_project);
    if (rc) {
	/*#hanqing.wang@EXP.BasicDrv.Audio add for clone 15089 and add the macor MSM_15062 and OPPO_15011 = OPPO_15018*/
		if(is_project(OPPO_14005)||is_project(OPPO_14023)||is_project(OPPO_15011)||is_project(OPPO_15018) || is_project(OPPO_15022))
		{
			chip->fast_charge_project = true;
		}
		else
		{
			chip->fast_charge_project = false;
		}
    }
    #elif defined(OPPO_USE_MTK)
    chip->fast_charge_project = 0;
    #endif

 

	//======================================================
 	// step2: Get charging control parameters
 	//======================================================
	#ifdef OPPO_USE_QCOMM
 	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
 	if (rc < 0) {
		 chip->iterm_ma = -EINVAL;
	}
	#elif defined(OPPO_USE_MTK)
 	chip->iterm_ma = 150;
	#endif
 	
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,recharge-mv", &chip->recharge_mv);
    if (rc < 0) {
        chip->recharge_mv = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->recharge_mv = 200;
    #endif

	
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma", &chip->fastchg_current_max_ma);
    if (rc) {
		chip->fastchg_current_max_ma = OPCHG_FAST_CHG_MAX_MA;
    }
    chip->fastchg_current_ma = chip->fastchg_current_max_ma;
    #elif defined(OPPO_USE_MTK)
    chip->fastchg_current_max_ma = OPCHG_FAST_CHG_MAX_MA;
    #endif
    
	#ifdef OPPO_USE_QCOMM
	rc = of_property_read_u32(node, "qcom,input-current-max-ma", &chip->limit_current_max_ma);
    if (rc) {
        chip->limit_current_max_ma = OPCHG_IMPUT_CURRENT_LIMIT_MAX_MA;
    }
    #elif defined(OPPO_USE_MTK)
    chip->limit_current_max_ma = OPCHG_IMPUT_CURRENT_LIMIT_MAX_MA;
    #endif
	
	#ifdef OPPO_USE_QCOMM
 	rc = of_property_read_u32(node, "qcom,float-voltage-mv", &chip->vfloat_mv);
 	if (rc < 0) {
		chip->vfloat_mv = -EINVAL;
 	}
	#elif defined(OPPO_USE_MTK)
 	chip->vfloat_mv = 4320;
	#endif

	//======================================================
 	// step3: Get charging temperature range of control parameters
 	//======================================================
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,hot_bat_decidegc", &chip->hot_bat_decidegc);
    if (rc < 0) {
        chip->hot_bat_decidegc = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->hot_bat_decidegc = 550;
    #endif
	
	#ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_hot_vfloat_mv", &chip->temp_hot_vfloat_mv);
    if (rc < 0) {
        chip->temp_hot_vfloat_mv = -EINVAL;
    }
	#elif defined(OPPO_USE_MTK)
    chip->temp_hot_vfloat_mv = 4100;
	#endif
    
	#ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_hot_fastchg_current_ma",
                            &chip->temp_hot_fastchg_current_ma);
    if (rc < 0) {
        chip->temp_hot_fastchg_current_ma = -EINVAL;
    }
	#elif defined(OPPO_USE_MTK)
    chip->temp_hot_fastchg_current_ma = 750;
	#endif
	
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,warm_bat_decidegc", &chip->warm_bat_decidegc);
    if (rc < 0) {
        chip->warm_bat_decidegc = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->warm_bat_decidegc = 450;
    #endif
	
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_warm_vfloat_mv", &chip->temp_warm_vfloat_mv);
    if (rc < 0) {
        chip->temp_warm_vfloat_mv = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->temp_warm_vfloat_mv = 4320;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_warm_fastchg_current_ma",
                            &chip->temp_warm_fastchg_current_ma);
    if (rc < 0) {
        chip->temp_warm_fastchg_current_ma = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->temp_warm_fastchg_current_ma = 1500;
    #endif
	
	#ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,pre_normal_bat_decidegc", &chip->pre_normal_bat_decidegc);
    if (rc < 0) {
        chip->pre_normal_bat_decidegc = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->pre_normal_bat_decidegc = 150;
    #endif
	
	#ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_pre_normal_vfloat_mv", &chip->temp_pre_normal_vfloat_mv);
    if (rc < 0) {
        chip->temp_pre_normal_vfloat_mv = -EINVAL;
    }
	#elif defined(OPPO_USE_MTK)
    chip->temp_pre_normal_vfloat_mv = 4320;
	#endif
    
	#ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_pre_normal_fastchg_current_ma",
                            &chip->temp_pre_normal_fastchg_current_ma);
    if (rc < 0) {
        chip->temp_pre_normal_fastchg_current_ma = -EINVAL;
    }
	#elif defined(OPPO_USE_MTK)
    chip->temp_pre_normal_fastchg_current_ma = 750;
	#endif
	
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,pre_cool_bat_decidegc", &chip->pre_cool_bat_decidegc);
    if (rc < 0) {
        chip->pre_cool_bat_decidegc = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->pre_cool_bat_decidegc = 100;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_pre_cool_vfloat_mv", &chip->temp_pre_cool_vfloat_mv);
    if (rc < 0) {
        chip->temp_pre_cool_vfloat_mv = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->temp_pre_cool_vfloat_mv = 4200;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_pre_cool_fastchg_current_ma",
                            &chip->temp_pre_cool_fastchg_current_ma);
    if (rc < 0) {
        chip->temp_pre_cool_fastchg_current_ma = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->temp_pre_cool_fastchg_current_ma = 750;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,cool_bat_decidegc", &chip->cool_bat_decidegc);
    if (rc < 0) {
        chip->cool_bat_decidegc = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->cool_bat_decidegc = 0;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_cool_vfloat_mv", &chip->temp_cool_vfloat_mv);
    if (rc < 0) {
        chip->temp_cool_vfloat_mv = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->temp_cool_vfloat_mv = 4000;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,temp_cool_fastchg_current_ma", &chip->temp_cool_fastchg_current_ma);
    if (rc < 0) {
        chip->temp_cool_fastchg_current_ma = -EINVAL;
    }
    #elif defined(OPPO_USE_MTK)
    chip->temp_cool_fastchg_current_ma = 250;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,cold_bat_decidegc", &batt_cold_degree_negative);
	if (rc < 0) {
        chip->cold_bat_decidegc = -EINVAL;
    }
	else {
        chip->cold_bat_decidegc = -batt_cold_degree_negative;
    }
    #elif defined(OPPO_USE_MTK)
    chip->cold_bat_decidegc = -100;
    #endif
    
    #ifdef OPPO_USE_QCOMM
    rc = of_property_read_u32(node, "qcom,bat_present_decidegc", &batt_present_degree_negative);
    if (rc < 0) {
        chip->bat_present_decidegc = -EINVAL;
    }
    else {
        chip->bat_present_decidegc = -batt_present_degree_negative;
    }
    #elif defined(OPPO_USE_MTK)
    chip->bat_present_decidegc = -200;
    #endif
    
    #ifdef OPPO_USE_QCOMM
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_mv",
						&chip->non_standard_vfloat_mv);
	if (rc < 0)
		chip->non_standard_vfloat_mv = -EINVAL;
	#elif defined(OPPO_USE_MTK)
	chip->non_standard_vfloat_mv = 4220;
	#endif

	#ifdef OPPO_USE_QCOMM
	rc = of_property_read_u32(node, "qcom,non_standard_fastchg_current_ma",
						&chip->non_standard_fastchg_current_ma);
	if (rc < 0)
		chip->non_standard_fastchg_current_ma = -EINVAL;
	#elif defined(OPPO_USE_MTK)
	chip->non_standard_fastchg_current_ma = 750;
	#endif

    pr_debug("chip->charging_disabled= %d,chip->irq_gpio= %d,chip->fast_charge_project= %d,\
		chip->iterm_ma= %d,chip->recharge_mv= %d,chip->limit_current_max_ma= %d,chip->vfloat_mv= %d,chip->fastchg_current_max_ma= %d,\n", 
		chip->charging_disabled,chip->irq_gpio,chip->fast_charge_project,
		chip->iterm_ma,chip->recharge_mv,chip->limit_current_max_ma,chip->vfloat_mv,chip->fastchg_current_max_ma);
    pr_debug("chip->hot_bat_decidegc= %d,chip->temp_hot_vfloat_mv= %d,chip->temp_hot_fastchg_current_ma= %d,\
		chip->warm_bat_decidegc= %d,chip->temp_warm_vfloat_mv= %d,chip->temp_warm_fastchg_current_ma= %d,\
		chip->pre_normal_bat_decidegc= %d,chip->temp_pre_normal_vfloat_mv= %d,chip->temp_pre_normal_fastchg_current_ma= %d,\
		chip->pre_cool_bat_decidegc= %d,chip->temp_pre_cool_vfloat_mv= %d,chip->temp_pre_cool_fastchg_current_ma= %d,\
		chip->cool_bat_decidegc= %d,chip->temp_cool_vfloat_mv= %d,chip->temp_cool_fastchg_current_ma= %d,\n", 
		chip->hot_bat_decidegc,chip->temp_hot_vfloat_mv,chip->temp_hot_fastchg_current_ma,
		chip->warm_bat_decidegc,chip->temp_warm_vfloat_mv,chip->temp_warm_fastchg_current_ma,
		chip->pre_normal_bat_decidegc,chip->temp_pre_normal_vfloat_mv,chip->temp_pre_normal_fastchg_current_ma,
		chip->pre_cool_bat_decidegc,chip->temp_pre_cool_vfloat_mv,chip->temp_pre_cool_fastchg_current_ma,
		chip->cool_bat_decidegc,chip->temp_cool_vfloat_mv,chip->temp_cool_fastchg_current_ma);
    pr_debug("chip->cold_bat_decidegc= %d,chip->bat_present_decidegc= %d,chip->non_standard_vfloat_mv= %d,chip->non_standard_fastchg_current_ma= %d\n", 
		chip->cold_bat_decidegc,chip->bat_present_decidegc,chip->non_standard_vfloat_mv,chip->non_standard_fastchg_current_ma);

    return 0;
}

void qpnp_battery_gauge_register(struct qpnp_battery_gauge *batt_gauge)
{
	if (qpnp_batt_gauge) {
		qpnp_batt_gauge = batt_gauge;
		pr_debug("qpnp-charger %s multiple battery gauge called\n",
								__func__);
	} else {
		qpnp_batt_gauge = batt_gauge;
	}
}
EXPORT_SYMBOL(qpnp_battery_gauge_register);

void qpnp_battery_gauge_unregister(struct qpnp_battery_gauge *batt_gauge)
{
	qpnp_batt_gauge = NULL;
}
EXPORT_SYMBOL(qpnp_battery_gauge_unregister);
static int opcharger_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc;
    struct opchg_charger *chip;
    struct power_supply *usb_psy;
    u8 reg = 0;
    int retval;
    
	pr_debug("opcharger start==================================\n");
    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy) {
        dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
        return -EPROBE_DEFER;
    }
    
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        dev_dbg(&client->dev, "Couldn't allocate memory\n");
        return -ENOMEM;
    }
    
    chip->client = client;
    chip->dev = &client->dev;
    chip->usb_psy = usb_psy;
    chip->fake_battery_soc = -EINVAL;

	chip->boot_mode = get_boot_mode();
	chip->driver_id = id->driver_data;
	if(is_project(OPPO_15035)){
		chip->driver_id = OPCHG_BQ24188_ID;
		dev_dbg(chip->dev, "opcharger_i2c_id=%d\n",chip->driver_id);
	}

	if(is_project(OPPO_15005) || is_project(OPPO_15025)|| is_project(OPPO_15035)){

	} else {
		opchg_chip = chip;
	}
    dev_dbg(chip->dev, "opcharger_i2c_id=%d,boot_mode =%d\n",chip->driver_id,chip->boot_mode);
    
    /* early for VADC get, defer probe if needed */
    chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
    if (IS_ERR(chip->vadc_dev)) {
        rc = PTR_ERR(chip->vadc_dev);
        if (rc != -EPROBE_DEFER) {
            pr_debug("vadc property missing\n");
        }
        return rc;
    }
    
    chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
    if(IS_ERR(chip->adc_tm_dev)) {
        rc = PTR_ERR(chip->adc_tm_dev);
        if (rc != -EPROBE_DEFER) {
            pr_debug("adc_tm property missing\n");
        }
        return rc;
	}
	
    /* i2c pull up Regulator configuration */
    chip->vcc_i2c = regulator_get(&client->dev, "vcc_i2c_opcharger");
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
    
    mutex_init(&chip->read_write_lock);
    /* probe the device to check if its actually connected */
	if(is_project(OPPO_15005)){
		if(get_PCB_Version() == 6 && chip->driver_id == OPCHG_SMB358_ID){
			pr_err("15005 no smb358 device\n");
			return -ENODEV;
		} else if(get_PCB_Version() != 6 && chip->driver_id == OPCHG_BQ24188_ID){
			pr_err("15005 no bq24188 device\n");
			return -ENODEV;
		}
	} else if(is_project(OPPO_15025)){
		if(get_PCB_Version() <= 5 && chip->driver_id == OPCHG_SMB358_ID){
			pr_err("15025 no smb358 device\n");
			return -ENODEV;
		} else if(get_PCB_Version() > 5 && chip->driver_id == OPCHG_BQ24157_ID){
			pr_err("15025 no bq24157 device\n");
			return -ENODEV;
		}
	}

	
	if(is_project(OPPO_15005) || is_project(OPPO_15025)|| is_project(OPPO_15035))
		opchg_chip = chip;
	
	rc = opchg_read_reg(chip, 0x00, &reg);
	if (rc) {
		pr_debug("Failed to detect OPCHARGER, device may be absent\n");
		return -ENODEV;
	}
	
    rc = opchg_parse_dt(chip);
    if (rc) {
        dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
        return rc;
    }
    
	opchg_init_charge_parameters(chip);


/*chaoying.chen@EXP.BaseDrv.charge,2015/07/02  modify  for 15085*/
	if(is_project(OPPO_14043)  || is_project(OPPO_14037) || is_project(OPPO_14051)|| 
		is_project(OPPO_15005) || is_project(OPPO_15057) || is_project(OPPO_15025)|| 
		is_project(OPPO_15009) || is_project(OPPO_15037) || is_project(OPPO_15035)||
        is_project(OPPO_15085) ) {
		if(gpio_is_valid(chip->usbin_switch_gpio)){
			rc = gpio_request(chip->usbin_switch_gpio,"opcharger_usbin_switch");
			if(rc)
				dev_err(&client->dev, "gpio_request for %d failed rc=%d\n", chip->usbin_switch_gpio, rc);
			//rc = gpio_direction_output(chip->usbin_switch_gpio,0);
			rc = gpio_direction_output(chip->usbin_switch_gpio,1);
			if(rc)
				dev_err(&client->dev, "set direction for usbin_switch fail\n");
			else
				dev_err(&client->dev, "usbin_switch:%d\n",gpio_get_value(chip->usbin_switch_gpio));
		}
	}	
	
	i2c_set_clientdata(client, chip);

	rc = opchg_hw_init(chip);
    if (rc) {
        dev_err(&client->dev, "Couldn't intialize hardware rc=%d\n", rc);
        goto fail_opchg_hw_init;
    }
	if(is_project(OPPO_15025) || is_project(OPPO_15005))
		opchg_dump_regs(chip);
	
	opchg_dc_property_config(chip);
	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register dc psy rc=%d\n", rc);
		return rc;
	}

	opchg_property_config(chip);
	
    wakeup_source_init(&chip->source, "opcharger_wake");
	
    rc = power_supply_register(chip->dev, &chip->batt_psy);
    if (rc < 0) {
        dev_err(&client->dev, "Couldn't register batt psy rc=%d\n", rc);
        return rc;
    }

	rc = opchg_regulator_init(chip);
    if (rc) {
        dev_err(&client->dev, "Couldn't initialize OPCHARGER ragulator rc=%d\n", rc);
        return rc;
    }
    
    opchg_works_init(chip);
	
    rc = opchg_get_initial_state(chip);
    if (rc) {
        dev_err(&client->dev, "Couldn't determine initial state rc=%d\n", rc);
        goto fail_opchg_hw_init;
    }	
    
    /* STAT irq configuration */
    if (gpio_is_valid(chip->irq_gpio) && (chip->driver_id == OPCHG_SMB358_ID)) {
        rc = gpio_request(chip->irq_gpio, "opcharger_stat");
        if (rc) {
            dev_err(&client->dev, "gpio_request for %d failed rc=%d\n", chip->irq_gpio, rc);
            goto fail_opchg_hw_init;
        }
        rc = gpio_direction_input(chip->irq_gpio);
        if (rc) {
            dev_err(&client->dev, "set_direction for stat gpio failed\n");
            goto fail_stat_irq;
        }
        client->irq = gpio_to_irq(chip->irq_gpio);
        if (client->irq < 0) {
            dev_err(&client->dev, "Invalid stat irq = %d\n", client->irq);
            goto fail_stat_irq;
        }
        rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
                            opchg_chg_irq_handler,
                            IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                            "opcharger_stat_irq", chip);
        if (rc) {
            dev_err(&client->dev, "Failed request irq=%d request rc = %d\n", client->irq, rc);
            goto fail_stat_irq;
        }
        enable_irq_wake(client->irq);
    }
    
//    /* usbphy_on-gpio configuration */
//    if (gpio_is_valid(chip->usbphy_on_gpio)) {
//        rc = gpio_request(chip->usbphy_on_gpio, "usbphy_on");
//        if (rc) {
//            dev_err(&client->dev, "gpio_request for %d failed rc=%d\n", chip->usbphy_on_gpio, rc);
//            goto fail_opchg_hw_init;
//        }
//        else {
//            rc = gpio_tlmm_config(GPIO_CFG(chip->usbphy_on_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//        }
//    }
    
    #ifdef OPPO_USE_ADC_TM_IRQ
    /* add hot/cold temperature monitor */
    chip->charging_opchg_temp_statu = OPCHG_CHG_TEMP_NORMAL;
    //chip->adc_param.low_temp = chip->pre_cool_bat_decidegc;
    chip->adc_param.low_temp = chip->pre_normal_bat_decidegc;
    chip->adc_param.high_temp = chip->warm_bat_decidegc;
    chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
    chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
    chip->adc_param.btm_ctx = chip;
    chip->adc_param.threshold_notification = opchg_get_adc_notification;
    chip->adc_param.channel = LR_MUX1_BATT_THERM;
    /* update battery missing info in tm_channel_measure*/
    rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param);
    if (rc) {
        pr_debug("requesting ADC error %d\n", rc);
    }
    #else
    /* add hot/cold temperature monitor */
    chip->charging_opchg_temp_statu = OPCHG_CHG_TEMP_NORMAL;
    //chip->adc_param.low_temp = chip->pre_cool_bat_decidegc;
    chip->adc_param.low_temp = chip->pre_normal_bat_decidegc;
    chip->adc_param.high_temp = chip->warm_bat_decidegc;
    chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
    #endif
    
    opchg_debugfs_create(chip);
	//opchg_dump_regs(chip);
	
    #ifdef VENDOR_EDIT
	/////////////////////////////////////////////////
	//solve the problems for into the RF mode Usb charging  
	if((get_boot_mode() == MSM_BOOT_MODE__RF) || (get_boot_mode() == MSM_BOOT_MODE__WLAN)) 
	{
		chip->multiple_test = 1; 
		//rc = opchg_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK, CMD_A_CHG_SUSP_EN_BIT); 
		opchg_config_suspend_enable(chip, FACTORY_ENABLE, 1);
		opchg_set_suspend_enable(chip, !!chip->suspend_status);  
		chip->g_is_changed = false;    
	}
	else 
	{
		chip->multiple_test = 0;
		opchg_config_suspend_enable(chip, FACTORY_ENABLE, 0); 
	}  
    #endif

	if(is_project(OPPO_14043) || is_project(OPPO_14037) || is_project(OPPO_15057) || is_project(OPPO_15025)){
		if(gpio_is_valid(chip->batt_id_gpio)){
			rc = gpio_request(chip->batt_id_gpio,"opcharger_batt_id");
			if(rc)
				dev_err(&client->dev, "gpio_request for %d failed rc=%d\n", chip->batt_id_gpio, rc);
			chip->batt_authen = oppo_battery_status_init(chip->batt_id_gpio);
			if(!chip->batt_authen){
				msleep(30);
				chip->batt_authen = oppo_battery_status_init(chip->batt_id_gpio);
			}
		}
	} else {
		chip->batt_authen = 1;
	}
	pr_debug("opcharger end========================= success,charger_present:%d\n",chip->chg_present);
    return 0;
	
err_set_vtg_i2c:
    if (regulator_count_voltages(chip->vcc_i2c) > 0) {
        regulator_set_voltage(chip->vcc_i2c, 0, OPCHARGER_I2C_VTG_MAX_UV);
    }
fail_stat_irq:
    if (gpio_is_valid(chip->irq_gpio)) {
        gpio_free(chip->irq_gpio);
    }
fail_opchg_hw_init:
    power_supply_unregister(&chip->batt_psy);
    regulator_unregister(chip->otg_vreg.rdev);
    return rc;
}

static int opcharger_charger_remove(struct i2c_client *client)
{
    struct opchg_charger *chip = i2c_get_clientdata(client);
    
    power_supply_unregister(&chip->batt_psy);
    if (gpio_is_valid(chip->chg_valid_gpio)) {
        gpio_free(chip->chg_valid_gpio);
    }
    
    regulator_disable(chip->vcc_i2c);
    mutex_destroy(&chip->read_write_lock);
    debugfs_remove_recursive(chip->debug_root);
    return 0;
}

static int opcharger_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct opchg_charger *chip = i2c_get_clientdata(client);
    int rc;
    
    cancel_delayed_work_sync(&chip->update_opchg_thread_work);
    
    chip->suspending = true;
	//Fuchun.Liao 2014-11-11 add for bq24196 wakeup AP frequently by wdt timeout
	opchg_set_wdt_timer(chip,false);
	chip->wdt_enable = false;
    disable_irq(client->irq);
	if(chip->vcc_i2c){
    	rc = regulator_disable(chip->vcc_i2c);
    	if (rc) {
        	dev_err(chip->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
        	return rc;
    	}
	}
    atomic_set(&chip->bms_suspended, 1);
    return 0;
}

static int opcharger_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct opchg_charger *chip = i2c_get_clientdata(client);
    int rc;
    //union power_supply_propval ret = {0, };

	if(chip->vcc_i2c){
    	rc = regulator_enable(chip->vcc_i2c);
    	if (rc) {
        	dev_err(chip->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        	return rc;
    	}
	}
    chip->suspending = false;
    enable_irq(client->irq);
	
    #if 0
    if (opchg_chip->otg_enable_pending) {	
        opchg_chip->otg_enable_pending = false;
        smb358_chg_otg_enable();
    }
	
    if (opchg_chip->otg_disable_pending) {		
        opchg_chip->otg_disable_pending = false;
        smb358_chg_otg_disable();
    }
    #endif
	
    #ifdef VENDOR_EDIT
    if (chip->bms_psy)
    {
        opchg_get_prop_batt_capacity(chip);
        power_supply_changed(&chip->batt_psy);	
    }
    #endif
	atomic_set(&chip->bms_suspended, 0);
    schedule_delayed_work(&chip->update_opchg_thread_work,
                            round_jiffies_relative(msecs_to_jiffies
                            (OPCHG_THREAD_INTERVAL)));
	
    return 0;
}

static void opcharger_charger_shutdown(struct i2c_client *client)
{
    struct opchg_charger *chip = i2c_get_clientdata(client);

	if(is_project(OPPO_15025)){
		if(chip->driver_id == OPCHG_BQ24157_ID){
			opchg_set_reset_charger(chip, true);
			msleep(20);
			pr_err("%s 15025 reset charger\n",__func__);
		}
	} else if(is_project(OPPO_15005)){
		if(chip->driver_id == OPCHG_BQ24188_ID){
			opchg_get_charging_status(chip);
			msleep(20);
			pr_err("%s 15005 read i2c\n",__func__);
		}
	}
}

static const struct dev_pm_ops opcharger_pm_ops = {
    .suspend	= opcharger_suspend,
    .resume		= opcharger_resume,
};

static struct of_device_id opcharger_match_table[] = {
    { .compatible = "qcom,smb358-charger"},
    { .compatible = "qcom,smb1357-charger"},
    { .compatible = "ti,bq24196-charger"},
    { .compatible = "ti,bq24157-charger"},
    { .compatible = "ti,bq24188-charger"},
    { },
};

static const struct i2c_device_id opcharger_charger_id[] = {
    {"smb358-charger", OPCHG_SMB358_ID},
    {"smb1357-charger", OPCHG_SMB1357_ID},
    {"bq24196-charger", OPCHG_BQ24196_ID},
    {"bq24157-charger", OPCHG_BQ24157_ID},
    {"bq24188-charger", OPCHG_BQ24188_ID},
    {},
};
MODULE_DEVICE_TABLE(i2c, opcharger_charger_id);

static struct i2c_driver opcharger_charger_driver = {
    .driver		= {
        .name		= "opcharger-charger",
        .owner		= THIS_MODULE,
        .of_match_table	= opcharger_match_table,
        .pm		= &opcharger_pm_ops,
    },
    .probe		= opcharger_charger_probe,
    .remove		= opcharger_charger_remove,
    .shutdown	= opcharger_charger_shutdown,
    .id_table	= opcharger_charger_id,
};

module_i2c_driver(opcharger_charger_driver);

MODULE_DESCRIPTION("OPCHARGER Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:opcharger-charger");

