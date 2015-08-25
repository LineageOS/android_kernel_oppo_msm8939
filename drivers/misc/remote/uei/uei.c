/*************************************************************
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : uei.c
 ** Description : uei 
 ** Date        : 2014-1-6 22:49
 ** Author      : Prd.SenDrv
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/ 
 
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <soc/oppo/device_info.h>

static struct manufacture_info uei_info = {
	.version = "1.0",
	.manufacture = "UEI",
};
#define IR_PROC_FILE_NAME "IR_remote"

#define UEI_DEV_NAME "uei"
static unsigned int UEI_POWER_GPIO = 0xFFFF;
#define UEI_LOG(format, args...) printk(KERN_EMERG UEI_DEV_NAME " "format,##args)

/*----------------------------------------------------------------*/
static struct of_device_id uei_device_id[] = {
	{.compatible = "uei",},
	{},
};

/*----------------------------------------------------------------*/
static int uei_probe(struct platform_device *pdev)
{
	int ret = 0;
    int power_gpio_status = 0;
	struct device_node *np = pdev->dev.of_node;
    
	UEI_LOG("%s Enter\n",__func__);
		
	UEI_POWER_GPIO = of_get_named_gpio(np, "uei,power-gpio", 0);
	UEI_LOG("GPIO %d use for uei power\n",UEI_POWER_GPIO);
	
	ret = gpio_request(UEI_POWER_GPIO, UEI_DEV_NAME);
	if (ret) 
    {
		UEI_LOG("unable to request gpio %d\n", UEI_POWER_GPIO);
		goto gpio_err;
	}
    
	ret = gpio_direction_output(UEI_POWER_GPIO, 1);
	if (ret)
		UEI_LOG("unable to set gpio %d\n direction", UEI_POWER_GPIO);

	power_gpio_status = __gpio_get_value(UEI_POWER_GPIO);
	UEI_LOG("gpio_status = %d\n", power_gpio_status);
	
	register_device_proc(IR_PROC_FILE_NAME, uei_info.version, uei_info.manufacture);

	UEI_LOG("%s Exit\n", __func__);
	return ret;
	
gpio_err:
	gpio_free(UEI_POWER_GPIO);

	return -1;
}

/*----------------------------------------------------------------*/
static int uei_remove(struct platform_device *dev)
{
	gpio_free(UEI_POWER_GPIO);

	return 0;
}

/*----------------------------------------------------------------*/
static int uei_pm_suspend(struct platform_device *dev, pm_message_t state)
{
	int ret = 0;

	UEI_LOG("%s\n", __func__);	
	
	ret = gpio_direction_output(UEI_POWER_GPIO, 0);
	
	if (ret)
		UEI_LOG("unable to set gpio %d\n direction", UEI_POWER_GPIO);
	
	return 0;
}

/*----------------------------------------------------------------*/
static int uei_pm_resume(struct platform_device *dev)
{
	int ret = 0;

	UEI_LOG("%s\n", __func__);	

	ret = gpio_direction_output(UEI_POWER_GPIO, 1);
	if (ret)
		UEI_LOG("unable to set gpio %d\n direction", UEI_POWER_GPIO);

	return 0;
}

/*----------------------------------------------------------------*/
static struct platform_driver uei_platform_driver = {
	.probe = uei_probe,
	.remove = uei_remove,
	.suspend = uei_pm_suspend,
	.resume = uei_pm_resume,
	.driver = {
		.name = UEI_DEV_NAME,
		.of_match_table = uei_device_id
	},
};

/*----------------------------------------------------------------*/
module_platform_driver(uei_platform_driver);
/*----------------------------------------------------------------*/
MODULE_AUTHOR("qiang.zhang <zhq@oppo.com>");
MODULE_DESCRIPTION("OPPO uei");
MODULE_LICENSE("GPL");

