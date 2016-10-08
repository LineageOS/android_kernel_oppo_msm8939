/**
 * Copyright 2008-2013 OPPO Mobile Comm Corp., Ltd, All rights reserved.
 * CONFIG_MACH_OPPO:
 * FileName:devinfo.c
 * ModuleName:devinfo
 * Author: wangjc
 * Create Date: 2013-10-23
 * Description:add interface to get device information.
 * History:
   <version >  <time>  <author>  <desc>
   1.0		2013-10-23	wangjc	init
   2.0      2015-04-13  hantong modify as platform device  to support diffrent configure in dts
*/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <soc/qcom/smem.h>
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "../../../fs/proc/internal.h"

#define DEVINFO_NAME "devinfo"

static struct of_device_id devinfo_id[] = {
	{.compatible = "oppo-devinfo",},
	{},
};

struct devinfo_data {
	struct platform_device *devinfo;
	int hw_id1_gpio;
	int hw_id2_gpio;
	int hw_id3_gpio;
	int sub_hw_id1;
	int sub_hw_id2;
};

static struct proc_dir_entry *parent = NULL;

static void *device_seq_start(struct seq_file *s, loff_t *pos)
{
	static unsigned long counter = 0;

	if (*pos == 0) {
		return &counter;
	} else {
		*pos = 0;
		return NULL;
	}
}

static void *device_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void device_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static int device_seq_show(struct seq_file *s, void *v)
{
	struct proc_dir_entry *pde = s->private;
	struct manufacture_info *info = pde->data;

	if (info)
		seq_printf(s, "Device version:\t\t%s\n"
			      "Device manufacture:\t\t%s\n",
			      info->version,
			      info->manufacture);

	return 0;
}

static struct seq_operations device_seq_ops = {
	.start = device_seq_start,
	.next = device_seq_next,
	.stop = device_seq_stop,
	.show = device_seq_show
};

static int device_proc_open(struct inode *inode,struct file *file)
{
	int ret = seq_open(file, &device_seq_ops);

	if (!ret) {
		struct seq_file *sf = file->private_data;
		sf->private = PDE(inode);
	}

	return ret;
}

static const struct file_operations device_node_fops = {
	.read =  seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
	.open = device_proc_open,
	.owner = THIS_MODULE,
};

int register_device_proc(char *name, char *version, char *manufacture)
{
	struct proc_dir_entry *d_entry;
	struct manufacture_info *info;

	if (!parent) {
		parent = proc_mkdir ("devinfo", NULL);
		if (!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	info = kzalloc(sizeof *info, GFP_KERNEL);
	info->version = version;
	info->manufacture = manufacture;
	d_entry = proc_create_data(name, S_IRUGO, parent, &device_node_fops,
			info);

	if (!d_entry) {
		pr_err("create %s proc failed.\n", name);
		kfree(info);
		return -ENOENT;
	}

	return 0;
}

static void dram_type_add(void)
{
	struct manufacture_info dram_info;
	int *p = NULL;

	p = (int *)smem_alloc(SMEM_ID_VENDOR2, 4, 0, 0);

	if (p) {
		switch (*p) {
			case DRAM_TYPE0:
				dram_info.version = "EDB8132B3PB-1D-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE1:
				dram_info.version = "EDB8132B3PB-1D-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE2:
				dram_info.version = "EDF8132A3PF-GD-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE3:
				dram_info.version = "K4E8E304ED-AGCC FBGA";
				dram_info.manufacture = "SAMSUNG";
				break;
			default:
				dram_info.version = "unknown";
				dram_info.manufacture = "unknown";
		}
	} else {
		dram_info.version = "unknown";
		dram_info.manufacture = "unknown";
	}

	register_device_proc("ddr", dram_info.version, dram_info.manufacture);
}

static int get_hw_operator_version(struct devinfo_data *devinfo_data)
{
	int hw_operator_name = 0;
	int ret;
	int id1 = -1;
	int id2 = -1;
	int id3 = -1;
	struct device_node *np;

	if (!devinfo_data) {
		pr_err("devinfo_data is NULL\n");
		return 0;
	}

	np = devinfo_data->devinfo->dev.of_node;

	devinfo_data->hw_id1_gpio = of_get_named_gpio(
			np, "Hw,operator-gpio1", 0);
	if (devinfo_data->hw_id1_gpio < 0)
		pr_err("devinfo_data->hw_id1_gpio not specified\n");

	devinfo_data->hw_id2_gpio = of_get_named_gpio(
			np, "Hw,operator-gpio2", 0);
	if (devinfo_data->hw_id2_gpio < 0)
		pr_err("devinfo_data->hw_id2_gpio not specified\n");

	devinfo_data->hw_id3_gpio = of_get_named_gpio(
			np, "Hw,operator-gpio3", 0);
	if (devinfo_data->hw_id3_gpio < 0)
		pr_err("devinfo_data->hw_id3_gpio not specified\n");

	if (devinfo_data->hw_id1_gpio >= 0) {
		ret = gpio_request(devinfo_data->hw_id1_gpio, "HW_ID1");
		if (ret)
			pr_err("unable to request gpio [%d]\n",
					devinfo_data->hw_id1_gpio);
		else
			id1 = gpio_get_value(devinfo_data->hw_id1_gpio);
	}

	if (devinfo_data->hw_id2_gpio >= 0) {
		ret = gpio_request(devinfo_data->hw_id2_gpio, "HW_ID2");
		if (ret)
			pr_err("unable to request gpio [%d]\n",
					devinfo_data->hw_id2_gpio);
		else
			id2 = gpio_get_value(devinfo_data->hw_id2_gpio);
	}

	if (devinfo_data->hw_id3_gpio >= 0) {
		ret = gpio_request(devinfo_data->hw_id3_gpio, "HW_ID2");
		if (ret)
			pr_err("unable to request gpio [%d]\n",
					devinfo_data->hw_id3_gpio);
		else
			id3 = gpio_get_value(devinfo_data->hw_id3_gpio);
	}

	if (is_project(OPPO_15018)) {
		if ((id1 == 0) && (id2 == 0))
			hw_operator_name = OPERATOR_CHINA_TELECOM;
		else if ((id1 == 0) && (id2 == 1))
			hw_operator_name = OPERATOR_ALL_CHINA_CARRIER;
		else
			hw_operator_name = OPERATOR_UNKNOWN;
	}

	pr_info("hw_operator_name [%d]\n", hw_operator_name);

	return hw_operator_name;
}

static void sub_mainboard_verify(struct devinfo_data *devinfo_data)
{
	int ret;
	int id1 = -1;
	int id2 = -1;
	static char temp_manufacture_sub[12];
	struct device_node *np;
	struct manufacture_info mainboard_info;

	if (!devinfo_data) {
		pr_err("devinfo_data is NULL\n");
		return;
	}

	np = devinfo_data->devinfo->dev.of_node;

	devinfo_data->sub_hw_id1 = of_get_named_gpio(np, "Hw,sub_hwid_1", 0);
	if (devinfo_data->sub_hw_id1 < 0)
		pr_err("devinfo_data->sub_hw_id1 not specified\n");

	devinfo_data->sub_hw_id2 = of_get_named_gpio(np, "Hw,sub_hwid_2", 0);
	if (devinfo_data->sub_hw_id2 < 0)
		pr_err("devinfo_data->sub_hw_id2 not specified\n");

	if (devinfo_data->sub_hw_id1 >= 0) {
		ret = gpio_request(devinfo_data->sub_hw_id1, "SUB_HW_ID1");
		if(ret)
			pr_err("unable to request gpio [%d]\n",
					devinfo_data->sub_hw_id1);
		else
			id1 = gpio_get_value(devinfo_data->sub_hw_id1);
	}

	if (devinfo_data->sub_hw_id2 >= 0) {
		ret = gpio_request(devinfo_data->sub_hw_id2, "SUB_HW_ID2");
		if(ret)
			pr_err("unable to request gpio [%d]\n",
					devinfo_data->sub_hw_id2);
		else
			id2 = gpio_get_value(devinfo_data->sub_hw_id2);
	}

	mainboard_info.manufacture = temp_manufacture_sub;
	mainboard_info.version = "Qcom";

	switch (get_project()) {
		case OPPO_15011: {
			if (id1 == 0)
				sprintf(mainboard_info.manufacture,
					"15011-%d", get_Operator_Version());
			else
				mainboard_info.manufacture = "UNSPECIFIED";
			break;
		}
		case OPPO_15018: {
			if ((id1 == 0) && (id2 == 0))
				sprintf(mainboard_info.manufacture,
					"%d-%d", get_project(),
					get_Operator_Version());
			else if ((id1 == 1) && (id2 == 0))
				sprintf(mainboard_info.manufacture,
					"15089-%d", get_Operator_Version());
			else
				mainboard_info.manufacture = "UNSPECIFIED";
			break;
		}
		case OPPO_15022: {
			if ((id1 == 0) && (id2 == 1))
				sprintf(mainboard_info.manufacture,
					"%d-%d", get_project(),
					get_Operator_Version());
			else
				mainboard_info.manufacture = "UNSPECIFIED";
			break;
		}
		case OPPO_15109: {
			if ((id1 == 1) && (id2 == 1))
				sprintf(mainboard_info.manufacture,
					"15109-%d", OPERATOR_CHINA_MOBILE);
			else if ((id1 == 0) && (id2 == 1))
				sprintf(mainboard_info.manufacture,
					"15109-%d", OPERATOR_ALL_CHINA_CARRIER);
			else if ((id1 == 1) && (id2 == 0))
				sprintf(mainboard_info.manufacture,
					"15109-%d", get_Operator_Version());
			else if ((id1 == 0) && (id2 == 0) &&
				 (get_Operator_Version() != OPERATOR_FOREIGN_TAIWAN))
				sprintf(mainboard_info.manufacture,
					"15109-%d", get_Operator_Version());
			else
				mainboard_info.manufacture = "UNSPECIFIED";
			break;
		}
		default: {
			sprintf(mainboard_info.manufacture,
				"%d-%d", get_project(), get_Operator_Version());
			break;
		}
	}

	register_device_proc("sub_mainboard", mainboard_info.version,
			mainboard_info.manufacture);
}

static void mainboard_verify(struct devinfo_data *devinfo_data)
{
	struct manufacture_info mainboard_info;
	int hw_operator_version = 0;
	static char temp_manufacture[12];

	if (!devinfo_data) {
		pr_err("devinfo_data is NULL\n");
		return;
	}

	hw_operator_version = get_hw_operator_version(devinfo_data);
	mainboard_info.manufacture = temp_manufacture;

	switch (get_PCB_Version()) {
		case HW_VERSION__10:
			mainboard_info.version ="10";
			sprintf(mainboard_info.manufacture, "%d-SA",
				hw_operator_version);
			break;
		case HW_VERSION__11:
			mainboard_info.version = "11";
			sprintf(mainboard_info.manufacture, "%d-SB",
				hw_operator_version);
			break;
		case HW_VERSION__12:
			mainboard_info.version = "12";
			sprintf(mainboard_info.manufacture, "%d-SC",
				hw_operator_version);
			break;
		case HW_VERSION__13:
			mainboard_info.version = "13";
			sprintf(mainboard_info.manufacture, "%d-SD",
				hw_operator_version);
			break;
		case HW_VERSION__14:
			mainboard_info.version = "14";
			sprintf(mainboard_info.manufacture, "%d-SE",
				hw_operator_version);
			break;
		case HW_VERSION__15:
			mainboard_info.version = "15";
			sprintf(mainboard_info.manufacture, "%d-(T3-T4)",
				hw_operator_version);
			break;
		default:
			mainboard_info.version = "UNKNOWN";
			sprintf(mainboard_info.manufacture, "%d-UNKNOWN",
				hw_operator_version);
	}

	register_device_proc("mainboard", mainboard_info.version,
			mainboard_info.manufacture);
}

static void pa_verify(void)
{
	struct manufacture_info pa_info;

	switch (get_Modem_Version()) {
		case 0:
			pa_info.version = "0";
			pa_info.manufacture = "RFMD PA";
			break;
		case 1:
			pa_info.version = "1";
			pa_info.manufacture = "SKY PA";
			break;
		case 3:
			pa_info.version = "3";
			pa_info.manufacture = "AVAGO PA";
			break;
		default:
			pa_info.version = "UNKNOWN";
			pa_info.manufacture = "UNKNOWN";
	}

	register_device_proc("pa", pa_info.version, pa_info.manufacture);
}

static int devinfo_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct devinfo_data *devinfo_data = NULL;

	devinfo_data = kzalloc(sizeof(struct devinfo_data), GFP_KERNEL);
	if (devinfo_data == NULL) {
		pr_err("devinfo_data kzalloc failed\n");
		ret = -ENOMEM;
		return ret;
	}

	devinfo_data->devinfo = pdev;

	if (!parent) {
		parent = proc_mkdir("devinfo", NULL);
		if (!parent) {
			pr_err("can't create devinfo proc\n");
			ret = -ENOENT;
		}
	}

	pa_verify();
	dram_type_add();
	mainboard_verify(devinfo_data);
	sub_mainboard_verify(devinfo_data);

	return ret;
}

static int devinfo_remove(struct platform_device *dev)
{
	remove_proc_entry(DEVINFO_NAME, NULL);
	return 0;
}

static struct platform_driver devinfo_platform_driver = {
	.probe = devinfo_probe,
	.remove = devinfo_remove,
	.driver = {
		.name = DEVINFO_NAME,
		.of_match_table = devinfo_id,
	},
};

module_platform_driver(devinfo_platform_driver);

MODULE_DESCRIPTION("OPPO device info");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wangjc <wjc@oppo.com>");
