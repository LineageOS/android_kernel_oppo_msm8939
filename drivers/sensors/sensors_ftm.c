/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd
 ** CONFIG_MACH_OPPO
 ** File        : sensors_ftm.c
 ** Description :
 ** Date        : 2014-10-31
 ** Author      : BSP.Sensor
 **
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/sensors_ftm.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define OPPO_FTM_NAME "oppo_ftm"
#define ftm_debug(formart, args...) printk(KERN_EMERG OPPO_FTM_NAME" "formart, ##args)

static struct kset *oppo_ftm_kset;

DECLARE_RWSEM(oppo_ftm_list_lock);
LIST_HEAD(oppo_ftm_list);

static struct ftm_srv* find_srv_by_name(const char* name)
{
	struct list_head *temp_list;
	struct ftm_srv *temp_ftm;
	temp_list = oppo_ftm_list.next;

	while (temp_list != NULL) {
		temp_ftm = container_of(temp_list, struct ftm_srv, node);
		if (!strcmp(name, temp_ftm->p_dev_ftm->name))
			break;
		temp_list = temp_ftm->node.next;
		temp_ftm = NULL;
	}

	return temp_ftm;
}

static ssize_t set_dev_reg_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t data[2];
	struct ftm_srv *dev_srv;
	s32 ret;

	ftm_debug("name: %s\n", kobj->name);
	dev_srv = find_srv_by_name(kobj->name);
	if (dev_srv != NULL) {
		sscanf(buf, "%x %x", (unsigned int *)&data[0],
				(unsigned int *)&data[1]);
		ret = i2c_smbus_write_byte_data(dev_srv->p_dev_ftm->i2c_client,
				data[0], data[1]);
		if (!ret)
			ftm_debug("SET_REG: 0x%x 0x%x\n", data[0], data[1]);
		else
			ftm_debug("Fail SET_REG: 0x%x 0x%x\n", data[0],
					data[1]);
	}

	return count;
}

static struct kobj_attribute set_dev_reg = {
	.attr = {"set_reg", 0664},
	.store = set_dev_reg_store,
};

static ssize_t get_dev_reg_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t addr = 0;
	uint8_t data = 0;
	struct ftm_srv *dev_srv;

	ftm_debug("name: %s\n", kobj->name);
	dev_srv = find_srv_by_name(kobj->name);
	if (dev_srv != NULL) {
		ftm_debug("dev_srv->name = %s\n", dev_srv->p_dev_ftm->name);
		sscanf(buf, "%x",&addr);
		data = i2c_smbus_read_byte_data(dev_srv->p_dev_ftm->i2c_client,
				addr);
		ftm_debug("GET_REG: 0x%x 0x%x\n", addr, data);
	}

	return count;
}

static struct kobj_attribute get_dev_reg = {
	.attr = {"get_reg", 0664},
	.store = get_dev_reg_store,
};

static const struct attribute *ftm_def_attrs[] = {
	&set_dev_reg.attr,
	&get_dev_reg.attr,
	NULL
};

int register_single_dev_ftm(struct dev_ftm *client)
{
	int ret = 0;
	int i = 0;
	struct ftm_srv *dev_dir;

	if (client == NULL) {
		ftm_debug("client param is NULL\n");
		return -EINVAL;
	}

	dev_dir = kzalloc(sizeof(*dev_dir), GFP_KERNEL);
	if (dev_dir == NULL) {
		ftm_debug("kzalloc dev_dir fail\n");
		return -EINVAL;
	}

	dev_dir->pkobj = kobject_create_and_add(client->name,
			&oppo_ftm_kset->kobj);
	if (IS_ERR(dev_dir->pkobj)) {
		ftm_debug("Create kobject fail\n");
		kfree(dev_dir);
		return -EINVAL;
	}

	dev_dir->p_dev_ftm = client;

	down_write(&oppo_ftm_list_lock);
	list_add_tail(&dev_dir->node, &oppo_ftm_list);
	up_write(&oppo_ftm_list_lock);

	if (client->attrs != NULL) {
		for (i = 0; client->attrs[i] != NULL; i++) {
			ret = sysfs_create_file(dev_dir->pkobj,
					client->attrs[i]);
			if (ret != 0) {
				ftm_debug("sysfs create attr fail\n");
				return ret;
			}
		}
	}
	ret = sysfs_create_files(dev_dir->pkobj, ftm_def_attrs);
	if (ret != 0) {
		ftm_debug("sysfs create default files fail\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(register_single_dev_ftm);

void unregister_single_dev_ftm(struct dev_ftm *client)
{
	struct ftm_srv *dev_dir;

	dev_dir = find_srv_by_name(client->name);
	if (dev_dir != NULL) {
		kobject_del(dev_dir->pkobj);
		down_write(&oppo_ftm_list_lock);
		list_del(&dev_dir->node);
		up_write(&oppo_ftm_list_lock);
		kfree(dev_dir);
	}
}
EXPORT_SYMBOL(unregister_single_dev_ftm);

static int __init oppo_ftm_dir_init(void)
{
	int ret = 0;
	oppo_ftm_kset = kset_create_and_add(OPPO_FTM_NAME, NULL, NULL);

	if (IS_ERR(oppo_ftm_kset)) {
		ftm_debug("Create oppo ftm kset fail\n");
		ret = -EINVAL;
	}

	return ret;
}
subsys_initcall(oppo_ftm_dir_init);

MODULE_AUTHOR("ye.zhang <zhye@oppo.com>");
MODULE_DESCRIPTION("About OPPO ftm");
MODULE_LICENSE("GPL");
