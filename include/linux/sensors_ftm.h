/*************************************************************
 ** Copyright (C), 2012-2016, OPPO Mobile Comm Corp., Ltd
 ** CONFIG_MACH_OPPO
 ** File        : sensors_ftm.h
 ** Description :
 ** Date        : 2014-10-31
 ** Author      : BSP.Sensor
 **
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

struct dev_ftm {
	const char *name;
	const struct attribute **attrs;
	struct i2c_client *i2c_client;
	void *priv_data;
};

struct ftm_srv {
	struct dev_ftm * p_dev_ftm;
	struct kobject *pkobj;
	struct list_head node;
};

extern int register_single_dev_ftm(struct dev_ftm *client);
extern void unregister_single_dev_ftm(struct dev_ftm *client);
