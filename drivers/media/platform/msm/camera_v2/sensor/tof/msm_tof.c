/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_tof.h"
#include "msm_cci.h"
#include "vl6180x_api.h"

#undef CDBG
#define CDBG(fmt, args...)

#define TOF_INIT_SIZE 41
#define TOF_RESULT_BYTE 52
#define TOF_SLAVE_ADDR 0x0029
#define TOF_RESULT_START_ADDR 0x0052
DEFINE_MSM_MUTEX(msm_tof_mutex);


static struct v4l2_file_operations msm_tof_v4l2_subdev_fops;

static struct msm_camera_i2c_fn_t msm_tof_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay = msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
};

static int32_t msm_tof_get_subdev_id(struct msm_tof_ctrl_t *tof_ctrl, void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (tof_ctrl->tof_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = tof_ctrl->pdev->id;
	else
		*subdev_id = tof_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	return 0;
}

static int msm_tof_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_tof_ctrl_t *tof_ctrl =  v4l2_get_subdevdata(sd);
	if (!tof_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (tof_ctrl->tof_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = tof_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&tof_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}

	return rc;
}

static const struct v4l2_subdev_internal_ops msm_tof_internal_ops = {
	.close = msm_tof_close,
};


static int msm_tof_init(struct msm_tof_ctrl_t *tof_ctrl)
{
	int rc = 0, status;
    struct msm_camera_cci_client *cci_client;
	CDBG("msm_tof_init Enter\n");

	if (!tof_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	if(tof_ctrl->tof_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		tof_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		cci_client = tof_ctrl->i2c_client.cci_client;
		cci_client->sid = TOF_SLAVE_ADDR;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = tof_ctrl->cci_master;
		rc = tof_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&tof_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	} else {
		tof_ctrl->i2c_client.client->addr = TOF_SLAVE_ADDR;
	}

	CDBG("msm_tof_init VL6180x_InitData\n");
	status = VL6180x_InitData(&tof_ctrl->i2c_client);
	if (status < 0)
		CDBG("IniData fail");
	if (status == CALIBRATION_WARNING)
		CDBG("calibration warning!!\n");

	status = VL6180x_FilterSetState(&tof_ctrl->i2c_client, 1); /* activate wrap around filter */
	if (status < 0)
		CDBG("filter set fail");

#if 0
	status = VL6180x_DisableGPIOxOut(sensor_i2c_client, 1); /* diable gpio 1 output, not needed when polling */
	if (status < 0)
		CDBG("disable gpio fail");
#endif

	rc = VL6180x_Prepare(&tof_ctrl->i2c_client);
	if (rc< 0)
		CDBG("prepare fail");

	rc =VL6180x_RangeSetSystemMode(&tof_ctrl->i2c_client, MODE_START_STOP|MODE_SINGLESHOT);
	if (rc < 0)
		CDBG("range set fail");

#if 0
	for (i = 0; i < TOF_INIT_SIZE; i++) {
		sensor_i2c_client->i2c_func_tbl->i2c_write(
		sensor_i2c_client,
		Tof_init_setting[i].addr,
		Tof_init_setting[i].data,
		MSM_CAMERA_I2C_BYTE_DATA);
	}
#endif


	CDBG("Exit\n");
	return rc;
}

static int32_t msm_tof_vreg_control(struct msm_tof_ctrl_t *tof_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_tof_vreg *vreg_cfg;

	vreg_cfg = &tof_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= MSM_TOF_MAX_VREGS) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(tof_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i], config);
	}
	return rc;
}


static int32_t msm_tof_power_down(struct msm_tof_ctrl_t *tof_ctrl)
{
	int32_t rc = 0;

	if (tof_ctrl->tof_state != TOF_POWER_DOWN) {

		rc = msm_tof_vreg_control(tof_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		tof_ctrl->tof_state = TOF_POWER_DOWN;
	}
	CDBG("Exit\n");
	return rc;
}


static int msm_tof_power_up (struct msm_tof_ctrl_t *tof_ctrl){
	int rc;

	if (!tof_ctrl) {
		pr_err("%s:%d failed: %p\n", __func__, __LINE__, tof_ctrl);
		return -EINVAL;
	}

	rc = msm_tof_vreg_control(tof_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	tof_ctrl->tof_state = TOF_POWER_UP;
	CDBG("Exit");
	return rc;
}

static int msm_tof_get_tof_data(struct msm_tof_ctrl_t *tof_ctrl,
			struct msm_tof_cfg_data *tof_data)
{
	int rc, status;
	VL6180x_RangeData_t RangeData;
	uint8_t *memptr;

	if (!tof_ctrl) {
		pr_err("%s:%d failed: %p\n",__func__, __LINE__, tof_ctrl);
		return -EINVAL;
	}

	status = VL6180x_RangePollMeasurement(&tof_ctrl->i2c_client, &RangeData);
	if (status < 0) {
	CDBG("VL6180x_RangePollMeasurement fail");
	} else {
	CDBG("SANK Range %d Signal_rate %d error_code %d "
		"Dmax %d, range_mm %d, raw_range %d, rtn_rate %d "
		"ref_rate %d, rtn_amb_rate %d, ref_amb_cnt %d "
		"rtn_conv_time %d, ref_conv_time %d",
		RangeData.range_mm,
		RangeData.signalRate_mcps,
		RangeData.errorStatus,
		RangeData.DMax,
		RangeData.FilteredData.range_mm,
		RangeData.FilteredData.rawRange_mm,
		RangeData.FilteredData.rtnRate,
		RangeData.FilteredData.refRate,
		RangeData.FilteredData.rtnAmbRate,
		RangeData.FilteredData.refAmbRate,
		RangeData.FilteredData.rtnConvTime,
		RangeData.FilteredData.refConvTime);
	}


	memptr = kzalloc(TOF_RESULT_BYTE, GFP_KERNEL);
	if(!memptr){
		pr_err("%s:%d, failed to alloc mem\n",__func__, __LINE__);
		return -ENOMEM;
	}

	memcpy(memptr, &RangeData, sizeof(VL6180x_RangeData_t));
	if (!memptr)
		return -EFAULT;

	tof_data->cfg.read_data.num_bytes = TOF_RESULT_BYTE;
	rc = copy_to_user(tof_data->cfg.read_data.dbuffer,
		memptr, tof_data->cfg.read_data.num_bytes);

	kfree(memptr);
	CDBG("Exit");
	return rc;
}



static int32_t msm_tof_config(struct msm_tof_ctrl_t *tof_ctrl,
	void __user *argp)
{
	struct msm_tof_cfg_data *cdata =
	(struct msm_tof_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(tof_ctrl->tof_mutex);

	switch (cdata->cfgtype) {
	case CFG_TOF_INIT:
		rc = msm_tof_init(tof_ctrl);
		break;

	case CFG_TOF_POWER_UP:
		rc = msm_tof_power_up(tof_ctrl);
		break;

	case CFG_TOF_POWER_DOWN:
		rc = msm_tof_power_down(tof_ctrl);
		break;

	case CFG_GET_TOF_DATA:
		rc = msm_tof_get_tof_data(tof_ctrl, cdata);
		break;

	default:
		break;
	}

	mutex_unlock(tof_ctrl->tof_mutex);
	return rc;
}

static long msm_tof_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_tof_ctrl_t *tof_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, tof_ctrl, argp);

	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_tof_get_subdev_id(tof_ctrl, argp);

	case VIDIOC_MSM_TOF_CFG:
		return msm_tof_config(tof_ctrl, argp);

	case MSM_SD_SHUTDOWN:
		msm_tof_close(sd, NULL);
		return 0;

	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long msm_tof_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	long rc = 0;
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct msm_tof_cfg_data32 *u32 = (struct msm_tof_cfg_data32 *)arg;
	struct msm_tof_cfg_data tof_data;
	void *parg = arg;

	switch (cmd) {
	case VIDIOC_MSM_TOF_CFG32:
		cmd = VIDIOC_MSM_TOF_CFG;
		switch (u32->cfg_type) {
		case CFG_GET_TOF_DATA:
			tof_data.cfgtype = u32->cfg_type;
			tof_data.cfg.read_data.num_bytes =
				u32->cfg.read_data.num_bytes;
			tof_data.cfg.read_data.dbuffer =
				compat_ptr(u32->cfg.read_data.dbuffer);
			parg = &tof_data;
			break;

		case CFG_TOF_POWER_UP:
			tof_data.cfgtype = u32->cfg_type;
			parg = &tof_data;
			break;

		case CFG_TOF_INIT:
			tof_data.cfgtype = u32->cfg_type;
			parg = &tof_data;
			break;

		case CFG_TOF_POWER_DOWN:
			tof_data.cfgtype = u32->cfg_type;
			parg = &tof_data;
			break;

		default:
			break;
		}
	}
	rc = msm_tof_subdev_ioctl(sd, cmd, parg);

	return rc;
}

static long msm_tof_subdev_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_tof_subdev_do_ioctl);
}
#endif

static int32_t msm_tof_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_tof_ctrl_t *tof_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	mutex_lock(tof_ctrl->tof_mutex);
	if (on)
		rc = msm_tof_power_up(tof_ctrl);
	else
		rc = msm_tof_power_down(tof_ctrl);
	mutex_unlock(tof_ctrl->tof_mutex);
	CDBG("Exit\n");
	return rc;
}


static struct v4l2_subdev_core_ops msm_tof_subdev_core_ops = {
	.ioctl = msm_tof_subdev_ioctl,
	.s_power = msm_tof_power,
};

static struct v4l2_subdev_ops msm_tof_subdev_ops = {
	.core = &msm_tof_subdev_core_ops,
};

static int32_t msm_tof_get_dt_data(struct device_node *of_node,
	struct msm_tof_ctrl_t *tof_ctrl)
{
	int rc = 0;
	struct msm_tof_vreg *vreg_cfg;
	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "cell-index",&tof_ctrl->pdev->id);
	if (rc < 0) {
		kfree(tof_ctrl);
		return rc;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&tof_ctrl->cci_master);
	if (rc < 0) {
		/* Set default master 0 */
		tof_ctrl->cci_master = MASTER_0;
		rc = 0;
	}

	if (of_find_property(of_node, "qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &tof_ctrl->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(tof_ctrl);
			return rc;
		}
	}

	return 0;

}

static int32_t msm_tof_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_tof_ctrl_t *tof_ctrl = NULL;

	CDBG("%s,%d Enter\n",__func__, __LINE__);
	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	tof_ctrl = kzalloc(sizeof(struct msm_tof_ctrl_t), GFP_KERNEL);
	if (!tof_ctrl) {
		pr_err("%s:%d failed to alloc mem\n", __func__, __LINE__);
		return -ENOMEM;
	}

	tof_ctrl->tof_v4l2_subdev_ops = &msm_tof_subdev_ops;
	tof_ctrl->tof_mutex = &msm_tof_mutex;
	tof_ctrl->pdev = pdev;
	tof_ctrl->tof_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	tof_ctrl->i2c_client.i2c_func_tbl = &msm_tof_cci_func_tbl;
	tof_ctrl->i2c_client.cci_client =  kzalloc(sizeof(struct msm_camera_cci_client), GFP_KERNEL);
	if (!tof_ctrl->i2c_client.cci_client) {
		kfree(tof_ctrl);
		pr_err("failed to alloc mem\n");
		return -ENOMEM;
	}

	rc = msm_tof_get_dt_data(pdev->dev.of_node, tof_ctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_tof_get_dt_data failed\n",
			__func__, __LINE__);
		kfree(tof_ctrl);
		return -EINVAL;
	}

	cci_client = tof_ctrl->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = MASTER_MAX;
	v4l2_subdev_init(&tof_ctrl->msm_sd.sd, tof_ctrl->tof_v4l2_subdev_ops);
	v4l2_set_subdevdata(&tof_ctrl->msm_sd.sd, tof_ctrl);
	tof_ctrl->msm_sd.sd.internal_ops = &msm_tof_internal_ops;
	tof_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(tof_ctrl->msm_sd.sd.name, ARRAY_SIZE(tof_ctrl->msm_sd.sd.name), "msm_tof");
	media_entity_init(&tof_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	tof_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	tof_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_TOF;
	tof_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&tof_ctrl->msm_sd);
	msm_tof_v4l2_subdev_fops = v4l2_subdev_fops;

#ifdef CONFIG_COMPAT
	msm_tof_v4l2_subdev_fops.compat_ioctl32 = msm_tof_subdev_fops_ioctl;
#endif
	tof_ctrl->msm_sd.sd.devnode->fops = &msm_tof_v4l2_subdev_fops;
	tof_ctrl->tof_state = TOF_POWER_DOWN;

	CDBG("%s,%d Exit\n",__func__, __LINE__);
	return 0;
}

static const struct of_device_id msm_tof_dt_match[] = {
	{.compatible = "qcom,tof", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_tof_dt_match);

static struct platform_driver msm_tof_platform_driver = {
	.probe = msm_tof_platform_probe,
	.driver = {
		.name = "qcom,tof",
		.owner = THIS_MODULE,
		.of_match_table = msm_tof_dt_match,
	},
};

static int __init msm_tof_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");

	rc = platform_driver_register(&msm_tof_platform_driver);
	if (rc != 0)
		pr_err("%s: %d, probe failed\n",__func__,__LINE__);

	return rc;

}

module_init(msm_tof_init_module);
MODULE_DESCRIPTION("MSM ACTUATOR");
MODULE_LICENSE("GPL v2");
