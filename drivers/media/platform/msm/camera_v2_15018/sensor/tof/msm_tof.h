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
#ifndef MSM_ACTUATOR_H
#define MSM_ACTUATOR_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"

#define	MSM_TOF_MAX_VREGS (10)


struct msm_tof_vreg {
	struct camera_vreg_t *cam_vreg;
	void *data[MSM_TOF_MAX_VREGS];
	int num_vreg;
};

enum msm_tof_state_t {
	TOF_POWER_UP,
	TOF_POWER_DOWN,
};


struct msm_tof_ctrl_t {
	struct platform_device *pdev;
	struct platform_driver *pdriver;
	struct msm_camera_i2c_client i2c_client;
	enum msm_camera_device_type_t tof_device_type;
	struct msm_sd_subdev msm_sd;
	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *tof_v4l2_subdev_ops;
	enum cci_i2c_master_t cci_master;
	uint32_t subdev_id;
	struct msm_tof_vreg vreg_cfg;
	enum msm_tof_state_t tof_state;
	struct mutex *tof_mutex;
};

#endif
