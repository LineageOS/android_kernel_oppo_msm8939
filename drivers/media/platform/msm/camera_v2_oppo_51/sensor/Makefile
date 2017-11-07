ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/
ifeq ($(CONFIG_MACH_OPPO),y)
obj-$(CONFIG_MSMB_CAMERA) += vl6180/
endif
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_OV5645) += ov5645.o
obj-$(CONFIG_OV5645) += ov7695.o
