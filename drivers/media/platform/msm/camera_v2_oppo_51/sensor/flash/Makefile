ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51/sensor/cci
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oppo_51/sensor/io
obj-$(CONFIG_MSMB_CAMERA) += msm_led_flash.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_i2c_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += adp1660.o
obj-$(CONFIG_MSMB_CAMERA) += bd7710.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_torch.o
obj-$(CONFIG_MSMB_CAMERA) += msm_flash.o
obj-$(CONFIG_MSMB_CAMERA) += lm3642.o
ifeq ($(CONFIG_MACH_OPPO),y)
obj-$(CONFIG_MSMB_CAMERA) += lm3643.o
endif
