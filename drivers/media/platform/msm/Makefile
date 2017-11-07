#
# Makefile for the QCOM spcific video device drivers
# based on V4L2.
#

obj-$(CONFIG_MSM_VIDC_V4L2)     += vidc/
obj-$(CONFIG_MSM_WFD) += wfd/
obj-y += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
ifeq ($(CONFIG_MSMB_CAMERA_DEFAULT),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
ifeq ($(CONFIG_OPPO_CAMERA_51),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_oppo_51/
endif
obj-$(CONFIG_MSM_VPU) += vpu/
