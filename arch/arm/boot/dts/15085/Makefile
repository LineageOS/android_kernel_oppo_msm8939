ifeq ($(CONFIG_OF),y)
#dtb-$(CONFIG_OPPO_COMMON_SOFT) += msm8939-mtp-15011.dtb
#dtb-$(CONFIG_OPPO_COMMON_SOFT) += msm8939-v3.0-mtp-15011.dtb
#ifdef VENDOR_EDIT
#huqiao@EXP.BasicDrv, 2015-06-17, add for 15085  hanqing.wang@EXP.Driver.Audio check msm8939_bc-mtp_15085
dtb-$(CONFIG_OPPO_COMMON_SOFT) += msm8939_bc-mtp_15085.dtb
#endif /*VENDOR_EDIT*/
endif	
DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))


$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
