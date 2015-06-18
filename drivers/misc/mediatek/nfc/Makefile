ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6795"))
	ifeq ($(CONFIG_MTK_TABLET_HARDWARE),y)
		obj-y += mt6795_tb/
	else
		obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
	endif
else
	obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
