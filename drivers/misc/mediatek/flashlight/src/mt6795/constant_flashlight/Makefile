#
# Makefile for misc devices that really don't fit anywhere else.
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

# FLASHLIGHT Drivers
obj-y								+= leds_strobe.o
obj-$(CONFIG_HTC_FLASHLIGHT_COMMON)	+= htc_flashlight_common.o
obj-$(CONFIG_FLASHLIGHT_SY7802)     += sy7802_flashlight.o
obj-$(CONFIG_FLASHLIGHT_TPS61310)	+= tps61310_flashlight.o
