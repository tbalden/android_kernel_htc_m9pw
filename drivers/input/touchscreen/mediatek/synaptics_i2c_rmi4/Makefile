include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/synaptics_i2c_rmi4/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_i2c_rmi4/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)	+= synaptics_driver.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)	+= rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += rmi_fw_update.o
