# oem device deriver
#
#obj-$(CONFIG_OP_TOUCHSCREEN) += touchscreen/
ifeq ($(CONFIG_OP_HIMAX),y)
obj-$(CONFIG_OP_HIMAX) += touchscreen_himax/
else
obj-$(CONFIG_OP_TOUCHSCREEN) += touchscreen/
endif 
obj-$(CONFIG_INPUT_FINGERPRINT) += fingerprint/
