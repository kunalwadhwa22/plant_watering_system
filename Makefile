#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := plant-watering-system

EXTRA_COMPONENT_DIRS := $(CURDIR)/../bwesp32/components $(CURDIR)/../bwhal/src/bwcommon $(CURDIR)/../bwhal/src/bwdriver $(CURDIR)/../bwhal/src/bwlog $(CURDIR)/../bwhal/module/bwgpio/gpioexpanders $(CURDIR)/../bwhal/module/bwled/bwled

CFLAGS := -DDEBUG -DCONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID

include $(IDF_PATH)/make/project.mk

