COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= fifo_full_16_bit_each_frame_comp_dis.c

API_LOCATION ?= ../..
COMMON_LOCATION ?= ..

C_SRCS += \
$(API_LOCATION)/bma5.c \
$(API_LOCATION)/bma580.c \
$(API_LOCATION)/bma580_features.c \
$(COMMON_LOCATION)/common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
$(COMMON_LOCATION)/common
COINES_BACKEND ?= COINES_BRIDGE
include $(COINES_INSTALL_PATH)/coines.mk