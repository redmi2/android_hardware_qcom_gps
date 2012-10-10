ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ATLAS_PLATFORM_LIST := msm7627a


LOCAL_MODULE := libloc_api_v02

LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := \
    libutils \
    libcutils \
    libloc_adapter \
    libgps.utils

ifneq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_SHARED_LIBRARIES += \
    libqmi_cci \
    libqmi_csi \
    libqmi_common_so
endif


ifeq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_SHARED_LIBRARIES += \
    libqmi_client_griffon
endif

LOCAL_SRC_FILES += \
    LocApiV02Adapter.cpp \
    loc_api_v02_log.c \
    loc_api_v02_client.c \
    loc_api_sync_req.c \
    location_service_v02.c

LOCAL_CFLAGS += \
    -fno-short-enums \
    -D_ANDROID_

## Includes
LOCAL_C_INCLUDES := \
    $(TARGET_OUT_HEADERS)/libloc_eng \
    $(TARGET_OUT_HEADERS)/gps.utils

ifneq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_C_INCLUDES += \
    $(TARGET_OUT_HEADERS)/qmi-framework/inc
endif


ifeq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_C_INCLUDES += \
    $(TOP)/vendor/qcom/proprietary/gps/griffon/qmi_framework/inc
endif


LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif # not BUILD_TINY_ANDROID
