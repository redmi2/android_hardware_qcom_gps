ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# Define all platforms that have ATLAS. This will decide where we pick up
# QMI Framework from. Atlas platforms will pick up the QMI Framework from
# the Griffon binary, while other platforms will pick up the QMI Framework
# provided by CoreBSP.
ATLAS_PLATFORM_LIST := msm7627a


LOCAL_MODULE := libloc_api_v02

LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := \
    libutils \
    libcutils \
    libloc_adapter \
    libgps.utils

# If not compiling for Atlas, pick up CoreBsp QMI framework
ifneq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_SHARED_LIBRARIES += \
    libqmi_cci \
    libqmi_csi \
    libqmi_common_so
endif

# If compiling for Atlas, pick up the QMI Framework in Griffon
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

# If not compiling for Atlas, setup include path to get QMI defs from
# corebsp
ifneq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_C_INCLUDES += \
    $(TARGET_OUT_HEADERS)/qmi-framework/inc \
    $(TARGET_OUT_HEADERS)/qmi/inc
endif

# If compiling for Atlas, setup include path to get QMI defs from GPS branch
# of corebsp QmiFW.
ifeq ($(call is-board-platform-in-list,$(ATLAS_PLATFORM_LIST)),true)
LOCAL_C_INCLUDES += \
    $(TARGET_OUT_HEADERS)/qmi-framework-gps/inc
endif


LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif # not BUILD_TINY_ANDROID
