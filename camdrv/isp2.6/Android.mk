#
# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


ifeq ($(strip $(TARGET_BOARD_CAMERA_ISP_VERSION)), $(filter $(TARGET_BOARD_CAMERA_ISP_VERSION), 2.5 2.6 2.7))
LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

ALG_DIR := ispalg

LOCAL_CFLAGS += -fno-strict-aliasing -Wunused-variable -Wunused-function  -Werror
LOCAL_CFLAGS += -DLOCAL_INCLUDE_ONLY

# ************************************************
# external header file
# ************************************************
LOCAL_C_INCLUDES := \
	$(TARGET_BSP_UAPI_PATH)/kernel/usr/include/video \
	$(LOCAL_PATH)/../../common/inc \
	$(LOCAL_PATH)/../../oem2v6/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/ae/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/ae/sprd/ae2.x/ae/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/ae/sprd/ae3.x/ae/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/ae/sprd/ae2.x/flash/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/ae/sprd/ae3.x/flash/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/awb/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/awb/alc_awb/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/awb/sprd/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/af/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/af/sprd/afv1/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/af/sprd/aft/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/af/sft_af/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/af/alc_af/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/lsc/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/common/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/afl/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/smart \
	$(LOCAL_PATH)/../../$(ALG_DIR)/pdaf/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/pdaf/sprd/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/ai/inc \
	$(LOCAL_PATH)/../../$(ALG_DIR)/tof \
	$(LOCAL_PATH)/../../sensor/inc \
	$(LOCAL_PATH)/../../kernel_module/interface

# ************************************************
# internal header file
# ************************************************
LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/middleware/inc \
	$(LOCAL_PATH)/isp_tune \
	$(LOCAL_PATH)/driver/inc \
	$(LOCAL_PATH)/param_manager \
	$(LOCAL_PATH)/bridge

#LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_BSP_UAPI_PATH)/kernel/usr

LOCAL_SRC_FILES := $(call all-c-files-under, driver) \
	$(call all-c-files-under, isp_tune) \
	$(call all-c-files-under, middleware)

include $(LOCAL_PATH)/../../SprdCtrl.mk

LOCAL_MODULE := libcamdrv

LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := libcutils libutils libdl liblog

LOCAL_SHARED_LIBRARIES += libcamsensor libcambr libcamcommon libcampm libxml2

LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../performance
LOCAL_SHARED_LIBRARIES += libcamperf

ifeq (1, $(strip $(shell expr $(ANDROID_MAJOR_VER) \>= 8)))
LOCAL_PROPRIETARY_MODULE := true
endif

LOCAL_CFLAGS += -DTEST_ON_HAPS
include $(BUILD_SHARED_LIBRARY)

include $(call first-makefiles-under,$(LOCAL_PATH))
endif
