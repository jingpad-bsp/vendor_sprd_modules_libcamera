LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS += -fno-strict-aliasing -Wno-unused-parameter -Werror -Wno-error=format
LOCAL_LDFLAGS += -ldl

#sensor makefile config
SENSOR_FILE_COMPILER := $(CAMERA_SENSOR_TYPE_BACK)
SENSOR_FILE_COMPILER += $(CAMERA_SENSOR_TYPE_FRONT)
SENSOR_FILE_COMPILER += $(CAMERA_SENSOR_TYPE_BACK_EXT)
SENSOR_FILE_COMPILER += $(CAMERA_SENSOR_TYPE_FRONT_EXT)

SENSOR_FILE_COMPILER := $(shell echo $(SENSOR_FILE_COMPILER))
#$(warning $(SENSOR_FILE_COMPILER))

sensor_comma:=,
sensor_empty:=
sensor_space:=$(sensor_empty)

split_sensor:=$(sort $(subst $(sensor_comma),$(sensor_space) ,$(shell echo $(SENSOR_FILE_COMPILER))))
#$(warning $(split_sensor))

sensor_macro:=$(shell echo $(split_sensor) | tr a-z A-Z)
#$(warning $(sensor_macro))
$(foreach item,$(sensor_macro), $(eval LOCAL_CFLAGS += -D$(shell echo $(item))))

ifeq ($(strip $(OEM_DIR)),oem2v6)
LOCAL_C_INCLUDES += \
    $(TARGET_BSP_UAPI_PATH)/kernel/usr/include/video \
    $(LOCAL_PATH)/inc \
    $(LOCAL_PATH)/../common/inc \
    $(LOCAL_PATH)/../jpeg \
    $(LOCAL_PATH)/../vsp/inc \
    $(LOCAL_PATH)/../tool/mtrace \
    $(LOCAL_PATH)/../arithmetic/facebeauty/inc \
    $(LOCAL_PATH)/../sensor/inc \
    $(LOCAL_PATH)/../sensor/dummy \
    $(LOCAL_PATH)/../sensor/af_drv \
    $(LOCAL_PATH)/../sensor/otp_drv \
    $(LOCAL_PATH)/../arithmetic/inc \
    $(LOCAL_PATH)/../$(CPP_DIR)/driver/inc \
    $(LOCAL_PATH)/../$(CPP_DIR)/algo/inc \
    $(LOCAL_PATH)/../arithmetic/sprd_easy_hdr/inc \
    $(LOCAL_PATH)/../kernel_module/interface

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/../$(ISPALG_DIR)/common/inc \
    $(LOCAL_PATH)/../$(ISPDRV_DIR)/isp_tune \
    $(LOCAL_PATH)/../$(ISPDRV_DIR)/middleware/inc \
    $(LOCAL_PATH)/../$(ISPDRV_DIR)/driver/inc

LOCAL_HEADER_LIBRARIES += liblog_headers
LOCAL_HEADER_LIBRARIES += jni_headers

#LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_BSP_UAPI_PATH)/kernel/usr

LOCAL_SRC_FILES+= \
    src/SprdOEMCamera.c \
    src/cmr_common.c \
    src/cmr_oem.c \
    src/cmr_setting.c \
    src/cmr_isptool.c \
    src/cmr_sensor.c \
    src/cmr_mem.c \
    src/cmr_scale.c \
    src/cmr_rotate.c \
    src/cmr_grab.c \
    src/cmr_jpeg.c \
    src/cmr_exif.c \
    src/cmr_preview.c \
    src/cmr_snapshot.c \
    src/cmr_ipm.c \
    src/cmr_focus.c \
    src/cmr_filter.c \
    src/cmr_img_debug.c \
    src/exif_writer.c \
    src/jpeg_stream.c \
    src/cmr_4in1.c

ifeq ($(strip $(TARGET_BOARD_CAMERA_SUPPORT_ULTRA_WIDE)),true)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/sprd_warp/inc
LOCAL_SRC_FILES += ../oemcommon/src/cmr_ultrawide.c
endif
ifeq ($(strip $(CONFIG_CAMERA_MM_DVFS_SUPPORT)),true)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../oemcommon/mm_dvfs/
LOCAL_SRC_FILES += ../oemcommon/mm_dvfs/cmr_mm_dvfs.c
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_FACE_DETECT)),true)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/sprdface/inc
LOCAL_SRC_FILES += src/cmr_fd_sprd.c
endif

LOCAL_SRC_FILES += src/cmr_ai_scene.c

ifeq ($(strip $(TARGET_BOARD_CAMERA_AUTO_TRACKING)),true)
LOCAL_SRC_FILES += ../oemcommon/src/cmr_auto_tracking.c
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/4dtracking/inc
LOCAL_SHARED_LIBRARIES += libSprdOTAlgo
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_EIS)),true)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/eis/inc
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_Y_DENOISE)),true)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/inc/ydenoise_paten
LOCAL_SRC_FILES += src/cmr_ydenoise.c
endif

#include fdr files
ifeq ($(strip $(TARGET_BOARD_CAMERA_FDR_CAPTURE)),true)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/sprd_fdr/inc
LOCAL_SHARED_LIBRARIES += libsprdfdr
LOCAL_SHARED_LIBRARIES += libsprdfdradapter
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_HDR_CAPTURE)),true)
LOCAL_SRC_FILES += src/cmr_hdr.c
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_UV_DENOISE)),true)
LOCAL_SRC_FILES += src/cmr_uvdenoise.c
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_3DNR_CAPTURE)),true)
ifneq ($(filter $(strip $(TARGET_BOARD_PLATFORM)),ums312 ud710 ums512 ums7520 ums518 ums518-zebu sp9832e sp9863a),)
LOCAL_SRC_FILES += src/cmr_3dnr_sw.c
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/libmfnr/blacksesame/inc
LOCAL_SHARED_LIBRARIES += libmfnr libui libEGL libGLESv2
LOCAL_SHARED_LIBRARIES += libSprdMfnrAdapter
endif
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_CNR_CAPTURE)),true)
LOCAL_CFLAGS += -DCONFIG_CAMERA_CNR
LOCAL_SRC_FILES+= src/cmr_cnr.c
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/libcnr/inc
LOCAL_SHARED_LIBRARIES += libsprdcnr
LOCAL_SHARED_LIBRARIES += libsprdcnradapter
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_DRE_CAPTURE)),true)
LOCAL_CFLAGS += -DCONFIG_CAMERA_DRE
LOCAL_SRC_FILES+= src/cmr_dre.c
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/libdre/inc
LOCAL_SHARED_LIBRARIES += libsprddre
LOCAL_SHARED_LIBRARIES += libsprddreadapter
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_DRE_PRO_CAPTURE)),true)
LOCAL_CFLAGS += -DCONFIG_CAMERA_DRE_PRO
LOCAL_SHARED_LIBRARIES += libsprddrepro
LOCAL_SHARED_LIBRARIES += libsprddreproadapter
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_FILTER_VERSION)),0)
LOCAL_CFLAGS += -DCONFIG_CAMERA_FILTER
LOCAL_CFLAGS += -DCONFIG_FILTER_VERSION=0
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/sprd_filter/inc
LOCAL_SRC_FILES+= src/sprd_filter.c
LOCAL_SHARED_LIBRARIES += libSprdImageFilter
else
LOCAL_CFLAGS += -DCONFIG_FILTER_VERSION=0xFF
endif

LOCAL_CFLAGS += -DCONFIG_LIBYUV
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../arithmetic/sprd_yuvprocess/inc
LOCAL_SRC_FILES += ../arithmetic/sprd_yuvprocess/src/cmr_yuvprocess.c
LOCAL_SHARED_LIBRARIES += libyuv

LOCAL_C_INCLUDES += $(LOCAL_PATH)/../performance
LOCAL_SHARED_LIBRARIES += libcamperf

LOCAL_CFLAGS += -D_VSP_LINUX_ -D_VSP_

include $(LOCAL_PATH)/../SprdCtrl.mk

LOCAL_MODULE := libcamoem
LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES += libutils libcutils libcamsensor libcamcommon libhardware libxml2
LOCAL_SHARED_LIBRARIES += libcamdrv
LOCAL_SHARED_LIBRARIES += liblog

LOCAL_SHARED_LIBRARIES += libcppdrv

ifeq ($(strip $(TARGET_BOARD_CAMERA_FACE_BEAUTY)),true)
LOCAL_SHARED_LIBRARIES += libcamfb libcamfacebeauty
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_SUPPORT_ULTRA_WIDE)),true)
LOCAL_SHARED_LIBRARIES += libsprdwarp
LOCAL_SHARED_LIBRARIES += libsprdwarpadapter
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_FACE_DETECT)),true)
LOCAL_SHARED_LIBRARIES += libsprdfa libsprdfd libsprdfd_hw
ifeq ($(strip $(TARGET_BOARD_SPRD_FD_VERSION)),1)
LOCAL_CFLAGS += -DCONFIG_SPRD_FD_LIB_VERSION_2
LOCAL_SHARED_LIBRARIES += libsprdfarcnn
else
LOCAL_CFLAGS += -DCONFIG_SPRD_FD_LIB_VERSION_1
LOCAL_SHARED_LIBRARIES += libsprdfar
endif
endif

ifeq ($(strip $(TARGET_BOARD_FD_HW_SUPPORT)),true)
LOCAL_CFLAGS += -DCONFIG_SPRD_FD_HW_SUPPORT
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_EIS)),true)
LOCAL_SHARED_LIBRARIES += libgyrostab
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_HDR_CAPTURE)),true)
ifeq ($(strip $(TARGET_BOARD_SPRD_HDR_VERSION)),2)
LOCAL_CFLAGS += -DCONFIG_SPRD_HDR_LIB_VERSION_2
LOCAL_SHARED_LIBRARIES += libsprdhdradapter
else
LOCAL_CFLAGS += -DCONFIG_SPRD_HDR_LIB
LOCAL_SHARED_LIBRARIES += libsprd_easy_hdr
endif
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_UV_DENOISE)),true)
LOCAL_SHARED_LIBRARIES += libuvdenoise
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_Y_DENOISE)),true)
LOCAL_SHARED_LIBRARIES += libynoise
endif


ifeq ($(strip $(TARGET_BOARD_CONFIG_CAMERA_RT_REFOCUS)),true)
LOCAL_SHARED_LIBRARIES += libalRnBLV
endif

ifeq (1, $(strip $(shell expr $(ANDROID_MAJOR_VER) \>= 8)))
LOCAL_PROPRIETARY_MODULE := true
endif

include $(BUILD_SHARED_LIBRARY)

endif
