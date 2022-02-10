/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "cmr_oem"
#define ATRACE_TAG (ATRACE_TAG_CAMERA | ATRACE_TAG_HAL)

#include <stdlib.h>
#include <sys/stat.h>
#include <math.h>
#include <cutils/properties.h>
#include <cutils/trace.h>
#include "cmr_oem.h"
#include "cmr_common.h"
#include <time.h>
#include <dlfcn.h>
#include "isp_otp_calibration.h"
#ifdef CONFIG_FACE_BEAUTY
#include "camera_face_beauty.h"
#endif
#include "sprd_img.h"
#include "isp_video.h"
#include "pthread.h"

/**********************************************************************************************/
#define PREVIEW_MSG_QUEUE_SIZE 100
#define SNAPSHOT_MSG_QUEUE_SIZE 50
#define CMR_EVT_INIT (CMR_EVT_OEM_BASE)
#define CMR_EVT_WAIT (CMR_EVT_OEM_BASE + 1)
#define CMR_EVT_EXIT (CMR_EVT_OEM_BASE + 2)
#define CMR_EVT_ISP_PM_MEM_INIT (CMR_EVT_OEM_BASE + 3)
#define CMR_EVT_ISP_PM_MEM_DEINIT (CMR_EVT_OEM_BASE + 4)
#define CMR_EVT_ISP_INIT (CMR_EVT_OEM_BASE + 5)

#define CAMERA_OEM_MSG_QUEUE_SIZE 10
#define CAMERA_RECOVER_CNT 3

#define OEM_HANDLE_HDR 1
#define OEM_HANDLE_3DNR 1
#define OEM_HANDLE_FILTER 1
#define FLASH_CAPTURE_SKIP_NUM_OFFSET 17

#define CAMERA_PATH_SHARE 1
#define OEM_RESTART_SUM 2
#define POWER2(x) (1 << (x))
#define ONE_HUNDRED 100
#define MS_TO_NANOSEC 1000
#define SEC_TO_NANOSEC 1000000000LL
#define BUF_BLOCK_SIZE (1024 * 1024)
enum oem_ev_level { OEM_EV_LEVEL_1, OEM_EV_LEVEL_2, OEM_EV_LEVEL_3 };

#define OFFLINE_CHANNEL_BIT 0x8
#define OFFLINE_CHANNEL 3

#define SBS_CMD_SBS_DUAL_MODE 4
#define SBS_CMD_MASTER_ONLY_MODE 5
#define SBS_CMD_SLAVE_ONELY_MODE 6

#define CHECK_HANDLE_VALID(handle)                                             \
    do {                                                                       \
        if (!handle) {                                                         \
            CMR_LOGE("err handle");                                            \
            return CMR_CAMERA_INVALID_PARAM;                                   \
        }                                                                      \
    } while (0)

#define CONVERED_CAMERA_INIT                                                   \
    ((cxt->camera_id >= 2 && (is_multi_camera_mode_oem == MODE_BLUR ||         \
                              is_multi_camera_mode_oem == MODE_SELF_SHOT ||    \
                              is_multi_camera_mode_oem == MODE_PAGE_TURN)))
/**********************************************************************************************/
typedef int (*INTERFACE_INIT)(char** error);
typedef int (*INTERFACE_CLOSE_ALL)(char **error);
typedef int (*INTERFACE_FOR_ALGO)(char *name, char **error, char **level);

struct image_sw_algorithm_buf src_sw_algorithm_buf;
struct image_sw_algorithm_buf dst_sw_algorithm_buf;
static cmr_uint sw_algorithm_buf_cnt = 0;

static uint32_t is_dual_capture = 0;
static pthread_mutex_t close_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t close_cond = PTHREAD_COND_INITIALIZER;
static uint32_t closing = 0;
static multiCameraMode is_multi_camera_mode_oem;
static uint8_t master_id_oem = 0;

/************************************internal interface
 * ***************************************/
static void camera_send_channel_data(cmr_handle oem_handle,
                                     cmr_handle receiver_handle, cmr_uint evt,
                                     void *data);
static cmr_int camera_sensor_streamctrl(cmr_u32 on_off, void *privdata);
static cmr_int camera_isp_ctrl_done(cmr_u32 cmd, void *data);
static void camera_sensor_evt_cb(cmr_int evt, void *data, void *privdata);
static cmr_int camera_is_need_change_fmt(cmr_handle oem_handle,
                                         struct frm_info *data_ptr);
static void camera_grab_evt_cb(cmr_int evt, void *data, void *privdata);
static void camera_grab_3dnr_evt_cb(cmr_int evt, void *data, void *privdata);
static void camera_scaler_evt_cb(cmr_int evt, void *data, void *privdata);
static void camera_jpeg_evt_cb(enum jpg_jpeg_evt evt, void *data,
                               void *privdata);
static cmr_int camera_isp_evt_cb(cmr_handle oem_handle, cmr_u32 evt, void *data,
                                 cmr_u32 data_len);
static void camera_focus_evt_cb(enum af_cb_type cb, cmr_uint param,
                                void *privdata);
static cmr_int camera_preview_cb(cmr_handle oem_handle,
                                 enum preview_cb_type cb_type,
                                 enum preview_func_type func, void *param);
static cmr_int camera_ipm_cb(cmr_u32 class_type,
                             struct ipm_frame_out *cb_param);
static void camera_snapshot_cb_to_hal(cmr_handle oem_handle,
                                      enum snapshot_cb_type cb,
                                      enum snapshot_func_type func,
                                      void *param);
static void camera_snapshot_state_handle(cmr_handle oem_handle,
                                         enum snapshot_cb_type cb,
                                         enum snapshot_func_type func,
                                         void *param);
static void camera_snapshot_cb(cmr_handle oem_handle, enum snapshot_cb_type cb,
                               enum snapshot_func_type func, void *param);
static cmr_int camera_before_set(cmr_handle oem_handle,
                                 enum preview_param_mode mode);
static cmr_int camera_after_set(cmr_handle oem_handle,
                                struct after_set_cb_param *param);
static cmr_int camera_focus_pre_proc(cmr_handle oem_handle);
static cmr_int camera_focus_post_proc(cmr_handle oem_handle,
                                      cmr_int will_capture);
static cmr_int camera_get_preview_status(cmr_handle oem_handle);
static cmr_int camera_sensor_init(cmr_handle oem_handle, cmr_uint is_autotest);
static cmr_int camera_sensor_deinit(cmr_handle oem_handle);
static cmr_int camera_grab_init(cmr_handle oem_handle);
static cmr_int camera_grab_deinit(cmr_handle oem_handle);
static cmr_int camera_jpeg_init(cmr_handle oem_handle);
static cmr_int camera_jpeg_deinit(cmr_handle oem_handle);
static cmr_int camera_scaler_init(cmr_handle oem_handle);
static cmr_int camera_scaler_deinit(cmr_handle oem_handle);
static cmr_int camera_rotation_init(cmr_handle oem_handle);
static cmr_int camera_rotation_deinit(cmr_handle oem_handle);
static cmr_int camera_isp_init_nosync(cmr_handle oem_handle);
static cmr_int camera_isp_init(cmr_handle oem_handle);
static cmr_int camera_isp_deinit_notice(cmr_handle oem_handle);
static cmr_int camera_isp_deinit(cmr_handle oem_handle);
static cmr_int camera_preview_init(cmr_handle oem_handle);
static cmr_int camera_preview_deinit(cmr_handle oem_handle);
static cmr_int camera_snapshot_init(cmr_handle oem_handle);
static cmr_int camera_snapshot_deinit(cmr_handle oem_handle);
static cmr_int camera_ipm_init(cmr_handle oem_handle);
static cmr_int camera_ipm_deinit(cmr_handle oem_handle);
static cmr_int camera_ipm_open_module(cmr_handle oem_handle);
static cmr_int camera_setting_init(cmr_handle oem_handle);
static cmr_int camera_setting_deinit(cmr_handle oem_handle);
static cmr_int camera_focus_init(cmr_handle oem_handle);
static cmr_int camera_focus_deinit(cmr_handle oem_handle);
static cmr_int camera_preview_cb_thread_proc(struct cmr_msg *message,
                                             void *data);
static cmr_int camera_snapshot_cb_thread_proc(struct cmr_msg *message,
                                              void *data);
static cmr_int camera_snapshot_secondary_thread_proc(struct cmr_msg *message,
                                                     void *data);
static cmr_int camera_snapshot_send_raw_thread_proc(struct cmr_msg *message,
                                                    void *data);
static cmr_int camera_create_prev_thread(cmr_handle oem_handle);
static cmr_int camera_destroy_prev_thread(cmr_handle oem_handle);
static cmr_int camera_create_snp_thread(cmr_handle oem_handle);
static cmr_int camera_destroy_snp_thread(cmr_handle oem_handle);
static cmr_int camera_init_thread(cmr_handle oem_handle);
static cmr_int camera_deinit_thread(cmr_handle oem_handle);
static cmr_int camera_res_init(cmr_handle);
static cmr_int camera_res_deinit(cmr_handle);
cmr_int camera_isp_pm_mem_init(cmr_handle oem_handle);
cmr_int camera_isp_pm_mem_deinit(cmr_handle oem_handle);
static cmr_int camera_res_init_internal(cmr_handle oem_handle);
static cmr_int camera_res_deinit_internal(cmr_handle oem_handle);
static cmr_int camera_init_thread_proc(struct cmr_msg *message, void *p_data);
static cmr_int camera_res_init_done(cmr_handle oem_handle);
static cmr_int camera_init_internal(cmr_handle oem_handle,
                                    cmr_uint is_autotest);
static cmr_int camera_deinit_internal(cmr_handle oem_handle);
static cmr_int camera_preview_pre_proc(cmr_handle oem_handle, cmr_u32 camera_id,
                                       cmr_u32 preview_sn_mode);
static cmr_int camera_preview_post_proc(cmr_handle oem_handle,
                                        cmr_u32 camera_id);
static cmr_int camera_start_encode(cmr_handle oem_handle,
                                   cmr_handle caller_handle,
                                   struct img_frm *src, struct img_frm *dst,
                                   struct cmr_op_mean *mean,
                                   struct jpeg_enc_cb_param *enc_cb_param);
static cmr_int camera_start_decode(cmr_handle oem_handle,
                                   cmr_handle caller_handle,
                                   struct img_frm *src, struct img_frm *dst,
                                   struct cmr_op_mean *mean);
static cmr_int camera_start_exif_encode(cmr_handle oem_handle,
                                        cmr_handle caller_handle,
                                        struct img_frm *pic_src,
                                        struct img_frm *thumb_src, void *exif,
                                        struct img_frm *dst,
                                        struct jpeg_wexif_cb_param *out_ptr);
static cmr_int
camera_start_exif_encode_simplify(cmr_handle oem_handle,
                                  struct img_frm *pic_src, struct img_frm *dst,
                                  struct jpeg_wexif_cb_param *out_ptr);
static cmr_int camera_stop_codec(cmr_handle oem_handle);
static cmr_int camera_start_scale(cmr_handle oem_handle,
                                  cmr_handle caller_handle, struct img_frm *src,
                                  struct img_frm *dst,
                                  struct cmr_op_mean *mean);
static cmr_int camera_start_rot(cmr_handle oem_handle, cmr_handle caller_handle,
                                struct img_frm *src, struct img_frm *dst,
                                struct cmr_op_mean *mean);
static cmr_int camera_ipm_pre_proc(cmr_handle oem_handle, void *private_data);
static cmr_int camera_capture_pre_proc(cmr_handle oem_handle, cmr_u32 camera_id,
                                       cmr_u32 preview_mode,
                                       cmr_u32 capture_mode, cmr_u32 is_restart,
                                       cmr_u32 is_sn_reopen);
static cmr_int camera_capture_post_proc(cmr_handle oem_handle,
                                        cmr_u32 camera_id);
static cmr_int camera_open_sensor(cmr_handle oem_handle, cmr_u32 camera_id);
static cmr_int camera_close_sensor(cmr_handle oem_handle, cmr_u32 camera_id);
static cmr_int camera_raw_proc(cmr_handle oem_handle, cmr_handle caller_handle,
                               struct raw_proc_param *param_ptr);
static cmr_int camera_isp_start_video(cmr_handle oem_handle,
                                      struct video_start_param *param_ptr);
static cmr_int camera_isp_stop_video(cmr_handle oem_handle);
static cmr_int camera_channel_cfg(cmr_handle oem_handle,
                                  cmr_handle caller_handle, cmr_u32 camera_id,
                                  struct channel_start_param *param_ptr,
                                  cmr_u32 *channel_id,
                                  struct img_data_end *endian);
static cmr_int camera_channel_start(cmr_handle oem_handle, cmr_u32 channel_bits,
                                    cmr_uint skip_number);
static cmr_int camera_channel_pause(cmr_handle oem_handle, cmr_uint channel_id,
                                    cmr_u32 reconfig_flag);
static cmr_int camera_channel_resume(cmr_handle oem_handle, cmr_uint channel_id,
                                     cmr_u32 skip_number, cmr_u32 deci_factor,
                                     cmr_u32 frm_num);
static cmr_int camera_channel_free_frame(cmr_handle oem_handle,
                                         cmr_u32 channel_id, cmr_u32 index);
static cmr_int camera_channel_stop(cmr_handle oem_handle, cmr_u32 channel_bits);
static cmr_int camera_channel_buff_cfg(cmr_handle oem_handle,
                                       struct buffer_cfg *buf_cfg);
static cmr_int
camera_channel_cap_cfg(cmr_handle oem_handle, cmr_handle caller_handle,
                       cmr_u32 camera_id, struct cap_cfg *cap_cfg,
                       cmr_u32 *channel_id, struct img_data_end *endian);
static cmr_int camera_isp_buff_cfg(cmr_handle oem_handle,
                                   struct buffer_cfg *buf_cfg);
static cmr_int camera_hdr_set_ev(cmr_handle oem_handle);
static cmr_int
camera_sw_3dnr_info_cfg(cmr_handle oem_handle,
                        struct sprd_img_3dnr_param *threednr_info);
static cmr_int camera_channel_scale_capability(cmr_handle oem_handle,
                                               cmr_u32 *width,
                                               cmr_u32 *sc_factor,
                                               cmr_u32 *sc_threshold);
static cmr_int
camera_channel_path_capability(cmr_handle oem_handle,
                               struct cmr_path_capability *capability);
static cmr_int camera_channel_get_cap_time(cmr_handle oem_handle, cmr_u32 *sec,
                                           cmr_u32 *usec);
static cmr_int camera_set_hal_cb(cmr_handle oem_handle,
                                 camera_cb_of_type hal_cb);
static cmr_int camera_ioctl_for_setting(cmr_handle oem_handle,
                                        cmr_uint cmd_type,
                                        struct setting_io_parameter *param_ptr);
static cmr_int camera_sensor_ioctl(cmr_handle oem_handle, cmr_uint cmd_type,
                                   struct common_sn_cmd_param *param_ptr);
static void camera_get_iso_value(cmr_handle oem_handle);
static cmr_int camera_get_ae_lum_value(cmr_handle oem_handle);

static cmr_int camera_get_setting_activity(cmr_handle oem_handle,
                                           cmr_uint *is_active);
static cmr_int camera_set_preview_param(cmr_handle oem_handle,
                                        enum takepicture_mode mode,
                                        cmr_uint is_snapshot);
static cmr_int camera_get_preview_param(cmr_handle oem_handle,
                                        enum takepicture_mode mode,
                                        cmr_uint is_snapshot,
                                        struct preview_param *out_param_ptr);
static cmr_int camera_get_snapshot_param(cmr_handle oem_handle,
                                         struct snapshot_param *out_ptr);
static cmr_int camera_get_sensor_info(cmr_handle oem_handle, cmr_uint sensor_id,
                                      struct sensor_exp_info *exp_info_ptr);
static cmr_int camera_get_sensor_fps_info(cmr_handle oem_handle,
                                          cmr_uint sensor_id, cmr_u32 sn_mode,
                                          struct sensor_mode_fps_tag *fps_info);
cmr_int camera_get_jpeg_param_info(cmr_handle oem_handle,
                                   struct jpeg_param *param);
static cmr_int camera_get_sensor_autotest_mode(cmr_handle oem_handle,
                                               cmr_uint sensor_id,
                                               cmr_uint *is_autotest);
static cmr_int camera_set_setting(cmr_handle oem_handle,
                                  enum camera_param_type id, cmr_uint param);
static void camera_set_3dnr_flag(struct camera_context *cxt,
                                 cmr_u32 threednr_flag);
static cmr_u32 camera_get_3dnr_flag(struct camera_context *cxt);
static cmr_u32 camera_get_cnr_flag(cmr_handle oem_handle);
static cmr_u32 camera_get_cnr_realtime_flag(cmr_handle oem_handle);
static cmr_u32 camera_get_sw_3dnr_flag(struct camera_context *cxt);
static cmr_int camera_open_3dnr(struct camera_context *cxt,
                                struct ipm_open_in *in_ptr,
                                struct ipm_open_out *out_ptr);
static cmr_int camera_close_3dnr(struct camera_context *cxt);
static cmr_int camera_open_cnr(struct camera_context *cxt,
                               struct ipm_open_in *in_ptr,
                               struct ipm_open_out *out_ptr);
static cmr_int camera_close_cnr(struct camera_context *cxt);

static void camera_set_hdr_flag(struct camera_context *cxt, cmr_u32 hdr_flag);
static cmr_u32 camera_get_hdr_flag(struct camera_context *cxt);
static cmr_int camera_open_hdr(struct camera_context *cxt,
                               struct ipm_open_in *in_ptr,
                               struct ipm_open_out *out_ptr);
static cmr_int camera_close_hdr(struct camera_context *cxt);
static void camera_set_filter_flag(struct camera_context *cxt,
                                   cmr_u32 filter_flag);
static cmr_uint camera_get_filter_flag(struct camera_context *cxt);
static cmr_int camera_open_filter(struct camera_context *cxt,
                                  struct ipm_open_in *in_ptr,
                                  struct ipm_open_out *out_ptr);
static cmr_int camera_close_filter(struct camera_context *cxt);
static void camera_snapshot_channel_handle(cmr_handle oem_handle, void *param);
static void camera_set_share_path_sm_flag(cmr_handle oem_handle, cmr_uint flag);
static cmr_uint camera_get_share_path_sm_flag(cmr_handle oem_handle);
static void camera_wait_share_path_available(cmr_handle oem_handle);
static void camera_set_discard_frame(cmr_handle oem_handle,
                                     cmr_uint is_discard);
static cmr_uint camera_get_is_discard_frame(cmr_handle oem_handle,
                                            struct frm_info *data);
static void camera_set_snp_req(cmr_handle oem_handle, cmr_uint is_req);
static cmr_uint camera_get_snp_req(cmr_handle oem_handle);
static cmr_int camera_get_cap_time(cmr_handle snp_handle);
static cmr_int camera_check_cap_time(cmr_handle snp_handle,
                                     struct frm_info *data);
static void camera_snapshot_started(cmr_handle oem_handle);
static cmr_uint camera_param_to_isp(cmr_uint cmd,
                                    struct common_isp_cmd_param *parm);
static cmr_int camera_isp_ev_switch(struct common_isp_cmd_param *parm);
static cmr_int camera_restart_rot(cmr_handle oem_handle);
static cmr_uint camera_set_vendor_hdr_ev(cmr_handle oem_handle);
static cmr_uint
camera_copy_sensor_fps_info_to_isp(struct isp_sensor_fps_info *out_isp_fps,
                                   SENSOR_MODE_FPS_T *in_fps);
static cmr_uint
camera_copy_sensor_ex_info_to_isp(struct isp_sensor_ex_info *out_isp_sn_ex_info,
                                  struct sensor_ex_info *in_sn_ex_info);
static cmr_uint camera_sensor_color_to_isp_color(cmr_u32 *isp_color,
                                                 cmr_u32 sensor_color);
static cmr_int camera_preview_get_isp_yimg(cmr_handle oem_handle,
                                           cmr_u32 camera_id,
                                           struct isp_yimg_info *yimg);
static cmr_int camera_preview_set_yimg_to_isp(cmr_handle oem_handle,
                                              cmr_u32 camera_id,
                                              struct yimg_info *yimg);
static cmr_int camera_preview_set_yuv_to_isp(cmr_handle oem_handle,
                                             cmr_u32 camera_id,
                                             struct yuv_info_t *yuv);
static cmr_int camera_set_hdr_ev(cmr_handle oem_handle, void *data);
static void camera_set_exif_exposure_time(cmr_handle oem_handle);
static cmr_int camera_ipm_process(cmr_handle oem_handle, void *data);
static cmr_int camera_nightpro_init(cmr_handle oem_handle);
static cmr_int camera_nightpro_deinit(cmr_handle oem_handle);
/**********************************************************************************************/

cmr_int camera_malloc(cmr_u32 mem_type, cmr_handle oem_handle,
                      cmr_u32 *size_ptr, cmr_u32 *sum_ptr, cmr_uint *phy_addr,
                      cmr_uint *vir_addr, cmr_s32 *fd) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !vir_addr || !size_ptr || !sum_ptr || !fd) {
        CMR_LOGE("error param mem_type 0x%x, oem_handle %p vir_addr %p, fd %p",
                 mem_type, oem_handle, vir_addr, fd);
        return -CMR_CAMERA_INVALID_PARAM;
    }
    CMR_LOGD("mem type %d size %d sum %d", mem_type, *size_ptr, *sum_ptr);
    if (cxt->hal_malloc) {
        ret = cxt->hal_malloc(mem_type, size_ptr, sum_ptr, phy_addr, vir_addr,
                              fd, cxt->client_data);
    }

    return ret;
}

cmr_int camera_free(cmr_u32 mem_type, cmr_handle oem_handle, cmr_uint *phy_addr,
                    cmr_uint *vir_addr, cmr_s32 *fd, cmr_u32 sum) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !vir_addr || !fd) {
        CMR_LOGE("error param mem_type=0x%x,oem_handle=%p,fd=%p,vir_addr=%p",
                 mem_type, oem_handle, fd, vir_addr);
        return -CMR_CAMERA_INVALID_PARAM;
    }
    CMR_LOGD("mem_type=%d, sum=%d", mem_type, sum);
    ret =
        cxt->hal_free(mem_type, phy_addr, vir_addr, fd, sum, cxt->client_data);

    return ret;
}

cmr_int camera_gpu_malloc(cmr_u32 mem_type, cmr_handle oem_handle,
                          cmr_u32 *size_ptr, cmr_u32 *sum_ptr,
                          cmr_uint *phy_addr, cmr_uint *vir_addr, cmr_s32 *fd,
                          void **handle, cmr_uint *width, cmr_uint *height) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !handle || !size_ptr || !sum_ptr) {
        CMR_LOGE("error param mem_type 0x%x, oem_handle %p handle %p", mem_type,
                 oem_handle, handle);
        return -CMR_CAMERA_INVALID_PARAM;
    }
    CMR_LOGD("mem type %d size %d sum %d", mem_type, *size_ptr, *sum_ptr);
    if (cxt->hal_gpu_malloc) {
        ret =
            cxt->hal_gpu_malloc(mem_type, size_ptr, sum_ptr, phy_addr, vir_addr,
                                fd, handle, width, height, cxt->client_data);
    }

    return ret;
}

cmr_int camera_write_sysfs_file(const char *filename, cmr_u32 value) {
    int32_t bytes = 0;
    char buffer[16];
    int ret = 0;
    int fd;
    CMR_LOGI("E");

    fd = open(filename, O_WRONLY);

    if (-1 == fd) {
        CMR_LOGE("Failed to open: sysfs_file %s", filename);
        return -EINVAL;
    }

    bytes = snprintf(buffer, sizeof(buffer), "0x%x", value);
    if (write(fd, buffer, bytes) != bytes) {
        CMR_LOGE("write failed\n");
        ret = -EINVAL;
    }

    close(fd);
    CMR_LOGI("X");

    return ret;
}

cmr_int camera_read_sysfs_file(const char *filename, cmr_u8 *value) {
    int32_t bytes = 0;
    int ret = 0;
    int fd;
    char buffer[4] = {0};

    CMR_LOGI("E");

    fd = open(filename, O_RDONLY);

    if (-1 == fd) {
        CMR_LOGE("Failed to open: sysfs_file %s", filename);
        return -EINVAL;
    }

    if (read(fd, buffer, sizeof(buffer)) <= 0) {
        CMR_LOGE("read failed\n");
        ret = -EINVAL;
    }
    CMR_LOGI("buffer %s", buffer);

    close(fd);
    *value = atoi(buffer);

    CMR_LOGI("X");

    return ret;
}

cmr_int camera_front_lcd_flash_activie(cmr_u32 face_type) {
    if (face_type == 1 && !strcmp(FRONT_CAMERA_FLASH_TYPE, "lcd"))
        return 1;

    return 0;
}

cmr_int camera_front_lcd_set_color_temperature(struct camera_context *cxt) {

    if (cxt->enhance == NULL) {
        CMR_LOGE("enhance object invalid");
        return -1;
    }

    if (cxt->enhance->set_value(cxt->color_temp))
        CMR_LOGE("set temperature %ld failed\n", cxt->color_temp);

    return 0;
}

cmr_int camera_front_lcd_enhance_module_init(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    const hw_module_t *module;

    if (!camera_front_lcd_flash_activie(cxt->facing)) {
        CMR_LOGI("flash is not lcd type");
        return -1;
    }

    if (hw_get_module(ENHANCE_HARDWARE_MODULE_ID, &module)) {
        CMR_LOGE("load enhance.so failed");
        return -1;
    }

    if (module->methods->open(module, "flash",
                              (struct hw_device_t **)&(cxt->enhance))) {
        CMR_LOGE("open enhance.so failed");
        return -1;
    }

    return 0;
}

cmr_int camera_front_lcd_enhance_module_deinit(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_u8 flip = 0;
    const char *refresh = "/sys/class/display/dispc0/refresh";
    const char *disable_flip = "/sys/class/display/dispc0/disable_flip";

    if (!camera_front_lcd_flash_activie(cxt->facing)) {
        CMR_LOGI("flash is not lcd type");
        return -1;
    }

    camera_read_sysfs_file(disable_flip, &flip);
    CMR_LOGI("disable_flip %d", flip);

    if (flip) {
        camera_write_sysfs_file(refresh, 0x01);
        cxt->color_temp = 0;
        camera_front_lcd_set_color_temperature(cxt);

        cxt->bg_color = 0;
        cxt->backlight_brightness = 0;
        cxt->lcd_flash_highlight = 0;
    }

    if (cxt->enhance) {
        cxt->enhance->common.close((struct hw_device_t *)cxt->enhance);
    }

    return 0;
}

cmr_int camera_front_lcd_flash_cfg(struct camera_context *cxt,
                                   struct sprd_flash_cfg_param *cfg) {

    cxt->lcd_flash_highlight = cfg->real_cell.type;
    cxt->backlight_brightness = cfg->real_cell.element[0].brightness;
    cxt->color_temp = cfg->real_cell.element[0].color_temp;
    cxt->bg_color = cfg->real_cell.element[0].bg_color;

    CMR_LOGI("lcd_flash_highlight:%d,brightness %d,color_temp %d,bg_color 0x%x",
             cxt->lcd_flash_highlight, cxt->backlight_brightness,
             cxt->color_temp, cxt->bg_color);

    return 0;
}

cmr_int camera_front_lcd_flash_callback(struct camera_context *cxt,
                                        cmr_u32 flash_mode) {
    const char *bg_color = "/sys/class/display/dispc0/bg_color";
    const char *brightness = "/sys/class/backlight/sprd_backlight/brightness";
    const char *refresh = "/sys/class/display/dispc0/refresh";
    const char *disable_flip = "/sys/class/display/dispc0/disable_flip";
    cmr_u8 flip = 0;

    switch (flash_mode) {
    case FLASH_OPEN:
        cxt->color_temp = cxt->color_temp ? cxt->color_temp : 0;
        camera_front_lcd_set_color_temperature(cxt);

        cxt->bg_color = cxt->bg_color ? cxt->bg_color : 0xffffff;
        camera_write_sysfs_file(bg_color, cxt->bg_color);

        camera_read_sysfs_file(brightness, &(cxt->backup_brightness));
        CMR_LOGI("backup_brightness:%d", cxt->backup_brightness);

        cxt->backlight_brightness =
            cxt->backlight_brightness ? cxt->backlight_brightness : 0xff;
        camera_write_sysfs_file(brightness, cxt->backlight_brightness);

        break;
    case FLASH_HIGH_LIGHT:
        CMR_LOGI("flash highlight");

        // cxt->color_temp = cxt->color_temp ? cxt->color_temp : 0;
        camera_front_lcd_set_color_temperature(cxt);

        cxt->bg_color = cxt->bg_color ? cxt->bg_color : 0xffffff;
        camera_write_sysfs_file(bg_color, cxt->bg_color);

        cxt->backlight_brightness =
            cxt->backlight_brightness ? cxt->backlight_brightness : 0xff;
        camera_write_sysfs_file(brightness, cxt->backlight_brightness);

        break;
    case FLASH_CLOSE_AFTER_OPEN:

        camera_read_sysfs_file(disable_flip, &flip);
        CMR_LOGI("disable_flip %d", flip);

        if (cxt->lcd_flash_highlight && flip) {
            camera_write_sysfs_file(brightness, cxt->backup_brightness);
            camera_write_sysfs_file(refresh, 0x01);
            cxt->color_temp = 0;
            camera_front_lcd_set_color_temperature(cxt);

            cxt->bg_color = 0;
            cxt->backlight_brightness = 0;
            cxt->lcd_flash_highlight = 0;
        }
        break;
    }

    return 0;
}

void camera_snapshot_started(cmr_handle oem_handle) {
    camera_snapshot_cb_to_hal(oem_handle, SNAPSHOT_EXIT_CB_PREPARE,
                              SNAPSHOT_FUNC_TAKE_PICTURE, 0);
}

void camera_set_discard_frame(cmr_handle oem_handle, cmr_uint is_discard) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    sem_wait(&cxt->access_sm);
    cxt->is_discard_frm = is_discard;
    sem_post(&cxt->access_sm);
    CMR_LOGI("%ld", cxt->is_discard_frm);
}

cmr_uint camera_get_is_discard_frame(cmr_handle oem_handle,
                                     struct frm_info *data) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;
    cmr_uint is_discard = 0;

    if (snp_cxt->channel_bits & (1 << data->channel_id)) {
        sem_wait(&cxt->access_sm);
        is_discard = cxt->is_discard_frm;
        sem_post(&cxt->access_sm);
    }
    CMR_LOGV("%ld", is_discard);
    return is_discard;
}

void camera_set_snp_req(cmr_handle oem_handle, cmr_uint is_req) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    sem_wait(&cxt->access_sm);
    cxt->snp_cxt.is_req_snp = is_req;
    sem_post(&cxt->access_sm);
    CMR_LOGI("%ld", cxt->snp_cxt.is_req_snp);
}

cmr_uint camera_get_snp_req(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_uint is_req;

    is_req = cxt->snp_cxt.is_req_snp;
    CMR_LOGV("%ld", is_req);

    return is_req;
}

cmr_uint camera_set_vendor_hdr_ev(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct common_sn_cmd_param sn_param;
    struct common_isp_cmd_param isp_param;
    cmr_uint ev_value;

    if (!(cxt->is_vendor_hdr))
        return ret;

    cxt->cap_cnt++;
    if (cxt->cap_cnt == 1) {
        /* set ev = 1 */
        ev_value = OEM_EV_LEVEL_2;
    } else if (cxt->cap_cnt == 2) {
        /*set ev = 2 */
        ev_value = OEM_EV_LEVEL_3;
    } else {
        /* set ev = 1 as default */
        cxt->cap_cnt = 0;
        ev_value = OEM_EV_LEVEL_2;
    }

    struct sensor_exp_info sensor_info;
    ret = camera_get_sensor_info(cxt, cxt->camera_id, &sensor_info);
    if (ret) {
        CMR_LOGE("get_sensor info failed!");
        ret = CMR_CAMERA_FAIL;
        goto exit;
    }

    if (SENSOR_IMAGE_FORMAT_RAW == sensor_info.image_format) {
        isp_param.cmd_value = ev_value;
        ret = camera_isp_ioctl(oem_handle, COM_ISP_SET_HDR, (void *)&isp_param);
    } else {
        sn_param.cmd_value = ev_value;
        ret = camera_sensor_ioctl(oem_handle, COM_SN_SET_HDR_EV,
                                  (void *)&sn_param);
    }

exit:
    return ret;
}

void camera_send_channel_data(cmr_handle oem_handle, cmr_handle receiver_handle,
                              cmr_uint evt, void *data) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct frm_info *frm_ptr = data;
    cmr_u32 chn_bit;
    struct buffer_cfg buf_cfg;
    cmr_int need_pause;

    (cmr_handle)(receiver_handle);
    if (!frm_ptr) {
        CMR_LOGE("err, frame is null");
        goto exit;
    }

    camera_local_normal_snapshot_need_pause(oem_handle, &need_pause);
    chn_bit = 1 << frm_ptr->channel_id;
    CMR_LOGV(
        "chn_id=%d, pre_chn_bits=%d snp_chn_bits=%d, total_num=%d,zsl_frame %d",
        frm_ptr->channel_id, cxt->prev_cxt.channel_bits,
        cxt->snp_cxt.channel_bits, cxt->snp_cxt.total_num,
        cxt->snp_cxt.zsl_frame);

    if (cxt->prev_cxt.channel_bits & chn_bit) {
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, evt, data);
    }
    if (cxt->prev_cxt.depthmap_channel_bits & chn_bit) {
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, evt, data);
    }
    if (cxt->prev_cxt.pdaf_channel_bits & chn_bit) {
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, evt, data);
    }
    if (cxt->prev_cxt.video_channel_bits & chn_bit) {
        cmr_copy(&cxt->prev_cxt.video_cur_chn_data, frm_ptr,
                 sizeof(struct frm_info));
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, evt, data);
    }
    if (cxt->snp_cxt.channel_bits & chn_bit) {
        if (TAKE_PICTURE_NEEDED == camera_get_snp_req((cmr_handle)cxt) &&
            CAMERA_ZSL_MODE != cxt->snp_cxt.snp_mode) {
            if (need_pause) {
                camera_set_discard_frame(cxt, 1);
            }
            ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                            SNAPSHOT_EVT_CHANNEL_DONE, data);
            if (need_pause) {
                if (CAMERA_ZSL_MODE == cxt->snp_cxt.snp_mode ||
                    1 != cxt->snp_cxt.total_num) {
                    ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                                   cxt->camera_id,
                                                   PREVIEW_CHN_PAUSE, data);
                    if (ret) {
                        CMR_LOGE("failed to pause path %ld", ret);
                    }
                    ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                                   cxt->camera_id, evt, data);
                } else {
                    ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                                   cxt->camera_id, evt, data);
                    camera_post_share_path_available(oem_handle);
                }
            } else {
                ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                               cxt->camera_id, evt, data);
                camera_post_share_path_available(oem_handle);
            }
        } else {
            if (cxt->snp_cxt.zsl_frame) {
                cmr_copy(&cxt->snp_cxt.cur_chn_data, frm_ptr,
                         sizeof(struct frm_info));
                ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                               cxt->camera_id, evt, data);
                CMR_LOGV("camera id = %d,  cur_chn_data.yaddr_vir 0x%x, "
                         "yaddr_vir 0x%x",
                         cxt->camera_id, cxt->snp_cxt.cur_chn_data.yaddr_vir,
                         frm_ptr->yaddr_vir);
            } else {
                ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                                SNAPSHOT_EVT_FREE_FRM, data);
            }
        }
    }
exit:
    if (ret) {
        CMR_LOGE("failed to send channel data %ld", ret);
    }
    ATRACE_END();
}

/*
*privdata:oem handle
*/
cmr_int camera_sensor_streamctrl(cmr_u32 on_off, void *privdata) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;

    if (!cxt) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (cxt->is_multi_mode == MODE_SBS) {
        ret = cmr_sensor_set_bypass_mode(cxt->sn_cxt.sensor_handle,
                                         cxt->camera_id, SBS_CMD_SBS_DUAL_MODE);
        if (ret) {
            CMR_LOGE("set_bypass_mode %d sensor failed %ld", cxt->camera_id,
                     ret);
        }
    }

#ifdef CONFIG_CAMERA_RT_REFOCUS
/*temp define, will del it when al3200 be merged into code*/
#define AL3200_CMD_DEPTH_MAP_MODE 2
#define AL3200_CMD_BYPASS_MODE 3
    /*temp define, end*/

    if (cxt->camera_id == SENSOR_MAIN) {
        CMR_LOGI("open 2 camera");
        if (cxt->is_refocus_mode == 1) {
            ret = cmr_sensor_set_bypass_mode(cxt->sn_cxt.sensor_handle,
                                             SENSOR_MAIN2,
                                             AL3200_CMD_DEPTH_MAP_MODE);
            if (ret) {
                CMR_LOGE("set_bypass_mode %d sensor failed %ld", cxt->camera_id,
                         ret);
            }
        } else {
            ret = cmr_sensor_set_bypass_mode(cxt->sn_cxt.sensor_handle,
                                             SENSOR_MAIN2,
                                             AL3200_CMD_BYPASS_MODE);
            if (ret) {
                CMR_LOGE("set_bypass_mode %d sensor failed %ld", cxt->camera_id,
                         ret);
            }
        }
        ret = cmr_sensor_stream_ctrl(cxt->sn_cxt.sensor_handle, SENSOR_MAIN2,
                                     on_off);
        if (ret) {
            CMR_LOGE("err to set stream %ld", ret);
        }
    }
#endif
    ret = cmr_sensor_stream_ctrl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                 on_off);
    if (ret) {
        CMR_LOGE("err to set stream %ld", ret);
    }
exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_isp_ctrl_done(cmr_u32 cmd, void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    (void *)data;
    if (cmd >= ISP_CTRL_MAX) {
        CMR_LOGE("isp wrong cmd %d", cmd);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGV("isp cmd, 0x%x, ret %ld", cmd, ret);
exit:
    return ret;
}

void camera_sensor_evt_cb(cmr_int evt, void *data, void *privdata) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;

    if (!cxt || !data || CMR_EVT_SENSOR_BASE != (CMR_EVT_SENSOR_BASE & evt)) {
        CMR_LOGE("error param, handle 0x%lx data 0x%lx evt 0x%lx",
                 (cmr_uint)cxt, (cmr_uint)data, evt);
        goto exit;
    }
    CMR_LOGI("evt 0x%lx, handle 0x%lx", evt, (cmr_uint)privdata);
    switch (evt) {
    case CMR_SENSOR_FOCUS_MOVE:
        if (1 == cxt->focus_cxt.inited) {
            ret = cmr_focus_sensor_handle(cxt->focus_cxt.focus_handle,
                                          CMR_SENSOR_FOCUS_MOVE, cxt->camera_id,
                                          data);
        } else {
            CMR_LOGE("err, focus hasn't been initialized");
        }
        break;
    case CMR_SENSOR_ERROR:
        if (1 == cxt->prev_cxt.inited) {
            ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                           cxt->camera_id, evt, data);
        } else {
            CMR_LOGE("err, preview hasn't been initialized");
        }
        break;
    default:
        CMR_LOGE("can't handle ths evt, 0x%lx", evt);
    }
exit:
    if (ret) {
        CMR_LOGE("failed %ld", ret);
    }
}

cmr_int camera_is_need_change_fmt(cmr_handle oem_handle,
                                  struct frm_info *data_ptr) {
    cmr_int is_change_fmt = 0;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;
    cmr_uint is_snp_frm = 0;

    is_snp_frm = ((1 << data_ptr->channel_id) & snp_cxt->channel_bits);
    if (is_snp_frm &&
        ((1 == camera_get_hdr_flag(cxt)) || (1 == camera_get_3dnr_flag(cxt)))) {
        if (IMG_DATA_TYPE_JPEG == data_ptr->fmt ||
            IMG_DATA_TYPE_RAW == data_ptr->fmt) {
            is_change_fmt = 1;
        }
    }
    return is_change_fmt;
}

cmr_int camera_get_cap_time(cmr_handle snp_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)snp_handle;
    cmr_u32 sec = 0;
    cmr_u32 usec = 0;

    sem_wait(&cxt->access_sm);
    ret = cmr_grab_get_cap_time(cxt->grab_cxt.grab_handle, &sec, &usec);
    CMR_LOGI("cap time %d %d", sec, usec);
    cxt->snp_cxt.cap_time_stamp = sec * SEC_TO_NANOSEC + usec * MS_TO_NANOSEC;
    sem_post(&cxt->access_sm);
    return ret;
}

cmr_int camera_check_cap_time(cmr_handle snp_handle, struct frm_info *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)snp_handle;
    cmr_s64 frame_time =
        data->sec * SEC_TO_NANOSEC + data->usec * MS_TO_NANOSEC;

    CMR_LOGV("time %ld, %ld", data->sec, data->usec);
    sem_wait(&cxt->access_sm);
    if (TAKE_PICTURE_NEEDED == cxt->snp_cxt.is_req_snp &&
        (cxt->snp_cxt.channel_bits & (1 << data->channel_id))) {
        if (frame_time <= cxt->snp_cxt.cap_time_stamp) {
            CMR_LOGW("frame is earlier than picture, drop!");
            ret = CMR_CAMERA_FAIL;
        } else {
            CMR_LOGV("frame time OK!");
        }
    }
    sem_post(&cxt->access_sm);
    return ret;
}

cmr_int camera_get_post_proc_chn_out_frm_id(struct img_frm *frame,
                                            struct frm_info *data) {
    cmr_int i;

    for (i = 0; i < CMR_CAPTURE_MEM_SUM; i++) {
        if ((cmr_int)frame[i].fd == data->fd)
            break;
    }
    CMR_LOGI("frm id %ld", i);
    return i;
}

void camera_grab_handle(cmr_int evt, void *data, void *privdata) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct ipm_context *ipm_cxt = &cxt->ipm_cxt;
    struct frm_info *frame = (struct frm_info *)data;
    cmr_u32 channel_id;
    cmr_handle receiver_handle;
    cmr_u32 chn_bits = (1 << frame->channel_id);
    cmr_u32 frm_id;
    struct buffer_cfg buf_cfg;
    struct setting_cmd_parameter setting_param;

    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_APPMODE,
                    &setting_param);

    if (cxt->snp_cxt.snp_mode != CAMERA_ZSL_MODE) {
        if (camera_get_is_discard_frame((cmr_handle)cxt, frame) ||
            camera_check_cap_time((cmr_handle)cxt, frame)) {
            memset(&buf_cfg, 0, sizeof(buf_cfg));
            buf_cfg.channel_id = frame->channel_id;
            buf_cfg.base_id = CMR_BASE_ID(frame->frame_id);
            buf_cfg.count = 1;
            buf_cfg.flag = BUF_FLAG_RUNNING;
            buf_cfg.addr[0].addr_y = frame->yaddr;
            buf_cfg.addr[0].addr_u = frame->uaddr;
            buf_cfg.addr[0].addr_v = frame->vaddr;
            buf_cfg.addr_vir[0].addr_y = frame->yaddr_vir;
            buf_cfg.addr_vir[0].addr_u = frame->uaddr_vir;
            buf_cfg.addr_vir[0].addr_v = frame->vaddr_vir;
            buf_cfg.fd[0] = frame->fd;
            camera_channel_buff_cfg(cxt, &buf_cfg);
            return;
        }
    }

    receiver_handle = cxt->grab_cxt.caller_handle[frame->channel_id];

    if (setting_param.cmd_type_value == CAMERA_MODE_NIGHT_PHOTO &&
        1 == camera_get_3dnr_flag(cxt) && (0 != cxt->snp_cxt.channel_bits) &&
        (TAKE_PICTURE_NEEDED == camera_get_snp_req((cmr_handle)cxt))) {
        cxt->dre_flag = 0;
        cxt->snp_cxt.sprd_3dnr_type = CAMERA_3DNR_TYPE_PREV_SW_CAP_SW;
        bzero(&src_sw_algorithm_buf, sizeof(struct image_sw_algorithm_buf));
        src_sw_algorithm_buf.height = cxt->snp_cxt.post_proc_setting.chn_out_frm[0].size.height;
        src_sw_algorithm_buf.width = cxt->snp_cxt.post_proc_setting.chn_out_frm[0].size.width;
        src_sw_algorithm_buf.fd = frame->fd;
        src_sw_algorithm_buf.format = frame->fmt;
        src_sw_algorithm_buf.y_vir_addr = frame->yaddr_vir;
        src_sw_algorithm_buf.y_phy_addr = frame->yaddr;

        // dst_sw_algorithm_buf use first intput frame for now
        if (sw_algorithm_buf_cnt == 0) {
            bzero(&dst_sw_algorithm_buf, sizeof(struct image_sw_algorithm_buf));
            dst_sw_algorithm_buf.height = cxt->snp_cxt.post_proc_setting.chn_out_frm[0].size.height;
            dst_sw_algorithm_buf.width = cxt->snp_cxt.post_proc_setting.chn_out_frm[0].size.width;
            dst_sw_algorithm_buf.fd = frame->fd;
            dst_sw_algorithm_buf.format = frame->fmt;
            dst_sw_algorithm_buf.y_vir_addr = frame->yaddr_vir;
            dst_sw_algorithm_buf.y_phy_addr = frame->yaddr;
        }
        sw_algorithm_buf_cnt++;
        sw_algorithm_buf_cnt = sw_algorithm_buf_cnt % CAP_3DNR_NUM;
    }

    if ((0 != cxt->snp_cxt.channel_bits) &&
        (TAKE_PICTURE_NEEDED == camera_get_snp_req((cmr_handle)cxt)) &&
        ((1 == camera_get_hdr_flag(cxt)) || (1 == camera_get_3dnr_flag(cxt)))) {
        struct img_frm out_param;
        struct ipm_frame_in ipm_in_param;
        struct ipm_frame_out imp_out_param;
        cmr_uint vir_addr_y = 0;
        void *threednr_handle = NULL;
        cmr_bzero(&out_param, sizeof(out_param));
        cmr_bzero(&ipm_in_param, sizeof(ipm_in_param));
        cmr_bzero(&imp_out_param, sizeof(imp_out_param));

        /* for bug 396318, will be removed later */
        // camera_set_discard_frame((cmr_handle)cxt, 1);
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, evt, data);
        if (ret) {
            CMR_LOGE("failed to send a frame to preview %ld", ret);
        }
        if (camera_is_need_change_fmt((cmr_handle)cxt, frame)) {
            ret = cmr_snapshot_format_convert((cmr_handle)cxt, data,
                                              &out_param); // sync
            if (ret) {
                CMR_LOGE("failed to format convert %ld", ret);
                goto exit;
            }
        } else {
            frm_id = camera_get_post_proc_chn_out_frm_id(
                cxt->snp_cxt.post_proc_setting.chn_out_frm, frame);
            /*if frm_id biger than 0,you should search hdr buffer in
              hdr buffer list. You can't use (frame->yaddr) on 64bit system*/
            if (frm_id >= CMR_CAPTURE_MEM_SUM) {
                if (1 == camera_get_hdr_flag(cxt))
                    ret = cmr_preview_get_hdr_buf(cxt->prev_cxt.preview_handle,
                                                  cxt->camera_id, frame,
                                                  &vir_addr_y);
                else if (1 == camera_get_3dnr_flag(cxt)) {
                    if (1 == camera_get_sw_3dnr_flag(cxt))
                        ret = cmr_preview_get_3dnr_buf_extra(
                            cxt->prev_cxt.preview_handle, cxt->camera_id, frame,
                            &vir_addr_y, 1, &threednr_handle);
                    else
                        ret = cmr_preview_get_3dnr_buf(
                            cxt->prev_cxt.preview_handle, cxt->camera_id, frame,
                            &vir_addr_y);
                }

                out_param.size =
                    cxt->snp_cxt.post_proc_setting.chn_out_frm[0].size;
                out_param.fd = frame->fd;
                out_param.addr_vir.addr_y = vir_addr_y;
                out_param.addr_phy.addr_y = frame->yaddr;
                out_param.addr_phy.addr_u = frame->uaddr;
                out_param.addr_phy.addr_v = frame->vaddr;
                out_param.fmt = frame->fmt;
                src_sw_algorithm_buf.phy_addr_u = frame->uaddr;
                src_sw_algorithm_buf.phy_addr_v = frame->vaddr;
                if (ret) {
                    CMR_LOGE("failed to get hdr buffer %ld", ret);
                    goto exit;
                }
            } else {
                /*if frm_id is 0,use default chn_out_frm.
                  This is also dest img buffer         */
                out_param = cxt->snp_cxt.post_proc_setting.chn_out_frm[0];
                if ((1 == camera_get_3dnr_flag(cxt)) &&
                    1 == camera_get_sw_3dnr_flag(cxt)) {
                    struct frm_info frame;
                    frame.yaddr = out_param.addr_vir.addr_y;
                    frame.uaddr = out_param.addr_phy.addr_u;
                    frame.vaddr = out_param.addr_phy.addr_v;
                    frame.fd = out_param.fd;
                    src_sw_algorithm_buf.phy_addr_u = frame.uaddr;
                    src_sw_algorithm_buf.phy_addr_v = frame.vaddr;
                    ret = cmr_preview_get_3dnr_buf_extra(
                        cxt->prev_cxt.preview_handle, cxt->camera_id, &frame,
                        &vir_addr_y, 0, &threednr_handle);
                }
            }
        }

        ipm_in_param.dst_frame = cxt->snp_cxt.post_proc_setting.chn_out_frm[0];
        cxt->snp_cxt.cur_frm_info = *frame;
        ipm_cxt->frm_num++;
        ipm_in_param.src_frame = out_param;
        ipm_in_param.private_data = (void *)privdata;
        if (cxt->ipm_cxt.hdr_version.major != 1) {
            memcpy(&ipm_in_param.ev[0], &cxt->snp_cxt.hdr_ev[0],
                   ((HDR_CAP_NUM)-1) * sizeof(float));
        }
        imp_out_param.dst_frame = out_param;
        imp_out_param.private_data = privdata;
        if (1 == camera_get_hdr_flag(cxt)) {
            ret = ipm_transfer_frame(ipm_cxt->hdr_handle, &ipm_in_param,
                                     &imp_out_param);
        } else if (1 == camera_get_3dnr_flag(cxt)) {
            if (1 == camera_get_sw_3dnr_flag(cxt)) {
                if (ipm_in_param.src_frame.size.width * 10 /
                        ipm_in_param.src_frame.size.height <=
                    13) {
                    imp_out_param.dst_frame.size.width =
                        CMR_3DNR_4_3_SMALL_WIDTH;
                    imp_out_param.dst_frame.size.height =
                        CMR_3DNR_4_3_SMALL_HEIGHT;
                } else if (ipm_in_param.src_frame.size.width * 10 /
                               ipm_in_param.src_frame.size.height <=
                           17) {
                    imp_out_param.dst_frame.size.width =
                        CMR_3DNR_16_9_SMALL_WIDTH;
                    imp_out_param.dst_frame.size.height =
                        CMR_3DNR_16_9_SMALL_HEIGHT;
                }
                imp_out_param.private_data = threednr_handle;
            }
            if (setting_param.cmd_type_value == CAMERA_MODE_NIGHT_PHOTO) {
                src_sw_algorithm_buf.reserved = threednr_handle;
                ret == cxt->night_cxt.sw_process((cmr_handle *)privdata,
                                                &src_sw_algorithm_buf, &dst_sw_algorithm_buf);
            } else {
                ret = ipm_transfer_frame(ipm_cxt->threednr_handle, &ipm_in_param,
                                         &imp_out_param);
            }
        }
        if (ret) {
            CMR_LOGE("failed to transfer frame to ipm %ld", ret);
            goto exit;
        }
    } else {
        camera_send_channel_data((cmr_handle)cxt, receiver_handle, evt, data);
    }
exit:
    ATRACE_END();
    return;
}

void camera_grab_evt_cb(cmr_int evt, void *data, void *privdata) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;
    struct frm_info *frame = (struct frm_info *)data;
    cmr_u32 channel_id;
    cmr_handle receiver_handle;

    if (!cxt || !data || !privdata ||
        CMR_EVT_GRAB_BASE != (CMR_EVT_GRAB_BASE & evt)) {
        CMR_LOGE("error param, handle 0x%lx data 0x%lx evt 0x%lx",
                 (cmr_uint)cxt, (cmr_uint)data, evt);
        return;
    }
    CMR_LOGV("evt 0x%lx, handle 0x%lx", evt, (cmr_uint)privdata);

    channel_id = frame->channel_id;
    if (channel_id >= GRAB_CHANNEL_MAX) {
        CMR_LOGE("error param, channel id %d", channel_id);
        return;
    }

    switch (evt) {
    case CMR_GRAB_TX_DONE:
#if defined OEM_HANDLE_HDR || defined OEM_HANDLE_3DNR
        camera_grab_handle(evt, data, privdata);
#else
        camera_send_channel_data((cmr_handle)cxt, receiver_handle, evt, data);
#endif
        break;
    case CMR_GRAB_CANCELED_BUF:
    case CMR_GRAB_TX_ERROR:
    case CMR_GRAB_TX_NO_MEM:
    case CMR_GRAB_CSI2_ERR:
    case CMR_GRAB_TIME_OUT:
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, evt, data);
        if (ret) {
            CMR_LOGE("fail to handle error, ret %ld", ret);
        }
        break;
    default:
        CMR_LOGE("don't support evt 0x%lx", evt);
        break;
    }
}

void camera_isp_dev_evt_cb(cmr_int evt, void *data, cmr_u32 data_len,
                           void *privdata) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;
    struct frm_info *frame = (struct frm_info *)data;
    cmr_u32 channel_id;
    cmr_handle receiver_handle;
    cmr_int flash_status = FLASH_CLOSE;
    struct setting_cmd_parameter setting_param;
}

void camera_grab_post_ynr_evt_cb(cmr_int evt, void *data, void *privdata) {
    struct camera_context *cxt = NULL;

    if (!privdata) {
        CMR_LOGE("err oem post ynr param, privdata is %p", privdata);
        return;
    }
    cxt = (struct camera_context *)privdata;
    if (cxt->isp_cxt.isp_handle) {
        // isp_ynr_post_proc(cxt->isp_cxt.isp_handle);
    }
    return;
}
void camera_grab_3dnr_evt_cb(cmr_int evt, void *data, void *privdata) {
    ATRACE_BEGIN(__FUNCTION__);
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;

    if (!cxt || !data || !privdata) {
        CMR_LOGE("err, scale callback param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return;
    }

    if (cxt->ipm_cxt.threednr_handle) {
        ret = cmr_ipm_post_proc(cxt->ipm_cxt.threednr_handle);
    } else {
        ret = prev_3dnr_evt_cb(cxt->prev_cxt.preview_handle, cxt->camera_id);
    }
    if (ret) {
        CMR_LOGE("fail to call_back for 3ndr %ld", ret);
    }
    ATRACE_END();
}

void camera_scaler_evt_cb(cmr_int evt, void *data, void *privdata) {
    ATRACE_BEGIN(__FUNCTION__);
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;

    if (!cxt || !data || !privdata) {
        CMR_LOGE("err, scale callback param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return;
    }
    CMR_LOGI("evt 0x%lx, handle 0x%lx", evt, (cmr_uint)privdata);

    if (CMR_IMG_CVT_SC_DONE == evt) {
        camera_take_snapshot_step(CMR_STEP_SC_E);
        cmr_snapshot_receive_data((cmr_handle)privdata, SNAPSHOT_EVT_SC_DONE,
                                  data);
    } else {
        CMR_LOGE("err, don't support evt 0x%lx", evt);
    }
    ATRACE_END();
}

void camera_jpeg_evt_cb(enum jpg_jpeg_evt evt, void *data, void *privdata) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;
    cmr_u32 temp_evt;

    if (NULL == data || !privdata ||
        CMR_EVT_JPEG_BASE != (CMR_EVT_JPEG_BASE & evt)) {
        CMR_LOGE("err, param, 0x%lx 0x%lx", (cmr_uint)data, evt);
        return;
    }
    CMR_LOGI("evt 0x%lx, handle 0x%lx", evt, (cmr_uint)privdata);

    switch (evt) {
    case CMR_JPEG_ENC_DONE:
        CMR_LOGI("evt CMR_JPEG_ENC_DONE");
        camera_take_snapshot_step(CMR_STEP_JPG_ENC_E);
        temp_evt = SNAPSHOT_EVT_JPEG_ENC_DONE;
        break;
    case CMR_JPEG_DEC_DONE:
        CMR_LOGI("evt CMR_JPEG_DEC_DONE");
        temp_evt = SNAPSHOT_EVT_JPEG_DEC_DONE;
        break;
    case CMR_JPEG_ENC_ERR:
        temp_evt = SNAPSHOT_EVT_JPEG_ENC_ERR;
        break;
    case CMR_JPEG_DEC_ERR:
        temp_evt = SNAPSHOT_EVT_JPEG_DEC_ERR;
        break;
    default:
        ret = -CMR_CAMERA_NO_SUPPORT;
        CMR_LOGE("err, don't support evt 0x%lx", evt);
    }
    if (ret) {
        CMR_LOGE("done %ld", ret);
        return;
    }
    ret =
        cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle, temp_evt, data);
    if (ret) {
        CMR_LOGE("failed %ld", ret);
    }
}

static cmr_int camera_isp_ctrl_flash(cmr_handle setting_handle, void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct setting_cmd_parameter setting_param;
    cmr_bzero(&setting_param, sizeof(struct setting_cmd_parameter));
    cmr_int flash_type = 0;
    if (NULL == data) {
        flash_type = FLASH_CLOSE;
        goto ctrl_flash;
    }

    switch (*(cmr_int *)data) {
    case 0:
        flash_type = FLASH_CLOSE;
        break;
    case 1:
        flash_type = FLASH_OPEN;
        break;
    case 2:
        flash_type = FLASH_HIGH_LIGHT;
        break;
    default:
        flash_type = FLASH_CLOSE;
        break;
    }

ctrl_flash:
    setting_param.ctrl_flash.flash_type = flash_type;
    ret = cmr_setting_ioctl(setting_handle, CAMERA_PARAM_ISP_FLASH,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to open flash");
    }

    return ret;
}

cmr_int camera_isp_evt_cb(cmr_handle oem_handle, cmr_u32 evt, void *data,
                          cmr_u32 data_len) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_u32 sub_type;
    cmr_u32 cmd = evt & 0xFF;
    cmr_int oem_cb;
    cmr_u32 *ae_info = NULL;

    if (!oem_handle || CMR_EVT_ISP_BASE != (CMR_EVT_ISP_BASE & evt)) {
        CMR_LOGE("err param, 0x%lx 0x%x 0x%lx", (cmr_uint)data, evt,
                 (cmr_uint)oem_handle);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGV("evt 0x%x, handle 0x%lx len %d", evt, (cmr_uint)oem_handle,
             data_len);

    sub_type = (~CMR_EVT_ISP_BASE) & evt;
    CMR_LOGV("sub_type %0x", sub_type);

    if ((sub_type & ISP_EVT_MASK) == 0) {
        ret = camera_isp_ctrl_done(cmd, data);
        goto exit;
    }

    switch (sub_type & ISP_EVT_MASK) {
    case ISP_PROC_CALLBACK:
        if (cxt->is_multi_mode == MODE_SBS) {
            CMR_LOGD("sbs yuv callback done,is_multi_mode %d",
                     cxt->is_multi_mode);
            sem_post(&cxt->sbs_sync_sm);
        } else if (camera_is_need_change_fmt((cmr_handle)cxt, data)) {
            ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                            SNAPSHOT_EVT_CVT_RAW_DATA, data);
        } else {
            ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                            SNAPSHOT_EVT_RAW_PROC, data);
        }
        break;
    case ISP_AF_NOTICE_CALLBACK:
        CMR_LOGV("ISP_AF_NOTICE_CALLBACK come here");
        if (1 == cxt->focus_cxt.inited) {
            ret = cmr_focus_isp_handle(cxt->focus_cxt.focus_handle,
                                       FOCUS_EVT_ISP_AF_NOTICE, cxt->camera_id,
                                       data);
        }
        break;
    /*	case ISP_FLASH_AE_CALLBACK:
                    ret = cmr_setting_isp_alg_done(oem_handle, data);
                    break;
            case ISP_AE_BAPASS_CALLBACK:
                    ret = cmr_setting_isp_alg_done(oem_handle, data);
                    break;
            case ISP_AF_STAT_CALLBACK:
                    ret = camera_isp_af_stat(data);
                    break;*/
    case ISP_AE_STAB_CALLBACK:
        ret =
            cmr_setting_isp_notice_done(cxt->setting_cxt.setting_handle, data);
        break;
    case ISP_QUICK_MODE_DOWN:
        cmr_setting_quick_ae_notice_done(cxt->setting_cxt.setting_handle, data);
        CMR_LOGI("ISP_QUICK_MODE_DOWN");
        break;
    case ISP_ONLINE_FLASH_CALLBACK:
        camera_isp_ctrl_flash(cxt->setting_cxt.setting_handle, data);
        break;
    case ISP_AE_STAB_NOTIFY:
        CMR_LOGV("ISP_AE_STAB_NOTIFY");
        oem_cb = CAMERA_EVT_CB_AE_STAB_NOTIFY;
        if (data != NULL) {
            ae_info = (cmr_u32 *)data;
            cxt->camera_cb(oem_cb, cxt->client_data,
                           CAMERA_FUNC_AE_STATE_CALLBACK, ae_info);
            cmr_preview_facedetect_set_ae_stab(cxt->prev_cxt.preview_handle,
                                               cxt->camera_id, ae_info);
        } else
            cxt->camera_cb(oem_cb, cxt->client_data,
                           CAMERA_FUNC_AE_STATE_CALLBACK, NULL);
        break;
    case ISP_AE_LOCK_NOTIFY:
        CMR_LOGI("ISP_AE_LOCK_NOTIFY");
        oem_cb = CAMERA_EVT_CB_AE_LOCK_NOTIFY;
        cxt->camera_cb(oem_cb, cxt->client_data, CAMERA_FUNC_AE_STATE_CALLBACK,
                       NULL);
        break;
    case ISP_AE_UNLOCK_NOTIFY:
        CMR_LOGI("ISP_AE_UNLOCK_NOTIFY");
        oem_cb = CAMERA_EVT_CB_AE_UNLOCK_NOTIFY;
        cxt->camera_cb(oem_cb, cxt->client_data, CAMERA_FUNC_AE_STATE_CALLBACK,
                       NULL);
        break;
    case ISP_AE_SYNC_INFO: {
        struct ispae_sync_info_output *info_output =
            (struct ispae_sync_info_output *)data;
        // TBD write info to al3200
    }
    case ISP_AE_EXP_TIME:
        CMR_LOGV("ISP_AE_EXP_TIME,data %lld", *(uint64_t *)data);
        prev_set_ae_time(cxt->prev_cxt.preview_handle, cxt->camera_id, data);
        break;
    /*    case ISP_VCM_STEP:
            CMR_LOGI("ISP_VCM_STEP,data %d", *(uint32_t *)data);
            prev_set_vcm_step(cxt->prev_cxt.preview_handle, cxt->camera_id,
       data);
            break;*/
    case ISP_HDR_EV_EFFECT_CALLBACK:
        CMR_LOGD("ISP_HDR_EV_EFFECT_CALLBACK");
        camera_set_hdr_ev(oem_handle, data);
        cmr_setting_isp_notice_done(cxt->setting_cxt.setting_handle, data);
        break;
    case ISP_AE_CB_FLASH_FIRED:
        oem_cb = CAMERA_EVT_CB_AE_FLASH_FIRED;
        cxt->camera_cb(oem_cb, cxt->client_data, CAMERA_FUNC_AE_STATE_CALLBACK,
                       data);
        break;
    case ISP_AUTO_HDR_STATUS_CALLBACK:
        oem_cb = CAMERA_EVT_CB_HDR_SCENE;
        cxt->camera_cb(oem_cb, cxt->client_data, CAMERA_FUNC_AE_STATE_CALLBACK,
                       data);
        break;
    case ISP_HIST_REPORT_CALLBACK:
        CMR_LOGD("ISP_HIST_REPORT_CALLBACK");
        oem_cb = CAMERA_EVT_CB_HIST_REPORT;
        cxt->camera_cb(oem_cb, cxt->client_data, CAMERA_FUNC_AE_STATE_CALLBACK,
                       data);
        cmr_preview_facedetect_set_hist(cxt->prev_cxt.preview_handle,
                                        cxt->camera_id, data);
        break;
    default:
        break;
    }
exit:
    return ret;
}

void camera_focus_evt_cb(enum af_cb_type cb, cmr_uint param, void *privdata) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)privdata;
    cmr_int oem_cb;
    struct cmr_focus_status *focus_status;

    if (!privdata) {
        CMR_LOGE("err, handle for callback");
        return;
    }
    switch (cb) {
    case AF_CB_DONE:
        oem_cb = CAMERA_EXIT_CB_DONE;
        cxt->is_focus = param;
        if (param) {
            return;
        }
        break;
    case AF_CB_FAILED:
        oem_cb = CAMERA_EXIT_CB_FAILED;
        cxt->is_focus = 0;
        break;
    case AF_CB_ABORT:
        oem_cb = CAMERA_EXIT_CB_ABORT;
        break;
    case AF_CB_FOCUS_MOVE:
        oem_cb = CAMERA_EVT_CB_FOCUS_MOVE;
        focus_status = (struct cmr_focus_status *)param;
        cxt->is_focus = focus_status->is_in_focus;
        if (cxt->is_focus) {
            cxt->focus_rect.x = 0;
            cxt->focus_rect.y = 0;
        }
        break;
    default:
        CMR_LOGE("failed focus cb %d", cb);
        ret = -CMR_CAMERA_NO_SUPPORT;
    }
    if (ret) {
        CMR_LOGE("done %ld", ret);
        return;
    }
    CMR_LOGD("param =0x%lx camera_cb 0x%lx focus cb %ld, oem cb 0x%lx",
             (cmr_uint)param, (cmr_uint)cb, (cmr_uint)oem_cb,
             (cmr_uint)cxt->camera_cb);
    if (cxt->camera_cb) {
        CMR_LOGV("cxt->camera_cb run");
        cxt->camera_cb(oem_cb, cxt->client_data, CAMERA_FUNC_START_FOCUS,
                       (void *)param);
    } else {
        CMR_LOGD("cxt->camera_cb null error");
    }
}

cmr_int camera_preview_cb(cmr_handle oem_handle, enum preview_cb_type cb_type,
                          enum preview_func_type func, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_uint oem_func;
    cmr_uint oem_cb_type;
    CMR_MSG_INIT(message);

    if (!oem_handle) {
        CMR_LOGE("error handle");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (PREVIEW_FUNC_START_PREVIEW == func) {
        oem_func = CAMERA_FUNC_START_PREVIEW;
    } else if (PREVIEW_FUNC_STOP_PREVIEW == func) {
        oem_func = CAMERA_FUNC_STOP_PREVIEW;
    } else if (PREVIEW_FUNC_START_CAPTURE == func) {
        oem_func = CAMERA_FUNC_TAKE_PICTURE;
    } else {
        CMR_LOGE("err, %d", func);
        goto exit;
    }
    CMR_LOGV("%ld %d", oem_func, cb_type);
    switch (cb_type) {
    case PREVIEW_RSP_CB_SUCCESS:
        oem_cb_type = CAMERA_RSP_CB_SUCCESS;
        break;
    case PREVIEW_EVT_CB_FRAME:
        oem_cb_type = CAMERA_EVT_CB_FRAME;
        break;
    case PREVIEW_EXIT_CB_FAILED:
        oem_cb_type = CAMERA_EXIT_CB_FAILED;
        break;
    case PREVIEW_EVT_CB_FLUSH:
        oem_cb_type = CAMERA_EVT_CB_FLUSH;
        break;
    case PREVIEW_EVT_CB_FD:
        oem_cb_type = CAMERA_EVT_CB_FD;
        if (param) {
            struct camera_frame_type *frame_param =
                (struct camera_frame_type *)param;
            struct isp_face_area face_area;
            int32_t sx = 0;
            int32_t sy = 0;
            int32_t ex = 0;
            int32_t ey = 0;
            int32_t i = 0;
            struct img_rect src_prev_rect;
            struct sensor_mode_info *sensor_mode_info = NULL;
            cmr_u32 sn_mode = 0;
            cmr_uint face_info_max_num =
                sizeof(face_area.face_info) / sizeof(struct isp_face_info);

            cxt->fd_face_area.frame_width = frame_param->width;
            cxt->fd_face_area.frame_height = frame_param->height;
            cxt->fd_face_area.face_num = frame_param->face_num;

            cmr_bzero(&face_area, sizeof(struct isp_face_area));
            // note:now we get the preview face crop.but ISP need sensor's
            // crop.so we need recovery crop.
            cmr_preview_get_prev_rect(cxt->prev_cxt.preview_handle,
                                      cxt->camera_id, &src_prev_rect);

            cmr_sensor_get_mode(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                &sn_mode);
            sensor_mode_info = &cxt->sn_cxt.sensor_info.mode_info[sn_mode];
            if (MODE_SBS == cxt->is_multi_mode) {
                face_area.frame_width = sensor_mode_info->scaler_trim.width;
                face_area.frame_height = sensor_mode_info->scaler_trim.height;
            } else {
                face_area.frame_width = sensor_mode_info->trim_width;
                face_area.frame_height = sensor_mode_info->trim_height;
            }
            face_area.face_num = frame_param->face_num;
            CMR_LOGV("face_num %d, size:%dx%d", face_area.face_num,
                     face_area.frame_width, face_area.frame_height);

            if (face_info_max_num < face_area.face_num) {
                face_area.face_num = face_info_max_num;
            }

            for (i = 0; i < face_area.face_num; i++) {
                sx = MIN(MIN(frame_param->face_info[i].sx,
                             frame_param->face_info[i].srx),
                         MIN(frame_param->face_info[i].ex,
                             frame_param->face_info[i].elx));
                sy = MIN(MIN(frame_param->face_info[i].sy,
                             frame_param->face_info[i].sry),
                         MIN(frame_param->face_info[i].ey,
                             frame_param->face_info[i].ely));
                ex = MAX(MAX(frame_param->face_info[i].sx,
                             frame_param->face_info[i].srx),
                         MAX(frame_param->face_info[i].ex,
                             frame_param->face_info[i].elx));
                ey = MAX(MAX(frame_param->face_info[i].sy,
                             frame_param->face_info[i].sry),
                         MAX(frame_param->face_info[i].ey,
                             frame_param->face_info[i].ely));
                // save face info in cmr cxt for other case.such as face beauty
                // takepicture
                cxt->fd_face_area.face_info[i].sx = sx;
                cxt->fd_face_area.face_info[i].sy = sy;
                cxt->fd_face_area.face_info[i].ex = ex;
                cxt->fd_face_area.face_info[i].ey = ey;
                cxt->fd_face_area.face_info[i].angle =
                    frame_param->face_info[i].angle;
                cxt->fd_face_area.face_info[i].pose =
                    frame_param->face_info[i].pose;

                face_area.face_info[i].sx = 1.0 * sx *
                                            (float)face_area.frame_width /
                                            (float)frame_param->width;
                face_area.face_info[i].sy = 1.0 * sy *
                                            (float)face_area.frame_height /
                                            (float)frame_param->height;
                face_area.face_info[i].ex = 1.0 * ex *
                                            (float)face_area.frame_width /
                                            (float)frame_param->width;
                face_area.face_info[i].ey = 1.0 * ey *
                                            (float)face_area.frame_height /
                                            (float)frame_param->height;
                face_area.face_info[i].brightness =
                    frame_param->face_info[i].brightness;
                face_area.face_info[i].angle = frame_param->face_info[i].angle;
                face_area.face_info[i].pose = frame_param->face_info[i].pose;
                CMR_LOGV("preview face info sx %d sy %d ex %d, ey %d",
                         face_area.face_info[i].sx, face_area.face_info[i].sy,
                         face_area.face_info[i].ex, face_area.face_info[i].ey);
            }
            // SBS requires to disable FAF
            if (MODE_SBS == cxt->is_multi_mode) {
                CMR_LOGV("SBS MODE disable FAF");
            } else {
                /* SS requires to disable FD when HDR is on */
                if (IMG_DATA_TYPE_RAW == cxt->sn_cxt.sensor_info.image_format &&
                    (!cxt->is_vendor_hdr)) {
                    isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_FACE_AREA,
                              (void *)&face_area);
                }
            }
        }

        break;
    case PREVIEW_EXIT_CB_PREPARE:
        oem_cb_type = CAMERA_EXIT_CB_PREPARE;
        break;
    case PREVIEW_EVT_CB_RESUME:
        oem_cb_type = CAMERA_EVT_CB_RESUME;
        break;

    default:
        CMR_LOGE("err, %d", cb_type);
        ret = -CMR_CAMERA_NO_SUPPORT;
    }
    if (ret) {
        goto exit;
    }

    /*if (CAMERA_FUNC_STOP_PREVIEW == oem_func && CAMERA_RSP_CB_SUCCESS ==
    oem_cb_type) {
            CMR_LOGV("stop preview response, notify directly");
            if (cxt->camera_cb) {
                    cxt->camera_cb(oem_cb_type, cxt->client_data, oem_func,
    param);
            }
            return ret;
    }*/

    if (param) {
        message.data = malloc(sizeof(struct camera_frame_type));
        if (!message.data) {
            CMR_LOGE("failed to malloc msg");
            ret = -CMR_CAMERA_NO_MEM;
            goto exit;
        }
        message.alloc_flag = 1;

        if (PREVIEW_EVT_CB_FRAME == cb_type &&
            CAMERA_FUNC_START_PREVIEW == oem_func) {
            struct exif_spec_pic_taking_cond_tag exif_pic_info;
            struct camera_frame_type *preview_frame =
                (struct camera_frame_type *)param;
        }

        if ((cxt->is_lls_enable) && (PREVIEW_EVT_CB_FRAME == cb_type)) {
            struct camera_frame_type *prev_frame =
                (struct camera_frame_type *)param;
            prev_frame->lls_info = camera_get_ae_lum_value(oem_handle);
        }

        memcpy(message.data, param, sizeof(struct camera_frame_type));
    }
    message.msg_type = oem_func;
    message.sub_msg_type = oem_cb_type;
    message.sync_flag = CMR_MSG_SYNC_NONE;
    ret = cmr_thread_msg_send(cxt->prev_cb_thr_handle, &message);
    if (ret) {
        CMR_LOGE("failed to send msg, ret %ld", ret);
        free(message.data);
    }
exit:
    return ret;
}

cmr_int camera_ipm_cb(cmr_u32 class_type, struct ipm_frame_out *cb_param) {
    UNUSED(class_type);
    cmr_int ret = CMR_CAMERA_SUCCESS;
    if (!cb_param) {
        CMR_LOGE("error param");
        return -CMR_CAMERA_INVALID_PARAM;
    }
    struct camera_context *cxt =
        (struct camera_context *)cb_param->private_data;
    struct frm_info frame;
    cmr_u32 width = 0, height = 0;

    if (!cb_param || !cb_param->private_data) {
        CMR_LOGE("error param");
        return -CMR_CAMERA_INVALID_PARAM;
    }

    CMR_LOGD("E");

    // for cache coherency
    cmr_snapshot_memory_flush(cxt->snp_cxt.snapshot_handle,
                              &cb_param->dst_frame);

    camera_post_share_path_available((cmr_handle)cxt);
    cxt->ipm_cxt.frm_num = 0;

    frame = cxt->snp_cxt.cur_frm_info;
    if (1 == camera_get_hdr_flag(cxt)) {
        ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                        SNAPSHOT_EVT_HDR_DONE, &frame);
    } else if (1 == camera_get_3dnr_flag(cxt)) {
        frame = cxt->snp_cxt.cur_frm_info;
        cmr_snapshot_memory_flush(cxt->snp_cxt.snapshot_handle,
                                  &cb_param->dst_frame);
        camera_post_share_path_available((cmr_handle)cxt);
        cxt->ipm_cxt.frm_num = 0;
        ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                        SNAPSHOT_EVT_3DNR_DONE, &frame);
        sem_post(&cxt->threednr_proc_sm);
    }
    if (ret) {
        CMR_LOGE("fail to send frame to snp %ld", ret);
        goto exit;
    }

    CMR_LOGD("X");

exit:
    return ret;
}

void camera_snapshot_cb_to_hal(cmr_handle oem_handle, enum snapshot_cb_type cb,
                               enum snapshot_func_type func, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_uint oem_func = CAMERA_FUNC_TYPE_MAX;
    cmr_uint oem_cb_type = CAMERA_CB_TYPE_MAX;
    cmr_handle send_thr_handle = cxt->snp_cb_thr_handle;
    struct camera_frame_type *frame_ptr = NULL;
    struct isp_af_ts af_ts;

    CMR_MSG_INIT(message);

    switch (func) {
    case SNAPSHOT_FUNC_RELEASE_PICTURE:
        oem_func = CAMERA_FUNC_RELEASE_PICTURE;
        break;
    case SNAPSHOT_FUNC_TAKE_PICTURE:
        oem_func = CAMERA_FUNC_TAKE_PICTURE;
        break;
    case SNAPSHOT_FUNC_ENCODE_PICTURE:
        oem_func = CAMERA_FUNC_ENCODE_PICTURE;
        break;
    default:
        oem_func = func;
        break;
    }

    switch (cb) {
    case SNAPSHOT_RSP_CB_SUCCESS:
        oem_cb_type = CAMERA_RSP_CB_SUCCESS;
        break;
    case SNAPSHOT_EVT_CB_FLUSH:
        oem_cb_type = CAMERA_EVT_CB_FLUSH;
        break;
    case SNAPSHOT_EVT_CB_ZSL_NEW_FRM:
        oem_cb_type = CAMERA_EVT_CB_ZSL_FRM;
        break;
    case SNAPSHOT_EXIT_CB_FAILED:
        oem_cb_type = CAMERA_EXIT_CB_FAILED;
        break;
    case SNAPSHOT_EVT_CB_SNAPSHOT_DONE:
        oem_cb_type = CAMERA_EVT_CB_SNAPSHOT_DONE;
        send_thr_handle = cxt->snp_send_raw_image_handle;
        break;
    case SNAPSHOT_EXIT_CB_DONE:
        if (0 == cxt->camera_id) {
            af_ts.timestamp = ((struct camera_frame_type *)param)->timestamp;
            af_ts.capture = 1;
            isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_SET_DCAM_TIMESTAMP,
                      &af_ts);
        }
        oem_cb_type = CAMERA_EXIT_CB_DONE;
        if (CAMERA_FUNC_TAKE_PICTURE == oem_func) {
            send_thr_handle = cxt->snp_send_raw_image_handle;
        }
        break;
    case SNAPSHOT_EVT_CB_CAPTURE_FRAME_DONE:
        oem_cb_type = CAMERA_EVT_CB_CAPTURE_FRAME_DONE;
        send_thr_handle = cxt->snp_secondary_thr_handle;
        break;
    case SNAPSHOT_EVT_CB_SNAPSHOT_JPEG_DONE:
        oem_cb_type = CAMERA_EVT_CB_SNAPSHOT_JPEG_DONE;
        break;
    case SNAPSHOT_EVT_RETURN_ZSL_BUF:
        oem_cb_type = CAMERA_EVT_CB_RETURN_ZSL_BUF;
        break;
    case SNAPSHOT_EXIT_CB_PREPARE:
        oem_cb_type = CAMERA_EXIT_CB_PREPARE;
        break;
    default:
        oem_cb_type = cb;
        break;
    }

    CMR_LOGI("camera_cb %ld %ld", oem_cb_type, oem_func);

    // TBD: remove camera_frame_type and cam_ion_buffer_t, only data and size
    if (param) {
        if (oem_cb_type == CAMERA_EVT_CB_FLUSH) {
            message.data = malloc(sizeof(cam_ion_buffer_t));
            if (!message.data) {
                CMR_LOGE("malloc failed");
                ret = -CMR_CAMERA_NO_MEM;
                goto exit;
            }
            message.alloc_flag = 1;
            memcpy(message.data, param, sizeof(cam_ion_buffer_t));
        } else {
            message.data = malloc(sizeof(struct camera_frame_type));
            if (!message.data) {
                CMR_LOGE("failed to malloc msg");
                ret = -CMR_CAMERA_NO_MEM;
                goto exit;
            }
            message.alloc_flag = 1;
            frame_ptr = (struct camera_frame_type *)message.data;
            memcpy(message.data, param, sizeof(struct camera_frame_type));
            struct exif_spec_pic_taking_cond_tag exif_pic_info;
            struct setting_cmd_parameter setting_param;
            EXIF_RATIONAL_T exposure_time;
            cmr_sensor_get_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                &exif_pic_info);
            frame_ptr->sensor_info.exposure_time_numerator =
                exif_pic_info.ExposureTime.numerator;
            frame_ptr->sensor_info.exposure_time_denominator =
                exif_pic_info.ExposureTime.denominator;
            exposure_time.numerator =
                frame_ptr->sensor_info.exposure_time_numerator;
            exposure_time.denominator =
                frame_ptr->sensor_info.exposure_time_denominator ;
            setting_param.camera_id = cxt->camera_id;
            setting_param.cmd_type_value = (cmr_uint)&exposure_time;
            cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                              SETTING_SET_EXIF_EXPOSURE_TIME, &setting_param);
        }
    }

    message.msg_type = oem_func;
    message.sub_msg_type = oem_cb_type;
    if (CAMERA_EVT_CB_CAPTURE_FRAME_DONE == oem_cb_type) {
        message.sync_flag = CMR_MSG_SYNC_RECEIVED;
    } else if (CAMERA_EXIT_CB_PREPARE == oem_cb_type ||
               CAMERA_EVT_CB_SNAPSHOT_JPEG_DONE == oem_cb_type ||
               CAMERA_EVT_CB_SNAPSHOT_DONE == oem_cb_type ||
               CAMERA_EVT_CB_RETURN_ZSL_BUF == oem_cb_type) {
        message.sync_flag = CMR_MSG_SYNC_NONE;
    } else {
        message.sync_flag = CMR_MSG_SYNC_PROCESSED;
    }
    ret = cmr_thread_msg_send(send_thr_handle, &message);
    if (ret) {
        CMR_LOGE("failed to send msg %ld", ret);
        if (message.data) {
            free(message.data);
        }
        goto exit;
    }

exit:
    CMR_LOGV("X");
}

void camera_set_3dnr_flag(struct camera_context *cxt, cmr_u32 threednr_flag) {
    char value[PROPERTY_VALUE_MAX] = {
        0,
    };
    CMR_LOGD("flag %d", threednr_flag);
    sem_wait(&cxt->threednr_flag_sm);
    property_get("debug.camera.3dnr.capture", value, "true");
    if (!strcmp(value, "false")) {
        threednr_flag = 0;
    }

    if (threednr_flag) { // the flag is set from user, and need reassign the
                         // value by the hw capability
        struct cmr_path_capability capability;
        cmr_bzero(&capability, sizeof(capability));
        cmr_grab_path_capability(cxt->grab_cxt.grab_handle, &capability);
        CMR_LOGD("'capability.support_3dnr_modes:%d",
                 capability.support_3dnr_mode);

        if (capability.support_3dnr_mode == SPRD_3DNR_SW) {
            threednr_flag = 1;
        } else if (capability.support_3dnr_mode == SPRD_3DNR_HW) {
            threednr_flag = 2;
        } else {
            threednr_flag = 0;
        }
    }

    cxt->snp_cxt.is_3dnr = threednr_flag;
    if (1 == cxt->snp_cxt.is_3dnr) {
        property_get("persist.vendor.cam.3dnr.version", value, "0");
        if (0 == atoi(value)) {
            cxt->snp_cxt.is_sw_3dnr = 0;
        } else {
            cxt->snp_cxt.is_sw_3dnr = 1;
        }
    } else {
        cxt->snp_cxt.is_sw_3dnr = 0;
    }
    CMR_LOGD("Set 3dnr is : %d, and sw 3dnr is : %d", cxt->snp_cxt.is_3dnr,
             cxt->snp_cxt.is_sw_3dnr);
    sem_post(&cxt->threednr_flag_sm);
}

cmr_u32 camera_get_3dnr_flag(struct camera_context *cxt) {
    cmr_u32 threednr_flag = 0;
    // sem_wait(&cxt->threednr_flag_sm);
    threednr_flag = cxt->snp_cxt.is_3dnr;
    // sem_post(&cxt->threednr_flag_sm);
    CMR_LOGD("%d", threednr_flag);
    return threednr_flag;
}

cmr_u32 camera_get_sw_3dnr_flag(struct camera_context *cxt) {
    cmr_u32 threednr_flag = 0;
    // sem_wait(&cxt->threednr_flag_sm);
    threednr_flag = cxt->snp_cxt.is_sw_3dnr;
    // sem_post(&cxt->threednr_flag_sm);
    CMR_LOGD("%d", threednr_flag);
    return threednr_flag;
}

cmr_u32 camera_get_cnr_realtime_flag(cmr_handle oem_handle) {

    int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;
    cmr_u32 cnr_ynr_flag = 0;

    cnr_ynr_flag = camera_get_cnr_flag(oem_handle);

#ifdef CAMERA_CNR3_ENABLE
    if (cnr_ynr_flag) {
        cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
        ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CNR2CNR3_YNR_EN, &isp_param);
        if (ret) {
            CMR_LOGE("isp get COM_ISP_GET_CNR2CNR3_YNR_EN  failed");
            return false;
        }
        CMR_LOGD("isp cnr3 enable %d", isp_param.cnr2cnr3_ynr_en);
        return isp_param.cnr2cnr3_ynr_en;
    }
#else
    if (cnr_ynr_flag) {
        cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
        ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CNR2_YNR_EN, &isp_param);
        if (ret) {
            CMR_LOGD("isp get COM_ISP_GET_CNR2_YNR_EN  failed");
            return false;
        }
        CMR_LOGD("isp cnr enable %d", isp_param.cnr2_ynr_en);
        return isp_param.cnr2_ynr_en;
    }
#endif

    return false;
}

cmr_u32 camera_get_cnr_flag(cmr_handle oem_handle) {

    int ret = CMR_CAMERA_SUCCESS;
    cmr_u32 cnr_flag = 0;
    struct setting_cmd_parameter setting_param;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = &cxt->setting_cxt;

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;

#ifndef CONFIG_CAMERA_CNR
    return ret;
#endif
    char value[PROPERTY_VALUE_MAX] = {
        0,
    };
    property_get("persist.vendor.cam.cnr.mode", value, "0");
    if (atoi(value)) {
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_CNRMODE, &setting_param);
        cnr_flag = setting_param.cmd_type_value;
        CMR_LOGV("cnr_flag is : %d", cnr_flag);
    }

    return cnr_flag;
}

cmr_int camera_open_3dnr(struct camera_context *cxt, struct ipm_open_in *in_ptr,
                         struct ipm_open_out *out_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    CMR_LOGI("start");
    sem_wait(&cxt->threednr_flag_sm);
    ret = cmr_ipm_open(cxt->ipm_cxt.ipm_handle, IPM_TYPE_3DNR, in_ptr, out_ptr,
                       &cxt->ipm_cxt.threednr_handle);
    sem_post(&cxt->threednr_flag_sm);
    CMR_LOGI("end");
    return ret;
}

cmr_int camera_close_3dnr(struct camera_context *cxt) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    sem_wait(&cxt->threednr_flag_sm);
    if (cxt->ipm_cxt.threednr_handle) {
        ret = cmr_ipm_close(cxt->ipm_cxt.threednr_handle);
        cxt->ipm_cxt.threednr_handle = 0;
    }
    sem_post(&cxt->threednr_flag_sm);
    sem_destroy(&cxt->threednr_proc_sm);
    CMR_LOGI("close 3dnr done %ld", ret);
    return ret;
}

cmr_int camera_open_cnr(struct camera_context *cxt, struct ipm_open_in *in_ptr,
                        struct ipm_open_out *out_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    CMR_LOGI("start");
    sem_wait(&cxt->cnr_flag_sm);
    if (NULL == cxt->ipm_cxt.cnr_handle) {
        ret = cmr_ipm_open(cxt->ipm_cxt.ipm_handle, IPM_TYPE_CNR, in_ptr,
                           out_ptr, &cxt->ipm_cxt.cnr_handle);
    }
    sem_post(&cxt->cnr_flag_sm);
    CMR_LOGI("end");

    return ret;
}

cmr_int camera_close_cnr(struct camera_context *cxt) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    sem_wait(&cxt->cnr_flag_sm);
    if (cxt->ipm_cxt.cnr_handle) {
        ret = cmr_ipm_close(cxt->ipm_cxt.cnr_handle);
        cxt->ipm_cxt.cnr_handle = 0;
    }
    sem_post(&cxt->cnr_flag_sm);
    CMR_LOGI("close cnr done %ld", ret);
    return ret;
}

void camera_set_hdr_flag(struct camera_context *cxt, cmr_u32 hdr_flag) {
    CMR_LOGV("flag %d", hdr_flag);
    sem_wait(&cxt->hdr_flag_sm);
    cxt->snp_cxt.is_hdr = hdr_flag;
    sem_post(&cxt->hdr_flag_sm);
}

cmr_u32 camera_get_hdr_flag(struct camera_context *cxt) {
    cmr_u32 hdr_flag = 0;
    // sem_wait(&cxt->hdr_flag_sm);
    hdr_flag = cxt->snp_cxt.is_hdr;
    // sem_post(&cxt->hdr_flag_sm);
    CMR_LOGV("%d", hdr_flag);
    return hdr_flag;
}

cmr_int camera_open_hdr(struct camera_context *cxt, struct ipm_open_in *in_ptr,
                        struct ipm_open_out *out_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    CMR_LOGI("start");
    sem_wait(&cxt->hdr_flag_sm);
    ret = cmr_ipm_open(cxt->ipm_cxt.ipm_handle, IPM_TYPE_HDR, in_ptr, out_ptr,
                       &cxt->ipm_cxt.hdr_handle);
    sem_post(&cxt->hdr_flag_sm);
    CMR_LOGI("end");
    return ret;
}

cmr_int camera_close_hdr(struct camera_context *cxt) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    sem_wait(&cxt->hdr_flag_sm);
    if (cxt->ipm_cxt.hdr_handle) {
        ret = cmr_ipm_close(cxt->ipm_cxt.hdr_handle);
        cxt->ipm_cxt.hdr_handle = 0;
    }
    sem_post(&cxt->hdr_flag_sm);
    CMR_LOGI("close hdr done %ld", ret);
    return ret;
}

void camera_set_filter_flag(struct camera_context *cxt, cmr_u32 filter_flag) {
    CMR_LOGV("flag %d", filter_flag);

    sem_wait(&cxt->filter_sm);
    cxt->snp_cxt.filter_type = filter_flag;
    sem_post(&cxt->filter_sm);
}

cmr_uint camera_get_filter_flag(struct camera_context *cxt) {
    cmr_u32 filter_flag = 0;

    sem_wait(&cxt->filter_sm);
    filter_flag = cxt->snp_cxt.filter_type;
    sem_post(&cxt->filter_sm);
    CMR_LOGD("%d", filter_flag);

    return filter_flag;
}

cmr_int camera_open_filter(struct camera_context *cxt,
                           struct ipm_open_in *in_ptr,
                           struct ipm_open_out *out_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    CMR_LOGI("start");
    sem_wait(&cxt->filter_sm);
    ret = cmr_ipm_open(cxt->ipm_cxt.ipm_handle, IPM_TYPE_FILTER, in_ptr,
                       out_ptr, &cxt->ipm_cxt.filter_handle);
    sem_post(&cxt->filter_sm);
    CMR_LOGI("end");
    return ret;
}

cmr_int camera_close_filter(struct camera_context *cxt) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    sem_wait(&cxt->filter_sm);
    if (cxt->ipm_cxt.filter_handle) {
        ret = cmr_ipm_close(cxt->ipm_cxt.filter_handle);
        cxt->ipm_cxt.filter_handle = 0;
    }
    sem_post(&cxt->filter_sm);
    CMR_LOGI("close filter done %ld", ret);
    return ret;
}

void camera_set_touch_xy(struct camera_context *cxt,
                         struct touch_coordinate touch_info) {
    CMR_LOGI("touch_info.touchX %d touch_info.touchY %d", touch_info.touchX,
             touch_info.touchY);
    cxt->snp_cxt.touch_xy.touchX = touch_info.touchX;
    cxt->snp_cxt.touch_xy.touchY = touch_info.touchY;
}

void camera_snapshot_channel_handle(cmr_handle oem_handle, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct frm_info frame;
    struct camera_jpeg_param *jpeg_param_ptr =
        &((struct camera_frame_type *)param)->jpeg_param;
    cmr_uint i;
    cmr_uint is_need_resume = 0;
    struct cmr_path_capability capability;
    cmr_int zsl_capture_need_pause;
    cmr_uint video_snapshot_type;
    struct setting_cmd_parameter setting_param;
    cmr_int normal_capture_need_pause;
#ifdef CAMERA_PATH_SHARE
    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    cmr_setting_ioctl(setting_cxt->setting_handle,
                      SETTING_GET_VIDEO_SNAPSHOT_TYPE, &setting_param);
    video_snapshot_type = setting_param.cmd_type_value;
    camera_local_zsl_snapshot_need_pause(oem_handle, &zsl_capture_need_pause);
    camera_local_normal_snapshot_need_pause(oem_handle,
                                            &normal_capture_need_pause);

    if (CAMERA_ZSL_MODE == cxt->snp_cxt.snp_mode &&
        video_snapshot_type != VIDEO_SNAPSHOT_VIDEO) {
        if (zsl_capture_need_pause || cxt->is_refocus_mode) {
            is_need_resume = 1;
        }
    } else if (1 != cxt->snp_cxt.total_num) {
        if (normal_capture_need_pause) {
            if (jpeg_param_ptr) {
                if (1 != jpeg_param_ptr->need_free &&
                    TAKE_PICTURE_NEEDED ==
                        camera_get_snp_req((cmr_handle)cxt) &&
                    1 != camera_get_hdr_flag(cxt) &&
                    1 != camera_get_3dnr_flag(cxt)) {
                    is_need_resume = 1;
                }
            }

            struct camera_frame_type *frame_type =
                (struct camera_frame_type *)param;
            if ((cxt->lls_shot_mode || cxt->is_vendor_hdr ||
                 cxt->is_pipviv_mode) &&
                (1 != frame_type->need_free)) {
                is_need_resume = 1;
            }
        } else {
            is_need_resume = 0;
        }
    }
    if (1 == is_need_resume) {
        for (i = 0; i < GRAB_CHANNEL_MAX; i++) {
            if (cxt->snp_cxt.channel_bits & (1 << i)) {
                break;
            }
        }
        frame.channel_id = i;
        ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, PREVIEW_CHN_RESUME,
                                       &frame);
        if (ret) {
            CMR_LOGE("failed to resume path %ld", ret);
        }
        CMR_LOGI("done");
    }
#endif
}
void camera_snapshot_state_handle(cmr_handle oem_handle,
                                  enum snapshot_cb_type cb,
                                  enum snapshot_func_type func, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    struct setting_context *setting_cxt = &cxt->setting_cxt;

    if(ret)
        CMR_LOGE("failed to get app mode");

    if (SNAPSHOT_FUNC_STATE == func) {
        switch (cb) {
        case SNAPSHOT_EVT_START_ROT:
            CMR_LOGI("start rot");
            break;
        case SNAPSHOT_EVT_ROT_DONE:
            CMR_LOGI("rot done");
            break;
        case SNAPSHOT_EVT_START_SCALE:
            CMR_LOGI("start scaler");
            break;
        case SNAPSHOT_EVT_SCALE_DONE:
            CMR_LOGI("scaler done");
            break;
        case SNAPSHOT_EVT_START_ENC:
            CMR_LOGI("start jpeg enc");
            break;
        case SNAPSHOT_EVT_ENC_DONE:
            CMR_LOGI("close hdr before jpeg enc done");
            setting_param.camera_id = cxt->camera_id;
            ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_APPMODE,
                            &setting_param);
            if (1 == camera_get_hdr_flag(cxt)) {
                ret = camera_close_hdr(cxt);
            } else if (1 == camera_get_3dnr_flag(cxt) &&
                setting_param.cmd_type_value == CAMERA_MODE_3DNR_PHOTO) {
                ret = camera_close_3dnr(cxt);
            } else if ((1 == camera_get_3dnr_flag(cxt) &&
                setting_param.cmd_type_value == CAMERA_MODE_NIGHT_PHOTO)
                 && cxt->night_cxt.is_authorized) {
                ret = cxt->night_cxt.sw_close(cxt);
            }
            CMR_LOGI("jpeg enc done");
            break;
        case SNAPSHOT_EVT_START_CONVERT_THUMB:
            CMR_LOGI("start to convert thumb");
            break;
        case SNAPSHOT_EVT_CONVERT_THUMB_DONE:
            CMR_LOGI("convert thumb done");
            break;
        case SNAPSHOT_EVT_ENC_THUMB:
            CMR_LOGI("start to enc thumb");
            break;
        case SNAPSHOT_EVT_ENC_THUMB_DONE:
            CMR_LOGI("thumb enc done");
            break;
        case SNAPSHOT_EVT_START_DEC:
            CMR_LOGI("start to jpeg dec");
            break;
        case SNAPSHOT_EVT_DEC_DONE:
            CMR_LOGI("jpeg dec done");
            break;
        case SNAPSHOT_EVT_START_EXIF_JPEG:
            CMR_LOGI("start to compound jpeg with exif");
            break;
        case SNAPSHOT_EVT_EXIF_JPEG_DONE:
            CMR_LOGI("jpeg done with exif");
            // camera_snapshot_channel_handle(cxt);
            break;
        case SNAPSHOT_EVT_STATE:
            cxt->snp_cxt.status = ((struct camera_frame_type *)param)->status;
            CMR_LOGI("snapshot state is %d", cxt->snp_cxt.status);
            break;
        default:
            CMR_LOGE("don't support cb %d", cb);
            break;
        }
    }
    if (SNAPSHOT_FUNC_RECOVERY == func) {
        // to do
    }
}

void camera_post_share_path_available(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGI("post beging");
    camera_set_share_path_sm_flag(oem_handle, 0);
    sem_post(&cxt->share_path_sm);
    CMR_LOGI("post end");
}

void camera_set_share_path_sm_flag(cmr_handle oem_handle, cmr_uint flag) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGI("%ld", flag);
    sem_wait(&cxt->access_sm);
    cxt->share_path_sm_flag = flag;
    sem_post(&cxt->access_sm);
}

cmr_uint camera_get_share_path_sm_flag(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_uint flag = 0;

    sem_wait(&cxt->access_sm);
    flag = cxt->share_path_sm_flag;
    sem_post(&cxt->access_sm);
    CMR_LOGI("%ld", flag);
    return flag;
}

void camera_wait_share_path_available(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct cmr_path_capability capability;
    cmr_int need_pause;
    cmr_uint video_snapshot_type;
    struct setting_cmd_parameter setting_param;

    CMR_LOGI("wait beging");

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    cmr_setting_ioctl(setting_cxt->setting_handle,
                      SETTING_GET_VIDEO_SNAPSHOT_TYPE, &setting_param);
    video_snapshot_type = setting_param.cmd_type_value;
    camera_local_zsl_snapshot_need_pause(oem_handle, &need_pause);
    camera_channel_path_capability(oem_handle, &capability);
    if (CAMERA_ZSL_MODE == cxt->snp_cxt.snp_mode && !need_pause) {
        CMR_LOGI("no need wait");
    } else {
        if (video_snapshot_type != VIDEO_SNAPSHOT_VIDEO) {
            camera_set_share_path_sm_flag(oem_handle, 1);
            sem_wait(&cxt->share_path_sm);
        }
    }
    CMR_LOGI("wait end");
}

void camera_snapshot_cb(cmr_handle oem_handle, enum snapshot_cb_type cb,
                        enum snapshot_func_type func, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_uint oem_func;
    cmr_uint oem_cb_type;
    struct camera_jpeg_param enc_param;

    if (!oem_handle) {
        CMR_LOGE("error handle");
        return;
    }
    CMR_LOGI("func %d", func);
    if (SNAPSHOT_FUNC_TAKE_PICTURE == func ||
        SNAPSHOT_FUNC_ENCODE_PICTURE == func) {
        if ((SNAPSHOT_FUNC_ENCODE_PICTURE == func) &&
            (SNAPSHOT_EXIT_CB_DONE == cb) && param) {
            enc_param = ((struct camera_frame_type *)param)->jpeg_param;
            if (enc_param.need_free) {
                camera_set_snp_req(oem_handle, TAKE_PICTURE_NO);
                camera_set_discard_frame(cxt, 0);
            }
        }
        camera_snapshot_cb_to_hal(oem_handle, cb, func, param);
        if ((SNAPSHOT_FUNC_ENCODE_PICTURE == func) &&
            (SNAPSHOT_EXIT_CB_DONE == cb)) {
            camera_snapshot_channel_handle(oem_handle, param);
        }

        if (cxt->lls_shot_mode || cxt->is_vendor_hdr || cxt->is_pipviv_mode) {
            if ((SNAPSHOT_FUNC_TAKE_PICTURE == func) &&
                (SNAPSHOT_EVT_CB_SNAPSHOT_JPEG_DONE == cb) && param) {
                struct camera_frame_type *frame_type =
                    (struct camera_frame_type *)param;

                if (frame_type->need_free) {
                    camera_set_snp_req(oem_handle, TAKE_PICTURE_NO);
                    camera_set_discard_frame(cxt, 0);
                }
                camera_snapshot_channel_handle(oem_handle, param);
            }
        }

        if ((SNAPSHOT_FUNC_TAKE_PICTURE == func) &&
            (SNAPSHOT_RSP_CB_SUCCESS == cb)) {
            camera_wait_share_path_available(oem_handle);
        }
    } else if (SNAPSHOT_FUNC_RELEASE_PICTURE == func) {
        if (cxt->camera_cb) {
            cxt->camera_cb(CAMERA_RSP_CB_SUCCESS, cxt->client_data,
                           CAMERA_FUNC_RELEASE_PICTURE, NULL);
        }
    } else {
        camera_snapshot_state_handle(oem_handle, cb, func, param);
    }
}

cmr_int camera_before_set(cmr_handle oem_handle, enum preview_param_mode mode) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle) {
        CMR_LOGE("error handle");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_preview_before_set_param(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, mode);
exit:
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_after_set(cmr_handle oem_handle,
                         struct after_set_cb_param *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_u32 skip_num = 0;

    if (!oem_handle) {
        CMR_LOGE("error handle");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (PREVIEWING == cmr_preview_get_status(cxt->prev_cxt.preview_handle,
                                             cxt->camera_id) &&
        (IMG_DATA_TYPE_RAW == cxt->sn_cxt.sensor_info.image_format)) {
        skip_num = 0;
    } else {
        skip_num = param->skip_number;
    }
    CMR_LOGD("sensor fmt %d, skip num %d", cxt->sn_cxt.sensor_info.image_format,
             skip_num);

    ret = cmr_preview_after_set_param(cxt->prev_cxt.preview_handle,
                                      cxt->camera_id, param->re_mode,
                                      param->skip_mode, skip_num);

exit:
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_focus_visit_flash_info(cmr_handle oem_handle,
                                      cmr_uint camera_id) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;

    setting_param.camera_id = camera_id;
    setting_param.cmd_type_value = 2;
    cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                      SETTING_GET_HW_FLASH_STATUS, &setting_param);

    return setting_param.cmd_type_value; // 1 stands for pre-flash turned on
}

cmr_int camera_focus_pre_proc(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    cmr_bzero(&setting_param, sizeof(struct setting_cmd_parameter));

    CMR_LOGD("E");

    if (cxt->camera_id == 1)
        goto exit;

    setting_param.camera_id = cxt->camera_id;
    setting_param.ctrl_flash.is_active = 1;
    setting_param.ctrl_flash.flash_type = FLASH_OPEN;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, SETTING_CTRL_FLASH,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to open flash");
    }

exit:
    CMR_LOGD("X");
    return ret;
}

cmr_int camera_focus_post_proc(cmr_handle oem_handle, cmr_int will_capture) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    cmr_int need_close_flash = 1;
    struct sensor_raw_info *raw_info_ptr = NULL;
    struct sensor_libuse_info *libuse_info = NULL;
    cmr_int product_id = 0;
    struct common_isp_cmd_param isp_param;
    cmr_uint video_snapshot_type;
    cmr_uint has_preflashed = 0;
    cmr_u32 flash_capture_skip_num = 0;
    struct sensor_exp_info exp_info_ptr;
    cmr_u32 offset = 0;

    CMR_LOGD("E");

    if (cxt->camera_id == 1)
        goto exit;

    cmr_bzero(&setting_param, sizeof(struct setting_cmd_parameter));
    cmr_bzero(&exp_info_ptr, sizeof(struct sensor_exp_info));

    /*close flash*/
    CMR_LOGI("will_capture %ld", will_capture);

    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_VIDEO_SNAPSHOT_TYPE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get video snapshot enabled flag %ld", ret);
    } else {
        video_snapshot_type = setting_param.cmd_type_value;
        if (video_snapshot_type == VIDEO_SNAPSHOT_VIDEO)
            need_close_flash = 0;
    }

    ret = camera_get_sensor_info(oem_handle, cxt->camera_id, &exp_info_ptr);
    if (ret) {
        CMR_LOGE("camera_get_sensor_info failed");
    }

    if (cxt->snp_cxt.snp_mode == CAMERA_ZSL_MODE ||
        cxt->snp_cxt.snp_mode == CAMERA_NORMAL_MODE) {
        offset = FLASH_CAPTURE_SKIP_NUM_OFFSET;
    }

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_PRE_LOWFLASH_VALUE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd zsl enabled flag %ld", ret);
        // goto exit;
    }
    has_preflashed = setting_param.cmd_type_value;
    CMR_LOGD("has_preflashed=%ld", has_preflashed);

    /*for third ae*/
    raw_info_ptr = cxt->sn_cxt.sensor_info.raw_info_ptr;
    if (raw_info_ptr) {
        libuse_info = raw_info_ptr->libuse_info;
        if (libuse_info) {
            product_id = libuse_info->ae_lib_info.product_id;
        }
    }
    if (product_id) {
        if (need_close_flash) {
            setting_param.ctrl_flash.is_active = 1;
            setting_param.ctrl_flash.work_mode = 0;
            setting_param.ctrl_flash.capture_mode.capture_mode = 0;
            setting_param.ctrl_flash.flash_type = FLASH_AF_DONE;
            setting_param.ctrl_flash.will_capture = will_capture;
            prev_set_preview_skip_frame_num(
                cxt->prev_cxt.preview_handle, cxt->camera_id,
                flash_capture_skip_num, has_preflashed);
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                    SETTING_CTRL_FLASH, &setting_param);
            if (ret) {
                CMR_LOGE("failed to open flash %ld", ret);
            }

            setting_param.ctrl_flash.is_active = 0;
            setting_param.ctrl_flash.work_mode = 0;
            setting_param.ctrl_flash.capture_mode.capture_mode = 0;
            setting_param.ctrl_flash.flash_type = FLASH_WAIT_TO_CLOSE;
            setting_param.ctrl_flash.will_capture = will_capture;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                    SETTING_CTRL_FLASH, &setting_param);
            if (ret) {
                CMR_LOGE("failed to open flash %ld", ret);
            }
        }
    } else {
        if (need_close_flash) {
            setting_param.ctrl_flash.is_active = 0;
            setting_param.ctrl_flash.work_mode = 0;
            setting_param.ctrl_flash.capture_mode.capture_mode = 0;
            setting_param.ctrl_flash.flash_type = FLASH_CLOSE_AFTER_OPEN;
            setting_param.ctrl_flash.will_capture = will_capture;
            ret = isp_ioctl(cxt->isp_cxt.isp_handle,
                            ISP_CTRL_GET_FLASH_SKIP_FRAME_NUM,
                            &flash_capture_skip_num);
            if (ret) {
                CMR_LOGE("failed to get preflash skip number %ld", ret);
            }
            CMR_LOGD("flash_capture_skip_num = %d", flash_capture_skip_num);
            prev_set_preview_skip_frame_num(
                cxt->prev_cxt.preview_handle, cxt->camera_id,
                flash_capture_skip_num, has_preflashed);
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                    SETTING_CTRL_FLASH, &setting_param);
            if (ret) {
                CMR_LOGE("failed to open flash %ld", ret);
            }
            if (!will_capture) {
                ret = camera_isp_ioctl(
                    oem_handle, COM_ISP_SET_SNAPSHOT_FINISHED, &isp_param);
                if (ret)
                    CMR_LOGE("failed to set snapshot finished %ld", ret);
            }
        }
    }

exit:
    CMR_LOGD("X");
    return ret;
}

cmr_int camera_get_preview_status(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt = &cxt->prev_cxt;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;
    struct setting_context *setting_cxt = &cxt->setting_cxt;

    if (1 != prev_cxt->inited) {
        CMR_LOGE("err, don't init preview");
        return ERROR;
    }

    return cmr_preview_get_status(cxt->prev_cxt.preview_handle, cxt->camera_id);
}

cmr_int camera_sensor_init(cmr_handle oem_handle, cmr_uint is_autotest) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_context *sn_cxt = NULL;
    struct sensor_init_param init_param;
    cmr_handle sensor_handle;
    cmr_u32 camera_id_bits = 0;
    struct sensor_ex_info *sns_ex_info_ptr = NULL;

    CHECK_HANDLE_VALID(oem_handle);
    sn_cxt = &cxt->sn_cxt;
    CHECK_HANDLE_VALID(sn_cxt);

    if (1 == sn_cxt->inited) {
        CMR_LOGD("sensor has been intialized");
        goto exit;
    }
    cmr_bzero(&init_param, sizeof(init_param));
    init_param.oem_handle = oem_handle;
    init_param.is_autotest = is_autotest;

    ret = cmr_sensor_init(&init_param, &sensor_handle);
    if (ret) {
        CMR_LOGE("failed to init sensor %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    sn_cxt->sensor_handle = sensor_handle;
    sn_cxt->inited = 1;
    camera_id_bits = 1 << cxt->camera_id;
    ret = cmr_sensor_open(sensor_handle, camera_id_bits);
    if (ret) {
        CMR_LOGE("open %d sensor failed %ld", cxt->camera_id, ret);
        goto sensor_exit;
    }
#ifdef CONFIG_CAMERA_RT_REFOCUS
    if (cxt->camera_id == SENSOR_MAIN) {
        camera_id_bits = 1 << SENSOR_MAIN2;
        ret = cmr_sensor_open(sensor_handle, camera_id_bits);
        if (ret) {
            CMR_LOGE("open %d sensor failed %ld", cxt->camera_id, ret);
            goto sensor_exit;
        }
    }
#endif
    cmr_sensor_event_reg(sensor_handle, cxt->camera_id, camera_sensor_evt_cb);
    cxt->sn_cxt.sensor_handle = sensor_handle;

    sns_ex_info_ptr = &sn_cxt->cur_sns_ex_info;
    ret = cmr_sensor_init_static_info(cxt);
    if (ret) {
        CMR_LOGE("init static info of  %d sensor failed %ld", cxt->camera_id,
                 ret);
        goto sensor_exit;
    }
    goto exit;

sensor_exit:
    cmr_sensor_deinit_static_info(cxt);
    cmr_sensor_deinit(sn_cxt->sensor_handle);
    sn_cxt->inited = 0;

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_sensor_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    cmr_handle sensor_handle;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_context *sn_cxt;

    CHECK_HANDLE_VALID(oem_handle);

    sn_cxt = &cxt->sn_cxt;
    if (0 == sn_cxt->inited) {
        CMR_LOGI("sensor has been de-intialized");
        goto exit;
    }
    sensor_handle = sn_cxt->sensor_handle;
    cmr_sensor_close(sensor_handle, (1 << cxt->camera_id));
#ifdef CONFIG_CAMERA_RT_REFOCUS
    if (cxt->camera_id == SENSOR_MAIN) {
        ret = cmr_sensor_close(sensor_handle, 1 << SENSOR_MAIN2);
        if (ret) {
            CMR_LOGE("close %d sensor failed %ld", cxt->camera_id, ret);
            goto exit;
        }
    }
#endif
    cmr_sensor_deinit_static_info(cxt);
    cmr_sensor_deinit(sensor_handle);
    cmr_bzero(sn_cxt, sizeof(*sn_cxt));

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_grab_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);
    struct phySensorInfo *phyPtr = NULL;
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct grab_context *grab_cxt = NULL;
    struct sensor_context *sn_cxt = NULL;
    cmr_handle grab_handle = NULL;
    struct grab_init_param grab_param;
    struct sensor_exp_info sensor_info;
    cmr_bzero(&sensor_info, sizeof(struct sensor_exp_info));
    CHECK_HANDLE_VALID(oem_handle);
    grab_cxt = &cxt->grab_cxt;
    CHECK_HANDLE_VALID(grab_cxt);
    sn_cxt = &(cxt->sn_cxt);
    CHECK_HANDLE_VALID(sn_cxt);

    ret = cmr_sensor_get_info(sn_cxt->sensor_handle, cxt->camera_id,
                              &(sn_cxt->sensor_info));
    if (ret) {
        CMR_LOGE("fail to get sensor info ret %ld", ret);
        goto exit;
    }
    phyPtr = sensorGetPhysicalSnsInfo(cxt->camera_id);

    if (0 == grab_cxt->inited) {
        grab_param.oem_handle = oem_handle;
        grab_param.sensor_id = phyPtr->slotId;
        grab_param.sensor_max_size.width = sn_cxt->sensor_info.source_width_max;
        grab_param.sensor_max_size.height =
            sn_cxt->sensor_info.source_height_max;
        ret = cmr_grab_init(&grab_param, &grab_handle);
        if (ret) {
            CMR_LOGE("failed to init grab %ld", ret);
            ret = -CMR_CAMERA_NO_SUPPORT;
            goto exit;
        }
        cmr_grab_evt_reg(grab_handle, camera_grab_evt_cb);
        cmr_grab_stream_cb(grab_handle, camera_sensor_streamctrl);
        /*only raw sensor should init isp*/
        if (IMG_DATA_TYPE_RAW == sn_cxt->sensor_info.image_format) {
            cmr_grab_isp_statis_evt_reg(grab_handle, isp_statis_evt_cb);
            cmr_grab_isp_irq_proc_evt_reg(grab_handle, isp_irq_proc_evt_cb);
        }
        cmr_grab_post_ynr_evt_reg(grab_handle, camera_grab_post_ynr_evt_cb);
        cmr_grab_3dnr_evt_reg(grab_handle, camera_grab_3dnr_evt_cb);
        grab_cxt->inited = 1;
        grab_cxt->grab_handle = grab_handle;
    }

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_grab_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct grab_context *grab_cxt = NULL;

    CHECK_HANDLE_VALID(oem_handle);
    grab_cxt = &cxt->grab_cxt;
    if (0 == grab_cxt->inited) {
        CMR_LOGD("GRAB has been de-intialized");
        goto exit;
    }

    ret = cmr_grab_deinit(grab_cxt->grab_handle);
    if (ret) {
        CMR_LOGE("failed to de-init grab %ld", ret);
        goto exit;
    }
    cmr_bzero(grab_cxt, sizeof(*grab_cxt));
exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_s32 camera_local_get_iommu_status(cmr_handle oem_handle) {
    cmr_s32 ret;
    struct grab_context *grab_cxt = NULL;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    grab_cxt = &cxt->grab_cxt;

    ret = cmr_grab_get_iommu_status(grab_cxt->grab_handle);

    return ret;
}

cmr_int camera_jpeg_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_context *jpeg_cxt = NULL;

    CHECK_HANDLE_VALID(oem_handle);
    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);

    if (0 == jpeg_cxt->inited) {
        ret = cmr_jpeg_init(oem_handle, &jpeg_cxt->jpeg_handle,
                            camera_jpeg_evt_cb);
        if (CMR_CAMERA_SUCCESS == ret) {
            jpeg_cxt->inited = 1;
        } else {
            CMR_LOGE("failed to init jpeg codec %ld", ret);
            ret = -CMR_CAMERA_NO_SUPPORT;
        }
    }
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_jpeg_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_context *jpeg_cxt = NULL;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);

    if (0 == jpeg_cxt->inited) {
        CMR_LOGD("jpeg codec has been de-intialized");
        goto exit;
    }

    ret = cmr_jpeg_deinit(jpeg_cxt->jpeg_handle);
    if (ret) {
        CMR_LOGE("failed to de-init jpeg codec %ld", ret);
        goto exit;
    }
    cmr_bzero(jpeg_cxt, sizeof(*jpeg_cxt));

    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_scaler_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct scaler_context *scaler_cxt = NULL;
    cmr_handle scale_handle;

    CHECK_HANDLE_VALID(oem_handle);
    scaler_cxt = &cxt->scaler_cxt;
    CHECK_HANDLE_VALID(scaler_cxt);

    if (1 == scaler_cxt->inited) {
        CMR_LOGD("scaler has been intialized");
        goto exit;
    }

    scale_handle = cxt->grab_cxt.grab_handle;
    ret = cmr_scale_open(&scale_handle);
    if (ret) {
        CMR_LOGE("failed to init scaler %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
    } else {
        scaler_cxt->scaler_handle = scale_handle;
        scaler_cxt->inited = 1;
    }

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_scaler_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct scaler_context *scaler_cxt;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    scaler_cxt = &cxt->scaler_cxt;
    CHECK_HANDLE_VALID(scaler_cxt);
    if (0 == scaler_cxt->inited) {
        CMR_LOGD("scaler has been de-intialized");
        goto exit;
    }

    ret = cmr_scale_close(scaler_cxt->scaler_handle);
    if (ret) {
        CMR_LOGE("failed to de-init scaler %ld", ret);
        goto exit;
    }
    cmr_bzero(scaler_cxt, sizeof(*scaler_cxt));

    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_rotation_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct rotation_context *rot_cxt = NULL;
    cmr_handle rot_handle;

    CHECK_HANDLE_VALID(oem_handle);
    rot_cxt = &cxt->rot_cxt;
    CHECK_HANDLE_VALID(rot_cxt);

    if (1 == rot_cxt->inited) {
        CMR_LOGD("rot has been intialized");
        goto exit;
    }

    rot_handle = cxt->grab_cxt.grab_handle;
    ret = cmr_rot_open(&rot_handle);
    if (ret) {
        CMR_LOGE("failed to init rot %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
    } else {
        rot_cxt->rotation_handle = rot_handle;
        rot_cxt->inited = 1;
    }

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_rotation_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct rotation_context *rot_cxt;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    rot_cxt = &cxt->rot_cxt;
    CHECK_HANDLE_VALID(rot_cxt);

    if (0 == rot_cxt->inited) {
        CMR_LOGD("rot has been de-intialized");
        goto exit;
    }

    ret = cmr_rot_close(rot_cxt->rotation_handle);
    if (ret) {
        CMR_LOGE("failed to de-init rot %ld", ret);
        goto exit;
    }
    cmr_bzero(rot_cxt, sizeof(*rot_cxt));

    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

void camera_calibrationconfigure_save(uint32_t start_addr, uint32_t data_size) {
    const char configfile[] = "/data/otpconfig.bin";

    FILE *configfile_handle = fopen(configfile, "wb");
    if (NULL == configfile_handle) {
        CMR_LOGE("failed");
        return;
    }
    fwrite(&start_addr, 1, 4, configfile_handle);
    fwrite(&data_size, 1, 4, configfile_handle);
    fclose(configfile_handle);
    CMR_LOGI("done");
}

void camera_calibrationconfigure_load(uint32_t *start_addr,
                                      uint32_t *data_size) {
    const char configfile[] = "/data/otpconfig.bin";

    FILE *configfile_handle = fopen(configfile, "rb");
    if (NULL == configfile_handle) {
        CMR_LOGE("failed");
        return;
    }
    fread(&start_addr, 1, 4, configfile_handle);
    fread(&data_size, 1, 4, configfile_handle);
    fclose(configfile_handle);
    CMR_LOGI("done");
}

cmr_int camera_get_otpinfo(cmr_handle oem_handle,
                           struct sensor_otp_cust_info *otp_data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    SENSOR_VAL_T val;

    if (NULL == oem_handle || NULL == otp_data) {
        ret = -CMR_CAMERA_INVALID_PARAM;
        CMR_LOGE("in parm error");
        goto exit;
    }
    val.type = SENSOR_VAL_TYPE_READ_OTP;
    val.pval = NULL;
    ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                           SENSOR_ACCESS_VAL, (cmr_uint)&val);
    if (ret) {
        CMR_LOGE("get sensor dual otp info failed %ld", ret);
        goto exit;
    }
    if (val.pval) {
        memcpy(otp_data, val.pval, sizeof(struct sensor_otp_cust_info));
        CMR_LOGI("%p, %p, size:%d", otp_data, otp_data->total_otp.data_ptr,
                 otp_data->total_otp.size);
    } else {
        ret = -1;
        CMR_LOGI("%ld, no dual otp data", ret);
        goto exit;
    }
exit:
    return ret;
}

int32_t camera_isp_flash_get_charge(void *handler,
                                    struct isp_flash_cfg *cfg_ptr,
                                    struct isp_flash_cell *cell) {
    int32_t ret = 0;
    struct camera_context *cxt = (struct camera_context *)handler;

    return ret;
}

int32_t camera_isp_flash_get_time(void *handler, struct isp_flash_cfg *cfg_ptr,
                                  struct isp_flash_cell *cell) {
    int32_t ret = 0;
    struct camera_context *cxt = (struct camera_context *)handler;

    return ret;
}

int32_t camera_isp_flash_set_charge(void *handler,
                                    struct isp_flash_cfg *cfg_ptr,
                                    struct isp_flash_element *element) {
    cmr_s32 ret = 0;
    struct camera_context *cxt = (struct camera_context *)handler;

    struct sprd_flash_cfg_param cfg;
    cmr_u8 real_type = 0;
    struct sprd_flash_cell real_cell;

    if (!cxt || !element) {
        CMR_LOGE("err param, %p %p", cxt, element);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto out;
    }

    switch (cfg_ptr->type) {
    case ISP_FLASH_TYPE_PREFLASH:
        real_type = FLASH_TYPE_PREFLASH;
        break;
    case ISP_FLASH_TYPE_MAIN:
        real_type = FLASH_TYPE_MAIN;
        break;
    default:
        CMR_LOGE("not support the type");
        goto out;
        break;
    }
    cmr_bzero(&real_cell, sizeof(real_cell));
    cfg.real_cell.type = real_type;
    cfg.real_cell.count = 1;
    cfg.real_cell.led_idx = cfg_ptr->led_idx;
    cfg.real_cell.element[0].index = element->index;
    cfg.real_cell.element[0].val = element->val;
    cfg.io_id = FLASH_IOID_SET_CHARGE;
    cfg.flash_idx = cxt->facing % 2;
    cfg.real_cell.element[0].brightness = element->brightness;
    cfg.real_cell.element[0].color_temp = element->color_temp;
    cfg.real_cell.element[0].bg_color = element->bg_color;
    CMR_LOGD("led_idx=%d, flash_type=%d, idx=%d", cfg_ptr->led_idx, real_type,
             element->index);
    if (camera_front_lcd_flash_activie(cfg.flash_idx))
        ret = camera_front_lcd_flash_cfg(cxt, &cfg);
    else
        ret = cmr_grab_cfg_flash(cxt->grab_cxt.grab_handle, &cfg);
out:
    return ret;
}

int32_t camera_isp_flash_ctrl(void *handler, struct isp_flash_cfg *cfg_ptr,
                              struct isp_flash_element *element) {

    int32_t ret = 0;
    struct camera_context *cxt = (struct camera_context *)handler;
    cmr_u8 real_type;
    struct grab_flash_opt flash_opt;
    bzero(&flash_opt, sizeof(struct grab_flash_opt));
    if (!cxt || !cfg_ptr) {
        CMR_LOGE("err param, %p %p", cxt, cfg_ptr);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto out;
    }
    CMR_LOGD("led0_enable=%d,led1_enable=%d, led_type=%d", cfg_ptr->led0_enable,
             cfg_ptr->led1_enable, cfg_ptr->type);
    switch (cfg_ptr->type) {
    case ISP_FLASH_TYPE_PREFLASH:
        if (cfg_ptr->led0_enable || cfg_ptr->led1_enable) {
            real_type = FLASH_OPEN;
        } else {
            real_type = FLASH_CLOSE_AFTER_OPEN;
            cfg_ptr->led0_enable = 1;
            cfg_ptr->led1_enable = 1;
        }
        break;
    case ISP_FLASH_TYPE_MAIN:
        if (cfg_ptr->led0_enable || cfg_ptr->led1_enable) {
            real_type = FLASH_HIGH_LIGHT;
        } else {
            real_type = FLASH_CLOSE_AFTER_OPEN;
            cfg_ptr->led0_enable = 1;
            cfg_ptr->led1_enable = 1;
        }
        break;
    default:
        CMR_LOGE("not support the type");
        goto out;
        break;
    }

    flash_opt.led0_enable = cfg_ptr->led0_enable;
    flash_opt.led1_enable = cfg_ptr->led1_enable;
    flash_opt.flash_mode = real_type;
    flash_opt.flash_index = cxt->facing % 2;
    if (camera_front_lcd_flash_activie(flash_opt.flash_index))
        ret = camera_front_lcd_flash_callback(cxt, flash_opt.flash_mode);
    else
        ret = cmr_grab_flash_cb(cxt->grab_cxt.grab_handle, &flash_opt);
out:
    return ret;
}

int32_t camera_isp_flash_set_time(void *handler, struct isp_flash_cfg *cfg_ptr,
                                  struct isp_flash_element *element) {
    int32_t ret = 0;
    struct camera_context *cxt = (struct camera_context *)handler;

    return ret;
}

cmr_int camera_isp_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    char value[PROPERTY_VALUE_MAX];
    char ba_portrait[PROPERTY_VALUE_MAX];
    char fr_portrait[PROPERTY_VALUE_MAX];

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = NULL;
    struct sensor_context *sn_cxt = NULL;
    struct sensor_exp_info *sensor_info_ptr;
    struct sensor_ex_info *sns_ex_info_ptr = NULL;
    struct isp_init_param isp_param;
    struct isp_video_limit isp_limit;
    SENSOR_VAL_T val;
    struct sensor_pdaf_info pdaf_info;
    struct grab_context *grab_cxt = NULL;
    struct cmr_grab *p_grab = NULL;
    struct isp_cali_param cali_param;
    struct isp_data_info cali_result;

    cmr_bzero(&cali_param, sizeof(cali_param));
    cmr_bzero(&cali_result, sizeof(cali_result));
    cmr_bzero(&isp_param, sizeof(isp_param));
    cmr_bzero(&pdaf_info, sizeof(pdaf_info));

    CHECK_HANDLE_VALID(oem_handle);
    isp_cxt = &(cxt->isp_cxt);
    CHECK_HANDLE_VALID(isp_cxt);
    sn_cxt = &(cxt->sn_cxt);
    CHECK_HANDLE_VALID(sn_cxt);
    grab_cxt = &(cxt->grab_cxt);
    CHECK_HANDLE_VALID(grab_cxt);

    if (1 == isp_cxt->inited) {
        CMR_LOGD("isp has been intialized");
        goto exit;
    }

	if (!cxt->isp_pm_context_addr) {
        CMR_LOGE("isp pm mem has not been intialized");
		ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
	isp_param.isp_pm_mem = (void *)cxt->isp_pm_context_addr;

    ret = cmr_sensor_get_info(sn_cxt->sensor_handle, cxt->camera_id,
                              &(sn_cxt->sensor_info));
    if (ret) {
        CMR_LOGE("fail to get sensor info ret %ld", ret);
        goto exit;
    }
    CMR_LOGV("sensor_info.name=%s, version=%s", sn_cxt->sensor_info.name,
             sn_cxt->sensor_info.sensor_version_info);

    if (IMG_DATA_TYPE_RAW != sn_cxt->sensor_info.image_format) {
        CMR_LOGD("no need to init isp %d ", sn_cxt->sensor_info.image_format);
        goto exit;
    }

    cxt->lsc_malloc_flag = 0;

    sensor_info_ptr = &(sn_cxt->sensor_info);
    CHECK_HANDLE_VALID(sensor_info_ptr);

    isp_param.setting_param_ptr = sensor_info_ptr->raw_info_ptr;
    if (0 != sensor_info_ptr->mode_info[SENSOR_MODE_COMMON_INIT].width) {
        isp_param.size.w =
            sensor_info_ptr->mode_info[SENSOR_MODE_COMMON_INIT].width;
        isp_param.size.h =
            sensor_info_ptr->mode_info[SENSOR_MODE_COMMON_INIT].height;
    } else {
        isp_param.size.w =
            sensor_info_ptr->mode_info[SENSOR_MODE_PREVIEW_ONE].width;
        isp_param.size.h =
            sensor_info_ptr->mode_info[SENSOR_MODE_PREVIEW_ONE].height;
    }
    isp_param.sensor_max_size.h = sensor_info_ptr->source_height_max;
    isp_param.sensor_max_size.w = sensor_info_ptr->source_width_max;
    isp_param.ops.flash_get_charge = camera_isp_flash_get_charge;
    isp_param.ops.flash_set_charge = camera_isp_flash_set_charge;
    isp_param.ops.flash_ctrl = camera_isp_flash_ctrl;
    isp_param.ops.flash_get_time = camera_isp_flash_get_time;
    isp_param.ops.flash_set_time = camera_isp_flash_set_time;
    isp_param.ctrl_callback = camera_isp_evt_cb;
    isp_param.oem_handle = oem_handle;
    isp_param.camera_id = cxt->camera_id;
    isp_param.facing = cxt->facing;
    isp_param.alloc_cb = camera_malloc;
    isp_param.free_cb = camera_free;

    camera_sensor_color_to_isp_color(&isp_param.image_pattern,
                                     sensor_info_ptr->image_pattern);

    p_grab = (struct cmr_grab *)(grab_cxt->grab_handle);
    if (NULL != p_grab)
        isp_param.dcam_fd = p_grab->fd;

    sns_ex_info_ptr = &sn_cxt->cur_sns_ex_info;
    if (sns_ex_info_ptr == NULL) {
        CMR_LOGE("sns_ex_info_ptr is null,it is impossible error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if ((NULL == sns_ex_info_ptr->name) ||
        (NULL == sns_ex_info_ptr->sensor_version_info)) {
        ret = cmr_sensor_init_static_info(cxt);
        if (ret) {
            cmr_sensor_deinit_static_info(cxt);
            CMR_LOGE("get sensor static info failed %ld", ret);
            goto exit;
        }
    } else {
        camera_copy_sensor_ex_info_to_isp(&isp_param.ex_info, sns_ex_info_ptr);
    }

    if (IMG_DATA_TYPE_RAW == sn_cxt->sensor_info.image_format) {
        isp_param.ex_info.preview_skip_num = 0;
        isp_param.ex_info.capture_skip_num = 0;
    }

    if ((NULL != sns_ex_info_ptr->name) &&
        (NULL != sns_ex_info_ptr->sensor_version_info)) {
        CMR_LOGD("sensorname=%s, version=%s", isp_param.ex_info.name,
                 isp_param.ex_info.sensor_version_info);
    } else {
        CMR_LOGE("fail to get static info");
    }

    property_get("persist.vendor.cam.pdaf.off", value, "0");
    if (atoi(value)) {
        isp_param.ex_info.pdaf_supported = 0;
    }

    if (SENSOR_PDAF_TYPE3_ENABLE == isp_param.ex_info.pdaf_supported) {
        val.type = SENSOR_VAL_TYPE_GET_PDAF_INFO;
        val.pval = &pdaf_info;
        ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                               SENSOR_ACCESS_VAL, (cmr_uint)&val);
        if (ret) {
            CMR_LOGE("get sensor pdaf info failed %ld", ret);
            goto exit;
        }
        isp_param.pdaf_info = &pdaf_info;
    }

    val.type = SENSOR_VAL_TYPE_READ_OTP;
    val.pval = NULL;
    ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                           SENSOR_ACCESS_VAL, (cmr_uint)&val);
    if (ret) {
        CMR_LOGE("get sensor otp failed %ld", ret);
        goto exit;
    }
    if (val.pval) {
        isp_param.otp_data = val.pval;
    }

    if (sensor_info_ptr->raw_info_ptr &&
        sensor_info_ptr->raw_info_ptr->ioctrl_ptr &&
        sensor_info_ptr->raw_info_ptr->ioctrl_ptr->set_focus) {
        isp_param.ex_info.af_supported = 1;
    } else {
        isp_param.ex_info.af_supported = 0;
    }

    property_get("persist.vendor.cam.ba.portrait.enable", ba_portrait, "0");
    property_get("persist.vendor.cam.fr.portrait.enable", fr_portrait, "0");
    if (cxt->is_multi_mode == MODE_SBS) {
        isp_param.multi_mode = ISP_DUAL_SBS;
    } else if (cxt->is_multi_mode == MODE_BOKEH ||
               cxt->is_multi_mode == MODE_SOFY_OPTICAL_ZOOM ||
               cxt->is_multi_mode == MODE_3D_CAPTURE ||
               cxt->is_multi_mode == MODE_3D_VIDEO ||
               cxt->is_multi_mode == MODE_3D_PREVIEW ||
               cxt->is_multi_mode == MODE_TUNING) {
        isp_param.multi_mode = ISP_DUAL_NORMAL;
    } else if (cxt->is_multi_mode == MODE_BLUR &&
               (atoi(fr_portrait) == 1 || atoi(ba_portrait) == 1)) {
        isp_param.multi_mode = ISP_BLUR_PORTRAIT;
    } else {
        isp_param.multi_mode = ISP_SINGLE;
    }

    if (cxt->is_multi_mode == MODE_DUAL_FACEID_UNLOCK ||
        cxt->is_multi_mode == MODE_SINGLE_FACEID_UNLOCK) {
        isp_param.is_faceId_unlock = 1;
    }

    if (cxt->camera_id == cxt->master_id)
        isp_param.is_master = 1;

    CMR_LOGI(
        "cxt->is_multi_mode=%d, isp_param.multi_mode=%d, ex_info: f_num=%d, "
        "focal_length=%d, max_fps=%d, max_adgain=%d, ois_supported=%d, "
        "pdaf_supported=%d, exp_valid_frame_num=%d, clamp_level=%d, "
        "adgain_valid_frame_num=%d, prev_skip_num=%d, cap_skip_num=%d, w=%d, "
        "h=%d, sensor_info_ptr->image_pattern=%d, isp_param.image_pattern=%d, "
        "is_faceId_unlock=%d",
        cxt->is_multi_mode, isp_param.multi_mode, isp_param.ex_info.f_num,
        isp_param.ex_info.focal_length, isp_param.ex_info.max_fps,
        isp_param.ex_info.max_adgain, isp_param.ex_info.ois_supported,
        isp_param.ex_info.pdaf_supported, isp_param.ex_info.exp_valid_frame_num,
        isp_param.ex_info.clamp_level, isp_param.ex_info.adgain_valid_frame_num,
        isp_param.ex_info.preview_skip_num, isp_param.ex_info.capture_skip_num,
        isp_param.size.w, isp_param.size.h, sensor_info_ptr->image_pattern,
        isp_param.image_pattern, isp_param.is_faceId_unlock);

    ret = isp_init(&isp_param, &isp_cxt->isp_handle);
    if (ret) {
        CMR_LOGE("failed to init isp %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }

    ret = isp_capability(isp_cxt->isp_handle, ISP_VIDEO_SIZE, &isp_limit);
    if (ret) {
        CMR_LOGE("failed to get the limitation of isp %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        isp_deinit(isp_cxt->isp_handle);
        goto exit;
    }
    isp_cxt->width_limit = isp_limit.width;
    isp_cxt->inited = 1;
    CMR_LOGD("width_limit=%d", isp_cxt->width_limit);

exit:
    if (NULL != cali_result.data_ptr) {
        free(cali_result.data_ptr);
        cali_result.data_ptr = NULL;
    }
    sem_post(&cxt->isp_sm);
    ATRACE_END();
    return ret;
}

cmr_int camera_isp_deinit_notice(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGI("E");

    ret = cmr_setting_cancel_notice_flash(cxt->setting_cxt.setting_handle);
    ret = cmr_focus_deinit_notice(cxt->focus_cxt.focus_handle);
    ret = cmr_grab_deinit_notice(cxt->grab_cxt.grab_handle);

    CMR_LOGI("X %ld", ret);

    return ret;
}

cmr_int camera_isp_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt;
    cmr_s32 fd = 0;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    isp_cxt = &cxt->isp_cxt;
    CHECK_HANDLE_VALID(isp_cxt);

    if (0 == isp_cxt->inited) {
        CMR_LOGD("isp has been de-intialized");
        goto exit;
    }

    ret = isp_deinit(isp_cxt->isp_handle);
    if (ret) {
        CMR_LOGE("failed to de-init isp %ld", ret);
        goto exit;
    }
    cmr_bzero(isp_cxt, sizeof(*isp_cxt));

    if (cxt->lsc_malloc_flag == 1) {
        if (cxt->hal_free) {
            cxt->hal_free(CAMERA_ISP_LSC, &cxt->isp_lsc_phys_addr,
                          &cxt->isp_lsc_virt_addr, &cxt->lsc_mfd,
                          ISP_LSC_BUF_NUM, cxt->client_data);
        }
        cxt->lsc_malloc_flag = 0;
    }

    if (cxt->is_real_bokeh) {
        CMR_LOGD("REAL TIME:malloc");
        // DEPTH
        if (cxt->swisp_depth_malloc_flag == 1) {
            cmr_uint depth_size = 960 * 720 * 3 / 2;
            cmr_uint depth_num = 1;
            if (cxt->hal_free) {
                cxt->hal_free(CAMERA_PREVIEW_DEPTH, &cxt->swisp_depth_phys_addr,
                              &cxt->swisp_depth_virt_addr,
                              &cxt->swisp_depth_mfd, depth_num,
                              cxt->client_data);
                cxt->swisp_depth_malloc_flag = 0;
            }
        }

        // SW_OUT
        if (cxt->swisp_out_malloc_flag == 1) {
            cmr_uint sw_out_size = 960 * 720 * 3 / 2;
            cmr_uint sw_out_num = 1;
            if (cxt->hal_free) {
                cxt->hal_free(CAMERA_PREVIEW_SW_OUT, &cxt->swisp_out_phys_addr,
                              &cxt->swisp_out_virt_addr, &cxt->swisp_out_mfd,
                              sw_out_num, cxt->client_data);
                cxt->swisp_out_malloc_flag = 0;
            }
        }
    }
    sem_destroy(&cxt->isp_sm);
    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_preview_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt = NULL;
    struct preview_init_param init_param;

    CHECK_HANDLE_VALID(oem_handle);
    prev_cxt = &cxt->prev_cxt;
    CHECK_HANDLE_VALID(prev_cxt);

    if (1 == prev_cxt->inited) {
        CMR_LOGD("preview has been intialized");
        goto exit;
    }
    init_param.oem_handle = oem_handle;
    init_param.ipm_handle = cxt->ipm_cxt.ipm_handle;
    init_param.ops.channel_cfg = camera_channel_cfg;
    init_param.ops.channel_start = camera_channel_start;
    init_param.ops.channel_pause = camera_channel_pause;
    init_param.ops.channel_resume = camera_channel_resume;
    init_param.ops.channel_free_frame = camera_channel_free_frame;
    init_param.ops.channel_stop = camera_channel_stop;
    init_param.ops.channel_buff_cfg = camera_channel_buff_cfg;
    init_param.ops.channel_cap_cfg = camera_channel_cap_cfg;
    init_param.ops.isp_start_video = camera_isp_start_video;
    init_param.ops.isp_stop_video = camera_isp_stop_video;
    init_param.ops.start_rot = camera_start_rot;
    init_param.ops.preview_pre_proc = camera_preview_pre_proc;
    init_param.ops.preview_post_proc = camera_preview_post_proc;
    init_param.ops.get_sensor_info = camera_get_sensor_info;
    init_param.ops.get_sensor_autotest_mode = camera_get_sensor_autotest_mode;
    init_param.ops.channel_scale_capability = camera_channel_scale_capability;
    init_param.ops.channel_path_capability = camera_channel_path_capability;
    init_param.ops.channel_get_cap_time = camera_channel_get_cap_time;
    init_param.ops.capture_pre_proc = camera_capture_pre_proc;
    init_param.ops.capture_post_proc = camera_capture_post_proc;
    init_param.ops.sensor_open = camera_open_sensor;
    init_param.ops.sensor_close = camera_close_sensor;
    init_param.ops.get_isp_yimg = camera_preview_get_isp_yimg;
    init_param.ops.set_preview_yimg = camera_preview_set_yimg_to_isp;
    init_param.ops.set_preview_yuv = camera_preview_set_yuv_to_isp;
    init_param.ops.get_sensor_fps_info = camera_get_sensor_fps_info;
    init_param.ops.get_sensor_otp = camera_get_otpinfo;
    init_param.ops.isp_buff_cfg = camera_isp_buff_cfg;
    init_param.ops.hdr_set_ev = camera_hdr_set_ev;
    init_param.ops.set_3dnr_ev = camera_3dnr_set_ev;
    init_param.ops.sw_3dnr_info_cfg = camera_sw_3dnr_info_cfg;
    init_param.ops.get_tuning_info = camera_get_tuning_info;
    init_param.ops.start_scale = camera_start_scale;
    init_param.oem_cb = camera_preview_cb;
    init_param.private_data = NULL;
    init_param.sensor_bits = (1 << cxt->camera_id);
    ret = cmr_preview_init(&init_param, &prev_cxt->preview_handle);
    if (ret) {
        CMR_LOGE("failed to init preview,ret %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    prev_cxt->inited = 1;

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_preview_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt;

    CHECK_HANDLE_VALID(oem_handle);
    prev_cxt = &cxt->prev_cxt;
    CHECK_HANDLE_VALID(prev_cxt);

    if (0 == prev_cxt->inited) {
        CMR_LOGD("preview has been de-intialized");
        goto exit;
    }

    ret = cmr_preview_deinit(prev_cxt->preview_handle);
    if (ret) {
        CMR_LOGE("failed to de-init preview %ld", ret);
        goto exit;
    }
    cmr_bzero(prev_cxt, sizeof(*prev_cxt));

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_snapshot_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt = NULL;
    struct snapshot_init_param init_param;

    CHECK_HANDLE_VALID(oem_handle);
    snp_cxt = &cxt->snp_cxt;
    CHECK_HANDLE_VALID(snp_cxt);

    if (1 == snp_cxt->inited) {
        CMR_LOGD("snp has been intialized");
        goto exit;
    }
    init_param.id = cxt->camera_id;
    init_param.oem_handle = oem_handle;
    init_param.ipm_handle = cxt->ipm_cxt.ipm_handle;
    init_param.oem_cb = camera_snapshot_cb;
    init_param.ops.start_encode = camera_start_encode;
    init_param.ops.start_decode = camera_start_decode;
    init_param.ops.start_exif_encode = camera_start_exif_encode;
    init_param.ops.start_scale = camera_start_scale;
    init_param.ops.start_rot = camera_start_rot;
    init_param.ops.capture_pre_proc = camera_capture_pre_proc;
    init_param.ops.capture_post_proc = camera_capture_post_proc;
    init_param.ops.raw_proc = camera_raw_proc;
    init_param.ops.isp_start_video = camera_isp_start_video;
    init_param.ops.isp_stop_video = camera_isp_stop_video;
    init_param.ops.channel_start = camera_channel_start;
    init_param.ops.channel_pause = camera_channel_pause;
    init_param.ops.channel_resume = camera_channel_resume;
    init_param.ops.channel_free_frame = camera_channel_free_frame;
    init_param.ops.channel_stop = camera_channel_stop;
    init_param.ops.channel_buff_cfg = camera_channel_buff_cfg;
    init_param.ops.channel_cap_cfg = camera_channel_cap_cfg;
    init_param.ops.get_sensor_info = camera_get_sensor_info;
    init_param.ops.get_tuning_info = camera_get_tuning_info;
    init_param.ops.get_jpeg_param_info = camera_get_jpeg_param_info;
    init_param.ops.stop_codec = camera_stop_codec;
    init_param.ops.ipm_process = camera_ipm_process;
    init_param.private_data = NULL;
    ret = cmr_snapshot_init(&init_param, &snp_cxt->snapshot_handle);
    if (ret) {
        CMR_LOGE("failed to init snapshot,ret %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    snp_cxt->inited = 1;
exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_snapshot_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt;

    CMR_LOGD("E");

    CHECK_HANDLE_VALID(oem_handle);
    snp_cxt = &cxt->snp_cxt;
    CHECK_HANDLE_VALID(snp_cxt);

    if (0 == snp_cxt->inited) {
        CMR_LOGD("snp has been de-intialized");
        goto exit;
    }

    if (cxt->ipm_cxt.filter_inited) {
        ret = camera_close_filter(cxt);
        cxt->ipm_cxt.filter_inited = 0;
    }

    ret = cmr_snapshot_deinit(snp_cxt->snapshot_handle);
    if (ret) {
        CMR_LOGE("failed to de-init snapshot %ld", ret);
        goto exit;
    }
    cmr_bzero(snp_cxt, sizeof(*snp_cxt));

    CMR_LOGD("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_ipm_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct ipm_context *ipm_cxt = NULL;
    struct ipm_init_in init_param;

    CHECK_HANDLE_VALID(oem_handle);
    ipm_cxt = &cxt->ipm_cxt;
    CHECK_HANDLE_VALID(ipm_cxt);

    if (1 == ipm_cxt->inited) {
        CMR_LOGD("ipm has been intialized");
        goto exit;
    }

    init_param.oem_handle = oem_handle;
    init_param.sensor_id = cxt->camera_id;
    init_param.get_sensor_info = camera_get_sensor_info;
    init_param.ipm_sensor_ioctl = camera_sensor_ioctl;
    init_param.ipm_isp_ioctl = camera_isp_ioctl;
    ret = cmr_ipm_init(&init_param, &ipm_cxt->ipm_handle);
    if (ret) {
        CMR_LOGE("failed to init ipm,ret %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    ipm_cxt->inited = 1;

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_ipm_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct ipm_context *ipm_cxt;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    ipm_cxt = &cxt->ipm_cxt;
    CHECK_HANDLE_VALID(ipm_cxt);

    if (cxt->ipm_cxt.cnr_inited) {
        ret = camera_close_cnr(cxt);
        if (ret) {
            CMR_LOGE("failed to close cnr");
        }
        cxt->ipm_cxt.cnr_inited = 0;
    }

    if (0 == ipm_cxt->inited) {
        CMR_LOGD("ipm has been de-intialized");
        goto exit;
    }

    ret = cmr_ipm_deinit(ipm_cxt->ipm_handle);
    if (ret) {
        CMR_LOGE("failed to de-init ipm %ld", ret);
        goto exit;
    }
    cmr_bzero(ipm_cxt, sizeof(*ipm_cxt));

    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_ipm_open_module(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct ipm_open_in in_param;
    struct ipm_open_out out_param;
    struct setting_cmd_parameter setting_param;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    cmr_int ret = CMR_CAMERA_SUCCESS;

    cmr_bzero(&in_param, sizeof(struct ipm_open_in));
    cmr_bzero(&out_param, sizeof(struct ipm_open_out));

    in_param.frame_rect.start_x = 0;
    in_param.frame_rect.start_y = 0;
    in_param.reg_cb = camera_ipm_cb;

    if (1 == camera_get_hdr_flag(cxt)) {
        in_param.frame_size.width = cxt->snp_cxt.request_size.width;
        in_param.frame_size.height = cxt->snp_cxt.request_size.height;
        in_param.frame_rect.width = in_param.frame_size.width;
        in_param.frame_rect.height = in_param.frame_size.height;
        in_param.adgain_valid_frame_num =
            cxt->sn_cxt.cur_sns_ex_info.adgain_valid_frame_num;
        in_param.is_plus = 0;
        ret = camera_open_hdr(cxt, &in_param, &out_param);
        if (ret) {
            CMR_LOGE("failed to open hdr %ld", ret);
            return ret;

        } else {
            cxt->ipm_cxt.hdr_num = out_param.total_frame_number;
            cxt->ipm_cxt.hdr_version.major = out_param.version.major;
            CMR_LOGI("get hdr num %d", cxt->ipm_cxt.hdr_num);
        }
    }

    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_APPMODE,
                            &setting_param);
    if(ret)
        CMR_LOGE("failed to get app mode");
    if(setting_param.cmd_type_value == CAMERA_MODE_NIGHT_PHOTO) {
          if(cxt->night_cxt.is_authorized) {
              ret= cxt->night_cxt.sw_open(cxt);
              if(ret)
                  CMR_LOGE("failed to open sw %ld", ret);
          }
          return ret;
    }

    if (1 == camera_get_sw_3dnr_flag(cxt) && 1 != cxt->is_3dnr_video) {
        struct isp_adgain_exp_info adgain_exp_info;
        in_param.frame_size.width = cxt->snp_cxt.request_size.width;
        in_param.frame_size.height = cxt->snp_cxt.request_size.height;
        in_param.frame_rect.width = in_param.frame_size.width;
        in_param.frame_rect.height = in_param.frame_size.height;
        in_param.reg_cb = camera_ipm_cb;
        in_param.adgain = 16;
        if (1 == camera_get_sw_3dnr_flag(cxt)) {
            ret = camera_get_tuning_info((cmr_handle)cxt, &adgain_exp_info);
            if (ret) {
                CMR_LOGE("failed to get gain %ld, and using default gain", ret);
                return ret;
            } else {
                in_param.adgain = adgain_exp_info.adgain / 128;
            }
            CMR_LOGI("SW 3DRN, Get Gain from ISP: %d", in_param.adgain);
        }

        ret = camera_open_3dnr(cxt, &in_param, &out_param);
        if (ret) {
            CMR_LOGE("failed to open 3dnr %ld", ret);
            return ret;

        } else {
            cxt->ipm_cxt.threednr_num = out_param.total_frame_number;
            CMR_LOGI("get 3dnr num %d,small size:%d %d",
                     cxt->ipm_cxt.threednr_num, out_param.small_image_width,
                     out_param.small_image_height);
        }
    }

    if (camera_get_cnr_flag(oem_handle) && !cxt->ipm_cxt.cnr_inited) {
        in_param.frame_size.width = cxt->snp_cxt.request_size.width;
        in_param.frame_size.height = cxt->snp_cxt.request_size.height;
        ret = camera_open_cnr(cxt, &in_param, NULL);
        if (ret) {
            CMR_LOGE("failed to open cnr %ld", ret);
            return ret;
        }
        cxt->ipm_cxt.cnr_inited = 1;
    }

    return ret;
}

cmr_int camera_setting_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = NULL;
    struct setting_init_in init_param;

    CHECK_HANDLE_VALID(oem_handle);
    setting_cxt = &cxt->setting_cxt;
    CHECK_HANDLE_VALID(setting_cxt);

    if (1 == setting_cxt->inited) {
        CMR_LOGD("setting has been de-intialized");
        goto exit;
    }
    init_param.oem_handle = oem_handle;
    init_param.camera_id_bits = (1 << cxt->camera_id);
    init_param.io_cmd_ioctl = camera_ioctl_for_setting;
    init_param.setting_sn_ioctl = camera_sensor_ioctl;
    init_param.setting_isp_ioctl = camera_isp_ioctl;
    init_param.get_setting_activity = camera_get_setting_activity;
    init_param.before_set_cb = camera_before_set;
    init_param.after_set_cb = camera_after_set;
    init_param.padding = 0;
    ret = cmr_setting_init(&init_param, &setting_cxt->setting_handle);
    if (ret) {
        CMR_LOGE("failed to init setting %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    setting_cxt->inited = 1;

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_setting_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    setting_cxt = &cxt->setting_cxt;
    CHECK_HANDLE_VALID(setting_cxt);

    if (0 == setting_cxt->inited) {
        CMR_LOGD("setting has been de-intialized");
        goto exit;
    }

    ret = cmr_setting_deinit(setting_cxt->setting_handle);
    if (ret) {
        CMR_LOGE("failed to de-init setting %ld", ret);
        goto exit;
    }
    cmr_bzero(setting_cxt, sizeof(*setting_cxt));

    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_focus_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct focus_context *focus_cxt;
    struct af_init_param init_param;

    CHECK_HANDLE_VALID(oem_handle);
    focus_cxt = &cxt->focus_cxt;
    CHECK_HANDLE_VALID(focus_cxt);

    if (1 == focus_cxt->inited) {
        CMR_LOGD("focus has been intialized");
        goto exit;
    }
    init_param.oem_handle = oem_handle;
    init_param.evt_cb = camera_focus_evt_cb;
    init_param.ops.af_pre_proc = camera_focus_pre_proc;
    init_param.ops.af_post_proc = camera_focus_post_proc;
    init_param.ops.af_isp_ioctrl = camera_isp_ioctl;
    init_param.ops.get_preview_status = camera_get_preview_status;
    init_param.ops.af_sensor_ioctrl = camera_sensor_ioctl;
    init_param.ops.get_sensor_info = camera_get_sensor_info;
    init_param.ops.get_flash_info = camera_focus_visit_flash_info;
    ret = cmr_focus_init(&init_param, cxt->camera_id, &focus_cxt->focus_handle);
    if (ret) {
        CMR_LOGE("failed to init focus,ret %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    focus_cxt->inited = 1;

exit:
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_focus_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct focus_context *focus_cxt;

    CMR_LOGI("E");

    CHECK_HANDLE_VALID(oem_handle);
    focus_cxt = &cxt->focus_cxt;
    CHECK_HANDLE_VALID(focus_cxt);

    if (0 == focus_cxt->inited) {
        CMR_LOGD("focus has been de-intialized");
        goto exit;
    }

    ret = cmr_focus_deinit(focus_cxt->focus_handle);
    if (ret) {
        CMR_LOGE("failed to de-init focus %ld", ret);
        goto exit;
    }
    cmr_bzero(focus_cxt, sizeof(*focus_cxt));

    CMR_LOGI("X");

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_preview_cb_thread_proc(struct cmr_msg *message, void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)data;
    camera_cb_of_type callback;

    if (!message || !data) {
        CMR_LOGE("param error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGV("msg_type 0x%x, sub msg type 0x%x client_data 0x%lx",
             message->msg_type, message->sub_msg_type,
             (cmr_uint)cxt->client_data);
    callback = cxt->camera_cb;
    /*CMR_PRINT_TIME;*/
    if (callback) {
        callback(message->sub_msg_type, cxt->client_data, message->msg_type,
                 message->data);
    } else {
        CMR_LOGE("err, camera cb is null");
        ret = -CMR_CAMERA_INVALID_PARAM;
    }
/*CMR_PRINT_TIME;*/
exit:
    CMR_LOGV("out ret %ld", ret);
    return ret;
}

cmr_int camera_snapshot_cb_thread_proc(struct cmr_msg *message, void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)data;
    camera_cb_of_type callback;

    if (!message || !data) {
        CMR_LOGE("param error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGV("msg_type 0x%x, sub msg type 0x%x", message->msg_type,
             message->sub_msg_type);

    callback = cxt->camera_cb;
    if (callback) {
        callback(message->sub_msg_type, cxt->client_data, message->msg_type,
                 message->data);
    } else {
        CMR_LOGE("err, camera cb is null");
        ret = -CMR_CAMERA_INVALID_PARAM;
    }

exit:
    return ret;
}

cmr_int camera_snapshot_secondary_thread_proc(struct cmr_msg *message,
                                              void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)data;
    camera_cb_of_type callback;

    if (!message || !data) {
        CMR_LOGE("param error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGD("msg_type 0x%x, sub msg type 0x%x", message->msg_type,
             message->sub_msg_type);
    callback = cxt->camera_cb;
    if (callback) {
        callback(message->sub_msg_type, cxt->client_data, message->msg_type,
                 message->data);
    } else {
        CMR_LOGE("err, camera cb is null");
        ret = -CMR_CAMERA_INVALID_PARAM;
    }
exit:
    return ret;
}

cmr_int camera_snapshot_send_raw_thread_proc(struct cmr_msg *message,
                                             void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)data;
    camera_cb_of_type callback;

    if (!message || !data) {
        CMR_LOGE("param error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGD("msg_type 0x%x, sub msg type 0x%x", message->msg_type,
             message->sub_msg_type);
    callback = cxt->camera_cb;
    if (callback) {
        callback(message->sub_msg_type, cxt->client_data, message->msg_type,
                 message->data);
    } else {
        CMR_LOGE("err, camera cb is null");
        ret = -CMR_CAMERA_INVALID_PARAM;
    }
exit:
    return ret;
}

cmr_int camera_create_prev_thread(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = cmr_thread_create(&cxt->prev_cb_thr_handle, PREVIEW_MSG_QUEUE_SIZE,
                            camera_preview_cb_thread_proc, (void *)oem_handle);

    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("create preview thread fail");
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    ret = cmr_thread_set_name(cxt->prev_cb_thr_handle, "preview_cb");
    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("fail to set thr name");
        ret = -CMR_MSG_SUCCESS;
        goto exit;
    }
exit:
    CMR_LOGD("done %ld", ret);
    return ret;
}

cmr_int camera_destroy_prev_thread(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (cxt->prev_cb_thr_handle) {
        ret = cmr_thread_destroy(cxt->prev_cb_thr_handle);
        if (!ret) {
            cxt->prev_cb_thr_handle = (cmr_handle)0;
        } else {
            CMR_LOGE("failed to destroy prev thr");
        }
    }
exit:
    CMR_LOGD("done %ld", ret);
    return ret;
}

cmr_int camera_create_snp_thread(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = cmr_thread_create(&cxt->snp_cb_thr_handle, SNAPSHOT_MSG_QUEUE_SIZE,
                            camera_snapshot_cb_thread_proc, (void *)oem_handle);

    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("failed to create snapshot thread %ld", ret);
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    ret = cmr_thread_set_name(cxt->snp_cb_thr_handle, "snap_cb");
    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("fail to set thr name");
        ret = -CMR_MSG_SUCCESS;
        goto exit;
    }

    ret = cmr_thread_create(
        &cxt->snp_secondary_thr_handle, SNAPSHOT_MSG_QUEUE_SIZE,
        camera_snapshot_secondary_thread_proc, (void *)oem_handle);

    if (CMR_MSG_SUCCESS != ret) {
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto destroy_cb_thr;
    }
    ret = cmr_thread_set_name(cxt->snp_secondary_thr_handle, "snap_sec");
    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("fail to set thr name");
        ret = -CMR_MSG_SUCCESS;
        goto exit;
    }

    ret = cmr_thread_create(
        &cxt->snp_send_raw_image_handle, SNAPSHOT_MSG_QUEUE_SIZE,
        camera_snapshot_send_raw_thread_proc, (void *)oem_handle);

    if (CMR_MSG_SUCCESS != ret) {
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto destroy_secondary_thr;
    } else {
        ret = cmr_thread_set_name(cxt->snp_send_raw_image_handle, "snap_raw");
        if (CMR_MSG_SUCCESS != ret) {
            CMR_LOGE("fail to set thr name");
            ret = CMR_MSG_SUCCESS;
        }
        goto exit;
    }

destroy_secondary_thr:
    cmr_thread_destroy(cxt->snp_secondary_thr_handle);
    cxt->snp_secondary_thr_handle = (cmr_handle)0;
destroy_cb_thr:
    cmr_thread_destroy(cxt->snp_cb_thr_handle);
    cxt->snp_cb_thr_handle = (cmr_handle)0;
exit:
    CMR_LOGD("done %ld", ret);
    return ret;
}

cmr_int camera_destroy_snp_thread(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (cxt->snp_cb_thr_handle) {
        ret = cmr_thread_destroy(cxt->snp_cb_thr_handle);
        if (!ret) {
            cxt->snp_cb_thr_handle = (cmr_handle)0;
        } else {
            CMR_LOGE("failed to destroy snp thr %ld", ret);
        }
    }
    if (cxt->snp_secondary_thr_handle) {
        ret = cmr_thread_destroy(cxt->snp_secondary_thr_handle);
        if (!ret) {
            cxt->snp_secondary_thr_handle = (cmr_handle)0;
        } else {
            CMR_LOGE("failed to destroy snp thr %ld", ret);
        }
    }
    if (cxt->snp_send_raw_image_handle) {
        ret = cmr_thread_destroy(cxt->snp_send_raw_image_handle);
        if (!ret) {
            cxt->snp_send_raw_image_handle = (cmr_handle)0;
        } else {
            CMR_LOGE("failed to destroy snp thr %ld", ret);
        }
    }
exit:
    CMR_LOGD("done %ld", ret);
    return ret;
}

cmr_int camera_init_thread(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    /*create preview thread*/
    ret = camera_create_prev_thread(oem_handle);
    if (ret) {
        CMR_LOGE("failed to create preview thread");
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto exit;
    }
    /*create snapshot thread*/
    ret = camera_create_snp_thread(oem_handle);
    if (ret) {
        CMR_LOGE("failed to create snapshot thread");
        ret = -CMR_CAMERA_NO_SUPPORT;
        goto prev_thr_destroy;
    } else {
        goto exit;
    }
prev_thr_destroy:
    camera_destroy_prev_thread(oem_handle);
exit:
    return ret;
}

cmr_int camera_deinit_thread(cmr_handle oem_handle) {
    CMR_LOGI("E");
    camera_destroy_prev_thread(oem_handle);
    camera_destroy_snp_thread(oem_handle);
    CMR_LOGI("X");

    return CMR_CAMERA_SUCCESS;
}

static cmr_int camera_res_init_internal(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    CMR_PRINT_TIME;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    /*for multicamera mode,when open convered sensor,only need to init setting*/
    if (CONVERED_CAMERA_INIT) {
        ret = camera_setting_init(oem_handle);
        if (ret) {
            CMR_LOGE("failed to init setting %ld", ret);
        }
        goto exit;
    }

#ifdef CONFIG_CAMERA_3DNR_CAPTURE
    ret = camera_nightpro_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init nightpro %ld", ret);
        goto exit;
    }
#endif

    ret = camera_ipm_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init ipm %ld", ret);
        goto exit;
    }

    ret = camera_setting_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init setting %ld", ret);
        goto exit;
    }

    ret = camera_focus_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init focus %ld", ret);
        goto exit;
    }

    ret = camera_jpeg_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init jpeg %ld", ret);
        goto exit;
    }

#if 0 // move it to front before isp init,because iommu flag need check through
      // grab_handle
	ret = camera_grab_init(oem_handle);
	if (ret) {
		CMR_LOGE("failed to init grab %ld", ret);
		goto exit;
	}
#endif
    ret = camera_scaler_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init scaler %ld", ret);
        goto exit;
    }

    ret = camera_rotation_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init rotation %ld", ret);
        goto exit;
    }

    ret = camera_preview_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init preview %ld", ret);
        goto exit;
    }

    ret = camera_snapshot_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init snapshot %ld", ret);
        goto exit;
    } else {
        CMR_LOGI("init mds ok");
        ret = camera_init_thread(oem_handle);
    }

exit:
    if (ret) {
        camera_res_deinit_internal(oem_handle);
    }

    return ret;
}

static cmr_int camera_res_deinit_internal(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    CMR_LOGI("E");
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    /*for multicamera mode,when close convered sensor,only need to deinit
     * setting*/
    if (CONVERED_CAMERA_INIT) {
        camera_setting_deinit(oem_handle);
        camera_deinit_thread(oem_handle);
        goto exit;
    }

    camera_snapshot_deinit(oem_handle);

    camera_preview_deinit(oem_handle);

    camera_jpeg_deinit(oem_handle);

    camera_focus_deinit(oem_handle);

    camera_setting_deinit(oem_handle);

    camera_rotation_deinit(oem_handle);

    camera_scaler_deinit(oem_handle);

    //	camera_grab_deinit(oem_handle);

    camera_ipm_deinit(oem_handle);

    if (cxt->night_cxt.is_authorized) {
        camera_nightpro_deinit(oem_handle);
    }

    camera_deinit_thread(oem_handle);

exit:

    CMR_LOGI("X");

    ATRACE_END();
    return CMR_CAMERA_SUCCESS;
}

static cmr_int camera_init_thread_proc(struct cmr_msg *message, void *p_data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    cmr_u32 msg_type = 0;
    cmr_uint evt = 0;
    cmr_u32 camera_id = CAMERA_ID_MAX;
    struct camera_context *cxt = (struct camera_context *)p_data;

    if (!message || !p_data) {
        return CMR_CAMERA_INVALID_PARAM;
    }

    msg_type = (cmr_u32)message->msg_type;

    switch (msg_type) {
    case CMR_EVT_INIT:
        cxt->err_code = camera_res_init_internal((cmr_handle)cxt);
        if (cxt->err_code) {
            camera_res_deinit_internal((cmr_handle)cxt);
        }
        CMR_LOGI("cb thread inited");
        break;

    case CMR_EVT_WAIT:
        CMR_LOGI("wait here");
        break;

    case CMR_EVT_EXIT:
        camera_res_deinit_internal((cmr_handle)cxt);
        CMR_LOGI("camera exit");
        break;

    case CMR_EVT_ISP_PM_MEM_INIT:
        pthread_mutex_lock(&cxt->isp_pm_mutex);
        cxt->isp_pm_initing = true;
        cxt->err_code = isp_mw_pm_mem_init(&cxt->isp_pm_context_addr);
        pthread_cond_signal(&cxt->isp_pm_cond);
        cxt->isp_pm_initing = false;
        pthread_mutex_unlock(&cxt->isp_pm_mutex);
        ISP_LOGI("pm_context_addr: 0x%x, ret: %d", cxt->isp_pm_context_addr, cxt->err_code);
        break;

    case CMR_EVT_ISP_INIT:
        cxt->err_code = camera_isp_init((cmr_handle)cxt);
        ISP_LOGI("CMR_EVT_ISP_INIT: 0x%x, ret: %d", cxt->isp_pm_context_addr, cxt->err_code);
        break;

    case CMR_EVT_ISP_PM_MEM_DEINIT:
        isp_mw_pm_mem_deinit((void *)cxt->isp_pm_context_addr);
        cxt->isp_pm_context_addr = 0;
        break;

    default:
        break;
    }

    return ret;
}

static cmr_int camera_create_init_thread(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);

    CMR_PRINT_TIME;

    cxt->err_code = CMR_CAMERA_SUCCESS;
    /*create thread*/
    ret = cmr_thread_create((cmr_handle *)&cxt->init_thread,
                            CAMERA_OEM_MSG_QUEUE_SIZE, camera_init_thread_proc,
                            (void *)cxt);
    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("create thread fail");
    }
    ret = cmr_thread_set_name(cxt->init_thread, "camera_init");
    if (CMR_MSG_SUCCESS != ret) {
        CMR_LOGE("fail to set thr name");
        ret = CMR_MSG_SUCCESS;
    }

    CMR_LOGI("init thread created");
    ATRACE_END();
    return ret;
}

static cmr_int camera_destroy_init_thread(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);

    CMR_LOGI("E");

    if (cxt->init_thread) {
        cmr_thread_destroy(cxt->init_thread);
        cxt->init_thread = 0;
    }

    CMR_LOGI("X");
    ATRACE_END();
    return ret;
}

cmr_int camera_res_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);

    CMR_PRINT_TIME;
#ifdef OEM_HANDLE_HDR
    sem_init(&cxt->hdr_flag_sm, 0, 1);
#endif
#ifdef OEM_HANDLE_3DNR
    sem_init(&cxt->threednr_flag_sm, 0, 1);
#endif
    sem_init(&cxt->cnr_flag_sm, 0, 1);
    sem_init(&cxt->filter_sm, 0, 1);

    sem_init(&cxt->share_path_sm, 0, 0);
    sem_init(&cxt->access_sm, 0, 1);
    sem_init(&cxt->sbs_sync_sm, 0, 0);
    sem_init(&cxt->snapshot_sm, 0, 1);

    cxt->err_code = CMR_CAMERA_SUCCESS;

    message.msg_type = CMR_EVT_INIT;
    message.sync_flag = CMR_MSG_SYNC_RECEIVED;
    ret = cmr_thread_msg_send(cxt->init_thread, &message);
    if (ret) {
        CMR_LOGE("send msg failed!");
        ret = CMR_CAMERA_FAIL;
    }
    CMR_LOGI("async msg sent");
    ATRACE_END();
    return ret;
}

static cmr_int camera_res_init_done(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);

    message.msg_type = CMR_EVT_WAIT;
    message.sync_flag = CMR_MSG_SYNC_PROCESSED;
    ret = cmr_thread_msg_send(cxt->init_thread, &message);
    if (ret) {
        CMR_LOGE("send msg failed!");
    }
    ret = cxt->err_code;
    CMR_LOGI("res init-ed");
    ATRACE_END();
    return ret;
}
static cmr_int camera_res_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);

    CMR_LOGI("E");

    message.msg_type = CMR_EVT_EXIT;
    message.sync_flag = CMR_MSG_SYNC_PROCESSED;
    ret = cmr_thread_msg_send(cxt->init_thread, &message);
    if (ret) {
        CMR_LOGE("send msg failed!");
    }

    if (cxt->camera_cb) {
        cxt->camera_cb(CAMERA_EXIT_CB_DONE, cxt->client_data, CAMERA_FUNC_STOP,
                       0);
    }

#ifdef OEM_HANDLE_HDR
    sem_destroy(&cxt->hdr_flag_sm);
#endif

#ifdef OEM_HANDLE_3DNR
    sem_destroy(&cxt->threednr_flag_sm);
#endif
    sem_destroy(&cxt->cnr_flag_sm);
    sem_destroy(&cxt->filter_sm);

    sem_destroy(&cxt->share_path_sm);
    sem_destroy(&cxt->access_sm);
    sem_destroy(&cxt->snapshot_sm);
    // for sbs mode capture
    sem_destroy(&cxt->sbs_sync_sm);

    CMR_LOGI("X");
    ATRACE_END();
    return ret;
}

cmr_int camera_isp_pm_mem_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);

    cxt->err_code = CMR_CAMERA_SUCCESS;

    message.msg_type = CMR_EVT_ISP_PM_MEM_INIT;
    message.sync_flag = CMR_MSG_SYNC_NONE;
    ret = cmr_thread_msg_send(cxt->init_thread, &message);
    if (ret) {
        CMR_LOGE("send msg failed!");
        ret = CMR_CAMERA_FAIL;
    }
    ATRACE_END();
    return ret;
}

cmr_int camera_isp_pm_mem_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);
    pthread_mutex_lock(&cxt->isp_pm_mutex);

    while (cxt->isp_pm_initing) {
        pthread_cond_wait(&cxt->isp_pm_cond, &cxt->isp_pm_mutex);
    }

    if (!cxt->isp_pm_context_addr) {
        CMR_LOGI("isp pm mem has been free!");
        return ret;
    }

    cxt->err_code = CMR_CAMERA_SUCCESS;

    message.msg_type = CMR_EVT_ISP_PM_MEM_DEINIT;
    message.sync_flag = CMR_MSG_SYNC_PROCESSED;
    ret = cmr_thread_msg_send(cxt->init_thread, &message);
    if (ret) {
        CMR_LOGE("send msg failed!");
        ret = CMR_CAMERA_FAIL;
    }
    pthread_mutex_unlock(&cxt->isp_pm_mutex);
    ATRACE_END();
    return ret;
}

cmr_int camera_isp_init_nosync(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);
    CMR_LOGI("E");
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_MSG_INIT(message);
    sem_init(&cxt->isp_sm, 0, 1);
    cxt->err_code = CMR_CAMERA_SUCCESS;

    message.msg_type = CMR_EVT_ISP_INIT;
    message.sync_flag = CMR_MSG_SYNC_NONE;
    ret = cmr_thread_msg_send(cxt->init_thread, &message);
    if (ret) {
        CMR_LOGE("send msg failed!");
        ret = CMR_CAMERA_FAIL;
    }
    ATRACE_END();
    CMR_LOGI("X");
    return ret;
}

cmr_int camera_init_internal(cmr_handle oem_handle, cmr_uint is_autotest) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    cmr_int ret_deinit = CMR_CAMERA_SUCCESS;
    pthread_mutex_lock(&close_mutex);
    while (closing != 0) {
        pthread_cond_wait(&close_cond, &close_mutex);
    }
    pthread_mutex_unlock(&close_mutex);
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGI("E");
    ret = camera_create_init_thread(oem_handle);
    if (ret) {
        CMR_LOGE("failed to create init thread %ld", ret);
        goto exit;
    }

    /*for multicamera mode,when open convered sensor,only need to init sensor
     * and res*/
    if (CONVERED_CAMERA_INIT) {
        ret = camera_sensor_init(oem_handle, is_autotest);
        if (ret) {
            CMR_LOGE("failed to init sensor %ld", ret);
            goto thread_deinit;
        }
        ret = camera_res_init(oem_handle);
        if (ret) {
            CMR_LOGE("failed to init res %ld", ret);
            goto grab_deinit;
        }
        ret = camera_res_init_done(oem_handle);
        goto exit;
    }

    ret = camera_isp_pm_mem_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init isp pm mem %ld", ret);
        goto thread_deinit;
    }

    ret = camera_sensor_init(oem_handle, is_autotest);
    if (ret) {
        CMR_LOGE("failed to init sensor %ld", ret);
        goto isp_pm_mem_deinit;
    }

    ret = camera_grab_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init grab %ld", ret);
        goto sensor_deinit;
    }

    ret = camera_res_init(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init res %ld", ret);
        goto grab_deinit;
    }

    ret = camera_isp_init_nosync(oem_handle);
    if (ret) {
        CMR_LOGE("failed to init isp %ld", ret);
        goto res_deinit;
    }
    ret = camera_res_init_done(oem_handle);
    camera_front_lcd_enhance_module_init(oem_handle);
    goto exit;

res_deinit:
    ret_deinit = camera_res_deinit(oem_handle);
    if (ret_deinit) {
        CMR_LOGE("failed to camera_res_deinit %ld", ret_deinit);
    }

grab_deinit:
    ret_deinit = camera_grab_deinit(oem_handle);
    if (ret_deinit) {
        CMR_LOGE("failed to camera_grab_deinit %ld", ret_deinit);
    }

sensor_deinit:
    ret_deinit = camera_sensor_deinit(oem_handle);
    if (ret_deinit) {
        CMR_LOGE("failed to camera_sensor_deinit %ld", ret_deinit);
    }

isp_pm_mem_deinit:
    ret_deinit = camera_isp_pm_mem_deinit(oem_handle);
    if (ret_deinit) {
        CMR_LOGE("failed to camera_isp_pm_mem_deinit %ld", ret_deinit);
    }

thread_deinit:
    ret_deinit = camera_destroy_init_thread(oem_handle);
    if (ret_deinit) {
        CMR_LOGE("failed to destory init thread %ld", ret_deinit);
    }

exit:
    CMR_LOGI("X");
    ATRACE_END();
    return ret;
}

cmr_int camera_deinit_internal(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    pthread_mutex_lock(&close_mutex);
    closing++;
    pthread_mutex_unlock(&close_mutex);
    CMR_LOGI("E");
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    /*for multicamera mode,when close convered sensor,only need to deinit sensor
     * and res*/
    if (CONVERED_CAMERA_INIT) {
        camera_res_deinit(oem_handle);
        camera_sensor_deinit(oem_handle);
        camera_destroy_init_thread(oem_handle);
        pthread_mutex_lock(&close_mutex);
        closing--;
        if (closing == 0) {
            pthread_cond_signal(&close_cond);
        }
        pthread_mutex_unlock(&close_mutex);
        goto exit;
    }

    camera_front_lcd_enhance_module_deinit(oem_handle);
    camera_isp_deinit_notice(oem_handle);
    camera_isp_deinit(oem_handle);
    camera_res_deinit(oem_handle);
    camera_sensor_deinit(oem_handle);
    camera_grab_deinit(oem_handle);
    camera_isp_pm_mem_deinit(oem_handle);
    camera_destroy_init_thread(oem_handle);
    pthread_mutex_lock(&close_mutex);
    closing--;
    if (closing == 0) {
        pthread_cond_signal(&close_cond);
    }
    pthread_mutex_unlock(&close_mutex);

    pthread_mutex_destroy(&cxt->isp_pm_mutex);
    pthread_cond_destroy(&cxt->isp_pm_cond);
exit:

    ATRACE_END();
    CMR_LOGI("X");
    return ret;
}

cmr_int camera_nightpro_init(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_LOGD(" E");
    cxt->night_cxt.sw_handle = dlopen("libnight.so", RTLD_NOW);
    if(!cxt->night_cxt.sw_handle) {
          CMR_LOGD("libnight open failed with %s",dlerror());
    } else {
        cxt->night_cxt.sw_open = dlsym(cxt->night_cxt.sw_handle, "camera_sw_open");
        cxt->night_cxt.sw_process = dlsym(cxt->night_cxt.sw_handle, "camera_sw_process");
        cxt->night_cxt.sw_close = dlsym(cxt->night_cxt.sw_handle, "camera_sw_close");
        cxt->night_cxt.ipmpro_init = dlsym(cxt->night_cxt.sw_handle, "camera_ipm_pro_init");
        cxt->night_cxt.ipmpro_deinit = dlsym(cxt->night_cxt.sw_handle, "camera_ipm_pro_deinit");
        cxt->night_cxt.ipmpro_process = dlsym(cxt->night_cxt.sw_handle, "camera_ipm_pro_process");
        if (!cxt->night_cxt.sw_open || !cxt->night_cxt.sw_process ||
            !cxt->night_cxt.sw_close || !cxt->night_cxt.ipmpro_init ||
            !cxt->night_cxt.ipmpro_deinit || !cxt->night_cxt.ipmpro_process ) {
            CMR_LOGD("func analyzing failed with %s", dlerror());
            dlclose(cxt->night_cxt.sw_handle);
        } else {
            cxt->night_cxt.is_authorized = 1;
        }
    }
    CMR_LOGD("is_authorized %d", cxt->night_cxt.is_authorized);

    if (cxt->night_cxt.is_authorized) {
        ret= cxt->night_cxt.ipmpro_init(oem_handle);
        if (ret) {
            dlclose(cxt->night_cxt.sw_handle);
            cxt->night_cxt.is_authorized = 0;
            CMR_LOGE("failed to init ipm pro %ld", ret);
        }
    }
    CMR_LOGD(" X");
    return ret;
    ATRACE_END();
}

cmr_int camera_nightpro_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cxt->night_flag = true;
    CMR_LOGD("D");
    if(cxt->night_cxt.ipmpro_deinit)
        cxt->night_cxt.ipmpro_deinit(oem_handle);
    dlclose(cxt->night_cxt.sw_handle);
    cxt->night_cxt.sw_open = NULL;
    cxt->night_cxt.sw_process = NULL;
    cxt->night_cxt.sw_close = NULL;
    cxt->night_cxt.ipmpro_init = NULL;
    cxt->night_cxt.ipmpro_deinit = NULL;
    cxt->night_cxt.ipmpro_process = NULL;
    cxt->night_cxt.is_authorized = 0;
    CMR_LOGD("X");
    return ret;
    ATRACE_END();
}

cmr_int camera_jpeg_encode_exif_simplify(cmr_handle oem_handle,
                                         struct enc_exif_param *param) {
    ATRACE_BEGIN(__FUNCTION__);

    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct img_frm src = param->src;
    struct img_frm pic_enc = param->pic_enc;
    struct img_frm dst = param->last_dst;
    struct cmr_op_mean mean;
    struct jpeg_enc_cb_param enc_cb_param;
    struct jpeg_context *jpeg_cxt = NULL;
    int ret = CMR_CAMERA_SUCCESS;
    int need_exif_flag = (dst.addr_vir.addr_y == 0) ? 0 : 1;
    cmr_u32 SUPER_FINE = 95;
    cmr_u32 FINE = 80;
    cmr_u32 NORMAL = 70;

    if (!oem_handle || !src.addr_vir.addr_y || !pic_enc.addr_vir.addr_y) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    jpeg_cxt = &cxt->jpeg_cxt;

    sem_wait(&cxt->access_sm);
    // 1.construct param
    memset(&mean, 0, sizeof(struct cmr_op_mean));
    mean.quality_level = SUPER_FINE;
    mean.slice_mode = JPEG_YUV_SLICE_ONE_BUF;
    mean.slice_height = pic_enc.size.height;
    mean.is_sync = 1;
    src.data_end.y_endian = 0;
    src.data_end.uv_endian = 2;
    // for cache coherency
    cmr_snapshot_memory_flush(cxt->snp_cxt.snapshot_handle, &src);
    // 2.call jpeg interface
    ret = cmr_jpeg_encode(jpeg_cxt->jpeg_handle, &src, &pic_enc,
                          (struct jpg_op_mean *)&mean, &enc_cb_param);

    if (ret) {
        cxt->jpeg_cxt.enc_caller_handle = (cmr_handle)0;
        CMR_LOGE("failed to jpeg enc %ld", ret);
        goto exit;
    }

    param->stream_real_size = enc_cb_param.stream_size;
    CMR_LOGD("need_exif_flag:%d, param->stream_real_size:%u,is_sync=%d",
             need_exif_flag, param->stream_real_size, mean.is_sync);
    if (need_exif_flag) {
        struct jpeg_wexif_cb_param enc_out_param;
        ret = camera_start_exif_encode_simplify(oem_handle, &pic_enc, &dst,
                                                &enc_out_param);
        if (ret) {
            CMR_LOGE("failed to camera_start_exif_encode %ld", ret);
            goto exit;
        }
    }

exit:
    sem_post(&cxt->access_sm);
    CMR_LOGD("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_start_encode(cmr_handle oem_handle, cmr_handle caller_handle,
                            struct img_frm *src, struct img_frm *dst,
                            struct cmr_op_mean *mean,
                            struct jpeg_enc_cb_param *enc_cb_param) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return ret;
    }
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_enc_in_param enc_in_param;
    cmr_uint stream_size;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct setting_cmd_parameter setting_param;
    cmr_uint rotation = 0;
    cmr_uint flip_on = 0;
    cmr_u32 is_raw_capture = 0;
    char value[PROPERTY_VALUE_MAX];
    cmr_u32 tmp = 0;
    struct timespec start_time, end_time;
    unsigned int duration;
    cmr_s32 filter_type = 0;
    struct jpeg_context *jpeg_cxt = NULL;

    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);
    cxt->jpeg_cxt.enc_caller_handle = caller_handle;
    sem_wait(&cxt->access_sm);

    if (!caller_handle || !oem_handle || !src || !dst || !mean) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    property_get("persist.vendor.cam.raw.mode", value, "jpeg");
    if (!strcmp(value, "raw")) {
        is_raw_capture = 1;
    }

    cmr_bzero((void *)&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;

    // workaround jpeg cant handle 16-noalign issue, when jpeg fix this issue,
    // we will remove these code
    if (is_raw_capture == 0) {
        if (dst->size.height == 1952 && dst->size.width == 2592) {
            dst->size.height = 1944;
        } else if (dst->size.height == 1840 && dst->size.width == 3264) {
            dst->size.height = 1836;
        } else if (dst->size.height == 368 && dst->size.width == 640) {
            dst->size.height = 360;
        }
    }

/* from sharkl2, jpeg support mirror/flip/rotation, mirror feature always use
 * jpeg if jpeg support it */
#ifdef MIRROR_FLIP_ROTATION_BY_JPEG
    /* raw capture not support mirror/flip/rotation*/
    mean->flip = 0;
    mean->rot = 0;
    mean->mirror = 0;

    if (is_raw_capture == 0) {
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_ENCODE_ROTATION, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get enc rotation %ld", ret);
            goto exit;
        }
        rotation = setting_param.cmd_type_value;

        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_FLIP_ON, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get preview sprd flip_on enabled flag %ld",
                     ret);
            goto exit;
        }
        flip_on = setting_param.cmd_type_value;
        CMR_LOGV("encode_rotation:%ld, flip:%ld", rotation, flip_on);

        if (0 != rotation) {
            if (90 == rotation)
                mean->rot = 1;
            else if (180 == rotation) {
                mean->flip = 1;
                mean->mirror = 1;
            } else if (270 == rotation) {
                mean->rot = 1;
                mean->flip = 1;
                mean->mirror = 1;
            }
        }

        if (flip_on) {
            if (mean->mirror)
                mean->mirror = 0;
            else
                mean->mirror = 1;
        }

        if ((90 == rotation || 270 == rotation)) {
            tmp = dst->size.height;
            dst->size.height = dst->size.width;
            dst->size.width = tmp;
        }
    }
#endif

    cxt->jpeg_cxt.enc_caller_handle = caller_handle;

    CMR_LOGI("src: fd=0x%x, y_offset=0x%lx, u_offset=0x%lx, virt_y=0x%lx, "
             "virt_u=0x%lx",
             src->fd, src->addr_phy.addr_y, src->addr_phy.addr_u,
             src->addr_vir.addr_y, src->addr_vir.addr_u);
    CMR_LOGI("src: width=%d, height=%d, y_endian=%d, uv_endian=%d, "
             "mirror=%d,flip=%d,rot=%d",
             src->size.width, src->size.height, src->data_end.y_endian,
             src->data_end.uv_endian, mean->mirror, mean->flip, mean->rot);
    CMR_LOGI("dst: fd=0x%x, stream_offset=0x%lx, stream_vir=0x%lx, width=%d, "
             "height=%d",
             dst->fd, dst->addr_phy.addr_y, dst->addr_vir.addr_y,
             dst->size.width, dst->size.height);

    cmr_snapshot_memory_flush(cxt->snp_cxt.snapshot_handle, dst);

    if (1 != mean->is_thumb) {
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_FILTER_TEYP, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get filtertype %ld", ret);
            goto exit;
        }
        filter_type = setting_param.cmd_type_value;

        if ((cxt->is_multi_mode == MODE_SINGLE_CAMERA ||
             cxt->is_multi_mode == MODE_SELF_SHOT ||
             (cxt->is_multi_mode == MODE_BLUR &&
              cxt->blur_facebeauty_flag == 1)) &&
            (filter_type == 0)) {
#ifdef CONFIG_FACE_BEAUTY
            struct face_beauty_levels beautyLevels;
            struct common_isp_cmd_param isp_param;
            int pic_width = src->size.width;
            int pic_height = src->size.height;
            int face_beauty_on = 0;
            ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                    SETTING_GET_PERFECT_SKINLEVEL,
                                    &setting_param);
            if (ret) {
                CMR_LOGE("failed to get perfect skinlevel %ld", ret);
                goto exit;
            }
            ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_ADGAIN_EXP, &isp_param);
            ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_COL_TEM, &isp_param);
            ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_SENS, &isp_param);
            beautyLevels.blemishLevel =
                (unsigned char)setting_param.fb_param.blemishLevel;
            beautyLevels.smoothLevel =
                (unsigned char)setting_param.fb_param.smoothLevel;
            beautyLevels.skinColor =
                (unsigned char)setting_param.fb_param.skinColor;
            beautyLevels.skinLevel =
                (unsigned char)setting_param.fb_param.skinLevel;
            beautyLevels.brightLevel =
                (unsigned char)setting_param.fb_param.brightLevel;
            beautyLevels.lipColor =
                (unsigned char)setting_param.fb_param.lipColor;
            beautyLevels.lipLevel =
                (unsigned char)setting_param.fb_param.lipLevel;
            beautyLevels.slimLevel =
                (unsigned char)setting_param.fb_param.slimLevel;
            beautyLevels.largeLevel =
                (unsigned char)setting_param.fb_param.largeLevel;
            beautyLevels.cameraBV = (int)isp_param.isp_adgain.bv;
            beautyLevels.cameraCT = (int)isp_param.isp_cur_ct;
            beautyLevels.cameraISO = (int)isp_param.isp_cur_iso;
            beautyLevels.cameraWork = (int)cxt->camera_id;
            CMR_LOGV("cameraBV %d, cameraWork %d, cameraCT %d, cameraISO %d",
                beautyLevels.cameraBV, beautyLevels.cameraWork,
                beautyLevels.cameraCT,  beautyLevels.cameraISO);

            ret =
                cmr_setting_ioctl(setting_cxt->setting_handle,
                                  SETTING_GET_ENCODE_ROTATION, &setting_param);
            if (ret) {
                CMR_LOGE("failed to get enc rotation %ld", ret);
            }

            face_beauty_on = beautyLevels.blemishLevel ||
                             beautyLevels.smoothLevel ||
                             beautyLevels.skinColor || beautyLevels.skinLevel ||
                             beautyLevels.brightLevel ||
                             beautyLevels.lipColor || beautyLevels.lipLevel ||
                             beautyLevels.slimLevel || beautyLevels.largeLevel;
            if (face_beauty_on) {
                CMR_LOGD("face_beauty smooth %d,bright %d,slim %d,large %d",
                         beautyLevels.smoothLevel, beautyLevels.brightLevel,
                         beautyLevels.slimLevel, beautyLevels.largeLevel);
                struct fb_beauty_face_t faceinfo;
                for (int i = 0; i < cxt->fd_face_area.face_num; i++) {
                    faceinfo.startX= (cxt->fd_face_area.face_info[i].sx * pic_width) /
                         (cxt->fd_face_area.frame_width);
                    faceinfo.startY= (cxt->fd_face_area.face_info[i].sy * pic_height) /
                         (cxt->fd_face_area.frame_height);
                    faceinfo.endX= (cxt->fd_face_area.face_info[i].ex * pic_width) /
                         (cxt->fd_face_area.frame_width);
                    faceinfo.endY= (cxt->fd_face_area.face_info[i].ey * pic_height) /
                         (cxt->fd_face_area.frame_height);
                    faceinfo.angle = cxt->fd_face_area.face_info[i].angle;
                    faceinfo.pose = cxt->fd_face_area.face_info[i].pose;
                    faceinfo.idx = i;
                    construct_fb_face(&(cxt->face_beauty), faceinfo);
                }

                fb_chipinfo chipinfo;
#if defined(CONFIG_ISP_2_3)
                chipinfo = SHARKLE;
#elif defined(CONFIG_ISP_2_4)
                chipinfo = PIKE2;
#elif defined(CONFIG_ISP_2_5)
                chipinfo = SHARKL3;
#elif defined(CONFIG_ISP_2_7)
                chipinfo = SHARKL5PRO;
#endif
                init_fb_handle(&(cxt->face_beauty), 0, 2, chipinfo);
                construct_fb_image(&(cxt->face_beauty), pic_width, pic_height,
                                   (unsigned char *)(src->addr_vir.addr_y),
                                   (unsigned char *)(src->addr_vir.addr_u), 0);
                construct_fb_level(&(cxt->face_beauty), beautyLevels);
                do_face_beauty(&(cxt->face_beauty), cxt->fd_face_area.face_num);
                deinit_fb_handle(&(cxt->face_beauty));

                // for cache coherency
                cmr_snapshot_memory_flush(cxt->snp_cxt.snapshot_handle, src);
                CMR_LOGD("face_beauty done");
            }
#endif // CONFIG_FACE_BEAUTY
        }

        CMR_LOGI("is_sync=%d, is_thumb=%d", mean->is_sync, mean->is_thumb);
        ret = cmr_jpeg_encode(jpeg_cxt->jpeg_handle, src, dst,
                              (struct jpg_op_mean *)mean, enc_cb_param);
        if (ret) {
            CMR_LOGE("failed to jpeg codec %ld", ret);
        }
    } else {
        CMR_LOGI("is_sync=%d, is_thumb=%d", mean->is_sync, mean->is_thumb);
        ret = cmr_jpeg_encode(jpeg_cxt->jpeg_handle, src, dst,
                              (struct jpg_op_mean *)mean, enc_cb_param);
        if (ret) {
            CMR_LOGE("failed to jpeg codec %ld", ret);
        }
    }
    if (ret) {
        cxt->jpeg_cxt.enc_caller_handle = (cmr_handle)0;
        CMR_LOGE("failed to enc start %ld", ret);
        goto exit;
    }

exit:
    sem_post(&cxt->access_sm);
    ATRACE_END();
    return ret;
}

cmr_int camera_start_decode(cmr_handle oem_handle, cmr_handle caller_handle,
                            struct img_frm *src, struct img_frm *dst,
                            struct cmr_op_mean *mean) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_dec_in_param dec_in_param;
    struct jpeg_context *jpeg_cxt = NULL;

    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);

    if (!caller_handle || !oem_handle || !src || !dst || !mean) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGI("src phy addr 0x%lx src vir addr 0x%lx", src->addr_phy.addr_y,
             src->addr_vir.addr_y);
    CMR_LOGI("dst phr addr 0x%lx  0x%lx vir addr 0x%lx 0x%lx",
             dst->addr_phy.addr_y, dst->addr_phy.addr_u, dst->addr_vir.addr_y,
             dst->addr_vir.addr_u);
    CMR_LOGI("src size %d %d", src->size.width, src->size.height);
    CMR_LOGI("out size %d %d", dst->size.width, dst->size.height);
    CMR_LOGI("temp size %d", mean->temp_buf.buf_size);
    CMR_LOGD("caller_handle 0x%lx", (cmr_uint)caller_handle);

    cxt->jpeg_cxt.dec_caller_handle = caller_handle;
    ret = cmr_jpeg_decode(jpeg_cxt->jpeg_handle, src, dst,
                          (struct jpg_op_mean *)mean);
    if (ret) {
        cxt->jpeg_cxt.dec_caller_handle = (cmr_handle)0;
        CMR_LOGE("dec start fail ret %ld", ret);
    }
exit:
    CMR_LOGD("done %ld", ret);
    return ret;
}

cmr_int camera_stop_codec(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_context *jpeg_cxt = NULL;

    CHECK_HANDLE_VALID(oem_handle);
    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);

    ret = cmr_stop_codec(jpeg_cxt->jpeg_handle);
    if (ret) {
        CMR_LOGE("failed to stop codec %ld", ret);
    }
    return ret;
}

cmr_int camera_start_exif_encode(cmr_handle oem_handle,
                                 cmr_handle caller_handle,
                                 struct img_frm *pic_src,
                                 struct img_frm *thumb_src, void *exif,
                                 struct img_frm *dst,
                                 struct jpeg_wexif_cb_param *out_ptr) {
    UNUSED(exif);
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;

    struct jpeg_enc_exif_param enc_exif_param;
    struct jpeg_wexif_cb_param out_pram;
    struct setting_cmd_parameter setting_param;
    struct common_isp_cmd_param isp_param;
    struct jpeg_context *jpeg_cxt = NULL;

    if (!caller_handle || !oem_handle || !pic_src || !dst || !thumb_src ||
        !out_ptr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);

    cmr_bzero(&enc_exif_param, sizeof(struct jpeg_enc_exif_param));
    cmr_bzero(&out_pram, sizeof(struct jpeg_wexif_cb_param));
    cmr_bzero(&setting_param, sizeof(struct setting_cmd_parameter));
    cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));

    if (1 == camera_get_hdr_flag(cxt)) {
        camera_set_exif_exposure_time(oem_handle);
    }

    enc_exif_param.jpeg_handle = cxt->jpeg_cxt.jpeg_handle;
    enc_exif_param.src_jpeg_addr_virt = pic_src->addr_vir.addr_y;
    enc_exif_param.thumbnail_addr_virt = thumb_src->addr_vir.addr_y;
    enc_exif_param.target_addr_virt = dst->addr_vir.addr_y;
    enc_exif_param.src_jpeg_size = pic_src->buf_size;
    enc_exif_param.thumbnail_size = thumb_src->buf_size;
    enc_exif_param.target_size = dst->buf_size;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_EXIF_INFO, &setting_param);
    enc_exif_param.exif_ptr = setting_param.exif_all_info_ptr;
    enc_exif_param.exif_isp_info = NULL;

    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_EXIF_DEBUG_INFO, &isp_param);
    if (ret) {
        CMR_LOGW("isp get exif debug info failed");
        enc_exif_param.exif_isp_debug_info.addr = NULL;
        enc_exif_param.exif_isp_debug_info.size = 0;
    } else {
        enc_exif_param.exif_isp_debug_info.addr = isp_param.isp_dbg_info.addr;
        enc_exif_param.exif_isp_debug_info.size = isp_param.isp_dbg_info.size;
    }

    CMR_LOGV("exif_isp_debug_info: addr=%p, size=%d",
             enc_exif_param.exif_isp_debug_info.addr,
             enc_exif_param.exif_isp_debug_info.size);

    enc_exif_param.padding = 0;
    out_pram.output_buf_virt_addr = 0;
    out_pram.output_buf_size = 0;
    ret = cmr_jpeg_enc_add_eixf(jpeg_cxt->jpeg_handle, &enc_exif_param,
                                &out_pram);
    if (!ret) {
        *out_ptr = out_pram;
        CMR_LOGI("out addr 0x%lx size %ld", out_ptr->output_buf_virt_addr,
                 out_ptr->output_buf_size);
    } else {
        CMR_LOGE("failed to compund exif jpeg %ld", ret);
    }

exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_start_exif_encode_simplify(cmr_handle oem_handle,
                                          struct img_frm *pic_src,
                                          struct img_frm *dst,
                                          struct jpeg_wexif_cb_param *out_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_context *jpeg_cxt = NULL;
    struct isp_context *isp_cxt = &cxt->isp_cxt;

    struct jpeg_enc_exif_param enc_exif_param;
    struct jpeg_wexif_cb_param out_pram;
    struct setting_cmd_parameter setting_param;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle || !pic_src || !dst || !out_ptr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    jpeg_cxt = &cxt->jpeg_cxt;
    CHECK_HANDLE_VALID(jpeg_cxt);

    cmr_bzero((void *)&enc_exif_param, sizeof(struct jpeg_enc_exif_param));
    cmr_bzero((void *)&out_pram, sizeof(struct jpeg_wexif_cb_param));
    cmr_bzero((void *)&setting_param, sizeof(struct setting_cmd_parameter));
    cmr_bzero((void *)&isp_param, sizeof(struct common_isp_cmd_param));

    enc_exif_param.jpeg_handle = cxt->jpeg_cxt.jpeg_handle;
    enc_exif_param.src_jpeg_addr_virt = pic_src->addr_vir.addr_y;
    // enc_exif_param.thumbnail_addr_virt = thumb_src->addr_vir.addr_y;
    enc_exif_param.target_addr_virt = dst->addr_vir.addr_y;
    enc_exif_param.src_jpeg_size = pic_src->buf_size;
    // enc_exif_param.thumbnail_size = thumb_src->buf_size;
    enc_exif_param.target_size = dst->buf_size;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_EXIF_INFO, &setting_param);
    enc_exif_param.exif_ptr = setting_param.exif_all_info_ptr;
    enc_exif_param.exif_isp_info = NULL;

    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_EXIF_DEBUG_INFO, &isp_param);
    if (ret) {
        CMR_LOGW("isp get exif debug info failed");
        enc_exif_param.exif_isp_debug_info.addr = NULL;
        enc_exif_param.exif_isp_debug_info.size = 0;
    } else {
        enc_exif_param.exif_isp_debug_info.addr = isp_param.isp_dbg_info.addr;
        enc_exif_param.exif_isp_debug_info.size = isp_param.isp_dbg_info.size;
    }

    CMR_LOGV("exif_isp_debug_info: addr=%p, size=%d",
             enc_exif_param.exif_isp_debug_info.addr,
             enc_exif_param.exif_isp_debug_info.size);

    enc_exif_param.padding = 0;
    out_pram.output_buf_virt_addr = 0;
    out_pram.output_buf_size = 0;
    ret = cmr_jpeg_enc_add_eixf(jpeg_cxt->jpeg_handle, &enc_exif_param,
                                &out_pram);
    if (!ret) {
        *out_ptr = out_pram;
        CMR_LOGI("out addr 0x%lx size %ld", out_ptr->output_buf_virt_addr,
                 out_ptr->output_buf_size);
    } else {
        CMR_LOGE("failed to compund exif jpeg %ld", ret);
    }

exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_start_scale(cmr_handle oem_handle, cmr_handle caller_handle,
                           struct img_frm *src, struct img_frm *dst,
                           struct cmr_op_mean *mean) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !src || !dst || !mean) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGD("caller_handle 0x%lx, is_sync %d, src fd 0x%x, dst fd 0x%x"
             ", src 0x%lx 0x%lx, dst 0x%lx 0x%lx",
             (cmr_uint)caller_handle, mean->is_sync, src->fd, dst->fd,
             src->addr_phy.addr_y, src->addr_phy.addr_u, dst->addr_phy.addr_y,
             dst->addr_phy.addr_u);

    CMR_LOGI(
        "src size %d %d, dst size %d %d, rect %d %d %d %d, endian %d %d, %d %d",
        src->size.width, src->size.height, dst->size.width, dst->size.height,
        src->rect.start_x, src->rect.start_y, src->rect.width, src->rect.height,
        src->data_end.y_endian, src->data_end.uv_endian, dst->data_end.y_endian,
        dst->data_end.uv_endian);

    if (1 != mean->is_sync) {
        ret = cmr_scale_start(cxt->scaler_cxt.scaler_handle, src, dst,
                              camera_scaler_evt_cb, (void *)caller_handle);
    } else {
        ret = cmr_scale_start(cxt->scaler_cxt.scaler_handle, src, dst,
                              (cmr_evt_cb)NULL, NULL);
    }
    if (ret) {
        CMR_LOGE("failed to start scaler, ret %ld", ret);
    }
exit:
    CMR_LOGI("done %ld", ret);

    ATRACE_END();
    return ret;
}

cmr_int camera_start_rot(cmr_handle oem_handle, cmr_handle caller_handle,
                         struct img_frm *src, struct img_frm *dst,
                         struct cmr_op_mean *mean) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct cmr_rot_param rot_param;
    cmr_uint restart_cnt = 0;

    if (!caller_handle || !oem_handle || !src || !dst || !mean) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    camera_take_snapshot_step(CMR_STEP_ROT_S);
    do {
        rot_param.angle = mean->rot;
        rot_param.handle = cxt->rot_cxt.rotation_handle;
        rot_param.src_img = *src;
        rot_param.dst_img = *dst;
        ret = cmr_rot(&rot_param);
        if (ret) {
            CMR_LOGE("failed to rotate %ld", ret);
            ret = camera_restart_rot(oem_handle);
        } else {
            goto rot_end;
        }
        restart_cnt++;
    } while (restart_cnt < OEM_RESTART_SUM);
rot_end:
    camera_take_snapshot_step(CMR_STEP_ROT_E);
exit:
    CMR_LOGI("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_preview_pre_proc(cmr_handle oem_handle, cmr_u32 camera_id,
                                cmr_u32 preview_sn_mode) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_u32 sensor_mode;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    /*
    if (CAMERA_ZSL_MODE != cxt->snp_cxt.snp_mode) {
            sensor_mode = cxt->prev_cxt.preview_sn_mode;
    } else {
            sensor_mode = cxt->snp_cxt.snapshot_sn_mode;
    } */
    sensor_mode = preview_sn_mode;
    cxt->prev_cxt.preview_sn_mode = preview_sn_mode;

    CMR_LOGI("camera id %d, sensor work mode %d", camera_id, sensor_mode);
    ret =
        cmr_sensor_set_mode(cxt->sn_cxt.sensor_handle, camera_id, sensor_mode);
    if (ret) {
        CMR_LOGE("failed to set sensor work mode %ld", ret);
        goto exit;
    }
    ret = cmr_sensor_set_mode_done(cxt->sn_cxt.sensor_handle, camera_id);
    if (ret) {
        CMR_LOGE("failed to wait done %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGI("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_preview_post_proc(cmr_handle oem_handle, cmr_u32 camera_id) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    cmr_int flash_status;

    if (1 == camera_id && (strcmp(FRONT_CAMERA_FLASH_TYPE, "lcd") &
                           strcmp(FRONT_CAMERA_FLASH_TYPE, "flash")))
        goto exit;

    setting_param.camera_id = camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_HW_FLASH_STATUS, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get flash mode %ld", ret);
        goto exit;
    }
    flash_status = setting_param.cmd_type_value;
    CMR_LOGI("HW flash_status=%ld", flash_status);
    /*close flash*/
    if ((CAMERA_ZSL_MODE != cxt->snp_cxt.snp_mode) &&
        ((FLASH_OPEN == flash_status) || (FLASH_HIGH_LIGHT == flash_status))) {
        memset(&setting_param, 0, sizeof(setting_param));
        setting_param.camera_id = camera_id;
        setting_param.ctrl_flash.capture_mode.capture_mode =
            cxt->snp_cxt.snp_mode;
        setting_param.ctrl_flash.is_active = 0;
        setting_param.ctrl_flash.flash_type = FLASH_CLOSE_AFTER_OPEN;
        setting_param.ctrl_flash.work_mode = 0;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_CTRL_FLASH, &setting_param);
        if (ret) {
            CMR_LOGE("failed to open flash %ld", ret);
        }
    }

exit:
    return ret;
}

cmr_int camera_capture_pre_proc(cmr_handle oem_handle, cmr_u32 camera_id,
                                cmr_u32 preview_mode, cmr_u32 capture_mode,
                                cmr_u32 is_restart, cmr_u32 is_sn_reopen) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt;
    struct setting_cmd_parameter setting_param;
    struct common_sn_cmd_param sn_param;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle || (camera_id != cxt->camera_id)) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    snp_cxt = &cxt->snp_cxt;

    CMR_LOGI("camera id %d, capture mode %d preview mode %d,  is_restart %d "
             "is_sn_reopen %d, snapshot_sn_mode %d",
             camera_id, capture_mode, preview_mode, is_restart, is_sn_reopen,
             snp_cxt->snp_mode);

    if ((CAMERA_ZSL_MODE != snp_cxt->snp_mode) &&
        (!is_restart || is_sn_reopen)) {
        snp_cxt = &cxt->snp_cxt;
        snp_cxt->snapshot_sn_mode = capture_mode;
        cxt->prev_cxt.preview_sn_mode = preview_mode;
        /*set sensor before snapshot*/
        ret = cmr_sensor_ioctl(
            cxt->sn_cxt.sensor_handle, camera_id, SENSOR_BEFORE_SNAPSHOT,
            (capture_mode | (cxt->prev_cxt.preview_sn_mode << CAP_MODE_BITS)));
        if (ret) {
            CMR_LOGE("failed to set sensor %ld", ret);
        }
    }

exit:
    CMR_LOGI("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_capture_post_proc(cmr_handle oem_handle, cmr_u32 camera_id) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt;
    struct setting_cmd_parameter setting_param;
    struct common_isp_cmd_param isp_param;
    cmr_int flash_status = FLASH_CLOSE;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    snp_cxt = &cxt->snp_cxt;

    if (0 == camera_id ||
        !(strcmp(FRONT_CAMERA_FLASH_TYPE, "lcd") &
          strcmp(FRONT_CAMERA_FLASH_TYPE, "flash"))) {
        setting_param.camera_id = camera_id;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_HW_FLASH_STATUS, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get flash mode %ld", ret);
            // goto exit;
        }
        flash_status = setting_param.cmd_type_value;
        CMR_LOGI("HW flash_status=%ld", flash_status);
        CMR_LOGI("start");
        if (FLASH_OPEN == flash_status || FLASH_HIGH_LIGHT == flash_status) {
            /*close flash*/
            memset(&setting_param, 0, sizeof(setting_param));
            setting_param.camera_id = camera_id;
            setting_param.ctrl_flash.capture_mode.capture_mode =
                snp_cxt->snp_mode;
            setting_param.ctrl_flash.is_active = 0;
            setting_param.ctrl_flash.work_mode = 1; // capture
            setting_param.ctrl_flash.flash_type = FLASH_CLOSE_AFTER_OPEN;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                    SETTING_CTRL_FLASH, &setting_param);
            if (ret) {
                CMR_LOGE("failed to open flash");
            }
        }
    }

    /*set sensor after snapshot*/
    if ((CAMERA_ZSL_MODE != snp_cxt->snp_mode) &&
        (CAMERA_ISP_TUNING_MODE != snp_cxt->snp_mode) &&
        (CAMERA_ISP_SIMULATION_MODE != snp_cxt->snp_mode)) {
        ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                               SENSOR_AFTER_SNAPSHOT,
                               cxt->prev_cxt.preview_sn_mode);
        if (ret) {
            CMR_LOGE("failed to set sensor %ld", ret);
        }
    }
    // notify isp after snapshot
    ret =
        camera_isp_ioctl(oem_handle, COM_ISP_SET_SNAPSHOT_FINISHED, &isp_param);
    if (ret) {
        CMR_LOGE("failed to set snapshot finished %ld", ret);
    }

exit:
    CMR_LOGI("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_local_snapshot_is_need_flash(cmr_handle oem_handle,
                                            cmr_u32 camera_id,
                                            cmr_u32 *is_need_flash) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt;
    struct setting_cmd_parameter setting_param;
    if (!oem_handle || (camera_id != cxt->camera_id)) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (camera_id == 1 && (strcmp(FRONT_CAMERA_FLASH_TYPE, "lcd") &
                           strcmp(FRONT_CAMERA_FLASH_TYPE, "flash")))
        goto exit;

    snp_cxt = &cxt->snp_cxt;

    CMR_LOGI("camera id %d, capture mode %d", camera_id, snp_cxt->snp_mode);

    memset(&setting_param, 0, sizeof(setting_param));
    setting_param.ctrl_flash.capture_mode.capture_mode = snp_cxt->snp_mode;
    setting_param.camera_id = camera_id;
    setting_param.ctrl_flash.is_active = 1;
    setting_param.ctrl_flash.flash_type = FLASH_HIGH_LIGHT;
    setting_param.ctrl_flash.work_mode = 1; // capture
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_FLASH_STATUS, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get flash mode %ld", ret);
        goto exit;
    }
    *is_need_flash = setting_param.cmd_type_value;
    CMR_LOGI("is_need_flash = %d", *is_need_flash);
exit:
    return ret;
}

cmr_int camera_capture_highflash(cmr_handle oem_handle, cmr_u32 camera_id) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt;
    struct setting_cmd_parameter setting_param;
    cmr_uint has_preflashed;
    cmr_uint flash_status;

    if (!oem_handle || (camera_id != cxt->camera_id)) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    snp_cxt = &cxt->snp_cxt;

    CMR_LOGI("camera id %d, capture mode %d", camera_id, snp_cxt->snp_mode);

    if ((1 != camera_get_hdr_flag(cxt)) && (1 != camera_get_3dnr_flag(cxt))) {
        /*open flash*/
        memset(&setting_param, 0, sizeof(setting_param));
        setting_param.ctrl_flash.capture_mode.capture_mode = snp_cxt->snp_mode;
        setting_param.camera_id = camera_id;
        setting_param.ctrl_flash.is_active = 1;
        setting_param.ctrl_flash.flash_type = FLASH_HIGH_LIGHT;
        setting_param.ctrl_flash.work_mode = 1; // capture
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_FLASH_STATUS, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get flash mode %ld", ret);
            goto exit;
        }
        flash_status = setting_param.cmd_type_value;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_PRE_LOWFLASH_VALUE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get preflashed flag %ld", ret);
        }
        has_preflashed = setting_param.cmd_type_value;
        CMR_LOGD("flash_status = %ld, has_preflashed=%ld", flash_status,
                 has_preflashed);
        if (!ret && flash_status && has_preflashed) {
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                    SETTING_CTRL_FLASH, &setting_param);
            if (ret) {
                cxt->snp_high_flash_time = 0;
                CMR_LOGE("failed to open flash");
            }
            cxt->snp_high_flash_time = systemTime(CLOCK_MONOTONIC);
            CMR_LOGV("snp_high_flash_time %lld", cxt->snp_high_flash_time);
        }
    }

exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_set_3dnr_video(cmr_handle oem_handle, cmr_uint is_3dnr_video) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cxt->is_3dnr_video = is_3dnr_video;
    CMR_LOGI("is_3dnr_video %ld", cxt->is_3dnr_video);
    return ret;
}

cmr_int camera_open_sensor(cmr_handle oem_handle, cmr_u32 camera_id) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_context *sn_cxt = &cxt->sn_cxt;
    cmr_handle sensor_handle;
    cmr_u32 camera_id_bits = 0;

    (void)camera_id;

    sensor_handle = sn_cxt->sensor_handle;
    camera_id_bits = 1 << cxt->camera_id;
    ret = cmr_sensor_open(sensor_handle, camera_id_bits);
    if (ret) {
        CMR_LOGE("open %d sensor failed %ld", cxt->camera_id, ret);
        return CMR_CAMERA_FAIL;
    }
    ATRACE_END();
    return ret;
}

cmr_int camera_close_sensor(cmr_handle oem_handle, cmr_u32 camera_id) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_context *sn_cxt = &cxt->sn_cxt;
    cmr_handle sensor_handle;
    cmr_u32 camera_id_bits = 0;

    (void)camera_id;
    sensor_handle = sn_cxt->sensor_handle;
    camera_id_bits = 1 << cxt->camera_id;
    ret = cmr_sensor_close(sensor_handle, camera_id_bits);
    if (ret) {
        CMR_LOGE("open %d sensor failed %ld", cxt->camera_id, ret);
        return CMR_CAMERA_FAIL;
    }
    ATRACE_END();
    return ret;
}

cmr_int camera_raw_proc(cmr_handle oem_handle, cmr_handle caller_handle,
                        struct raw_proc_param *param_ptr) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct sensor_ex_info *sns_ex_info_ptr = NULL;

    if (!oem_handle || !param_ptr || !caller_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGI("slice num %d avail height %d slice height %d",
             param_ptr->slice_num, param_ptr->src_avail_height,
             param_ptr->src_slice_height);
    CMR_LOGI("src addr 0x%lx 0x%lx pattern %d,dst addr 0x%lx 0x%lx, dst2 addr "
             "0x%lx 0x%lx",
             param_ptr->src_frame.addr_phy.addr_y,
             param_ptr->src_frame.addr_phy.addr_u,
             param_ptr->src_frame.format_pattern,
             param_ptr->dst_frame.addr_phy.addr_y,
             param_ptr->dst_frame.addr_phy.addr_u,
             param_ptr->dst2_frame.addr_phy.addr_y,
             param_ptr->dst2_frame.addr_phy.addr_u);
    CMR_LOGI("fd: src.fd=0x%x, dst.fd=0x%x, dst2.fd=0x%x",
             param_ptr->src_frame.fd, param_ptr->dst_frame.fd,
             param_ptr->dst2_frame.fd);

    if (1 == param_ptr->slice_num) {
        struct ips_in_param in_param;
        struct ips_out_param out_param;

        in_param.oem_handle = oem_handle;
        in_param.alloc_cb = camera_malloc;
        in_param.free_cb = camera_free;

        in_param.src_frame.img_fmt = param_ptr->src_frame.fmt;
        in_param.src_frame.img_size.w = param_ptr->src_frame.size.width;
        in_param.src_frame.img_size.h = param_ptr->src_frame.size.height;
        in_param.src_frame.img_fd.y = param_ptr->src_frame.fd;
        in_param.src_frame.img_fd.u = param_ptr->src_frame.fd;
        in_param.src_frame.img_addr_phy.chn0 =
            param_ptr->src_frame.addr_phy.addr_y;
        in_param.src_frame.img_addr_phy.chn1 =
            param_ptr->src_frame.addr_phy.addr_u;
        in_param.src_frame.img_addr_vir.chn0 =
            param_ptr->src_frame.addr_vir.addr_y;
        in_param.src_frame.img_addr_vir.chn1 =
            param_ptr->src_frame.addr_vir.addr_u;
        //		in_param.src_frame.format_pattern =
        // param_ptr->src_frame.format_pattern;
        in_param.src_avail_height = param_ptr->src_avail_height;
        in_param.src_slice_height = param_ptr->src_slice_height;
        in_param.dst_frame.img_fmt = param_ptr->dst_frame.fmt;
        in_param.dst_frame.img_size.w = param_ptr->dst_frame.size.width;
        in_param.dst_frame.img_size.h = param_ptr->dst_frame.size.height;
        in_param.dst_frame.img_fd.y = param_ptr->dst_frame.fd;
        in_param.dst_frame.img_fd.u = param_ptr->dst_frame.fd;
        in_param.dst_frame.img_addr_phy.chn0 =
            param_ptr->dst_frame.addr_phy.addr_y;
        in_param.dst_frame.img_addr_phy.chn1 =
            param_ptr->dst_frame.addr_phy.addr_u;
        in_param.dst_frame.img_addr_vir.chn0 =
            param_ptr->dst_frame.addr_vir.addr_y;
        in_param.dst_frame.img_addr_vir.chn1 =
            param_ptr->dst_frame.addr_vir.addr_u;
        in_param.dst_slice_height = param_ptr->dst_slice_height;

        in_param.dst2_frame.img_fmt = param_ptr->dst2_frame.fmt;
        in_param.dst2_frame.img_size.w = param_ptr->dst2_frame.size.width;
        in_param.dst2_frame.img_size.h = param_ptr->dst2_frame.size.height;
        in_param.dst2_frame.img_fd.y = param_ptr->dst2_frame.fd;
        in_param.dst2_frame.img_fd.u = param_ptr->dst2_frame.fd;
        in_param.dst2_frame.img_addr_phy.chn0 =
            param_ptr->dst2_frame.addr_phy.addr_y;
        in_param.dst2_frame.img_addr_phy.chn1 =
            param_ptr->dst2_frame.addr_phy.addr_u;
        in_param.dst2_frame.img_addr_vir.chn0 =
            param_ptr->dst2_frame.addr_vir.addr_y;
        in_param.dst2_frame.img_addr_vir.chn1 =
            param_ptr->dst2_frame.addr_vir.addr_u;
        in_param.dst2_slice_height = param_ptr->dst2_slice_height;
        struct sensor_mode_info *sensor_mode_info;
        cmr_u32 sn_mode = 0;
        ret = cmr_sensor_get_mode(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                  &sn_mode);
        // we think OEM has get sensor info and save it into sensor context,so
        // we get mode info from cxt
        sensor_mode_info = &cxt->sn_cxt.sensor_info.mode_info[sn_mode];
        SENSOR_MODE_FPS_T fps_info;
        ret = cmr_sensor_get_fps_info(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                      sn_mode, &fps_info);
        camera_copy_sensor_fps_info_to_isp(&in_param.sensor_fps, &fps_info);
        if (ret) {
            CMR_LOGE("get sensor fps info failed %ld", ret);
            goto exit;
        }
        in_param.resolution_info.sensor_size.w = sensor_mode_info->width;
        in_param.resolution_info.sensor_size.h = sensor_mode_info->height;
        in_param.resolution_info.line_time = sensor_mode_info->line_time;
        in_param.resolution_info.frame_line = sensor_mode_info->frame_line;
        in_param.resolution_info.size_index = sn_mode;
        in_param.resolution_info.crop.st_x = sensor_mode_info->trim_start_x;
        in_param.resolution_info.crop.st_y = sensor_mode_info->trim_start_y;
        in_param.resolution_info.crop.width = sensor_mode_info->trim_width;
        in_param.resolution_info.crop.height = sensor_mode_info->trim_height;
        in_param.resolution_info.fps.max_fps = in_param.sensor_fps.max_fps;
        in_param.resolution_info.fps.min_fps = in_param.sensor_fps.min_fps;
        in_param.resolution_info.sensor_max_size.w =
            cxt->sn_cxt.sensor_info.source_width_max;
        in_param.resolution_info.sensor_max_size.h =
            cxt->sn_cxt.sensor_info.source_height_max;
        in_param.resolution_info.sensor_output_size.w =
            sensor_mode_info->out_width;
        in_param.resolution_info.sensor_output_size.h =
            sensor_mode_info->out_height;
        CMR_LOGI("ips_in_param:sensor fps info: is_high_fps %d, "
                 "high_fps_skip_num is %d",
                 in_param.sensor_fps.is_high_fps,
                 in_param.sensor_fps.high_fps_skip_num);
        CMR_LOGI("ips_in_param:sensor fps info:	max_fps is %d, min_fps is %d",
                 in_param.sensor_fps.max_fps, in_param.sensor_fps.min_fps);
        CMR_LOGI("ips_in_param: sensor max w h, %d %d",
                 in_param.resolution_info.sensor_max_size.w,
                 in_param.resolution_info.sensor_max_size.h);
        CMR_LOGI("ips_in_param sensor output w h, %d %d",
                 in_param.resolution_info.sensor_output_size.w,
                 in_param.resolution_info.sensor_output_size.h);
        CMR_LOGI("ips_in_param: sensor crop startx start w h, %d %d %d %d",
                 in_param.resolution_info.crop.st_x,
                 in_param.resolution_info.crop.st_y,
                 in_param.resolution_info.crop.width,
                 in_param.resolution_info.crop.height);

        sns_ex_info_ptr = &cxt->sn_cxt.cur_sns_ex_info;
        if (NULL == sns_ex_info_ptr) {
            CMR_LOGE("sns_ex_info_ptr is null, It is impossible error!");
            ret = -CMR_CAMERA_INVALID_PARAM;
            goto exit;
        }
        if ((NULL == sns_ex_info_ptr->name) ||
            (NULL == sns_ex_info_ptr->sensor_version_info)) {
            ret = cmr_sensor_init_static_info(cxt);
            if (ret) {
                cmr_sensor_deinit_static_info(cxt);
                CMR_LOGE("init sensor static info failed %ld", ret);
                goto exit;
            }
        }
        in_param.resolution_info.max_gain = sns_ex_info_ptr->max_adgain;
        CMR_LOGI("ips_in_param:max_gain:%d ",
                 in_param.resolution_info.max_gain);

        {
            char value[PROPERTY_VALUE_MAX];
            property_get("debug.camera.save.snpfile", value, "0");
            if (atoi(value) == 1 || atoi(value) == 100 ||
                (atoi(value) & (1 << 1))) {
                camera_save_yuv_to_file(
                    FORM_DUMPINDEX(0x4000, cxt->dump_cnt, 0), IMG_DATA_TYPE_RAW,
                    param_ptr->src_frame.size.width,
                    param_ptr->src_frame.size.height,
                    &param_ptr->src_frame.addr_vir);
            }
        }

        ret = isp_proc_start(isp_cxt->isp_handle, &in_param, &out_param);

        if (ret) {
            CMR_LOGE("failed to start proc %ld", ret);
        }

        {
            char value[PROPERTY_VALUE_MAX];
            property_get("debug.camera.save.snpfile", value, "0");
            if (atoi(value) == 1 || atoi(value) == 100 ||
                (atoi(value) & (1 << 1))) {

                camera_save_yuv_to_file(
                    FORM_DUMPINDEX(0x6000, cxt->dump_cnt, 0),
                    IMG_DATA_TYPE_YUV420, param_ptr->dst_frame.size.width,
                    param_ptr->dst_frame.size.height,
                    &param_ptr->dst_frame.addr_vir);
            }
        }

    } else {
        struct ipn_in_param in_param;
        struct ips_out_param out_param;
        in_param.src_avail_height = param_ptr->src_avail_height;
        in_param.src_slice_height = param_ptr->src_slice_height;
        in_param.src_addr_phy.chn0 = param_ptr->src_frame.addr_phy.addr_y;
        in_param.src_addr_phy.chn1 = param_ptr->src_frame.addr_phy.addr_u;
        in_param.dst_addr_phy.chn0 = param_ptr->dst_frame.addr_phy.addr_y;
        in_param.dst_addr_phy.chn1 = param_ptr->dst_frame.addr_phy.addr_u;
        ret = isp_proc_next(isp_cxt->isp_handle, &in_param, &out_param);
        if (ret) {
            CMR_LOGE("failed to start proc %ld", ret);
        }
    }
    if (CMR_CAMERA_SUCCESS == ret) {
        cxt->isp_cxt.caller_handle = caller_handle;
        CMR_LOGD("caller handle 0x%lx", (cmr_uint)caller_handle);
    }
exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_isp_start_video(cmr_handle oem_handle,
                               struct video_start_param *param_ptr) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct isp_video_start isp_param;
    cmr_int work_mode = 0;
    cmr_int dv_mode = 0;
    cmr_int raw_buf_size;
    cmr_int highiso_buf_size;
    cmr_int tmp_buf_size;
    cmr_int num = 0;
    cmr_int highiso_buf_num = 0;
    cmr_int isp_raw_buf_num = 0;
    struct setting_cmd_parameter setting_param;
    struct sensor_ex_info *sns_ex_info_ptr;
    struct sensor_mode_info *sensor_mode_info = NULL;
    cmr_u32 sn_mode = 0;
    struct sensor_exp_info exp_info;

    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.cam.dual.preview", prop, "0");

    if (!param_ptr || !oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    cmr_bzero(&isp_param, sizeof(struct isp_video_start));
    cmr_bzero(&exp_info, sizeof(struct sensor_exp_info));

    if (cxt->is_multi_mode == MODE_SBS && strcmp(prop, "1")) {
        cxt->is_real_bokeh = 1;
        isp_param.size.w = 1600;
        isp_param.size.h = 1200;
    } else {
        isp_param.size.w = param_ptr->size.width;
        isp_param.size.h = param_ptr->size.height;
    }

    isp_param.format = ISP_DATA_NORMAL_RAW10;
    isp_param.mode = param_ptr->video_mode;

    if (cxt->lsc_malloc_flag == 0) {
        cmr_u32 lsc_buf_size = 0;
        cmr_u32 lsc_buf_num = 0;
        isp_param.lsc_buf_num = ISP_LSC_BUF_NUM;
        isp_param.lsc_buf_size = ISP_LSC_BUF_SIZE;
        lsc_buf_size = isp_param.lsc_buf_size;
        lsc_buf_num = isp_param.lsc_buf_num;

        if (cxt->hal_malloc == NULL) {
            ret = -1;
            CMR_LOGE("hal_malloc is null");
            goto exit;
        }

        cxt->hal_malloc(CAMERA_ISP_LSC, &lsc_buf_size, &lsc_buf_num,
                        &cxt->isp_lsc_phys_addr, &cxt->isp_lsc_virt_addr,
                        &cxt->lsc_mfd, cxt->client_data);
        cxt->lsc_malloc_flag = 1;
    }

    isp_param.lsc_phys_addr = cxt->isp_lsc_phys_addr;
    isp_param.lsc_virt_addr = cxt->isp_lsc_virt_addr;
    isp_param.lsc_mfd = cxt->lsc_mfd;
    isp_param.cb_of_malloc = cxt->hal_malloc;
    isp_param.cb_of_free = cxt->hal_free;
    isp_param.buffer_client_data = cxt->client_data;
    isp_param.oem_handle = oem_handle;
    isp_param.alloc_cb = camera_malloc;
    isp_param.free_cb = camera_free;

    if (0 == param_ptr->work_mode) {
        work_mode = param_ptr->work_mode;

        cmr_bzero(&setting_param, sizeof(setting_param));
        setting_param.camera_id = cxt->camera_id;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_DV_MODE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get dv mode %ld", ret);
            goto exit;
        }
        if (setting_param.cmd_type_value) {
            work_mode = 0;
        }
        dv_mode = setting_param.cmd_type_value;
    } else {
        work_mode = param_ptr->work_mode;
    }

    isp_param.work_mode = work_mode;
    isp_param.is_snapshot = param_ptr->is_snapshot;

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_CAPTURE_MODE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get dv mode %ld", ret);
        goto exit;
    }
    isp_param.capture_mode = setting_param.cmd_type_value;
    isp_param.dv_mode = dv_mode;

    if (cxt->is_multi_mode == MODE_SBS &&
        (cxt->camera_id == 0 || cxt->camera_id == 1)) {
        sn_mode = 2;
    } else {
        ret = cmr_sensor_get_mode(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                  &sn_mode);
        if (ret) {
            CMR_LOGE("cmr_sensor_get_mode failed");
            goto exit;
        }
    }

    ret = cmr_sensor_get_info(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                              &exp_info);
    if (ret) {
        CMR_LOGE("cmr_sensor_get_info failed ret=%ld", ret);
        goto exit;
    }
    sensor_mode_info = &exp_info.mode_info[sn_mode];

    if (cxt->is_multi_mode == MODE_SBS && strcmp(prop, "1")) {
        isp_param.resolution_info.sensor_size.w = 1600;
        isp_param.resolution_info.sensor_size.h = 1200;
        isp_param.resolution_info.crop.width =
            sensor_mode_info->scaler_trim.width;
        isp_param.resolution_info.crop.height =
            sensor_mode_info->scaler_trim.height;
        isp_param.resolution_info.sensor_output_size.w = 1600;
        isp_param.resolution_info.sensor_output_size.h = 1200;
    } else {
        isp_param.resolution_info.sensor_size.w = sensor_mode_info->width;
        isp_param.resolution_info.sensor_size.h = sensor_mode_info->height;
        isp_param.resolution_info.crop.width = sensor_mode_info->trim_width;
        isp_param.resolution_info.crop.height = sensor_mode_info->trim_height;
        isp_param.resolution_info.sensor_output_size.w =
            sensor_mode_info->out_width;
        isp_param.resolution_info.sensor_output_size.h =
            sensor_mode_info->out_height;
    }
    isp_param.resolution_info.line_time = sensor_mode_info->line_time;
    isp_param.resolution_info.frame_line = sensor_mode_info->frame_line;
    isp_param.resolution_info.size_index = sn_mode;
    isp_param.resolution_info.crop.st_x = sensor_mode_info->trim_start_x;
    isp_param.resolution_info.crop.st_y = sensor_mode_info->trim_start_y;
    isp_param.resolution_info.sensor_max_size.w =
        cxt->sn_cxt.sensor_info.source_width_max;
    isp_param.resolution_info.sensor_max_size.h =
        cxt->sn_cxt.sensor_info.source_height_max;
    isp_param.is_refocus = cxt->is_refocus_mode;

    if (isp_param.resolution_info.sensor_max_size.w ==
        isp_param.resolution_info.sensor_output_size.w) {
        isp_param.pdaf_enable = 1;
    }

    SENSOR_MODE_FPS_T fps_info;
    ret = cmr_sensor_get_fps_info(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                  sn_mode, &fps_info);
    if (ret) {
        CMR_LOGE("get sensor fps info failed %ld", ret);
        goto exit;
    }
    camera_copy_sensor_fps_info_to_isp(&isp_param.sensor_fps, &fps_info);

    isp_param.resolution_info.fps.max_fps = isp_param.sensor_fps.max_fps;
    isp_param.resolution_info.fps.min_fps = isp_param.sensor_fps.min_fps;

    sns_ex_info_ptr = &cxt->sn_cxt.cur_sns_ex_info;
    if (NULL == sns_ex_info_ptr) {
        CMR_LOGE("sns_ex_info_ptr is null, It is impossible error!");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if ((NULL == sns_ex_info_ptr->name) ||
        (NULL == sns_ex_info_ptr->sensor_version_info)) {
        ret = cmr_sensor_init_static_info(cxt);
        if (ret) {
            cmr_sensor_deinit_static_info(cxt);
            CMR_LOGE("init sensor static info failed %ld", ret);
            goto exit;
        }
    }

    isp_param.resolution_info.max_gain = sns_ex_info_ptr->max_adgain;

    ispmw_dev_buf_cfg_evt_cb(isp_cxt->isp_handle, camera_isp_dev_evt_cb);

    if (1 == cxt->isp_to_dram) {
        isp_param.capture_mode = ISP_CAP_MODE_DRAM;
    }

    if (cxt->is_real_bokeh) {
        CMR_LOGD("real bokeh");
        isp_param.is_real_bokeh = cxt->is_real_bokeh;
        // DEPTH
        if (cxt->swisp_depth_malloc_flag == 0) {
            cmr_u32 depth_size = 960 * 720 * 3 / 2;
            cmr_u32 depth_num = 1;
            if (cxt->hal_malloc) {
                cxt->hal_malloc(CAMERA_PREVIEW_DEPTH, &depth_size, &depth_num,
                                &cxt->swisp_depth_phys_addr,
                                &cxt->swisp_depth_virt_addr,
                                &cxt->swisp_depth_mfd, cxt->client_data);
                cxt->swisp_depth_malloc_flag = 1;
            } else {
                ret = -CMR_CAMERA_NO_MEM;
                CMR_LOGE("failed to malloc isp lsc buffer");
                goto exit;
            }
            isp_param.s_yuv_depth.img_size.w = 960;
            isp_param.s_yuv_depth.img_size.h = 720;
            isp_param.s_yuv_depth.img_fd.y = cxt->swisp_depth_mfd;
            isp_param.s_yuv_depth.img_addr_vir.chn0 =
                cxt->swisp_depth_virt_addr;
        }

        // SW_OUT
        if (cxt->swisp_out_malloc_flag == 0) {
            cmr_u32 sw_out_size = 960 * 720 * 3 / 2;
            cmr_u32 sw_out_num = 1;
            if (cxt->hal_malloc) {
                cxt->hal_malloc(CAMERA_PREVIEW_SW_OUT, &sw_out_size,
                                &sw_out_num, &cxt->swisp_out_phys_addr,
                                &cxt->swisp_out_virt_addr, &cxt->swisp_out_mfd,
                                cxt->client_data);
                cxt->swisp_out_malloc_flag = 1;
            } else {
                ret = -CMR_CAMERA_NO_MEM;
                CMR_LOGE("failed to malloc isp lsc buffer");
                goto exit;
            }
            isp_param.s_yuv_sw_out.img_size.w = 800;
            isp_param.s_yuv_sw_out.img_size.h = 600;
            isp_param.s_yuv_sw_out.img_fd.y = cxt->swisp_out_mfd;
            isp_param.s_yuv_sw_out.img_addr_vir.chn0 = cxt->swisp_out_virt_addr;
        }
    }

    CMR_LOGI(
        "sensor_max_size: w=%d, h=%d, sensor_output_size: w=%d,h=%d, "
        "sn_mode=%d, crop: st_x=%d, st_y=%d, w=%d, h=%d, max_gain=%d, "
        "sensor_fps: is_high_fps=%d, high_fps_skip_num=%d, max_fps=%d, "
        "min_fps=%d, isp_param.size: w=%d, h=%d, lsc_phys_addr=0x%lx, "
        "lsc_virt_addr=0x%lx, work_mode=%d, dv_mode=%d, capture_mode=%d",
        isp_param.resolution_info.sensor_max_size.w,
        isp_param.resolution_info.sensor_max_size.h,
        isp_param.resolution_info.sensor_output_size.w,
        isp_param.resolution_info.sensor_output_size.h, sn_mode,
        isp_param.resolution_info.crop.st_x,
        isp_param.resolution_info.crop.st_y,
        isp_param.resolution_info.crop.width,
        isp_param.resolution_info.crop.height,
        isp_param.resolution_info.max_gain, isp_param.sensor_fps.is_high_fps,
        isp_param.sensor_fps.high_fps_skip_num, isp_param.sensor_fps.max_fps,
        isp_param.sensor_fps.min_fps, isp_param.size.w, isp_param.size.h,
        isp_param.lsc_phys_addr, isp_param.lsc_virt_addr, isp_param.work_mode,
        isp_param.dv_mode, isp_param.capture_mode);
    isp_param.is_restart = param_ptr->is_restart;

    ret = isp_video_start(isp_cxt->isp_handle, &isp_param);
    if (ret) {
        isp_cxt->is_work = 0;
        CMR_LOGE("failed to start isp, ret %ld", ret);
        goto exit;
    }
    isp_cxt->is_work = 1;

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_isp_stop_video(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (cxt->isp_cxt.is_work) {
        ret = isp_video_stop(isp_cxt->isp_handle);
        if (ret) {
            CMR_LOGE("failed to stop isp %ld", ret);
        } else {
            cxt->isp_cxt.is_work = 0;
        }
    }
exit:
    CMR_LOGI("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_channel_cfg(cmr_handle oem_handle, cmr_handle caller_handle,
                           cmr_u32 camera_id,
                           struct channel_start_param *param_ptr,
                           cmr_u32 *channel_id, struct img_data_end *endian) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    cmr_uint i;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_context *sn_cxt = &cxt->sn_cxt;
    struct sensor_exp_info sensor_info;
    struct sensor_mode_info *sensor_mode_info, *tmp;
    struct sn_cfg sensor_cfg;
    char prop[PROPERTY_VALUE_MAX] = {
        0,
    };
    property_get("persist.vendor.cam.dual.preview", prop, "0");

    if (!oem_handle || !caller_handle || !param_ptr || !channel_id || !endian) {
        CMR_LOGE("in parm error 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx",
                 (cmr_uint)oem_handle, (cmr_uint)caller_handle,
                 (cmr_uint)param_ptr, (cmr_uint)channel_id, (cmr_uint)endian);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    cmr_bzero(&sensor_cfg, sizeof(struct sn_cfg));
    param_ptr->cap_inf_cfg.buffer_cfg_isp = 0;
    CMR_LOGV("param_ptr->cap_inf_cfg.buffer_cfg_isp %d",
             param_ptr->cap_inf_cfg.buffer_cfg_isp);
    if (param_ptr->is_lightly) {
        CMR_LOGD("channel id %d, caller_handle 0x%lx, skip num %d", *channel_id,
                 (cmr_uint)caller_handle, param_ptr->skip_num);
        if (*channel_id >= CHN_MAX)
        {
            CMR_LOGE("invalid chanel_id: %d", *channel_id);
            ret = -CMR_CAMERA_INVALID_PARAM;
            goto exit;
        }

        ret = cmr_grab_cap_cfg_lightly(cxt->grab_cxt.grab_handle,
                                       &param_ptr->cap_inf_cfg, *channel_id);
        if (ret) {
            CMR_LOGE("failed to cap cfg %ld", ret);
            goto exit;
        }
        goto exit;
    }

    // cmr_grab_set_zoom_mode(cxt->grab_cxt.grab_handle, 1);

    ret = cmr_sensor_get_info(sn_cxt->sensor_handle, camera_id, &sensor_info);
    if (ret) {
        CMR_LOGE("failed to get sensor info %ld", ret);
        goto exit;
    }

    CMR_LOGD("frm_num 0x%x", param_ptr->frm_num);

    sensor_mode_info = &sensor_info.mode_info[param_ptr->sensor_mode];
    sensor_cfg.sn_size.width = sensor_mode_info->width;
    sensor_cfg.sn_size.height = sensor_mode_info->height;
    sensor_cfg.frm_num = param_ptr->frm_num;
    sensor_cfg.sn_trim.start_x = sensor_mode_info->trim_start_x;
    sensor_cfg.sn_trim.start_y = sensor_mode_info->trim_start_y;
    if (cxt->is_multi_mode == MODE_SBS && strcmp(prop, "1")) {
        sensor_cfg.sn_trim.width = sensor_mode_info->scaler_trim.width;
        sensor_cfg.sn_trim.height = sensor_mode_info->scaler_trim.height;
    } else {
        sensor_cfg.sn_trim.width = sensor_mode_info->trim_width;
        sensor_cfg.sn_trim.height = sensor_mode_info->trim_height;
    }
    CMR_LOGD("3D, sn_size: width=%d, height=%d", sensor_cfg.sn_size.width,
             sensor_cfg.sn_size.height);
    CMR_LOGD("3D, sn_trim: width=%d, height=%d", sensor_cfg.sn_trim.width,
             sensor_cfg.sn_trim.height);
    CMR_LOGD("3D, scaler_trim: width=%d, height=%d",
             sensor_mode_info->scaler_trim.width,
             sensor_mode_info->scaler_trim.height);
    param_ptr->sn_if.sensor_width = sensor_mode_info->width;
    param_ptr->sn_if.sensor_height = sensor_mode_info->height;
    param_ptr->sn_if.if_spec.mipi.pclk = sensor_mode_info->bps_per_lane;
    if (cxt->isp_to_dram)
        param_ptr->sn_if.res[0] = 1;
    else
        param_ptr->sn_if.res[0] = 0;

    sensor_cfg.sensor_max_size.width = sensor_info.source_width_max;
    sensor_cfg.sensor_max_size.height = sensor_info.source_height_max;
    CMR_LOGD("sensor_max_size: width=%d, height=%d",
             sensor_cfg.sensor_max_size.width,
             sensor_cfg.sensor_max_size.height);

    ret = cmr_grab_if_cfg(cxt->grab_cxt.grab_handle, &param_ptr->sn_if);
    if (ret) {
        CMR_LOGE("failed interface cfg %ld", ret);
        goto exit;
    }

    if (cxt->is_multi_mode == MODE_SBS) {
        sensor_cfg.sbs_mode = SPRD_SBS_MODE_ON;
    }

    ret = cmr_grab_sn_cfg(cxt->grab_cxt.grab_handle, &sensor_cfg);
    if (ret) {
        CMR_LOGE("failed to sn cfg %ld", ret);
        goto exit;
    }

    if (caller_handle == cxt->prev_cxt.preview_handle) {
        cxt->prev_cxt.rect = param_ptr->cap_inf_cfg.cfg.src_img_rect;
        CMR_LOGD("prev rect %d %d %d %d", cxt->prev_cxt.rect.start_x,
                 cxt->prev_cxt.rect.start_y, cxt->prev_cxt.rect.width,
                 cxt->prev_cxt.rect.height);
    }

    param_ptr->cap_inf_cfg.sensor_id = camera_id;
    param_ptr->cap_inf_cfg.cfg.need_3dnr =
        (2 == camera_get_3dnr_flag(cxt)) ? 1 : 0;

    ret = cmr_grab_cap_cfg(cxt->grab_cxt.grab_handle,
                           &param_ptr->cap_inf_cfg, channel_id, endian);
    if (ret) {
        CMR_LOGE("failed to cap cfg %ld", ret);
        goto exit;
    }

    cxt->grab_cxt.caller_handle[*channel_id] = caller_handle;
    cxt->grab_cxt.skip_number[*channel_id] = param_ptr->skip_num;
    CMR_LOGD("channel id %d, caller_handle 0x%lx, skip num %d", *channel_id,
             (cmr_uint)caller_handle, param_ptr->skip_num);
exit:
    CMR_LOGV("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_channel_buff_cfg(cmr_handle oem_handle,
                                struct buffer_cfg *buf_cfg) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    if (buf_cfg->channel_id >= CHN_MAX)
    {
        CMR_LOGE("invalid chanel_id: %d", buf_cfg->channel_id);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    ret = cmr_grab_buff_cfg(cxt->grab_cxt.grab_handle, buf_cfg);
    if (ret) {
        CMR_LOGE("failed to buf cfg %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGV("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_channel_cap_cfg(cmr_handle oem_handle, cmr_handle caller_handle,
                               cmr_u32 camera_id, struct cap_cfg *cap_cfg,
                               cmr_u32 *channel_id,
                               struct img_data_end *endian) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    if (*channel_id >= CHN_MAX)
    {
        CMR_LOGE("invalid chanel_id: %d", *channel_id);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    cap_cfg->buffer_cfg_isp = 0;
    cap_cfg->sensor_id = camera_id;
    ret = cmr_grab_cap_cfg(cxt->grab_cxt.grab_handle, cap_cfg, channel_id,
                           endian);
    if (ret) {
        CMR_LOGE("failed to buf cfg %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_get_sg(cmr_handle oem_handle, struct sprd_img_iova *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_grab_get_sg(cxt->grab_cxt.grab_handle, param);

    if (ret) {
        CMR_LOGE("buf get sg failed fd 0x%x size 0x%x.", param->fd,
                 param->size);
    }

exit:
    return ret;
}

cmr_int camera_map_iommu(cmr_handle oem_handle, struct sprd_img_iova *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_grab_map_iommu(cxt->grab_cxt.grab_handle, param);

    if (ret) {
        CMR_LOGE("buf map failed fd 0x%x size 0x%x.", param->fd, param->size);
    }

exit:
    return ret;
}

cmr_int camera_unmap_iommu(cmr_handle oem_handle, struct sprd_img_iova *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_grab_unmap_iommu(cxt->grab_cxt.grab_handle, param);

    if (ret) {
        CMR_LOGE("buf unmap failed fd 0x%x size 0x%x.", param->fd, param->size);
    }

exit:
    return ret;
}

cmr_int camera_isp_buff_cfg(cmr_handle oem_handle, struct buffer_cfg *buf_cfg) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct isp_img_param isp_buf_cfg;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    int i;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    if (ret) {
        CMR_LOGE("failed to buf cfg %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGD("done %ld", ret);
    return ret;
}

cmr_int camera_channel_start(cmr_handle oem_handle, cmr_u32 channel_bits,
                             cmr_uint skip_number) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    cmr_uint is_zsl_enable = 0;
    cmr_uint video_snapshot_type = VIDEO_SNAPSHOT_NONE;
    struct sprd_img_capture_param capture_param;
    struct timespec ts;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGI("skip num %ld %d", skip_number, channel_bits);
    camera_take_snapshot_step(CMR_STEP_CAP_S);

    if (clock_gettime(CLOCK_REALTIME, &ts)) {
        CMR_LOGE("get time failed");
        goto exit;
    }
    ts.tv_nsec += ms2ns(600);
    if (ts.tv_nsec > 1000000000) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
    }
    if (cxt->isp_cxt.inited != 1) {
        CMR_LOGI(" wait for isp inited");
        ret = sem_timedwait(&cxt->isp_sm, &ts);
        CMR_LOGI("ret %ld", ret);
    }

    ret = cmr_grab_cap_start(cxt->grab_cxt.grab_handle, skip_number);
    if (ret) {
        CMR_LOGE("failed to start cap %ld", ret);
        goto exit;
    }

    if (cxt->is_start_snapshot != 1) {
        cmr_bzero(&setting_param, sizeof(setting_param));
        setting_param.camera_id = cxt->camera_id;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_SPRD_ZSL_ENABLED, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get preview sprd zsl enabled flag %ld", ret);
            goto exit;
        }
        is_zsl_enable = setting_param.cmd_type_value;

        cmr_bzero(&setting_param, sizeof(setting_param));
        ret =
            cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                              SETTING_GET_VIDEO_SNAPSHOT_TYPE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get preview sprd eis enabled flag %ld", ret);
            goto exit;
        }
        video_snapshot_type = setting_param.cmd_type_value;

/* for sharkl2 offline path */
#ifdef SHARKL2_DCAM_ISP
        if ((channel_bits & OFFLINE_CHANNEL_BIT) && is_zsl_enable == 0 &&
            video_snapshot_type != VIDEO_SNAPSHOT_VIDEO) {
            cmr_bzero(&capture_param, sizeof(capture_param));
            capture_param.type = 1;
            ret = cmr_grab_start_capture(cxt->grab_cxt.grab_handle,
                                         capture_param);
            if (ret) {
                CMR_LOGE("failed to start off the fly path,ret %ld", ret);
            }
        }
#endif
    }
exit:
    CMR_LOGV("done %ld", ret);
    cxt->is_start_snapshot = 0;
    return ret;
}

cmr_int camera_channel_pause(cmr_handle oem_handle, cmr_uint channel_id,
                             cmr_u32 reconfig_flag) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return ret;
    }
    if (channel_id >= CHN_MAX)
    {
        CMR_LOGE("invalid chanel_id: %d", channel_id);
        ret = -CMR_CAMERA_INVALID_PARAM;
        return ret;
    }

    CMR_LOGI("channel id %ld, reconfig flag %d", channel_id, reconfig_flag);

    /* for sharkl2 offline path */
    if (channel_id == OFFLINE_CHANNEL) {
        ret = cmr_grab_stop_capture(cxt->grab_cxt.grab_handle);
        if (ret) {
            CMR_LOGE("failed to stop off the fly path %ld", ret);
        }
    }

    ret = cmr_grab_cap_pause(cxt->grab_cxt.grab_handle, channel_id,
                             reconfig_flag);
    if (ret) {
        CMR_LOGE("failed to pause channel %ld", ret);
        goto exit;
    }
exit:
    if (TAKE_PICTURE_NEEDED == camera_get_snp_req((cmr_handle)cxt) ||
        (1 == camera_get_share_path_sm_flag(oem_handle))) {
        camera_post_share_path_available(oem_handle);
    }
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_channel_resume(cmr_handle oem_handle, cmr_uint channel_id,
                              cmr_u32 skip_number, cmr_u32 deci_factor,
                              cmr_u32 frm_num) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_sn_cmd_param param;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    if (channel_id >= CHN_MAX)
    {
        CMR_LOGE("invalid chanel_id: %d", channel_id);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGI("channel id %ld, skip num %d, deci %d, frm num %d", channel_id,
             skip_number, deci_factor, frm_num);
    camera_set_discard_frame(cxt, 0);
    camera_set_vendor_hdr_ev(oem_handle);

    /* for sharkl2 offline path */
    if (channel_id == OFFLINE_CHANNEL) {
        camera_local_start_capture(oem_handle);
    }

    ret = cmr_grab_cap_resume(cxt->grab_cxt.grab_handle, channel_id,
                              skip_number, deci_factor, frm_num);
    if (ret) {
        CMR_LOGE("failed to resume channel,ret %ld", ret);
        goto exit;
    }

exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_channel_free_frame(cmr_handle oem_handle, cmr_u32 channel_id,
                                  cmr_u32 index) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    if (channel_id >= CHN_MAX)
    {
        CMR_LOGE("invalid chanel_id: %d", channel_id);
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    ret = cmr_grab_free_frame(cxt->grab_cxt.grab_handle, channel_id, index);
    if (ret) {
        CMR_LOGE("failed to free frame %d %ld", channel_id, ret);
        goto exit;
    }
exit:
    return ret;
}

cmr_int camera_channel_stop(cmr_handle oem_handle, cmr_u32 channel_bits) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    (cmr_u32) channel_bits;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    /* for sharkl2 offline path */
    if (channel_bits & OFFLINE_CHANNEL_BIT) {
        ret = cmr_grab_stop_capture(cxt->grab_cxt.grab_handle);
        if (ret) {
            CMR_LOGE("failed to stop off the fly path %ld", ret);
        }
    }

    ret = cmr_grab_cap_stop(cxt->grab_cxt.grab_handle);
    if (ret) {
        CMR_LOGE("failed to stop channel %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_channel_scale_capability(cmr_handle oem_handle, cmr_u32 *width,
                                        cmr_u32 *sc_factor,
                                        cmr_u32 *sc_threshold) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !width | !sc_factor) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_grab_scale_capability(cxt->grab_cxt.grab_handle, width, sc_factor,
                                    sc_threshold);
exit:
    return ret;
}

cmr_int camera_channel_path_capability(cmr_handle oem_handle,
                                       struct cmr_path_capability *capability) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle || !capability) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_grab_path_capability(cxt->grab_cxt.grab_handle, capability);
exit:
    return ret;
}
cmr_int camera_channel_get_cap_time(cmr_handle oem_handle, cmr_u32 *sec,
                                    cmr_u32 *usec) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !sec | !usec) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_grab_get_cap_time(cxt->grab_cxt.grab_handle, sec, usec);
exit:
    return ret;
}

cmr_int camera_get_sensor_info(cmr_handle oem_handle, cmr_uint sensor_id,
                               struct sensor_exp_info *exp_info_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !exp_info_ptr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    ret =
        cmr_sensor_get_info(cxt->sn_cxt.sensor_handle, sensor_id, exp_info_ptr);
    if (ret) {
        CMR_LOGE("failed to get sensor info %ld", ret);
        goto exit;
    }
exit:
    return ret;
}

cmr_int camera_get_sensor_fps_info(cmr_handle oem_handle, cmr_uint sensor_id,
                                   cmr_u32 sn_mode,
                                   struct sensor_mode_fps_tag *fps_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !fps_info) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    ret = cmr_sensor_get_fps_info(cxt->sn_cxt.sensor_handle, sensor_id, sn_mode,
                                  fps_info);
    if (ret) {
        CMR_LOGE("failed to get sensor info %ld", ret);
        goto exit;
    }
exit:
    return ret;
}

cmr_int
camera_get_tuning_info(cmr_handle oem_handle,
                       struct isp_adgain_exp_info *adgain_exp_info_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct common_isp_cmd_param isp_param;

    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_ADGAIN_EXP, &isp_param);
    adgain_exp_info_ptr->adgain = isp_param.isp_adgain.adgain;
    adgain_exp_info_ptr->exp_time = isp_param.isp_adgain.exp_time;
    adgain_exp_info_ptr->bv = isp_param.isp_adgain.bv;
    CMR_LOGV("adgain=%d, exp_time=%d, bv=%d", adgain_exp_info_ptr->adgain,
             adgain_exp_info_ptr->exp_time, adgain_exp_info_ptr->bv);
    return ret;
}

cmr_int camera_get_sensor_autotest_mode(cmr_handle oem_handle,
                                        cmr_uint sensor_id,
                                        cmr_uint *is_autotest) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !is_autotest) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return ret;
    }
    CMR_LOGI("sensor id %ld", sensor_id);
    ret = cmr_sensor_get_autotest_mode(cxt->sn_cxt.sensor_handle, sensor_id,
                                       is_autotest);
    if (ret) {
        CMR_LOGE("failed to get sensor info %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGV("ret %ld, is_autotest %ld", ret, *is_autotest);
    return ret;
}

cmr_int cmr_get_leds_ctrl(cmr_handle oem_handle, cmr_u32 *led0_enable,
                          cmr_u32 *led1_enable) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct common_isp_cmd_param isp_param;

    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_LEDS_CTRL, &isp_param);
    if (ret) {
        CMR_LOGE("failed to get leds_ctrl %ld", ret);
        goto exit;
    }

    *led0_enable = isp_param.leds_ctrl.led0_ctrl;
    *led1_enable = isp_param.leds_ctrl.led1_ctrl;

exit:
    return ret;
}

cmr_int camera_ioctl_for_setting(cmr_handle oem_handle, cmr_uint cmd_type,
                                 struct setting_io_parameter *param_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_handle grab_handle;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    grab_handle = cxt->grab_cxt.grab_handle;

    switch (cmd_type) {
    case SETTING_IO_GET_CAPTURE_SIZE:
        param_ptr->size_param = cxt->snp_cxt.post_proc_setting.snp_size;
        break;
    case SETTING_IO_GET_ACTUAL_CAPTURE_SIZE:
        // param_ptr->size_param =
        // cxt->snp_cxt.post_proc_setting.actual_snp_size;
        param_ptr->size_param =
            cxt->snp_cxt.post_proc_setting.dealign_actual_snp_size;
        break;
    case SETTING_IO_CTRL_FLASH: {
        struct grab_flash_opt flash_opt;

        if (FLASH_OPEN == param_ptr->cmd_value ||
            FLASH_HIGH_LIGHT == param_ptr->cmd_value ||
            FLASH_TORCH == param_ptr->cmd_value) {
            cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                SENSOR_EXIF_CTRL_FLASH, 1);
        }
        char value1[PROPERTY_VALUE_MAX];
        property_get("persist.vendor.cam.flash.torch.cfg", value1, "06");

        /*cfg torch value*/
        if (param_ptr->cmd_value == FLASH_TORCH) {
            struct sprd_flash_cfg_param cfg;
            cfg.real_cell.type = FLASH_TYPE_TORCH;
            cfg.real_cell.count = 1;
            cfg.real_cell.led_idx = 0x01;                  /*main*/
            cfg.real_cell.element[0].index = atoi(value1); // 0x06;
            cfg.real_cell.element[0].val = 0;
            cfg.io_id = FLASH_IOID_SET_CHARGE;
            cfg.flash_idx = cxt->facing % 2;
            if (!camera_front_lcd_flash_activie(cfg.flash_idx))
                ret = cmr_grab_cfg_flash(grab_handle, &cfg);
        }

        if ((param_ptr->cmd_value == FLASH_OPEN ||
             param_ptr->cmd_value == FLASH_HIGH_LIGHT) &&
            cxt->camera_id == 0) {
            cmr_get_leds_ctrl(oem_handle, &flash_opt.led0_enable,
                              &flash_opt.led1_enable);
        } else {
            flash_opt.led0_enable = 1;
            flash_opt.led1_enable = 1;
        }

        flash_opt.flash_mode = param_ptr->cmd_value;
        flash_opt.flash_index = cxt->facing % 2;
        CMR_LOGV("led0_enable=%d, led1_enable=%d", flash_opt.led0_enable,
                 flash_opt.led1_enable);
        if (camera_front_lcd_flash_activie(flash_opt.flash_index)) {
            camera_front_lcd_flash_callback(cxt, flash_opt.flash_mode);
        } else {
            cmr_grab_flash_cb(grab_handle, &flash_opt);
        }
    } break;
    case SETTING_IO_GET_PREVIEW_MODE:
        param_ptr->cmd_value = cxt->prev_cxt.preview_sn_mode;
        break;
    case SETTING_IO_GET_FLASH_MAX_CAPACITY: {
#if 0

		struct sprd_flash_capacity capacity;
		struct sprd_flash_cfg_param cfg;

		cfg.io_id = FLASH_IOID_GET_MAX_CAPACITY;
		cfg.data = &capacity;
		ret = cmr_grab_cfg_flash(cxt->grab_cxt.grab_handle, &cfg);
		if (0 == ret) {
			param_ptr->flash_capacity.max_charge = capacity.max_charge;
			param_ptr->flash_capacity.max_time = capacity.max_time;
		}
#endif
    }
    case SETTING_IO_SET_TOUCH: {
        prev_set_preview_touch_info(cxt->prev_cxt.preview_handle,
                                    cxt->camera_id, &param_ptr->touch_xy);
    } break;

    default:
        CMR_LOGE("don't support cmd %ld", cmd_type);
        ret = CMR_CAMERA_NO_SUPPORT;
        break;
    }
exit:
    return ret;
}

cmr_int camera_sensor_ioctl(cmr_handle oem_handle, cmr_uint cmd_type,
                            struct common_sn_cmd_param *param_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    void *isp_param_ptr = NULL;
    cmr_u32 ptr_flag = 0;
    cmr_u32 isp_cmd = ISP_CTRL_MAX;
    cmr_uint cmd = SENSOR_CMD_MAX;
    cmr_uint sensor_param = 0;
    cmr_uint set_exif_flag = 0;
    SENSOR_EXIF_CTRL_E exif_cmd;
    SENSOR_EXT_FUN_PARAM_T_PTR hdr_ev_param_ptr = 0;

    if (!oem_handle || !param_ptr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGV("cmd_type =%ld", cmd_type);

    switch (cmd_type) {
    case COM_ISP_GET_AE_FPS_RANGE:
        isp_cmd = ISP_CTRL_GET_AE_FPS_RANGE;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->ae_fps_range;
        break;
    case COM_SN_GET_AUTO_FLASH_STATE:
        cmd = SENSOR_CHECK_NEED_FLASH;
        sensor_param = (cmr_uint)&param_ptr->cmd_value;
        break;
    case COM_SN_SET_BRIGHTNESS:
        cmd = SENSOR_BRIGHTNESS;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_BRIGHTNESSVALUE;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_CONTRAST:
        cmd = SENSOR_CONTRAST;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_CONTRAST;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_SATURATION:
        cmd = SENSOR_SATURATION;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_SATURATION;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_SHARPNESS:
        cmd = SENSOR_SHARPNESS;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_SHARPNESS;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_IMAGE_EFFECT:
        cmd = SENSOR_IMAGE_EFFECT;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_EXPOSURE_COMPENSATION:
        cmd = SENSOR_EXPOSURE_COMPENSATION;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_EXPOSURE_LEVEL:
        cmd = SENSOR_WRITE_EV;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_WB_MODE:
        cmd = SENSOR_SET_WB_MODE;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_LIGHTSOURCE;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_PREVIEW_MODE:
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_SCENECAPTURETYPE;
        break;
    case COM_SN_SET_ANTI_BANDING:
        cmd = SENSOR_FLIP;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_ISO:
        cmd = SENSOR_ISO;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_VIDEO_MODE:
        cmd = SENSOR_VIDEO_MODE;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_FPS_LLS_MODE:
        break;
    case COM_SN_SET_BEFORE_SNAPSHOT:
        cmd = SENSOR_BEFORE_SNAPSHOT;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_AFTER_SNAPSHOT:
        cmd = SENSOR_AFTER_SNAPSHOT;
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_EXT_FUNC:
        break;
    case COM_SN_SET_AE_ENABLE: // don't support
        break;
    case COM_SN_SET_EXIF_FOCUS:
        sensor_param = param_ptr->cmd_value;
        break;
    case COM_SN_SET_FOCUS:
        cmd = SENSOR_FOCUS;
        sensor_param = (cmr_uint) & (param_ptr->yuv_sn_af_param);
        CMR_LOGE("COM_SN_SET_FOCUS cmd =%ld ", cmd);
        break;
    case COM_SN_GET_PREVIEW_MODE:
        break;
    case COM_SN_GET_CAPTURE_MODE:
        break;
    case COM_SN_GET_SENSOR_ID:
        break;
    case COM_SN_GET_VIDEO_MODE:
        break;
    case COM_SN_GET_EXIF_IMAGE_INFO:
        ret = cmr_sensor_get_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                  &param_ptr->exif_pic_info);
        if (ret) {
            CMR_LOGE("sn get exif image info failed!");
        } else {
            memcpy((void *)&cxt->sn_cxt.exif_info,
                   (void *)&param_ptr->exif_pic_info,
                   sizeof(EXIF_SPEC_PIC_TAKING_COND_T));
        }
        return ret;
    case COM_SN_SET_HDR_EV: {
        cmd = SENSOR_SET_HDR_EV;
        hdr_ev_param_ptr = malloc(sizeof(SENSOR_EXT_FUN_PARAM_T));
        if (!hdr_ev_param_ptr) {
            CMR_LOGE("fail to malloc");
            return CMR_CAMERA_NO_MEM;
        }
        hdr_ev_param_ptr->cmd = SENSOR_EXT_EV;
        hdr_ev_param_ptr->param = param_ptr->cmd_value;
        sensor_param = (cmr_uint)hdr_ev_param_ptr;
        CMR_LOGI("vendor hdr ev_val = %d", param_ptr->cmd_value);
        break;
    }
    case COM_SN_GET_INFO:
        ret = cmr_sensor_get_info(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                  &param_ptr->sensor_static_info);
        if (ret) {
            CMR_LOGE("sn get info failed!");
        }
        return ret;
    case COM_SN_GET_FLASH_LEVEL:
        ret = cmr_sensor_get_flash_info(
            cxt->sn_cxt.sensor_handle, cxt->camera_id, &param_ptr->flash_level);
        if (ret) {
            CMR_LOGE("sn get flash level failed!");
        }
        CMR_LOGI("flash level low_light = %d, high_light = %d",
                 param_ptr->flash_level.low_light,
                 param_ptr->flash_level.high_light);
        return ret;
    default:
        CMR_LOGE("don't support cmd %ld", cmd_type);
        ret = CMR_CAMERA_NO_SUPPORT;
        break;
    }
    if (!ret) {
        ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id, cmd,
                               sensor_param);
        if ((COM_SN_SET_HDR_EV == cmd_type) && hdr_ev_param_ptr) {
            free(hdr_ev_param_ptr);
            hdr_ev_param_ptr = 0;
        }
        if (ret) {
            CMR_LOGE("failed to sn ioctrl %ld", ret);
        }
        if (set_exif_flag) {
            CMR_LOGV("ERIC set exif");
            if (cmd_type == COM_SN_SET_WB_MODE) {
                cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                    exif_cmd, param_ptr->cmd_value);
                cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                    SENSOR_EXIF_CTRL_WHITEBALANCE,
                                    param_ptr->cmd_value);
            } else
                cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                    exif_cmd, sensor_param);
        }
    }
exit:
    return ret;
}

cmr_int cmr_get_ae_fps_range(cmr_handle oem_handle, cmr_u32 camera_id,
                          struct ae_fps_range_info *ae_fps_range) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
    isp_param.camera_id = cxt->camera_id;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_AE_FPS_RANGE,
                           &isp_param);
    if (ret) {
        CMR_LOGE("get isp vcm range error %ld", ret);
        goto exit;
    }

    memcpy(ae_fps_range, &isp_param.ae_fps_range, sizeof(struct ae_fps_range_info));
    CMR_LOGD("AE_FPS_RANGE_INFO in DV mode:isp_param.range [%d, %d]", ae_fps_range->dv_fps_min,
             ae_fps_range->dv_fps_max);

exit:
    return ret;
}

cmr_uint camera_param_to_isp(cmr_uint cmd, struct common_isp_cmd_param *parm) {
    cmr_uint in_param = parm->cmd_value;
    cmr_uint out_param = in_param;

    switch (cmd) {
    case COM_ISP_SET_AWB_MODE: {
        switch (in_param) {
        case CAMERA_WB_AUTO:
            out_param = ISP_AWB_AUTO;
            break;

        case CAMERA_WB_INCANDESCENT:
            out_param = ISP_AWB_INDEX1;
            break;

        case CAMERA_WB_FLUORESCENT:
            out_param = ISP_AWB_INDEX4;
            break;

        case CAMERA_WB_DAYLIGHT:
            out_param = ISP_AWB_INDEX5;
            break;

        case CAMERA_WB_CLOUDY_DAYLIGHT:
            out_param = ISP_AWB_INDEX6;
            break;

        default:
            break;
        }
        break;
    }
    case COM_ISP_SET_EV: {
        switch (in_param) {
        case 0:
            out_param = 1;
            break;

        case 1:
            out_param = 2;
            break;

        case 2:
            out_param = 3;
            break;

        case 3:
            out_param = 4;
            break;

        case 4:
            out_param = 5;
            break;

        case 5:
            out_param = 6;
            break;

        case 6:
            out_param = 7;
            break;

        default:
            break;
        }
        break;
    }
    default:
        break;
    }

    return out_param;
}

cmr_int camera_isp_ev_switch(struct common_isp_cmd_param *parm) {
    cmr_int in_param = parm->ae_compensation_param.ae_exposure_compensation;
    cmr_int out_param = in_param;
    switch (in_param) {
    case -6:
        out_param = 0;
        break;

    case -4:
        out_param = 1;
        break;

    case -2:
        out_param = 2;
        break;

    case 0:
        out_param = 3;
        break;

    case 2:
        out_param = 4;
        break;

    case 4:
        out_param = 5;
        break;

    case 6:
        out_param = 6;
        break;

    default:
        break;
    }
    return out_param;
}

cmr_int camera_local_get_isp_info(cmr_handle oem_handle, void **addr,
                                  int *size) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct isp_info isp_info;

    if (!oem_handle || !addr || !size) {
        CMR_LOGE("err,invlid param");
        return CMR_CAMERA_INVALID_PARAM;
    }
    *addr = 0;
    *size = 0;
    cmr_bzero(&isp_info, sizeof(isp_info));
    if (IMG_DATA_TYPE_RAW == cxt->sn_cxt.sensor_info.image_format) {
        ret = isp_ioctl(isp_cxt->isp_handle, ISP_CTRL_GET_INFO,
                        (void *)&isp_info);
        if (!ret) {
            *addr = isp_info.addr;
            *size = isp_info.size;
        } else {
            CMR_LOGE("fail to get isp information");
        }
    }
    CMR_LOGI("%p %d", *addr, *size);
    return ret;
}

cmr_int camera_isp_ioctl(cmr_handle oem_handle, cmr_uint cmd_type,
                         struct common_isp_cmd_param *param_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_u32 isp_cmd = ISP_CTRL_MAX;
    cmr_u32 isp_param = 0;
    void *isp_param_ptr = NULL;
    cmr_u32 ptr_flag = 0;
    cmr_uint set_exif_flag = 0;
    cmr_uint set_isp_flag = 1;
    SENSOR_EXIF_CTRL_E exif_cmd = SENSOR_EXIF_CTRL_MAX;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct isp_pos_rect trim;
    struct isp_range_fps isp_fps;
    struct isp_ae_fps ae_fps;
    struct isp_hdr_param hdr_param;
    struct isp_3dnr_ctrl_param param_3dnr;
    struct isp_exp_compensation ae_compensation;
    if (!oem_handle || !param_ptr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    switch (cmd_type) {
    case COM_ISP_SET_AE_MODE:
        if (cxt->is_lls_enable) {
            isp_cmd = ISP_CTRL_SET_AE_NIGHT_MODE;
        } else {
            isp_cmd = ISP_CTRL_SCENE_MODE;
        }

        {
            struct sensor_raw_info *raw_info_ptr = NULL;
            struct sensor_libuse_info *libuse_info = NULL;
            cmr_int product_id = 0;

            /*for third ae*/
            raw_info_ptr = cxt->sn_cxt.sensor_info.raw_info_ptr;
            if (raw_info_ptr) {
                libuse_info = raw_info_ptr->libuse_info;
                if (libuse_info) {
                    product_id = libuse_info->ae_lib_info.product_id;
                }
            }
            if (product_id) {
                isp_cmd = ISP_CTRL_SCENE_MODE;
            }
        }
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_SCENECAPTURETYPE;
        isp_param = param_ptr->cmd_value;
        if (ISP_AE_MODE_MAX == isp_param) {
            set_isp_flag = 0;
        }
        CMR_LOGD("ae mode %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_AE_MEASURE_LUM:
        isp_cmd = ISP_CTRL_AE_MEASURE_LUM;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("aw measure lum %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_AE_METERING_AREA:
        isp_cmd = ISP_CTRL_AE_TOUCH;
        trim.start_x = param_ptr->win_area.rect[0].start_x;
        trim.start_y = param_ptr->win_area.rect[0].start_y;
        if (0 == param_ptr->win_area.rect[0].width)
            trim.end_x = 0;
        else
            trim.end_x = param_ptr->win_area.rect[0].start_x +
                         param_ptr->win_area.rect[0].width - 1;

        if (0 == param_ptr->win_area.rect[0].height)
            trim.end_y = 0;
        else
            trim.end_y = param_ptr->win_area.rect[0].start_y +
                         param_ptr->win_area.rect[0].height - 1;

        CMR_LOGD("AE ROI (%d,%d,%d,%d)", trim.start_x, trim.start_y, trim.end_x,
                 trim.end_y);
        ptr_flag = 1;
        isp_param_ptr = (void *)&trim;
        break;
    case COM_ISP_SET_BRIGHTNESS:
        isp_cmd = ISP_CTRL_BRIGHTNESS;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_BRIGHTNESSVALUE;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("brightness %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_CONTRAST:
        isp_cmd = ISP_CTRL_CONTRAST;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_CONTRAST;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("contrast %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_SATURATION:
        isp_cmd = ISP_CTRL_SATURATION;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_SATURATION;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("saturation %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_SHARPNESS:
        isp_cmd = ISP_CTRL_SHARPNESS;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_SHARPNESS;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("sharpness %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_SPECIAL_EFFECT:
        isp_cmd = ISP_CTRL_SPECIAL_EFFECT;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("effect %d", param_ptr->cmd_value);
        break;
    case COM_ISP_GET_MFNR_PARAM:
        isp_cmd = ISP_CTRL_GET_MFNR_PARAM;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->mfnr_param;
        CMR_LOGD("ISP_CTRL_GET_MFNR_PARAM begin");
        break;
    case COM_ISP_SET_EV:
        isp_cmd = ISP_CTRL_AE_EXP_COMPENSATION;
        ptr_flag = 1;
        ae_compensation.comp_val =
            param_ptr->ae_compensation_param.ae_exposure_compensation;
        ae_compensation.comp_range.min =
            param_ptr->ae_compensation_param.ae_compensation_range[0];
        ae_compensation.comp_range.max =
            param_ptr->ae_compensation_param.ae_compensation_range[1];
        ae_compensation.step_numerator =
            param_ptr->ae_compensation_param.ae_compensation_step_numerator;
        ae_compensation.step_denominator =
            param_ptr->ae_compensation_param.ae_compensation_step_denominator;
        isp_param_ptr = (void *)&ae_compensation;
        CMR_LOGD("ae compensation: comp_val=%d, range.min=%d, range.max=%d, "
                 "step_numerator=%d, step_denominator=%d",
                 ae_compensation.comp_val, ae_compensation.comp_range.min,
                 ae_compensation.comp_range.max, ae_compensation.step_numerator,
                 ae_compensation.step_denominator);
        break;
    case COM_ISP_SET_AWB_MODE:
        CMR_LOGD("awb mode 00 %d isp param %d", param_ptr->cmd_value,
                 isp_param);
        isp_cmd = ISP_CTRL_AWB_MODE;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_LIGHTSOURCE;
        isp_param = camera_param_to_isp(COM_ISP_SET_AWB_MODE, param_ptr);
        CMR_LOGD("awb mode %d isp param %d", param_ptr->cmd_value, isp_param);
        break;
    case COM_ISP_SET_ANTI_BANDING:
        isp_cmd = ISP_CTRL_FLICKER;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("flicker %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_ISO:
        isp_cmd = ISP_CTRL_ISO;
        set_exif_flag = 1;
        exif_cmd = SENSOR_EXIF_CTRL_ISOSPEEDRATINGS;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("iso %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_VIDEO_MODE:
        isp_cmd = ISP_CTRL_VIDEO_MODE;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("isp video mode %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_FLASH_EG:
        isp_cmd = ISP_CTRL_FLASH_EG;
        isp_param = param_ptr->cmd_value;
        break;
    case COM_ISP_GET_EXIF_IMAGE_INFO:
        isp_cmd = ISP_CTRL_GET_EXIF_INFO;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->exif_pic_info;
        break;
    case COM_ISP_GET_EXIF_DEBUG_INFO:
        isp_cmd = ISP_CTRL_GET_EXIF_DEBUG_INFO;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->isp_dbg_info;
        break;
    case COM_ISP_GET_CUR_ADGAIN_EXP:
        isp_cmd = ISP_CTRL_GET_CUR_ADGAIN_EXP;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->isp_adgain;
        break;
    case COM_ISP_GET_CUR_COL_TEM:
        isp_cmd = ISP_CTRL_GET_AWB_CT;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->isp_cur_ct;
        break;
    case COM_ISP_GET_CUR_SENS:
        isp_cmd = ISP_CUR_ISO;
        isp_param_ptr = (void *)&param_ptr->isp_cur_iso;
        ret = isp_capability(isp_cxt->isp_handle, isp_cmd, isp_param_ptr);
        if (ret) {
            CMR_LOGE("Failed to read isp capability ret = %ld", ret);
        }
        return ret;
    case COM_ISP_SET_AF:
        isp_cmd = ISP_CTRL_AF;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->af_param;
        CMR_LOGD("isp_cmd =%d", isp_cmd);
        break;
    case COM_ISP_SET_AF_MODE:
        isp_cmd = ISP_CTRL_AF_MODE;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("af mode %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_AF_STOP:
        isp_cmd = ISP_CTRL_AF_STOP;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("isp_cmd =%d af mode %d", isp_cmd, param_ptr->cmd_value);
        break;
    case COM_ISP_GET_LOW_LUX_EB:
        isp_cmd = ISP_LOW_LUX_EB;
        isp_param_ptr = (void *)&param_ptr->cmd_value;

        ret = isp_capability(isp_cxt->isp_handle, isp_cmd, isp_param_ptr);
        if (ret) {
            CMR_LOGE("Failed to read isp capability ret = %ld", ret);
        }
        return ret;
    case COM_ISP_SET_FLASH_NOTICE:
        isp_cmd = ISP_CTRL_FLASH_NOTICE;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->flash_notice;
        CMR_LOGD("isp_cmd = %d, mode = %d", isp_cmd,
                 param_ptr->flash_notice.mode);
        break;
    case COM_ISP_SET_FACE_AREA:
        isp_cmd = ISP_CTRL_FACE_AREA;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->fd_param;
        CMR_LOGD("isp_cmd = %d, face_num = %d", isp_cmd,
                 param_ptr->fd_param.face_num);
        break;
    case COM_ISP_SET_RANGE_FPS:
        isp_cmd = ISP_CTRL_RANGE_FPS;
        ptr_flag = 1;
        isp_fps.min_fps = param_ptr->range_fps.min_fps;
        isp_fps.max_fps = param_ptr->range_fps.max_fps;
        isp_param_ptr = (void *)&isp_fps;
        break;

    case COM_ISP_SET_FPS_LLS_MODE:
        isp_cmd = ISP_CTRL_SET_AE_FPS;
        ae_fps = param_ptr->fps_param;
        ptr_flag = 1;
        isp_param_ptr = (void *)&ae_fps;
        break;

    case COM_ISP_SET_HDR:
        isp_cmd = ISP_CTRL_HDR;
        hdr_param.hdr_enable = param_ptr->cmd_value;
#ifdef CONFIG_SPRD_HDR_LIB_VERSION_2
        hdr_param.ev_effect_valid_num =
            cxt->sn_cxt.cur_sns_ex_info.exp_valid_frame_num + 2;
#else
        hdr_param.ev_effect_valid_num =
            cxt->sn_cxt.cur_sns_ex_info.exp_valid_frame_num + 1;
#endif
        ptr_flag = 1;
        CMR_LOGD("set HDR enable %d, ev_effect_valid_num %d",
                 hdr_param.hdr_enable, hdr_param.ev_effect_valid_num);
        isp_param_ptr = (void *)&hdr_param;
        break;

    case COM_ISP_SET_3DNR:
        isp_cmd = ISP_CTRL_3DNR;
        param_3dnr.enable = param_ptr->cmd_value;
        ptr_flag = 1;
        CMR_LOGD("set 3dnr enable %d", param_3dnr.enable);
        isp_param_ptr = (void *)&param_3dnr;
        break;

    case COM_ISP_SET_AE_LOCK_UNLOCK:
        isp_cmd = ISP_CTRL_SET_AE_LOCK_UNLOCK;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("set AE Lock & Unlock %d", isp_param);
        break;

    case COM_ISP_SET_ROI_CONVERGENCE_REQ:
        isp_cmd = ISP_CTRL_SET_CONVERGENCE_REQ;
        ptr_flag = 1;
        /**add for 3d calibration set 3d calibration flag begin*/
        CMR_LOGD("ISP_CTRL_SET_CONVERGENCE_REQ, is 3dcalibration enable:%d, "
                 "params value:%d",
                 cxt->is_3dcalibration_mode, param_ptr->cmd_value);
        if (cxt->is_3dcalibration_mode && 1 == param_ptr->cmd_value) {
            ptr_flag = 0;
            isp_param = param_ptr->cmd_value;
            CMR_LOGD(
                "ISP_CTRL_SET_CONVERGENCE_REQ, isp_param:%d, &isp_param:%p",
                isp_param, &isp_param);
        }
        /**add for 3d calibration set 3d calibration flag end*/
        CMR_LOGI("ISP_CTRL_SET_CONVERGENCE_REQ");
        break;

    case COM_ISP_SET_SNAPSHOT_FINISHED:
        isp_cmd = ISP_CTRL_SET_SNAPSHOT_FINISHED;
        ptr_flag = 1;
        CMR_LOGI("ISP_CTRL_SET_SNAPSHOT_FINISHED");
        break;

    case COM_ISP_SET_FLASH_MODE:
        isp_cmd = ISP_CTRL_SET_FLASH_MODE;
        isp_param = param_ptr->cmd_value;
        CMR_LOGI("flash mode %d", param_ptr->cmd_value);
        break;

    case COM_ISP_GET_YIMG_INFO:
        isp_cmd = ISP_CTRL_GET_YIMG_INFO;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->isp_yimg;
        break;

    case COM_ISP_SET_PREVIEW_YIMG:
        isp_cmd = ISP_CTRL_SET_PREV_YIMG;
        isp_param = param_ptr->cmd_value;
        break;

    case COM_ISP_SET_PREVIEW_YUV:
        isp_cmd = ISP_CTRL_SET_PREV_YUV;
        isp_param = param_ptr->cmd_value;
        break;
    case COM_ISP_GET_VCM_INFO:
        isp_cmd = ISP_CTRL_GET_VCM_INFO;
        // isp_param = param_ptr->cmd_value;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->vcm_step;
        break;

    case COM_ISP_SET_PREVIEW_PDAF_RAW:
        isp_cmd = ISP_CTRL_SET_PREV_PDAF_RAW;
        ptr_flag = 1;
        isp_param_ptr = param_ptr->cmd_ptr;
        break;

    case COM_ISP_SET_AWB_LOCK_UNLOCK:
        isp_cmd = ISP_CTRL_SET_AE_AWB_LOCK_UNLOCK;
        isp_param = param_ptr->cmd_value;
        if (isp_param == ISP_AWB_UNLOCK)
            isp_param = ISP_AWB_UNLOCK;
        else if (isp_param == ISP_AWB_LOCK)
            isp_param = ISP_AWB_LOCK;
        break;

    case COM_ISP_GET_LEDS_CTRL:
        ptr_flag = 1;
        isp_cmd = ISP_CTRL_GET_LEDS_CTRL;
        isp_param_ptr = (void *)&param_ptr->leds_ctrl;
        break;

    case COM_ISP_SET_SPRD_APP_MODE:
        isp_cmd = ISP_CTRL_SET_APP_MODE;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("set app mode id = %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_AUTO_HDR:
        CMR_LOGI("set auto hdr %d", param_ptr->cmd_value);
        isp_cmd = ISP_CTRL_AUTO_HDR_MODE;
        isp_param = param_ptr->cmd_value;
        break;
    case COM_ISP_SET_AE_MODE_CONTROL:
        isp_cmd = ISP_CTRL_SET_AE_MODE;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("ae_mode %d", param_ptr->cmd_value);
        break;
    case COM_ISP_SET_EXPOSURE_TIME:
        isp_cmd = ISP_CTRL_SET_AE_EXP_TIME;
        isp_param = param_ptr->cmd_value;
        CMR_LOGD("exposure time %d", isp_param);
        break;

#ifdef CAMERA_CNR3_ENABLE
    case COM_ISP_GET_CNR3_PARAM:
#ifdef CONFIG_CAMERA_CNR
        CMR_LOGD("into this case COM_ISP_GET_CNR3_PARAM");
        isp_cmd = ISP_CTRL_GET_CNR3_PARAM;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->cnr3_param;
#else
        isp_cmd = ISP_CTRL_MAX;
#endif
        break;
#endif

    case COM_ISP_GET_CNR2_PARAM:
#ifdef CONFIG_CAMERA_CNR
        isp_cmd = ISP_CTRL_GET_CNR2_PARAM;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->cnr2_param;
#else
        isp_cmd = ISP_CTRL_MAX;
#endif
        break;
    case COM_ISP_GET_YNRS_PARAM:
#ifdef CONFIG_CAMERA_CNR
        isp_cmd = ISP_CTRL_GET_YNRS_PARAM;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->ynr_param;
#else
        isp_cmd = ISP_CTRL_MAX;
#endif
        break;
#ifdef CAMERA_CNR3_ENABLE
    case COM_ISP_GET_CNR2CNR3_YNR_EN:
        isp_cmd = ISP_CTRL_GET_CNR2CNR3_YNR_EN;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->cnr2cnr3_ynr_en;
        break;
#else
    case COM_ISP_GET_CNR2_YNR_EN:
        isp_cmd = ISP_CTRL_GET_CNR2_YNR_EN;
        ptr_flag = 1;
        isp_param_ptr = (void *)&param_ptr->cnr2_ynr_en;
        break;
#endif
    default:
        CMR_LOGE("don't support cmd %ld", cmd_type);
        ret = CMR_CAMERA_NO_SUPPORT;
        break;
    }

    if (ptr_flag) {
        ret = isp_ioctl(isp_cxt->isp_handle, isp_cmd, isp_param_ptr);
        if (ret) {
            CMR_LOGE("failed isp ioctl %ld", ret);
        }
        CMR_LOGV("done %ld and direct return", ret);
        return ret;
    }

    if (set_isp_flag) {
        ret = isp_ioctl(isp_cxt->isp_handle, isp_cmd, (void *)&isp_param);
        if (ret) {
            CMR_LOGE("failed isp ioctl %ld", ret);
        } else {
            if (COM_ISP_SET_ISO == cmd_type) {
                if (0 == param_ptr->cmd_value) {
                    isp_capability(isp_cxt->isp_handle, ISP_CUR_ISO,
                                   (void *)&isp_param);
                    cxt->setting_cxt.is_auto_iso = 1;
                } else {
                    cxt->setting_cxt.is_auto_iso = 0;
                }
                isp_param = POWER2(isp_param - 1) * ONE_HUNDRED;
                CMR_LOGI("auto iso %d, exif iso %d",
                         cxt->setting_cxt.is_auto_iso, isp_param);
            }
        }
    }

    if (set_exif_flag) {
        CMR_LOGV("ERIC set exif");
        if (COM_ISP_SET_AWB_MODE == cmd_type) {
            cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                exif_cmd, param_ptr->cmd_value);
            cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                SENSOR_EXIF_CTRL_WHITEBALANCE,
                                param_ptr->cmd_value);
        } else {
            cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                exif_cmd, isp_param);
        }
    }
exit:
    CMR_LOGV("done %ld", ret);
    return ret;
}

void camera_get_iso_value(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_u32 isp_param = 0;
    if (cxt->isp_cxt.is_work && (1 == cxt->setting_cxt.is_auto_iso)) {
        struct isp_context *isp_cxt = &cxt->isp_cxt;
        isp_capability(isp_cxt->isp_handle, ISP_CUR_ISO, (void *)&isp_param);
        CMR_LOGI("iso value is %d", isp_param);
        cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                            SENSOR_EXIF_CTRL_ISOSPEEDRATINGS, isp_param);
    }
}

void camera_set_exif_exposure_time(cmr_handle oem_handle) {

    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    struct exif_spec_pic_taking_cond_tag exif_pic_info;
    EXIF_RATIONAL_T exposure_time;
    cmr_sensor_get_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                        &exif_pic_info);
    exposure_time.numerator = exif_pic_info.ExposureTime.numerator;
    exposure_time.denominator = exif_pic_info.ExposureTime.denominator;

    setting_param.camera_id = cxt->camera_id;
    setting_param.cmd_type_value = (cmr_uint)&exposure_time;
    cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                      SETTING_SET_EXIF_EXPOSURE_TIME, &setting_param);

    CMR_LOGD("exposure_time_numerator %d exposure_time_denominator %d",
             exposure_time.numerator, exposure_time.denominator);
}

cmr_int camera_get_ae_lum_value(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_int lum_val = 0;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    isp_capability(isp_cxt->isp_handle, ISP_CTRL_GET_AE_LUM, (void *)&lum_val);

    return lum_val;
}

cmr_int camera_get_setting_activity(cmr_handle oem_handle,
                                    cmr_uint *is_active) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !is_active) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    *is_active = cxt->setting_cxt.is_active;
    CMR_LOGV("%ld", (cmr_uint)*is_active);
exit:
    return ret;
}

cmr_int camera_get_preview_param(cmr_handle oem_handle,
                                 enum takepicture_mode mode,
                                 cmr_uint is_snapshot,
                                 struct preview_param *out_param_ptr) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt = &cxt->prev_cxt;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct jpeg_context *jpeg_cxt = &cxt->jpeg_cxt;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;
    struct setting_cmd_parameter setting_param;
    struct sensor_context *sn_cxt = &cxt->sn_cxt;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    char value[PROPERTY_VALUE_MAX] = {
        0,
    };
    cmr_u32 is_cfg_snp = 0;
    cmr_u32 rotation = 0;
    cmr_u8 haf_enable = 0;
    cmr_u32 is_raw_capture = 0;

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    cmr_bzero((void *)out_param_ptr, sizeof(*out_param_ptr));

    out_param_ptr->memory_setting.alloc_mem = camera_malloc;
    out_param_ptr->memory_setting.free_mem = camera_free;
    out_param_ptr->memory_setting.gpu_alloc_mem = camera_gpu_malloc;

    property_get("persist.vendor.cam.raw.mode", value, "jpeg");
    if (!strcmp(value, "raw")) {
        is_raw_capture = 1;
    }

    if (CAMERA_SNAPSHOT != is_snapshot) {
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_PREVIEW_FORMAT, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get preview fmt %ld", ret);
            goto exit;
        }
        out_param_ptr->preview_fmt = setting_param.cmd_type_value;
        out_param_ptr->is_fd_on = cxt->fd_on_off;
        out_param_ptr->preview_eb = 1;
        out_param_ptr->is_support_fd = cxt->is_support_fd;
        out_param_ptr->is_lls_enable = cxt->is_lls_enable;
        /*get prev rot*/
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_PREVIEW_ANGLE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get preview angle %ld", ret);
            goto exit;
        }
        out_param_ptr->prev_rot = setting_param.cmd_type_value;
        /*get raw capture size*/
        if (is_raw_capture == 1) {
            ret =
                cmr_setting_ioctl(setting_cxt->setting_handle,
                                  SETTING_GET_RAW_CAPTURE_SIZE, &setting_param);
            if (ret) {
                CMR_LOGE("failed to get prev size %ld", ret);
                goto exit;
            }
            out_param_ptr->raw_capture_size = setting_param.size_param;
        }

        if (cxt->is_multi_mode == MODE_SBS) {
            ret =
                cmr_setting_ioctl(setting_cxt->setting_handle,
                                  SETTING_GET_RAW_CAPTURE_SIZE, &setting_param);
            if (ret) {
                CMR_LOGE("failed to get prev size %ld", ret);
                goto exit;
            }
            out_param_ptr->raw_capture_size = setting_param.size_param;
            CMR_LOGI("sbs mode raw capture width %d, height %d",
                     out_param_ptr->raw_capture_size.width,
                     out_param_ptr->raw_capture_size.height);
        }

        /*get prev size*/
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_PREVIEW_SIZE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get prev size %ld", ret);
            goto exit;
        }
        prev_cxt->size = setting_param.size_param;
        out_param_ptr->preview_size = prev_cxt->size;
        /*get video size*/
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_VIDEO_SIZE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get video size %ld", ret);
            goto exit;
        }
        prev_cxt->video_size = setting_param.size_param;
        out_param_ptr->video_size = prev_cxt->video_size;

        /*get dv mode*/
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_DV_MODE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get dv mode %ld", ret);
            goto exit;
        }
        out_param_ptr->is_dv = setting_param.cmd_type_value;
        if ((0 != out_param_ptr->video_size.width) &&
            (0 != out_param_ptr->video_size.height)) {
            out_param_ptr->video_eb = 1;
        }

        cmr_bzero(&setting_param, sizeof(setting_param));
        setting_param.camera_id = cxt->camera_id;
        ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                                SETTING_GET_SLOW_MOTION_FLAG, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get dv mode %ld", ret);
            goto exit;
        }
        out_param_ptr->video_slowmotion_eb = setting_param.cmd_type_value;
        CMR_LOGD("video_slowmotion_eb = %d",
                 out_param_ptr->video_slowmotion_eb);

        CMR_LOGD("mode = %d", mode);
        if (CAMERA_ZSL_MODE == mode) {
            is_cfg_snp = 1;
        }

    } else {
        is_cfg_snp = 1;
    }

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_REFOCUS_ENABLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview refocus enabled flag %ld", ret);
        goto exit;
    }
    cxt->is_refocus_mode = setting_param.cmd_type_value;
    CMR_LOGD("is_refocus_mode %d", cxt->is_refocus_mode);
    if (cxt->is_refocus_mode == 2 && cxt->camera_id == 2 && is_snapshot) {
        // cxt->isp_to_dram = 1;
        // out_param_ptr->isp_to_dram = 1;
    }

    /*TBD need to get refocus flag*/
    out_param_ptr->refocus_eb = cxt->is_refocus_mode;
    CMR_LOGD("refocus_eb %d", out_param_ptr->refocus_eb);

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_CAPTURE_FORMAT, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get cap fmt %ld", ret);
        goto exit;
    }
    out_param_ptr->cap_fmt = setting_param.cmd_type_value;

    /*get 3dnr  flag*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_3DNR,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get 3dnr %ld", ret);
        goto exit;
    }
    camera_set_3dnr_flag(cxt, setting_param.cmd_type_value);
    property_get("debug.camera.3dnr.preview", value, "true");
    if (!strcmp(value, "false")) {
        out_param_ptr->is_3dnr = 0;
        out_param_ptr->is_sw_3dnr = 0;
    } else {
        // out_param_ptr->is_3dnr = setting_param.cmd_type_value;
        out_param_ptr->is_3dnr = camera_get_3dnr_flag(cxt);
        if (1 == out_param_ptr->is_3dnr) {
            property_get("persist.vendor.cam.3dnr.version", value, "0");
            if (0 == atoi(value))
                out_param_ptr->is_sw_3dnr = 0;
            else
                out_param_ptr->is_sw_3dnr = 1;
        } else {
            out_param_ptr->is_sw_3dnr = 0;
        }
    }
    CMR_LOGD("get preview 3dnr param: %d, %d", out_param_ptr->is_3dnr,
             out_param_ptr->is_sw_3dnr);
    /*get zoom param*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_ZOOM_PARAM,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get zoom param %ld", ret);
        goto exit;
    }
    /*get pdaf enable flag*/
    if (1 == out_param_ptr->video_slowmotion_eb ||
        0 == sn_cxt->cur_sns_ex_info.pdaf_supported ||
        CAMERA_ISP_TUNING_MODE == mode || CAMERA_UTEST_MODE == mode ||
        CAMERA_AUTOTEST_MODE == mode || CAMERA_ISP_SIMULATION_MODE == mode ||
        1 == out_param_ptr->video_eb || 1 == is_raw_capture ||
        1 == out_param_ptr->is_dv || cxt->is_multi_mode)
        out_param_ptr->pdaf_eb = 0;
    else if (1 == out_param_ptr->preview_eb)
        out_param_ptr->pdaf_eb = 0;

    property_get("persist.vendor.cam.pdaf.off", value, "0");
    if (atoi(value)) {
        out_param_ptr->pdaf_eb = 0;
    }
#if defined(CONFIG_ISP_PDAF_EXTRACTOR)
    out_param_ptr->pdaf_eb = 0;
#endif
    haf_enable = out_param_ptr->pdaf_eb;
    CMR_LOGD("haf_enable %d", haf_enable);

    out_param_ptr->zoom_setting = setting_param.zoom_param;
    CMR_LOGV("aspect ratio prev=%f, video=%f, cap=%f",
             out_param_ptr->zoom_setting.zoom_info.prev_aspect_ratio,
             out_param_ptr->zoom_setting.zoom_info.video_aspect_ratio,
             out_param_ptr->zoom_setting.zoom_info.capture_aspect_ratio);

    cxt->snp_cxt.snp_mode = mode;
    if (0 == is_cfg_snp) {
        CMR_LOGI("don't need cfg snp");
        goto exit;
    }
    out_param_ptr->snapshot_eb = is_cfg_snp;
    // out_param_ptr->flip_on = cxt->flip_on;

    out_param_ptr->isp_width_limit = cxt->isp_cxt.width_limit;
    out_param_ptr->tool_eb = 0;
    if (CAMERA_ISP_TUNING_MODE == mode || CAMERA_UTEST_MODE == mode ||
        CAMERA_AUTOTEST_MODE == mode || CAMERA_ISP_SIMULATION_MODE == mode) {
        out_param_ptr->tool_eb = 1;
    }

    /*get snapshot size*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_CAPTURE_SIZE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get capture size %ld", ret);
        goto exit;
    }
    out_param_ptr->picture_size = setting_param.size_param;
    cxt->snp_cxt.request_size = setting_param.size_param;
    /*get snapshot angle*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_CAPTURE_ANGLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get capture rot %ld", ret);
        goto exit;
    }
    out_param_ptr->cap_rot = setting_param.cmd_type_value;
    /*get rotation snapshot cfg*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_ROTATION_CAPTURE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get capture rot %ld", ret);
        goto exit;
    }
    out_param_ptr->is_cfg_rot_cap = setting_param.cmd_type_value;
    cxt->snp_cxt.is_cfg_rot_cap = out_param_ptr->is_cfg_rot_cap;

    /*get hdr flag*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_HDR,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get envir %ld", ret);
        goto exit;
    }
    camera_set_hdr_flag(cxt, setting_param.cmd_type_value);
    out_param_ptr->is_hdr = setting_param.cmd_type_value;

    /*get android zsl flag*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_ANDROID_ZSL_FLAG, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get android zsl %ld", ret);
        goto exit;
    }
    cxt->is_android_zsl = setting_param.cmd_type_value;
    /*get jpeg param*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_JPEG_QUALITY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get image quality %ld", ret);
        goto exit;
    }
    jpeg_cxt->param.quality = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_THUMB_QUALITY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get thumb quality %ld", ret);
        goto exit;
    }
    jpeg_cxt->param.thumb_quality = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_THUMB_SIZE,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get thumb size %ld", ret);
        goto exit;
    }
    jpeg_cxt->param.thum_size = setting_param.size_param;
    out_param_ptr->thumb_size = jpeg_cxt->param.thum_size;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_ENCODE_ANGLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get enc angle %ld", ret);
        goto exit;
    }
    out_param_ptr->encode_angle = setting_param.cmd_type_value;
    jpeg_cxt->param.set_encode_rotation = setting_param.cmd_type_value;
    if (cxt->snp_cxt.is_cfg_rot_cap) {
        cxt->snp_cxt.cfg_cap_rot = jpeg_cxt->param.set_encode_rotation;
    } else {
        cxt->snp_cxt.cfg_cap_rot = 0;
    }

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SHOT_NUMBER, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get shot num %ld", ret);
        goto exit;
    }
    cxt->snp_cxt.total_num = setting_param.cmd_type_value;

/* from sharkl2, jpeg support mirror/flip, so mirror feature use jpeg, not use
 * isp */
#ifdef MIRROR_FLIP_BY_ISP
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_ENCODE_ROTATION, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get enc rotation %ld", ret);
        goto exit;
    }
    rotation = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_FLIP_ON, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd flip_on enabled flag %ld", ret);
        goto exit;
    }
    out_param_ptr->flip_on = setting_param.cmd_type_value;
    if (out_param_ptr->flip_on) {
        CMR_LOGI("encode_rotation:%d, flip:%d", rotation,
                 out_param_ptr->flip_on);
        if (90 == rotation || 270 == rotation) {
            out_param_ptr->flip_on = 0x1; // flip
        } else if (0 == rotation || 180 == rotation) {
            out_param_ptr->flip_on = 0x3; // mirror
        }
    }
#endif

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SPRD_ZSL_ENABLED, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd zsl enabled flag %ld", ret);
        goto exit;
    }
    out_param_ptr->sprd_zsl_enabled = setting_param.cmd_type_value;
    CMR_LOGD("sprd zsl_enabled flag %d", out_param_ptr->sprd_zsl_enabled);

    ret = camera_ipm_open_module((cmr_handle)cxt);
    if (ret) {
        CMR_LOGE("failed to open ipm module%ld", ret);
        goto exit;
    }

    if (CAMERA_ZSL_MODE == mode) {
        out_param_ptr->frame_count = FRAME_NUM_MAX;
        out_param_ptr->frame_ctrl = FRAME_CONTINUE;
    } else {
        if (camera_get_hdr_flag(cxt)) {
            out_param_ptr->frame_count = cxt->ipm_cxt.hdr_num;
            out_param_ptr->frame_ctrl = FRAME_HDR_PROC;
        } else if (1 == camera_get_3dnr_flag(cxt)) {
            out_param_ptr->frame_count = cxt->ipm_cxt.threednr_num;
            out_param_ptr->frame_ctrl = FRAME_3DNR_PROC;
        } else {
            out_param_ptr->frame_count = cxt->snp_cxt.total_num;
            if (1 == cxt->snp_cxt.total_num) {
                out_param_ptr->frame_ctrl = FRAME_STOP;
            } else {
                out_param_ptr->frame_ctrl = FRAME_CONTINUE;
            }
        }
    }

#if defined(CONFIG_CAMERA_FLASH_HIGH_AE_MEASURE)
    setting_param.ctrl_flash.capture_mode.capture_mode = snp_cxt->snp_mode;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_FLASH_STATUS, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get flash mode %ld", ret);
    }
    CMR_LOGI("setting_param.cmd_type_value = %d", setting_param.cmd_type_value);
    if (!ret && setting_param.cmd_type_value) {
        out_param_ptr->frame_count = FRAME_FLASH_MAX;
        out_param_ptr->frame_ctrl = FRAME_CONTINUE;
    }
#endif

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SPRD_PIPVIV_ENABLED, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd pipviv enabled flag %ld", ret);
        goto exit;
    }
    out_param_ptr->sprd_pipviv_enabled = setting_param.cmd_type_value;
    cxt->is_pipviv_mode = setting_param.cmd_type_value;

    if (cxt->is_refocus_mode == 2) {
        out_param_ptr->sprd_pipviv_enabled = 1;
        cxt->is_pipviv_mode = 1;
        if (cxt->camera_id == 2) {
            // cxt->isp_to_dram = 1;
            // out_param_ptr->isp_to_dram = 1;
        }
    }

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SPRD_EIS_ENABLED, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd eis enabled flag %ld", ret);
        goto exit;
    }
    out_param_ptr->sprd_eis_enabled = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_VIDEO_SNAPSHOT_TYPE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd eis enabled flag %ld", ret);
        goto exit;
    }
    out_param_ptr->video_snapshot_type = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SPRD_3DCAL_ENABLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview sprd eis enabled flag %ld", ret);
        goto exit;
    }
    cxt->is_3dcalibration_mode = setting_param.cmd_type_value;
    out_param_ptr->sprd_3dcalibration_enabled = cxt->is_3dcalibration_mode;

    CMR_LOGI(
        "sprd pipviv_enabled flag %d, cxt->is_refocus_mode %d, camera id %d"
        ", isp_to_dram %d, sprd eis_enabled flag %d, video_snapshot_type %d"
        ", sprd_3dcalibration_enabled flag %d",
        out_param_ptr->sprd_pipviv_enabled, cxt->is_refocus_mode,
        cxt->camera_id, out_param_ptr->isp_to_dram,
        out_param_ptr->sprd_eis_enabled, out_param_ptr->video_snapshot_type,
        out_param_ptr->sprd_3dcalibration_enabled);

exit:
    CMR_LOGD(
        "prev size %d %d, pic size %d %d, video size %d %d, android zsl flag "
        "%d, prev rot %ld snp rot %d rot snp %d, zoom mode %ld fd %ld is dv %d "
        "tool eb %d, q %d thumb q %d enc angle %d thumb size %d %d, frame cnt "
        "%d, out_param_ptr->flip_on %d, is_3dnr %d",
        out_param_ptr->preview_size.width, out_param_ptr->preview_size.height,
        out_param_ptr->picture_size.width, out_param_ptr->picture_size.height,
        out_param_ptr->video_size.width, out_param_ptr->video_size.height,
        cxt->is_android_zsl, out_param_ptr->prev_rot, out_param_ptr->cap_rot,
        out_param_ptr->is_cfg_rot_cap, out_param_ptr->zoom_setting.mode,
        out_param_ptr->is_fd_on, out_param_ptr->is_dv, out_param_ptr->tool_eb,
        jpeg_cxt->param.quality, jpeg_cxt->param.thumb_quality,
        jpeg_cxt->param.set_encode_rotation, jpeg_cxt->param.thum_size.width,
        jpeg_cxt->param.thum_size.height, out_param_ptr->frame_count,
        out_param_ptr->flip_on, out_param_ptr->is_3dnr);

    ATRACE_END();
    return ret;
}

cmr_int camera_set_preview_param(cmr_handle oem_handle,
                                 enum takepicture_mode mode,
                                 cmr_uint is_snapshot) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt = &cxt->prev_cxt;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct preview_param start_param;
    struct preview_out_param preview_out;

    if (1 != prev_cxt->inited) {
        CMR_LOGE("err, don't init preview");
        ret = -CMR_CAMERA_INVALID_STATE;
        goto exit;
    }
    if (PREVIEWING ==
        cmr_preview_get_status(cxt->prev_cxt.preview_handle, cxt->camera_id)) {
        CMR_LOGI("prev has been started");
        goto exit;
    }

    ret = camera_get_preview_param(oem_handle, mode, is_snapshot, &start_param);
    if (ret) {
        CMR_LOGE("failed to get prev param %ld", ret);
        goto exit;
    }

    prev_cxt->actual_picture_size.width = start_param.picture_size.width;
    prev_cxt->actual_picture_size.height = start_param.picture_size.height;

    cmr_bzero(&preview_out, sizeof(struct preview_out_param));
    ret = cmr_preview_set_param(prev_cxt->preview_handle, cxt->camera_id,
                                &start_param, &preview_out);
    if (ret) {
        CMR_LOGE("failed to set prev param %ld", ret);
        goto exit;
    }
    prev_cxt->preview_sn_mode = preview_out.preview_sn_mode;
    prev_cxt->channel_bits = preview_out.preview_chn_bits;
    prev_cxt->data_endian = preview_out.preview_data_endian;
    prev_cxt->video_sn_mode = preview_out.video_sn_mode;
    prev_cxt->video_channel_bits = preview_out.video_chn_bits;
    prev_cxt->depthmap_channel_bits = preview_out.depthmap_chn_bits;
    prev_cxt->pdaf_channel_bits = preview_out.pdaf_chn_bits;
    prev_cxt->actual_video_size = preview_out.actual_video_size;
    prev_cxt->video_data_endian = preview_out.video_data_endian;
    snp_cxt->snapshot_sn_mode = preview_out.snapshot_sn_mode;
    snp_cxt->channel_bits = preview_out.snapshot_chn_bits;
    snp_cxt->post_proc_setting = preview_out.post_proc_setting;
    snp_cxt->data_endian = preview_out.snapshot_data_endian;
    snp_cxt->zsl_frame = preview_out.zsl_frame;
    snp_cxt->actual_capture_size = preview_out.actual_snapshot_size;
    cmr_copy((void *)&snp_cxt->post_proc_setting,
             (void *)&preview_out.post_proc_setting,
             sizeof(snp_cxt->post_proc_setting));
    CMR_LOGI("prev mode %d prev chn %d snp mode %d snp chn %d",
             prev_cxt->preview_sn_mode, prev_cxt->channel_bits,
             snp_cxt->snapshot_sn_mode, snp_cxt->channel_bits);
    CMR_LOGI("rot angle %ld", snp_cxt->post_proc_setting.rot_angle);

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_local_set_thumb_size(cmr_handle oem_handle,
                                    struct img_size thum_size) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prv_cxt = &cxt->prev_cxt;

    cmr_preview_set_thumb_size(prv_cxt->preview_handle, cxt->camera_id,
                               thum_size);

    return 0;
}

cmr_int camera_get_snapshot_param(cmr_handle oem_handle,
                                  struct snapshot_param *out_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct jpeg_context *jpeg_cxt = &cxt->jpeg_cxt;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct setting_cmd_parameter setting_param;
    cmr_int i;
    cmr_u32 chn_bits = cxt->snp_cxt.channel_bits;
    cmr_uint cnr_typ;

    out_ptr->total_num = 0;
    out_ptr->rot_angle = 0;
    out_ptr->hdr_handle = cxt->ipm_cxt.hdr_handle;
    out_ptr->hdr_need_frm_num = cxt->ipm_cxt.hdr_num;
    out_ptr->threednr_handle = cxt->ipm_cxt.threednr_handle;
    out_ptr->threednr_need_frm_num = cxt->ipm_cxt.threednr_num;
    out_ptr->post_proc_setting.data_endian = cxt->snp_cxt.data_endian;
    out_ptr->lls_shot_mode = cxt->lls_shot_mode;
    out_ptr->is_vendor_hdr = cxt->is_vendor_hdr;
    out_ptr->is_pipviv_mode = cxt->is_pipviv_mode;
    out_ptr->is_3dcalibration_mode = cxt->is_3dcalibration_mode;
    setting_param.camera_id = cxt->camera_id;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SPRD_3DCAL_ENABLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get 3dcal flag %ld", ret);
        goto exit;
    }
    cxt->is_3dcalibration_mode = setting_param.cmd_type_value;
    out_ptr->is_3dcalibration_mode = setting_param.cmd_type_value;
    ret =
        cmr_setting_ioctl(setting_cxt->setting_handle,
                          SETTING_GET_SPRD_YUV_CALLBACK_ENABLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get yuv callback flag %ld", ret);
        goto exit;
    }
    cxt->is_yuv_callback_mode = setting_param.cmd_type_value;
    out_ptr->is_yuv_callback_mode = setting_param.cmd_type_value;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_HDR,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get envir %ld", ret);
        goto exit;
    }
    camera_set_hdr_flag(cxt, setting_param.cmd_type_value);

    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_3DNR,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get 3dnr %ld", ret);
        goto exit;
    }
    camera_set_3dnr_flag(cxt, setting_param.cmd_type_value);

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_SHOT_NUMBER, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get shot num %ld", ret);
        goto exit;
    }
    out_ptr->total_num = setting_param.cmd_type_value;
    cxt->snp_cxt.total_num = out_ptr->total_num;

    /*get jpeg param*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_JPEG_QUALITY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get image quality %ld", ret);
        goto exit;
    }
    jpeg_cxt->param.quality = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_THUMB_QUALITY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get thumb quality %ld", ret);
        goto exit;
    }
    jpeg_cxt->param.thumb_quality = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_THUMB_SIZE,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get thumb size %ld", ret);
        goto exit;
    }
    jpeg_cxt->param.thum_size = setting_param.size_param;

    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_TOUCH_XY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to open flash");
    }
    camera_set_touch_xy(cxt, setting_param.touch_param);

    /*get filter*/
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_FILTER_TEYP, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get filtertype %ld", ret);
        goto exit;
    }
    camera_set_filter_flag(cxt, setting_param.cmd_type_value);
    out_ptr->filter_type = setting_param.cmd_type_value;

    if (camera_get_filter_flag(cxt) && !cxt->ipm_cxt.filter_inited) {
        struct ipm_open_in in_param;
        struct ipm_open_out out_param;
        in_param.frame_size.width = cxt->snp_cxt.request_size.width;
        in_param.frame_size.height = cxt->snp_cxt.request_size.height;
        in_param.frame_rect.start_x = 0;
        in_param.frame_rect.start_y = 0;
        in_param.frame_rect.width = in_param.frame_size.width;
        in_param.frame_rect.height = in_param.frame_size.height;
        in_param.reg_cb = camera_ipm_cb;
        ret = camera_open_filter(cxt, &in_param, &out_param);
        if (ret) {
            CMR_LOGE("failed to open filter %ld", ret);
            goto exit;
        }
        cxt->ipm_cxt.filter_inited = 1;
    }

    cnr_typ = camera_get_cnr_realtime_flag(oem_handle);
    out_ptr->nr_flag = cnr_typ;
    cxt->nr_flag = cnr_typ;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_ENCODE_ANGLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get enc angle %ld", ret);
        goto exit;
    }
    out_ptr->rot_angle = setting_param.cmd_type_value;
    jpeg_cxt->param.set_encode_rotation = setting_param.cmd_type_value;

    ret = cmr_preview_get_post_proc_param(
        cxt->prev_cxt.preview_handle, cxt->camera_id,
        (cmr_u32)setting_param.cmd_type_value, &out_ptr->post_proc_setting);
    if (ret) {
        CMR_LOGE("failed to get rot angle %ld", ret);
        goto exit;
    }
    out_ptr->camera_id = cxt->camera_id;
    out_ptr->is_hdr = camera_get_hdr_flag(cxt);
    out_ptr->is_3dnr = camera_get_3dnr_flag(cxt);
    out_ptr->is_android_zsl = cxt->is_android_zsl;
    out_ptr->mode = cxt->snp_cxt.snp_mode;
    out_ptr->is_cfg_rot_cap = cxt->snp_cxt.is_cfg_rot_cap;
    out_ptr->jpeg_setting = jpeg_cxt->param;
    out_ptr->req_size = cxt->snp_cxt.request_size;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_VIDEO_SNAPSHOT_TYPE, &setting_param);

    if (ret) {
        CMR_LOGE("failed to get preview sprd eis enabled flag %ld", ret);
        goto exit;
    }
    if (setting_param.cmd_type_value == VIDEO_SNAPSHOT_VIDEO) {
        chn_bits = cxt->prev_cxt.video_channel_bits;
        out_ptr->is_video_snapshot = 1;
    } else if (CAMERA_ZSL_MODE == cxt->snp_cxt.snp_mode) {
        out_ptr->is_zsl_snapshot = 1;
    }

    CMR_LOGI("chn_bits %d actual size %d %d", chn_bits,
             out_ptr->post_proc_setting.actual_snp_size.width,
             out_ptr->post_proc_setting.actual_snp_size.height);
    out_ptr->channel_id = GRAB_CHANNEL_MAX + 1;
    for (i = 0; i < GRAB_CHANNEL_MAX; i++) {
        if (chn_bits & (1 << i)) {
            out_ptr->channel_id = i;
            break;
        }
    }

exit:
    CMR_LOGI("done,total num %d enc angle %d", out_ptr->total_num,
             out_ptr->rot_angle);
    return ret;
}

cmr_int camera_set_setting(cmr_handle oem_handle, enum camera_param_type id,
                           cmr_uint param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;

    setting_param.camera_id = cxt->camera_id;
    switch (id) {
    case CAMERA_PARAM_ZOOM:
        if (param) {
            setting_param.zoom_param = *((struct cmr_zoom_param *)param);
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, zoom param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_ENCODE_ROTATION:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_CONTRAST:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_BRIGHTNESS:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SHARPNESS:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_WB:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_EFFECT:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_FLASH:
    case CAMERA_PARAM_ISP_FLASH:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_ANTIBANDING:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_AUTO_EXPOSURE_MODE:
        setting_param.ae_param = *(struct cmr_ae_param *)param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_AE_REGION:
        setting_param.ae_param = *(struct cmr_ae_param *)param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_ISO:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_EXPOSURE_COMPENSATION:
        setting_param.ae_compensation_param =
            *(struct cmr_ae_compensation_param *)param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_PREVIEW_FPS:
        if (param) {
            setting_param.preview_fps_param =
                *(struct cmr_preview_fps_param *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, fps param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;

    case CAMERA_PARAM_PREVIEW_LLS_FPS:
        if (param) {
            /*to do */
            // use the cmr_preview_fps_param temporaily, to be replaced later
            setting_param.preview_fps_param =
                *(struct cmr_preview_fps_param *)param;
            setting_param.cmd_type_value =
                setting_param.preview_fps_param.frame_rate;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, fps param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;

    case CAMERA_PARAM_SPRD_3DNR_ENABLED:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;

    case CAMERA_PARAM_SPRD_ENABLE_CNR:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;

    case CAMERA_PARAM_SATURATION:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SCENE_MODE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_JPEG_QUALITY:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_THUMB_QUALITY:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SENSOR_ORIENTATION:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_FOCAL_LENGTH:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SENSOR_ROTATION:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_PERFECT_SKIN_LEVEL:
        setting_param.fb_param = *(struct beauty_info *)param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_FLIP_ON:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SHOT_NUM:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_ROTATION_CAPTURE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_POSITION:
        if (param) {
            setting_param.position_info = *(struct camera_position_type *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, postition param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_RAW_CAPTURE_SIZE:
        if (param) {
            setting_param.size_param = *(struct img_size *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, raw capture size param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_PREVIEW_SIZE:
        if (param) {
            setting_param.size_param = *(struct img_size *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, prev size param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_PREVIEW_FORMAT:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_CAPTURE_SIZE:
        if (param) {
            setting_param.size_param = *(struct img_size *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, capture size param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_CAPTURE_FORMAT:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_CAPTURE_MODE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_THUMB_SIZE:
        if (param) {
            setting_param.size_param = *(struct img_size *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, thumb size param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_VIDEO_SIZE:
        if (param) {
            setting_param.size_param = *(struct img_size *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, video size param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_RANGE_FPS:
        if (param) {
            setting_param.range_fps = *(struct cmr_range_fps_param *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, range fps param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_SPRD_ZSL_ENABLED:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SLOW_MOTION_FLAG:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_ISP_AE_LOCK_UNLOCK:
        CMR_LOGD("CAMERA_PARAM_ISP_AE_LOCK_UNLOCK");
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SPRD_PIPVIV_ENABLED:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SPRD_EIS_ENABLED:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_REFOCUS_ENABLE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_TOUCH_XY:
        if (param) {
            setting_param.size_param = *(struct img_size *)param;
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                    &setting_param);
        } else {
            CMR_LOGE("err, video size param is null");
            ret = -CMR_CAMERA_INVALID_PARAM;
        }
        break;
    case CAMERA_PARAM_VIDEO_SNAPSHOT_TYPE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SPRD_3DCAL_ENABLE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SPRD_YUV_CALLBACK_ENABLE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_ISP_AWB_LOCK_UNLOCK:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_EXIF_MIME_TYPE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_FILTER_TYPE:
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_AE_MODE:
        setting_param.cmd_type_value = param;
        CMR_LOGD("ae_mode=%lu", param);
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_EXPOSURE_TIME:
        CMR_LOGD("exposure time=%lu", param);
        setting_param.cmd_type_value = param;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SPRD_SET_APPMODE:
        setting_param.cmd_type_value = param;
        CMR_LOGD("sprd app mode id = %d", setting_param.cmd_type_value);
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    case CAMERA_PARAM_SPRD_AUTO_HDR_ENABLED:
        setting_param.cmd_type_value = param;
        CMR_LOGI("auto hdr=%lu", param);
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
    case CAMERA_PARAM_SET_DEVICE_ORIENTATION:
        setting_param.cmd_type_value = param;
        CMR_LOGD("frame_num %u", param);
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, id,
                                &setting_param);
        break;
    default:
        CMR_LOGI("don't support %d", id);
    }
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_restart_rot(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    ret = camera_rotation_deinit(oem_handle);
    if (ret) {
        CMR_LOGE("failed to de-init rotate %ld", ret);
    } else {
        ret = camera_rotation_init(oem_handle);
        if (ret) {
            CMR_LOGE("failed to initizalize rot");
        }
    }
exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

/*****************************************external
 * function*****************************************/

cmr_int camera_local_int(cmr_u32 camera_id, camera_cb_of_type callback,
                         void *client_data, cmr_uint is_autotest,
                         cmr_handle *oem_handle, void *cb_of_malloc,
                         void *cb_of_free) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = NULL;
    struct phySensorInfo *phyPtr = NULL;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    *oem_handle = (cmr_handle)0;
    cxt = (struct camera_context *)malloc(sizeof(struct camera_context));
    if (NULL == cxt) {
        CMR_LOGE("failed to create context");
        ret = -CMR_CAMERA_NO_MEM;
        goto exit;
    }
    cmr_bzero(cxt, sizeof(*cxt));
    cxt->camera_id = camera_id;
    cxt->master_id = master_id_oem;
    cxt->camera_cb = callback;
    cxt->client_data = client_data;
    cxt->hal_malloc = cb_of_malloc;
    cxt->hal_free = cb_of_free;
    cxt->hal_gpu_malloc = NULL;
    cxt->is_multi_mode = is_multi_camera_mode_oem;
    cxt->blur_facebeauty_flag = 0;
    phyPtr = sensorGetPhysicalSnsInfo(camera_id);
    cxt->facing = phyPtr->face_type;
    cxt->isp_pm_initing = false;
    pthread_mutex_init(&cxt->isp_pm_mutex, NULL);
    pthread_cond_init(&cxt->isp_pm_cond, NULL);

    CMR_LOGI("cxt=%p, client_data=%p", cxt, cxt->client_data);
    ret = camera_init_internal((cmr_handle)cxt, is_autotest);
    if (ret) {
        CMR_LOGE("camera_init_internal failed");
        goto exit;
    }

exit:
    if (CMR_CAMERA_SUCCESS == ret) {
        cxt->inited = 1;
        *oem_handle = (cmr_handle)cxt;
    } else {
        if (cxt) {
            free((void *)cxt);
        }
    }
    return ret;
}

cmr_int camera_local_set_gpu_mem_ops(cmr_handle oem_handle, void *cb_of_malloc,
                                     void *cb_of_free) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cxt->hal_gpu_malloc = cb_of_malloc;
    return ret;
}

cmr_int camera_local_deinit(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    camera_deinit_internal(oem_handle);
    free((void *)oem_handle);
    ATRACE_END();
    return ret;
}

cmr_int camera_local_start_preview(cmr_handle oem_handle,
                                   enum takepicture_mode mode,
                                   cmr_uint is_snapshot) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt = &cxt->prev_cxt;
    struct setting_cmd_parameter setting_param;

    cxt->setting_cxt.is_active = 1;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_SET_ENVIRONMENT, &setting_param);

    ret = camera_set_preview_param(oem_handle, mode, is_snapshot);
    if (ret) {
        CMR_LOGE("failed to set prev param %ld", ret);
        goto exit;
    }

    ret = cmr_preview_start(prev_cxt->preview_handle, cxt->camera_id);
    if (ret) {
        CMR_LOGE("failed to start prev %ld", ret);
    }

    cxt->camera_mode = mode;
    CMR_LOGI("camera mode %d", cxt->camera_mode);

#if 1
    ret = cmr_sensor_focus_init(cxt->sn_cxt.sensor_handle, cxt->camera_id);
#endif

exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_local_stop_preview(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    cmr_int prev_ret = CMR_CAMERA_SUCCESS;
    cmr_int snp_ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    struct common_isp_cmd_param param;

    CMR_LOGI("E");

    if (PREVIEWING !=
        cmr_preview_get_status(cxt->prev_cxt.preview_handle, cxt->camera_id)) {
        CMR_LOGI("don't previewing");
        goto exit;
    }

    if (CAMERA_ZSL_MODE == cxt->camera_mode) {
        if (IDLE != cxt->snp_cxt.status) {
            snp_ret = cmr_snapshot_stop(cxt->snp_cxt.snapshot_handle);
            if (snp_ret) {
                CMR_LOGE("failed to stop snp %ld", ret);
            }
        }
    }

    camera_get_iso_value(oem_handle);

    if(cxt->snp_cxt.status == CODEC_WORKING) {
        CMR_LOGI("wait for jpeg encode");
        int time = 0;
        while (1) {
            if (time > 2000) {
                CMR_LOGE("timeout");
                break;
            }
            time++;
            if (cxt->snp_cxt.status != CODEC_WORKING) {
                CMR_LOGI("jpeg encode ok wait up");
                break;
            }
            usleep(1000);
        }
    }

    prev_ret = cmr_preview_stop(cxt->prev_cxt.preview_handle, cxt->camera_id);
    if (ret) {
        CMR_LOGE("failed to stop prev %ld", ret);
    }

    cmr_bzero(&setting_param, sizeof(setting_param));
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_SPRD_ZSL_ENABLED, &setting_param);

    if (CAMERA_ZSL_MODE == cxt->camera_mode &&
        setting_param.cmd_type_value == 1) {
        prev_ret =
            cmr_setting_cancel_notice_flash(cxt->setting_cxt.setting_handle);
        CMR_LOGD("zsl takpicture calling stop_preview in stop flow %ld", ret);
    }

    ret = prev_ret | snp_ret;
    cxt->setting_cxt.is_active = 0;
    cxt->prev_cxt.video_size.width = 0;
    cxt->prev_cxt.video_size.height = 0;

    if (cxt->night_cxt.is_authorized) {
        cxt->night_flag = false;
        cxt->night_cxt.ipmpro_deinit(oem_handle);
    }

exit:
    CMR_LOGD("X");
    ATRACE_END();
    return ret;
}

cmr_int camera_local_highflash_ae_measure(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;

    if (CAMERA_ZSL_MODE != cxt->snp_cxt.snp_mode) {
        setting_param.camera_id = cxt->camera_id;
        setting_param.ctrl_flash.capture_mode.capture_mode = CAMERA_NORMAL_MODE;
        ret =
            cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                              SETTING_SET_HIGHFLASH_AE_MEASURE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to open flash");
        }
    }

    return ret;
}

/**add for 3d capture to reset reprocessing capture size begin*/
cmr_int camera_local_set_cap_size(cmr_handle oem_handle,
                                  cmr_u32 is_reprocessing, cmr_u32 camera_id,
                                  cmr_u32 width, cmr_u32 height) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prv_cxt = &cxt->prev_cxt;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;

    cmr_preview_set_cap_size(prv_cxt->preview_handle, is_reprocessing,
                             camera_id, width, height);
    snp_cxt->request_size.width = width;
    snp_cxt->request_size.height = height;

    return 0;
}
/**add for 3d capture to reset reprocessing capture size end*/

cmr_int camera_local_start_snapshot(cmr_handle oem_handle,
                                    enum takepicture_mode mode,
                                    cmr_uint is_snapshot) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt;
    struct snapshot_param snp_param;
    struct common_sn_cmd_param param;
    struct setting_cmd_parameter setting_param;
    cmr_int flash_status = FLASH_CLOSE;
    cmr_s32 sm_val = 0;

    if (!oem_handle) {
        CMR_LOGE("error handle");
        goto exit;
    }
    camera_take_snapshot_step(CMR_STEP_TAKE_PIC);
    prev_cxt = &cxt->prev_cxt;

    sem_getvalue(&cxt->share_path_sm, &sm_val);
    if (0 != sm_val) {
        sem_destroy(&cxt->share_path_sm);
        sem_init(&cxt->share_path_sm, 0, 0);
        CMR_LOGI("re-initialize share_path_sm");
    }

    sem_wait(&cxt->snapshot_sm);
    if (CAMERA_ZSL_MODE != mode) {
        ret = camera_set_preview_param(oem_handle, mode, is_snapshot);
        if (ret) {
            CMR_LOGE("failed to set preview param %ld", ret);
            goto exit;
        }
    } else {
        setting_param.camera_id = cxt->camera_id;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_THUMB_SIZE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get thumb size %ld", ret);
            goto exit;
        }
        // third party APP do video snapshot may update the thumb_size
        ret = camera_local_set_thumb_size(cxt, setting_param.size_param);
        if (ret) {
            CMR_LOGE("failed to update thumb size");
        }
    }

    cmr_bzero(&snp_param, sizeof(struct snapshot_param));
    ret = camera_get_snapshot_param(oem_handle, &snp_param);
    if (ret) {
        CMR_LOGE("failed to get snp num %ld", ret);
        goto exit;
    }

    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_HW_FLASH_STATUS, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get flash mode %ld", ret);
        // goto exit;
    } else {
        flash_status = setting_param.cmd_type_value;
    }
    CMR_LOGI("HW flash_status=%ld", flash_status);
    if (flash_status != FLASH_TORCH) {
        cmr_sensor_set_exif(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                            SENSOR_EXIF_CTRL_FLASH, 0);
    }

    ret = cmr_snapshot_post_proc(cxt->snp_cxt.snapshot_handle, &snp_param);
    if (ret) {
        CMR_LOGE("failed to snp post proc %ld", ret);
        goto exit;
    }

    cxt->is_start_snapshot = 1;
#ifdef CONFIG_FACE_BEAUTY
    memset(&(cxt->face_beauty), 0, sizeof(struct class_fb));
#endif
    cxt->snp_cxt.actual_capture_size =
        snp_param.post_proc_setting.chn_out_frm[0].size;
    if (CAMERA_ISP_SIMULATION_MODE == mode) {
        cxt->camera_mode = mode;
        // clean up the buffer had set to kernel
        cmr_preview_stop(prev_cxt->preview_handle, cxt->camera_id);
    } else if (CAMERA_ZSL_MODE != mode) {
        ret = cmr_preview_start(prev_cxt->preview_handle, cxt->camera_id);
        if (ret) {
            CMR_LOGE("failed to start prev %ld", ret);
        }
        cxt->camera_mode = mode;
    }

    if (1 == camera_get_hdr_flag(cxt)) {
        ret = camera_hdr_set_ev(oem_handle);
        if (ret)
            CMR_LOGE("fail to set hdr ev");
    } else if (1 == camera_get_3dnr_flag(cxt) &&
               (0 == camera_get_sw_3dnr_flag(cxt))) {
        sem_init(&cxt->threednr_proc_sm, 0, 0);
        ret = camera_3dnr_set_ev(oem_handle, 1);
        if (ret)
            CMR_LOGE("fail to set 3dnr ev");
    } else {
        if (cxt->camera_id == 0 ||
            !(strcmp(FRONT_CAMERA_FLASH_TYPE, "lcd") &
              strcmp(FRONT_CAMERA_FLASH_TYPE, "flash"))) {
            ret = camera_capture_highflash(oem_handle, cxt->camera_id);
            if (ret)
                CMR_LOGE("open high flash fail");
        }
    }

    ret = camera_local_start_capture(oem_handle);
    if (ret) {
        CMR_LOGE("camera_start_capture failed");
        goto exit;
    }

    camera_set_snp_req((cmr_handle)cxt, TAKE_PICTURE_NEEDED);
    camera_snapshot_started((cmr_handle)cxt);
    camera_get_iso_value(oem_handle);
    camera_set_exif_exposure_time(oem_handle);
    ret = camera_get_cap_time((cmr_handle)cxt);
    cxt->snp_cxt.status = SNAPSHOTING;
    cxt->snp_cxt.post_proc_setting.actual_snp_size =
        snp_param.post_proc_setting.actual_snp_size;
    cmr_copy(cxt->snp_cxt.post_proc_setting.chn_out_frm,
             snp_param.post_proc_setting.chn_out_frm,
             sizeof(cxt->snp_cxt.post_proc_setting.chn_out_frm));
    cxt->snp_cxt.post_proc_setting.actual_snp_size =
        snp_param.post_proc_setting.actual_snp_size;
    cxt->snp_cxt.post_proc_setting.dealign_actual_snp_size =
        snp_param.post_proc_setting
            .dealign_actual_snp_size; /**add for 3D Capture, modify reprocess
                                         request exif size*/
    cxt->snp_cxt.post_proc_setting.snp_size =
        snp_param.post_proc_setting.snp_size; /**add for 3D Capture, modify
                                                 reprocess request exif size*/

#if defined(CONFIG_CAMERA_FLASH_HIGH_AE_MEASURE)
    camera_set_discard_frame(cxt, 1);
    camera_local_highflash_ae_measure(oem_handle);
    camera_get_iso_value(oem_handle);
#endif

#if 0
	CMR_LOGI("is_refocus_mode=%ld",cxt->is_refocus_mode);
	if(cxt->is_refocus_mode == 2)
	{
		camera_set_discard_frame(cxt, 1);
		is_dual_capture ++;
		int i = 0;
		for(i=0;i<10;i++)
		{
			if(is_dual_capture == 2)
			{
				CMR_LOGD("dual camera capture is come,cxt->is_refocus_mode %ld, is_dual_capture %ld, i = %d", cxt->is_refocus_mode,is_dual_capture,i);
				is_dual_capture = 0;
				break;
			}
			usleep(10);
		}
		CMR_LOGD("dual camera capture is not come,cxt->is_refocus_mode %ld, is_dual_capture %ld, i = %d", cxt->is_refocus_mode,is_dual_capture,i);
	}
#endif

    camera_set_discard_frame(cxt, 0);

    if (CAMERA_ISP_SIMULATION_MODE == mode) {
        ret = cmr_isp_simulation_proc(cxt, &snp_param);
        if (ret) {
            CMR_LOGE("camera isp simulation mode failed");
            goto exit;
        }
        camera_post_share_path_available(oem_handle);
    }

exit:
    sem_post(&cxt->snapshot_sm);
    CMR_LOGV("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_local_stop_snapshot(cmr_handle oem_handle) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;
    memset(&setting_param, 0, sizeof(setting_param));
    sem_wait(&cxt->snapshot_sm);
    if (1 == camera_get_3dnr_flag(cxt)) {
#ifdef OEM_HANDLE_3DNR
        if (0 != cxt->ipm_cxt.frm_num) {
            cxt->ipm_cxt.frm_num = 0;
        }
#endif
        setting_param.camera_id = cxt->camera_id;
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle, SETTING_GET_APPMODE,
                            &setting_param);
        if(ret)
                CMR_LOGE("failed to get app mode");
        if((setting_param.cmd_type_value == CAMERA_MODE_NIGHT_PHOTO)
                 && cxt->night_cxt.is_authorized) {
                ret = cxt->night_cxt.sw_close(cxt);
        } else{
             ret = camera_close_3dnr(cxt);
             if (ret)
                CMR_LOGE("failed to close 3dnr");
        }
    }

    ret = cmr_snapshot_stop(cxt->snp_cxt.snapshot_handle);
    if (ret) {
        CMR_LOGE("failed to stop snp %ld", ret);
        goto exit;
    }

    if (TAKE_PICTURE_NEEDED == camera_get_snp_req(oem_handle)) {
        camera_set_snp_req(oem_handle, TAKE_PICTURE_NO);
        camera_channel_stop(oem_handle, cxt->snp_cxt.channel_bits);
    } else {
        camera_set_snp_req(oem_handle, TAKE_PICTURE_NO);
    }

    ret = cmr_preview_cancel_snapshot(cxt->prev_cxt.preview_handle,
                                      cxt->camera_id);
    if (ret) {
        CMR_LOGE("failed to cancel %ld", ret);
        goto exit;
    }

    if (camera_get_hdr_flag(cxt)) {
#ifdef OEM_HANDLE_HDR
        if (0 != cxt->ipm_cxt.frm_num) {
            cxt->ipm_cxt.frm_num = 0;
        }
#endif
        ret = camera_close_hdr(cxt);
        if (ret) {
            CMR_LOGE("failed to close hdr");
        }
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_CLEAR_HDR, &setting_param);
        if (ret) {
            CMR_LOGE("failed to clear hdr sem");
        }
    }

    cxt->snp_cxt.status = IDLE;
exit:
    sem_post(&cxt->snapshot_sm);
    CMR_LOGV("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_local_get_prev_rect(cmr_handle oem_handle,
                                   struct img_rect *param_ptr) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!param_ptr) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    ret = cmr_preview_get_prev_rect(cxt->prev_cxt.preview_handle,
                                    cxt->camera_id, param_ptr);
    CMR_LOGI("%d %d %d %d", param_ptr->start_x, param_ptr->start_y,
             param_ptr->width, param_ptr->height);
exit:
    return ret;
}

cmr_int camera_get_sensor_mode_info(cmr_handle oem_handle,
                                    struct sensor_mode_info *mode_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_exp_info *sensor_info = NULL;
    cmr_uint sensor_mode = SENSOR_MODE_MAX;
    cmr_uint i;

    if (!oem_handle || !mode_info) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    sensor_info =
        (struct sensor_exp_info *)malloc(sizeof(struct sensor_exp_info));
    if (!sensor_info) {
        CMR_LOGE("No mem!");
        ret = CMR_CAMERA_NO_MEM;
        goto exit;
    }
    ret = camera_get_sensor_info(cxt, cxt->camera_id, sensor_info);
    if (ret) {
        CMR_LOGE("get_sensor info failed!");
        ret = CMR_CAMERA_FAIL;
        goto exit;
    }

    for (i = SENSOR_MODE_COMMON_INIT; i < SENSOR_MODE_MAX; i++) {
        mode_info[i].trim_start_x = sensor_info->mode_info[i].trim_start_x;
        mode_info[i].trim_start_y = sensor_info->mode_info[i].trim_start_y;
        mode_info[i].trim_width = sensor_info->mode_info[i].trim_width;
        mode_info[i].trim_height = sensor_info->mode_info[i].trim_height;
    }

exit:
    if (sensor_info) {
        free(sensor_info);
        sensor_info = NULL;
    }
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_get_sensor_mode_trim(cmr_handle oem_handle,
                                    struct img_rect *sn_trim) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct sensor_exp_info *sensor_info = NULL;
    cmr_u32 sensor_mode = SENSOR_MODE_MAX;

    if (!oem_handle || !sn_trim) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    sensor_info =
        (struct sensor_exp_info *)malloc(sizeof(struct sensor_exp_info));
    if (!sensor_info) {
        CMR_LOGE("No mem!");
        ret = CMR_CAMERA_NO_MEM;
        goto exit;
    }
    ret = camera_get_sensor_info(cxt, cxt->camera_id, sensor_info);
    if (ret) {
        CMR_LOGE("get_sensor info failed!");
        ret = CMR_CAMERA_FAIL;
        goto exit;
    }

    ret = cmr_sensor_get_mode(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                              &sensor_mode);
    CMR_LOGI("camera_id =%d sns mode =%d", cxt->camera_id, sensor_mode);
    if (sensor_mode >= SENSOR_MODE_MAX) {
        // note:cmr_sensor_get_mode would not set parameter.so sensor_mode
        // ==SENSOR_MODE_MAX
        // then mode_info[sensor_mode] would overflow.
        CMR_LOGE("cmr_sensor_get_mode failed!");
        ret = CMR_CAMERA_FAIL;
        goto exit;
    }
    sn_trim->start_x = sensor_info->mode_info[sensor_mode].trim_start_x;
    sn_trim->start_y = sensor_info->mode_info[sensor_mode].trim_start_y;
    if (cxt->is_multi_mode == MODE_SBS) {
        sn_trim->width = sensor_info->mode_info[sensor_mode].scaler_trim.width;
        sn_trim->height =
            sensor_info->mode_info[sensor_mode].scaler_trim.height;
    } else {
        sn_trim->width = sensor_info->mode_info[sensor_mode].trim_width;
        sn_trim->height = sensor_info->mode_info[sensor_mode].trim_height;
    }
    CMR_LOGI("sensor x=%d y=%d w=%d h=%d", sn_trim->start_x, sn_trim->start_y,
             sn_trim->width, sn_trim->height);
exit:
    if (sensor_info) {
        free(sensor_info);
        sensor_info = NULL;
    }
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_uint camera_get_preview_angle(cmr_handle oem_handle) {
    cmr_uint ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = NULL;
    struct setting_cmd_parameter setting_param;

    if (!oem_handle) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    setting_cxt = &cxt->setting_cxt;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_PREVIEW_ANGLE, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get preview angle %ld", ret);
        goto exit;
    }
    ret = setting_param.cmd_type_value;
exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_uint camera_get_exif_info(cmr_handle oem_handle,
                              struct exif_info *exif_info) {
    cmr_uint ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = NULL;
    struct setting_cmd_parameter setting_param;
    EXIF_SPEC_PIC_TAKING_COND_T *exif_spec = NULL;

    if (!oem_handle) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    setting_cxt = &cxt->setting_cxt;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_EXIF_INFO, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get exif %ld", ret);
        goto exit;
    }
    exif_spec = setting_param.exif_all_info_ptr->spec_ptr->pic_taking_cond_ptr;
    exif_info->aperture = (float)exif_spec->ApertureValue.numerator /
                          (float)exif_spec->ApertureValue.denominator;
    exif_info->focus_distance = (float)exif_spec->FocalLength.numerator /
                                (float)exif_spec->FocalLength.denominator;

exit:
    CMR_LOGD("apet %f focus dist %f", exif_info->aperture,
             exif_info->focus_distance);
    return ret;
}

cmr_uint camera_get_result_exif_info(
    cmr_handle oem_handle,
    struct exif_spec_pic_taking_cond_tag *exif_pic_info) {
    cmr_uint ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = NULL;
    struct setting_cmd_parameter setting_param;

    if (!oem_handle || !exif_pic_info) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    setting_cxt = &cxt->setting_cxt;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_GET_EXIF_PIC_INFO, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get exif %ld", ret);
        goto exit;
    }
    memcpy(exif_pic_info, &setting_param.exif_pic_cond_info,
           sizeof(struct exif_spec_pic_taking_cond_tag));
exit:
    return ret;
}

cmr_int camera_local_get_zsl_info(cmr_handle oem_handle, cmr_uint *is_support,
                                  cmr_uint *max_width, cmr_uint *max_height) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct preview_zsl_info zsl_info;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!oem_handle || !is_support || !max_width || !max_height) {
        CMR_LOGE("error param");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    ret = cmr_preview_is_support_zsl(cxt->prev_cxt.preview_handle,
                                     cxt->camera_id, is_support);
    if (ret) {
        CMR_LOGE("failed to get zsl info %ld", ret);
        goto exit;
    }

    ret = cmr_preview_get_max_cap_size(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, max_width, max_height);
    if (ret) {
        CMR_LOGE("failed to get max cap size %ld", ret);
        goto exit;
    }

exit:
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_local_redisplay_data(
    cmr_handle oem_handle, cmr_s32 output_fd, cmr_uint output_addr,
    cmr_uint output_vir_addr, cmr_uint output_width, cmr_uint output_height,
    cmr_s32 input_fd, cmr_uint input_addr_y, cmr_uint input_addr_uv,
    cmr_uint input_vir_addr, cmr_uint input_width, cmr_uint input_height) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct img_frm src_img;
    struct img_frm dst_img;
    struct cmr_rot_param rot_param;
    struct img_rect rect;
    enum img_angle angle = IMG_ANGLE_0;
    cmr_uint img_len = (cmr_uint)(output_width * output_height);
    struct setting_context *setting_cxt;
    struct setting_cmd_parameter setting_param;

    if (!oem_handle) {
        CMR_LOGE("err, handle is null");
        goto exit;
    }
    CMR_LOGI("0x%lx %d", (cmr_uint)oem_handle, cxt->snp_cxt.cfg_cap_rot);
    setting_cxt = &cxt->setting_cxt;
    cmr_bzero(&src_img, sizeof(struct img_frm));
    cmr_bzero(&dst_img, sizeof(struct img_frm));
    rect.start_x = 0;
    rect.start_y = 0;
    rect.width = input_width;
    rect.height = input_height;
    src_img.size.width = input_width;
    src_img.size.height = input_height;
    src_img.fmt = IMG_DATA_TYPE_YUV420;
    cmr_grab_get_dcam_endian(&cxt->snp_cxt.data_endian, &src_img.data_end);
    src_img.fd = input_fd;
    src_img.addr_phy.addr_y = input_addr_y;
    src_img.addr_phy.addr_u = input_addr_uv;
    if (IMG_ANGLE_90 == cxt->snp_cxt.cfg_cap_rot ||
        IMG_ANGLE_270 == cxt->snp_cxt.cfg_cap_rot) {
        dst_img.size.width = output_height;
        dst_img.size.height = output_width;
        dst_img.fd = output_fd;
        dst_img.addr_phy.addr_y = output_addr + ((img_len * 3) >> 1);
        dst_img.addr_phy.addr_u = dst_img.addr_phy.addr_y + img_len;
    } else if (IMG_ANGLE_180 == cxt->snp_cxt.cfg_cap_rot) {
        dst_img.size.width = output_width;
        dst_img.size.height = output_height;
        dst_img.fd = output_fd;
        dst_img.addr_phy.addr_y = output_addr + ((img_len * 3) >> 1);
        dst_img.addr_phy.addr_u = dst_img.addr_phy.addr_y + img_len;
    } else {
        dst_img.size.width = output_width;
        dst_img.size.height = output_height;
        dst_img.fd = output_fd;
        dst_img.addr_phy.addr_y = output_addr;
        dst_img.addr_phy.addr_u = dst_img.addr_phy.addr_y + img_len;
    }
    dst_img.addr_phy.addr_v = 0;
    dst_img.fmt = IMG_DATA_TYPE_YUV420;
    cmr_grab_get_dcam_endian(&cxt->snp_cxt.data_endian, &dst_img.data_end);
    CMR_LOGI("data_end %d %d", cxt->snp_cxt.data_endian.y_endian,
             cxt->snp_cxt.data_endian.uv_endian);
    rect.start_x = 0;
    rect.start_y = 0;
    rect.width = input_width;
    rect.height = input_height;
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_ZOOM_PARAM,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get zoom param %ld", ret);
        goto exit;
    }
    if (setting_param.zoom_param.mode == ZOOM_LEVEL) {
        ret = camera_get_trim_rect(&rect, 0, &dst_img.size);
    } else {
        float zoomRatio = 1.0f;
        float dstRatio = (float)dst_img.size.width / (float)dst_img.size.height;
        ret = camera_get_trim_rect2(&rect, zoomRatio, dstRatio, input_width,
                                    input_height, IMG_ANGLE_0);
    }
    if (ret) {
        CMR_LOGE("failed to get trim %ld", ret);
        goto exit;
    }
    src_img.rect = rect;

#ifdef CAMERA_BRINGUP
    CMR_LOGD("cpp is not ok, sw scale crashed, so dont do scale here");
// camera_scale_down_software(&src_img, &dst_img);
#else
    CMR_LOGD("src_img with %d, height %d, dst_img with %d, height %d",
             src_img.size.width, src_img.size.height, dst_img.size.width,
             dst_img.size.height);
    ret = cmr_scale_start(cxt->scaler_cxt.scaler_handle, &src_img, &dst_img,
                          (cmr_evt_cb)NULL, NULL);
    if (ret) {
        CMR_LOGE("failed to start start %ld", ret);
        ret = -CMR_CAMERA_FAIL;
        goto exit;
    }
#endif

/*for redisplay scale debug*/
#if 0
	struct img_addr src_addr;
	src_addr.addr_y = input_vir_addr;
	src_addr.addr_u = input_vir_addr + src_img.size.width * src_img.size.height;
	camera_save_yuv_to_file(11111,
		IMG_DATA_TYPE_YUV420,
		src_img.size.width,
		src_img.size.height,
		&src_addr);
	struct img_addr dst_addr;
	dst_addr.addr_y = output_vir_addr;
	dst_addr.addr_u = output_vir_addr + dst_img.size.width * dst_img.size.height;
	camera_save_yuv_to_file(22222,
		IMG_DATA_TYPE_YUV420,
		dst_img.size.width,
		dst_img.size.height,
		&dst_addr);
#endif

    /* start roattion*/
    if (IMG_ANGLE_0 != cxt->snp_cxt.cfg_cap_rot) {
        if (IMG_ANGLE_90 == cxt->snp_cxt.cfg_cap_rot) {
            angle = IMG_ANGLE_270;
        } else if (IMG_ANGLE_270 == cxt->snp_cxt.cfg_cap_rot) {
            angle = IMG_ANGLE_90;
        } else {
            angle = cxt->snp_cxt.cfg_cap_rot;
        }
        rect.start_x = 0;
        rect.start_y = 0;
        rect.width = dst_img.size.width;
        rect.height = dst_img.size.height;
        src_img.fd = dst_img.fd;
        src_img.addr_phy.addr_y = dst_img.addr_phy.addr_y;
        src_img.addr_phy.addr_u = dst_img.addr_phy.addr_u;
        src_img.addr_phy.addr_v = 0;
        src_img.size.width = dst_img.size.width;
        src_img.size.height = dst_img.size.height;
        src_img.fmt = IMG_DATA_TYPE_YUV420;
        src_img.data_end = cxt->snp_cxt.data_endian;
        dst_img.fd = output_fd;
        dst_img.addr_phy.addr_y = output_addr;
        dst_img.addr_phy.addr_u = dst_img.addr_phy.addr_y + img_len;
        dst_img.data_end = cxt->snp_cxt.data_endian;
        rot_param.handle = cxt->rot_cxt.rotation_handle;
        rot_param.angle = angle;
        rot_param.src_img = src_img;
        rot_param.dst_img = dst_img;
        src_img.rect = rect;
        ret = cmr_rot(&rot_param);
        if (ret) {
            CMR_LOGI("failed to rotate %ld", ret);
            goto exit;
        }
    }
exit:
    ATRACE_END();
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_local_get_focus_point(cmr_handle oem_handle, cmr_s32 *point_x,
                                     cmr_s32 *point_y) {
    CMR_LOGD("E");

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    *point_x = cxt->focus_rect.x;
    *point_y = cxt->focus_rect.y;

    CMR_LOGD("X");
    return ret;
}

cmr_s32 camera_local_isp_sw_check_buf(cmr_handle oem_handle,
                                      cmr_uint *param_ptr) {
    CMR_LOGD("E");

    cmr_int is_using = 1;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;

    // is_using = isp_sw_check_buf(isp_cxt->isp_handle, param_ptr);

    CMR_LOGD("X");
    return is_using;
}

cmr_int camera_local_isp_sw_proc(cmr_handle oem_handle,
                                 struct soft_isp_frm_param *param_ptr) {

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    CMR_LOGD("E");

    param_ptr->af_status = cxt->is_focus;
    if (0 != cxt->focus_rect.x || 0 != cxt->focus_rect.y) {
        param_ptr->weightparam.sel_x = cxt->focus_rect.x * 960 / 1600;
        param_ptr->weightparam.sel_y = cxt->focus_rect.y * 960 / 1600;
    }
    CMR_LOGD("af status %d, f_rect x %d, y %d", param_ptr->af_status,
             param_ptr->weightparam.sel_x, param_ptr->weightparam.sel_y);
// ret = isp_sw_proc(isp_cxt->isp_handle, param_ptr);

exit:
    CMR_LOGD("X");
    return ret;
}

cmr_int camera_local_raw_proc(cmr_handle oem_handle,
                              struct raw_proc_param *param_ptr) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct sensor_ex_info *sns_ex_info_ptr = NULL;
    struct timespec ts;
    if (!oem_handle || !param_ptr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    CMR_LOGI("slice num %d avail height %d slice height %d",
             param_ptr->slice_num, param_ptr->src_avail_height,
             param_ptr->src_slice_height);
    CMR_LOGI(
        "src addr 0x%lx 0x%lx pattern %d,dst addr 0x%lx 0x%lx, src w %d, h %d"
        "dst w %d, h %d",
        param_ptr->src_frame.addr_phy.addr_y,
        param_ptr->src_frame.addr_phy.addr_u,
        param_ptr->src_frame.format_pattern,
        param_ptr->dst_frame.addr_phy.addr_y,
        param_ptr->dst_frame.addr_phy.addr_u, param_ptr->src_frame.size.width,
        param_ptr->src_frame.size.height, param_ptr->dst_frame.size.width,
        param_ptr->dst_frame.size.height);
    CMR_LOGI("fd: src.fd=0x%x, dst.fd=0x%x, dst2.fd=0x%x",
             param_ptr->src_frame.fd, param_ptr->dst_frame.fd,
             param_ptr->dst2_frame.fd);

    if (1 == param_ptr->slice_num) {
        struct ips_in_param in_param;
        struct ips_out_param out_param;

        in_param.oem_handle = oem_handle;
        in_param.alloc_cb = camera_malloc;
        in_param.free_cb = camera_free;

        in_param.src_frame.img_fmt = param_ptr->src_frame.fmt;
        in_param.src_frame.img_size.w = param_ptr->src_frame.size.width;
        in_param.src_frame.img_size.h = param_ptr->src_frame.size.height;
        in_param.src_frame.img_fd.y = param_ptr->src_frame.fd;
        in_param.src_frame.img_fd.u = param_ptr->src_frame.fd;
        in_param.src_frame.img_addr_phy.chn0 =
            param_ptr->src_frame.addr_phy.addr_y;
        in_param.src_frame.img_addr_phy.chn1 =
            param_ptr->src_frame.addr_phy.addr_u;
        in_param.src_frame.img_addr_vir.chn0 =
            param_ptr->src_frame.addr_vir.addr_y;
        in_param.src_frame.img_addr_vir.chn1 =
            param_ptr->src_frame.addr_vir.addr_u;
        //		in_param.src_frame.format_pattern =
        // param_ptr->src_frame.format_pattern;
        in_param.src_avail_height = param_ptr->src_avail_height;
        in_param.src_slice_height = param_ptr->src_slice_height;
        in_param.dst_frame.img_fmt = param_ptr->dst_frame.fmt;
        in_param.dst_frame.img_size.w = param_ptr->dst_frame.size.width;
        in_param.dst_frame.img_size.h = param_ptr->dst_frame.size.height;
        in_param.dst_frame.img_fd.y = param_ptr->dst_frame.fd;
        in_param.dst_frame.img_fd.u = param_ptr->dst_frame.fd;
        in_param.dst_frame.img_addr_phy.chn0 =
            param_ptr->dst_frame.addr_phy.addr_y;
        in_param.dst_frame.img_addr_phy.chn1 =
            param_ptr->dst_frame.addr_phy.addr_u;
        in_param.dst_frame.img_addr_vir.chn0 =
            param_ptr->dst_frame.addr_vir.addr_y;
        in_param.dst_frame.img_addr_vir.chn1 =
            param_ptr->dst_frame.addr_vir.addr_u;
        in_param.dst_slice_height = param_ptr->dst_slice_height;

        in_param.sbs_info.sbs_mode = param_ptr->sbs_info.sbs_mode;
        in_param.sbs_info.img_size.w = param_ptr->sbs_info.size.width;
        in_param.sbs_info.img_size.h = param_ptr->sbs_info.size.height;
        in_param.sensor_id = cxt->camera_id;
        struct sensor_mode_info *sensor_mode_info;
        cmr_u32 sn_mode = 0;
        if (cxt->is_multi_mode == MODE_SBS)
            sn_mode = 1;
        else
            cmr_sensor_get_mode(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                &sn_mode);
        // we think OEM has get sensor info and save it into sensor context,so
        // we get mode info from cxt
        sensor_mode_info = &cxt->sn_cxt.sensor_info.mode_info[sn_mode];
        SENSOR_MODE_FPS_T fps_info;
        ret = cmr_sensor_get_fps_info(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                      sn_mode, &fps_info);
        camera_copy_sensor_fps_info_to_isp(&in_param.sensor_fps, &fps_info);
        if (ret) {
            CMR_LOGE("get sensor fps info failed %ld", ret);
            goto exit;
        }
        in_param.resolution_info.sensor_size.w = sensor_mode_info->width;
        in_param.resolution_info.sensor_size.h = sensor_mode_info->height;
        in_param.resolution_info.line_time = sensor_mode_info->line_time;
        in_param.resolution_info.frame_line = sensor_mode_info->frame_line;
        in_param.resolution_info.size_index = sn_mode;
        in_param.resolution_info.crop.st_x = sensor_mode_info->trim_start_x;
        in_param.resolution_info.crop.st_y = sensor_mode_info->trim_start_y;
        in_param.resolution_info.crop.width = sensor_mode_info->trim_width;
        in_param.resolution_info.crop.height = sensor_mode_info->trim_height;
        in_param.resolution_info.fps.max_fps = in_param.sensor_fps.max_fps;
        in_param.resolution_info.fps.min_fps = in_param.sensor_fps.min_fps;
        in_param.resolution_info.sensor_max_size.w =
            cxt->sn_cxt.sensor_info.source_width_max;
        in_param.resolution_info.sensor_max_size.h =
            cxt->sn_cxt.sensor_info.source_height_max;
        in_param.resolution_info.sensor_output_size.w =
            sensor_mode_info->out_width;
        in_param.resolution_info.sensor_output_size.h =
            sensor_mode_info->out_height;
        CMR_LOGI("ips_in_param:sensor fps info: is_high_fps %d, "
                 "high_fps_skip_num is %d",
                 in_param.sensor_fps.is_high_fps,
                 in_param.sensor_fps.high_fps_skip_num);
        CMR_LOGI("ips_in_param:sensor fps info:	max_fps is %d, min_fps is %d",
                 in_param.sensor_fps.max_fps, in_param.sensor_fps.min_fps);
        CMR_LOGI("ips_in_param: sensor max w h, %d %d",
                 in_param.resolution_info.sensor_max_size.w,
                 in_param.resolution_info.sensor_max_size.h);
        CMR_LOGI("ips_in_param sensor output w h, %d %d",
                 in_param.resolution_info.sensor_output_size.w,
                 in_param.resolution_info.sensor_output_size.h);
        CMR_LOGI("ips_in_param: sensor crop startx start w h, %d %d %d %d",
                 in_param.resolution_info.crop.st_x,
                 in_param.resolution_info.crop.st_y,
                 in_param.resolution_info.crop.width,
                 in_param.resolution_info.crop.height);

        sns_ex_info_ptr = &cxt->sn_cxt.cur_sns_ex_info;
        if (NULL == sns_ex_info_ptr) {
            CMR_LOGE("sns_ex_info_ptr is null, It is impossible error!");
            ret = -CMR_CAMERA_INVALID_PARAM;
            goto exit;
        }
        if ((NULL == sns_ex_info_ptr->name) ||
            (NULL == sns_ex_info_ptr->sensor_version_info)) {
            ret = cmr_sensor_init_static_info(cxt);
            if (ret) {
                cmr_sensor_deinit_static_info(cxt);
                CMR_LOGE("init sensor static info failed %ld", ret);
                goto exit;
            }
        }
        in_param.resolution_info.max_gain = sns_ex_info_ptr->max_adgain;
        CMR_LOGI("ips_in_param:max_gain:%d ",
                 in_param.resolution_info.max_gain);

        ret = isp_proc_start(isp_cxt->isp_handle, &in_param, &out_param);

        if (ret) {
            CMR_LOGE("failed to start proc %ld", ret);
        }
    } else {
        struct ipn_in_param in_param;
        struct ips_out_param out_param;
        in_param.src_avail_height = param_ptr->src_avail_height;
        in_param.src_slice_height = param_ptr->src_slice_height;
        in_param.src_addr_phy.chn0 = param_ptr->src_frame.addr_phy.addr_y;
        in_param.src_addr_phy.chn1 = param_ptr->src_frame.addr_phy.addr_u;
        in_param.dst_addr_phy.chn0 = param_ptr->dst_frame.addr_phy.addr_y;
        in_param.dst_addr_phy.chn1 = param_ptr->dst_frame.addr_phy.addr_u;
        ret = isp_proc_next(isp_cxt->isp_handle, &in_param, &out_param);
        if (ret) {
            CMR_LOGE("failed to start proc %ld", ret);
        }
    }

    if (clock_gettime(CLOCK_REALTIME, &ts)) {
        CMR_LOGE("get time failed");
        goto exit;
    }
    ts.tv_nsec += ms2ns(100);
    if (ts.tv_nsec > 1000000000) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
    }

    if (cxt->is_multi_mode == MODE_SBS) {
        CMR_LOGI("start wait raw proc");
        ret = sem_timedwait(&cxt->sbs_sync_sm, &ts);
        CMR_LOGI("ret %ld", ret);
    }
exit:
    ATRACE_END();
    return ret;
}

cmr_int camera_local_fd_start(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGV("fd %d", cxt->fd_on_off);
    if (PREVIEWING ==
        cmr_preview_get_status(cxt->prev_cxt.preview_handle, cxt->camera_id)) {
        ret = cmr_preview_ctrl_facedetect(cxt->prev_cxt.preview_handle,
                                          cxt->camera_id, cxt->fd_on_off);
        if (ret) {
            CMR_LOGE("failed to fd ctrl %ld", ret);
        }
    }
    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_local_set_param(cmr_handle oem_handle, enum camera_param_type id,
                               cmr_uint param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CHECK_HANDLE_VALID(oem_handle);

    switch (id) {
    case CAMERA_PARAM_FOCUS_RECT:
        CMR_LOGI("set focus rect 0x%lx", param);
        ret = cmr_focus_set_param(cxt->focus_cxt.focus_handle, cxt->camera_id,
                                  id, (void *)param);
        break;
    case CAMERA_PARAM_AF_MODE:
        CMR_LOGI("set focus af mode 0x%lx", param);
        ret = cmr_focus_set_param(cxt->focus_cxt.focus_handle, cxt->camera_id,
                                  id, (void *)param);
        break;
    case CAMERA_PARAM_ZOOM: {
        float zoom_factor = 1.0;
        const float EPSINON = 0.0001f;
        struct cmr_zoom_param *zoom_param = NULL;
        cmr_uint zoom_factor_changed = 0;
        if (param) {
            zoom_param = (struct cmr_zoom_param *)param;
            if (zoom_param->mode == ZOOM_INFO) {
                ret = cmr_preview_get_zoom_factor(cxt->prev_cxt.preview_handle,
                                                  cxt->camera_id, &zoom_factor);
                if (ret) {
                    CMR_LOGE("fail to get zoom factor  %ld", ret);
                } else {
                    if (fabs(zoom_param->zoom_info.zoom_ratio - zoom_factor) >
                        EPSINON) {
                        zoom_factor_changed = 1;
                    }
                }
            }
        }

        ret = cmr_preview_update_zoom(cxt->prev_cxt.preview_handle,
                                      cxt->camera_id,
                                      (struct cmr_zoom_param *)param);
        if (ret) {
            CMR_LOGE("failed to update zoom %ld", ret);
        }
        ret = camera_set_setting(oem_handle, id, param);
        if (ret) {
            CMR_LOGE("failed to set camera setting of zoom %ld", ret);
        }
        if (zoom_factor_changed) {
            ret = cmr_set_zoom_factor_to_isp(oem_handle,
                                             &zoom_param->zoom_info.zoom_ratio);
            if (ret) {
                CMR_LOGE("failed to set zoom factor to isp  %ld", ret);
            }
        }
        cxt->zoom_ratio = zoom_param->zoom_info.zoom_ratio;
        break;
    }
    case CAMERA_PARAM_ISO:
        cxt->setting_cxt.iso_value = param;
        ret = camera_set_setting(oem_handle, id, param);
        break;
    default:
        ret = camera_set_setting(oem_handle, id, param);
        break;
    }
    if (ret) {
        CMR_LOGE("failed to set param %ld", ret);
    }
exit:
    return ret;
}

cmr_int camera_local_start_focus(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = cmr_pre_flash_notice_flash(cxt->setting_cxt.setting_handle);
    ret = cmr_af_start_notice_focus(cxt->focus_cxt.focus_handle);
    ret = cmr_focus_start(cxt->focus_cxt.focus_handle, cxt->camera_id);

    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_local_cancel_focus(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = cmr_setting_cancel_notice_flash(cxt->setting_cxt.setting_handle);
    ret = cmr_af_cancel_notice_focus(cxt->focus_cxt.focus_handle);
    ret = cmr_focus_stop(cxt->focus_cxt.focus_handle, cxt->camera_id, 1);

    CMR_LOGV("done %ld", ret);
    return ret;
}

cmr_int camera_local_transfer_caf_to_af(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = cmr_transfer_caf_to_af(cxt->focus_cxt.focus_handle);

    return ret;
}

cmr_int camera_local_transfer_af_to_caf(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = cmr_transfer_af_to_caf(cxt->focus_cxt.focus_handle);

    return ret;
}

cmr_int camera_local_pre_flash(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_cmd_parameter setting_param;

    /*start preflash*/
    setting_param.camera_id = cxt->camera_id;
    setting_param.ctrl_flash.capture_mode.capture_mode = CAMERA_NORMAL_MODE;
    setting_param.ctrl_flash.flash_type = FLASH_CLOSE;
    ret = cmr_pre_flash_notice_flash(cxt->setting_cxt.setting_handle);
    ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                            SETTING_SET_PRE_LOWFLASH, &setting_param);
    if (ret) {
        CMR_LOGE("failed to open flash");
    }

    return ret;
}

cmr_int camera_local_get_viewangle(cmr_handle oem_handle,
                                   struct sensor_view_angle *view_angle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct sensor_exp_info exp_info;

    if (!oem_handle || !view_angle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    cxt = (struct camera_context *)oem_handle;
    ret = cmr_sensor_get_info(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                              &exp_info);
    if (ret) {
        CMR_LOGE("failed to get sensor info %ld", ret);
        goto exit;
    }

    view_angle->horizontal_val = exp_info.view_angle.horizontal_val;
    view_angle->vertical_val = exp_info.view_angle.vertical_val;
exit:
    return ret;
}
cmr_int camera_local_set_preview_buffer(cmr_handle oem_handle,
                                        cmr_uint src_phy_addr,
                                        cmr_uint src_vir_addr, cmr_s32 fd) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct sensor_exp_info exp_info;
    if (!oem_handle || !fd || !src_vir_addr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cxt = (struct camera_context *)oem_handle;
    ret = cmr_preview_set_preview_buffer(cxt->prev_cxt.preview_handle,
                                         cxt->camera_id, src_phy_addr,
                                         src_vir_addr, fd);
    if (ret) {
        CMR_LOGE("failed to set preview buffer %ld", ret);
        goto exit;
    }
exit:
    return ret;
}
cmr_int camera_local_set_video_buffer(cmr_handle oem_handle,
                                      cmr_uint src_phy_addr,
                                      cmr_uint src_vir_addr, cmr_s32 fd) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct sensor_exp_info exp_info;
    if (!oem_handle || !fd || !src_vir_addr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cxt = (struct camera_context *)oem_handle;
    ret = cmr_preview_set_video_buffer(cxt->prev_cxt.preview_handle,
                                       cxt->camera_id, src_phy_addr,
                                       src_vir_addr, fd);
    if (ret) {
        CMR_LOGE("failed to set video buffer %ld", ret);
        goto exit;
    }
exit:
    return ret;
}
cmr_int camera_local_set_zsl_buffer(cmr_handle oem_handle,
                                    cmr_uint src_phy_addr,
                                    cmr_uint src_vir_addr, cmr_s32 fd) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct sensor_exp_info exp_info;
    if (!oem_handle || !fd || !src_vir_addr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cxt = (struct camera_context *)oem_handle;
    ret =
        cmr_preview_set_zsl_buffer(cxt->prev_cxt.preview_handle, cxt->camera_id,
                                   src_phy_addr, src_vir_addr, fd);
    if (ret) {
        CMR_LOGE("failed to set zsl buffer %ld", ret);
        goto exit;
    }
exit:
    return ret;
}

cmr_int camera_local_set_video_snapshot_buffer(cmr_handle oem_handle,
                                               cmr_uint src_phy_addr,
                                               cmr_uint src_vir_addr,
                                               cmr_s32 fd) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct frm_info chn_data;
    cmr_u32 buffer_size = 0;
    if (!oem_handle || !fd || !src_vir_addr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    camera_get_iso_value(oem_handle);

    cxt = (struct camera_context *)oem_handle;
    CMR_LOGI("in video w=%d h=%d cap w=%d h=%d",
             cxt->prev_cxt.actual_video_size.width,
             cxt->prev_cxt.actual_video_size.height,
             cxt->snp_cxt.actual_capture_size.width,
             cxt->snp_cxt.actual_capture_size.height);

    if (TAKE_PICTURE_NEEDED == camera_get_snp_req((cmr_handle)cxt)) {
        if (cxt->snp_cxt.actual_capture_size.width *
                cxt->snp_cxt.actual_capture_size.height <
            cxt->prev_cxt.actual_video_size.width *
                cxt->prev_cxt.actual_video_size.height)
            buffer_size = cxt->snp_cxt.actual_capture_size.width *
                          cxt->snp_cxt.actual_capture_size.height;
        else
            buffer_size = cxt->prev_cxt.actual_video_size.width *
                          cxt->prev_cxt.actual_video_size.height;
        cmr_bzero(&chn_data, sizeof(struct frm_info));
        cmr_copy(&chn_data, &cxt->prev_cxt.video_cur_chn_data,
                 sizeof(struct frm_info));
        chn_data.fd = fd;
        chn_data.yaddr = src_phy_addr;
        chn_data.uaddr = src_phy_addr + buffer_size;
        chn_data.vaddr = 0;
        chn_data.yaddr_vir = src_vir_addr;
        chn_data.uaddr_vir = src_vir_addr + buffer_size;
        chn_data.vaddr_vir = 0;
        ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                        SNAPSHOT_EVT_CHANNEL_DONE,
                                        (void *)&chn_data);
    } else {
        CMR_LOGE("snapshot is not ready");
    }
    if (ret) {
        CMR_LOGE("failed to set snapshot buffer %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGI("out");
    ATRACE_END();
    return ret;
}

cmr_int camera_local_set_zsl_snapshot_buffer(cmr_handle oem_handle,
                                             cmr_uint src_phy_addr,
                                             cmr_uint src_vir_addr,
                                             cmr_s32 fd) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct frm_info chn_data;
    cmr_u32 buffer_size = 0;
    cmr_int need_pause = 0;

    if (!oem_handle || !src_vir_addr) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }

    cxt = (struct camera_context *)oem_handle;
    camera_local_zsl_snapshot_need_pause(oem_handle, &need_pause);
    if (TAKE_PICTURE_NEEDED == camera_get_snp_req((cmr_handle)cxt)) {
        camera_take_snapshot_step(CMR_STEP_CAP_S);
        buffer_size = cxt->snp_cxt.actual_capture_size.width *
                      cxt->snp_cxt.actual_capture_size.height;
        cmr_bzero(&chn_data, sizeof(struct frm_info));
        cmr_copy(&chn_data, &cxt->snp_cxt.cur_chn_data,
                 sizeof(struct frm_info));
        chn_data.yaddr = src_phy_addr;
        chn_data.uaddr = src_phy_addr + buffer_size;
        chn_data.vaddr = 0;
        chn_data.yaddr_vir = src_vir_addr;
        chn_data.uaddr_vir = src_vir_addr + buffer_size;
        chn_data.vaddr_vir = 0;
        chn_data.fd = fd;

#ifdef PERFORMANCE_OPTIMIZATION
        // update postprocess params
        struct img_frm img_frame;
        memset(&img_frame, 0, sizeof(struct img_frm));
        img_frame.fmt = chn_data.fmt;
        img_frame.buf_size = buffer_size * 3 / 2; // ? or buffer_size * 3 /2
        img_frame.rect.start_x = 0;
        img_frame.rect.start_y = 0;
        img_frame.rect.width = cxt->snp_cxt.actual_capture_size.width;
        img_frame.rect.height = cxt->snp_cxt.actual_capture_size.height;
        img_frame.size.width = cxt->snp_cxt.actual_capture_size.width;
        img_frame.size.height = cxt->snp_cxt.actual_capture_size.height;
        img_frame.addr_phy.addr_y = src_phy_addr;
        img_frame.addr_phy.addr_u = src_phy_addr + buffer_size;
        img_frame.addr_phy.addr_v = 0;
        img_frame.addr_vir.addr_y = src_vir_addr;
        img_frame.addr_vir.addr_u = src_vir_addr + buffer_size;
        img_frame.addr_vir.addr_v = 0;
        img_frame.data_end.y_endian = 1;  // ?
        img_frame.data_end.uv_endian = 2; // ?
        img_frame.fd = fd;
        CMR_LOGI("in src_phy_addr 0x%lx src_vir_addr 0x%lx, img_frame.fd 0x%x",
                 src_phy_addr, src_vir_addr, img_frame.fd);

        zsl_snp_update_post_proc_param(cxt->snp_cxt.snapshot_handle,
                                       (void *)&img_frame);
#endif
        ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                        SNAPSHOT_EVT_CHANNEL_DONE,
                                        (void *)&chn_data);
        if (need_pause) {
            camera_set_discard_frame(cxt, 1);
            ret = cmr_preview_receive_data(cxt->prev_cxt.preview_handle,
                                           cxt->camera_id, PREVIEW_CHN_PAUSE,
                                           (void *)&chn_data);
        }
        camera_capture_post_proc(cxt, cxt->camera_id);
    } else {
        CMR_LOGE("snapshot is not ready");
    }
    if (ret) {
        CMR_LOGE("failed to set snapshot buffer %ld", ret);
        goto exit;
    }
exit:
    CMR_LOGI("out");
    ATRACE_END();
    return ret;
}

cmr_int camera_local_zsl_snapshot_need_pause(cmr_handle oem_handle,
                                             cmr_int *flag) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct cmr_path_capability capability;

    if (!oem_handle || !flag) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGV("in");
    cxt = (struct camera_context *)oem_handle;

    camera_channel_path_capability(oem_handle, &capability);
    //	*flag = capability.capture_pause;
    *flag = 0;
    CMR_LOGV("out flag %ld", *flag);
exit:
    return ret;
}

cmr_int camera_local_normal_snapshot_need_pause(cmr_handle oem_handle,
                                                cmr_int *flag) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt;
    struct cmr_path_capability capability;

    if (!oem_handle || !flag) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    CMR_LOGV("in");
    cxt = (struct camera_context *)oem_handle;

    camera_channel_path_capability(oem_handle, &capability);
    *flag = capability.capture_pause;
    CMR_LOGV("out flag %ld", *flag);
exit:
    return ret;
}

void camera_local_start_burst_notice(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    uint32_t caf_switch = 0;
    isp_ioctl(isp_cxt->isp_handle, ISP_CTRL_BURST_NOTICE, &caf_switch);
}

void camera_local_end_burst_notice(cmr_handle oem_handle) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    uint32_t caf_switch = 1;
    isp_ioctl(isp_cxt->isp_handle, ISP_CTRL_BURST_NOTICE, &caf_switch);
}

cmr_int
camera_isp_set_sensor_info_to_af(cmr_handle oem_handle,
                                 struct cmr_af_aux_sensor_info *sensor_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct af_aux_sensor_info_t aux_sensor_info;
    if (cxt && isp_cxt && sensor_info) {
        memset(&aux_sensor_info, 0, sizeof(aux_sensor_info));
        switch (sensor_info->type) {
        case CAMERA_AF_ACCELEROMETER:
            aux_sensor_info.type = AF_ACCELEROMETER;
            aux_sensor_info.gsensor_info.timestamp =
                sensor_info->gsensor_info.timestamp;
            aux_sensor_info.gsensor_info.vertical_up =
                sensor_info->gsensor_info.vertical_up;
            aux_sensor_info.gsensor_info.vertical_down =
                sensor_info->gsensor_info.vertical_down;
            aux_sensor_info.gsensor_info.horizontal =
                sensor_info->gsensor_info.horizontal;
            ret = isp_ioctl(isp_cxt->isp_handle, ISP_CTRL_SET_AUX_SENSOR_INFO,
                            &aux_sensor_info);
            break;
        case CAMERA_AF_MAGNETIC_FIELD:
            aux_sensor_info.type = AF_MAGNETIC_FIELD;
            break;
        case CAMERA_AF_GYROSCOPE:
            aux_sensor_info.type = AF_GYROSCOPE;
            aux_sensor_info.gyro_info.timestamp =
                sensor_info->gyro_info.timestamp;
            aux_sensor_info.gyro_info.x = sensor_info->gyro_info.x;
            aux_sensor_info.gyro_info.y = sensor_info->gyro_info.y;
            aux_sensor_info.gyro_info.z = sensor_info->gyro_info.z;
            ret = isp_ioctl(isp_cxt->isp_handle, ISP_CTRL_SET_AUX_SENSOR_INFO,
                            &aux_sensor_info);
            break;
        case CAMERA_AF_LIGHT:
            aux_sensor_info.type = AF_LIGHT;
            break;
        case CAMERA_AF_PROXIMITY:
            aux_sensor_info.type = AF_PROXIMITY;
            break;
        default:
            CMR_LOGE("NO this sensor type: %d ", sensor_info->type);
            ret = CMR_CAMERA_INVALID_PARAM;
        }
    } else {
        CMR_LOGE("input param is null!");
        ret = CMR_CAMERA_INVALID_PARAM;
    }
    return ret;
}

static cmr_uint
camera_copy_sensor_fps_info_to_isp(struct isp_sensor_fps_info *out_isp_fps,
                                   SENSOR_MODE_FPS_T *in_fps) {
    if (NULL == in_fps || NULL == out_isp_fps) {
        CMR_LOGE("input param or out param is null!");
        return CMR_CAMERA_FAIL;
    }
    out_isp_fps->mode = in_fps->mode;
    out_isp_fps->max_fps = in_fps->max_fps;
    out_isp_fps->min_fps = in_fps->min_fps;
    out_isp_fps->is_high_fps = in_fps->is_high_fps;
    out_isp_fps->high_fps_skip_num = in_fps->high_fps_skip_num;

    return CMR_CAMERA_SUCCESS;
}

static cmr_uint
camera_copy_sensor_ex_info_to_isp(struct isp_sensor_ex_info *out_isp_sn_ex_info,
                                  struct sensor_ex_info *in_sn_ex_info) {
    if (NULL == in_sn_ex_info || NULL == out_isp_sn_ex_info) {
        CMR_LOGE("input param or out param is null!");
        return CMR_CAMERA_FAIL;
    }
    out_isp_sn_ex_info->f_num = in_sn_ex_info->f_num;
    out_isp_sn_ex_info->focal_length = in_sn_ex_info->focal_length;
    out_isp_sn_ex_info->max_fps = in_sn_ex_info->max_fps;
    out_isp_sn_ex_info->max_adgain = in_sn_ex_info->max_adgain;
    out_isp_sn_ex_info->ois_supported = in_sn_ex_info->ois_supported;
    out_isp_sn_ex_info->pdaf_supported = in_sn_ex_info->pdaf_supported;
    out_isp_sn_ex_info->exp_valid_frame_num =
        in_sn_ex_info->exp_valid_frame_num;
    out_isp_sn_ex_info->clamp_level = in_sn_ex_info->clamp_level;
    out_isp_sn_ex_info->adgain_valid_frame_num =
        in_sn_ex_info->adgain_valid_frame_num;
    out_isp_sn_ex_info->preview_skip_num = in_sn_ex_info->preview_skip_num;
    out_isp_sn_ex_info->capture_skip_num = in_sn_ex_info->capture_skip_num;
    out_isp_sn_ex_info->name = in_sn_ex_info->name;
    out_isp_sn_ex_info->sensor_version_info =
        in_sn_ex_info->sensor_version_info;
    out_isp_sn_ex_info->pos_dis.up2hori = in_sn_ex_info->pos_dis.up2hori;
    out_isp_sn_ex_info->pos_dis.hori2down = in_sn_ex_info->pos_dis.hori2down;

    return CMR_CAMERA_SUCCESS;
}

static cmr_uint camera_sensor_color_to_isp_color(cmr_u32 *isp_color,
                                                 cmr_u32 sensor_color) {
    /*    switch (sensor_color) {
        case SENSOR_IMAGE_PATTERN_RAWRGB_GR:
            *isp_color = COLOR_ORDER_GR;
            break;
        case SENSOR_IMAGE_PATTERN_RAWRGB_R:
            *isp_color = COLOR_ORDER_RG;
            break;
        case SENSOR_IMAGE_PATTERN_RAWRGB_B:
            *isp_color = COLOR_ORDER_BG;
            break;
        case SENSOR_IMAGE_PATTERN_RAWRGB_GB:
            *isp_color = COLOR_ORDER_GB;
            break;
        default:
            CMR_LOGE("sensor color maybe error,not found in definition.set isp "
                     "color to COLOR_ORDER_GR");
            *isp_color = COLOR_ORDER_GR;
            break;
        } */
    return CMR_CAMERA_SUCCESS;
}

cmr_int camera_preview_get_isp_yimg(cmr_handle oem_handle, cmr_u32 camera_id,
                                    struct isp_yimg_info *yimg) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle || NULL == yimg) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
    isp_param.camera_id = camera_id;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_YIMG_INFO, &isp_param);
    if (ret) {
        CMR_LOGE("get isp y stat error %ld", ret);
        goto exit;
    }
    memcpy(yimg, &isp_param.isp_yimg, sizeof(struct isp_yimg_info));
    CMR_LOGI("%d", isp_param.isp_yimg.lock[0]);

exit:
    return ret;
}

cmr_int camera_preview_set_yimg_to_isp(cmr_handle oem_handle, cmr_u32 camera_id,
                                       struct yimg_info *yimg) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle || NULL == yimg) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
    isp_param.camera_id = camera_id;
    isp_param.cmd_value = (cmr_uint)yimg;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_SET_PREVIEW_YIMG, &isp_param);

exit:
    return ret;
}

cmr_int camera_preview_set_yuv_to_isp(cmr_handle oem_handle, cmr_u32 camera_id,
                                      struct yuv_info_t *yuv) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle || NULL == yuv) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
    isp_param.camera_id = camera_id;
    isp_param.cmd_value = (cmr_uint)yuv;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_SET_PREVIEW_YUV, &isp_param);

exit:
    return ret;
}

cmr_int cmr_sensor_init_static_info(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = NULL;
    struct sensor_ex_info *sns_ex_info_ptr = NULL;
    SENSOR_VAL_T val;
    CHECK_HANDLE_VALID(oem_handle);
    cxt = (struct camera_context *)oem_handle;
    sns_ex_info_ptr = &cxt->sn_cxt.cur_sns_ex_info;
    cmr_bzero(sns_ex_info_ptr, sizeof(struct sensor_ex_info));
    if (cxt->sn_cxt.inited && cxt->sn_cxt.sensor_handle) {
        val.type = SENSOR_VAL_TYPE_GET_STATIC_INFO;
        val.pval = sns_ex_info_ptr;
        ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                               SENSOR_ACCESS_VAL, (cmr_uint)&val);

    } else {
        ret = CMR_CAMERA_FAIL;
    }
    if (ret) {
        CMR_LOGE("oem not init or get sensor static info failed, we set "
                 "sn_ex_info to zero. ");
    }
    return ret;
}

cmr_int cmr_sensor_deinit_static_info(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = NULL;
    SENSOR_VAL_T val;
    CHECK_HANDLE_VALID(oem_handle);
    cxt = (struct camera_context *)oem_handle;
    cmr_bzero(&cxt->sn_cxt.cur_sns_ex_info, sizeof(struct sensor_ex_info));
    return ret;
}

cmr_int cmr_get_sensor_max_fps(cmr_handle oem_handle, cmr_u32 camera_id,
                               cmr_u32 *max_fps) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = NULL;
    struct sensor_ex_info *sns_ex_info_ptr = NULL;
    CHECK_HANDLE_VALID(oem_handle);
    CHECK_HANDLE_VALID(max_fps);
    cxt = (struct camera_context *)oem_handle;
    sns_ex_info_ptr = &cxt->sn_cxt.cur_sns_ex_info;
    if (NULL == sns_ex_info_ptr) {
        CMR_LOGE("sns_ex_info_ptr is null, It is impossible error!");
        return -CMR_CAMERA_INVALID_PARAM;
    }

    if ((NULL == sns_ex_info_ptr->name) ||
        (NULL == sns_ex_info_ptr->sensor_version_info)) {
        ret = cmr_sensor_init_static_info(cxt);
    }
    if (ret) {
        CMR_LOGE(
            "failed to init sensor static info,we set max fps to default: %d ",
            *max_fps);
        *max_fps = 30;
    } else {
        *max_fps = sns_ex_info_ptr->max_fps;
    }
    return ret;
}

cmr_int cmr_get_blur_covered_type(cmr_handle oem_handle, cmr_s32 *type) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_context *isp_cxt = &cxt->isp_cxt;
    struct common_isp_cmd_param isp_param;
    cmr_u32 bv;
    cmr_s32 isp_rt;

    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_ADGAIN_EXP, &isp_param);
    bv = isp_param.isp_adgain.bv;
    CMR_LOGD("bv=%d", bv);

    // Just 4 bv test tmp
    char min_bv[PROPERTY_VALUE_MAX];
    char max_bv[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.cam.minbv.test", min_bv, "500");
    property_get("persist.vendor.cam.maxbv.test", max_bv, "700");

    cmr_u32 Min_bv;
    cmr_u32 Max_bv;
    Min_bv = atoi(min_bv);
    Max_bv = atoi(max_bv);

    if (bv < Min_bv) {
        *type = BLUR_TIPS_NEED_LIGHT;
        CMR_LOGD("depth cannot work under this bv, (%d~%d), type=%d", Min_bv,
                 Max_bv, *type);
        return ret;
    }
    if (bv < Max_bv && BLUR_TIPS_NEED_LIGHT == *type) {
        CMR_LOGD("still need more light");
        return ret;
    }

    if (cxt->is_focus) {
        *type = BLUR_TIPS_UNABLED;
        CMR_LOGD("is focusing");
        return ret;
    }

    /* isp_rt = isp_sw_get_bokeh_status(isp_cxt->isp_handle);
    if (0 == isp_rt) {
        *type = BLUR_TIPS_OK;
    } else if (1 == isp_rt) {
        *type = BLUR_TIPS_FURTHER;
    } else if (2 == isp_rt) {
        *type = BLUR_TIPS_CLOSE;
    } else {
        *type = BLUR_TIPS_UNABLED;
    } */

    CMR_LOGD("type =%d", *type);

    return ret;
}

cmr_int cmr_set_zoom_factor_to_isp(cmr_handle oem_handle, float *zoomFactor) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = NULL;
    struct isp_context *isp_cxt = NULL;
    CHECK_HANDLE_VALID(oem_handle);

    cxt = (struct camera_context *)oem_handle;
    isp_cxt = &cxt->isp_cxt;
    if (NULL == isp_cxt->isp_handle || NULL == zoomFactor) {
        CMR_LOGE("isp handle or zoomFactor is null!");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return ret;
    }
    ret = isp_ioctl(isp_cxt->isp_handle, ISP_CTRL_SET_DZOOM_FACTOR, zoomFactor);
    if (ret) {
        CMR_LOGE("isp_ioctl-ISP_CTRL_SET_DZOOM_FACTOR failed:zoomFactor is %f",
                 *zoomFactor);
    }
    return ret;
}

cmr_int cmr_get_sensor_vcm_step(cmr_handle oem_handle, cmr_u32 camera_id,
                                cmr_u32 *vcm_step) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;

    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        goto exit;
    }
    cmr_bzero(&isp_param, sizeof(struct common_isp_cmd_param));
    isp_param.camera_id = cxt->camera_id;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_VCM_INFO, &isp_param);
    if (ret) {
        CMR_LOGE("get isp vcm step error %ld", ret);
        goto exit;
    }
    CMR_LOGD("isp_param.vcm_step = %d", isp_param.vcm_step);
    *vcm_step = isp_param.vcm_step;

exit:
    return ret;
}

cmr_int camera_local_set_sensor_close_flag(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    SENSOR_VAL_T val;

    val.type = SENSOR_VAL_TYPE_SET_SENSOR_CLOSE_FLAG;
    val.pval = NULL;
    ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                           SENSOR_ACCESS_VAL, (cmr_uint)&val);
    if (ret) {
        CMR_LOGE("failed to set sensor close flag");
    }
    return ret;
}

cmr_int camera_hdr_set_ev(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_sn_cmd_param sn_param;
    struct common_isp_cmd_param isp_param;
    struct setting_cmd_parameter setting_param;
    memset(&setting_param, 0, sizeof(setting_param));

    if (1 == camera_get_hdr_flag(cxt)) {
        ret = cmr_ipm_pre_proc(cxt->ipm_cxt.hdr_handle);
        if (ret) {
            CMR_LOGE("failed to ipm pre proc, %ld", ret);
        } else {
            ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                    SETTING_CTRL_HDR, &setting_param);
            if (ret) {
                CMR_LOGE("failed to wait hdr ev effect");
            }
        }
    }
    if (cxt->is_vendor_hdr) {
        cxt->cap_cnt = 0;
        struct sensor_exp_info sensor_info;
        ret = camera_get_sensor_info(cxt, cxt->camera_id, &sensor_info);
        if (ret) {
            CMR_LOGE("get_sensor info failed!");
            ret = CMR_CAMERA_FAIL;
            goto exit;
        }

        if (SENSOR_IMAGE_FORMAT_RAW == sensor_info.image_format) {
            isp_param.cmd_value = OEM_EV_LEVEL_1;
            ret = camera_isp_ioctl(oem_handle, COM_ISP_SET_HDR,
                                   (void *)&isp_param);
        } else {
            sn_param.cmd_value = OEM_EV_LEVEL_1;
            ret = camera_sensor_ioctl(oem_handle, COM_SN_SET_HDR_EV,
                                      (void *)&sn_param);
        }
    }
exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_3dnr_set_ev(cmr_handle oem_handle, cmr_u32 enable) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct common_isp_cmd_param isp_param;
    struct sensor_exp_info sensor_info;
    ret = camera_get_sensor_info(cxt, cxt->camera_id, &sensor_info);
    if (ret) {
        CMR_LOGE("get_sensor info failed!");
        ret = CMR_CAMERA_FAIL;
        goto exit;
    }

    if (SENSOR_IMAGE_FORMAT_RAW == sensor_info.image_format) {
        isp_param.cmd_value = enable;
        ret =
            camera_isp_ioctl(oem_handle, COM_ISP_SET_3DNR, (void *)&isp_param);
    }

exit:
    CMR_LOGI("done %ld", ret);
    return ret;
}

cmr_int camera_sw_3dnr_info_cfg(cmr_handle oem_handle,
                                struct sprd_img_3dnr_param *threednr_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_LOGD("call camera_sw_3dnr_info_cfg, %d, %d", camera_get_3dnr_flag(cxt),
             camera_get_sw_3dnr_flag(cxt));
    if ((1 == camera_get_3dnr_flag(cxt)) && (1 == camera_get_sw_3dnr_flag(cxt)))
        ret = cmr_grab_sw_3dnr_cfg(cxt->grab_cxt.grab_handle, threednr_info);

    return ret;
}

cmr_int camera_get_jpeg_param_info(cmr_handle oem_handle,
                                   struct jpeg_param *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct setting_cmd_parameter setting_param;

    setting_param.camera_id = cxt->camera_id;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_JPEG_QUALITY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get image quality %ld", ret);
        goto exit;
    }
    param->quality = setting_param.cmd_type_value;

    ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_THUMB_QUALITY, &setting_param);
    if (ret) {
        CMR_LOGE("failed to get thumb quality %ld", ret);
        goto exit;
    }
    param->thumb_quality = setting_param.cmd_type_value;

    // thum_size is not use for now
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_THUMB_SIZE,
                            &setting_param);
    if (ret) {
        CMR_LOGE("failed to get thumb size %ld", ret);
        goto exit;
    }
    param->thum_size = setting_param.size_param;

    CMR_LOGV("quality=%d, thumb_quality=%d, thum_size: %d %d", param->quality,
             param->thumb_quality, param->thum_size.width,
             param->thum_size.height);
exit:
    return ret;
}

cmr_int camera_local_start_capture(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;
    cmr_u32 flash_status = 0;
    struct sprd_img_capture_param capture_param;

    cmr_bzero(&capture_param, sizeof(capture_param));
    capture_param.type = DCAM_CAPTURE_START;
    camera_local_snapshot_is_need_flash(oem_handle, cxt->camera_id,
                                        &flash_status);
    if (flash_status > 0)
        capture_param.type = DCAM_CAPTURE_START_WITH_FLASH;
    else if (1 == camera_get_hdr_flag(cxt))
        capture_param.type = DCAM_CAPTURE_START_HDR;
    else if (1 == camera_get_3dnr_flag(cxt))
        capture_param.type = DCAM_CAPTURE_START_3DNR;
/*
else if (cxt->is_multi_mode == MODE_BOKEH) {
    if (cxt->is_yuv_callback_mode) {
        capture_param.type = DCAM_CAPTURE_START_WITH_TIMESTAMP;
        capture_param.timestamp = snp_cxt->cap_need_time_stamp;
    } else {
        return ret;
    }
}
*/
#ifdef SHARKL2_DCAM_ISP
    ret = cmr_grab_start_capture(cxt->grab_cxt.grab_handle, capture_param);
    if (ret) {
        CMR_LOGE("cmr_grab_start_capture failed");
        goto exit;
    }
#endif

exit:
    return ret;
}

cmr_int camera_local_stop_capture(cmr_handle oem_handle) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

#ifdef SHARKL2_DCAM_ISP
    ret = cmr_grab_stop_capture(cxt->grab_cxt.grab_handle);
    if (ret) {
        CMR_LOGE("cmr_grab_start_capture failed");
        goto exit;
    }
#endif
exit:
    return ret;
}

void camera_set_oem_multimode(multiCameraMode camera_mode) {
    CMR_LOGD("camera_mode %d", camera_mode);
    is_multi_camera_mode_oem = camera_mode;
}

void camera_set_oem_masterid(uint8_t master_id) {
    CMR_LOGD("master id %d", master_id);
    master_id_oem = master_id;
}

cmr_int camera_local_get_cover(cmr_handle oem_handle,
                               struct dual_sensor_luma_info *luma_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    SENSOR_VAL_T val;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGD("E id=%d", cxt->camera_id);
    if (cxt->camera_id < 2) {

        struct camera_context *cxt = (struct camera_context *)oem_handle;
        struct isp_context *isp_cxt = &cxt->isp_cxt;
        struct common_isp_cmd_param isp_param;

        ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_ADGAIN_EXP,
                               &isp_param);
        luma_info->main_gain = isp_param.isp_adgain.adgain;
        cmr_u32 exp_time = isp_param.isp_adgain.exp_time;
        luma_info->main_luma = isp_param.isp_adgain.bv;
        CMR_LOGD("tadgain=%d, exp_time=%d, bv=%d", luma_info->main_gain,
                 exp_time, luma_info->main_luma);
    } else {
        val.type = SENSOR_VAL_TYPE_GET_BV;
        val.pval = (void *)&luma_info->sub_luma;
        ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                               SENSOR_ACCESS_VAL, (cmr_uint)&val);
    }
    CMR_LOGD("done");
    return ret;
}

cmr_int camera_stream_ctrl(cmr_handle oem_handle, cmr_u32 on_off) {
    cmr_int ret = 0;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct video_start_param param_ptr;

    memset(&param_ptr, 0, sizeof(struct video_start_param));
    CMR_LOGD("stream on/off(1/0) %u, multi_mode %d", on_off,
             cxt->is_multi_mode);
    if (cxt->is_multi_mode != MODE_SBS) {
        if (cxt->camera_id != SENSOR_MAIN2)
            CMR_LOGW("should not direct open sensors except for SENSOR_MAIN2");
        if (on_off == 0)
            ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                   SENSOR_STREAM_OFF, 0);
        else
            ret = cmr_sensor_ioctl(cxt->sn_cxt.sensor_handle, cxt->camera_id,
                                   SENSOR_STREAM_ON, 0);
    }
    CMR_LOGD("done");
    return ret;
}

cmr_int cmr_get_isp_af_fullscan(cmr_handle oem_handle,
                                struct isp_af_fullscan_info *af_fullscan_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    void *isp_param_ptr = (void *)af_fullscan_info;

    CMR_LOGI("E");
    ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_GET_FULLSCAN_INFO,
                    isp_param_ptr);

    return ret;
}

cmr_int cmr_set_af_pos(cmr_handle oem_handle, cmr_u32 af_pos) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGI("E af_pos =%d", af_pos);

    ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_SET_AF_POS,
                    (void *)&af_pos);

    return ret;
}

cmr_int cmr_set_3a_bypass(cmr_handle oem_handle, cmr_u32 value) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CMR_LOGI("E af_pos =%d", value);
    ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_SET_AF_BYPASS,
                    (void *)&value);
    if (value == 1) {
        cmr_u32 ae = 2;
        cmr_u32 awb = 3;
        ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_AEAWB_BYPASS,
                        (void *)&ae);
        ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_AEAWB_BYPASS,
                        (void *)&awb);
    } else {
        ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_AEAWB_BYPASS,
                        (void *)&value);
    }
    return ret;
}

cmr_int cmr_get_ae_fps(cmr_handle oem_handle, cmr_u32 *ae_fps) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    void *isp_param_ptr = (void *)ae_fps;

    ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_GET_FPS, isp_param_ptr);

    return ret;
}

cmr_int cmr_set_snapshot_timestamp(cmr_handle oem_handle, int64_t timestamp) {
    cmr_int ret = CMR_CAMERA_SUCCESS;

    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct snapshot_context *snp_cxt = &cxt->snp_cxt;

    snp_cxt->cap_need_time_stamp = timestamp;

    CMR_LOGI("E timestamp=%lld", snp_cxt->cap_need_time_stamp);

    return ret;
}

cmr_int cmr_get_microdepth_param(cmr_handle oem_handle, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_GET_MICRODEPTH_PARAM,
                    param);

    return ret;
}

cmr_int cmr_set_microdepth_debug_info(cmr_handle oem_handle, void *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    ret = isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_SET_MICRODEPTH_DEBUG_INFO,
                    param);
    return ret;
}

cmr_int camera_local_get_sensor_format(cmr_handle oem_handle,
                                       cmr_u32 *sensor_format) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CHECK_HANDLE_VALID(cxt);
    struct sensor_context *sn_cxt = NULL;
    sn_cxt = &(cxt->sn_cxt);
    CHECK_HANDLE_VALID(sn_cxt);

    ret = cmr_sensor_get_info(sn_cxt->sensor_handle, cxt->camera_id,
                              &(sn_cxt->sensor_info));
    if (ret)
        CMR_LOGE("fail to get sensor info ret %ld", ret);
    else
        *sensor_format = sn_cxt->sensor_info.image_format;
    CMR_LOGD("sensor_format is %d", *sensor_format);
    return ret;
}

cmr_int camera_local_get_tuning_param(cmr_handle oem_handle,
                                      struct tuning_param_info *tuning_info) {
    CMR_LOGD("E");

    cmr_int ret = CMR_CAMERA_SUCCESS;
    tuning_info->gain = 0;
    tuning_info->shutter = 0;
    tuning_info->bv = 0;
    tuning_info->pos = 0;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct isp_adgain_exp_info adgain_exp_info;
    (struct isp_awb_info) tuning_info->awb_info;

    camera_get_tuning_info(oem_handle, &adgain_exp_info);

    tuning_info->gain = adgain_exp_info.adgain;
    tuning_info->shutter = adgain_exp_info.exp_time;
    tuning_info->bv = adgain_exp_info.bv;
    isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_GET_AWB_GAIN,
              (void *)&(tuning_info->awb_info));
    isp_ioctl(cxt->isp_cxt.isp_handle, ISP_CTRL_GET_AF_POS,
              (void *)&(tuning_info->pos));

    CMR_LOGD("X");
    return ret;
}

cmr_int camera_local_set_capture_fb(cmr_handle oem_handle, cmr_u32 *on) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    CHECK_HANDLE_VALID(cxt);
    cxt->blur_facebeauty_flag = *on;
    CMR_LOGD("blur_facebeauty_flag %d", cxt->blur_facebeauty_flag);

    return ret;
}

cmr_int camera_local_reprocess_yuv_for_jpeg(cmr_handle oem_handle,
                                            enum takepicture_mode cap_mode,
                                            cmr_uint yaddr, cmr_uint yaddr_vir,
                                            cmr_uint fd) {
    ATRACE_BEGIN(__FUNCTION__);

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    struct preview_context *prev_cxt;
    struct snapshot_param snp_param;
    struct common_sn_cmd_param param;
    struct setting_cmd_parameter setting_param;
    struct frm_info frame_info;
    struct frm_info *frm_data = &frame_info;
    cmr_uint buffer_size;
    cmr_int flash_status = FLASH_CLOSE;
    cmr_u32 sec = 0;
    cmr_u32 usec = 0;
    cmr_s32 sm_val = 0;

#if 0
    if (!oem_handle) {
        CMR_LOGE("error handle");
        goto exit;
    }
    camera_take_snapshot_step(CMR_STEP_TAKE_PIC);
    prev_cxt = &cxt->prev_cxt;

    sem_getvalue(&cxt->share_path_sm, &sm_val);
    if (0 != sm_val) {
        sem_destroy(&cxt->share_path_sm);
        sem_init(&cxt->share_path_sm, 0, 0);
        CMR_LOGI("re-initialize share_path_sm");
    }

    sem_wait(&cxt->access_sm);
    ret = cmr_grab_get_cap_time(cxt->grab_cxt.grab_handle, &sec, &usec);
    sem_post(&cxt->access_sm);

    cmr_bzero(frm_data, sizeof(struct frm_info));
    frm_data->channel_id = camera_get_channel_id(cxt->snp_cxt.channel_bits);
    frm_data->frame_id = CMR_CAP0_ID_BASE;
    frm_data->frame_real_id = 0;
    frm_data->base = CMR_CAP0_ID_BASE;
    frm_data->height = cxt->snp_cxt.request_size.height;
    frm_data->base = CMR_CAP0_ID_BASE;
    frm_data->fmt = IMG_DATA_TYPE_YUV420;
    frm_data->yaddr = yaddr;
    frm_data->yaddr_vir = yaddr_vir;
    frm_data->fd = fd;
    frm_data->sec = sec;
    frm_data->usec = usec;
    frm_data->zoom_ratio = 1000;

    buffer_size =
        cxt->snp_cxt.request_size.width * cxt->snp_cxt.request_size.height;
    frm_data->uaddr = yaddr + buffer_size;
    frm_data->vaddr = 0;
    frm_data->uaddr_vir = yaddr_vir + buffer_size;
    frm_data->vaddr_vir = 0;

    ret = camera_get_snapshot_param(oem_handle, &snp_param);

    // check snp size
    if (snp_param.post_proc_setting.snp_size.height == 0 ||
        snp_param.post_proc_setting.snp_size.width == 0 ||
        snp_param.post_proc_setting.actual_snp_size.height == 0 ||
        snp_param.post_proc_setting.actual_snp_size.width == 0) {
        cmr_bzero(&setting_param, sizeof(setting_param));
        setting_param.camera_id = cxt->camera_id;
        CMR_LOGI("camera id: %ld", setting_param.camera_id);

        /*get snapshot size*/
        ret = cmr_setting_ioctl(cxt->setting_cxt.setting_handle,
                                SETTING_GET_CAPTURE_SIZE, &setting_param);
        if (ret) {
            CMR_LOGE("failed to get capture size %ld", ret);
            goto exit;
        }
        snp_param.post_proc_setting.actual_snp_size = setting_param.size_param;
        snp_param.post_proc_setting.snp_size = setting_param.size_param;
    }
    if (snp_param.channel_id == 0 || snp_param.channel_id >= GRAB_CHANNEL_MAX) {
        snp_param.channel_id = frm_data->channel_id;
    }
    CMR_LOGI("chn id: %d ,picture: width:%d, height:%d", snp_param.channel_id,
             cxt->snp_cxt.post_proc_setting.actual_snp_size.width,
             cxt->snp_cxt.post_proc_setting.actual_snp_size.height);

    snp_param.post_proc_setting.chn_out_frm[0].addr_vir.addr_y =
        frm_data->yaddr_vir;
    snp_param.post_proc_setting.chn_out_frm[0].addr_vir.addr_u =
        frm_data->uaddr_vir;
    snp_param.post_proc_setting.chn_out_frm[0].addr_vir.addr_v =
        frm_data->vaddr_vir;
    snp_param.post_proc_setting.chn_out_frm[0].addr_phy.addr_y =
        frm_data->yaddr;
    snp_param.post_proc_setting.chn_out_frm[0].addr_phy.addr_u =
        frm_data->uaddr;
    snp_param.post_proc_setting.chn_out_frm[0].addr_phy.addr_v =
        frm_data->vaddr;
    snp_param.post_proc_setting.chn_out_frm[0].fd = frm_data->fd;
    snp_param.post_proc_setting.chn_out_frm[0].fmt = frm_data->fmt;

    snp_param.post_proc_setting.mem[0].target_yuv.addr_vir.addr_y =
        frm_data->yaddr_vir;
    snp_param.post_proc_setting.mem[0].target_yuv.addr_vir.addr_u =
        frm_data->uaddr_vir;
    snp_param.post_proc_setting.mem[0].target_yuv.addr_vir.addr_v =
        frm_data->vaddr_vir;
    snp_param.post_proc_setting.mem[0].target_yuv.addr_phy.addr_y =
        frm_data->yaddr;
    snp_param.post_proc_setting.mem[0].target_yuv.addr_phy.addr_u =
        frm_data->uaddr;
    snp_param.post_proc_setting.mem[0].target_yuv.addr_phy.addr_v =
        frm_data->vaddr;
    snp_param.post_proc_setting.mem[0].target_yuv.fd = frm_data->fd;
    snp_param.post_proc_setting.mem[0].target_yuv.fmt = frm_data->fmt;
    camera_set_snp_req((cmr_handle)cxt, TAKE_PICTURE_NEEDED);

    ret = cmr_snapshot_post_proc(cxt->snp_cxt.snapshot_handle, &snp_param);

    // because of only hdr plus(normal pic) need to backup the normal pic.
    // so directly use HDR post-snapshot.
    ret = cmr_snapshot_receive_data(cxt->snp_cxt.snapshot_handle,
                                    SNAPSHOT_EVT_HDR_DONE, frm_data);
    camera_post_share_path_available(oem_handle);
#endif

exit:
    CMR_LOGV("done %ld", ret);
    ATRACE_END();
    return ret;
}

cmr_int camera_set_thumb_yuv_proc(cmr_handle oem_handle,
                                  struct snp_thumb_yuv_param *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_LOGI("E");

    ret = cmr_snapshot_thumb_yuv_proc(cxt->snp_cxt.snapshot_handle, param);

    if (ret) {
        CMR_LOGE("snp_thumb_yuv_proc failed.");
    }

    return ret;
}

cmr_int camera_get_blur_covered_type(cmr_handle oem_handle, cmr_s32 *param) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    CMR_LOGI("E");

    ret = cmr_get_blur_covered_type(cxt, param);

    if (ret) {
        CMR_LOGE("get blur covered type failed.");
    }

    return ret;
}
cmr_int camera_set_snp_face_detect_value(cmr_handle oem_handle,
                                         cmr_u16 is_enable) {
    ATRACE_BEGIN(__FUNCTION__);

    CMR_LOGD("E");

    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    if (!oem_handle) {
        CMR_LOGE("in parm error");
        ret = -CMR_CAMERA_INVALID_PARAM;
        return ret;
    }

    cxt->prev_cxt.snp_fd_enable = is_enable;

    CMR_LOGD("X");

    return ret;
}

cmr_int camera_set_hdr_ev(cmr_handle oem_handle, void *data) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct camera_context *cxt = (struct camera_context *)oem_handle;

    if (!data || !cxt) {
        CMR_LOGI("don't need set hdr ev");
        return ret;
    }
    memcpy((void *)&cxt->snp_cxt.hdr_ev[0], data,
           ((HDR_CAP_NUM)-1) * sizeof(float));
    CMR_LOGI("hdr ev %f, %f", cxt->snp_cxt.hdr_ev[0], cxt->snp_cxt.hdr_ev[1]);
    return ret;
}

cmr_int camera_ipm_process(cmr_handle oem_handle, void *data) {
    struct camera_context *cxt = (struct camera_context *)oem_handle;
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct frm_info *frame = (struct frm_info *)data;
    struct ipm_context *ipm_cxt = &cxt->ipm_cxt;
    struct ipm_frame_in ipm_in_param;
    struct ipm_frame_out imp_out_param;
    struct img_frm *img_frame = (struct img_frm *)data;
    struct setting_context *setting_cxt = &cxt->setting_cxt;
    struct setting_cmd_parameter setting_param;
    cmr_uint is_filter;
    CMR_LOGD("E");
    CMR_LOGI("camera_ipm_process fd =%p",img_frame->fd);
    setting_param.camera_id = cxt->camera_id;
    ret = cmr_setting_ioctl(setting_cxt->setting_handle, SETTING_GET_APPMODE,
                            &setting_param);
    if(ret)
        CMR_LOGE("failed to get app mode");
    if(setting_param.cmd_type_value == CAMERA_MODE_NIGHT_PHOTO) {
        if(cxt->night_cxt.is_authorized) {
            ret= cxt->night_cxt.ipmpro_process(oem_handle, data);
            if(ret) {
                CMR_LOGE("failed to process ipmpro %ld",ret);
            }
        }
        goto exit;
    }

    CHECK_HANDLE_VALID(oem_handle);
    CHECK_HANDLE_VALID(data);
    CHECK_HANDLE_VALID(ipm_cxt);

    is_filter = cxt->snp_cxt.filter_type;
    if (cxt->nr_flag || is_filter) {
        cmr_bzero(&ipm_in_param, sizeof(ipm_in_param));
        cmr_bzero(&imp_out_param, sizeof(imp_out_param));

        ipm_cxt->frm_num++;
        ipm_in_param.src_frame = *img_frame;
        ipm_in_param.private_data = (void *)cxt;
        imp_out_param.dst_frame = *img_frame;
        // do cnr
        if (cxt->nr_flag)
            ret = ipm_transfer_frame(ipm_cxt->cnr_handle, &ipm_in_param, NULL);
        if (ret) {
            CMR_LOGE("failed to do cnr process %ld", ret);
            goto exit;
        }

        CMR_LOGD("img_frame width = %d , height = %d ",
                 imp_out_param.dst_frame.size.width,
                 imp_out_param.dst_frame.size.height);
        // do filter:
        if (is_filter) {
            struct setting_cmd_parameter setting_param;
            struct setting_context *setting_cxt = &cxt->setting_cxt;

            /*get rotation*/
            cmr_bzero(&setting_param, sizeof(setting_param));
            setting_param.camera_id = cxt->camera_id;
            ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            CAMERA_PARAM_GET_SENSOR_ORIENTATION, &setting_param);
            if (ret) {
                 CMR_LOGE("failed to get enc rotation %ld", ret);
            }
            ipm_in_param.orientation = setting_param.cmd_type_value;
            /*get flip*/
            cmr_bzero(&setting_param, sizeof(setting_param));
            setting_param.camera_id = cxt->camera_id;
            ret = cmr_setting_ioctl(setting_cxt->setting_handle,
                            SETTING_GET_FLIP_ON, &setting_param);
            if (ret) {
                 CMR_LOGE("failed to get flip %ld", ret);
            }
            ipm_in_param.flip_on = setting_param.cmd_type_value;
            imp_out_param.private_data = (void *)(cxt->snp_cxt.filter_type);
            ret = ipm_transfer_frame(ipm_cxt->filter_handle, &ipm_in_param,
                                     &imp_out_param);
        }
        cmr_snapshot_memory_flush(cxt->snp_cxt.snapshot_handle, img_frame);
        if (ret) {
            CMR_LOGE("failed to do filter process %ld", ret);
            goto exit;
        }
    }
exit:
    CMR_LOGI("X");
    return ret;
}

cmr_int camera_get_bv_info(cmr_handle oem_handle, cmr_u32 *bv_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct common_isp_cmd_param isp_param;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_ADGAIN_EXP, &isp_param);
    if (ret) {
          CMR_LOGE("failed to get isp param %d",ret);
          return ret;
    }
    *bv_info = isp_param.isp_adgain.bv;
    return ret;
}

cmr_int camera_get_ct_info(cmr_handle oem_handle, cmr_u32 *ct_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct common_isp_cmd_param isp_param;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_COL_TEM, &isp_param);
    if (ret) {
          CMR_LOGE("failed to get isp param %d",ret);
          return ret;
    }
    *ct_info = isp_param.isp_cur_ct;
    return ret;
}

cmr_int camera_get_iso_info(cmr_handle oem_handle, cmr_u32 *iso_info) {
    cmr_int ret = CMR_CAMERA_SUCCESS;
    struct common_isp_cmd_param isp_param;
    ret = camera_isp_ioctl(oem_handle, COM_ISP_GET_CUR_SENS, &isp_param);
    if (ret) {
          CMR_LOGE("failed to get isp param %d",ret);
          return ret;
    }
    *iso_info = isp_param.isp_cur_iso;
    return ret;
}
