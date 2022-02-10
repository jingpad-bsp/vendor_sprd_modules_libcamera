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
#ifndef _CMR_OEM_H_
#define _CMR_OEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cmr_common.h"
#include "SprdOEMCamera.h"
#include "cmr_cvt.h"
#include "cmr_focus.h"
#include "cmr_grab.h"
#include "cmr_ipm.h"
#include "cmr_isptool.h"
#include "cmr_jpeg.h"
#include "cmr_mem.h"
#include "cmr_msg.h"
#include "cmr_preview.h"
#include "cmr_sensor.h"
#include "cmr_setting.h"
#include "cmr_snapshot.h"
#include "isp_app.h"
#include "jpeg_exif_header.h"
#ifdef CONFIG_CAMERA_MM_DVFS_SUPPORT
#include "cmr_mm_dvfs.h"
#endif
#ifdef CONFIG_FACE_BEAUTY
#include "sprd_facebeauty_adapter.h"
#endif
#include <hardware/enhance.h>
#define ISP_LSC_BUF_SIZE (32 * 1024)
#define ISP_LSC_BUF_NUM 1
#define ISP_ANTI_FLICKER_BUF_SIZE (750 * 1024) /* 3120*4*61 */
#define ISP_ANTI_FLICKER_BUF_NUM 1
#define ISP_B4AWB_BUF_CNT 2
#define ISP_B4AWB_BUF_SIZE 640 * 480 * 2

#define ISP_CLOSE_3DNR_TIMEOUT 2 /*sec*/
/* should only define just one of the following two */
#define MIRROR_FLIP_ROTATION_BY_JPEG 1
//#define MIRROR_FLIP_BY_ISP                         1

struct debug_context {
    cmr_handle debug_handle;
    cmr_u32 inited;
    cmr_u32 dump_bits;
    char tags[128];
    struct buffer_cfg buff_cfg;
};

struct grab_context {
    cmr_handle grab_handle;
    /*	struct process_status    proc_status;*/
    cmr_handle caller_handle[GRAB_CHANNEL_MAX];
    cmr_u32 skip_number[GRAB_CHANNEL_MAX];
    cmr_u32 inited;
};

struct sensor_context {
    cmr_handle sensor_handle;
    cmr_u32 cur_id;
    cmr_u32 inited;
    struct sensor_if sn_if;
    struct sensor_exp_info sensor_info;
    struct sensor_exp_info sensor_info_slv;
    EXIF_SPEC_PIC_TAKING_COND_T exif_info;
    struct sensor_ex_info cur_sns_ex_info;
    struct sensor_4in1_info info_4in1;
};
struct prev_threednr_info {
    struct img_frm frm_preview;
    struct img_frm frm_smallpreview;
    struct img_frm frm_video;
    struct camera_frame_type framtype;
    unsigned long camera_id;
    void *caller_handle;
    struct frm_info data;
};
struct isp_context {
    cmr_handle isp_handle;
    cmr_handle caller_handle;
    cmr_u32 isp_state; // 0 for preview, 1 for post process;
    cmr_u32 inited;
    cmr_u32 width_limit;
    cmr_u32 is_work;
    cmr_u32 is_snapshot;
    cmr_u32 is_real_bokeh;
};

struct jpeg_context {
    cmr_handle jpeg_handle;
    cmr_u32 jpeg_state;
    cmr_u32 inited;
    cmr_handle enc_caller_handle;
    cmr_handle dec_caller_handle;
    struct jpeg_param param;
};

struct scaler_context {
    cmr_handle scaler_handle;
    cmr_u32 scale_state;
    cmr_u32 inited;
    cmr_handle caller_handle;
};

struct rotation_context {
    cmr_handle rotation_handle;
    cmr_u32 rot_state;
    cmr_u32 inited;
    cmr_handle caller_handle;
};

struct ipm_context {
    cmr_handle ipm_handle;
    cmr_handle hdr_handle;
    cmr_handle fdr_handle;
    cmr_handle filter_handle;
    cmr_handle uvde_handle;
    cmr_handle yde_handle;
    cmr_handle refocus_handle;
    cmr_handle threednr_handle;
    cmr_handle handle_4in1;
    cmr_handle ai_scene_handle;
    cmr_handle cnr_handle;
    cmr_handle ee_handle;
    cmr_handle dre_handle;
    cmr_u32 inited;
    cmr_u32 frm_num;
    cmr_u32 hdr_num;
    cmr_u32 threednr_num;
    cmr_u32 padding;
    cmr_u32 filter_inited;
    cmr_u32 ai_scene_inited;
    cmr_u32 cnr_inited;
    cmr_u32 ee_inited;
    cmr_u32 dre_inited;
    struct ipm_version hdr_version;
    cmr_u32 four_in_one_inited;
};

struct ipm_pro_context {
    cmr_handle ipm_pro_handle;
    cmr_handle mfnr_handle;
    cmr_handle cnr_pro_handle;
    cmr_handle dre_pro_handle;
    cmr_u32 pro_inited;
    cmr_u32 frm_num;
    cmr_u32 mfnr_num;
    cmr_u32 cnr_pro_inited;
    cmr_u32 dre_pro_inited;
};

struct ipm_fdr_context {
    cmr_handle ipm_fdr_handle;
    cmr_handle fdr_handle;
    cmr_handle cnr_fdr_handle;
    cmr_handle ee_fdr_handle;
    cmr_u32 fdr_inited;
    cmr_u32 fdr_num;
    cmr_u32 frm_num;
    cmr_u32 cnr_fdr_inited;
    cmr_u32 ee_fdr_inited;
};


struct nightpro_context {
    cmr_uint is_authorized;
    void *sw_handle;
    int (*sw_open)(cmr_handle oem_handle);
    int (*sw_process)(cmr_handle oem_handle,
                                 struct image_sw_algorithm_buf *src_buf,
                                 struct image_sw_algorithm_buf *dst_buf);
    int (*sw_close)(cmr_handle oem_handle);
    int (*ipmpro_init)(cmr_handle oem_handle);
    int (*ipmpro_deinit)(cmr_handle oem_handle);
    int (*ipmpro_process)(cmr_handle oem_handle, void *data);
};

struct camera_core_context {
    cmr_uint is_authorized;
    void *sw_handle;
    int (*sw_open)(cmr_handle oem_handle);
    int (*sw_process)(cmr_handle oem_handle, struct img_frm *in_frame,
                                 struct img_frm *out_frame);
    int (*sw_close)(cmr_handle oem_handle);
    int (*ipmcore_init)(cmr_handle oem_handle);
    int (*ipmcore_deinit)(cmr_handle oem_handle);
    int (*ipmcore_process)(cmr_handle oem_handle, void *data);
};

struct preview_context {
    cmr_handle preview_handle;
    cmr_u32 inited;
    cmr_u32 skip_num;
    cmr_u32 preview_eb;
    cmr_u32 preview_channel_id;
    cmr_u32 preview_sn_mode;
    struct img_data_end data_endian;
    cmr_u32 video_eb;
    cmr_u32 video_channel_id;
    cmr_u32 video_sn_mode;
    struct img_data_end video_data_endian;
    cmr_u32 channel0_eb;
    cmr_u32 channel0_chn_id;
    cmr_u32 channel0_sn_mode;
    struct img_data_end channel0_endian;
    cmr_u32 channel1_eb;
    cmr_u32 channel1_chn_id;
    cmr_u32 channel1_sn_mode;
    struct img_data_end channel1_endian;
    cmr_u32 channel2_eb;
    cmr_u32 channel2_chn_id;
    cmr_u32 channel2_sn_mode;
    struct img_data_end channel2_endian;
    cmr_u32 channel3_eb;
    cmr_u32 channel3_chn_id;
    cmr_u32 channel3_sn_mode;
    struct img_data_end channel3_endian;
    cmr_u32 channel4_eb;
    cmr_u32 channel4_chn_id;
    cmr_u32 channel4_sn_mode;
    struct img_data_end channel4_endian;
    cmr_u32 snapshot_eb;
    cmr_uint status;
    struct img_size size;
    struct img_size video_size;
    struct img_size actual_video_size;
    struct frm_info video_cur_chn_data;
    struct img_rect rect;
};

struct snapshot_context {
    cmr_handle snapshot_handle;
    cmr_u32 inited;
    cmr_u32 snapshot_sn_mode;
    cmr_u32 skip_num;
    cmr_u32 channel_id;
    cmr_u32 is_hdr;
    cmr_u32 is_fdr;
    cmr_u32 is_3dnr;
    cmr_u32 sprd_3dnr_type;
    cmr_u32 total_num;
    cmr_u32 snp_mode;
    cmr_u32 is_cfg_rot_cap;
    cmr_u32 cfg_cap_rot;
    cmr_u32 status;
    cmr_u32 zsl_frame;
    cmr_uint filter_type;
    cmr_uint is_req_snp;
    cmr_u8 is_super;
    // fix burst called cmr_grab_start_capture repeatedly
    cmr_u32 start_capture_flag;
    cmr_s64 cap_time_stamp;
    cmr_s64 cap_need_time_stamp;
    float hdr_ev[HDR_CAP_NUM];
    struct img_size request_size;
    struct img_size capture_align_size;
    struct img_size actual_capture_size;
    struct frm_info cur_frm_info;
    struct snp_proc_param post_proc_setting;
    struct img_data_end data_endian;
    struct frm_info cur_chn_data;
    struct touch_coordinate touch_xy;
    struct isp_blkpm_t fdr_tuning_param;
    struct isp_blkpm_t ee_tuning_param;
    struct common_isp_info fdr_exif_log;
    void *fdr_ae_info;
    void *ae_common_info;
};

struct focus_context {
    cmr_handle focus_handle;
    cmr_u32 inited;
    cmr_u32 padding;
};

struct setting_context {
    cmr_handle setting_handle;
    cmr_u32 inited;
    cmr_u32 is_active;
    cmr_u32 is_auto_iso;
    cmr_uint iso_value;
};
#ifdef CONFIG_CAMERA_MM_DVFS_SUPPORT
struct mm_dvfs_context {
    cmr_handle mm_dvfs_handle;
    cmr_u32 inited;
};
#endif

struct camera_settings {
    cmr_u32 preview_width;
    cmr_u32 preview_height;
    cmr_u32 snapshot_width;
    cmr_u32 snapshot_height;
    cmr_u32 focal_len;
    cmr_u32 brightness;
    cmr_u32 contrast;
    cmr_u32 effect;
    cmr_u32 expo_compen;
    cmr_u32 wb_mode;
    cmr_u32 saturation;
    cmr_u32 sharpness;
    cmr_u32 scene_mode;
    cmr_u32 flash;
    cmr_u32 auto_flash_status;
    cmr_u32 night_mode;
    cmr_u32 flicker_mode;
    cmr_u32 focus_rect;
    cmr_u32 af_mode;
    cmr_u32 iso;
    cmr_u32 luma_adapt;
    cmr_u32 video_mode;
    cmr_u32 frame_rate;
    cmr_u32 sensor_mode;
    cmr_u32 auto_exposure_mode;
    cmr_u32 preview_env;
    /*snapshot param*/
    cmr_u32 quality;
    cmr_u32 thumb_quality;
    cmr_u32 set_encode_rotation;
    struct img_size thum_size;
    cmr_u32 cap_rot;
    cmr_u32 is_cfg_rot_cap;
    cmr_u32 is_dv; /*1 for DV, 0 for DC*/
    cmr_u32 is_hdr;
    cmr_u32 total_cap_num;
    cmr_u32 is_andorid_zsl;

    /*all the above value will be set as 0xFFFFFFFF after inited*/
    cmr_u32 set_end;
    struct cmr_zoom_param zoom_param;
    uint32_t isp_alg_timeout;
    sem_t isp_alg_sem;
    pthread_mutex_t isp_alg_mutex;
};

typedef enum {
    JPEG_ENCODE_MIN = 0,
    JPEG_ENCODING,
    JPEG_ENCODE_DONE,
    JPEG_ENCODE_MAX,
} jpg_encode_status;

struct camera_context {
    /*for the device OEM layer owned*/
    struct grab_context grab_cxt;
    struct sensor_context sn_cxt;
    struct isp_context isp_cxt;
    struct jpeg_context jpeg_cxt;
    struct scaler_context scaler_cxt;
    struct rotation_context rot_cxt;
    struct preview_context prev_cxt;
    struct snapshot_context snp_cxt;
    struct focus_context focus_cxt;
    struct ipm_context ipm_cxt;
    struct ipm_pro_context ipm_pro_cxt;
    struct ipm_fdr_context ipm_fdr_cxt;
    struct setting_context setting_cxt;
   struct debug_context dbg_cxt;
#ifdef CONFIG_CAMERA_MM_DVFS_SUPPORT
    struct mm_dvfs_context mm_dvfs_cxt;
#endif
    /*for the workflow management*/
    cmr_u32 camera_id;
    cmr_u32 err_code;
    camera_cb_of_type camera_cb;
    void *client_data;
    cmr_u32 inited;
    cmr_u32 camera_mode;
    cmr_uint is_discard_frm;
    sem_t fdr_flag_sm;
    sem_t hdr_sync_sm;
    sem_t hdr_flag_sm;
    sem_t ai_scene_flag_sm;
    sem_t cnr_flag_sm;
    sem_t dre_flag_sm;
    sem_t dre_pro_flag_sm;
    sem_t threednr_flag_sm;
    sem_t threednr_proc_sm;
    sem_t cnr_fdr_flag_sm;
    sem_t ee_fdr_flag_sm;
    sem_t filter_sm;
    sem_t share_path_sm;
    sem_t access_sm;
    sem_t sbs_sync_sm;
    cmr_uint share_path_sm_flag;
    cmr_handle init_thread;
    cmr_int facing;

    /*callback thread to hal*/
    cmr_handle prev_cb_thr_handle;
    cmr_handle snp_cb_thr_handle;
    cmr_handle snp_secondary_thr_handle;
    cmr_handle snp_send_raw_image_handle;
    /* for fdr snapshot image handle */
    cmr_handle snp_fdr_thr_handle;
    /*video face beauty*/
    cmr_handle video_cb_thr_handle;
    struct fb_beauty_param video_face_beauty;
    bool mvideofb;
    struct fb_beauty_param prev_face_beauty;
    bool mflagfb;
    cmr_u32 start_video_face_beauty;
    cmr_int video_face_beauty_en;

    /*for setting*/
    cmr_u32 ref_camera_id;
    cmr_uint ai_scene_enable;
    struct camera_settings cmr_set;
    cmr_u32 is_support_fd;
    cmr_u32 fd_on_off;
    struct isp_face_area fd_face_area;
    cmr_u32 is_android_zsl;
    cmr_u32 flip_on;
    cmr_u32 is_lls_enable;
    cmr_u32 lls_shot_mode;
    cmr_u32 is_vendor_hdr;
    cmr_u32 is_vendor_fdr;
    cmr_u32 is_pipviv_mode;
    cmr_u32 isp_to_dram;
    cmr_int cap_cnt;
    multiCameraMode is_multi_mode;
    uint8_t master_id;
    cmr_u32 is_refocus_mode;
    cmr_u32 is_3dcalibration_mode;
    cmr_uint is_yuv_callback_mode;

    /*memory func*/
    camera_cb_of_malloc hal_malloc;
    camera_cb_of_free hal_free;
    camera_cb_of_gpu_malloc hal_gpu_malloc;
    void *hal_mem_privdata;

    /*for isp lsc buffer*/
    cmr_uint lsc_malloc_flag;
    cmr_uint isp_lsc_phys_addr;
    cmr_uint isp_lsc_virt_addr;
    cmr_s32 lsc_mfd;

    /*for b4awb buffer*/
    cmr_uint isp_b4awb_flag;
    cmr_uint b4awb_phys_addr[ISP_B4AWB_BUF_CNT];
    cmr_uint b4awb_virt_addr[ISP_B4AWB_BUF_CNT];
#ifdef CONFIG_FACE_BEAUTY
    struct fb_beauty_param face_beauty;
#endif
    cmr_u8 flag_highiso_alloc_mem;
    cmr_uint dump_cnt;
    cmr_uint is_start_snapshot;
    cmr_uint is_3dnr_video;
    cmr_u32 blur_facebeauty_flag;
    cmr_uint is_ultra_wide;
    cmr_uint is_fov_fusion;
    cmr_u32 is_real_bokeh;
    cmr_u32 is_focus;
    struct isp_pos focus_rect;
    cmr_int lcd_flash_highlight;
    cmr_u8 backlight_brightness;
    cmr_u8 backup_brightness;
    cmr_u16 color_temp;
    cmr_u32 bg_color;
    enhance_device_t *enhance;

    /*for sw isp depth buffer*/
    cmr_uint swisp_depth_malloc_flag;
    cmr_uint swisp_depth_phys_addr;
    cmr_uint swisp_depth_virt_addr;
    cmr_s32 swisp_depth_mfd;

    /*for sw isp out buffer*/
    cmr_uint swisp_out_malloc_flag;
    cmr_uint swisp_out_phys_addr;
    cmr_uint swisp_out_virt_addr;
    cmr_s32 swisp_out_mfd;

    cmr_s64 hdr_capture_timestamp;
    cmr_s64 fdr_capture_timestamp;
    cmr_s64 capture_timestamp;
    cmr_u32 hdr_skip_frame_enable;
    cmr_u32 hdr_skip_frame_cnt;
    cmr_u32 fdr_skip_frame_enable;
    cmr_u32 fdr_skip_frame_cnt;
    cmr_u32 fdr_capture_frame_cnt;
    cmr_u32 fdr_call_stream_off;
    cmr_u32 skip_frame_enable;
    cmr_u32 skip_frame_cnt;
    enum camera_snapshot_tpye snapshot_type;
    struct img_rect trim_reset_info;
    int fdr_total_frame_cnt;
    int fdr_ref_frame_num;
    cmr_u8 nr_flag;
    cmr_u8 ee_flag;
    cmr_u8 dre_flag;
    cmr_u8 gtm_flag;
    cmr_u8 predre_flag;
    cmr_u8 skipframe;
    bool night_flag;
    struct nightpro_context night_cxt;
    struct camera_core_context cam_core_cxt;

    /*for flash skip preview frame*/
    cmr_s64 flash_handle_timestamp;
    cmr_u32 flash_skip_frame_enable;
    cmr_u32 flash_skip_frame_cnt;
    cmr_u32 flash_skip_frame_num;
    struct isp_face_area fd_face_area_capture;
    bool is_capture_face;
    cmr_u32 zsl_enabled; /* 1: zsl,0: non-zsl */

    /* new 4in1 plan, 20191028 */
    cmr_u32 is_4in1_sensor; /* as is_4in1_sensor, should rename later */
    cmr_u32 remosaic_type; /* 1: software, 2: hardware, 0:other(sensor output bin size) */
    cmr_u32 ambient_highlight; /* 4in1: 1:highlight,0:lowlight; other sensor:0 */
    cmr_uint is_high_res_mode;
    cmr_handle jpeg_async_init_handle;
    /*for ynr room ratio*/
    float zoom_ratio;
    jpg_encode_status jpg_encode;
    cmr_u8 nightscepro_flag;
};

struct prev_ai_scene_info {
    struct img_frm frm_preview;
    unsigned long camera_id;
    void *caller_handle;
    struct frm_info data;
};

struct prev_auto_tracking_info {
    struct img_frm frm_preview;
    unsigned long camera_id;
    void *caller_handle;
    struct frm_info data;
    cmr_u32 frm_cnt;
};
cmr_int camera_local_int(cmr_u32 camera_id, camera_cb_of_type callback,
                         void *client_data, cmr_uint is_autotest,
                         cmr_handle *oem_handle, void *cb_of_malloc,
                         void *cb_of_free);

cmr_int camera_local_deinit(cmr_handle oem_handle);

cmr_int camera_local_fd_start(cmr_handle oem_handle);

cmr_int camera_local_start_preview(cmr_handle oem_handle,
                                   enum takepicture_mode mode,
                                   cmr_uint is_snapshot);
cmr_int camera_local_stop_preview(cmr_handle oem_handle);

cmr_int camera_local_start_snapshot(cmr_handle oem_handle,
                                    enum takepicture_mode mode,
                                    cmr_uint is_snapshot);

cmr_int camera_local_stop_snapshot(cmr_handle oem_handle);

cmr_int camera_local_redisplay_data(
    cmr_handle oem_handle, cmr_s32 output_fd, cmr_uint output_addr,
    cmr_uint output_vir_addr, cmr_uint output_width, cmr_uint output_height,
    cmr_s32 input_fd, cmr_uint input_addr_y, cmr_uint input_addr_uv,
    cmr_uint input_vir_addr, cmr_uint input_width, cmr_uint input_height);

cmr_int camera_local_get_prev_rect(cmr_handle oem_handle,
                                   struct img_rect *param_ptr);

cmr_int camera_get_sensor_mode_info(cmr_handle oem_handle,
                                    struct sensor_mode_info *mode_info);

cmr_int camera_get_sensor_mode_trim(cmr_handle oem_handle,
                                    struct img_rect *sn_trim);

cmr_int camera_get_senor_mode_trim2(cmr_handle oem_handle,
                                    struct img_rect *sn_trim);

cmr_uint camera_get_preview_angle(cmr_handle oem_handle);

cmr_uint camera_get_exif_info(cmr_handle oem_handle,
                              struct exif_info *exif_info);
cmr_uint camera_get_result_exif_info(
    cmr_handle oem_handle, struct exif_spec_pic_taking_cond_tag *exif_pic_info);

cmr_int camera_local_start_focus(cmr_handle oem_handle);

cmr_int camera_local_cancel_focus(cmr_handle oem_handle);

cmr_int prev_set_preview_skip_frame_num(cmr_handle preview_handle,
                                        cmr_u32 camera_id, cmr_uint skip_num,
                                        cmr_uint has_preflashed);

cmr_int camera_isp_set_params(cmr_handle camera_handle,
                              enum camera_param_type id, cmr_uint param);

cmr_int camera_local_set_param(cmr_handle camera_handle,
                               enum camera_param_type id, cmr_uint param);

cmr_int camera_local_get_zsl_info(cmr_handle oem_handle, cmr_uint *is_support,
                                  cmr_uint *max_width, cmr_uint *max_height);

cmr_int camera_local_fast_ctrl(cmr_handle oem_handle);

cmr_int camera_local_pre_flash(cmr_handle oem_handle);

cmr_int camera_local_get_viewangle(cmr_handle oem_handle,
                                   struct sensor_view_angle *view_angle);

cmr_int camera_local_set_preview_buffer(cmr_handle oem_handle,
                                        cmr_uint src_phy_addr,
                                        cmr_uint src_vir_addr, cmr_s32 fd);
cmr_int camera_local_set_video_buffer(cmr_handle oem_handle,
                                      cmr_uint src_phy_addr,
                                      cmr_uint src_vir_addr, cmr_s32 fd);
cmr_int camera_local_set_zsl_buffer(cmr_handle oem_handle,
                                    cmr_uint src_phy_addr,
                                    cmr_uint src_vir_addr, cmr_s32 fd);

cmr_s32 local_queue_buffer(cmr_handle camera_handle, cam_buffer_info_t buffer,
                           int steam_type);

cmr_int camera_local_set_video_snapshot_buffer(cmr_handle oem_handle,
                                               cmr_uint src_phy_addr,
                                               cmr_uint src_vir_addr,
                                               cmr_s32 fd);
cmr_int camera_local_set_zsl_snapshot_buffer(cmr_handle oem_handle,
                                             cmr_uint src_phy_addr,
                                             cmr_uint src_vir_addr, cmr_s32 fd);
cmr_int camera_local_zsl_snapshot_need_pause(cmr_handle oem_handle,
                                             cmr_int *flag);
cmr_int camera_local_normal_snapshot_need_pause(cmr_handle oem_handle,
                                                cmr_int *flag);
void camera_calibrationconfigure_save(uint32_t start_addr, uint32_t data_size);
cmr_int camera_local_get_isp_info(cmr_handle oem_handle, void **addr,
                                  int *size);

void camera_local_start_burst_notice(cmr_handle oem_handle);
void camera_local_end_burst_notice(cmr_handle oem_handle);

cmr_s32 camera_local_get_iommu_status(cmr_handle oem_handle);

cmr_int camera_set_security(cmr_handle oem_handle,
                            struct sprd_cam_sec_cfg *sec_cfg);
cmr_int camera_set_hdr_disable(cmr_handle oem_handle, cmr_u32 param);

cmr_int
camera_isp_set_sensor_info_to_af(cmr_handle oem_handle,
                                 struct cmr_af_aux_sensor_info *sensor_info);
cmr_int cmr_get_sensor_max_fps(cmr_handle oem_handle, cmr_u32 camera_id,
                               cmr_u32 *max_fps);
cmr_int cmr_get_blur_covered_type(cmr_handle oem_handle, cmr_s32 *type);
cmr_int cmr_sensor_init_static_info(cmr_handle oem_handle);
cmr_int cmr_sensor_deinit_static_info(cmr_handle oem_handle);

cmr_int cmr_set_zoom_factor_to_isp(cmr_handle oem_handle, float *zoomFactor);

cmr_int prev_set_preview_touch_info(cmr_handle preview_handle,
                                    cmr_u32 camera_id,
                                    struct touch_coordinate *touch_xy);

cmr_int camera_local_snapshot_is_need_flash(cmr_handle oem_handle,
                                            cmr_u32 camera_id,
                                            cmr_u32 *is_need_flash);
cmr_int camera_get_otpinfo(cmr_handle oem_handle, cmr_u8 dual_flag,
                           struct sensor_otp_cust_info *otp_data);
cmr_int camera_get_onlinebuffer(cmr_handle oem_handle, void *cali_info);
cmr_int prev_set_vcm_step(cmr_handle preview_handle, cmr_u32 camera_id,
                          void *data);
cmr_int cmr_get_sensor_vcm_step(cmr_handle oem_handle, cmr_u32 camera_id,
                                cmr_u32 *max_fps);
cmr_int cmr_get_vcm_range(cmr_handle oem_handle, cmr_u32 camera_id,
                          struct vcm_range_info *vcm_range);
cmr_int cmr_get_ae_fps_range(cmr_handle oem_handle, cmr_u32 camera_id,
                          struct ae_fps_range_info *ae_fps_range);
cmr_int cmr_set_vcm_disc(cmr_handle oem_handle, cmr_u32 camera_id,
                         struct vcm_disc_info *vcm_disc);
int af_state_focus_to_hal(cmr_u32 valid_win);
cmr_int camera_local_set_sensor_close_flag(cmr_handle oem_handle);
cmr_int camera_local_set_cap_size(
    cmr_handle oem_handle, cmr_u32 is_reprocessing, cmr_u32 camera_id,
    cmr_u32 width,
    cmr_u32 height); /**add for 3d capture to reset reprocessing capture size*/

cmr_int camera_local_start_capture(cmr_handle oem_handle);
cmr_int camera_local_stop_capture(cmr_handle oem_handle);

void camera_set_oem_multimode(multiCameraMode camera_mode);
void camera_set_oem_masterid(uint8_t master_id);
cmr_int camera_local_set_ref_camera_id(cmr_handle oem_handle,
                                       cmr_u32 *ref_camera_id);
cmr_int camera_local_set_visible_region(cmr_handle oem_handle,
                                        struct visible_region_info *info);
cmr_int camera_local_set_global_zoom_ratio(cmr_handle oem_handle, float *ratio);
cmr_int camera_local_cap_state(cmr_handle oem_handle,
                                       bool *flag);

cmr_int camera_local_get_cover(cmr_handle cmr_handle,
                               struct dual_sensor_luma_info *cover_value);
cmr_int camera_stream_ctrl(cmr_handle cmr_handle, cmr_u32 on_off);
cmr_int cmr_get_isp_af_fullscan(cmr_handle oem_handle,
                                struct isp_af_fullscan_info *af_fullscan_info);
cmr_int cmr_set_af_pos(cmr_handle oem_handle, cmr_u32 af_pos);
cmr_int cmr_set_3a_bypass(cmr_handle oem_handle, cmr_u32 value);
cmr_int cmr_get_ae_fps(cmr_handle oem_handle, cmr_u32 *ae_fps);
cmr_int camera_local_reprocess_yuv_for_jpeg(cmr_handle oem_handle,
                                            enum takepicture_mode mode,
                                            cmr_uint yaddr, cmr_uint yaddr_vir,
                                            cmr_uint fd);
cmr_int camera_set_3dnr_video(cmr_handle oem_handle, cmr_uint is_3dnr_video);
cmr_int camera_set_ultra_wide_mode(cmr_handle oem_handle,
                                   cmr_uint is_ultra_wide);
cmr_int camera_set_fov_fusion_mode(cmr_handle oem_handle,
                                   cmr_uint is_fov_fusion);
cmr_int cmr_set_snapshot_timestamp(cmr_handle oem_handle, int64_t timestamp);
cmr_int cmr_get_microdepth_param(cmr_handle oem_handle, void *param);
cmr_int cmr_set_microdepth_debug_info(cmr_handle oem_handle, void *param);
cmr_int camera_local_get_sensor_format(cmr_handle cmr_handle,
                                       cmr_u32 *sensor_format);
cmr_int camera_local_set_capture_fb(cmr_handle oem_handle, cmr_u32 *on);
cmr_int camera_set_thumb_yuv_proc(cmr_handle oem_handle,
                                  struct snp_thumb_yuv_param *param);
cmr_int camera_get_blur_covered_type(cmr_handle oem_handle, cmr_s32 *param);
cmr_int camera_yuv_do_face_beauty_simplify(cmr_handle oem_handle,
                                           struct img_frm *src);
cmr_int camera_jpeg_encode_exif_simplify(cmr_handle oem_handle,
                                         struct enc_exif_param *param);
cmr_int camera_jpeg_decode_simplify(cmr_handle oem_handle,
                                         struct enc_exif_param *param);

cmr_int camera_local_set_gpu_mem_ops(cmr_handle oem_handle, void *cb_of_malloc,
                                     void *cb_of_free);
cmr_int camera_get_grab_capability(cmr_handle oem_handle,
                                   struct cmr_path_capability *capability);
cmr_int camera_get_af_support(cmr_handle oem_handle, cmr_u16 *af_support);

cmr_int camera_local_image_sw_algorithm_processing(
    cmr_handle oem_handle, struct image_sw_algorithm_buf *src_sw_algorithm_buf,
    struct image_sw_algorithm_buf *dst_sw_algorithm_buf,
    sprd_cam_image_sw_algorithm_type_t sw_algorithm_type,
    cam_img_format_t format);
cmr_int camera_local_start_scale(cmr_handle oem_handle,
                                 struct img_frm **scale_param);
cmr_int camera_local_start_rotate(cmr_handle oem_handle,
                                  struct rotate_param *rotate_param);
int dump_image_with_3a_info(cmr_handle oem_handle, uint32_t img_fmt,
                            uint32_t width, uint32_t height, uint32_t dump_size,
                            struct img_addr *addr);
#ifdef CONFIG_CAMERA_MM_DVFS_SUPPORT
cmr_int camera_local_set_mm_dvfs_policy(cmr_handle oem_handle,
                                        enum DVFS_MM_MODULE module,
                                        enum CamProcessingState camera_state);
#endif
cmr_int camera_3dnr_set_ev(cmr_handle oem_handle, cmr_u32 enable);
cmr_int camera_snapshot_set_ev(cmr_handle oem_handle, cmr_u32 value ,enum camera_snapshot_tpye type);
cmr_int camera_get_sensor_info(cmr_handle oem_handle, cmr_uint sensor_id,
                               struct sensor_exp_info *exp_info_ptr);
cmr_int camera_sensor_ioctl(cmr_handle oem_handle, cmr_uint cmd_type,
                            struct common_sn_cmd_param *param_ptr);
cmr_int camera_channel_reproc(cmr_handle oem_handle, struct buffer_cfg *buf_cfg);
cmr_int camera_get_tuning_info(cmr_handle oem_handle,
                               struct isp_adgain_exp_info *adgain_exp_info_ptr);
cmr_int camera_isp_ioctl(cmr_handle oem_handle, cmr_uint cmd_type,
                                struct common_isp_cmd_param *param_ptr);
cmr_int cmr_get_reboke_data(cmr_handle oem_handle,
                            struct af_relbokeh_oem_data *golden_distance);
cmr_int camera_local_get_tuning_param(cmr_handle oem_handle,
                                      struct tuning_param_info *tuning_info);
cmr_int camera_get_fdr_tuning_param(cmr_handle oem_handle,
                                      struct isp_blkpm_t *tuning_param);
cmr_int camera_get_fdr_tuning_flag(cmr_handle oem_handle,
                                      cmr_int * tuning_flag);
cmr_int camera_get_ee_tuning_param(cmr_handle oem_handle,
                                      struct isp_blkpm_t *tuning_param);
cmr_int camera_get_blc_info(cmr_handle oem_handle, struct isp_blc_data *blc);
cmr_int camera_get_fdr_ae_info(cmr_handle oem_handle, void **fdr_ae_info);
cmr_int camera_get_ae_common_info(cmr_handle oem_handle, void **ae_common_info);
cmr_int camera_get_adgain_exp_info(cmr_handle oem_handle, struct isp_adgain_exp_info *isp_adgain);
cmr_int camera_get_fdr_free_buf(cmr_handle oem_handle, struct buffer_cfg *buf_cfg);
cmr_int camera_fdr_get_frame_cnt(cmr_handle oem_handle, int *total_frame_num, int *ref_frame_num);
// the last time to get fdr frame cnt before start capture
cmr_int camera_fdr_get_last_frm_cnt(cmr_handle oem_handle, int *total_frame_num, int *ref_frame_num);

cmr_int cmr_get_bokeh_sn_trim(cmr_handle handle,
                              struct sprd_img_path_rect *trim_param);

cmr_int camera_get_remosaic_type(struct sensor_4in1_info *p,
							  cmr_u32 sensor_w, cmr_u32 sensor_h);
cmr_int camera_get_is_4in1_sensor(struct sensor_4in1_info *p);
cmr_int camera_get_4in1_info(cmr_handle handle, struct fin1_info *param);
cmr_int camera_set_high_res_mode(cmr_handle oem_handle,cmr_uint is_high_res_mode);
cmr_int camera_get_fb_param(cmr_handle oem_handle, struct isp_fb_param_info *param);
cmr_int camera_get_bv_info(cmr_handle oem_handle, cmr_u32 *bv_info);
cmr_int camera_get_ct_info(cmr_handle oem_handle, cmr_u32 *ct_info);
cmr_u32 camera_get_cnr_flag(cmr_handle oem_handle);
cmr_u32 camera_get_ee_flag(cmr_handle oem_handle);
cmr_u32 camera_get_fdr_flag(struct camera_context *cxt);
void camera_grab_handle(cmr_int evt, void *data, void *privdata);
cmr_int camera_get_iso_info(cmr_handle oem_handle, cmr_u32 *iso_info);


#ifdef __cplusplus
}
#endif

#endif
