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
#define LOG_TAG "cmr_mem"

#include "cmr_mem.h"
#include "cmr_oem.h"
#include <unistd.h>
#include "cutils/properties.h"
#include "isp_video.h"

/*
          to add more.........
     8M-----------16M-------------8M
     5M-----------16M-------------0M
     3M-----------8M--------------2M
     2M-----------4M--------------2M
     1.3M---------4M--------------1M
*/

#define PIXEL_1P3_MEGA 0x180000   // actually 1.5 *1024*1024
#define PIXEL_2P0_MEGA 0x200000   // actually 2.0 *1024*1024
#define PIXEL_3P0_MEGA 0x300000   // actually 3.0 *1024*1024
#define PIXEL_4P0_MEGA 0x400000   // actually 3.0 *1024*1024
#define PIXEL_5P0_MEGA 0x500000   // 5.0 *1024*1024
#define PIXEL_6P0_MEGA 0x600000   // 6.0 *1024*1024
#define PIXEL_7P0_MEGA 0x700000   // 7.0 *1024*1024
#define PIXEL_8P0_MEGA 0x800000   // 8.0 *1024*1024
#define PIXEL_9P0_MEGA 0x900000   // 9.0 *1024*1024
#define PIXEL_AP0_MEGA 0xA00000   // 10.0 *1024*1024
#define PIXEL_BP0_MEGA 0xB00000   // 11.0 *1024*1024
#define PIXEL_CP0_MEGA 0xC00000   // 12.0 *1024*1024
#define PIXEL_DP0_MEGA 0xD00000   // 13.0 *1024*1024
#define PIXEL_10P0_MEGA 0x1000000 // 16.0 *1024*1024
#define PIXEL_15P0_MEGA 0x1500000 // 21.0 *1024*1024
#define PIXEL_32P0_MEGA 0x3200000 // 21.0 *1024*1024
#define PIXEL_48P0_MEGA 0x4800000 // 21.0 *1024*1024
#define MIN_GAP_LINES 0x80

#define ISP_YUV_TO_RAW_GAP CMR_SLICE_HEIGHT
#define BACK_CAMERA_ID 0
#define FRONT_CAMERA_ID 1
#define DEV2_CAMERA_ID 2
#define DEV3_CAMERA_ID 3
#define JPEG_SMALL_SIZE (300 * 1024)
#define ADDR_BY_WORD(a) (((a) + 3) & (~3))
#define CMR_NO_MEM(a, b)                                                       \
    do {                                                                       \
        if ((a) > (b)) {                                                       \
            CMR_LOGE("No memory, 0x%x 0x%x", (a), (b));                        \
            return -1;                                                         \
        }                                                                      \
    } while (0)

enum {
    IMG_1P3_MEGA = 0,
    IMG_2P0_MEGA,
    IMG_3P0_MEGA,
    IMG_4P0_MEGA,
    IMG_5P0_MEGA,
    IMG_6P0_MEGA,
    IMG_7P0_MEGA,
    IMG_8P0_MEGA,
    IMG_9P0_MEGA,
    IMG_AP0_MEGA,
    IMG_BP0_MEGA,
    IMG_CP0_MEGA,
    IMG_DP0_MEGA,
    IMG_10P0_MEGA,
    IMG_15P0_MEGA,
    IMG_32P0_MEGA,
    IMG_48P0_MEGA,
    IMG_SIZE_NUM
};

enum {
    JPEG_TARGET = 0,
    THUM_YUV,
    THUM_JPEG,

    BUF_TYPE_NUM
};

typedef uint32_t (*cmr_get_size)(uint32_t width, uint32_t height,
                                 uint32_t thum_width, uint32_t thum_height);

static multiCameraMode is_multi_camera_mode_mem;

extern int camera_get_is_noscale(void);

static uint32_t get_jpeg_size(uint32_t width, uint32_t height,
                              uint32_t thum_width, uint32_t thum_height);
static uint32_t get_thum_yuv_size(uint32_t width, uint32_t height,
                                  uint32_t thum_width, uint32_t thum_height);
static uint32_t get_thum_jpeg_size(uint32_t width, uint32_t height,
                                   uint32_t thum_width, uint32_t thum_height);
static uint32_t get_jpg_tmp_size(uint32_t width, uint32_t height,
                                 uint32_t thum_width, uint32_t thum_height);
static uint32_t get_scaler_tmp_size(uint32_t width, uint32_t height,
                                    uint32_t thum_width, uint32_t thum_height);
static uint32_t get_isp_tmp_size(uint32_t width, uint32_t height,
                                 uint32_t thum_width, uint32_t thum_height);

static int arrange_raw_buf(struct cmr_cap_2_frm *cap_2_frm,
                           struct img_size *sn_size, struct img_rect *sn_trim,
                           struct img_size *image_size, uint32_t orig_fmt,
                           struct img_size *cap_size,
                           struct img_size *thum_size,
                           struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                           uint32_t *io_mem_res, uint32_t *io_mem_end,
                           uint32_t *io_channel_size, cmr_u32 is_loose);

static int arrange_4in1_raw_buf(
    struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
    struct img_rect *sn_trim, struct img_size *image_size, uint32_t orig_fmt,
    struct img_size *cap_size, struct img_size *thum_size,
    struct cmr_cap_mem *capture_mem, uint32_t need_rot, uint32_t *io_mem_res,
    uint32_t *io_mem_end, uint32_t *io_channel_size, cmr_u32 is_loose);

static int arrange_jpeg_buf(struct cmr_cap_2_frm *cap_2_frm,
                            struct img_size *sn_size, struct img_rect *sn_trim,
                            struct img_size *image_size, uint32_t orig_fmt,
                            struct img_size *cap_size,
                            struct img_size *thum_size,
                            struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                            uint32_t *io_mem_res, uint32_t *io_mem_end,
                            uint32_t *io_channel_size);

static int arrange_yuv_buf(struct cmr_cap_2_frm *cap_2_frm,
                           struct img_size *sn_size, struct img_rect *sn_trim,
                           struct img_size *image_size, uint32_t orig_fmt,
                           struct img_size *cap_size,
                           struct img_size *thum_size,
                           struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                           uint32_t *io_mem_res, uint32_t *io_mem_end,
                           uint32_t *io_channel_size);

static int arrange_misc_buf(struct cmr_cap_2_frm *cap_2_frm,
                            struct img_size *sn_size, struct img_rect *sn_trim,
                            struct img_size *image_size, uint32_t orig_fmt,
                            struct img_size *cap_size,
                            struct img_size *thum_size,
                            struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                            uint32_t *io_mem_res, uint32_t *io_mem_end,
                            uint32_t *io_channel_size);

static int arrange_rot_buf(struct cmr_cap_2_frm *cap_2_frm,
                           struct img_size *sn_size, struct img_rect *sn_trim,
                           struct img_size *image_size, uint32_t orig_fmt,
                           struct img_size *cap_size,
                           struct img_size *thum_size,
                           struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                           uint32_t need_scale, uint32_t *io_mem_res,
                           uint32_t *io_mem_end, uint32_t *io_channel_size);

static const cmr_get_size get_size[BUF_TYPE_NUM] = {
    get_jpeg_size, get_thum_yuv_size, get_thum_jpeg_size,

};

#define SENSOR_MAX_NUM 6

static cmr_u16 largest_picture_width[SENSOR_MAX_NUM];
static cmr_u16 largest_picture_height[SENSOR_MAX_NUM];

int camera_set_largest_pict_size(cmr_u32 camera_id, cmr_u16 width,
                                 cmr_u16 height) {
    largest_picture_width[camera_id] = width;
    largest_picture_height[camera_id] = height;

    return 0;
}

/* how to calculate cap size:
* for raw capture:
* target_yuv = w * h * 3 / 2;
* cap_yuv = w * h * 3 / 2;
* cap_raw = w * h * 3 / 2, resue target_yuv or cap_yuv
* cap_raw2 = w * h * 3 / 2
* thum_yuv = thumW * thumH * 3 / 2; for thumbnail yuv
* thum_jpeg = thumW * thumH * 3 / 2; for thumbnail jpeg
* target_jpeg, resue
* so, the total cap size is:
* w * h * 3 / 2 + w * h * 3 / 2 + w * h * 3 / 2 + thumW * thumH * 3 = 3 * w * h
* + thumW * thumH
* * 3(bytes);
*
* for yuv capture:
* target_yuv = w * h * 3 / 2;
* cap_yuv = w * h * 3 / 2;
* cap_raw = w * h * 3 / 2, resue target_yuv or cap_yuv
* thum_yuv = thumW * thumH * 3 / 2; for thumbnail yuv
* thum_jpeg = thumW * thumH * 3 / 2; for thumbnail jpeg
* target_jpeg, resue
* so, the total cap size is:
* w * h * 3 / 2 + w * h * 3 / 2 + thumW * thumH * 3 = 3 * w * h + thumW * thumH
* * 3(bytes);
*/
int camera_get_raw_postproc_capture_size(cmr_u32 camera_id, cmr_u32 *pp_cap_size, cmr_u32 is_loose) {
    cmr_u32 max_w, max_h, thumb_w, thumb_h;
    cmr_u32 redundance_size;
    char value[PROPERTY_VALUE_MAX];

    if (pp_cap_size == NULL) {
        CMR_LOGE("pp_cap_size=%p", pp_cap_size);
        return -1;
    }

    char prop[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.vendor.cam.res.multi.camera.fullsize", prop, "0");
    if (atoi(prop) == 1 && camera_id == sensorGetRole(MODULE_SPW_NONE_BACK) &&
        sensorGetRole(MODULE_OPTICSZOOM_WIDE_BACK) != -1) {
        if (largest_picture_width[sensorGetRole(MODULE_OPTICSZOOM_WIDE_BACK)] != 0) {
            largest_picture_width[camera_id] =
                largest_picture_width[sensorGetRole(MODULE_OPTICSZOOM_WIDE_BACK)];
            largest_picture_height[camera_id] =
                largest_picture_height[sensorGetRole(MODULE_OPTICSZOOM_WIDE_BACK)];
        }
    }

    max_w = largest_picture_width[camera_id];
    max_h = largest_picture_height[camera_id];

    // we assume that thumb size is not bigger than 512*512
    thumb_w = 512;
    thumb_h = 512;

    // alloc more redundance_size(1M) memory for alignment use
    redundance_size = 1 * 1024 * 1024;

    *pp_cap_size = 3 * max_w * max_h + 3 * thumb_w * thumb_h + redundance_size;
#ifndef CONFIG_ISP_2_3
    // for raw capture
    if (max_w < 9216) {
	    property_get("persist.vendor.cam.raw.mode", value, "jpeg");
	    if ((!strcmp(value, "raw"))||isp_video_get_simulation_flag()) {
	        if (is_loose == ISP_RAW_HALF14 || is_loose == ISP_RAW_HALF10) {
	            *pp_cap_size += 5 * max_w * max_h / 2;
	        } else {
	            *pp_cap_size += 3 * max_w * max_h / 2;
	        }
	    }
    }
#endif
    // the above is default configuration, for some special case, you can change
    // it like this:
    // if (max_w * max_h <= xyyz && max_w * max_h > abcd) {
    //    *pp_cap_size = you_wanted_size;
    //}

    CMR_LOGD("postporc_capture_size=%d", *pp_cap_size);

    return 0;
}

int camera_get_4in1_postproc_capture_size(cmr_u32 camera_id,
                                          cmr_u32 *pp_cap_size, cmr_u32 is_loose) {
    cmr_u32 max_w, max_h, thumb_w, thumb_h, small_w, small_h;
    cmr_u32 redundance_size;
    char value[PROPERTY_VALUE_MAX];

    if (pp_cap_size == NULL) {
        CMR_LOGE("pp_cap_size=%p", pp_cap_size);
        return -1;
    }

    max_w = largest_picture_width[camera_id];
    max_h = largest_picture_height[camera_id];
    small_w = max_w >> 1;
    small_h = max_h >> 1;

    // we assume that thumb size is not bigger than 512*512
    thumb_w = 512;
    thumb_h = 512;

    // alloc more redundance_size(1M) memory for alignment use
    redundance_size = 1 * 1024 * 1024;

    *pp_cap_size = 3 * max_w * max_h + 3 * thumb_w * thumb_h + redundance_size;
    *pp_cap_size += 3 * small_w * small_h;

    // for raw capture
    property_get("persist.vendor.cam.raw.mode", value, "jpeg");
    if (!strcmp(value, "raw")) {
        if (is_loose == ISP_RAW_HALF14 || is_loose == ISP_RAW_HALF10) {
            *pp_cap_size += 5 * max_w * max_h / 2;
            *pp_cap_size += 5 * small_w * small_h / 2;
        } else {
            *pp_cap_size += 3 * max_w * max_h / 2;
            *pp_cap_size += 3 * small_w * small_h / 2;
        }
    }

    // the above is default configuration, for some special case, you can change
    // it like this:
    // if (max_w * max_h <= xyyz && max_w * max_h > abcd) {
    //    *pp_cap_size = you_wanted_size;
    //}

    CMR_LOGD("postporc_capture_size=0x%x, small_w =%d, small_h =%d",
             *pp_cap_size, small_w, small_h);

    return 0;
}

int camera_get_postproc_capture_size(cmr_u32 camera_id,
                              cmr_u32 * pp_cap_size, cmr_u32 channel_size) {
    cmr_u32 thumb_w, thumb_h;
    cmr_u32 redundance_size;

    if (pp_cap_size == NULL) {
        CMR_LOGE("pp_cap_size=%p", pp_cap_size);
        return -1;
    }

    // we assume that thumb size is not bigger than 512*512
    thumb_w = 512;
    thumb_h = 512;

    // alloc more redundance_size(1M) memory for alignment use
    redundance_size = 1 * 1024 * 1024;

    *pp_cap_size = 3 * channel_size + 3 * thumb_w * thumb_h + redundance_size;

    CMR_LOGD("postporc_capture_size=%d", *pp_cap_size);
    return 0;
}

int camera_arrange_capture_buf(
    struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
    struct img_rect *sn_trim, struct img_size *image_size, uint32_t orig_fmt,
    struct img_size *cap_size, struct img_size *thum_size,
    struct cmr_cap_mem *capture_mem,struct sensor_exp_info *sn_if,
    uint32_t need_rot, uint32_t need_scale,
    uint32_t image_cnt, cmr_u32 is_4in1_mode) {

    if (NULL == cap_2_frm || NULL == image_size || NULL == thum_size ||
        NULL == capture_mem || NULL == sn_size || NULL == sn_trim) {
        CMR_LOGE("Parameter error 0x%p 0x%p 0x%p 0x%p 0x%p 0x%p", cap_2_frm,
                 image_size, thum_size, capture_mem, sn_size, sn_trim);
        return -1;
    }

    uint32_t channel_size;
    uint32_t mem_res = 0, mem_end = 0;
    uint32_t i = 0;
    int32_t ret = -1;
    struct cmr_cap_mem *cap_mem = &capture_mem[0];

    channel_size = (uint32_t)(image_size->width * image_size->height);
    mem_res = cap_2_frm->mem_frm.buf_size;

    CMR_LOGD("mem frame, 0x%lx 0x%lx, fd 0x%x, buf_size = %d bytes, "
             "is_4in1_mode = %d",
             cap_2_frm->mem_frm.addr_phy.addr_y,
             cap_2_frm->mem_frm.addr_vir.addr_y, cap_2_frm->mem_frm.fd,
             cap_2_frm->mem_frm.buf_size, is_4in1_mode);

    CMR_LOGD("channel_size, 0x%x, image_cnt %d, rot %d, orig_fmt %d",
             channel_size, image_cnt, need_rot, orig_fmt);

    CMR_LOGD(
        "sn_size %d %d, sn_trim %d %d %d %d, image_size %d %d, cap_size %d %d",
        sn_size->width, sn_size->height, sn_trim->start_x, sn_trim->start_y,
        sn_trim->width, sn_trim->height, image_size->width, image_size->height,
        cap_size->width, cap_size->height);

    /* get target_jpeg buffer size first, will be used later */
    memset((void *)cap_mem, 0, sizeof(struct cmr_cap_mem));
    cap_mem->target_jpeg.buf_size =
        get_jpeg_size(image_size->width, image_size->height, thum_size->width,
                      thum_size->height);

    if (CAM_IMG_FMT_BAYER_MIPI_RAW == orig_fmt && is_4in1_mode > 0) {
        ret = arrange_4in1_raw_buf(cap_2_frm, sn_size, sn_trim, image_size,
                                   orig_fmt, cap_size, thum_size, cap_mem,
                                   need_rot, &mem_res, &mem_end, &channel_size, sn_if->sn_interface.is_loose);
        if (ret) {
            CMR_LOGE("raw fmt arrange failed!");
            return -1;
        }
    } else if (CAM_IMG_FMT_BAYER_MIPI_RAW == orig_fmt) {
        ret = arrange_raw_buf(cap_2_frm, sn_size, sn_trim, image_size, orig_fmt,
                              cap_size, thum_size, cap_mem, need_rot, &mem_res,
                              &mem_end, &channel_size, sn_if->sn_interface.is_loose);
        if (ret) {
            CMR_LOGE("raw fmt arrange failed!");
            return -1;
        }
    } else if (CAM_IMG_FMT_JPEG == orig_fmt) {
        ret = arrange_jpeg_buf(cap_2_frm, sn_size, sn_trim, image_size,
                               orig_fmt, cap_size, thum_size, cap_mem, need_rot,
                               &mem_res, &mem_end, &channel_size);
        if (ret) {
            CMR_LOGE("jpeg fmt arrange failed!");
            return -1;
        }
    } else {
        ret = arrange_yuv_buf(cap_2_frm, sn_size, sn_trim, image_size, orig_fmt,
                              cap_size, thum_size, cap_mem, need_rot, &mem_res,
                              &mem_end, &channel_size);
        if (ret) {
            CMR_LOGE("yuv fmt arrange failed!");
            return -1;
        }
    }

    /* arrange misc buffers */
    ret = arrange_misc_buf(cap_2_frm, sn_size, sn_trim, image_size, orig_fmt,
                           cap_size, thum_size, cap_mem, need_rot, &mem_res,
                           &mem_end, &channel_size);
    if (ret) {
        CMR_LOGE("misc buf arrange failed!");
        return -1;
    }

    /* arrange rot buf */
    if (need_rot) {
        ret = arrange_rot_buf(cap_2_frm, sn_size, sn_trim, image_size, orig_fmt,
                              cap_size, thum_size, cap_mem, need_rot,
                              need_scale, &mem_res, &mem_end, &channel_size);
        if (ret) {
            CMR_LOGE("rot buf arrange failed!");
            return -1;
        }
    }

    CMR_LOGD("mem_end, mem_res: 0x%x 0x%x ", mem_end, mem_res);

    /* resize target jpeg buffer */
    cap_mem->target_jpeg.addr_phy.addr_y =
        cap_mem->target_jpeg.addr_phy.addr_y + JPEG_EXIF_SIZE;
    cap_mem->target_jpeg.addr_vir.addr_y =
        cap_mem->target_jpeg.addr_vir.addr_y + JPEG_EXIF_SIZE;
    cap_mem->target_jpeg.buf_size =
        cap_mem->target_jpeg.buf_size - JPEG_EXIF_SIZE;

    CMR_LOGD("target_yuv, phy offset 0x%lx 0x%lx, vir 0x%lx 0x%lx, fd 0x%x, "
             "size 0x%x",
             cap_mem->target_yuv.addr_phy.addr_y,
             cap_mem->target_yuv.addr_phy.addr_u,
             cap_mem->target_yuv.addr_vir.addr_y,
             cap_mem->target_yuv.addr_vir.addr_u, cap_mem->target_yuv.fd,
             cap_mem->target_yuv.buf_size);

    CMR_LOGD(
        "cap_yuv, phy offset 0x%lx 0x%lx, vir 0x%lx 0x%lx, fd 0x%x, size 0x%x",
        cap_mem->cap_yuv.addr_phy.addr_y, cap_mem->cap_yuv.addr_phy.addr_u,
        cap_mem->cap_yuv.addr_vir.addr_y, cap_mem->cap_yuv.addr_vir.addr_u,
        cap_mem->cap_yuv.fd, cap_mem->cap_yuv.buf_size);

    CMR_LOGD("cap_yuv_rot, phy offset 0x%lx 0x%lx, vir 0x%lx 0x%lx, fd 0x%x, "
             "size 0x%x",
             cap_mem->cap_yuv_rot.addr_phy.addr_y,
             cap_mem->cap_yuv_rot.addr_phy.addr_u,
             cap_mem->cap_yuv_rot.addr_vir.addr_y,
             cap_mem->cap_yuv_rot.addr_vir.addr_u, cap_mem->cap_yuv_rot.fd,
             cap_mem->cap_yuv_rot.buf_size);

    CMR_LOGD(
        "cap_raw, phy offset 0x%lx 0x%lx, vir 0x%lx 0x%lx, fd 0x%x, size 0x%x",
        cap_mem->cap_raw.addr_phy.addr_y, cap_mem->cap_raw.addr_phy.addr_u,
        cap_mem->cap_raw.addr_vir.addr_y, cap_mem->cap_raw.addr_vir.addr_u,
        cap_mem->cap_raw.fd, cap_mem->cap_raw.buf_size);

    CMR_LOGD(
        "cap_raw2, phy offset 0x%lx 0x%lx, vir 0x%lx 0x%lx, fd 0x%x, size 0x%x",
        cap_mem->cap_raw2.addr_phy.addr_y, cap_mem->cap_raw2.addr_phy.addr_u,
        cap_mem->cap_raw2.addr_vir.addr_y, cap_mem->cap_raw2.addr_vir.addr_u,
        cap_mem->cap_raw2.fd, cap_mem->cap_raw2.buf_size);

    CMR_LOGD("target_jpeg, phy offset 0x%lx, vir 0x%lx, fd 0x%x, size 0x%x",
             cap_mem->target_jpeg.addr_phy.addr_y,
             cap_mem->target_jpeg.addr_vir.addr_y, cap_mem->target_jpeg.fd,
             cap_mem->target_jpeg.buf_size);

    CMR_LOGD(
        "thum_yuv, phy offset 0x%lx 0x%lx, vir 0x%lx 0x%lx, fd 0x%x, size 0x%x",
        cap_mem->thum_yuv.addr_phy.addr_y, cap_mem->thum_yuv.addr_phy.addr_u,
        cap_mem->thum_yuv.addr_vir.addr_y, cap_mem->thum_yuv.addr_vir.addr_u,
        cap_mem->thum_yuv.fd, cap_mem->thum_yuv.buf_size);

    CMR_LOGD("thum_jpeg, phy offset 0x%lx, vir 0x%lx, fd 0x%x, size 0x%x",
             cap_mem->thum_jpeg.addr_phy.addr_y,
             cap_mem->thum_jpeg.addr_vir.addr_y, cap_mem->thum_jpeg.fd,
             cap_mem->thum_jpeg.buf_size);

    CMR_LOGD("scale_tmp, phy offset 0x%lx, vir 0x%lx, fd 0x%x, size 0x%x",
             cap_mem->scale_tmp.addr_phy.addr_y,
             cap_mem->scale_tmp.addr_vir.addr_y, cap_mem->scale_tmp.fd,
             cap_mem->scale_tmp.buf_size);

    for (i = 1; i < image_cnt; i++)
        memcpy((void *)&capture_mem[i], (void *)&capture_mem[0],
               sizeof(struct cmr_cap_mem));

    return 0;
}

int arrange_raw_buf(struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
                    struct img_rect *sn_trim, struct img_size *image_size,
                    uint32_t orig_fmt, struct img_size *cap_size,
                    struct img_size *thum_size, struct cmr_cap_mem *capture_mem,
                    uint32_t need_rot, uint32_t *io_mem_res,
                    uint32_t *io_mem_end, uint32_t *io_channel_size, cmr_u32 is_loose) {
    UNUSED(thum_size);
    if (CAM_IMG_FMT_BAYER_MIPI_RAW != orig_fmt || NULL == io_mem_res ||
        NULL == io_mem_end || NULL == io_channel_size) {
        return -1;
    }

    uint32_t mem_res = 0, mem_end = 0;
    uint32_t offset = 0;
    uint32_t raw_size = 0, raw2_size = 0;
    uint32_t tmp1, tmp2, tmp3, max_size;
    struct cmr_cap_mem *cap_mem = capture_mem;
    uint32_t fetch_pitch;

    mem_res = *io_mem_res;
    mem_end = *io_mem_end;

#ifdef CAMERA_PITCH_SUPPORT
    #define _pitch(w)  (((w + 3) / 4 * 5 + 3) & (~0x3))
    fetch_pitch = _pitch(sn_size->width);
    raw_size = fetch_pitch * sn_size->height;
    raw2_size = fetch_pitch * sn_size->height;
    CMR_LOGI("fetch_pitch, %d\n", fetch_pitch);
#else
    CMR_LOGI("is_loose %d", is_loose);
    if (is_loose == ISP_RAW_HALF14 || is_loose == ISP_RAW_HALF10) {
        raw_size = sn_size->width * sn_size->height * RAWRGB_RAW14BIT_WIDTH / 8;
        raw2_size = sn_size->width * sn_size->height * RAWRGB_RAW14BIT_WIDTH / 8;
    } else {
        raw_size = sn_size->width * sn_size->height * RAWRGB_BIT_WIDTH / 8;
        raw2_size = sn_size->width * sn_size->height * RAWRGB_BIT_WIDTH / 8;
    }

#endif

    tmp1 = image_size->width * image_size->height * 3 / 2;
    tmp2 = cap_size->width * cap_size->height * 3 / 2;
    tmp3 = sn_size->width * sn_size->height * 3 / 2;
    max_size = tmp1 > tmp2 ? tmp1 : tmp2;
    max_size = max_size > tmp3 ? max_size : tmp3;
    max_size = (max_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
    CMR_LOGV("image_size %d %d, cap_size %d %d, sn_size %d %d",
                         image_size->width, image_size->height,
                         cap_size->width, cap_size->height,
                         sn_size->width, sn_size->height);
    CMR_LOGV("tmp1 %d, tmp2 %d, tmp3 %d, max_size %d", tmp1, tmp2, tmp3, max_size);

    cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->mem_frm.addr_phy.addr_y;
    cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->mem_frm.addr_vir.addr_y;
    cap_mem->target_yuv.addr_phy.addr_u =
        cap_mem->target_yuv.addr_phy.addr_y +
        image_size->width * image_size->height;
    cap_mem->target_yuv.addr_vir.addr_u =
        cap_mem->target_yuv.addr_vir.addr_y +
        image_size->width * image_size->height;
    cap_mem->target_yuv.fd = cap_2_frm->mem_frm.fd;
    cap_mem->target_yuv.buf_size =
        image_size->width * image_size->height * 3 / 2;
    cap_mem->target_yuv.size.width = image_size->width;
    cap_mem->target_yuv.size.height = image_size->height;
    cap_mem->target_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->cap_yuv.addr_phy.addr_y =
        cap_2_frm->mem_frm.addr_phy.addr_y + max_size;
    cap_mem->cap_yuv.addr_vir.addr_y =
        cap_2_frm->mem_frm.addr_vir.addr_y + max_size;
    cap_mem->cap_yuv.addr_phy.addr_u =
        cap_mem->cap_yuv.addr_phy.addr_y + cap_size->width * cap_size->height;
    cap_mem->cap_yuv.addr_vir.addr_u =
        cap_mem->cap_yuv.addr_vir.addr_y + cap_size->width * cap_size->height;
    cap_mem->cap_yuv.fd = cap_2_frm->mem_frm.fd;
    cap_mem->cap_yuv.buf_size = cap_size->width * cap_size->height * 3 / 2;
    cap_mem->cap_yuv.size.width = cap_size->width;
    cap_mem->cap_yuv.size.height = cap_size->height;
    cap_mem->cap_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
    cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
    cap_mem->cap_raw.fd = cap_mem->cap_yuv.fd;
    cap_mem->cap_raw.buf_size = raw_size;
    cap_mem->cap_raw.size.width = sn_size->width;
    cap_mem->cap_raw.size.height = sn_size->height;
    cap_mem->cap_raw.fmt = CAM_IMG_FMT_BAYER_MIPI_RAW;

/* sharkLE no need use raw2 image */
#if defined(CONFIG_ISP_2_3)
        raw2_size = 0;
#else
    cap_mem->cap_raw2.addr_phy.addr_y =
        cap_2_frm->mem_frm.addr_phy.addr_y + max_size + raw_size;
    cap_mem->cap_raw2.addr_vir.addr_y =
        cap_2_frm->mem_frm.addr_vir.addr_y + max_size + raw_size;
    cap_mem->cap_raw2.fd = cap_2_frm->mem_frm.fd;
    cap_mem->cap_raw2.buf_size = raw2_size;
    cap_mem->cap_raw2.size.width = sn_size->width;
    cap_mem->cap_raw2.size.height = sn_size->height;
    cap_mem->cap_raw2.fmt = CAM_IMG_FMT_BAYER_SPRD_DCAM_RAW;
#endif
    cap_mem->target_jpeg.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
    cap_mem->target_jpeg.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
    cap_mem->target_jpeg.fd = cap_mem->cap_yuv.fd;

    tmp1 = max_size + raw_size + raw2_size;
    tmp2 = raw_size + raw2_size + cap_mem->target_jpeg.buf_size;
    tmp3 = tmp1 > tmp2 ? tmp1 : tmp2;
    tmp3 = (tmp3 + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
    CMR_LOGV("max_size %d, raw_size %d, raw2_size%d, mem_res %d, buf_size %d",
                         max_size, raw_size, raw2_size, mem_res, cap_mem->target_jpeg.buf_size);
    CMR_LOGV("tmp1 %d, tmp2 %d, tmp3 %d, mem_res %d", tmp1, tmp2, tmp3, mem_res);

    CMR_NO_MEM(tmp3, mem_res);

    *io_mem_res = mem_res - tmp3;
    *io_mem_end = mem_end + tmp3;

    return 0;
}

int arrange_4in1_raw_buf(struct cmr_cap_2_frm *cap_2_frm,
                         struct img_size *sn_size, struct img_rect *sn_trim,
                         struct img_size *image_size, uint32_t orig_fmt,
                         struct img_size *cap_size, struct img_size *thum_size,
                         struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                         uint32_t *io_mem_res, uint32_t *io_mem_end,
                         uint32_t *io_channel_size, cmr_u32 is_loose) {
    UNUSED(thum_size);
    if (CAM_IMG_FMT_BAYER_MIPI_RAW != orig_fmt || NULL == io_mem_res ||
        NULL == io_mem_end || NULL == io_channel_size) {
        return -1;
    }

    cmr_u32 small_w, small_h;
    uint32_t mem_res = 0, mem_end = 0;
    uint32_t offset = 0;
    uint32_t raw_size = 0, raw2_size = 0, raw_size_4K_align;
    uint32_t tmp1, tmp2, tmp3, max_size;
    struct cmr_cap_mem *cap_mem = capture_mem;

    small_w = sn_size->width >> 1;
    small_h = sn_size->height >> 1;

    mem_res = *io_mem_res;
    mem_end = *io_mem_end;

    if (is_loose == ISP_RAW_HALF14 || is_loose == ISP_RAW_HALF10) {
        raw_size = sn_size->width * sn_size->height * RAWRGB_RAW14BIT_WIDTH / 8;
        raw2_size = sn_size->width * sn_size->height * RAWRGB_RAW14BIT_WIDTH / 8;
    } else {
        raw_size = sn_size->width * sn_size->height * RAWRGB_BIT_WIDTH / 8;
        raw2_size = sn_size->width * sn_size->height * RAWRGB_BIT_WIDTH / 8;
    }
    raw_size_4K_align = (raw_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    tmp1 = image_size->width * image_size->height * 3 / 2 +
           small_w * small_h * 3 / 2;
    tmp2 =
        cap_size->width * cap_size->height * 3 / 2 + small_w * small_h * 3 / 2;
    tmp3 = sn_size->width * sn_size->height * 3 / 2 + small_w * small_h * 3 / 2;
    max_size = tmp1 > tmp2 ? tmp1 : tmp2;
    max_size = max_size > tmp3 ? max_size : tmp3;
    max_size = (max_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->mem_frm.addr_phy.addr_y;
    cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->mem_frm.addr_vir.addr_y;
    cap_mem->target_yuv.addr_phy.addr_u =
        cap_mem->target_yuv.addr_phy.addr_y +
        image_size->width * image_size->height;
    cap_mem->target_yuv.addr_vir.addr_u =
        cap_mem->target_yuv.addr_vir.addr_y +
        image_size->width * image_size->height;
    cap_mem->target_yuv.fd = cap_2_frm->mem_frm.fd;
    cap_mem->target_yuv.buf_size =
        image_size->width * image_size->height * 3 / 2;
    cap_mem->target_yuv.size.width = image_size->width;
    cap_mem->target_yuv.size.height = image_size->height;
    cap_mem->target_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->cap_yuv.addr_phy.addr_y =
        cap_2_frm->mem_frm.addr_phy.addr_y + max_size;
    cap_mem->cap_yuv.addr_vir.addr_y =
        cap_2_frm->mem_frm.addr_vir.addr_y + max_size;
    cap_mem->cap_yuv.addr_phy.addr_u =
        cap_mem->cap_yuv.addr_phy.addr_y + cap_size->width * cap_size->height;
    cap_mem->cap_yuv.addr_vir.addr_u =
        cap_mem->cap_yuv.addr_vir.addr_y + cap_size->width * cap_size->height;
    cap_mem->cap_yuv.fd = cap_2_frm->mem_frm.fd;
    cap_mem->cap_yuv.buf_size = cap_size->width * cap_size->height * 3 / 2;
    cap_mem->cap_yuv.size.width = cap_size->width;
    cap_mem->cap_yuv.size.height = cap_size->height;
    cap_mem->cap_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
    cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
    cap_mem->cap_raw.fd = cap_mem->cap_yuv.fd;
    if (is_loose == ISP_RAW_HALF14 || is_loose == ISP_RAW_HALF10){
        cap_mem->cap_raw.buf_size =
        raw_size_4K_align + small_w * small_h * RAWRGB_RAW14BIT_WIDTH/ 8;
    } else {
        cap_mem->cap_raw.buf_size =
        raw_size_4K_align + small_w * small_h * RAWRGB_BIT_WIDTH / 8;
    }
    cap_mem->cap_raw.size.width = sn_size->width;
    cap_mem->cap_raw.size.height = sn_size->height;
    cap_mem->cap_raw.fmt = CAM_IMG_FMT_BAYER_MIPI_RAW;

    CMR_LOGD("cap_raw.buf_size= 0x%x, small_w =%d, small_h =%d",
             cap_mem->cap_raw.buf_size, small_w, small_h);

    cap_mem->cap_raw2.addr_phy.addr_y =
        cap_2_frm->mem_frm.addr_phy.addr_y + max_size + raw_size;
    cap_mem->cap_raw2.addr_vir.addr_y =
        cap_2_frm->mem_frm.addr_vir.addr_y + max_size + raw_size;
    cap_mem->cap_raw2.fd = cap_2_frm->mem_frm.fd;
    cap_mem->cap_raw2.buf_size = raw2_size;
    cap_mem->cap_raw2.size.width = sn_size->width;
    cap_mem->cap_raw2.size.height = sn_size->height;
    cap_mem->cap_raw2.fmt = CAM_IMG_FMT_BAYER_SPRD_DCAM_RAW;

    cap_mem->target_jpeg.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
    cap_mem->target_jpeg.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
    cap_mem->target_jpeg.fd = cap_mem->cap_yuv.fd;

    tmp1 = max_size + raw_size + raw2_size;
    tmp2 = raw_size + raw2_size + cap_mem->target_jpeg.buf_size;
    tmp3 = tmp1 > tmp2 ? tmp1 : tmp2;
    tmp3 = (tmp3 + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    CMR_NO_MEM(tmp3, mem_res);

    *io_mem_res = mem_res - tmp3;
    *io_mem_end = mem_end + tmp3;

    return 0;
}

int arrange_jpeg_buf(struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
                     struct img_rect *sn_trim, struct img_size *image_size,
                     uint32_t orig_fmt, struct img_size *cap_size,
                     struct img_size *thum_size,
                     struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                     uint32_t *io_mem_res, uint32_t *io_mem_end,
                     uint32_t *io_channel_size) {
    UNUSED(thum_size);

    uint32_t channel_size;
    uint32_t mem_res = 0, mem_end = 0;
    uint32_t offset = 0;
    uint32_t yy_to_y = 0, tmp = 0;
    uint32_t y_end = 0, uv_end = 0;
    uint32_t gap_size = 0;

    struct cmr_cap_mem *cap_mem = capture_mem; /*&capture_mem[0];*/
    struct img_size align16_image_size, align16_cap_size;

    if (CAM_IMG_FMT_JPEG != orig_fmt || NULL == io_mem_res ||
        NULL == io_mem_end || NULL == io_channel_size) {
        return -1;
    }
    CMR_LOGD("jpeg fmt buf arrange");

    align16_image_size.width =
        camera_get_aligned_size(cap_2_frm->type, image_size->width);
    align16_image_size.height =
        camera_get_aligned_size(cap_2_frm->type, image_size->height);
    align16_cap_size.width =
        camera_get_aligned_size(cap_2_frm->type, cap_size->width);
    align16_cap_size.height =
        camera_get_aligned_size(cap_2_frm->type, cap_size->height);

    mem_res = *io_mem_res;
    mem_end = *io_mem_end;
    // channel_size = *io_channel_size;

    channel_size =
        (uint32_t)(align16_image_size.width * align16_image_size.height);
    cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->mem_frm.addr_phy.addr_y;
    cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->mem_frm.addr_vir.addr_y;
    cap_mem->target_yuv.fd = cap_2_frm->mem_frm.fd;

    yy_to_y =
        (uint32_t)((sn_size->height - sn_trim->start_y - sn_trim->height) *
                   sn_size->width);
    tmp = (uint32_t)(sn_size->height * sn_size->width);
    y_end = yy_to_y + MAX(channel_size, tmp);

    CMR_LOGD("yy_to_y, 0x%x, tmp 0x%x", yy_to_y, tmp);

    cap_mem->target_yuv.addr_phy.addr_u =
        cap_mem->target_yuv.addr_phy.addr_y + y_end;
    cap_mem->target_yuv.addr_vir.addr_u =
        cap_mem->target_yuv.addr_vir.addr_y + y_end;
    cap_mem->cap_yuv.addr_phy.addr_y =
        cap_mem->target_yuv.addr_phy.addr_u - tmp;
    cap_mem->cap_yuv.fd = cap_mem->target_yuv.fd;
    /*to confirm the gap of scaling source and dest should be large than
     * MIN_GAP_LINES*/
    if ((cap_mem->cap_yuv.addr_phy.addr_y -
         cap_mem->target_yuv.addr_phy.addr_y) <
        (MIN_GAP_LINES * align16_image_size.width)) {
        gap_size = (MIN_GAP_LINES * align16_image_size.width) -
                   (cap_mem->cap_yuv.addr_phy.addr_y -
                    cap_mem->target_yuv.addr_phy.addr_y);
        gap_size = (gap_size + PAGE_SIZE - 1) & (~(PAGE_SIZE - 1));
    }

    cap_mem->cap_yuv.addr_phy.addr_y += gap_size;
    cap_mem->cap_yuv.addr_vir.addr_y =
        cap_mem->target_yuv.addr_vir.addr_u - tmp + gap_size;

    cap_mem->target_yuv.addr_phy.addr_u += gap_size;
    cap_mem->target_yuv.addr_vir.addr_u += gap_size;

    cap_mem->cap_yuv.addr_phy.addr_u =
        cap_mem->target_yuv.addr_phy.addr_u + y_end - tmp + gap_size;
    cap_mem->cap_yuv.addr_vir.addr_u =
        cap_mem->target_yuv.addr_vir.addr_u + y_end - tmp + gap_size;

    if (need_rot) {
        offset = ((uint32_t)(y_end * 3) >> 1) + 2 * gap_size;
    } else {
        offset = ((y_end) << 1) + 2 * gap_size;
    }

    CMR_NO_MEM(offset, mem_res);
    mem_end = offset;
    mem_res = mem_res - mem_end;

    if (!need_rot) {
        CMR_NO_MEM(cap_mem->target_jpeg.buf_size, mem_res);
        cap_mem->target_jpeg.addr_phy.addr_y =
            cap_2_frm->mem_frm.addr_phy.addr_y + mem_end;
        cap_mem->target_jpeg.addr_vir.addr_y =
            cap_2_frm->mem_frm.addr_vir.addr_y + mem_end;
        cap_mem->target_jpeg.fd = cap_2_frm->mem_frm.fd;
        mem_end += cap_mem->target_jpeg.buf_size;
        mem_res -= cap_mem->target_jpeg.buf_size;
    }

    cap_mem->cap_yuv.buf_size = channel_size * 3 / 2;
    cap_mem->cap_yuv.size.width = align16_cap_size.width;
    cap_mem->cap_yuv.size.height = align16_cap_size.height;
    cap_mem->cap_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->target_yuv.buf_size = channel_size * 3 / 2;
    cap_mem->target_yuv.size.width = align16_image_size.width;
    cap_mem->target_yuv.size.height = align16_image_size.height;
    cap_mem->target_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    /* update io param */
    *io_mem_res = mem_res;
    *io_mem_end = mem_end;
    *io_channel_size = channel_size;

    return 0;
}

int arrange_yuv_buf(struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
                    struct img_rect *sn_trim, struct img_size *image_size,
                    uint32_t orig_fmt, struct img_size *cap_size,
                    struct img_size *thum_size, struct cmr_cap_mem *capture_mem,
                    uint32_t need_rot, uint32_t *io_mem_res,
                    uint32_t *io_mem_end, uint32_t *io_channel_size) {
    // UNUSED(sn_size);
    UNUSED(sn_trim);
    UNUSED(thum_size);

    if (CAM_IMG_FMT_JPEG == orig_fmt || CAM_IMG_FMT_BAYER_MIPI_RAW == orig_fmt ||
        NULL == io_mem_res || NULL == io_mem_end || NULL == io_channel_size) {
        return -1;
    }

    uint32_t mem_res = 0, mem_end = 0;
    uint32_t tmp1, tmp2, tmp3, max_size;
    struct cmr_cap_mem *cap_mem = capture_mem;

    mem_res = *io_mem_res;
    mem_end = *io_mem_end;

    tmp1 = image_size->width * image_size->height * 3 / 2;
    tmp2 = cap_size->width * cap_size->height * 3 / 2;
//    tmp3 = sn_size->width * sn_size->height * 3 / 2;
    max_size = tmp1 > tmp2 ? tmp1 : tmp2;
//    max_size = max_size > tmp3 ? max_size : tmp3;
    max_size = (max_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->mem_frm.addr_phy.addr_y;
    cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->mem_frm.addr_vir.addr_y;
    cap_mem->target_yuv.addr_phy.addr_u =
        cap_mem->target_yuv.addr_phy.addr_y +
        image_size->width * image_size->height;
    cap_mem->target_yuv.addr_vir.addr_u =
        cap_mem->target_yuv.addr_vir.addr_y +
        image_size->width * image_size->height;
    cap_mem->target_yuv.fd = cap_2_frm->mem_frm.fd;
    cap_mem->target_yuv.buf_size =
        image_size->width * image_size->height * 3 / 2;
    cap_mem->target_yuv.size.width = image_size->width;
    cap_mem->target_yuv.size.height = image_size->height;
    cap_mem->target_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->cap_yuv.addr_phy.addr_y =
        cap_mem->target_yuv.addr_phy.addr_y + max_size;
    cap_mem->cap_yuv.addr_vir.addr_y =
        cap_mem->target_yuv.addr_vir.addr_y + max_size;
    cap_mem->cap_yuv.addr_phy.addr_u =
        cap_mem->cap_yuv.addr_phy.addr_y + cap_size->width * cap_size->height;
    cap_mem->cap_yuv.addr_vir.addr_u =
        cap_mem->cap_yuv.addr_vir.addr_y + cap_size->width * cap_size->height;
    cap_mem->cap_yuv.fd = cap_mem->target_yuv.fd;
    cap_mem->cap_yuv.buf_size = cap_size->width * cap_size->height * 3 / 2;
    cap_mem->cap_yuv.size.width = cap_size->width;
    cap_mem->cap_yuv.size.height = cap_size->height;
    cap_mem->cap_yuv.fmt = CAM_IMG_FMT_YUV420_NV21;

    cap_mem->target_jpeg.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
    cap_mem->target_jpeg.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
    cap_mem->target_jpeg.fd = cap_mem->cap_yuv.fd;

    tmp1 = max_size + max_size;
    tmp2 = max_size + cap_mem->target_jpeg.buf_size;
    tmp2 = tmp1 > tmp2 ? tmp1 : tmp2;
    tmp2 = (tmp2 + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    CMR_NO_MEM(tmp2, mem_res);

    *io_mem_res = mem_res - tmp2;
    *io_mem_end = mem_end + tmp2;

    return 0;
}

int arrange_misc_buf(struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
                     struct img_rect *sn_trim, struct img_size *image_size,
                     uint32_t orig_fmt, struct img_size *cap_size,
                     struct img_size *thum_size,
                     struct cmr_cap_mem *capture_mem, uint32_t need_rot,
                     uint32_t *io_mem_res, uint32_t *io_mem_end,
                     uint32_t *io_channel_size) {
    UNUSED(sn_size);
    UNUSED(sn_trim);
    UNUSED(orig_fmt);
    UNUSED(need_rot);

    if (NULL == io_mem_res || NULL == io_mem_end || NULL == io_channel_size) {
        return -1;
    }

    uint32_t size_pixel;
    uint32_t mem_res = 0, mem_end = 0;
    uint32_t thumb_yuv_size, thumb_jpeg_size;
    uint32_t i = 0;
    struct cmr_cap_mem *cap_mem = capture_mem; /*&capture_mem[0];*/
    struct img_frm img_frame[BUF_TYPE_NUM];

    thumb_yuv_size = thum_size->width * thum_size->width * 3 / 2;
    thumb_yuv_size = (thumb_yuv_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    thumb_jpeg_size = thum_size->width * thum_size->width * 3 / 2;
    thumb_jpeg_size = (thumb_jpeg_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

    mem_res = *io_mem_res;
    mem_end = *io_mem_end;

    if (thumb_yuv_size + thumb_jpeg_size > mem_res) {
        CMR_LOGE("reserved memory is not big enough");
        return -1;
    }

    cap_mem->thum_yuv.buf_size = thumb_yuv_size;
    cap_mem->thum_yuv.addr_phy.addr_y =
        cap_2_frm->mem_frm.addr_phy.addr_y + mem_end;
    cap_mem->thum_yuv.addr_phy.addr_u = cap_mem->thum_yuv.addr_phy.addr_y +
                                        thum_size->width * thum_size->height;
    cap_mem->thum_yuv.addr_vir.addr_y =
        cap_2_frm->mem_frm.addr_vir.addr_y + mem_end;
    cap_mem->thum_yuv.addr_vir.addr_u = cap_mem->thum_yuv.addr_vir.addr_y +
                                        thum_size->width * thum_size->height;
    cap_mem->thum_yuv.fd = cap_2_frm->mem_frm.fd;
    cap_mem->thum_yuv.size.width = thum_size->width;
    cap_mem->thum_yuv.size.height = thum_size->height;

    mem_res -= thumb_yuv_size;
    mem_end += thumb_yuv_size;

    cap_mem->thum_jpeg.buf_size = thumb_jpeg_size;
    cap_mem->thum_jpeg.addr_phy.addr_y =
        cap_2_frm->mem_frm.addr_phy.addr_y + mem_end;
    cap_mem->thum_jpeg.addr_vir.addr_y =
        cap_2_frm->mem_frm.addr_vir.addr_y + mem_end;
    cap_mem->thum_jpeg.fd = cap_2_frm->mem_frm.fd;

    cap_mem->jpeg_tmp.buf_size = cap_mem->cap_yuv.buf_size;
    cap_mem->jpeg_tmp.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_u;
    cap_mem->jpeg_tmp.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_u;
    cap_mem->jpeg_tmp.fd = cap_mem->cap_yuv.fd;

    mem_res -= thumb_jpeg_size;
    mem_end += thumb_jpeg_size;

    *io_mem_res = mem_res;
    *io_mem_end = mem_end;

    CMR_LOGD("mem_res, mem_end 0x%x, 0x%x", mem_res, mem_end);

    return 0;
}

int arrange_rot_buf(struct cmr_cap_2_frm *cap_2_frm, struct img_size *sn_size,
                    struct img_rect *sn_trim, struct img_size *image_size,
                    uint32_t orig_fmt, struct img_size *cap_size,
                    struct img_size *thum_size, struct cmr_cap_mem *capture_mem,
                    uint32_t need_rot, uint32_t need_scale,
                    uint32_t *io_mem_res, uint32_t *io_mem_end,
                    uint32_t *io_channel_size) {
    if (NULL == io_mem_res || NULL == io_mem_end || NULL == io_channel_size) {
        return -1;
    }

    if (!need_rot) {
        return -1;
    }

    uint32_t size_pixel;
    uint32_t mem_res = 0, mem_end = 0;
    uint32_t offset = 0, offset_1;
    uint32_t tmp1, tmp2, tmp3, max_size;
    struct cmr_cap_mem *cap_mem = capture_mem; /*&capture_mem[0];*/

    mem_res = *io_mem_res;
    mem_end = *io_mem_end;

    tmp1 = image_size->width * image_size->height;
    tmp2 = cap_size->width * cap_size->height;
    tmp3 = sn_size->width * sn_size->height;
    max_size = tmp1 > tmp2 ? tmp1 : tmp2;
    max_size = max_size > tmp3 ? max_size : tmp3;

    if (ZOOM_POST_PROCESS == cap_2_frm->zoom_post_proc ||
        ZOOM_POST_PROCESS_WITH_TRIM == cap_2_frm->zoom_post_proc) {
        cap_mem->cap_yuv_rot.addr_phy.addr_y =
            cap_mem->target_yuv.addr_phy.addr_y;
        cap_mem->cap_yuv_rot.addr_vir.addr_y =
            cap_mem->target_yuv.addr_vir.addr_y;
        cap_mem->cap_yuv_rot.addr_phy.addr_u =
            cap_mem->cap_yuv_rot.addr_phy.addr_y +
            cap_size->width * cap_size->height;
        cap_mem->cap_yuv_rot.addr_vir.addr_u =
            cap_mem->cap_yuv_rot.addr_vir.addr_y +
            cap_size->width * cap_size->height;
        cap_mem->cap_yuv_rot.fd = cap_mem->target_yuv.fd;

        cap_mem->cap_yuv_rot.size.width = cap_size->height;
        cap_mem->cap_yuv_rot.size.height = cap_size->width;
        cap_mem->cap_yuv_rot.buf_size =
            cap_size->width * cap_size->width * 3 / 2;
        cap_mem->cap_yuv_rot.fmt = CAM_IMG_FMT_YUV420_NV21;

        cap_mem->target_yuv.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
        cap_mem->target_yuv.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
        cap_mem->target_yuv.addr_phy.addr_u =
            cap_mem->target_yuv.addr_phy.addr_y +
            image_size->width * image_size->height;
        cap_mem->target_yuv.addr_vir.addr_u =
            cap_mem->target_yuv.addr_vir.addr_y +
            image_size->width * image_size->height;
        cap_mem->target_yuv.fd = cap_mem->cap_yuv.fd;

        /* mem reuse when rot */
        cap_mem->target_jpeg.addr_phy.addr_y =
            cap_mem->cap_yuv_rot.addr_phy.addr_y;
        cap_mem->target_jpeg.addr_vir.addr_y =
            cap_mem->cap_yuv_rot.addr_vir.addr_y;
        cap_mem->target_jpeg.fd = cap_mem->cap_yuv_rot.fd;
    } else {
        if (need_scale) {
            cap_mem->cap_yuv_rot.addr_phy.addr_y =
                cap_mem->target_yuv.addr_phy.addr_y;
            cap_mem->cap_yuv_rot.addr_vir.addr_y =
                cap_mem->target_yuv.addr_vir.addr_y;
            cap_mem->cap_yuv_rot.addr_phy.addr_u =
                cap_mem->cap_yuv_rot.addr_phy.addr_y +
                cap_size->width * cap_size->height;
            cap_mem->cap_yuv_rot.addr_vir.addr_u =
                cap_mem->cap_yuv_rot.addr_vir.addr_y +
                cap_size->width * cap_size->height;
            cap_mem->cap_yuv_rot.fd = cap_mem->target_yuv.fd;

            cap_mem->cap_yuv_rot.size.width = cap_size->height;
            cap_mem->cap_yuv_rot.size.height = cap_size->width;
            cap_mem->cap_yuv_rot.buf_size =
                cap_size->height * cap_size->width * 3 / 2;
            cap_mem->cap_yuv_rot.fmt = CAM_IMG_FMT_YUV420_NV21;
        } else {
            cap_mem->cap_yuv_rot.addr_phy.addr_y =
                cap_mem->cap_yuv.addr_phy.addr_y;
            cap_mem->cap_yuv_rot.addr_vir.addr_y =
                cap_mem->cap_yuv.addr_vir.addr_y;
            cap_mem->cap_yuv_rot.addr_phy.addr_u =
                cap_mem->cap_yuv_rot.addr_phy.addr_y +
                cap_size->width * cap_size->height;
            cap_mem->cap_yuv_rot.addr_vir.addr_u =
                cap_mem->cap_yuv_rot.addr_vir.addr_y +
                cap_size->width * cap_size->height;
            cap_mem->cap_yuv_rot.fd = cap_mem->cap_yuv.fd;

            cap_mem->cap_yuv_rot.size.width = cap_size->height;
            cap_mem->cap_yuv_rot.size.height = cap_size->width;
            cap_mem->cap_yuv_rot.buf_size =
                cap_size->height * cap_size->width * 3 / 2;
            cap_mem->cap_yuv_rot.fmt = CAM_IMG_FMT_YUV420_NV21;
        }

        /* mem reuse when rot */
        cap_mem->target_jpeg.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
        cap_mem->target_jpeg.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
        cap_mem->target_jpeg.fd = cap_mem->cap_yuv.fd;
    }

    CMR_LOGD("mem_res=%d, mem_end=%d", mem_res, mem_end);
    return 0;
}

uint32_t get_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width,
                       uint32_t thum_height) {
    uint32_t size;
    (void)thum_width;
    (void)thum_height;

    if ((width * height) <= JPEG_SMALL_SIZE) {
        size = CMR_JPEG_SZIE(width, height) + JPEG_EXIF_SIZE;
    } else {
        size = CMR_JPEG_SZIE(width, height);
    }

    return ADDR_BY_WORD(size);
}

uint32_t get_thum_yuv_size(uint32_t width, uint32_t height, uint32_t thum_width,
                           uint32_t thum_height) {
    (void)width;
    (void)height;
    return (uint32_t)(CMR_ADDR_ALIGNED((thum_width * thum_height)) * 3 / 2);
}
uint32_t get_thum_jpeg_size(uint32_t width, uint32_t height,
                            uint32_t thum_width, uint32_t thum_height) {
    uint32_t size;

    (void)width;
    (void)height;
    size = CMR_JPEG_SZIE(thum_width, thum_height);
    return ADDR_BY_WORD(size);
}

uint32_t get_jpg_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width,
                          uint32_t thum_height) {
    (void)thum_width;
    (void)thum_height;
    return (uint32_t)(width * CMR_SLICE_HEIGHT * 2); // TBD
}
uint32_t get_scaler_tmp_size(uint32_t width, uint32_t height,
                             uint32_t thum_width, uint32_t thum_height) {
    UNUSED(width);
    UNUSED(height);

    uint32_t slice_buffer;

    (void)thum_width;
    (void)thum_height;

    slice_buffer = 0; /*(uint32_t)(width * CMR_SLICE_HEIGHT *
                         CMR_ZOOM_FACTOR);only UV422 need dwon sample to UV420*/

    return slice_buffer;
}

uint32_t get_isp_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width,
                          uint32_t thum_height) {
    uint32_t slice_buffer;

    (void)thum_width;
    (void)thum_height;
    UNUSED(width);
    UNUSED(height);

    slice_buffer = (uint32_t)(
        width * CMR_SLICE_HEIGHT); /*only UV422 need dwon sample to UV420*/

    return slice_buffer;
}

uint32_t camera_get_aligned_size(uint32_t type, uint32_t size) {
    uint32_t size_aligned = 0;

    CMR_LOGD("type %d", type);
    if (CAMERA_MEM_NO_ALIGNED == type) {
        size_aligned = size;
    } else {
        size_aligned = CAMERA_ALIGNED_16(size);
    }

    return size_aligned;
}

void camera_set_mem_multimode(multiCameraMode camera_mode) {
    CMR_LOGD("camera_mode %d", camera_mode);
    is_multi_camera_mode_mem = camera_mode;
}
