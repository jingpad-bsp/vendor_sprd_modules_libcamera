/*
 *******************************************************************************
 * $Header$
 *
 *  Copyright (c) 2016-2025 Spreadtrum Inc. All rights reserved.
 *
 *  +-----------------------------------------------------------------+
 *  | THIS SOFTWARE IS FURNISHED UNDER A LICENSE AND MAY ONLY BE USED |
 *  | AND COPIED IN ACCORDANCE WITH THE TERMS AND CONDITIONS OF SUCH  |
 *  | A LICENSE AND WITH THE INCLUSION OF THE THIS COPY RIGHT NOTICE. |
 *  | THIS SOFTWARE OR ANY OTHER COPIES OF THIS SOFTWARE MAY NOT BE   |
 *  | PROVIDED OR OTHERWISE MADE AVAILABLE TO ANY OTHER PERSON. THE   |
 *  | OWNERSHIP AND TITLE OF THIS SOFTWARE IS NOT TRANSFERRED.        |
 *  |                                                                 |
 *  | THE INFORMATION IN THIS SOFTWARE IS SUBJECT TO CHANGE WITHOUT   |
 *  | ANY PRIOR NOTICE AND SHOULD NOT BE CONSTRUED AS A COMMITMENT BY |
 *  | SPREADTRUM INC.                                                 |
 *  +-----------------------------------------------------------------+
 *
 *
 *******************************************************************************
 */

#ifndef _CMR_TYPES_H_
#define _CMR_TYPES_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include <sys/types.h>
#include <utils/Log.h>

enum camera_mem_cb_type {
    CAMERA_PREVIEW = 0,
    CAMERA_SNAPSHOT,
    CAMERA_SNAPSHOT_ZSL,
    CAMERA_VIDEO,
    CAMERA_PREVIEW_RESERVED,
    CAMERA_SNAPSHOT_ZSL_RESERVED_MEM,
    CAMERA_SNAPSHOT_ZSL_RESERVED,
    CAMERA_VIDEO_RESERVED,
    CAMERA_ISP_LSC,
    CAMERA_ISP_BINGING4AWB,
    CAMERA_SNAPSHOT_PATH,
    CAMERA_ISP_FIRMWARE,
    CAMERA_SNAPSHOT_HIGHISO,
    CAMERA_ISP_RAW_DATA,
    CAMERA_ISP_ANTI_FLICKER,
    CAMERA_ISP_RAWAE,
    CAMERA_ISP_PREVIEW_Y,
    CAMERA_ISP_PREVIEW_YUV,
    CAMERA_DEPTH_MAP,
    CAMERA_DEPTH_MAP_RESERVED,
    CAMERA_PDAF_RAW,
    CAMERA_PDAF_RAW_RESERVED,
    CAMERA_ISP_STATIS,
    CAMERA_SNAPSHOT_3DNR,
    CAMERA_SNAPSHOT_3DNR_DST,
    CAMERA_PREVIEW_3DNR,
    CAMERA_PREVIEW_SCALE_3DNR,
    CAMERA_PREVIEW_SCALE_AI_SCENE,
    CAMERA_PREVIEW_SCALE_AUTO_TRACKING,
    CAMERA_PREVIEW_DEPTH,
    CAMERA_PREVIEW_SW_OUT,
    CAMERA_PREVIEW_ULTRA_WIDE,
    CAMERA_VIDEO_ULTRA_WIDE,
    CAMERA_VIDEO_EIS_ULTRA_WIDE,
    CAMERA_SNAPSHOT_ULTRA_WIDE,
    CAMERA_SNAPSHOT_SW3DNR,
    CAMERA_SNAPSHOT_SW3DNR_PATH,
    CAMERA_4IN1_PROC,
    CAMERA_SNAPSHOT_SLAVE_RESERVED,
    CAMERA_ISPSTATS_AEM,
    CAMERA_ISPSTATS_AFM,
    CAMERA_ISPSTATS_AFL,
    CAMERA_ISPSTATS_PDAF,
    CAMERA_ISPSTATS_BAYERHIST,
    CAMERA_ISPSTATS_YUVHIST,
    CAMERA_ISPSTATS_LSCM,
    CAMERA_ISPSTATS_3DNR,
    CAMERA_ISPSTATS_EBD,
    CAMERA_ISPSTATS_DEBUG,
    CAMERA_CHANNEL_0_RESERVED,
    CAMERA_CHANNEL_1,
    CAMERA_CHANNEL_1_RESERVED,
    CAMERA_CHANNEL_2,
    CAMERA_CHANNEL_2_RESERVED,
    CAMERA_CHANNEL_3,
    CAMERA_CHANNEL_3_RESERVED,
    CAMERA_CHANNEL_4,
    CAMERA_CHANNEL_4_RESERVED,
    CAMERA_FD_SMALL,
    CAMERA_SNAPSHOT_SW3DNR_SMALL_PATH,
    CAMERA_MACRO,
    CAMERA_SNAPSHOT_ZSL_RAW,
    CAMERA_MEM_CB_TYPE_MAX
};

typedef unsigned long cmr_uint;
typedef long cmr_int;
typedef uint64_t cmr_u64;
typedef int64_t cmr_s64;
typedef unsigned int cmr_u32;
typedef int cmr_s32;
typedef unsigned short cmr_u16;
typedef short cmr_s16;
typedef unsigned char cmr_u8;
typedef signed char cmr_s8;
typedef void *cmr_handle;

#ifndef bzero
#define bzero(p, len) memset(p, 0, len);
#endif

#ifndef UNUSED
#define UNUSED(x) (void) x
#endif
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef cmr_int (*cmr_malloc)(cmr_u32 mem_type, cmr_handle oem_handle,
                              cmr_u32 *size, cmr_u32 *sum, cmr_uint *phy_addr,
                              cmr_uint *vir_addr, cmr_s32 *fd);
typedef cmr_int (*cmr_free)(cmr_u32 mem_type, cmr_handle oem_handle,
                            cmr_uint *phy_addr, cmr_uint *vir_addr, cmr_s32 *fd,
                            cmr_u32 sum);
typedef cmr_int (*cmr_gpu_malloc)(cmr_u32 mem_type, cmr_handle oem_handle,
                                  cmr_u32 *size, cmr_u32 *sum,
                                  cmr_uint *phy_addr, cmr_uint *vir_addr,
                                  cmr_s32 *fd, void **handle, cmr_uint *width,
                                  cmr_uint *height);

typedef cmr_int (*cmr_invalidate_buf)(cmr_handle oem_handle,
                              cmr_s32 buf_fd, cmr_u32 size,
                              cmr_uint phy_addr, cmr_uint vir_addr);
typedef cmr_int (*cmr_flush_buf)(cmr_handle oem_handle,
                              cmr_s32 buf_fd, cmr_u32 size,
                              cmr_uint phy_addr, cmr_uint vir_addr);
#endif
