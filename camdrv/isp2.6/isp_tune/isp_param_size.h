/*
 * Copyright (C) 2018 The Android Open Source Project
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
#ifndef _ISP_PARAM_SIZE_H_
#define _ISP_PARAM_SIZE_H_

#include <sys/types.h>
#include <cmr_types.h>

#ifdef   __cplusplus
extern "C" {
#endif

#define ISP_SIZE_480x270   (1<<0)
#define ISP_SIZE_640x480   (1<<1)
#define ISP_SIZE_480x360   (1<<2)
#define ISP_SIZE_800x600   (1<<3)
#define ISP_SIZE_1280x960  (1<<4)
#define ISP_SIZE_1296x972  (1<<5)
#define ISP_SIZE_1280x1024 (1<<6)
#define ISP_SIZE_1600x1200 (1<<7)
#define ISP_SIZE_1632x1224 (1<<8)
#define ISP_SIZE_1920x1080 (1<<9)
#define ISP_SIZE_2048x1536 (1<<10)
#define ISP_SIZE_2112x1568 (1<<11)
#define ISP_SIZE_2592x1944 (1<<12)
#define ISP_SIZE_3264x2448 (1<<13)
#define ISP_SIZE_4144x3106 (1<<14)
#define ISP_SIZE_4208x3120 (1<<15)
#define ISP_SIZE_END 0xffffffff

struct isp_size_info {
	cmr_u32 size_id;
	cmr_u32 width;
	cmr_u32 height;
};

struct isp_size_info *ISP_ParamGetSizeInfo(void);


#ifdef   __cplusplus
}
#endif
#endif
