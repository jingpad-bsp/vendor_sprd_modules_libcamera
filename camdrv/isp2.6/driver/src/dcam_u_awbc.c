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

#define LOG_TAG "dcam_u_awbc"

#include "isp_drv.h"

cmr_s32 dcam_u_awbc_block(cmr_handle handle, void *block_info)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;
	struct isp_u_blocks_info *block_param = (struct isp_u_blocks_info *)block_info;

	if (!handle || !block_info) {
		ISP_LOGE("fail to get handle: handle = %p, block_info = %p.", handle, block_info);
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.scene_id = block_param->scene_id;
	param.sub_block = DCAM_BLOCK_AWBC;
	param.property = DCAM_PRO_AWBC_BLOCK;
	param.property_param = block_param->block_info;

	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);

	return ret;
}

cmr_s32 dcam_u_awbc_bypass(cmr_handle handle, cmr_u32 bypass, cmr_u32 scene_id)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;

	if (!handle) {
		ISP_LOGE("fail to get handle: handle = %p", handle);
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.scene_id = scene_id;
	param.sub_block = DCAM_BLOCK_AWBC;
	param.property = DCAM_PRO_AWBC_BYPASS;
	param.property_param = &bypass;

	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);

	return ret;
}

cmr_s32 dcam_u_awbc_gain(cmr_handle handle, void *block_info)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;
	struct isp_u_blocks_info *block_param = (struct isp_u_blocks_info *)block_info;

	if (!handle || !block_info) {
		ISP_LOGE("fail to get handle.");
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.scene_id = block_param->scene_id;
	param.sub_block = DCAM_BLOCK_AWBC;
	param.property = DCAM_PRO_AWBC_GAIN;
	param.property_param = block_param->block_info;

	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);

	return ret;
}