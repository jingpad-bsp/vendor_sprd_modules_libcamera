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

#define LOG_TAG "isp_u_fetch"

#include "isp_drv.h"

cmr_s32 isp_u_reg_isr(cmr_handle handle, uint32_t *mode)
{
       cmr_s32 ret = 0;
       struct isp_file *file = NULL;

       if (!handle) {
               ISP_LOGE("failed to get handle");
               return -1;
       }

       file = (struct isp_file *)handle;

       ret = ioctl(file->fd, SPRD_ISP_IO_REG_ISP_ISR, mode);
       return ret;
}

/*add this code to ensure isp isr user func is registered*/
/*in isp ddr_in_ddr_out mode, so store done will be handled*/
/*and send to user space*/
cmr_s32 isp_u_raw_proc_start(cmr_handle handle)
{
	uint32_t mode = 1;
	return isp_u_reg_isr(handle, &mode);
}

cmr_s32 isp_u_raw_proc_end(cmr_handle handle)
{
	uint32_t mode = 0;
	return isp_u_reg_isr(handle, &mode);
}

cmr_s32 isp_u_fetch_block(cmr_handle handle, void *block_info)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;

	if (!handle || !block_info) {
		ISP_LOGE("fail to get handle: handle = %p, block_info = %p.", handle, block_info);
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.isp_id = file->isp_id;
	param.sub_block = ISP_BLOCK_FETCH;
	param.property = ISP_PRO_FETCH_BLOCK;
	param.property_param = block_info;

	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);

	return ret;
}

cmr_s32 isp_u_fetch_raw_transaddr(cmr_handle handle, struct isp_dev_block_addr * addr)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;

	if (!handle) {
		ISP_LOGE("fail to get handle.");
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.isp_id = file->isp_id;
	param.sub_block = ISP_BLOCK_FETCH;
	param.property = ISP_PRO_FETCH_TRANSADDR;
	param.property_param = addr;

	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);

	return ret;
}

cmr_s32 isp_u_fetch_slice_size(cmr_handle handle, cmr_u32 w, cmr_u32 h)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;
	struct isp_img_size size;

	if (!handle) {
		ISP_LOGE("fail to get handle.");
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.isp_id = file->isp_id;
	param.sub_block = ISP_BLOCK_FETCH;
	param.property = ISP_PRO_FETCH_SLICE_SIZE;
	size.width = w;
	size.height = h;
	param.property_param = &size;

	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);

	return ret;
}

cmr_s32 isp_u_fetch_start_isp(cmr_handle handle, cmr_u32 fetch_start)
{
	cmr_s32 ret = 0;
	struct isp_file *file = NULL;
	struct isp_io_param param;

	if (!handle) {
		ISP_LOGE("fail to get handle.");
		return -1;
	}

	file = (struct isp_file *)(handle);
	param.isp_id = file->isp_id;
	param.sub_block = ISP_BLOCK_FETCH;
	param.property = ISP_PRO_FETCH_START_ISP;
	param.property_param = &fetch_start;

	ret = isp_u_raw_proc_start(handle);
	ret = ioctl(file->fd, SPRD_ISP_IO_CFG_PARAM, &param);
	return ret;
}
