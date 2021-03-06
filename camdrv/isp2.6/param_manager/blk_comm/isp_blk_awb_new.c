/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *		http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "isp_blk_awb_new"
#include "isp_blocks_cfg.h"

cmr_s32 _pm_awb_new_init(void *dst_awb_new, void *src_awb_new, void *param1, void *param2)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_awb_param *dst_ptr = (struct isp_awb_param *)dst_awb_new;
	struct isp_pm_block_header *header_ptr = (struct isp_pm_block_header *)param1;
	UNUSED(src_awb_new);
	UNUSED(param2);

	dst_ptr->ct_value = 5000;
	memset((void *)&dst_ptr->cur, 0x00, sizeof(dst_ptr->cur));

	dst_ptr->cur.awbc_bypass = header_ptr->bypass;
	dst_ptr->cur.gain.r = 0x700;
	dst_ptr->cur.gain.gr = 0x400;
	dst_ptr->cur.gain.gb = 0x400;
	dst_ptr->cur.gain.b = 0x5d0;
	dst_ptr->cur.thrd.r = 0x3ff;
	dst_ptr->cur.thrd.gr = 0x3ff;
	dst_ptr->cur.thrd.gb = 0x3ff;
	dst_ptr->cur.thrd.b = 0x3ff;
	dst_ptr->cur.gain_offset.r = 0;
	dst_ptr->cur.gain_offset.gr = 0;
	dst_ptr->cur.gain_offset.gb = 0;
	dst_ptr->cur.gain_offset.b = 0;

	header_ptr->is_update = ISP_ONE;

	return rtn;

}

cmr_s32 _pm_awb_new_set_param(void *awb_new_param, cmr_u32 cmd, void *param_ptr0, void *param_ptr1)
{

	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_awb_param *dst_ptr = (struct isp_awb_param *)awb_new_param;
	struct isp_pm_block_header *awb_header_ptr = (struct isp_pm_block_header *)param_ptr1;

	switch (cmd) {
	case ISP_PM_BLK_AWBC:
		{
			struct isp_awbc_cfg *cfg_ptr = (struct isp_awbc_cfg *)param_ptr0;

			if (cfg_ptr->r_gain == 0 && cfg_ptr->g_gain == 0 && cfg_ptr->b_gain == 0) {
				ISP_LOGW("warn: zero value\n");
				return 0;
			}

			dst_ptr->cur.gain.r = cfg_ptr->r_gain;
			dst_ptr->cur.gain.gr = cfg_ptr->g_gain;
			dst_ptr->cur.gain.gb = cfg_ptr->g_gain;
			dst_ptr->cur.gain.b = cfg_ptr->b_gain;
			dst_ptr->cur.gain_offset.r = cfg_ptr->r_offset;
			dst_ptr->cur.gain_offset.gr = cfg_ptr->g_offset;
			dst_ptr->cur.gain_offset.gb = cfg_ptr->g_offset;
			dst_ptr->cur.gain_offset.b = cfg_ptr->b_offset;
			awb_header_ptr->is_update = ISP_ONE;
		}
		break;

	case ISP_PM_BLK_AWBC_BYPASS:
		dst_ptr->cur.awbc_bypass = *((cmr_u32 *) param_ptr0);
		awb_header_ptr->is_update = ISP_ONE;
		break;

	default:
		awb_header_ptr->is_update = ISP_ZERO;
		break;
	}

	return rtn;

}

cmr_s32 _pm_awb_new_get_param(void *awb_new_param, cmr_u32 cmd, void *rtn_param0, void *rtn_param1)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_awb_param *awb_param_ptr = (struct isp_awb_param *)awb_new_param;
	struct isp_pm_param_data *param_data_ptr = (struct isp_pm_param_data *)rtn_param0;
	cmr_u32 *update_flag = (cmr_u32 *) rtn_param1;

	param_data_ptr->cmd = cmd;

	switch (cmd) {
	case ISP_PM_BLK_ISP_SETTING:
		param_data_ptr->data_ptr = &awb_param_ptr->cur;
		param_data_ptr->data_size = sizeof(awb_param_ptr->cur);
		*update_flag = 0;
		break;

	default:
		break;
	}

	return rtn;

}
