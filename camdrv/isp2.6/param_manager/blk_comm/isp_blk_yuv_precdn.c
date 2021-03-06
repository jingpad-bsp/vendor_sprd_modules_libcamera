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
#define LOG_TAG "isp_blk_yuv_precdn"
#include "isp_blocks_cfg.h"

static cmr_u32 _pm_yuv_precdn_convert_param(
	void *dst_param, cmr_u32 strength_level, cmr_u32 mode_flag, cmr_u32 scene_flag)
{
	cmr_s32 rtn = ISP_SUCCESS;
	cmr_s32 i = 0;
	cmr_s32 j = 0;
	cmr_u32 total_offset_units = 0;
	struct isp_yuv_precdn_param *dst_ptr = (struct isp_yuv_precdn_param *)dst_param;
	struct sensor_yuv_precdn_level *precdn_param = PNULL;

	if (SENSOR_MULTI_MODE_FLAG != dst_ptr->nr_mode_setting) {
		precdn_param = (struct sensor_yuv_precdn_level *)(dst_ptr->param_ptr);
	} else {
		cmr_u32 *multi_nr_map_ptr = PNULL;
		multi_nr_map_ptr = (cmr_u32 *) dst_ptr->scene_ptr;
		total_offset_units = _pm_calc_nr_addr_offset(mode_flag, scene_flag, multi_nr_map_ptr);
		precdn_param = (struct sensor_yuv_precdn_level *)((cmr_u8 *) dst_ptr->param_ptr +
				total_offset_units * dst_ptr->level_num * sizeof(struct sensor_yuv_precdn_level));
	}
	strength_level = PM_CLIP(strength_level, 0, dst_ptr->level_num - 1);
	if (precdn_param != PNULL) {
		dst_ptr->cur.mode = precdn_param[strength_level].precdn_comm.precdn_mode;
		dst_ptr->cur.median_mode = precdn_param[strength_level].precdn_comm.median_mode;
		dst_ptr->cur.median_thr = precdn_param[strength_level].precdn_comm.median_thr;
		dst_ptr->cur.median_writeback_en = precdn_param[strength_level].precdn_comm.median_writeback_en;
		dst_ptr->cur.den_stren = precdn_param[strength_level].precdn_comm.den_stren;
		dst_ptr->cur.uv_joint = precdn_param[strength_level].precdn_comm.uv_joint;
		dst_ptr->cur.uv_thr = precdn_param[strength_level].precdn_comm.uv_thr;
		dst_ptr->cur.y_thr = precdn_param[strength_level].precdn_comm.y_thr;
		dst_ptr->cur.median_thr_uv.thru0 = precdn_param[strength_level].precdn_comm.median_thr_u[0];
		dst_ptr->cur.median_thr_uv.thru1 = precdn_param[strength_level].precdn_comm.median_thr_u[1];
		dst_ptr->cur.median_thr_uv.thrv0 = precdn_param[strength_level].precdn_comm.median_thr_v[0];
		dst_ptr->cur.median_thr_uv.thrv1 = precdn_param[strength_level].precdn_comm.median_thr_v[1];

		for (i = 0; i < 2; i++) {
			for (j = 0; j < 7; j++) {
				dst_ptr->cur.r_segu[i][j] = precdn_param[strength_level].r_segu[i][j];
				dst_ptr->cur.r_segv[i][j] = precdn_param[strength_level].r_segv[i][j];
				dst_ptr->cur.r_segy[i][j] = precdn_param[strength_level].r_segy[i][j];
			}
		}
		for (i = 0; i < 25; i++) {
			dst_ptr->cur.r_distw[i] = precdn_param[strength_level].dist_w[i];
		}

		dst_ptr->cur.bypass = precdn_param[strength_level].bypass;
	}

	return rtn;

}

cmr_s32 _pm_yuv_precdn_init(
	void *dst_pre_cdn_param, void *src_pre_cdn_param,
	void *param1, void *param2)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_pm_nr_header_param *src_ptr = (struct isp_pm_nr_header_param *)src_pre_cdn_param;
	struct isp_yuv_precdn_param *dst_ptr = (struct isp_yuv_precdn_param *)dst_pre_cdn_param;
	struct isp_pm_block_header *header_ptr = (struct isp_pm_block_header *)param1;
	UNUSED(param2);

	dst_ptr->cur.bypass = header_ptr->bypass;

	dst_ptr->cur_level = src_ptr->default_strength_level;
	dst_ptr->level_num = src_ptr->level_number;
	dst_ptr->param_ptr = src_ptr->param_ptr;
	dst_ptr->scene_ptr = src_ptr->multi_nr_map_ptr;
	dst_ptr->nr_mode_setting = src_ptr->nr_mode_setting;
	if (!header_ptr->bypass)
		rtn = _pm_yuv_precdn_convert_param(dst_ptr, dst_ptr->cur_level, ISP_MODE_ID_COMMON, ISP_SCENEMODE_AUTO);
	dst_ptr->cur.bypass |= header_ptr->bypass;
	if (ISP_SUCCESS != rtn) {
		ISP_LOGE("fail to  convert pm yuv precdn param!");
		return rtn;
	}
	header_ptr->is_update = ISP_ONE;
	return rtn;
}

cmr_s32 _pm_yuv_precdn_set_param(void *pre_cdn_param, cmr_u32 cmd, void *param_ptr0, void *param_ptr1)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_yuv_precdn_param *dst_ptr = (struct isp_yuv_precdn_param *)pre_cdn_param;
	struct isp_pm_block_header *header_ptr = (struct isp_pm_block_header *)param_ptr1;

	switch (cmd) {
	case ISP_PM_BLK_YUV_PRE_CDN_BYPASS:
		dst_ptr->cur.bypass = *((cmr_u32 *) param_ptr0);
		header_ptr->is_update = ISP_ONE;
		break;

	case ISP_PM_BLK_YUV_PRE_CDN_STRENGTH_LEVEL:
		dst_ptr->cur_level = *((cmr_u32 *) param_ptr0);
		header_ptr->is_update = ISP_ONE;
		break;

	case ISP_PM_BLK_SMART_SETTING:
		{
			struct smart_block_result *block_result = (struct smart_block_result *)param_ptr0;
			struct isp_range val_range = { 0, 0 };
			cmr_u32 cur_level = 0;

			if (!block_result->update || header_ptr->bypass) {
				ISP_LOGV("do not need update\n");
				return ISP_SUCCESS;
			}
			val_range.min = 0;
			val_range.max = 255;
			rtn = _pm_check_smart_param(block_result, &val_range, 1, ISP_SMART_Y_TYPE_VALUE);
			if (ISP_SUCCESS != rtn) {
				ISP_LOGE("fail to check pm smart param !");
				return rtn;
			}

			cur_level = (cmr_u32) block_result->component[0].fix_data[0];

			if (cur_level != dst_ptr->cur_level || nr_tool_flag[ISP_BLK_YUV_PRECDN_T] || block_result->mode_flag_changed) {
				dst_ptr->cur_level = cur_level;
				header_ptr->is_update = ISP_ONE;
				nr_tool_flag[ISP_BLK_YUV_PRECDN_T] = 0;

				rtn = _pm_yuv_precdn_convert_param(dst_ptr,
						dst_ptr->cur_level, header_ptr->mode_id, block_result->scene_flag);
				dst_ptr->cur.bypass |= header_ptr->bypass;
				if (ISP_SUCCESS != rtn) {
					ISP_LOGE("fail to  convert pm yuv precdn param!");
					return rtn;
				}
			}
			ISP_LOGV("ISP_SMART_NR: cmd=%d, update=%d, precdn_level=%d", cmd, header_ptr->is_update, dst_ptr->cur_level);
		}
		break;

	default:
		break;
	}

	return rtn;
}

cmr_s32 _pm_yuv_precdn_get_param(
	void *pre_cdn_param, cmr_u32 cmd, void *rtn_param0, void *rtn_param1)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_yuv_precdn_param *pre_cdn_ptr = (struct isp_yuv_precdn_param *)pre_cdn_param;
	struct isp_pm_param_data *param_data_ptr = (struct isp_pm_param_data *)rtn_param0;
	cmr_u32 *update_flag = (cmr_u32 *) rtn_param1;

	param_data_ptr->cmd = cmd;

	switch (cmd) {
	case ISP_PM_BLK_ISP_SETTING:
		param_data_ptr->data_ptr = (void *)&pre_cdn_ptr->cur;
		param_data_ptr->data_size = sizeof(pre_cdn_ptr->cur);
		*update_flag = 0;
		break;

	default:
		break;
	}

	return rtn;
}
