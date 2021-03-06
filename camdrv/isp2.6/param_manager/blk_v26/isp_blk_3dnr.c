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
#define LOG_TAG "isp_blk_3dnr"
#include "isp_blocks_cfg.h"

cmr_u32 _pm_3dnr_convert_param(
	void *dst_3dnr_param, cmr_u32 strength_level,
	cmr_u32 mode_flag, cmr_u32 scene_flag)
{
	cmr_s32 rtn = ISP_SUCCESS;
	cmr_u32 total_offset_units = 0;
	struct isp_nr3d_param *dst_ptr = (struct isp_nr3d_param *)dst_3dnr_param;
	struct sensor_3dnr_level *nr_3d_param = PNULL;
	cmr_s32 i = 0;

	if (SENSOR_MULTI_MODE_FLAG != dst_ptr->nr_mode_setting) {
		nr_3d_param = (struct sensor_3dnr_level *)(dst_ptr->param_ptr);
	} else {
		cmr_u32 *multi_nr_map_ptr = PNULL;
		multi_nr_map_ptr = (cmr_u32 *) dst_ptr->scene_ptr;
		total_offset_units = _pm_calc_nr_addr_offset(mode_flag, scene_flag, multi_nr_map_ptr);
		nr_3d_param = (struct sensor_3dnr_level *)((cmr_u8 *) dst_ptr->param_ptr +
				total_offset_units * dst_ptr->level_num * sizeof(struct sensor_3dnr_level));
	}
	strength_level = PM_CLIP(strength_level, 0, dst_ptr->level_num - 1);

	if (nr_3d_param != NULL) {
		dst_ptr->cur.fast_me.nr3_channel_sel = nr_3d_param[strength_level].fast_me.channel_sel;
		dst_ptr->cur.fast_me.nr3_project_mode = nr_3d_param[strength_level].fast_me.project_mode;

		dst_ptr->cur.blend.fusion_mode = nr_3d_param[strength_level].fusion_mode;
		dst_ptr->cur.blend.filter_switch = nr_3d_param[strength_level].filter_swt_en;

		for (i = 0; i < 4; i++) {
			dst_ptr->cur.blend.y_pixel_src_weight[i] = nr_3d_param[strength_level].yuv_cfg.y_cfg.src_wgt[i];
			dst_ptr->cur.blend.u_pixel_src_weight[i] = nr_3d_param[strength_level].yuv_cfg.u_cfg.src_wgt[i];
			dst_ptr->cur.blend.v_pixel_src_weight[i] = nr_3d_param[strength_level].yuv_cfg.v_cfg.src_wgt[i];
		}

		dst_ptr->cur.blend.y_pixel_noise_threshold = nr_3d_param[strength_level].yuv_cfg.y_cfg.nr_thr;
		dst_ptr->cur.blend.u_pixel_noise_threshold = nr_3d_param[strength_level].yuv_cfg.u_cfg.nr_thr;
		dst_ptr->cur.blend.v_pixel_noise_threshold = nr_3d_param[strength_level].yuv_cfg.v_cfg.nr_thr;
		dst_ptr->cur.blend.y_pixel_noise_weight = nr_3d_param[strength_level].yuv_cfg.y_cfg.nr_wgt;
		dst_ptr->cur.blend.u_pixel_noise_weight = nr_3d_param[strength_level].yuv_cfg.u_cfg.nr_wgt;
		dst_ptr->cur.blend.v_pixel_noise_weight = nr_3d_param[strength_level].yuv_cfg.v_cfg.nr_wgt;

		dst_ptr->cur.blend.threshold_radial_variation_u_range_min = nr_3d_param[strength_level].sensor_3dnr_cor.u_range.min_cap;
		dst_ptr->cur.blend.threshold_radial_variation_u_range_max = nr_3d_param[strength_level].sensor_3dnr_cor.u_range.max_cap;
		dst_ptr->cur.blend.threshold_radial_variation_v_range_min = nr_3d_param[strength_level].sensor_3dnr_cor.v_range.min_cap;
		dst_ptr->cur.blend.threshold_radial_variation_v_range_max = nr_3d_param[strength_level].sensor_3dnr_cor.v_range.max_cap;

		dst_ptr->cur.blend.y_threshold_polyline_0 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[0];
		dst_ptr->cur.blend.y_threshold_polyline_1 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[1];
		dst_ptr->cur.blend.y_threshold_polyline_2 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[2];
		dst_ptr->cur.blend.y_threshold_polyline_3 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[3];
		dst_ptr->cur.blend.y_threshold_polyline_4 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[4];
		dst_ptr->cur.blend.y_threshold_polyline_5 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[5];
		dst_ptr->cur.blend.y_threshold_polyline_6 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[6];
		dst_ptr->cur.blend.y_threshold_polyline_7 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[7];
		dst_ptr->cur.blend.y_threshold_polyline_8 = nr_3d_param[strength_level].yuv_cfg.y_cfg.thr_polyline_cap[8];

		dst_ptr->cur.blend.u_threshold_polyline_0 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[0];
		dst_ptr->cur.blend.u_threshold_polyline_1 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[1];
		dst_ptr->cur.blend.u_threshold_polyline_2 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[2];
		dst_ptr->cur.blend.u_threshold_polyline_3 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[3];
		dst_ptr->cur.blend.u_threshold_polyline_4 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[4];
		dst_ptr->cur.blend.u_threshold_polyline_5 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[5];
		dst_ptr->cur.blend.u_threshold_polyline_6 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[6];
		dst_ptr->cur.blend.u_threshold_polyline_7 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[7];
		dst_ptr->cur.blend.u_threshold_polyline_8 = nr_3d_param[strength_level].yuv_cfg.u_cfg.thr_polyline_cap[8];

		dst_ptr->cur.blend.v_threshold_polyline_0 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[0];
		dst_ptr->cur.blend.v_threshold_polyline_1 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[1];
		dst_ptr->cur.blend.v_threshold_polyline_2 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[2];
		dst_ptr->cur.blend.v_threshold_polyline_3 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[3];
		dst_ptr->cur.blend.v_threshold_polyline_4 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[4];
		dst_ptr->cur.blend.v_threshold_polyline_5 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[5];
		dst_ptr->cur.blend.v_threshold_polyline_6 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[6];
		dst_ptr->cur.blend.v_threshold_polyline_7 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[7];
		dst_ptr->cur.blend.v_threshold_polyline_8 = nr_3d_param[strength_level].yuv_cfg.v_cfg.thr_polyline_cap[8];

		dst_ptr->cur.blend.y_intensity_gain_polyline_0 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[0];
		dst_ptr->cur.blend.y_intensity_gain_polyline_1 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[1];
		dst_ptr->cur.blend.y_intensity_gain_polyline_2 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[2];
		dst_ptr->cur.blend.y_intensity_gain_polyline_3 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[3];
		dst_ptr->cur.blend.y_intensity_gain_polyline_4 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[4];
		dst_ptr->cur.blend.y_intensity_gain_polyline_5 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[5];
		dst_ptr->cur.blend.y_intensity_gain_polyline_6 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[6];
		dst_ptr->cur.blend.y_intensity_gain_polyline_7 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[7];
		dst_ptr->cur.blend.y_intensity_gain_polyline_8 = nr_3d_param[strength_level].yuv_cfg.y_cfg.gain_polyline_cap[8];

		dst_ptr->cur.blend.u_intensity_gain_polyline_0 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[0];
		dst_ptr->cur.blend.u_intensity_gain_polyline_1 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[1];
		dst_ptr->cur.blend.u_intensity_gain_polyline_2 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[2];
		dst_ptr->cur.blend.u_intensity_gain_polyline_3 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[3];
		dst_ptr->cur.blend.u_intensity_gain_polyline_4 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[4];
		dst_ptr->cur.blend.u_intensity_gain_polyline_5 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[5];
		dst_ptr->cur.blend.u_intensity_gain_polyline_6 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[6];
		dst_ptr->cur.blend.u_intensity_gain_polyline_7 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[7];
		dst_ptr->cur.blend.u_intensity_gain_polyline_8 = nr_3d_param[strength_level].yuv_cfg.u_cfg.gain_polyline_cap[8];

		dst_ptr->cur.blend.v_intensity_gain_polyline_0 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[0];
		dst_ptr->cur.blend.v_intensity_gain_polyline_1 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[1];
		dst_ptr->cur.blend.v_intensity_gain_polyline_2 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[2];
		dst_ptr->cur.blend.v_intensity_gain_polyline_3 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[3];
		dst_ptr->cur.blend.v_intensity_gain_polyline_4 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[4];
		dst_ptr->cur.blend.v_intensity_gain_polyline_5 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[5];
		dst_ptr->cur.blend.v_intensity_gain_polyline_6 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[6];
		dst_ptr->cur.blend.v_intensity_gain_polyline_7 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[7];
		dst_ptr->cur.blend.v_intensity_gain_polyline_8 = nr_3d_param[strength_level].yuv_cfg.v_cfg.gain_polyline_cap[8];

		dst_ptr->cur.blend.gradient_weight_polyline_0 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[0];
		dst_ptr->cur.blend.gradient_weight_polyline_1 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[1];
		dst_ptr->cur.blend.gradient_weight_polyline_2 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[2];
		dst_ptr->cur.blend.gradient_weight_polyline_3 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[3];
		dst_ptr->cur.blend.gradient_weight_polyline_4 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[4];
		dst_ptr->cur.blend.gradient_weight_polyline_5 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[5];
		dst_ptr->cur.blend.gradient_weight_polyline_6 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[6];
		dst_ptr->cur.blend.gradient_weight_polyline_7 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[7];
		dst_ptr->cur.blend.gradient_weight_polyline_8 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[8];
		dst_ptr->cur.blend.gradient_weight_polyline_9 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[9];
		dst_ptr->cur.blend.gradient_weight_polyline_10 = nr_3d_param[strength_level].yuv_cfg.grad_wgt_polyline_cap[10];

		dst_ptr->cur.blend.u_threshold_factor0 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_thr[0];
		dst_ptr->cur.blend.u_threshold_factor1 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_thr[1];
		dst_ptr->cur.blend.u_threshold_factor2 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_thr[2];
		dst_ptr->cur.blend.u_threshold_factor3 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_thr[3];
		dst_ptr->cur.blend.v_threshold_factor0 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_thr[0];
		dst_ptr->cur.blend.v_threshold_factor1 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_thr[1];
		dst_ptr->cur.blend.v_threshold_factor2 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_thr[2];
		dst_ptr->cur.blend.v_threshold_factor3 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_thr[3];

		dst_ptr->cur.blend.u_divisor_factor0 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_div[0];
		dst_ptr->cur.blend.u_divisor_factor1 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_div[1];
		dst_ptr->cur.blend.u_divisor_factor2 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_div[2];
		dst_ptr->cur.blend.u_divisor_factor3 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.u_div[3];
		dst_ptr->cur.blend.v_divisor_factor0 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_div[0];
		dst_ptr->cur.blend.v_divisor_factor1 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_div[1];
		dst_ptr->cur.blend.v_divisor_factor2 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_div[2];
		dst_ptr->cur.blend.v_divisor_factor3 = nr_3d_param[strength_level].sensor_3dnr_cor.uv_factor.v_div[3];

		dst_ptr->cur.blend.r1_circle = nr_3d_param[strength_level].sensor_3dnr_cor.r_circle_cap[0];
		dst_ptr->cur.blend.r2_circle = nr_3d_param[strength_level].sensor_3dnr_cor.r_circle_cap[1];
		dst_ptr->cur.blend.r3_circle = nr_3d_param[strength_level].sensor_3dnr_cor.r_circle_cap[2];
		dst_ptr->cur.blend.r1_circle_factor = nr_3d_param[strength_level].sensor_3dnr_cor.r_circle_cap_factor[0];
		dst_ptr->cur.blend.r2_circle_factor = nr_3d_param[strength_level].sensor_3dnr_cor.r_circle_cap_factor[1];
		dst_ptr->cur.blend.r3_circle_factor = nr_3d_param[strength_level].sensor_3dnr_cor.r_circle_cap_factor[2];
		dst_ptr->cur.blend.r_circle_base = nr_3d_param[strength_level].radius_base;
	}
	return rtn;
}

cmr_s32 _pm_3dnr_init(void *dst_3d_nr_param, void *src_3d_nr_param, void *param1, void *param_ptr2)
{
	cmr_s32 rtn = ISP_SUCCESS;

	struct isp_pm_nr_header_param *src_ptr = (struct isp_pm_nr_header_param *)src_3d_nr_param;
	struct isp_nr3d_param *dst_ptr = (struct isp_nr3d_param *)dst_3d_nr_param;
	struct isp_pm_block_header *nr_3d_header_ptr = (struct isp_pm_block_header *)param1;
	UNUSED(param_ptr2);

	dst_ptr->cur_level = src_ptr->default_strength_level;
	dst_ptr->level_num = src_ptr->level_number;
	dst_ptr->param_ptr = src_ptr->param_ptr;
	dst_ptr->scene_ptr = src_ptr->multi_nr_map_ptr;
	dst_ptr->nr_mode_setting = src_ptr->nr_mode_setting;

	rtn = _pm_3dnr_convert_param(dst_ptr, dst_ptr->cur_level, ISP_MODE_ID_COMMON, ISP_SCENEMODE_AUTO);
	if (ISP_SUCCESS != rtn) {
		ISP_LOGE("fail to convert pm 3d nr cap param  !");
		return rtn;
	}
	nr_3d_header_ptr->is_update = ISP_ONE;

	return rtn;
}

cmr_s32 _pm_3dnr_set_param(void *nr_3d_param, cmr_u32 cmd, void *param_ptr0, void *param_ptr1)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_pm_block_header *header_ptr = (struct isp_pm_block_header *)param_ptr1;
	struct isp_nr3d_param *dst_ptr = (struct isp_nr3d_param *)nr_3d_param;

	struct smart_block_result *block_result = (struct smart_block_result *)param_ptr0;
	struct isp_range val_range = { 0, 0 };
	cmr_u32 level = 0;

	switch (cmd) {
	case ISP_PM_BLK_3D_NR_BYPASS:
		header_ptr->is_update = ISP_ONE;
		break;

	case ISP_PM_BLK_3D_NR_STRENGTH_LEVEL:
		dst_ptr->cur_level = *((cmr_u32 *) param_ptr0);
		header_ptr->is_update = ISP_ONE;
		break;

	case ISP_PM_BLK_SMART_SETTING:
		val_range.min = 0;
		val_range.max = 255;

		if (0 == block_result->update) {
			ISP_LOGV("do not need update\n");
			return ISP_SUCCESS;
		}

		rtn = _pm_check_smart_param(block_result, &val_range, 1, ISP_SMART_Y_TYPE_VALUE);
		if (ISP_SUCCESS != rtn) {
			ISP_LOGE("fail to check pm smart param !");
			return rtn;
		}

		level = (cmr_u32) block_result->component[0].fix_data[0];

		if (level != dst_ptr->cur_level || nr_tool_flag[ISP_BLK_3DNR_T] || block_result->mode_flag_changed) {
			dst_ptr->cur_level = level;
			header_ptr->is_update = ISP_ONE;
			nr_tool_flag[ISP_BLK_3DNR_T] = 0;

			rtn = _pm_3dnr_convert_param(dst_ptr, dst_ptr->cur_level, header_ptr->mode_id, block_result->scene_flag);
			if (ISP_SUCCESS != rtn) {
				ISP_LOGE("fail to convert pm 3d nr cap param !");
				return rtn;
			}
		}
		break;

	default:

		break;
	}

	return rtn;
}

cmr_s32 _pm_3dnr_get_param(void *nr_3d_param, cmr_u32 cmd, void *rtn_param0, void *rtn_param1)
{
	cmr_s32 rtn = ISP_SUCCESS;
	struct isp_nr3d_param *nr_3d_ptr = (struct isp_nr3d_param *)nr_3d_param;
	struct isp_pm_param_data *param_data_ptr = (struct isp_pm_param_data *)rtn_param0;
	cmr_u32 *update_flag = (cmr_u32 *) rtn_param1;

	param_data_ptr->cmd = cmd;

	switch (cmd) {
	case ISP_PM_BLK_ISP_SETTING:
		param_data_ptr->data_ptr = &nr_3d_ptr->cur;
		param_data_ptr->data_size = sizeof(nr_3d_ptr->cur);
		*update_flag = ISP_ZERO;
		break;

	default:
		break;
	}

	return rtn;
}
