/*
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef _AE_CTRL_COMMON_H_
#define _AE_CTRL_COMMON_H_
#include "cmr_types.h"
#define SENSOR_GAMMA_POINT_NUM	257

struct ae_senseor_out {
	cmr_s8 stable;
	cmr_s8 f_stable;
	cmr_u8 near_stable;
	cmr_s16 cur_index;			/*the current index of ae table in ae now: 1~1024 */
	cmr_u32 exposure_time;		/*exposure time, unit: 0.1us */
	float cur_fps;				/*current fps:1~120 */
	cmr_u16 cur_exp_line;		/*current exposure line: the value is related to the resolution */
	cmr_u16 cur_dummy;			/*dummy line: the value is related to the resolution & fps */
	cmr_s16 cur_again;			/*current analog gain */
	cmr_s16 cur_dgain;			/*current digital gain */
	cmr_s32 cur_bv;
	cmr_u32 frm_len;
	cmr_u32 frm_len_def;
	cmr_u8 binning_mode;
};

struct ae_sample_s {
	cmr_s16 x;
	cmr_s16 y;
};

struct ae_point_type_s {
	cmr_s16 x;
	cmr_s16 y;
};

struct ae_size_s {
	cmr_u32 w;
	cmr_u32 h;
};

struct ae_ev_setting_param_s {
	cmr_u32 ae_idx;
	cmr_u32 exp_time;
	cmr_u32 line_time;
	cmr_u32 exp_line;
	cmr_u32 dmy_line;
	cmr_u32 frm_len;
	cmr_u32 ae_gain;
	cmr_u32 calc_y;
};
struct ae_rgbgamma_curve_s {
	struct ae_sample_s points_r[SENSOR_GAMMA_POINT_NUM];/*gamma curve for r channel*/
	struct ae_sample_s points_g[SENSOR_GAMMA_POINT_NUM];/*gamma curve for g channel*/
	struct ae_sample_s points_b[SENSOR_GAMMA_POINT_NUM];/*gamma curve for b channel*/
};
struct ae_ygamma_curve_s {
	struct ae_sample_s points_y[SENSOR_GAMMA_POINT_NUM];/*gamma curve for y channel*/
};

struct ae_lib_calc_out_2_x {
	cmr_u32 version;			/*version No. for this structure */
	cmr_s16 cur_lum;			/*the lum of image:0 ~255 */
	cmr_s16 cur_lum_avg;		/*the lum without weight of image:0 ~255*/
	cmr_s16 target_lum;			/*the ae target lum: 0 ~255 */
	cmr_s16 target_range_in;		/*ae target lum stable zone: 0~255 */
	cmr_s16 target_range_out;
	cmr_s16 target_zone;		/*ae target lum stable zone: 0~255 */
	cmr_s16 target_lum_ori;		/*the ae target lum(original): 0 ~255 */
	cmr_s16 target_zone_ori;	/*the ae target lum stable zone(original):0~255 */
	cmr_u32 frame_id;
	cmr_s16 cur_bv;				/*bv parameter */
	cmr_s16 cur_bv_nonmatch;
	cmr_s16 *histogram;			/*luma histogram of current frame */
	//for flash
	cmr_s32 flash_effect;
	cmr_s8 flash_status;
	cmr_s16 mflash_exp_line;
	cmr_s16 mflash_dummy;
	cmr_s16 mflash_gain;
	//for touch
	cmr_s8 tcAE_status;
	cmr_s8 tcRls_flag;
	//for face debug
	cmr_u32 face_lum;
	void *pmulaes;
	void *pflat;
	void *pregion;
	void *pai;
	void *pabl;
	void *ppcp;
	void *phm;
	void *pns;
	void *ptc;					/*Bethany add touch info to debug info */
	void *pface_ae;
	struct ae_senseor_out wts;
	cmr_handle log;
	cmr_u32 flag4idx;
	cmr_u32 face_stable;
	cmr_u32 face_enable;
	cmr_u32 face_trigger;
	cmr_u32 target_offset;
	cmr_u32 privated_data;
	cmr_u32 abl_weighting;
	cmr_s32 evd_value;
};

struct ae_lib_calc_out_3_x  {
	cmr_u32 frame_id;
	cmr_u32 stable;
	cmr_u32 face_stable;
	cmr_u32 face_enable;
	cmr_u32 near_stable;
	cmr_s32 cur_bv;
	cmr_s32 cur_bv_nonmatch;
	cmr_u16 cur_lum;			/*the lum of image:0 ~255 */
	cmr_u16 cur_lum_avg;	/*the lum without weight of image:0 ~255*/
	cmr_u16 target_lum;
	cmr_u16 stab_zone_in;
	cmr_u16 stab_zone_out;
	cmr_u32 flash_status;
	cmr_s8 touch_status;
	float cur_fps;				/*current fps:1~120 */
	cmr_u16 abl_confidence;
	cmr_s32 evd_value;
	struct ae_ev_setting_param_s ev_setting;
	struct ae_rgbgamma_curve_s gamma_curve;/*will be used in future*/
	struct ae_ygamma_curve_s ygamma_curve;/*will be used in future*/
	/*AEM ROI setting*/
	struct ae_point_type_s aem_roi_st;
	struct ae_size_s aem_blk_size;
	/*Bayer Hist ROI setting*/
	//struct ae_rect adjust_hist_roi;
	/*APEX parameters*/
	float bv;
	float av;
	float tv;
	float sv;
	/*mlog information(LCD display)*/
	void *log_buf;
	cmr_u32 log_len;
	/*debug information(JPG Exif)*/
	void *debug_info;
	cmr_u32 debug_len;
	/*privated information*/
	cmr_u32 privated_data;
};

struct ae_callback_param {
	cmr_s32 cur_bv;
	cmr_u32 face_stable;
	cmr_u32 face_num;
	cmr_u32 total_gain;
	cmr_u32 sensor_gain;
	cmr_u32 isp_gain;
	cmr_u32 exp_line;
	cmr_u32 exp_time;
	cmr_u32 ae_stable;
};

struct ae_lib_calc_out_common   {
	struct ae_lib_calc_out_2_x ae_result_2_x;
	struct ae_lib_calc_out_3_x ae_result_3_x;
};

#endif
