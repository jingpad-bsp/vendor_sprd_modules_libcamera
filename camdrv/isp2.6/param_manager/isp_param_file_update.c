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
#define LOG_TAG "isp_para_update"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "isp_type.h"

#include "isp_param_file_update.h"

#define LNC_MAP_NUM	9
#define SCENE_INFO_NUM	10
#define PM_VER_CHIP_MASK	(0xFFFF0000)
#define PM_VER_SW_MASK	(0x0000FFFF)
#define ISP_PARAM_VERSION_V25 0x00090000
#define ISP_PARAM_VERSION_V26 0x000A0000
#define ISP_PARAM_VERSION_V27 0x000B0000

#ifdef CONFIG_ISP_2_5   /* for SharkL3 */
char nr_param_name[ISP_BLK_NR_MAX][20] = {
	"bayer_nr",
	"vst",
	"ivst",
	"rgb_dither",
	"bpc",
	"grgb",
	"cfai",
	"rgb_afm",
	"cce_uvdiv",
	"pre_3dnr",
	"cap_3dnr",
	"yuv_precdn",
	"uv_cdn",
	"uv_postcdn",
	"ynr",
	"ee",
	"iircnr",
	"yuv_noisefilter",
	"cnr",
	"ynrs",
	"cnr3",
	"mfnr",
};
#elif defined CONFIG_ISP_2_6 /* for SharkL5 */
char nr_param_name[ISP_BLK_NR_MAX][20] = {
	"bayer_nr",
	"vst",
	"ivst",
	"rgb_dither",
	"bpc",
	"grgb",
	"cfai",
	"rgb_afm",
	"cce_uvdiv",
	"3dnr",
	"ppe",
	"yuv_precdn",
	"uv_cdn",
	"uv_postcdn",
	"ynr",
	"ee",
	"iircnr",
	"yuv_noisefilter",
	"cnr",
	"imbalance",
	"ltm",
	"sw3dnr",
	"ynrs",
	"cnr3",
	"mfnr",
};
#elif defined CONFIG_ISP_2_7 /* for SharkL5Pro */
char nr_param_name[ISP_BLK_NR_MAX][20] = {
	/* todo */
	"bayer_nr",
	"vst",
	"ivst",
	"rgb_dither",
	"bpc",
	"grgb",
	"cfai",
	"rgb_afm",
	"cce_uvdiv",
	"3dnr",
	"ppe",
	"yuv_precdn",
	"uv_cdn",
	"uv_postcdn",
	"ynr",
	"ee",
	"iircnr",
	"yuv_noisefilter",
	"cnr",
	"imbalance",
	"sw3dnr",
	"bwu_bwd",
	"ynrs",
	"cnr3",
	"mfnr",
	"post_ee",
};
#endif


char nr_mode_name[MAX_MODE_NUM][12] = {
	"common",
	"prv_0",
	"prv_1",
	"prv_2",
	"prv_3",
	"cap_0",
	"cap_1",
	"cap_2",
	"cap_3",
	"video_0",
	"video_1",
	"video_2",
	"video_3",
	"fdr_cap0",
	"fdr_cap1",
	"fdr_cap2",
};

char nr_scene_name[MAX_SCENEMODE_NUM][12] = {
	"normal",
	"night",
	"sport",
	"portrait",
	"landscape",
	"panorama",
	"hdr",
};

cmr_s32 read_nr_level_number_info(FILE * fp, cmr_u8 * data_ptr)
{
	cmr_s32 rtn = 0x00;

	char line_buff[256] = { 0 };
	cmr_s32 i = 0;
	cmr_u8 *param_buf = data_ptr;

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;
		if (fgets(line_buff, 256, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "}};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u8) c[i];
		}
	}
	return rtn;
}

cmr_s32 read_nr_scene_map_info(FILE * fp, cmr_u32 * data_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 i = 0;
	char line_buf[256] = { 0 };
	cmr_u32 *param_buf = data_ptr;
	while (!feof(fp)) {
		cmr_u32 c1[16];
		cmr_s32 n = 0;

		if (fgets(line_buf, 256, fp) == NULL) {
			break;
		}

		if (strstr(line_buf, "{") != NULL) {
			continue;
		}

		if (strstr(line_buf, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buf, "}};") != NULL) {
			break;
		}

		n = sscanf(line_buf, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c1[0], &c1[1], &c1[2], &c1[3], &c1[4], &c1[5], &c1[6], &c1[7], &c1[8], &c1[9],
				&c1[10], &c1[11], &c1[12], &c1[13], &c1[14], &c1[15]);

		for (i = 0; i < n; i++) {
			*param_buf++ = c1[i];
		}
	}

	return rtn;
}

cmr_s32 read_note_name(FILE * fp, cmr_u8 * data_ptr, cmr_u32 * data_len)
{
	cmr_s32 rtn = 0x00;

	cmr_u8 *param_buf = data_ptr;
	char line_buff[256];
	cmr_s32 i;

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;
		if (fgets(line_buff, 256, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u8) c[i];
		}
	}
	*data_len = (cmr_int) param_buf - (cmr_int) (data_ptr);
	return rtn;
}

cmr_s32 read_tune_info(FILE * fp, cmr_u8 * data_ptr, cmr_u32 * data_len)
{
	cmr_s32 rtn = 0x00;

	cmr_u8 *param_buf = data_ptr;
	char line_buff[256];
	cmr_s32 i;

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;
		if (fgets(line_buff, 256, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u8) c[i];
		}
	}
	*data_len = (cmr_int) param_buf - (cmr_int) (data_ptr);
	return rtn;
}

cmr_s32 read_ae_table_32bit(FILE * fp, cmr_u32 * data_ptr, cmr_u32 * data_len)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 i;
	cmr_u32 *param_buf = data_ptr;
	char line_buf[512];

	while (!feof(fp)) {
		cmr_u32 c1[16];
		cmr_s32 n = 0;

		if (fgets(line_buf, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buf, "{") != NULL) {
			continue;
		}

		if (strstr(line_buf, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buf, "},") != NULL) {
			break;
		}

		n = sscanf(line_buf, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c1[0], &c1[1], &c1[2], &c1[3], &c1[4], &c1[5], &c1[6], &c1[7], &c1[8], &c1[9],
				&c1[10], &c1[11], &c1[12], &c1[13], &c1[14], &c1[15]);

		for (i = 0; i < n; i++) {
			*param_buf++ = c1[i];
		}
	}

	*data_len = (cmr_int) param_buf - (cmr_int) data_ptr;

	return rtn;
}

cmr_s32 read_ae_table_16bit(FILE * fp, cmr_u16 * data_ptr, cmr_u32 * data_len)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 i;
	cmr_u16 *param_buf = data_ptr;
	char line_buf[512];

	while (!feof(fp)) {
		cmr_u32 c2[16];
		cmr_s32 n = 0;

		if (fgets(line_buf, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buf, "{") != NULL) {
			continue;
		}

		if (strstr(line_buf, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buf, "},") != NULL) {
			break;
		}

		n = sscanf(line_buf, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c2[0], &c2[1], &c2[2], &c2[3], &c2[4], &c2[5], &c2[6], &c2[7], &c2[8], &c2[9],
				&c2[10], &c2[11], &c2[12], &c2[13], &c2[14], &c2[15]);

		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c2[i];
		}
	}

	*data_len = (cmr_int) param_buf - (cmr_int) data_ptr;

	return rtn;
}

cmr_s32 read_ae_weight(FILE * fp, struct ae_weight_tab * weight_ptr)
{
	cmr_u8 *param_buf = weight_ptr->weight_table;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}
		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}
		if (strstr(line_buff, "},") != NULL) {
			break;
		}
		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u8) c[i];
		}
	}
	weight_ptr->len = (cmr_int) param_buf - (cmr_int) (weight_ptr->weight_table);

	return 0;
}

#ifdef CONFIG_ISP_2_7
cmr_s32 read_ae_scene_info(FILE * fp, struct sensor_ae_tab * ae_ptr, cmr_s32 scene_mode)
{
	cmr_s32 rtn = 0x00;

	cmr_u16 *param_buf = ae_ptr->scene_info[scene_mode][0].scene_info;
	cmr_u16 *param_buf1 = ae_ptr->scene_info[scene_mode][1].scene_info;
	cmr_s32 i = 0;
	char line_buff[512] = { 0 };

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}
		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}
		if (strstr(line_buff, "},") != NULL) {
			break;
		}
		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);

		for (i = 0; i < n; i++) {
			*param_buf++ = c[i];
			*param_buf1++ = c[i];
		}
	}
	ae_ptr->scene_info[scene_mode][0].scene_info_len =
		(cmr_int) param_buf - (cmr_int) (ae_ptr->scene_info[scene_mode][0].scene_info);
	ae_ptr->scene_info[scene_mode][1].scene_info_len =
		(cmr_int) param_buf - (cmr_int) (ae_ptr->scene_info[scene_mode][1].scene_info);

	return rtn;
}

cmr_s32 read_ae_reserve(FILE * fp, struct ae_reserve * ae_reserve)
{
	cmr_u32 *param_buf = ae_reserve->ae_reserve;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	ae_reserve->len = (cmr_int) param_buf - (cmr_int) (ae_reserve->ae_reserve);

	return 0;
}

cmr_s32 read_fix_ae_info(FILE * fp, struct sensor_ae_tab * ae_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 flag_end = 0;
	cmr_s32 i, j;
	char *ae_tab_info[3] = { NULL };
	char *ae_scene_info[6] = { NULL };
	char ae_weight_info[64];

	char *line_buf = (char *)malloc(512 * sizeof(char));
	if (NULL == line_buf) {
		ISP_LOGE("fail to malloc mem!");
		rtn = 0x01;
		return rtn;
	}

	for (i = 0; i < 3; i++) {
		ae_tab_info[i] = (char *)malloc(64 * sizeof(char));
		if (NULL == ae_tab_info[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}

	for (i = 0; i < 4; i++) {
		ae_scene_info[i] = (char *)malloc(64 * sizeof(char));
		if (NULL == ae_scene_info[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}
	while (!feof(fp)) {
		if (NULL == fgets(line_buf, 512, fp)) {
			break;
		}
		if (strstr(line_buf, "_common_ae_tab_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				cmr_s32 break_flag = 0;
				for (j = 0; j < AEC_ISO_NUM; j++) {
					sprintf(ae_tab_info[0], "_common_ae_tab_index_%d%d", i, j);
					sprintf(ae_tab_info[1], "_common_ae_tab_gain_%d%d", i, j);
					sprintf(ae_tab_info[2], "_common_ae_tab_exposure_%d%d", i, j);

					if (strstr(line_buf, ae_tab_info[0]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_tab[i][j].index, &ae_ptr->ae_tab[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[1]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_tab[i][j].gain, &ae_ptr->ae_tab[i][j].gain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[2]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_tab[i][j].exposure, &ae_ptr->ae_tab[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}

		if (strstr(line_buf, "_ae_table_cus_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				cmr_s32 break_flag = 0;
				for (j = 0; j < AE_ISO_NUM_NEW; j++) {
					sprintf(ae_tab_info[0], "_ae_table_cus_index_%d%d", i, j);
					sprintf(ae_tab_info[1], "_ae_table_cus_gain_%d%d", i, j);
					sprintf(ae_tab_info[2], "_ae_table_cus_exposure_%d%d", i, j);

					if (strstr(line_buf, ae_tab_info[0]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_table_cus[i][j].index, &ae_ptr->ae_table_cus[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[1]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_table_cus[i][j].gain, &ae_ptr->ae_table_cus[i][j].gain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[2]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_table_cus[i][j].exposure, &ae_ptr->ae_table_cus[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}
		if (strstr(line_buf, "_ae_weight_") != NULL) {
			for (i = 0; i < AE_WEIGHT_TABLE_SZ; i++) {
				sprintf(ae_weight_info, "_ae_weight_%d", i);
				if (strstr(line_buf, ae_weight_info) != NULL) {

					rtn = read_ae_weight(fp, &ae_ptr->weight_tab[i]);
					break;
				}
			}
		}
		if ((strstr(line_buf, "_scene_") != NULL) && (strstr(line_buf, "_ae_") != NULL)) {
			for (i = 0; i < AE_SCENE_NUM; i++) {
				cmr_s32 break_flag = 0;
				sprintf(ae_scene_info[0], "_ae_scene_info_%d", i);
				if (strstr(line_buf, ae_scene_info[0]) != NULL) {

					rtn = read_ae_scene_info(fp, ae_ptr, i);
					if (rtn != 0x00) {
						rtn = 0x01;
						goto exit;
					}
					break;
				}
				for (j = 0; j < AE_FLICKER_NUM; j++) {
					sprintf(ae_scene_info[1], "_scene_ae_tab_index_%d%d", i, j);
					sprintf(ae_scene_info[2], "_scene_ae_tab_gain_%d%d", i, j);
					sprintf(ae_scene_info[3], "_scene_ae_tab_exposure_%d%d", i, j);
					if (strstr(line_buf, ae_scene_info[1]) != NULL) {

						rtn = read_ae_table_16bit(fp, ae_ptr->scene_info[i][j].index, &ae_ptr->scene_info[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[2]) != NULL) {

						rtn = read_ae_table_16bit(fp, ae_ptr->scene_info[i][j].gain, &ae_ptr->scene_info[i][j].gain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[3]) != NULL) {

						rtn = read_ae_table_32bit(fp, ae_ptr->scene_info[i][j].exposure, &ae_ptr->scene_info[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}
		if (strstr(line_buf, "ae_tab_reserved") != NULL) {
					rtn = read_ae_reserve(fp, &ae_ptr->ae_reserve);
					if (0x00 != rtn) {
						goto exit;
					}
						flag_end = 1;
					break;
		}
	}

exit:
	if (NULL != line_buf) {
		free(line_buf);
	}
	for (i = 0; i < 3; i++) {
		if (NULL != ae_tab_info[i]) {
			free(ae_tab_info[i]);
		}
	}
	for (i = 0; i < 4; i++) {
		if (NULL != ae_scene_info[i]) {
			free(ae_scene_info[i]);
		}
	}
	return rtn;
}
#else
cmr_s32 read_ae_scene_info(FILE * fp, struct sensor_ae_tab * ae_ptr, cmr_s32 scene_mode)
{
	cmr_s32 rtn = 0x00;

	cmr_u32 *param_buf = ae_ptr->scene_tab[scene_mode][0].scene_info;
	cmr_u32 *param_buf1 = ae_ptr->scene_tab[scene_mode][1].scene_info;
	cmr_s32 i = 0;
	char line_buff[512] = { 0 };

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}
		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}
		if (strstr(line_buff, "},") != NULL) {
			break;
		}
		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);

		for (i = 0; i < n; i++) {
			*param_buf++ = c[i];
			*param_buf1++ = c[i];
		}
	}
	ae_ptr->scene_tab[scene_mode][0].scene_info_len =
		(cmr_int) param_buf - (cmr_int) (ae_ptr->scene_tab[scene_mode][0].scene_info);
	ae_ptr->scene_tab[scene_mode][1].scene_info_len =
		(cmr_int) param_buf - (cmr_int) (ae_ptr->scene_tab[scene_mode][1].scene_info);

	return rtn;
}

cmr_s32 read_ae_auto_iso(FILE * fp, struct ae_auto_iso_tab_v1 * auto_iso_ptr)
{
	cmr_u16 *param_buf = auto_iso_ptr->auto_iso_tab;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "},") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	auto_iso_ptr->len = (cmr_int) param_buf - (cmr_int) (auto_iso_ptr->auto_iso_tab);

	return 0;
}

cmr_s32 read_fix_ae_info(FILE * fp, struct sensor_ae_tab * ae_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 flag_end = 0;
	cmr_s32 i, j;
	char *ae_tab_info[5] = { NULL };
	char *ae_scene_info[6] = { NULL };
	char ae_auto_iso_info[64];
	char ae_weight_info[64];

	char *line_buf = (char *)malloc(512 * sizeof(char));
	if (NULL == line_buf) {
		ISP_LOGE("fail to malloc mem!");
		rtn = 0x01;
		return rtn;
	}

	for (i = 0; i < 5; i++) {
		ae_tab_info[i] = (char *)malloc(64 * sizeof(char));
		if (NULL == ae_tab_info[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}

	for (i = 0; i < 6; i++) {
		ae_scene_info[i] = (char *)malloc(64 * sizeof(char));
		if (NULL == ae_scene_info[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}
	while (!feof(fp)) {
		if (NULL == fgets(line_buf, 512, fp)) {
			break;
		}
		if (strstr(line_buf, "_common_ae_tab_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				cmr_s32 break_flag = 0;
				for (j = 0; j < AE_ISO_NUM_NEW; j++) {
					sprintf(ae_tab_info[0], "_common_ae_tab_index_%d%d", i, j);
					sprintf(ae_tab_info[1], "_common_ae_tab_exposure_%d%d", i, j);
					sprintf(ae_tab_info[2], "_common_ae_tab_dummy_%d%d", i, j);
					sprintf(ae_tab_info[3], "_common_ae_tab_again_%d%d", i, j);
					sprintf(ae_tab_info[4], "_common_ae_tab_dgain_%d%d", i, j);

					if (strstr(line_buf, ae_tab_info[0]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_tab[i][j].index, &ae_ptr->ae_tab[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[1]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_tab[i][j].exposure, &ae_ptr->ae_tab[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[2]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_tab[i][j].dummy, &ae_ptr->ae_tab[i][j].dummy_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[3]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_tab[i][j].again, &ae_ptr->ae_tab[i][j].again_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[4]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_tab[i][j].dgain, &ae_ptr->ae_tab[i][j].dgain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}

		if (strstr(line_buf, "_ae_flash_tab_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				cmr_s32 break_flag = 0;
				for (j = 0; j < AE_ISO_NUM_NEW; j++) {
					sprintf(ae_tab_info[0], "_ae_flash_tab_index_%d%d", i, j);
					sprintf(ae_tab_info[1], "_ae_flash_tab_exposure_%d%d", i, j);
					sprintf(ae_tab_info[2], "_ae_flash_tab_dummy_%d%d", i, j);
					sprintf(ae_tab_info[3], "_ae_flash_tab_again_%d%d", i, j);
					sprintf(ae_tab_info[4], "_ae_flash_tab_dgain_%d%d", i, j);

					if (strstr(line_buf, ae_tab_info[0]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_flash_tab[i][j].index, &ae_ptr->ae_flash_tab[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[1]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_flash_tab[i][j].exposure, &ae_ptr->ae_flash_tab[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[2]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_flash_tab[i][j].dummy, &ae_ptr->ae_flash_tab[i][j].dummy_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[3]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_flash_tab[i][j].again, &ae_ptr->ae_flash_tab[i][j].again_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[4]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_flash_tab[i][j].dgain, &ae_ptr->ae_flash_tab[i][j].dgain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}
		if (strstr(line_buf, "_ae_weight_") != NULL) {
			for (i = 0; i < AE_WEIGHT_TABLE_NUM; i++) {
				sprintf(ae_weight_info, "_ae_weight_%d", i);
				if (strstr(line_buf, ae_weight_info) != NULL) {

					rtn = read_ae_weight(fp, &ae_ptr->weight_tab[i]);
					break;
				}
			}
		}
		if ((strstr(line_buf, "_scene_") != NULL) && (strstr(line_buf, "_ae_") != NULL)) {
			for (i = 0; i < AE_SCENE_NUM; i++) {
				cmr_s32 break_flag = 0;
				sprintf(ae_scene_info[0], "_ae_scene_info_%d", i);
				if (strstr(line_buf, ae_scene_info[0]) != NULL) {

					rtn = read_ae_scene_info(fp, ae_ptr, i);
					if (rtn != 0x00) {
						rtn = 0x01;
						goto exit;
					}
					break;
				}
				for (j = 0; j < AE_FLICKER_NUM; j++) {
					sprintf(ae_scene_info[1], "_scene_ae_tab_index_%d%d", i, j);
					sprintf(ae_scene_info[2], "_scene_ae_tab_exposure_%d%d", i, j);
					sprintf(ae_scene_info[3], "_scene_ae_tab_dummy_%d%d", i, j);
					sprintf(ae_scene_info[4], "_scene_ae_tab_again_%d%d", i, j);
					sprintf(ae_scene_info[5], "_scene_ae_tab_dgain_%d%d", i, j);
					if (strstr(line_buf, ae_scene_info[1]) != NULL) {

						rtn = read_ae_table_32bit(fp, ae_ptr->scene_tab[i][j].index, &ae_ptr->scene_tab[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[2]) != NULL) {

						rtn = read_ae_table_32bit(fp, ae_ptr->scene_tab[i][j].exposure, &ae_ptr->scene_tab[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[3]) != NULL) {

						rtn = read_ae_table_32bit(fp, ae_ptr->scene_tab[i][j].dummy, &ae_ptr->scene_tab[i][j].dummy_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[4]) != NULL) {

						rtn = read_ae_table_16bit(fp, ae_ptr->scene_tab[i][j].again, &ae_ptr->scene_tab[i][j].again_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[5]) != NULL) {

						rtn = read_ae_table_16bit(fp, ae_ptr->scene_tab[i][j].dgain, &ae_ptr->scene_tab[i][j].dgain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}
		if (strstr(line_buf, "_ae_auto_iso_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				sprintf(ae_auto_iso_info, "_ae_auto_iso_%d", i);
				if (strstr(line_buf, ae_auto_iso_info) != NULL) {

					rtn = read_ae_auto_iso(fp, &ae_ptr->auto_iso_tab[i]);
					if (0x00 != rtn) {
						goto exit;
					}
					if (1 == i)
						flag_end = 1;
					break;
				}
			}
		}
		if (0 != flag_end)
			break;
	}

exit:
	if (NULL != line_buf) {
		free(line_buf);
	}
	for (i = 0; i < 5; i++) {
		if (NULL != ae_tab_info[i]) {
			free(ae_tab_info[i]);
		}
	}
	for (i = 0; i < 6; i++) {
		if (NULL != ae_scene_info[i]) {
			free(ae_scene_info[i]);
		}
	}
	return rtn;
}

cmr_s32 read_ae3_scene_info(FILE * fp, struct sensor_ae_tab_3_x * ae_ptr, cmr_s32 scene_mode)
{
	cmr_s32 rtn = 0x00;

	cmr_u16 *param_buf = ae_ptr->scene_info[scene_mode][0].scene_info;
	cmr_u16 *param_buf1 = ae_ptr->scene_info[scene_mode][1].scene_info;
	cmr_s32 i = 0;
	char line_buff[512] = { 0 };

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}
		if (strstr(line_buff, "{") != NULL) {
			continue;
		}
		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}
		if (strstr(line_buff, "},") != NULL) {
			break;
		}
		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);

		for (i = 0; i < n; i++) {
			*param_buf++ = c[i];
			*param_buf1++ = c[i];
		}
	}
	ae_ptr->scene_info[scene_mode][0].scene_info_len =
		(cmr_int) param_buf - (cmr_int) (ae_ptr->scene_info[scene_mode][0].scene_info);
	ae_ptr->scene_info[scene_mode][1].scene_info_len =
		(cmr_int) param_buf - (cmr_int) (ae_ptr->scene_info[scene_mode][1].scene_info);

	return rtn;
}

cmr_s32 read_ae3_reserve(FILE * fp, struct ae_reserve * ae_reserve)
{
	cmr_u32 *param_buf = ae_reserve->ae_reserve;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	ae_reserve->len = (cmr_int) param_buf - (cmr_int) (ae_reserve->ae_reserve);

	return 0;
}

cmr_s32 read_fix_ae3_info(FILE * fp, struct sensor_ae_tab_3_x * ae_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 flag_end = 0;
	cmr_s32 i, j;
	char *ae_tab_info[3] = { NULL };
	char *ae_scene_info[6] = { NULL };
	char ae_weight_info[64];

	char *line_buf = (char *)malloc(512 * sizeof(char));
	if (NULL == line_buf) {
		ISP_LOGE("fail to malloc mem!");
		rtn = 0x01;
		return rtn;
	}

	for (i = 0; i < 3; i++) {
		ae_tab_info[i] = (char *)malloc(64 * sizeof(char));
		if (NULL == ae_tab_info[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}

	for (i = 0; i < 4; i++) {
		ae_scene_info[i] = (char *)malloc(64 * sizeof(char));
		if (NULL == ae_scene_info[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}
	while (!feof(fp)) {
		if (NULL == fgets(line_buf, 512, fp)) {
			break;
		}
		if (strstr(line_buf, "_common_ae_tab_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				cmr_s32 break_flag = 0;
				for (j = 0; j < AEC_ISO_NUM; j++) {
					sprintf(ae_tab_info[0], "_common_ae_tab_index_%d%d", i, j);
					sprintf(ae_tab_info[1], "_common_ae_tab_gain_%d%d", i, j);
					sprintf(ae_tab_info[2], "_common_ae_tab_exposure_%d%d", i, j);

					if (strstr(line_buf, ae_tab_info[0]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_tab[i][j].index, &ae_ptr->ae_tab[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[1]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_tab[i][j].gain, &ae_ptr->ae_tab[i][j].gain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[2]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_tab[i][j].exposure, &ae_ptr->ae_tab[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}

		if (strstr(line_buf, "_ae_table_cus_") != NULL) {
			for (i = 0; i < AE_FLICKER_NUM; i++) {
				cmr_s32 break_flag = 0;
				for (j = 0; j < AE_ISO_NUM_NEW; j++) {
					sprintf(ae_tab_info[0], "_ae_table_cus_index_%d%d", i, j);
					sprintf(ae_tab_info[1], "_ae_table_cus_gain_%d%d", i, j);
					sprintf(ae_tab_info[2], "_ae_table_cus_exposure_%d%d", i, j);

					if (strstr(line_buf, ae_tab_info[0]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_table_cus[i][j].index, &ae_ptr->ae_table_cus[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[1]) != NULL) {
						rtn = read_ae_table_16bit(fp, ae_ptr->ae_table_cus[i][j].gain, &ae_ptr->ae_table_cus[i][j].gain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_tab_info[2]) != NULL) {
						rtn = read_ae_table_32bit(fp, ae_ptr->ae_table_cus[i][j].exposure, &ae_ptr->ae_table_cus[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}
		if (strstr(line_buf, "_ae_weight_") != NULL) {
			for (i = 0; i < AE_WEIGHT_TABLE_SZ; i++) {
				sprintf(ae_weight_info, "_ae_weight_%d", i);
				if (strstr(line_buf, ae_weight_info) != NULL) {

					rtn = read_ae_weight(fp, &ae_ptr->weight_tab[i]);
					break;
				}
			}
		}
		if ((strstr(line_buf, "_scene_") != NULL) && (strstr(line_buf, "_ae_") != NULL)) {
			for (i = 0; i < AE_SCENE_NUM; i++) {
				cmr_s32 break_flag = 0;
				sprintf(ae_scene_info[0], "_ae_scene_info_%d", i);
				if (strstr(line_buf, ae_scene_info[0]) != NULL) {

					rtn = read_ae3_scene_info(fp, ae_ptr, i);
					if (rtn != 0x00) {
						rtn = 0x01;
						goto exit;
					}
					break;
				}
				for (j = 0; j < AE_FLICKER_NUM; j++) {
					sprintf(ae_scene_info[1], "_scene_ae_tab_index_%d%d", i, j);
					sprintf(ae_scene_info[2], "_scene_ae_tab_gain_%d%d", i, j);
					sprintf(ae_scene_info[3], "_scene_ae_tab_exposure_%d%d", i, j);
					if (strstr(line_buf, ae_scene_info[1]) != NULL) {

						rtn = read_ae_table_16bit(fp, ae_ptr->scene_info[i][j].index, &ae_ptr->scene_info[i][j].index_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[2]) != NULL) {

						rtn = read_ae_table_16bit(fp, ae_ptr->scene_info[i][j].gain, &ae_ptr->scene_info[i][j].gain_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
					if (strstr(line_buf, ae_scene_info[3]) != NULL) {

						rtn = read_ae_table_32bit(fp, ae_ptr->scene_info[i][j].exposure, &ae_ptr->scene_info[i][j].exposure_len);
						if (0x00 != rtn) {
							goto exit;
						}
						break_flag = 1;
						break;
					}
				}
				if (0 != break_flag)
					break;
			}
		}
		if (strstr(line_buf, "ae_tab_reserved") != NULL) {
					rtn = read_ae3_reserve(fp, &ae_ptr->ae_reserve);
					if (0x00 != rtn) {
						goto exit;
					}
						flag_end = 1;
					break;
		}
	}

exit:
	if (NULL != line_buf) {
		free(line_buf);
	}
	for (i = 0; i < 3; i++) {
		if (NULL != ae_tab_info[i]) {
			free(ae_tab_info[i]);
		}
	}
	for (i = 0; i < 4; i++) {
		if (NULL != ae_scene_info[i]) {
			free(ae_scene_info[i]);
		}
	}
	return rtn;
}
#endif

cmr_s32 read_awb_win_map(FILE * fp, struct sensor_awb_map * awb_map_ptr)
{
	cmr_u16 *param_buf = awb_map_ptr->addr;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "},") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	awb_map_ptr->len = (cmr_int) param_buf - (cmr_int) (awb_map_ptr->addr);

	return 0;
}

cmr_s32 read_awb_pos_weight(FILE * fp, struct sensor_awb_weight * awb_weight_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_u8 *param_buf = awb_weight_ptr->addr;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "},") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u8) c[i];
		}
	}

	awb_weight_ptr->weight_len = (cmr_int) param_buf - (cmr_int) (awb_weight_ptr->addr);

	return rtn;
}

cmr_s32 read_awb_width_height(FILE * fp, struct sensor_awb_weight * awb_weight_ptr)
{
	cmr_u16 *param_buf = awb_weight_ptr->size;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "},") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	awb_weight_ptr->size_param_len = (cmr_int) param_buf - (cmr_int) (awb_weight_ptr->size);

	return 0;
}

cmr_s32 read_fix_awb_info(FILE * fp, struct sensor_awb_map_weight_param * awb_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_s32 flag_end = 0;
	char line_buf[512];

	while (!feof(fp)) {
		if (NULL == fgets(line_buf, 512, fp)) {
			break;
		}
		if (strstr(line_buf, "_awb_win_map") != NULL) {
			rtn = read_awb_win_map(fp, &awb_ptr->awb_map);
			if (0x00 != rtn) {
				return rtn;
			}
		}
		if ((strstr(line_buf, "_awb_pos_weight") != NULL) && (strstr(line_buf, "height") != NULL)) {
			rtn = read_awb_pos_weight(fp, &awb_ptr->awb_weight);
			if (0x00 != rtn) {
				return rtn;
			}
		}
		if (strstr(line_buf, "_awb_pos_weight_width_height") != NULL) {
			rtn = read_awb_width_height(fp, &awb_ptr->awb_weight);
			if (0x00 != rtn) {
				return rtn;
			}
			flag_end = 1;
		}
		if (0 != flag_end)
			break;
	}

	return rtn;
}

cmr_s32 read_lnc_map_info(FILE * fp, struct sensor_lens_map * lnc_map_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_u32 *param_buf = lnc_map_ptr->map_info;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "},") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = c[i];
		}
	}

	lnc_map_ptr->map_info_len = (cmr_int) param_buf - (cmr_int) (lnc_map_ptr->map_info);

	return rtn;
}

cmr_s32 read_lnc_weight_info(FILE * fp, struct sensor_lens_map * lnc_map_ptr)
{
	cmr_s32 rtn = 0x00;
	cmr_s32 i;
	char line_buff[512];
	cmr_u16 *param_buf = PNULL;
	param_buf = lnc_map_ptr->weight_info;
	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "},") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	lnc_map_ptr->weight_info_len = (cmr_int) param_buf - (cmr_int) (lnc_map_ptr->weight_info);

	return rtn;
}

cmr_s32 read_lnc_info(FILE * fp, struct sensor_lens_map * lnc_map_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_u16 *param_buf = lnc_map_ptr->lnc_addr;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			if (strstr(line_buff, "_lnc") != NULL) {
				break;
			} else {
				continue;
			}
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = (cmr_u16) c[i];
		}
	}

	lnc_map_ptr->lnc_len = (cmr_int) param_buf - (cmr_int) (lnc_map_ptr->lnc_addr);
	return rtn;
}

cmr_s32 read_fix_lnc_info(FILE * fp, struct sensor_lsc_map * lnc_ptr)
{
	cmr_s32 rtn = 0x00;
	cmr_s32 i;

	char lnc_map_info[50];
	char lnc_weight_info[50];
	char lnc_info[20];

	char *line_buf = (char *)malloc(512 * sizeof(char));
	if (NULL == line_buf) {
		ISP_LOGE("fail to malloc mem!");
		rtn = 0x01;
		return rtn;
	}
	while (!feof(fp)) {
		if (NULL == fgets(line_buf, 512, fp)) {
			break;
		}
		if (NULL != strstr(line_buf, "_lnc_map_info")) {
			for (i = 0; i < LNC_MAP_NUM; i++) {
				sprintf(lnc_map_info, "_lnc_map_info_0%d", i);
				if (strstr(line_buf, lnc_map_info) != NULL) {
					rtn = read_lnc_map_info(fp, &lnc_ptr->map[i]);
					if (0x00 != rtn) {
						return rtn;
					}
					break;
				}
			}
		}
		if (NULL != strstr(line_buf, "_lnc_weight_0")) {
			for (i = 0; i < LNC_MAP_NUM; i++) {
				sprintf(lnc_weight_info, "_lnc_weight_0%d", i);
				if (strstr(line_buf, lnc_weight_info) != NULL) {
					rtn = read_lnc_weight_info(fp, &lnc_ptr->map[i]);
					if (0x00 != rtn) {
						return rtn;
					}
					break;
				}
			}
		}
		if (NULL != strstr(line_buf, "_lnc_0")) {
			for (i = 0; i < LNC_MAP_NUM; i++) {
				sprintf(lnc_info, "_lnc_0%d", i);
				rtn = read_lnc_info(fp, &lnc_ptr->map[i]);
				if (0x00 != rtn) {
					return rtn;
				}
			}
			break;
		}
	}

	if (NULL != line_buf) {
		free(line_buf);
		line_buf = NULL;
	}
	return rtn;
}

cmr_s32 read_lnc_map(FILE * fp, struct sensor_lens_map * lnc_map_ptr)
{
	cmr_s32 rtn = 0x00;

	cmr_u32 *param_buf = lnc_map_ptr->map_info;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = c[i];
		}
	}

	lnc_map_ptr->map_info_len = (cmr_int) param_buf - (cmr_int) (lnc_map_ptr->map_info);

	return rtn;
}

cmr_s32 read_libuse_info(FILE * fp, struct sensor_raw_info * sensor_raw_ptr)
{
	cmr_u32 *param_buf = (cmr_u32 *) sensor_raw_ptr->libuse_info;
	cmr_s32 i;
	char line_buff[512];

	while (!feof(fp)) {
		cmr_u32 c[16];
		cmr_s32 n = 0;

		if (fgets(line_buff, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buff, "{") != NULL) {
			continue;
		}

		if (strstr(line_buff, "/*") != NULL) {
			continue;
		}

		if (strstr(line_buff, "};") != NULL) {
			break;
		}

		n = sscanf(line_buff, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10],
				&c[11], &c[12], &c[13], &c[14], &c[15]);
		for (i = 0; i < n; i++) {
			*param_buf++ = c[i];
		}
	}
	return 0;
}

cmr_s32 read_nr_param(struct sensor_raw_info * sensor_raw_ptr, const char *sensor_name, char *param_path)
{
	cmr_s32 rtn = ISP_SUCCESS;
	cmr_u32 i = 0, j = 0, k = 0;
	cmr_u32 nr_set_size[ISP_BLK_NR_MAX] = {0};
	struct sensor_nr_scene_map_param *nr_map_ptr = PNULL;
	cmr_u32 *multi_nr_scene_map_ptr = PNULL;
	struct sensor_nr_level_map_param *nr_level_number_ptr = PNULL;
	struct sensor_raw_fix_info *fix_data_ptr = PNULL;
	struct nr_set_group_unit *nr_ptr = PNULL;
	cmr_u8 *nr_param_ptr = PNULL;
	cmr_u32 size_of_per_unit = 0;
	FILE *fp = NULL;
	char filename[256];

	if (PNULL == sensor_raw_ptr) {
		ISP_LOGE("fail to get valid param : sensor_raw_ptr = %p", sensor_raw_ptr);
		rtn = ISP_ERROR;
		return rtn;
	}

	for (i = 0; i < ISP_BLK_NR_MAX; i++) {
		cmr_u32 nr_type;
		cmr_u32 blk_id;
		nr_type = nr_blocks_info[i].nr_type;
		blk_id = nr_blocks_info[i].blk_id;
		if (nr_type < ISP_BLK_NR_MAX && blk_id)
			nr_set_size[nr_type] = nr_blocks_info[i].unit_size;
	}

	nr_map_ptr = sensor_raw_ptr->nr_fix.nr_scene_ptr;
	multi_nr_scene_map_ptr = (cmr_u32 *)&(nr_map_ptr->nr_scene_map[0]);
	nr_level_number_ptr = sensor_raw_ptr->nr_fix.nr_level_number_ptr;

	if ((PNULL == multi_nr_scene_map_ptr) || (PNULL == nr_level_number_ptr)) {
		ISP_LOGE("fail to get valid param : multi_nr_scene_map_ptr = %p, nr_level_number_ptr = %p",
			multi_nr_scene_map_ptr, nr_level_number_ptr);
		rtn = ISP_ERROR;
		return rtn;
	}

	fix_data_ptr = sensor_raw_ptr->fix_ptr[0];
	nr_ptr = (struct nr_set_group_unit *)&(fix_data_ptr->nr.nr_set_group);

	for (k = 0; k < ISP_BLK_NR_MAX; k++) {
		size_of_per_unit = nr_set_size[k] * nr_level_number_ptr->nr_level_map[k];
		nr_param_ptr = nr_ptr[k].nr_ptr;
		ISP_LOGD("check NR block %s\n",  nr_param_name[k]);

		for (i = 0; i < MAX_MODE_NUM; i++) {
			if (multi_nr_scene_map_ptr[i] == 0)
				continue;
			if (PNULL == nr_param_ptr) {
				ISP_LOGD("no param for mode %d, nr %d", i, k);
				continue;
			}
			for (j = 0; j < MAX_SCENEMODE_NUM; j++) {
				if ((multi_nr_scene_map_ptr[i] >> j) & 0x01) {
					sprintf(filename, "%s%s_%s_%s_%s_param.bin", param_path, sensor_name,
						nr_mode_name[i], nr_scene_name[j], nr_param_name[k]);
					if (0 != access(filename, R_OK)) {
						ISP_LOGI("no such file : %s",filename);
					} else {
						if (NULL != (fp = fopen(filename, "rb"))) {
							ISP_LOGD("OKOK to fopen %s",filename);
							rtn = fread((void *)nr_param_ptr, 1, size_of_per_unit, fp);
							if (rtn < 0) {
								ISP_LOGE("fail to fread %s",filename);
								fclose(fp);
								nr_param_ptr += size_of_per_unit;
								continue;
							}
							rtn = 0;
							fclose(fp);
						} else {
							ISP_LOGE("fail to fopen %s",filename);
						}
					}
					nr_param_ptr += size_of_per_unit;
				}
			}
		}
	}

	return rtn;
}

cmr_s32 update_params(struct sensor_raw_info * sensor_raw_ptr, const char *sensor_name, char *param_path)
{
	cmr_s32 rtn = 0x00;
	cmr_s32 i = 0;
	cmr_u32 version_id;
	cmr_u32 is_ae3x = 0;

	char tune_info[128];
	char note_name[128];
	char libuse_info[128];
	char ae_tab[128];
	char ae3_tab[128];
	char awb_tab[128];
	char lsc_tab[128];
	char lsc_2d_map_flag1[64];
	char lsc_2d_map_flag2[64];
	char nr_level_number[256];
	char nr_default_level[256];
	char nr_scene_map[256];
	struct sensor_nr_level_map_param *nr_level_number_ptr = PNULL;
	struct sensor_nr_level_map_param *nr_default_level_ptr = PNULL;
	struct sensor_nr_scene_map_param *nr_map_ptr = PNULL;

	char *line_buf = (char *)malloc(512 * sizeof(char));
	char *filename[MAX_MODE_NUM] = { PNULL };
	FILE *fp = PNULL;

	version_id = sensor_raw_ptr->version_info->version_id;
	if (((version_id & PM_VER_CHIP_MASK) < PM_CHIP_VER_V27) &&
			((version_id & PM_VER_SW_MASK) < PM_SW_VER_V27))
		is_ae3x = 0;
	else
		is_ae3x = 1;
	ISP_LOGD("ae version %d\n", is_ae3x);

	if (NULL == line_buf) {
		ISP_LOGE("fail to malloc mem!");
		rtn = 0x01;
		return rtn;
	}

	for (i = 0; i < MAX_MODE_NUM; i++) {
		filename[i] = (char *)malloc(256 * sizeof(char));
		if (NULL == filename[i]) {
			ISP_LOGE("fail to malloc mem!");
			rtn = 0x01;
			goto exit;
		}
	}

	sprintf(filename[0], "%s%s", param_path, "isp_nr.h");
	sprintf(nr_level_number, "static struct sensor_nr_level_map_param s_%s_nr_level_number_map_param", sensor_name);
	sprintf(nr_default_level, "static struct sensor_nr_level_map_param s_%s_default_nr_level_map_param", sensor_name);
	sprintf(nr_scene_map, "static struct sensor_nr_scene_map_param s_%s_nr_scene_map_param", sensor_name);
	sprintf(lsc_2d_map_flag1, "#undef LSC_2D_MAP_0");
	sprintf(lsc_2d_map_flag2, "#undef LSC_2D_MAP_0_OFFSET");
	ISP_LOGD("parse nr file %s\n", filename[0]);

	nr_level_number_ptr = sensor_raw_ptr->nr_fix.nr_level_number_ptr;
	nr_default_level_ptr = sensor_raw_ptr->nr_fix.nr_default_level_ptr;
	nr_map_ptr = sensor_raw_ptr->nr_fix.nr_scene_ptr;
	fp = fopen(filename[0], "r");
	if (NULL == fp) {
		ISP_LOGE("fail to open file %s!\n", filename[0]);
		rtn = 0x01;
		goto exit;
	}
	ISP_LOGD("OKOK to fopen %s",filename[0]);

	while (!feof(fp)) {
		if (fgets(line_buf, 512, fp) == NULL) {
			break;
		}

		if (strstr(line_buf, nr_level_number) != NULL) {
			if (nr_level_number_ptr != NULL) {
				rtn = read_nr_level_number_info(fp, &(nr_level_number_ptr->nr_level_map[0]));
				if (0x00 != rtn) {
					ISP_LOGE("fail to check nr_level_number_info!");
					fclose(fp);
					goto exit;
				}
			}
			continue;
		}
		if (strstr(line_buf, nr_default_level) != NULL) {
			if (nr_default_level_ptr != NULL) {
				rtn = read_nr_level_number_info(fp, &(nr_default_level_ptr->nr_level_map[0]));
				if (0x00 != rtn) {
					ISP_LOGE("fail to check nr_default_level_info!");
					fclose(fp);
					goto exit;
				}
			}
			continue;
		}
		if (strstr(line_buf, nr_scene_map) != NULL) {
			if (nr_default_level_ptr != NULL) {
				rtn = read_nr_scene_map_info(fp, &(nr_map_ptr->nr_scene_map[0]));
				if (0x00 != rtn) {
					ISP_LOGE("fail to check nr_scene_map_info!");
					fclose(fp);
					goto exit;
				}
			}
			break;
		}
	}
	fclose(fp);

	sprintf(libuse_info, "static uint32_t s_%s_libuse_info", sensor_name);

	sprintf(filename[0], "%ssensor_%s_raw_param_main.c", param_path, sensor_name);
	fp = fopen(filename[0], "r");
	if (NULL == fp) {
		ISP_LOGE("fail to open file %s!\n", filename[0]);
		rtn = 0x01;
		goto exit;
	}
	ISP_LOGD("OKOK to fopen %s",filename[0]);

	while (!feof(fp)) {
		if (fgets(line_buf, 512, fp) == NULL) {
			break;
		}
		if (strstr(line_buf, libuse_info) != NULL) {
			rtn = read_libuse_info(fp, sensor_raw_ptr);
			if (0x00 != rtn) {
				ISP_LOGE("fail to check libuse_info!");
				fclose(fp);
				goto exit;
			}
			break;
		}
	}
	fclose(fp);

	for (i = 0; i < MAX_MODE_NUM; i++) {
		sprintf(filename[i], "%ssensor_%s_raw_param_%s.c", param_path, sensor_name, &nr_mode_name[i][0]);
		sprintf(tune_info, "static uint8_t s_%s_tune_info_%s", sensor_name, &nr_mode_name[i][0]);
		sprintf(note_name, "static uint8_t s_%s_%s_tool_ui_input", sensor_name, &nr_mode_name[i][0]);
		sprintf(ae_tab, "static struct ae_table_param_2 s_%s_%s_ae_table_param", sensor_name, &nr_mode_name[i][0]);
		sprintf(ae3_tab, "static struct ae_table_param_3 s_%s_%s_ae_table_param", sensor_name, &nr_mode_name[i][0]);
		sprintf(awb_tab, "static struct sensor_awb_table_param s_%s_%s_awb_table_param", sensor_name, &nr_mode_name[i][0]);
		sprintf(lsc_tab, "static struct sensor_lsc_2d_table_param s_%s_%s_lsc_2d_table_param", sensor_name, &nr_mode_name[i][0]);

		fp = fopen(filename[i], "r");
		if (NULL == fp) {
			ISP_LOGE("fail to open file %s!\n", filename[i]);
			continue;
		} else
			ISP_LOGD("OKOK to open file %s\n", filename[i]);

		while (!feof(fp)) {
			if (fgets(line_buf, 512, fp) == NULL) {
				break;
			}

			if (strstr(line_buf, tune_info) != NULL) {
				rtn = read_tune_info(fp, sensor_raw_ptr->mode_ptr[i].addr, &sensor_raw_ptr->mode_ptr[i].len);
				if (0x00 != rtn) {
					ISP_LOGE("fail to read_tune_info!");
					fclose(fp);
					goto exit;
				}
				continue;
			}
#ifdef CONFIG_ISP_2_7
			if (strstr(line_buf, ae_tab) != NULL) {
				if (sensor_raw_ptr->fix_ptr[i]->ae.ae_param.ae != NULL) {
					rtn = read_fix_ae_info(fp, &sensor_raw_ptr->fix_ptr[i]->ae);
					if (0x00 != rtn) {
						ISP_LOGE("fail to read_ae_info!");
						fclose(fp);
						goto exit;
					}
				}
				continue;
			}
#elif defined(CONFIG_ISP_2_6)
			if (is_ae3x == 0) {
				if (strstr(line_buf, ae_tab) != NULL) {
					if (sensor_raw_ptr->fix_ptr[i]->ae.ae_param.ae != NULL) {
						rtn = read_fix_ae_info(fp, &sensor_raw_ptr->fix_ptr[i]->ae);
						if (0x00 != rtn) {
							ISP_LOGE("fail to read_ae_info!");
							fclose(fp);
							goto exit;
						}
					}
				continue;
				}
			}
			if (is_ae3x == 1) {
				if (strstr(line_buf, ae3_tab) != NULL) {
					if (sensor_raw_ptr->fix_ptr[i]->ae3x.ae_param.ae != NULL) {
						rtn = read_fix_ae3_info(fp, &sensor_raw_ptr->fix_ptr[i]->ae3x);
						if (0x00 != rtn) {
							ISP_LOGE("fail to read_ae_info!");
							fclose(fp);
							goto exit;
						}
					}
					continue;
				}
			}
#else
			if (strstr(line_buf, ae_tab) != NULL) {
				if (sensor_raw_ptr->fix_ptr[i]->ae.ae_param.ae != NULL) {
					rtn = read_fix_ae_info(fp, &sensor_raw_ptr->fix_ptr[i]->ae);
					if (0x00 != rtn) {
						ISP_LOGE("fail to read_ae_info!");
						fclose(fp);
						goto exit;
					}
				}
				continue;
			}
#endif
			if (strstr(line_buf, awb_tab) != NULL) {
				if (sensor_raw_ptr->fix_ptr[i]->awb.awb_param.awb != NULL) {
					rtn = read_fix_awb_info(fp, &sensor_raw_ptr->fix_ptr[i]->awb);
					if (0x00 != rtn) {
						ISP_LOGE("fail to read_awb_info!");
						fclose(fp);
						goto exit;
					}
				}
				continue;
			}
			if (strstr(line_buf, lsc_tab) != NULL) {
				if (sensor_raw_ptr->fix_ptr[i]->lnc.lnc_param.lnc != NULL) {
					rtn = read_fix_lnc_info(fp, &sensor_raw_ptr->fix_ptr[i]->lnc);
					if (0x00 != rtn) {
						ISP_LOGE("fail to read_lnc_info!");
						fclose(fp);
						goto exit;
					}
				}
			}
			if (strstr(line_buf, note_name) != NULL) {
				rtn = read_note_name(fp, sensor_raw_ptr->note_ptr[i].note, &sensor_raw_ptr->note_ptr[i].node_len);
				if (0x00 != rtn) {
					ISP_LOGE("fail to read_tune_info!");
					fclose(fp);
					goto exit;
				}
				continue;
			}
			if (strstr(line_buf, note_name) != NULL) {
				ISP_LOGE("fail to update file _tool_ui_input");
				break;
			}
		}
		fclose(fp);
	}

	rtn = read_nr_param(sensor_raw_ptr, sensor_name, param_path);
	if (0x00 != rtn) {
		ISP_LOGE("fail to read_nr_param!");
		goto exit;
	}

exit:
	if (PNULL != line_buf) {
		free(line_buf);
	}
	for (i = 0; i < MAX_MODE_NUM; i++) {
		if (PNULL != filename[i]) {
			free(filename[i]);
		}
	}
	return rtn;
}

cmr_u32 isp_pm_raw_para_update_from_file(struct sensor_raw_info * raw_info_ptr, char *data_path)
{
	cmr_s32 rtn = 0x00;
	const char *sensor_name = PNULL;
	struct sensor_raw_info *sensor_raw_info_ptr = raw_info_ptr;

	cmr_s32 version = 0;
	char path[256];
	cmr_u32 version_id;
	char *filename = NULL;
	char filename0[256];
	char filename1[256];
	sensor_name = (char *)&sensor_raw_info_ptr->version_info->sensor_ver_name.sensor_name;

	if (data_path && data_path[0] != 0) {
		ISP_LOGD("tuning param data path %s\n", data_path);
		if (strlen(data_path) < sizeof(path)) {
			strcpy(path, data_path);
		} else {
			ISP_LOGE("error: data_path length %s is overrun 256\n", data_path);
			strcpy(path, CAMERA_DUMP_PATH);
		}
		strcpy(path, data_path);
	} else {
		strcpy(path, CAMERA_DUMP_PATH);
	}
	sprintf(filename0, "%ssensor_%s_raw_param.c", path, sensor_name);
	sprintf(filename1, "%ssensor_%s_raw_param_common.c", path, sensor_name);

	if (-1 != access(filename0, 0)) {
		ISP_LOGV("access %s!\n", filename0);
		filename = filename0;
		version = 1;
	}
	if (NULL == filename) {
		if (-1 != access(filename1, 0)) {
			ISP_LOGV("access %s!\n", filename1);
			filename = filename1;
			version = 2;
		}
	}
	if (NULL == filename && 0 == version) {
		ISP_LOGI("there is no param file!\n");
		return rtn;
	} else {
		ISP_LOGI("the param file is %s, version = %d", filename, version);
	}

	version_id = sensor_raw_info_ptr->version_info->version_id;

	if ((version_id & PM_VER_CHIP_MASK) >= ISP_PARAM_VERSION_V25) {
		rtn = update_params(sensor_raw_info_ptr, sensor_name, path);
		if (0x00 != rtn) {
			ISP_LOGE("fail to update param!");
			return rtn;
		}
	}
	return rtn;
}
#if 0
#ifndef WIN32
cmr_u32 isp_raw_para_update_from_file(SENSOR_INFO_T * sensor_info_ptr, SENSOR_ID_E sensor_id)
{
	cmr_s32 rtn = 0x00;
	struct sensor_raw_info *sensor_raw_info_ptr = *(sensor_info_ptr->raw_info_ptr);
	UNUSED(sensor_id);
	isp_pm_raw_para_update_from_file(sensor_raw_info_ptr);

	return rtn;
}
#endif
#endif
