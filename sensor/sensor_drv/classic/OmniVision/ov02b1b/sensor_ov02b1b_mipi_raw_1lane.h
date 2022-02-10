/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * V1.0
 */
 /*History
 *Date                  Modification                                 Reason
 *
 */

#ifndef _SENSOR_ov02b1b_MIPI_RAW_H_
#define _SENSOR_ov02b1b_MIPI_RAW_H_


#include <utils/Log.h>
#include "sensor.h"
#include "jpeg_exif_header.h"
#include "sensor_drv_u.h"
#include "sensor_raw.h"

//#include "parameters/sensor_ov02b1b_raw_param_main.c"

//#define FEATURE_OTP

#define VENDOR_NUM 1
#define SENSOR_NAME				"ov02b1b_mipi_raw"

#define I2C_SLAVE_ADDR			0x78		/* 8bit slave address*/
//#define I2C_DUAL_SLAVE_ADDR     0x7B 

#define ov02b1b_PID_ADDR				0x02
#define ov02b1b_PID_VALUE			0x00
#define ov02b1b_VER_ADDR				0x03
#define ov02b1b_VER_VALUE			0x2b

/* sensor parameters begin */

/* effective sensor output image size */
#define PREVIEW_WIDTH			1600
#define PREVIEW_HEIGHT			1200
#define SNAPSHOT_WIDTH			1600 
#define SNAPSHOT_HEIGHT			1200

/*Raw Trim parameters*/
#define PREVIEW_TRIM_X			0
#define PREVIEW_TRIM_Y			0
#define PREVIEW_TRIM_W			PREVIEW_WIDTH
#define PREVIEW_TRIM_H			PREVIEW_HEIGHT
#define SNAPSHOT_TRIM_X			0
#define SNAPSHOT_TRIM_Y			0
#define SNAPSHOT_TRIM_W			SNAPSHOT_WIDTH
#define SNAPSHOT_TRIM_H			SNAPSHOT_HEIGHT

/*Mipi output*/
#define LANE_NUM			1
#define RAW_BITS			10

#define PREVIEW_MIPI_PER_LANE_BPS	  660  /* 2*Mipi clk */
#define SNAPSHOT_MIPI_PER_LANE_BPS	  660  /* 2*Mipi clk */

/*line time unit: 1ns*/
#define PREVIEW_LINE_TIME		  27150
#define SNAPSHOT_LINE_TIME		  27150

/* frame length*/
#define PREVIEW_FRAME_LENGTH		1221
#define SNAPSHOT_FRAME_LENGTH		1221

/* please ref your spec */
#define FRAME_OFFSET			7
#define SENSOR_MAX_GAIN			0xf8
#define SENSOR_BASE_GAIN		0x10
#define SENSOR_MIN_SHUTTER		4
#define SENSOR_MIN_VBLANKING         1

/* please ref your spec
 * 1 : average binning
 * 2 : sum-average binning
 * 4 : sum binning
 */
#define BINNING_FACTOR			1

/* please ref spec
 * 1: sensor auto caculate
 * 0: driver caculate
 */
/* sensor parameters end */

/* isp parameters, please don't change it*/
#define ISP_BASE_GAIN			0x80

/* please don't change it */
#define EX_MCLK				24

/*==============================================================================
 * Description:
 * register setting
 *============================================================================*/

static const SENSOR_REG_T ov02b1b_init_setting[] = {
	{0xfc, 0x01},
	{0xfd, 0x00},
	{0xfd, 0x00},
	{0x24, 0x02},
	{0x25, 0x06},
	{0x29, 0x03},
	{0x2a, 0x34},
	{0x1e, 0x17},
	{0x33, 0x07},
	{0x35, 0x07},
	{0x4a, 0x0c},
	{0x3a, 0x05},
	{0x3b, 0x02},
	{0x3e, 0x00},
	{0x46, 0x01},
	{0x6d, 0x03},
	{0xfd, 0x01},
	{0x0e, 0x02},
	{0x0f, 0x1a},
	{0x18, 0x00},
	{0x22, 0xff},
	{0x23, 0x02},
	//{0x12, 0x10},
	{0x17, 0x2c},
	{0x19, 0x20},
	{0x1b, 0x06},
	{0x1c, 0x04},
	{0x20, 0x03},
	{0x30, 0x01},
	{0x33, 0x01},
	{0x31, 0x0a},
	{0x32, 0x09},
	{0x38, 0x01},
	{0x39, 0x01},
	{0x3a, 0x01},
	{0x3b, 0x01},
	{0x4f, 0x04},
	{0x4e, 0x05},
	{0x50, 0x01},
	{0x35, 0x0c},
	{0x45, 0x2a},
	{0x46, 0x2a},
	{0x47, 0x2a},
	{0x48, 0x2a},
	{0x4a, 0x2c},
	{0x4b, 0x2c},
	{0x4c, 0x2c},
	{0x4d, 0x2c},
	{0x56, 0x3a},
	{0x57, 0x0a},
	{0x58, 0x24},
	{0x59, 0x20},
	{0x5a, 0x0a},
	{0x5b, 0xff},
	{0x37, 0x0a},
	{0x42, 0x0e},
	{0x68, 0x90},
	{0x69, 0xcd},
	{0x6a, 0x8f},
	{0x7c, 0x0a},
	{0x7d, 0x0a},
	{0x7e, 0x0a},
	{0x7f, 0x08},
	{0x83, 0x14},
	{0x84, 0x14},
	{0x86, 0x14},
	{0x87, 0x07},
	{0x88, 0x0f},
	{0x94, 0x02},
	{0x98, 0xd1},
	{0xfe, 0x02},
	{0xfd, 0x03},
	{0x97, 0x6c},
	{0x98, 0x60},
	{0x99, 0x60},
	{0x9a, 0x6c},
	{0xa1, 0x40},
	{0xaf, 0x04},
	{0xb1, 0x40},
	{0xae, 0x0d},
	{0x88, 0x5b},
	{0x89, 0x7c},
	{0xb4, 0x05},
	{0x8c, 0x40},
	{0x8e, 0x40},
	{0x90, 0x40},
	{0x92, 0x40},
	{0x9b, 0x46},
	{0xac, 0x40},
	{0xfd, 0x00},
	{0x5a, 0x15},
	{0x74, 0x01},
	{0xfd, 0x00},
	{0x50, 0x40},
	{0x52, 0xb0},
	{0xfd, 0x01},
	{0x03, 0x70},
	{0x05, 0x10},
	{0x07, 0x20},
	{0x09, 0xb0},
	{0xfd, 0x03},
	{0xc2, 0x00},
	{0xfb, 0x01},
	{0xfd, 0x01},
};

static const SENSOR_REG_T ov02b1b_snapshot_setting[] = {
};


static struct sensor_res_tab_info s_ov02b1b_resolution_tab_raw[VENDOR_NUM] = {
	{
      .module_id = MODULE_SUNNY,
      .reg_tab = {
        {ADDR_AND_LEN_OF_ARRAY(ov02b1b_init_setting), PNULL, 0,
        .width = 0, .height = 0,
        .xclk_to_sensor = EX_MCLK, .image_format = SENSOR_IMAGE_FORMAT_RAW},

        {ADDR_AND_LEN_OF_ARRAY(ov02b1b_snapshot_setting), PNULL, 0,
        .width = SNAPSHOT_WIDTH, .height = SNAPSHOT_HEIGHT,
        .xclk_to_sensor = EX_MCLK, .image_format = SENSOR_IMAGE_FORMAT_RAW}
		}
	}

	/*If there are multiple modules,please add here*/
};

static SENSOR_TRIM_T s_ov02b1b_resolution_trim_tab[VENDOR_NUM] = {
	{
     .module_id = MODULE_SUNNY,
     .trim_info = {
       {0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}},

	   {
        .trim_start_x = SNAPSHOT_TRIM_X, .trim_start_y = SNAPSHOT_TRIM_Y,
        .trim_width = SNAPSHOT_TRIM_W,   .trim_height = SNAPSHOT_TRIM_H,
        .line_time = SNAPSHOT_LINE_TIME, .bps_per_lane = SNAPSHOT_MIPI_PER_LANE_BPS,
        .frame_line = SNAPSHOT_FRAME_LENGTH,
        .scaler_trim = {.x = SNAPSHOT_TRIM_X, .y = SNAPSHOT_TRIM_Y, .w = SNAPSHOT_TRIM_W, .h = SNAPSHOT_TRIM_H}},
       }
	}

    /*If there are multiple modules,please add here*/

};

static SENSOR_REG_T ov02b1b_shutter_reg[] = {
    {0xfd, 0x01}, 
	{0x0e, 0x07}, 
	{0x0f, 0x85}, 
	{0xfe, 0x02}, 

};

static struct sensor_i2c_reg_tab ov02b1b_shutter_tab = {
    .settings = ov02b1b_shutter_reg, 
	.size = ARRAY_SIZE(ov02b1b_shutter_reg),
};

static SENSOR_REG_T ov02b1b_again_reg[] = {
    {0xfd, 0x01}, 
	{0x22, 0x10}, 
	{0xfe, 0x02}, 
 
};

static struct sensor_i2c_reg_tab ov02b1b_again_tab = {
    .settings = ov02b1b_again_reg, 
	.size = ARRAY_SIZE(ov02b1b_again_reg),
};

static SENSOR_REG_T ov02b1b_dgain_reg[] = {
   
};

static struct sensor_i2c_reg_tab ov02b1b_dgain_tab = {
    .settings = ov02b1b_dgain_reg, 
	.size = ARRAY_SIZE(ov02b1b_dgain_reg),
};

static SENSOR_REG_T ov02b1b_frame_length_reg[] = {
    {0xfd, 0x01},
    {0x14, 0x00},
    {0x15, 0x02},
    {0xfe, 0x02},
};

static struct sensor_i2c_reg_tab ov02b1b_frame_length_tab = {
    .settings = ov02b1b_frame_length_reg,
    .size = ARRAY_SIZE(ov02b1b_frame_length_reg),
};

static struct sensor_aec_i2c_tag ov02b1b_aec_info = {
    .slave_addr = (I2C_SLAVE_ADDR >> 1),
    .addr_bits_type = SENSOR_I2C_REG_8BIT,
    .data_bits_type = SENSOR_I2C_VAL_8BIT,
    .shutter = &ov02b1b_shutter_tab,
    .again = &ov02b1b_again_tab,
    .dgain = &ov02b1b_dgain_tab,
    .frame_length = &ov02b1b_frame_length_tab,
};


static SENSOR_STATIC_INFO_T s_ov02b1b_static_info[VENDOR_NUM] = {
    {.module_id = MODULE_SUNNY,
     .static_info = {
        .f_num = 200,
        .focal_length = 354,
        .max_fps = 0,
        .max_adgain = 15.5,
        .ois_supported = 0,
        .pdaf_supported = 0,
        .exp_valid_frame_num = 1,
        .clamp_level = 64,
        .adgain_valid_frame_num = 1,
        .fov_info = {{4.614f, 3.444f}, 4.222f},
        .mono_sensor = 1
     }
    }
    /*If there are multiple modules,please add here*/
};


static SENSOR_MODE_FPS_INFO_T s_ov02b1b_mode_fps_info[VENDOR_NUM] = {
    {.module_id = MODULE_SUNNY,
       {.is_init = 0,
         {{SENSOR_MODE_COMMON_INIT, 0, 1, 0, 0},
         {SENSOR_MODE_PREVIEW_ONE, 0, 1, 0, 0},
         {SENSOR_MODE_SNAPSHOT_ONE_FIRST, 0, 1, 0, 0},
         {SENSOR_MODE_SNAPSHOT_ONE_SECOND, 0, 1, 0, 0},
         {SENSOR_MODE_SNAPSHOT_ONE_THIRD, 0, 1, 0, 0},
         {SENSOR_MODE_PREVIEW_TWO, 0, 1, 0, 0},
         {SENSOR_MODE_SNAPSHOT_TWO_FIRST, 0, 1, 0, 0},
         {SENSOR_MODE_SNAPSHOT_TWO_SECOND, 0, 1, 0, 0},
         {SENSOR_MODE_SNAPSHOT_TWO_THIRD, 0, 1, 0, 0}}}
    }
    /*If there are multiple modules,please add here*/
};


static struct sensor_module_info s_ov02b1b_module_info_tab[VENDOR_NUM] = {
    {.module_id = MODULE_SUNNY,
     .module_info = {
         .major_i2c_addr = I2C_SLAVE_ADDR >> 1,
         .minor_i2c_addr = I2C_SLAVE_ADDR >> 1,

         .reg_addr_value_bits = SENSOR_I2C_REG_8BIT | SENSOR_I2C_VAL_8BIT |
                                SENSOR_I2C_FREQ_400,

         .avdd_val = SENSOR_AVDD_2800MV,
         .iovdd_val = SENSOR_AVDD_1800MV,
         .dvdd_val = SENSOR_AVDD_CLOSED,

         .image_pattern = SENSOR_IMAGE_PATTERN_RAWRGB_B, //  SENSOR_IMAGE_PATTERN_RAWRGB_R

         .preview_skip_num = 1,
         .capture_skip_num = 1,
         .flash_capture_skip_num = 6,
         .mipi_cap_skip_num = 0,
         .preview_deci_num = 0,
         .video_preview_deci_num = 0,

         .threshold_eb = 0,
         .threshold_mode = 0,
         .threshold_start = 0,
         .threshold_end = 0,

         .sensor_interface = {
              .type = SENSOR_INTERFACE_TYPE_CSI2,
              .bus_width = LANE_NUM,
              .pixel_width = RAW_BITS,
               #ifdef _SENSOR_RAW_SHARKL5PRO_H_,
			.is_loose = 2,
	        #else
			.is_loose = 0,
		 #endif
          },
         .change_setting_skip_num = 1,
         .horizontal_view_angle = 65,
         .vertical_view_angle = 60
      }
    }

/*If there are multiple modules,please add here*/
};

static struct sensor_ic_ops s_ov02b1b_ops_tab;
struct sensor_raw_info *s_ov02b1b_mipi_raw_info_ptr = PNULL;


/*==============================================================================
 * Description:
 * sensor all info
 * please modify this variable acording your spec
 *============================================================================*/
SENSOR_INFO_T g_ov02b1b_mipi_raw_info = {
    .hw_signal_polarity = SENSOR_HW_SIGNAL_PCLK_P | SENSOR_HW_SIGNAL_VSYNC_P |
                          SENSOR_HW_SIGNAL_HSYNC_P,
    .environment_mode = SENSOR_ENVIROMENT_NORMAL | SENSOR_ENVIROMENT_NIGHT,
    .image_effect = SENSOR_IMAGE_EFFECT_NORMAL |
                    SENSOR_IMAGE_EFFECT_BLACKWHITE | SENSOR_IMAGE_EFFECT_RED |
                    SENSOR_IMAGE_EFFECT_GREEN | SENSOR_IMAGE_EFFECT_BLUE |
                    SENSOR_IMAGE_EFFECT_YELLOW | SENSOR_IMAGE_EFFECT_NEGATIVE |
                    SENSOR_IMAGE_EFFECT_CANVAS,

    .wb_mode = 0,
    .step_count = 7,
    .reset_pulse_level = SENSOR_LOW_PULSE_RESET,
    .reset_pulse_width = 50,
    .power_down_level = SENSOR_LOW_PULSE_RESET,
    .identify_count = 1,
    .identify_code =
        {{ .reg_addr = ov02b1b_PID_ADDR, .reg_value = ov02b1b_PID_VALUE},
         { .reg_addr = ov02b1b_VER_ADDR, .reg_value = ov02b1b_VER_VALUE}},

    .source_width_max = SNAPSHOT_WIDTH,
    .source_height_max = SNAPSHOT_HEIGHT,
    .name = (cmr_s8 *)SENSOR_NAME,
    .image_format = SENSOR_IMAGE_FORMAT_RAW,

    .module_info_tab = s_ov02b1b_module_info_tab,
    .module_info_tab_size = ARRAY_SIZE(s_ov02b1b_module_info_tab),

    .resolution_tab_info_ptr = s_ov02b1b_resolution_tab_raw,
    .sns_ops = &s_ov02b1b_ops_tab,
    .raw_info_ptr = &s_ov02b1b_mipi_raw_info_ptr,

    .video_tab_info_ptr = NULL,
    .sensor_version_info = (cmr_s8 *)"ov02b1b_v1",
};

#endif
