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

#ifndef _SENSOR_s5k4h9yx_MIPI_RAW_H_
#define _SENSOR_s5k4h9yx_MIPI_RAW_H_


#include <utils/Log.h>
#include "sensor.h"
#include "jpeg_exif_header.h"
#include "sensor_drv_u.h"
#include "sensor_raw.h"

//#define FEATURE_OTP

#define VENDOR_NUM 1
#define SENSOR_NAME			"s5k4h9yx_mipi_raw"

#define I2C_SLAVE_ADDR 			0x20 		/* 8bit slave address*/

#define s5k4h9yx_PID_ADDR			0x0000
#define s5k4h9yx_PID_VALUE			0x4089
#define s5k4h9yx_VER_ADDR			0x0000
#define s5k4h9yx_VER_VALUE			0x4089

/* sensor parameters begin */

/* effective sensor output image size */
#define VIDEO_WIDTH				1280
#define VIDEO_HEIGHT			720
#define PREVIEW_WIDTH			1632
#define PREVIEW_HEIGHT			1224
#define SNAPSHOT_WIDTH			3264 
#define SNAPSHOT_HEIGHT			2448

/*Raw Trim parameters*/
#define VIDEO_TRIM_X			0
#define VIDEO_TRIM_Y			0
#define VIDEO_TRIM_W			1280
#define VIDEO_TRIM_H			720
#define PREVIEW_TRIM_X			0
#define PREVIEW_TRIM_Y			0
#define PREVIEW_TRIM_W			1632
#define PREVIEW_TRIM_H			1224
#define SNAPSHOT_TRIM_X			0
#define SNAPSHOT_TRIM_Y			0
#define SNAPSHOT_TRIM_W			3264
#define SNAPSHOT_TRIM_H			2448

/*Mipi output*/
#define LANE_NUM			2
#define RAW_BITS			10

#define VIDEO_MIPI_PER_LANE_BPS	  	  360  /* 2*Mipi clk */
#define PREVIEW_MIPI_PER_LANE_BPS	  792  /* 2*Mipi clk */
#define SNAPSHOT_MIPI_PER_LANE_BPS	  1212  /* 2*Mipi clk */

/*line time unit: 1ns*/
#define VIDEO_LINE_TIME		  	  13194
#define PREVIEW_LINE_TIME		  12933
#define SNAPSHOT_LINE_TIME		  16222

/* frame length*/
#define VIDEO_FRAME_LENGTH			842
#define PREVIEW_FRAME_LENGTH		2564
#define SNAPSHOT_FRAME_LENGTH		2564

/* please ref your spec */
#define FRAME_OFFSET			6
#define SENSOR_MAX_GAIN			0x0200
#define SENSOR_BASE_GAIN		0x0020
#define SENSOR_MIN_SHUTTER		8

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
static const SENSOR_REG_T s5k4h9yx_init_setting1[] = {
{0x6028, 0x4000},  
{0x6010, 0x0001},  
{0xffff, 0x000a},//must delay 3ms here
{0x6028, 0x4000},  
{0x6214, 0x7971},  
{0x6218, 0x7150},  
{0xF468, 0x0017},  
{0xF466, 0x0010},  
{0xF416, 0x44C6},  
{0xF418, 0x002F},  
{0xF482, 0x09DC},  
{0xF410, 0x0005},  
{0xF412, 0x0002},  
{0xF452, 0x001A},  
{0xF448, 0x0010},  
{0xF450, 0x000C},  
{0x364C, 0x0014},  
{0x3646, 0x0DE7},  
{0x3648, 0x03FD},  
{0x3C74, 0x0040},  
{0x6028, 0x2000},  
{0x602A, 0x1444},  
{0x6F12, 0x8010},  
{0x6028, 0x4000},  
{0x0200, 0x0000},  
{0x31B2, 0x0001},  
{0xB136, 0x0000},  
{0xB138, 0x0000},  
{0x31BA, 0x0200},  
                   

};
static const SENSOR_REG_T s5k4h9yx_preview_setting1[] = {
/*res1	            
2x2binning size_30fp
width	1640          
height	1232        
frame rate(fps)	30.1
vt_pix_clk(Mhz)	360 
mipi_lane_num	2     
mipi speed(Mbps/lane
line time(0.1us unit
frame length	2564  
Extclk(Mhz)	24      
Address	Data (Hex) */
{0x6028, 0x4000},   
{0x360A, 0x0705},   
{0x3644, 0x084C},   
{0x3C90, 0x06C0},   
{0x3C92, 0x076C},   
{0x35E4, 0x0001},   
{0x3240, 0x005A},   
{0x3244, 0x0071},   
{0x324C, 0x0074},   
{0x3460, 0x008B},   
{0x3464, 0x0073},   
{0x3468, 0x0031},   
{0x346C, 0x000E},   
{0x3470, 0x0085},   
{0x3474, 0x0079},   
{0x3478, 0x002B},   
{0x3480, 0x002E},   
{0x3484, 0x0011},   
{0x3488, 0x0085},   
{0x348C, 0x0079},   
{0x3490, 0x002B},   
{0x3254, 0x010F},   
{0x3258, 0x005A},   
{0x325C, 0x010D},   
{0x3264, 0x0077},   
{0x326C, 0x0021},   
{0x3274, 0x001D},   
{0x327C, 0x0020},   
{0x3290, 0x0071},   
{0x3294, 0x007B},   
{0x32A4, 0x0006},   
{0x32A8, 0x0025},   
{0x32AC, 0x010D},   
{0x32B0, 0x002E},   
{0x32B8, 0x0088},   
{0x32BC, 0x010D},   
{0x32C0, 0x002C},   
{0x32C8, 0x0086},   
{0x32CC, 0x010F},   
{0x32D4, 0x0005},   
{0x32F8, 0x0091},   
{0x32FC, 0x010B},   
{0x3348, 0x008F},   
{0x334C, 0x010D},   
{0x3380, 0x002A},   
{0x3384, 0x0111},   
{0x3388, 0x002B},   
{0x338C, 0x002D},   
{0x3398, 0x010D},   
{0x339C, 0x010F},   
{0x349C, 0x006E},   
{0x34B8, 0x0063},   
{0x34CC, 0x006E},   
{0x34D0, 0x0015},   
{0x34D8, 0x0063},   
{0x34E4, 0x0074},   
{0x34FC, 0x00C8},   
{0x3500, 0x0063},   
{0x3514, 0x0074},   
{0x3518, 0x0015},   
{0x3520, 0x0063},   
{0x3232, 0x0006},   
{0x323A, 0x0007},   
{0x323E, 0x0061},   
{0x3242, 0x0078},   
{0x3246, 0x005F},   
{0x345E, 0x0097},   
{0x3462, 0x0089},   
{0x3466, 0x003B},   
{0x346E, 0x0091},   
{0x3472, 0x008F},   
{0x3476, 0x0035},   
{0x347E, 0x0038},   
{0x3486, 0x0091},   
{0x348A, 0x008F},   
{0x348E, 0x0035},   
{0x324E, 0x005F},   
{0x3252, 0x011E},   
{0x3256, 0x0061},   
{0x325A, 0x011C},   
{0x325E, 0x005F},   
{0x3262, 0x007E},   
{0x326A, 0x001F},   
{0x3272, 0x001B},   
{0x327A, 0x001E},   
{0x3282, 0x005F},   
{0x328E, 0x0078},   
{0x3292, 0x0085},   
{0x329A, 0x0006},   
{0x32A6, 0x0023},   
{0x32AA, 0x011C},   
{0x32AE, 0x0030},   
{0x32B2, 0x0061},   
{0x32B6, 0x0090},   
{0x32BA, 0x011C},   
{0x32BE, 0x002E},   
{0x32C2, 0x0063},   
{0x32C6, 0x008E},   
{0x32CA, 0x011E},   
{0x32CE, 0x0004},   
{0x32D2, 0x0003},   
{0x32EE, 0x003A},   
{0x32F2, 0x005F},   
{0x32F6, 0x009A},   
{0x32FA, 0x011A},   
{0x32FE, 0x0067},   
{0x3302, 0x007E},   
{0x3306, 0x006E},   
{0x330A, 0x0086},   
{0x330E, 0x0076},   
{0x3312, 0x0086},   
{0x3316, 0x0067},   
{0x331A, 0x0069},   
{0x3326, 0x006E},   
{0x332A, 0x0086},   
{0x3336, 0x0067},   
{0x333A, 0x0069},   
{0x333E, 0x0038},   
{0x3342, 0x0061},   
{0x3346, 0x0098},   
{0x334A, 0x011C},   
{0x335E, 0x0067},   
{0x3362, 0x0081},   
{0x336E, 0x0067},   
{0x3372, 0x0069},   
{0x337E, 0x0025},   
{0x3382, 0x0120},   
{0x3386, 0x0026},   
{0x338A, 0x0028},   
{0x338E, 0x0061},   
{0x3392, 0x0063},   
{0x3396, 0x011C},   
{0x339A, 0x011E},   
{0x339E, 0x0061},   
{0x349A, 0x0080},   
{0x34B2, 0x00DC},   
{0x34B6, 0x0061},   
{0x34CA, 0x0080},   
{0x34CE, 0x000B},   
{0x34D6, 0x0061},   
{0x34E2, 0x0086},   
{0x34FA, 0x00E2},   
{0x34FE, 0x0061},   
{0x3512, 0x0086},   
{0x3516, 0x000B},   
{0x351E, 0x0061},   
{0x3652, 0x007B},   
{0x3654, 0x005F},   
{0x0344, 0x0000},   
{0x0348, 0x0CCF},   
{0x0346, 0x0000},   
{0x034A, 0x099F},   
{0x034C, 0x0668},   
{0x034E, 0x04D0},   
{0x31AC, 0x0004},   
{0x31B0, 0x0005},   
{0x0202, 0x0100},   
{0x0B04, 0x0101},   
{0x307C, 0x0340},   
{0x030E, 0x0079},   
{0x300A, 0x0000},   
{0x0342, 0x1230},   
{0x0340, 0x0A04},   
{0x3178, 0x003D},   
{0x0900, 0x0112},   
{0x0380, 0x0001},   
{0x0382, 0x0001},   
{0x0384, 0x0001},   
{0x0386, 0x0003},   
{0x0400, 0x0001},   
{0x0404, 0x0020},   
{0x0B00, 0x0080},   
//{0x0100, 0x0100},   


};
static const SENSOR_REG_T s5k4h9yx_snapshot_setting1[] = {
/*res0	           
full size_30fps	   
width	3280         
height	2464       
frame rate(fps)	30.
vt_pix_clk(Mhz)	360
mipi_lane_num	2    
mipi speed(Mbps/lan
line time(0.1us uni
frame length	2564 
Extclk(Mhz)	24     
Address	Data (Hex)*/
{0x6028, 0x4000},  
{0x360A, 0x0705},  
{0x3644, 0x084C},  
{0x3C90, 0x06C0},  
{0x3C92, 0x076C},  
{0x35E4, 0x0001},  
{0x3240, 0x005A},  
{0x3244, 0x0071},  
{0x324C, 0x0074},  
{0x3460, 0x008B},  
{0x3464, 0x0073},  
{0x3468, 0x0031},  
{0x346C, 0x000E},  
{0x3470, 0x0085},  
{0x3474, 0x0079},  
{0x3478, 0x002B},  
{0x3480, 0x002E},  
{0x3484, 0x0011},  
{0x3488, 0x0085},  
{0x348C, 0x0079},  
{0x3490, 0x002B},  
{0x3254, 0x010F},  
{0x3258, 0x005A},  
{0x325C, 0x010D},  
{0x3264, 0x0077},  
{0x326C, 0x0021},  
{0x3274, 0x001D},  
{0x327C, 0x0020},  
{0x3290, 0x0071},  
{0x3294, 0x007B},  
{0x32A4, 0x0006},  
{0x32A8, 0x0025},  
{0x32AC, 0x010D},  
{0x32B0, 0x002E},  
{0x32B8, 0x0088},  
{0x32BC, 0x010D},  
{0x32C0, 0x002C},  
{0x32C8, 0x0086},  
{0x32CC, 0x010F},  
{0x32D4, 0x0005},  
{0x32F8, 0x0091},  
{0x32FC, 0x010B},  
{0x3348, 0x008F},  
{0x334C, 0x010D},  
{0x3380, 0x002A},  
{0x3384, 0x0111},  
{0x3388, 0x002B},  
{0x338C, 0x002D},  
{0x3398, 0x010D},  
{0x339C, 0x010F},  
{0x349C, 0x006E},  
{0x34B8, 0x0063},  
{0x34CC, 0x006E},  
{0x34D0, 0x0015},  
{0x34D8, 0x0063},  
{0x34E4, 0x0074},  
{0x34FC, 0x00C8},  
{0x3500, 0x0063},  
{0x3514, 0x0074},  
{0x3518, 0x0015},  
{0x3520, 0x0063},  
{0x3232, 0x0006},  
{0x323A, 0x0007},  
{0x323E, 0x0061},  
{0x3242, 0x0078},  
{0x3246, 0x005F},  
{0x345E, 0x0097},  
{0x3462, 0x0089},  
{0x3466, 0x003B},  
{0x346E, 0x0091},  
{0x3472, 0x008F},  
{0x3476, 0x0035},  
{0x347E, 0x0038},  
{0x3486, 0x0091},  
{0x348A, 0x008F},  
{0x348E, 0x0035},  
{0x324E, 0x005F},  
{0x3252, 0x011E},  
{0x3256, 0x0061},  
{0x325A, 0x011C},  
{0x325E, 0x005F},  
{0x3262, 0x007E},  
{0x326A, 0x001F},  
{0x3272, 0x001B},  
{0x327A, 0x001E},  
{0x3282, 0x005F},  
{0x328E, 0x0078},  
{0x3292, 0x0085},  
{0x329A, 0x0006},  
{0x32A6, 0x0023},  
{0x32AA, 0x011C},  
{0x32AE, 0x0030},  
{0x32B2, 0x0061},  
{0x32B6, 0x0090},  
{0x32BA, 0x011C},  
{0x32BE, 0x002E},  
{0x32C2, 0x0063},  
{0x32C6, 0x008E},  
{0x32CA, 0x011E},  
{0x32CE, 0x0004},  
{0x32D2, 0x0003},  
{0x32EE, 0x003A},  
{0x32F2, 0x005F},  
{0x32F6, 0x009A},  
{0x32FA, 0x011A},  
{0x32FE, 0x0067},  
{0x3302, 0x007E},  
{0x3306, 0x006E},  
{0x330A, 0x0086},  
{0x330E, 0x0076},  
{0x3312, 0x0086},  
{0x3316, 0x0067},  
{0x331A, 0x0069},  
{0x3326, 0x006E},  
{0x332A, 0x0086},  
{0x3336, 0x0067},  
{0x333A, 0x0069},  
{0x333E, 0x0038},  
{0x3342, 0x0061},  
{0x3346, 0x0098},  
{0x334A, 0x011C},  
{0x335E, 0x0067},  
{0x3362, 0x0081},  
{0x336E, 0x0067},  
{0x3372, 0x0069},  
{0x337E, 0x0025},  
{0x3382, 0x0120},  
{0x3386, 0x0026},  
{0x338A, 0x0028},  
{0x338E, 0x0061},  
{0x3392, 0x0063},  
{0x3396, 0x011C},  
{0x339A, 0x011E},  
{0x339E, 0x0061},  
{0x349A, 0x0080},  
{0x34B2, 0x00DC},  
{0x34B6, 0x0061},  
{0x34CA, 0x0080},  
{0x34CE, 0x000B},  
{0x34D6, 0x0061},  
{0x34E2, 0x0086},  
{0x34FA, 0x00E2},  
{0x34FE, 0x0061},  
{0x3512, 0x0086},  
{0x3516, 0x000B},  
{0x351E, 0x0061},  
{0x3652, 0x007B},  
{0x3654, 0x005F},  
{0x0344, 0x0000},  
{0x0348, 0x0CCF},  
{0x0346, 0x0000},  
{0x034A, 0x099F},  
{0x034C, 0x0CD0},  
{0x034E, 0x09A0},  
{0x31AC, 0x0004},  
{0x31B0, 0x0005},  
{0x0202, 0x0100},  
{0x0B04, 0x0101},  
{0x307C, 0x0340},  
{0x030E, 0x0079},  
{0x300A, 0x0000},  
{0x0342, 0x1230},  
{0x0340, 0x0A04},  
{0x3178, 0x007B},  
{0x0900, 0x0011},  
{0x0380, 0x0001},  
{0x0382, 0x0001},  
{0x0384, 0x0001},  
{0x0386, 0x0001},  
{0x0400, 0x0000},  
{0x0404, 0x0010},  
{0x0B00, 0x0080},  
//{0x0100, 0x0100},   


};
static const SENSOR_REG_T s5k4h9yx_init_setting[] = {
	//Sensor Information////////////////////////////
	//Sensor	  : s5k4h9yx                            
	//Date		  : 2017-10-19                       
	//Customer        : SPRD_validation                                            
	//MCLK	          : 24MHz                   
	//MIPI            : 4 Lane                                   
	//Pixel order 	  : B 1st            
	//BLC offset	  : 64code                       
	//Firmware Ver.   : v1.0                         
	////////////////////////////////////////////////

	{0x6028 ,0x4000},
	{0x6010 ,0x0001},
	//must delay 3ms here	
	{0xffff, 0x000a}, // must add delay >3ms
	{0x6028 ,0x4000},
	{0x6214 ,0x7971},
	{0x6218 ,0x7150},
	{0xF468 ,0x0017},
	{0xF466 ,0x0010},
	{0xF416 ,0x44C6},
	{0xF418 ,0x002F},
	{0xF482 ,0x09DC},
	{0xF410 ,0x0005},
	{0xF412 ,0x0002},
	{0xF452 ,0x001A},
	{0xF448 ,0x0010},
	{0xF450 ,0x000C},
	{0x364C ,0x0014},
	{0x3646 ,0x0DE7},
	{0x3648 ,0x03FD},
	{0x3C74 ,0x0040},
	{0x6028 ,0x2000},
	{0x602A ,0x1444},
	{0x6F12 ,0x8010},
	{0x6028 ,0x4000},
	{0x0200 ,0x0000},
	{0x31B2 ,0x0001},
	{0xB136 ,0x0000},
	{0xB138 ,0x0000},
	{0x31BA ,0x0200},

};

static const SENSOR_REG_T s5k4h9yx_preview_setting[] = {
	//Sensor Information////////////////////////////
	//Sensor	  : s5k4h9yx
	//Date		  : 2017-10-19                       
	//Customer        : SPRD_validation
	//Image size	  : 1632x1224
	//MCLK/PCLK	  : 24MHz /288Mhz
	//MIPI speed(Mbps): 360Mbps x 4Lane
	//Frame Length	  : 2492
	//Line Length 	  : 3800
	//line Time       :13194 
	//Max Fps 	  : 30.00fps
	//Pixel order 	  : Green 1st (=GB)
	//X/Y-flip        : X-flip
	//BLC offset	    : 64code   
	//Firmware Ver.   : v1.0
	////////////////////////////////////////////////
	{0x6028 ,0x4000},
	{0x360A ,0x0705},
	{0x3644 ,0x084C},
	{0x3C90 ,0x06C0},
	{0x3C92 ,0x076C},
	{0x35E4 ,0x0001},
	{0x3240 ,0x005A},
	{0x3244 ,0x0071},
	{0x324C ,0x0074},
	{0x3460 ,0x008B},
	{0x3464 ,0x0073},
	{0x3468 ,0x0031},
	{0x346C ,0x000E},
	{0x3470 ,0x0085},
	{0x3474 ,0x0079},
	{0x3478 ,0x002B},
	{0x3480 ,0x002E},
	{0x3484 ,0x0011},
	{0x3488 ,0x0085},
	{0x348C ,0x0079},
	{0x3490 ,0x002B},
	{0x3254 ,0x010F},
	{0x3258 ,0x005A},
	{0x325C ,0x010D},
	{0x3264 ,0x0077},
	{0x326C ,0x0021},
	{0x3274 ,0x001D},
	{0x327C ,0x0020},
	{0x3290 ,0x0071},
	{0x3294 ,0x007B},
	{0x32A4 ,0x0006},
	{0x32A8 ,0x0025},
	{0x32AC ,0x010D},
	{0x32B0 ,0x002E},
	{0x32B8 ,0x0088},
	{0x32BC ,0x010D},
	{0x32C0 ,0x002C},
	{0x32C8 ,0x0086},
	{0x32CC ,0x010F},
	{0x32D4 ,0x0005},
	{0x32F8 ,0x0091},
	{0x32FC ,0x010B},
	{0x3348 ,0x008F},
	{0x334C ,0x010D},
	{0x3380 ,0x002A},
	{0x3384 ,0x0111},
	{0x3388 ,0x002B},
	{0x338C ,0x002D},
	{0x3398 ,0x010D},
	{0x339C ,0x010F},
	{0x349C ,0x006E},
	{0x34B8 ,0x0063},
	{0x34CC ,0x006E},
	{0x34D0 ,0x0015},
	{0x34D8 ,0x0063},
	{0x34E4 ,0x0074},
	{0x34FC ,0x00C8},
	{0x3500 ,0x0063},
	{0x3514 ,0x0074},
	{0x3518 ,0x0015},
	{0x3520 ,0x0063},
	{0x3232 ,0x0006},
	{0x323A ,0x0007},
	{0x323E ,0x0061},
	{0x3242 ,0x0078},
	{0x3246 ,0x005F},
	{0x345E ,0x0097},
	{0x3462 ,0x0089},
	{0x3466 ,0x003B},
	{0x346E ,0x0091},
	{0x3472 ,0x008F},
	{0x3476 ,0x0035},
	{0x347E ,0x0038},
	{0x3486 ,0x0091},
	{0x348A ,0x008F},
	{0x348E ,0x0035},
	{0x324E ,0x005F},
	{0x3252 ,0x011E},
	{0x3256 ,0x0061},
	{0x325A ,0x011C},
	{0x325E ,0x005F},
	{0x3262 ,0x007E},
	{0x326A ,0x001F},
	{0x3272 ,0x001B},
	{0x327A ,0x001E},
	{0x3282 ,0x005F},
	{0x328E ,0x0078},
	{0x3292 ,0x0085},
	{0x329A ,0x0006},
	{0x32A6 ,0x0023},
	{0x32AA ,0x011C},
	{0x32AE ,0x0030},
	{0x32B2 ,0x0061},
	{0x32B6 ,0x0090},
	{0x32BA ,0x011C},
	{0x32BE ,0x002E},
	{0x32C2 ,0x0063},
	{0x32C6 ,0x008E},
	{0x32CA ,0x011E},
	{0x32CE ,0x0004},
	{0x32D2 ,0x0003},
	{0x32EE ,0x003A},
	{0x32F2 ,0x005F},
	{0x32F6 ,0x009A},
	{0x32FA ,0x011A},
	{0x32FE ,0x0067},
	{0x3302 ,0x007E},
	{0x3306 ,0x006E},
	{0x330A ,0x0086},
	{0x330E ,0x0076},
	{0x3312 ,0x0086},
	{0x3316 ,0x0067},
	{0x331A ,0x0069},
	{0x3326 ,0x006E},
	{0x332A ,0x0086},
	{0x3336 ,0x0067},
	{0x333A ,0x0069},
	{0x333E ,0x0038},
	{0x3342 ,0x0061},
	{0x3346 ,0x0098},
	{0x334A ,0x011C},
	{0x335E ,0x0067},
	{0x3362 ,0x0081},
	{0x336E ,0x0067},
	{0x3372 ,0x0069},
	{0x337E ,0x0025},
	{0x3382 ,0x0120},
	{0x3386 ,0x0026},
	{0x338A ,0x0028},
	{0x338E ,0x0061},
	{0x3392 ,0x0063},
	{0x3396 ,0x011C},
	{0x339A ,0x011E},
	{0x339E ,0x0061},
	{0x349A ,0x0080},
	{0x34B2 ,0x00DC},
	{0x34B6 ,0x0061},
	{0x34CA ,0x0080},
	{0x34CE ,0x000B},
	{0x34D6 ,0x0061},
	{0x34E2 ,0x0086},
	{0x34FA ,0x00E2},
	{0x34FE ,0x0061},
	{0x3512 ,0x0086},
	{0x3516 ,0x000B},
	{0x351E ,0x0061},
	{0x3652 ,0x007B},
	{0x3654 ,0x005F},
	{0x0344 ,0x0000},
	{0x0348 ,0x0CCF},
	{0x0346 ,0x0000},
	{0x034A ,0x099F},
	{0x034C ,0x0660},//0x0668  1632X1224
	{0x034E ,0x04c8},//0x04D0
	{0x31AC ,0x0004},
	{0x31B0 ,0x0005},
	{0x0202 ,0x0100},
	{0x0B04 ,0x0101},
	{0x307C ,0x0340},
	{0x030E,0x0084},
    {0x300A,0x0001},
    //{0x0342,0x2460},
	//{0x030E ,0x0079},
	//{0x300A ,0x0000},
	{0x0342 ,0x1230},
	{0x0340 ,0x0A04},
	{0x3178 ,0x003D},
	{0x0900 ,0x0112},
	{0x0380 ,0x0001},
	{0x0382 ,0x0001},
	{0x0384 ,0x0001},
	{0x0386 ,0x0003},
	{0x0400 ,0x0001},
	{0x0404 ,0x0020},
	{0x0B00 ,0x0080},

};

static const SENSOR_REG_T s5k4h9yx_snapshot_setting[] = {
	//Sensor Information////////////////////////////
	//Sensor	  : s5k4h9yx                            
	//Date		  : 2017-04-06                       
	//Customer        : SPRD_validation                      
	//Image size	  : 3264x2448                     
	//MCLK/PCLK	  : 24MHz /288Mhz                   
	//MIPI speed(Mbps): 720Mbps x 4Lane             
	//Frame Length	  : 2492                        
	//Line Length 	  : 3800
	//line Time       :13194                         
	//Max Fps 	  : 30.00fps                                              
	//Pixel order 	  : Green 1st (=GB)             
	//X/Y-flip        : X-flip  
	//BLC offset	    : 64code                       
	//Firmware Ver.   : v1.0                        
	////////////////////////////////////////////////
	{0x6028,0x4000},
	{0x360A,0x0705},
	{0x3644,0x084C},
	{0x3C90,0x06C0},
	{0x3C92,0x076C},
	{0x35E4,0x0001},
	{0x3240,0x005A},
	{0x3244,0x0071},
	{0x324C,0x0074},
	{0x3460,0x008B},
	{0x3464,0x0073},
	{0x3468,0x0031},
	{0x346C,0x000E},
	{0x3470,0x0085},
	{0x3474,0x0079},
	{0x3478,0x002B},
	{0x3480,0x002E},
	{0x3484,0x0011},
	{0x3488,0x0085},
	{0x348C,0x0079},
	{0x3490,0x002B},
	{0x3254,0x010F},
	{0x3258,0x005A},
	{0x325C,0x010D},
	{0x3264,0x0077},
	{0x326C,0x0021},
	{0x3274,0x001D},
	{0x327C,0x0020},
	{0x3290,0x0071},
	{0x3294,0x007B},
	{0x32A4,0x0006},
	{0x32A8,0x0025},
	{0x32AC,0x010D},
	{0x32B0,0x002E},
	{0x32B8,0x0088},
	{0x32BC,0x010D},
	{0x32C0,0x002C},
	{0x32C8,0x0086},
	{0x32CC,0x010F},
	{0x32D4,0x0005},
	{0x32F8,0x0091},
	{0x32FC,0x010B},
	{0x3348,0x008F},
	{0x334C,0x010D},
	{0x3380,0x002A},
	{0x3384,0x0111},
	{0x3388,0x002B},
	{0x338C,0x002D},
	{0x3398,0x010D},
	{0x339C,0x010F},
	{0x349C,0x006E},
	{0x34B8,0x0063},
	{0x34CC,0x006E},
	{0x34D0,0x0015},
	{0x34D8,0x0063},
	{0x34E4,0x0074},
	{0x34FC,0x00C8},
	{0x3500,0x0063},
	{0x3514,0x0074},
	{0x3518,0x0015},
	{0x3520,0x0063},
	{0x3232,0x0006},
	{0x323A,0x0007},
	{0x323E,0x0061},
	{0x3242,0x0078},
	{0x3246,0x005F},
	{0x345E,0x0097},
	{0x3462,0x0089},
	{0x3466,0x003B},
	{0x346E,0x0091},
	{0x3472,0x008F},
	{0x3476,0x0035},
	{0x347E,0x0038},
	{0x3486,0x0091},
	{0x348A,0x008F},
	{0x348E,0x0035},
	{0x324E,0x005F},
	{0x3252,0x011E},
	{0x3256,0x0061},
	{0x325A,0x011C},
	{0x325E,0x005F},
	{0x3262,0x007E},
	{0x326A,0x001F},
	{0x3272,0x001B},
	{0x327A,0x001E},
	{0x3282,0x005F},
	{0x328E,0x0078},
	{0x3292,0x0085},
	{0x329A,0x0006},
	{0x32A6,0x0023},
	{0x32AA,0x011C},
	{0x32AE,0x0030},
	{0x32B2,0x0061},
	{0x32B6,0x0090},
	{0x32BA,0x011C},
	{0x32BE,0x002E},
	{0x32C2,0x0063},
	{0x32C6,0x008E},
	{0x32CA,0x011E},
	{0x32CE,0x0004},
	{0x32D2,0x0003},
	{0x32EE,0x003A},
	{0x32F2,0x005F},
	{0x32F6,0x009A},
	{0x32FA,0x011A},
	{0x32FE,0x0067},
	{0x3302,0x007E},
	{0x3306,0x006E},
	{0x330A,0x0086},
	{0x330E,0x0076},
	{0x3312,0x0086},
	{0x3316,0x0067},
	{0x331A,0x0069},
	{0x3326,0x006E},
	{0x332A,0x0086},
	{0x3336,0x0067},
	{0x333A,0x0069},
	{0x333E,0x0038},
	{0x3342,0x0061},
	{0x3346,0x0098},
	{0x334A,0x011C},
	{0x335E,0x0067},
	{0x3362,0x0081},
	{0x336E,0x0067},
	{0x3372,0x0069},
	{0x337E,0x0025},
	{0x3382,0x0120},
	{0x3386,0x0026},
	{0x338A,0x0028},
	{0x338E,0x0061},
	{0x3392,0x0063},
	{0x3396,0x011C},
	{0x339A,0x011E},
	{0x339E,0x0061},
	{0x349A,0x0080},
	{0x34B2,0x00DC},
	{0x34B6,0x0061},
	{0x34CA,0x0080},
	{0x34CE,0x000B},
	{0x34D6,0x0061},
	{0x34E2,0x0086},
	{0x34FA,0x00E2},
	{0x34FE,0x0061},
	{0x3512,0x0086},
	{0x3516,0x000B},
	{0x351E,0x0061},
	{0x3652,0x007B},
	{0x3654,0x005F},
	{0x0344,0x0008},
	{0x0348,0x0CC7},
	{0x0346,0x0008},
	{0x034A,0x0997},
	{0x034C,0x0CC0},
	{0x034E,0x0990},
	{0x31AC,0x0004},
	{0x31B0,0x0005},
	{0x0202,0x0100},
	{0x0B04,0x0101},
	{0x307C,0x0340},
	{0x030E,0x0065},//0x0084
	{0x300A,0x0000},//0x0001
	{0x0342,0x16D0},//0x2460
	{0x0340,0x0A04},
	{0x3178,0x007B},
	{0x0900,0x0011},
	{0x0380,0x0001},
	{0x0382,0x0001},
	{0x0384,0x0001},
	{0x0386,0x0001},
	{0x0400,0x0000},
	{0x0404,0x0010},
	{0x0B00,0x0080},
};

static const SENSOR_REG_T s5k4h9yx_video_setting[] = {
	//Sensor Information////////////////////////////
	//Sensor	  : s5k4h9yx
	//Date		  : 2017-04-06                       
	//Customer        : SPRD_validation
	//Image size	  : 1280x720
	//MCLK/PCLK	  : 24MHz /288Mhz
	//MIPI speed(Mbps): 360Mbps x 4Lane
	//Frame Length	  :  842
	//Line Length 	  : 3800
	//line Time       :13194 
	//Max Fps 	  : 90.00fps
	//Pixel order 	  : Green 1st (=GB)
	//X/Y-flip        : X-flip
	//BLC offset	    : 64code   
	//Firmware Ver.   : v1.0
	////////////////////////////////////////////////
};




static struct sensor_res_tab_info s_s5k4h9yx_resolution_tab_raw[VENDOR_NUM] = {
	{
      .module_id = MODULE_SUNNY,
      .reg_tab = {
        {ADDR_AND_LEN_OF_ARRAY(s5k4h9yx_init_setting), PNULL, 0,
        .width = 0, .height = 0,
        .xclk_to_sensor = EX_MCLK, .image_format = SENSOR_IMAGE_FORMAT_RAW},

		/*{ADDR_AND_LEN_OF_ARRAY(s5k4h9yx_video_setting), PNULL, 0,
        .width = VIDEO_WIDTH, .height = VIDEO_HEIGHT,
        .xclk_to_sensor = EX_MCLK, .image_format = SENSOR_IMAGE_FORMAT_RAW},*/
		
        {ADDR_AND_LEN_OF_ARRAY(s5k4h9yx_preview_setting), PNULL, 0,
        .width = PREVIEW_WIDTH, .height = PREVIEW_HEIGHT,
        .xclk_to_sensor = EX_MCLK, .image_format = SENSOR_IMAGE_FORMAT_RAW},

        {ADDR_AND_LEN_OF_ARRAY(s5k4h9yx_snapshot_setting), PNULL, 0,
        .width = SNAPSHOT_WIDTH, .height = SNAPSHOT_HEIGHT,
        .xclk_to_sensor = EX_MCLK, .image_format = SENSOR_IMAGE_FORMAT_RAW}
		}
	}

/*If there are multiple modules,please add here*/
};

static SENSOR_TRIM_T s_s5k4h9yx_resolution_trim_tab[VENDOR_NUM] = {
{
     .module_id = MODULE_SUNNY,
     .trim_info = {
       {0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}},
	   
	   /*{.trim_start_x = VIDEO_TRIM_X, .trim_start_y = VIDEO_TRIM_Y,
        .trim_width = VIDEO_TRIM_W,   .trim_height = VIDEO_TRIM_H,
        .line_time = VIDEO_LINE_TIME, .bps_per_lane = VIDEO_MIPI_PER_LANE_BPS,
        .frame_line = VIDEO_FRAME_LENGTH,
        .scaler_trim = {.x = VIDEO_TRIM_X, .y = VIDEO_TRIM_Y, .w = VIDEO_TRIM_W, .h = VIDEO_TRIM_H}},*/
	   
	   {.trim_start_x = PREVIEW_TRIM_X, .trim_start_y = PREVIEW_TRIM_Y,
        .trim_width = PREVIEW_TRIM_W,   .trim_height = PREVIEW_TRIM_H,
        .line_time = PREVIEW_LINE_TIME, .bps_per_lane = PREVIEW_MIPI_PER_LANE_BPS,
        .frame_line = PREVIEW_FRAME_LENGTH,
        .scaler_trim = {.x = PREVIEW_TRIM_X, .y = PREVIEW_TRIM_Y, .w = PREVIEW_TRIM_W, .h = PREVIEW_TRIM_H}},
       
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

static SENSOR_REG_T s5k4h9yx_shutter_reg[] = {
    {0x0202, 0x0000}, 
};

static struct sensor_i2c_reg_tab s5k4h9yx_shutter_tab = {
    .settings = s5k4h9yx_shutter_reg, 
	.size = ARRAY_SIZE(s5k4h9yx_shutter_reg),
};

static SENSOR_REG_T s5k4h9yx_again_reg[] = {
	{0x0204, 0x0000}, 
};

static struct sensor_i2c_reg_tab s5k4h9yx_again_tab = {
    .settings = s5k4h9yx_again_reg, 
	.size = ARRAY_SIZE(s5k4h9yx_again_reg),
};

static SENSOR_REG_T s5k4h9yx_dgain_reg[] = {
   
};

static struct sensor_i2c_reg_tab s5k4h9yx_dgain_tab = {
    .settings = s5k4h9yx_dgain_reg, 
	.size = ARRAY_SIZE(s5k4h9yx_dgain_reg),
};

static SENSOR_REG_T s5k4h9yx_frame_length_reg[] = {
    {0x0340, 0x0000}, 
};

static struct sensor_i2c_reg_tab s5k4h9yx_frame_length_tab = {
    .settings = s5k4h9yx_frame_length_reg,
    .size = ARRAY_SIZE(s5k4h9yx_frame_length_reg),
};

static struct sensor_aec_i2c_tag s5k4h9yx_aec_info = {
    .slave_addr = (I2C_SLAVE_ADDR >> 1),
    .addr_bits_type = SENSOR_I2C_REG_16BIT,
    .data_bits_type = SENSOR_I2C_VAL_16BIT,
    .shutter = &s5k4h9yx_shutter_tab,
    .again = &s5k4h9yx_again_tab,
    .dgain = &s5k4h9yx_dgain_tab,
    .frame_length = &s5k4h9yx_frame_length_tab,
};

static SENSOR_STATIC_INFO_T s_s5k4h9yx_static_info[VENDOR_NUM] = {
    {.module_id = MODULE_SUNNY,
     .static_info = {
        .f_num = 200,
        .focal_length = 354,
        .max_fps = 30,
        .max_adgain = 8,
        .ois_supported = 0,
		.pdaf_supported = 0,
        .exp_valid_frame_num = 1,
        .clamp_level = 64,
        .adgain_valid_frame_num = 1,
        .fov_info = {{4.614f, 3.444f}, 4.222f}}
    }
    /*If there are multiple modules,please add here*/
};


static SENSOR_MODE_FPS_INFO_T s_s5k4h9yx_mode_fps_info[VENDOR_NUM] = {
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


static struct sensor_module_info s_s5k4h9yx_module_info_tab[VENDOR_NUM] = {
    {.module_id = MODULE_SUNNY,
     .module_info = {
         .major_i2c_addr = 0x20 >> 1,
         .minor_i2c_addr = 0x5a >> 1,

         .reg_addr_value_bits = SENSOR_I2C_REG_16BIT | SENSOR_I2C_VAL_16BIT |
                                SENSOR_I2C_FREQ_400,

         .avdd_val = SENSOR_AVDD_2800MV,
         .iovdd_val = SENSOR_AVDD_1800MV,
         .dvdd_val = SENSOR_AVDD_1000MV,

         .image_pattern = SENSOR_IMAGE_PATTERN_RAWRGB_R,

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
              .is_loose = 0,
          },
         .change_setting_skip_num = 1,
         .horizontal_view_angle = 65,
         .vertical_view_angle = 60
      }
    }

/*If there are multiple modules,please add here*/
};

static struct sensor_ic_ops s_s5k4h9yx_ops_tab;
struct sensor_raw_info *s_s5k4h9yx_mipi_raw_info_ptr = PNULL;


/*==============================================================================
 * Description:
 * sensor all info
 * please modify this variable acording your spec
 *============================================================================*/
SENSOR_INFO_T g_s5k4h9yx_mipi_raw_info = {
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
    .power_down_level = SENSOR_LOW_LEVEL_PWDN,
    .identify_count = 1,
    .identify_code =
        {{ .reg_addr = s5k4h9yx_PID_ADDR, .reg_value = s5k4h9yx_PID_VALUE},
         { .reg_addr = s5k4h9yx_VER_ADDR, .reg_value = s5k4h9yx_VER_VALUE}},

    .source_width_max = SNAPSHOT_WIDTH,
    .source_height_max = SNAPSHOT_HEIGHT,
    .name = (cmr_s8 *)SENSOR_NAME,
    .image_format = SENSOR_IMAGE_FORMAT_RAW,

    .module_info_tab = s_s5k4h9yx_module_info_tab,
    .module_info_tab_size = ARRAY_SIZE(s_s5k4h9yx_module_info_tab),

    .resolution_tab_info_ptr = s_s5k4h9yx_resolution_tab_raw,
    .sns_ops = &s_s5k4h9yx_ops_tab,
    .raw_info_ptr = &s_s5k4h9yx_mipi_raw_info_ptr,

    .video_tab_info_ptr = NULL,
    .sensor_version_info = (cmr_s8 *)"s5k4h9yx_v1",
};

#endif
