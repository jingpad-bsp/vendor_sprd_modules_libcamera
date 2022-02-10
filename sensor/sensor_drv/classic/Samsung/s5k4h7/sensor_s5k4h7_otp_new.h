#include <utils/Log.h>
#include "sensor.h"
#include "sensor_drv_u.h"
#include "sensor_raw.h"
#include <cutils/properties.h>

#define LSC_PARAM_QTY 240
#define I2C_SLAVE_ADDR 0x20



#define GBGR_RATIO_TYPICAL 0x3ff
#define RG_RATIO_TYPICAL 0x22d
#define BG_RATIO_TYPICAL 0x2ec


struct otp_info_t {
    uint16_t flag;
    uint16_t module_id;
    uint16_t lens_id;
    uint16_t vcm_id;
    uint16_t vcm_driver_id;
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t rg_ratio_current;
    uint16_t bg_ratio_current;
    uint16_t gbgr_ratio_current;
    uint16_t rg_ratio_typical;
    uint16_t bg_ratio_typical;
    uint16_t gbgr_ratio_typical;
    uint16_t r_current;
    uint16_t gr_current;
    uint16_t gb_current;
    uint16_t b_current;
    uint16_t r_typical;
    uint16_t b_typical;
    uint16_t gr_typical;
    uint16_t gb_typical;
    uint16_t g_typical;
    uint16_t vcm_dac_start;
    uint16_t vcm_dac_inifity;
    uint16_t vcm_dac_macro;
    uint16_t lsc_param[LSC_PARAM_QTY];
};
static struct otp_info_t s_s5k4h7_sunwin_v800_otp_info;

#define RG_RATIO_TYPICAL_s5k4h7_sunwin_v800 0x22d // 0x108
#define BG_RATIO_TYPICAL_s5k4h7_sunwin_v800 0x2ec // 0x142
#define GBGR_RATIO_TYPICAL_s5k4h7_sunwin_v800 0x3ff
#define R_TYPICAL_s5k4h7_sunwin_v800 0x0000
#define G_TYPICAL_s5k4h7_sunwin_v800 0x0000
#define B_TYPICAL_s5k4h7_sunwin_v800 0x0000

static uint16_t Sensor_readreg8bits(cmr_handle handle, uint16_t addr) {
    SENSOR_IC_CHECK_HANDLE(handle);
    struct sensor_ic_drv_cxt *sns_drv_cxt = (struct sensor_ic_drv_cxt *)handle;
    uint8_t cmd_val[5] = {0x00};
    uint16_t slave_addr = 0;
    uint16_t cmd_len = 0;
    uint32_t ret_value = SENSOR_SUCCESS;

    slave_addr = I2C_SLAVE_ADDR >> 1;

    uint16_t reg_value = 0;
    cmd_val[0] = addr >> 8;
    cmd_val[1] = addr & 0xff;
    cmd_len = 2;
    ret_value =
        hw_sensor_read_i2c(sns_drv_cxt->hw_handle, slave_addr, (uint8_t *)&cmd_val[0], cmd_len);
    if (SENSOR_SUCCESS == ret_value) {
        reg_value = cmd_val[0];
    }
    return reg_value;
}
static uint32_t Sensor_writereg8bits(cmr_handle handle, uint16_t addr,
                                     uint8_t val) {
    SENSOR_IC_CHECK_HANDLE(handle);
    uint8_t cmd_val[5] = {0x00};
    uint16_t slave_addr = 0;
    uint16_t cmd_len = 0;
    uint32_t ret_value = SENSOR_SUCCESS;
    struct sensor_ic_drv_cxt *sns_drv_cxt = (struct sensor_ic_drv_cxt *)handle;

    slave_addr = I2C_SLAVE_ADDR >> 1;

    cmd_val[0] = addr >> 8;
    cmd_val[1] = addr & 0xff;
    cmd_val[2] = val;
    cmd_len = 3;
    ret_value = hw_sensor_write_i2c(sns_drv_cxt->hw_handle, slave_addr, (uint8_t *)&cmd_val[0],
                                    cmd_len);

    return ret_value;
}

cmr_u8 s5k4h7_Sensor_OTP_read(cmr_handle handle, uint16_t otp_addr) {
    SENSOR_IC_CHECK_HANDLE(handle);
    return Sensor_readreg8bits(handle, otp_addr); // OTP data read
}
static void s5k4h7_sunwin_v800_enable_awb_otp(void) {
    /*TODO enable awb otp update*/
    // Sensor_writereg8bits(0x021c, 0x04 | Sensor_readreg8bits(0x021c));
}




static uint32_t s5k4h7_sunwin_v800_update_awb(cmr_handle handle, void *param_ptr) {
    uint32_t rtn = SENSOR_SUCCESS;
    struct otp_info_t *otp_info = &s_s5k4h7_sunwin_v800_otp_info;   //(struct otp_info_t *)param_ptr;
	struct sensor_ic_drv_cxt *sns_drv_cxt = (struct sensor_ic_drv_cxt *)handle;
    #if 1
    /*TODO*/
    int rg,bg,R_gain,G_gain,B_gain,Base_gain;
	int RG_Ratio_Typical,BG_Ratio_Typical ;
	//apply otp wb calibration
	rg = otp_info->rg_ratio_current;// m_otpdata_read.awb_unit.rg_ratio;
	bg = otp_info->bg_ratio_current;//m_otpdata_read.awb_unit.bg_ratio;
	
	RG_Ratio_Typical = RG_RATIO_TYPICAL;//otp_info->rg_ratio_typical;//m_otpdata_read.awb_golden.rg_ratio;
	BG_Ratio_Typical = BG_RATIO_TYPICAL;//otp_info->bg_ratio_typical;//m_otpdata_read.awb_golden.bg_ratio;

	//SENSOR_PRINT(" --rg=0x%x  bg=0x%x   RG_Ratio_Typical=0x%x BG_Ratio_Typical=0x%x---",rg ,bg,RG_Ratio_Typical,BG_Ratio_Typical);
	if (RG_Ratio_Typical == 0 || BG_Ratio_Typical == 0)
	{
		//SENSOR_PRINT("WB Typical error");
		return 0;
	}
	//calibration G gain
	R_gain = (RG_Ratio_Typical*1000)/rg;
	B_gain = (BG_Ratio_Typical*1000)/bg;
	G_gain = 1000;
	
	//SENSOR_PRINT(" 1--R_gain=0x%x  B_gain=0x%x  G_gain=0x%x----",R_gain,B_gain,G_gain);
	//SENSOR_PRINT(" 1--R_gain=%d  B_gain=%d  G_gain=%d----",R_gain,B_gain,G_gain);
	
	if(R_gain < 1000 || B_gain < 1000)
	{
		if(R_gain < B_gain)
			Base_gain = R_gain;
		else
			Base_gain = B_gain;
	}
	else
	{
		Base_gain = G_gain;
	}
	//SENSOR_PRINT(" ----Base_gain=%d ---",Base_gain);
	R_gain = 0x100*R_gain/(Base_gain);
	B_gain = 0x100*B_gain/(Base_gain);
	G_gain = 0x100*G_gain/(Base_gain);

	//SENSOR_PRINT(" 2--R_gain=0x%x  B_gain=0x%x  G_gain=0x%x----",R_gain,B_gain,G_gain);
	//SENSOR_PRINT(" 2--R_gain=%d  B_gain=%d  G_gain=%d----",R_gain,B_gain,G_gain);
	hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x3C0F, 0x00);
	//update sensor wb gainkin

    cmr_u8 resd_otp_buffer[2];
    uint32_t loop;
    
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0210, R_gain >> 8);
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0211, R_gain & 0x00ff);
    
    for(loop = 0;loop < 2;loop++)
    {
        resd_otp_buffer[loop] = hw_sensor_read_reg(sns_drv_cxt->hw_handle, 0x0210 + loop);
        //SENSOR_PRINT("SPRD ReadAWBRigest Addr: [%04x] -- 0x%02x",0x0210 + loop,resd_otp_buffer[loop]);
    }

    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x020E, G_gain >> 8);
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x020F, G_gain & 0x00ff);    
    
    for(loop = 0;loop < 2;loop++)
    {
        resd_otp_buffer[loop] = hw_sensor_read_reg(sns_drv_cxt->hw_handle, 0x020E + loop);
        //SENSOR_PRINT("SPRD ReadAWBRigest Addr: [%04x] -- 0x%02x",0x020E + loop,resd_otp_buffer[loop]);
    }

    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0214, G_gain >> 8);
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0215, G_gain & 0x00ff);    
    
    for(loop = 0;loop < 2;loop++)
    {
        resd_otp_buffer[loop] = hw_sensor_read_reg(sns_drv_cxt->hw_handle, 0x0214 + loop);
        //SENSOR_PRINT("SPRD ReadAWBRigest Addr: [%04x] -- 0x%02x",0x0214 + loop,resd_otp_buffer[loop]);
    }
            
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0212, B_gain >> 8);
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0213, B_gain & 0x00ff);    
    
    for(loop = 0;loop < 2;loop++)
    {
        resd_otp_buffer[loop] = hw_sensor_read_reg(sns_drv_cxt->hw_handle, 0x0212 + loop);
        //SENSOR_PRINT("SPRD ReadAWBRigest Addr: [%04x] -- 0x%02x",0x0212 + loop,resd_otp_buffer[loop]);
    }

    #endif 
    return rtn;
}

static void s5k4h7_sunwin_v800_enable_lsc_otp(void) { /*TODO enable lsc otp update*/
}

static uint32_t s5k4h7_sunwin_v800_update_lsc(cmr_handle handle, void *param_ptr) {
    uint32_t rtn = SENSOR_SUCCESS;
    SENSOR_IC_CHECK_HANDLE(handle);
    //SENSOR_PRINT("s5k4h7_sunwin_v800_update_lsc");
	struct sensor_ic_drv_cxt *sns_drv_cxt = (struct sensor_ic_drv_cxt *)handle;
    struct otp_info_t *otp_info = &s_s5k4h7_sunwin_v800_otp_info;   //(struct otp_info_t *)param_ptr;
	
    //s5k4h7_sunwin_v800_enable_lsc_otp();
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x3400, 0x00);
    hw_sensor_write_reg(sns_drv_cxt->hw_handle, 0x0B00, 0x01);
    /*TODO*/

    return rtn;
}

static uint32_t s5k4h7_sunwin_v800_test_awb(void *param_ptr) {
    uint32_t flag = 1;
    struct otp_info_t *otp_info = &s_s5k4h7_sunwin_v800_otp_info;   //(struct otp_info_t *)param_ptr;
    char value[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.cam.otp.awb", value, "on");
    
    //SENSOR_PRINT("s5k4h7_sunwin_v800_test_awb in");
    
    if (!strcmp(value, "on")) {
        //SENSOR_PRINT("apply awb otp normally!");
#if 1
        otp_info->rg_ratio_typical = RG_RATIO_TYPICAL_s5k4h7_sunwin_v800;
        otp_info->bg_ratio_typical = BG_RATIO_TYPICAL_s5k4h7_sunwin_v800;
        otp_info->gbgr_ratio_typical = GBGR_RATIO_TYPICAL_s5k4h7_sunwin_v800;
        otp_info->r_typical = otp_info->r_typical;
        otp_info->g_typical = otp_info->g_typical;
        otp_info->b_typical = otp_info->b_typical;
#endif

    } else if (!strcmp(value, "test")) {
        //SENSOR_PRINT("apply awb otp on test mode!");
        otp_info->rg_ratio_typical = RG_RATIO_TYPICAL_s5k4h7_sunwin_v800 * 1.5;
        otp_info->bg_ratio_typical = BG_RATIO_TYPICAL_s5k4h7_sunwin_v800 * 1.5;
        otp_info->gbgr_ratio_typical = GBGR_RATIO_TYPICAL_s5k4h7_sunwin_v800 * 1.5;
        otp_info->r_typical = otp_info->g_typical;
        otp_info->g_typical = otp_info->g_typical;
        otp_info->b_typical = otp_info->g_typical;

    } else {
        //SENSOR_PRINT("without apply awb otp!");
        flag = 0;
    }
    return flag;
}

static uint32_t s5k4h7_sunwin_v800_test_lsc(void) {
    uint32_t flag = 1;
    char value[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.cam.otp.lsc", value, "on");

    if (!strcmp(value, "on")) {
        //SENSOR_PRINT("apply lsc otp normally!");
        flag = 1;
    } else {
        //SENSOR_PRINT("without apply lsc otp!");
        flag = 0;
    }
    return flag;
}
static uint32_t s5k4h7_sunwin_v800_read_otp_info(cmr_handle handle, void *param_ptr
                                           ) {
   uint32_t rtn = SENSOR_SUCCESS;
    struct otp_info_t *otp_info = &s_s5k4h7_sunwin_v800_otp_info;   //(struct otp_info_t *)param_ptr;
    SENSOR_IC_CHECK_HANDLE(handle);
    struct sensor_ic_drv_cxt *sns_drv_cxt = (struct sensor_ic_drv_cxt *)handle;
    cmr_u8 otp_buffer[64 * 2 + 10];
    cmr_bzero(otp_buffer, sizeof(otp_buffer));

    uint16_t flag1, flag2, flag3, start_address, wb_rg_golden, wb_bg_golden,
        wb_gg_golden, reg_offset, awb_address;
    uint32_t checksum, i, checksum_reg;

    /*OTP Initial Setting,read buffer*/
    Sensor_writereg8bits(handle, 0x0100, 0x01); // Streaming On

    usleep(50 * 1000);
    Sensor_writereg8bits(handle, 0x0A02, 0x15); // page 127
    Sensor_writereg8bits(handle, 0x0A00, 0x01); // Read command
    usleep(55 * 1000);
    start_address = 0x0A04;
#define reg_offset 0x0A04
    for (i = 0; i < 64; i++) {
        otp_buffer[i] = s5k4h7_Sensor_OTP_read(handle, start_address + i);
         //SENSOR_PRINT("read reg value %d:0x%x\n", i,otp_buffer[i]);
    }
    Sensor_writereg8bits(handle, 0x0A00, 0x00);

    //------------AWB Information------------
  
	
    /////////////////////* module information*///////////////////////
    /*flag check*/
    flag1 = otp_buffer[0x0A04 - reg_offset];
    otp_info->flag = flag1;
    //SENSOR_PRINT("module info flag1=0x%x,flag2=0x%x,flag3=0x%x\n", flag1, flag2, flag3);

	
    if (flag1 == 0x55) {
        start_address = 0x0A04 - reg_offset; // group1
        awb_address = 0x0A24 -reg_offset;
        checksum_reg = 0x0A0C - reg_offset;
    } else if (flag2 == 0x10) {
        start_address = 0x0A14 - reg_offset; // group2
        awb_address = 0x0A1A -reg_offset;
        checksum_reg = 0x0A22 - reg_offset;
    } else {
        start_address = 0x00;
        //SENSOR_PRINT("no module information in otp\n");
    }
	
    //SENSOR_PRINT("information start_address = 0x%02x\n", start_address);
   
	
    /*checksum*/
    checksum = 0;
    for (i = 0; i < 8; i++) {
        checksum = checksum + otp_buffer[start_address + i];
    }
     //SENSOR_PRINT("1111111111111111 checksum=0x%02x\n", checksum);
    checksum = (checksum % 0xff)+1;
    //SENSOR_PRINT("module information checksum=0x%02x\n", checksum);

    if (checksum == otp_buffer[checksum_reg]) {
        //SENSOR_PRINT("module information checksum success\n");
        //otp_info->flag |= 0x80;
    } else {
        //SENSOR_PRINT("module information checksum error\n");
        //otp_info->flag &= 0x7f;
    }
	
	
    /*read module information*/
    otp_info->flag = otp_buffer[0];
    otp_info->module_id = otp_buffer[start_address+1];
    otp_info->year = otp_buffer[start_address  + 5];
    otp_info->month = otp_buffer[start_address + 6];
    otp_info->day = otp_buffer[start_address   + 7];
    /////////////////////* module information end*/////////////////

	

    /////////////////////* awb information*///////////////////////
    /*read awb information*/
   
#if 1
    otp_info->rg_ratio_current =
        (otp_buffer[start_address + 22] << 8) + otp_buffer[start_address + 21];
    otp_info->bg_ratio_current =
        (otp_buffer[start_address + 24] << 8) + otp_buffer[start_address + 23];
    otp_info->gbgr_ratio_current =
        (otp_buffer[start_address + 26] << 8) + otp_buffer[start_address + 25];
    otp_info->rg_ratio_typical =
        (otp_buffer[start_address + 28] << 8) + otp_buffer[start_address + 27];
    otp_info->bg_ratio_typical =
        (otp_buffer[start_address + 30] << 8) + otp_buffer[start_address + 39];
    otp_info->gbgr_ratio_typical =
        (otp_buffer[start_address + 32] << 8) + otp_buffer[start_address + 31];
 #endif

    //SENSOR_PRINT("wb_rg_golden=0x%x", otp_info->rg_ratio_typical);
    //SENSOR_PRINT("wb_bg_golden=0x%x", otp_info->bg_ratio_typical);
    //SENSOR_PRINT("wb_gg_golden=0x%x", otp_info->gbgr_ratio_typical);

    /*print otp information*/
    //SENSOR_PRINT("flag=0x%x", otp_info->flag);
    //SENSOR_PRINT("module_id=0x%x", otp_info->module_id);

    //SENSOR_PRINT("data=%d-%d-%d", otp_info->year, otp_info->month,
     //            otp_info->day);
    //SENSOR_PRINT("rg_ratio_current=0x%x", otp_info->rg_ratio_current);
    //SENSOR_PRINT("bg_ratio_current=0x%x", otp_info->bg_ratio_current);
    //SENSOR_PRINT("gbgr_ratio_current=0x%x", otp_info->gbgr_ratio_current);
    //SENSOR_PRINT("rg_ratio_typical=0x%x", otp_info->rg_ratio_typical);
    //SENSOR_PRINT("bg_ratio_typical=0x%x", otp_info->bg_ratio_typical);
    //SENSOR_PRINT("gbgr_ratio_typical=0x%x", otp_info->gbgr_ratio_typical);

	//SENSOR_PRINT("******************************************\n");
    //SENSOR_PRINT("r_current=0x%x", otp_info->r_current);
    //SENSOR_PRINT("gr_current=0x%x", otp_info->gr_current);
    //SENSOR_PRINT("gb_current=0x%x", otp_info->gb_current);
    //SENSOR_PRINT("b_current=0x%x", otp_info->b_current);
    //SENSOR_PRINT("r_typical=0x%x", otp_info->r_typical);
    //SENSOR_PRINT("gr_typical=0x%x", otp_info->gr_typical);
    //SENSOR_PRINT("gb_typical=0x%x", otp_info->gb_typical);
    //SENSOR_PRINT("b_typical=0x%x", otp_info->b_typical);


    rtn = s5k4h7_sunwin_v800_update_awb(handle, param_ptr);
    if (rtn != SENSOR_SUCCESS) {
        //SENSOR_PRINT_ERR("OTP awb apply error!");
        return rtn;
     }
 

     /*update lsc*/

    rtn = s5k4h7_sunwin_v800_update_lsc(handle, param_ptr);
    if (rtn != SENSOR_SUCCESS) {
        //SENSOR_PRINT_ERR("OTP lsc apply error!");
        return rtn;
    }
  
    return rtn;
}

static uint32_t s5k4h7_sunwin_v800_identify_otp(cmr_handle handle, void *param_ptr
                                          ) {
    uint32_t rtn = SENSOR_SUCCESS;
    struct otp_info_t *otp_info = &s_s5k4h7_sunwin_v800_otp_info;   //(struct otp_info_t *)param_ptr;
    SENSOR_IC_CHECK_HANDLE(handle);
    //SENSOR_PRINT("cyw:Mesin enter\n");
    rtn = s5k4h7_sunwin_v800_read_otp_info(handle, param_ptr);
   // SENSOR_IC_CHECK_PTR(p_data);
    if (rtn != SENSOR_SUCCESS) {
        //SENSOR_PRINT_ERR("read otp information failed\n!");
        return rtn;
    } else {
        //SENSOR_PRINT("identify otp success mode_id = 0x%x !",
               //      otp_info->module_id);
    }

    return rtn;
}

// static struct raw_param_info_tab s_s5k4h7_sunwin_v800_raw_param_tab[] =
//{MODULE_ID_s5k4h7_sunwin_v800, &s_s5k4h7_mipi_raw_info, s5k4h7_sunwin_v800_identify_otp,
// s5k4h7_sunwin_v800_update_otp};
