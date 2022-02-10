/*versionid=0x00070005*/
/*maxGain=0.00*/
/*param0.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x004D,0x00E0,0x0200,0x03FF,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x002C,0x0018,0x0010,0x000B,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x000B,0x0011,0x0018,0x0022,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x00C8,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0005,
			/*max*/
			0x0005,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x001E,
		/*rd_ration*/
		0x0028,

	},
	/*bypass*/
	0x00000000,
}
,
/*param1.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x00,
		/*rd_mode*/
		0x01,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0001,0x000F,0x004F,0x00EB,0x022B,0x03FF,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x030A,0x00A0,0x003D,0x0020,0x0014,0x000E,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0003,0x0009,0x0012,0x001D,0x002A,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x0000,0x0002,0x0004,0x0006/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0258,
		/*texture_th*/
		0x012C,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0002,
			/*max*/
			0x0002,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x002D,

	},
	/*bypass*/
	0x00000000,
}
,
/*param2.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x00,
		/*rd_mode*/
		0x01,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0001,0x000F,0x004F,0x00EB,0x022B,0x03FF,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x030A,0x00A0,0x003D,0x0020,0x0014,0x000E,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0003,0x0009,0x0012,0x001D,0x002A,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x0000,0x0001,0x0002,0x0003/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x012C,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0000,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x00,
		/*lowoffset*/
		0x00,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x002D,

	},
	/*bypass*/
	0x00000000,
}
,
/*param3.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x00,
		/*rd_mode*/
		0x01,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0001,0x000F,0x004F,0x00EB,0x022B,0x03FF,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x030A,0x00A0,0x003D,0x0020,0x0014,0x000E,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0003,0x0009,0x0012,0x001D,0x002A,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x0000,0x001E,0x003C,0x0078/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0096,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0000,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x002D,

	},
	/*bypass*/
	0x00000000,
}
,
/*param4.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param5.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param6.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param7.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param8.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param9.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param10.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param11.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param12.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param13.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param14.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param15.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param16.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param17.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param18.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param19.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param20.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param21.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param22.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param23.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
/*param24.&BasePoint=1&*/
/*v21_sensor_bpc_level*/
{
	/*bpc_comm*/
	{
		/*bpc_mode*/
		0x00,
		/*hv_mode*/
		0x01,
		/*rd_mode*/
		0x00,
		/*reserved*/
		0x00,
		/*lut_level*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*slope_k*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*intercept_b*/
		{
			0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000/*0-7*/
		},
		/*dtol*/
		0.20,

	},
	/*bpc_pos*/
	{
		/*pos_out_en*/
		0x00000000,

	},
	/*bpc_thr*/
	{
		/*double_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*three_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*four_th*/
		{
			0x03FF,0x03FF,0x03FF,0x03FF/*0-3*/
		},
		/*shift*/
		{
			0x07,0x08,0x09/*0-2*/
		},
		/*reserved*/
		0x00,
		/*flat_th*/
		0x0190,
		/*texture_th*/
		0x0078,

	},
	/*bpc_rules*/
	{
		/*k_val*/
		{
			/*min*/
			0x0001,
			/*max*/
			0x0000,

		},
		/*lowcoeff*/
		0x02,
		/*lowoffset*/
		0x02,
		/*highcoeff*/
		0x00,
		/*highoffset*/
		0x00,
		/*hv_ratio*/
		0x0028,
		/*rd_ration*/
		0x001E,

	},
	/*bypass*/
	0x00000000,
}
,
