/*versionid=0x00070005*/
/*maxGain=0.00*/
/*param0.&BasePoint=1&*/
/*v21_sensor_nlm_level*/
{
    /*first_lum*/
    {
        /*nlm_flat_opt_bypass*/
        0x00,
            /*flat_opt_mode*/
            0x00,
            /*first_lum_bypass*/
            0x00,
            /*reserved*/
            0x00,
            /*lum_thr0*/
            0x00C8,
            /*lum_thr1*/
            0x01F4,
        /*nlm_lum*/
        {
            /*[0x0]*/
            {
                /*nlm_flat*/
                {
                    /*[0x0]*/
                    {
                        /*flat_inc_str*/
                        0x19,
                            /*flat_match_cnt*/
                            0x15,
                            /*flat_thresh*/
                            0x006E,
                            /*addback0*/
                            0x0058,
                            /*addback1*/
                            0x0058,
                            /*addback_clip_max*/
                            0x0008,
                            /*addback_clip_min*/
                            0xFFF8,
                    }
                    ,
                        /*[0x1]*/
                        {
                            /*flat_inc_str*/
                            0x0F,
                            /*flat_match_cnt*/
                            0x15,
                            /*flat_thresh*/
                            0x008C,
                            /*addback0*/
                            0x0058,
                            /*addback1*/
                            0x0058,
                            /*addback_clip_max*/
                            0x000A,
                            /*addback_clip_min*/
                            0xFFF6,

                        },
                    /*[0x2]*/
                    {
                        /*flat_inc_str*/
                        0x00,
                            /*flat_match_cnt*/
                            0x10,
                            /*flat_thresh*/
                            0x00C8,
                            /*addback0*/
                            0x0058,
                            /*addback1*/
                            0x0058,
                            /*addback_clip_max*/
                            0x000A,
                            /*addback_clip_min*/
                            0xFFF6,
                    }
                }
                ,
                /*nlm_texture*/
                {
                    /*texture_dec_str*/
                    0x3F,
                        /*addback30*/
                        0x40,
                        /*addback31*/
                        0x40,
                        /*reserved*/
                        0x00,
                        /*addback_clip_max*/
                        0x000C,
                        /*addback_clip_min*/
                        0xFFF4,
                }
            }
            ,
                /*[0x1]*/
                { /*nlm_flat*/
                 {/*[0x0]*/
                  {
                      /*flat_inc_str*/
                      0x14,
                      /*flat_match_cnt*/
                      0x15,
                      /*flat_thresh*/
                      0x006E,
                      /*addback0*/
                      0x0058,
                      /*addback1*/
                      0x0058,
                      /*addback_clip_max*/
                      0x0008,
                      /*addback_clip_min*/
                      0xFFF8,

                  },
                  /*[0x1]*/
                  {
                      /*flat_inc_str*/
                      0x0C,
                      /*flat_match_cnt*/
                      0x15,
                      /*flat_thresh*/
                      0x00BE,
                      /*addback0*/
                      0x0058,
                      /*addback1*/
                      0x0058,
                      /*addback_clip_max*/
                      0x000A,
                      /*addback_clip_min*/
                      0xFFF6,

                  },
                  /*[0x2]*/
                  {
                      /*flat_inc_str*/
                      0xFB,
                      /*flat_match_cnt*/
                      0x10,
                      /*flat_thresh*/
                      0x00F0,
                      /*addback0*/
                      0x0058,
                      /*addback1*/
                      0x0058,
                      /*addback_clip_max*/
                      0x000A,
                      /*addback_clip_min*/
                      0xFFF6,

                  }},
                 /*nlm_texture*/
                 {
                     /*texture_dec_str*/
                     0x3F,
                     /*addback30*/
                     0x40,
                     /*addback31*/
                     0x40,
                     /*reserved*/
                     0x00,
                     /*addback_clip_max*/
                     0x000A,
                     /*addback_clip_min*/
                     0xFFF6,

                 }},
            /*[0x2]*/
            {
                /*nlm_flat*/
                {
                    /*[0x0]*/
                    {
                        /*flat_inc_str*/
                        0x0A,
                            /*flat_match_cnt*/
                            0x15,
                            /*flat_thresh*/
                            0x008C,
                            /*addback0*/
                            0x0058,
                            /*addback1*/
                            0x0058,
                            /*addback_clip_max*/
                            0x000A,
                            /*addback_clip_min*/
                            0xFFF6,
                    }
                    ,
                        /*[0x1]*/
                        {
                            /*flat_inc_str*/
                            0xF6,
                            /*flat_match_cnt*/
                            0x15,
                            /*flat_thresh*/
                            0x00B4,
                            /*addback0*/
                            0x0058,
                            /*addback1*/
                            0x0058,
                            /*addback_clip_max*/
                            0x000C,
                            /*addback_clip_min*/
                            0xFFF4,

                        },
                    /*[0x2]*/
                    {
                        /*flat_inc_str*/
                        0xE0,
                            /*flat_match_cnt*/
                            0x10,
                            /*flat_thresh*/
                            0x00F0,
                            /*addback0*/
                            0x0058,
                            /*addback1*/
                            0x0058,
                            /*addback_clip_max*/
                            0x000C,
                            /*addback_clip_min*/
                            0xFFF4,
                    }
                }
                ,
                /*nlm_texture*/
                {
                    /*texture_dec_str*/
                    0x3F,
                        /*addback30*/
                        0x40,
                        /*addback31*/
                        0x40,
                        /*reserved*/
                        0x00,
                        /*addback_clip_max*/
                        0x000C,
                        /*addback_clip_min*/
                        0xFFF4,
                }
            }
        }
    }
    ,
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x01,
            /*simple_bpc_thr*/
            0x0C,
            /*simple_bpc_lum_thr*/
            0x00C8,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003CF, 0x0000034D, 0x0000029A, 0x000001DC,
             0x00000136, 0x000000B7, 0x00000062, 0x00000030, 0x00000015,
             0x00000009, 0x00000003, 0x00000001, 0x00000000, 0x00000000,
             0x00000000, /*0-15*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
}
,
    /*param1.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x1C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0082,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x12,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00B4,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x08,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x00A0,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x18,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00E6,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x11,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0118,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x06,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0104,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x0E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x010E,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0xF0,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0140,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0E,
            /*simple_bpc_lum_thr*/
            0x00C8,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003DF, 0x00000385, 0x00000301, 0x00000267,
             0x000001CE, 0x00000146, 0x000000D8, 0x00000086, 0x0000004E,
             0x0000002B, 0x00000016, 0x0000000B, 0x00000005, 0x00000002,
             0x00000001, /*0-15*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param2.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x1E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0096,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00C8,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0C,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x00B4,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x1C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0118,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0122,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x06,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0xF8,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0D,
            /*simple_bpc_lum_thr*/
            0x00C8,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003EC, 0x000003B4, 0x0000035E, 0x000002F3,
             0x0000027C, 0x00000204, 0x00000193, 0x0000012F, 0x000000DB,
             0x00000099, 0x00000067, 0x00000042, 0x00000029, 0x00000019,
             0x0000000E, /*0-15*/
             0x00000008, 0x00000004, 0x00000002, 0x00000001, 0x00000001,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param3.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x1F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00A0,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00D2,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0C,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x00BE,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x1E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0104,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x16,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0D,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0122,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x12,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x08,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0172,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0xFC,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0C,
            /*simple_bpc_lum_thr*/
            0x00C8,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F1, 0x000003C9, 0x0000038A, 0x00000338,
             0x000002D9, 0x00000274, 0x0000020F, 0x000001AE, 0x00000155,
             0x00000108, 0x000000C7, 0x00000091, 0x00000068, 0x00000048,
             0x00000031, /*0-15*/
             0x00000020, 0x00000014, 0x0000000D, 0x00000008, 0x00000005,
             0x00000003, 0x00000001, 0x00000001, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param4.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00AA,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x18,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00DC,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x00C8,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x010E,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x18,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0140,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0C,
            /*simple_bpc_lum_thr*/
            0x00C8,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F4, 0x000003D5, 0x000003A3, 0x00000361,
             0x00000313, 0x000002BD, 0x00000263, 0x0000020A, 0x000001B5,
             0x00000166, 0x0000011F, 0x000000E1, 0x000000AD, 0x00000082,
             0x00000060, /*0-15*/
             0x00000045, 0x00000031, 0x00000022, 0x00000017, 0x0000000F,
             0x0000000A, 0x00000006, 0x00000004, 0x00000002, 0x00000001,
             0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param5.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x21,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00BE,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x19,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00F0,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x00DC,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0122,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x18,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0140,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0C,
            /*simple_bpc_lum_thr*/
            0x00DC,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B3, 0x0000037C,
             0x00000339, 0x000002EF, 0x000002A0, 0x0000024F, 0x000001FE,
             0x000001B2, 0x0000016A, 0x00000129, 0x000000F0, 0x000000BE,
             0x00000094, /*0-15*/
             0x00000072, 0x00000056, 0x0000003F, 0x0000002E, 0x00000021,
             0x00000017, 0x00000010, 0x0000000B, 0x00000007, 0x00000005,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param6.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x22,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00D2,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0104,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x00F0,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x21,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x19,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01A4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0B,
            /*simple_bpc_lum_thr*/
            0x00F0,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F8, 0x000003E2, 0x000003BE, 0x0000038F,
             0x00000355, 0x00000314, 0x000002CD, 0x00000283, 0x00000239,
             0x000001EF, 0x000001A9, 0x00000168, 0x0000012C, 0x000000F7,
             0x000000C8, /*0-15*/
             0x000000A0, 0x0000007E, 0x00000062, 0x0000004B, 0x00000038,
             0x0000002A, 0x0000001F, 0x00000016, 0x00000010, 0x0000000B,
             0x00000008, 0x00000005, 0x00000003, 0x00000002, 0x00000001,
             0x00000001, /*16-31*/
             0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param7.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x23,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00E6,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1B,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0122,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0104,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x21,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x19,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000C,
               /*addback_clip_min*/
               0xFFF4,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0172,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01B8,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01A4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0A,
            /*simple_bpc_lum_thr*/
            0x0104,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F9, 0x000003E6, 0x000003C7, 0x0000039D,
             0x0000036A, 0x00000330, 0x000002F0, 0x000002AD, 0x00000267,
             0x00000222, 0x000001DF, 0x0000019E, 0x00000162, 0x0000012B,
             0x000000F9, /*0-15*/
             0x000000CD, 0x000000A7, 0x00000086, 0x0000006A, 0x00000053,
             0x00000040, 0x00000031, 0x00000025, 0x0000001C, 0x00000014,
             0x0000000F, 0x0000000B, 0x00000007, 0x00000005, 0x00000004,
             0x00000002, /*16-31*/
             0x00000002, 0x00000001, 0x00000001, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param8.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x24,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00E6,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x22,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0118,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x019A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x0A,
            /*simple_bpc_lum_thr*/
            0x0118,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F9, 0x000003E9, 0x000003CD, 0x000003A8,
             0x0000037B, 0x00000346, 0x0000030C, 0x000002CE, 0x0000028E,
             0x0000024D, 0x0000020C, 0x000001CD, 0x00000192, 0x0000015A,
             0x00000127, /*0-15*/
             0x000000F8, 0x000000CF, 0x000000AB, 0x0000008B, 0x00000070,
             0x00000059, 0x00000046, 0x00000037, 0x0000002A, 0x00000020,
             0x00000018, 0x00000012, 0x0000000D, 0x0000000A, 0x00000007,
             0x00000005, /*16-31*/
             0x00000004, 0x00000002, 0x00000002, 0x00000001, 0x00000001,
             0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param9.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x25,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1D,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x23,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1B,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x09,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FA, 0x000003EB, 0x000003D3, 0x000003B1,
             0x00000388, 0x00000358, 0x00000323, 0x000002EA, 0x000002AE,
             0x00000270, 0x00000233, 0x000001F6, 0x000001BC, 0x00000185,
             0x00000151, /*0-15*/
             0x00000121, 0x000000F5, 0x000000CE, 0x000000AC, 0x0000008E,
             0x00000074, 0x0000005E, 0x0000004B, 0x0000003B, 0x0000002F,
             0x00000024, 0x0000001C, 0x00000015, 0x00000010, 0x0000000C,
             0x00000009, /*16-31*/
             0x00000007, 0x00000005, 0x00000003, 0x00000002, 0x00000002,
             0x00000001, 0x00000001, 0x00000001, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param10.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x26,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0104,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x24,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0136,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x019A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01B8,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0186,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0186,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FA, 0x000003ED, 0x000003D7, 0x000003B8,
             0x00000393, 0x00000367, 0x00000336, 0x00000301, 0x000002C9,
             0x0000028F, 0x00000254, 0x0000021A, 0x000001E1, 0x000001AB,
             0x00000177, /*0-15*/
             0x00000146, 0x0000011A, 0x000000F1, 0x000000CC, 0x000000AC,
             0x0000008F, 0x00000076, 0x00000061, 0x0000004E, 0x0000003F,
             0x00000032, 0x00000028, 0x0000001F, 0x00000018, 0x00000012,
             0x0000000E, /*16-31*/
             0x0000000B, 0x00000008, 0x00000006, 0x00000004, 0x00000003,
             0x00000002, 0x00000002, 0x00000001, 0x00000001, 0x00000001,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param11.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x28,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0118,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x12,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0154,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x30,
               /*addback30*/
               0x32,
               /*addback31*/
               0x32,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x25,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x019A,
                /*addback0*/
                0x0038,
                /*addback1*/
                0x0038,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x30,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x12,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0168,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x09,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01CC,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000C,
                /*addback_clip_min*/
                0xFFF4,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x30,
               /*addback30*/
               0x50,
               /*addback31*/
               0x50,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x0050,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FB, 0x000003EE, 0x000003DA, 0x000003BF,
             0x0000039C, 0x00000374, 0x00000346, 0x00000314, 0x000002E0,
             0x000002A9, 0x00000271, 0x00000239, 0x00000202, 0x000001CD,
             0x00000199, /*0-15*/
             0x00000169, 0x0000013C, 0x00000112, 0x000000EB, 0x000000C9,
             0x000000AA, 0x0000008F, 0x00000077, 0x00000062, 0x00000050,
             0x00000041, 0x00000035, 0x0000002A, 0x00000021, 0x0000001A,
             0x00000014, /*16-31*/
             0x00000010, 0x0000000C, 0x00000009, 0x00000007, 0x00000005,
             0x00000004, 0x00000003, 0x00000002, 0x00000002, 0x00000001,
             0x00000001, 0x00000001, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param12.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x2A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x22,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0004,
                /*addback_clip_min*/
                0xFFFC,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x14,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x20,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x26,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1D,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01C2,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0F,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x20,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x10,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x08,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01F4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01E0,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x24,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x00C8,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x07,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FB, 0x000003F0, 0x000003DD, 0x000003C4,
             0x000003A4, 0x0000037E, 0x00000354, 0x00000325, 0x000002F4,
             0x000002C0, 0x0000028B, 0x00000255, 0x00000220, 0x000001EC,
             0x000001B9, /*0-15*/
             0x00000189, 0x0000015B, 0x00000131, 0x00000109, 0x000000E5,
             0x000000C5, 0x000000A7, 0x0000008E, 0x00000077, 0x00000063,
             0x00000052, 0x00000043, 0x00000037, 0x0000002C, 0x00000023,
             0x0000001C, /*16-31*/
             0x00000016, 0x00000011, 0x0000000E, 0x0000000A, 0x00000008,
             0x00000006, 0x00000005, 0x00000003, 0x00000003, 0x00000002,
             0x00000001, 0x00000001, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param13.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x2C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x014A,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x24,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0004,
                /*addback_clip_min*/
                0xFFFC,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x16,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x019A,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x10,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x27,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x017C,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1D,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01E0,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0E,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01CC,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x14,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x0E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x07,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0212,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01FE,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x1E,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x00C8,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x06,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FB, 0x000003F1, 0x000003E0, 0x000003C8,
             0x000003AA, 0x00000387, 0x00000360, 0x00000334, 0x00000305,
             0x000002D4, 0x000002A1, 0x0000026E, 0x0000023A, 0x00000207,
             0x000001D6, /*0-15*/
             0x000001A6, 0x00000179, 0x0000014E, 0x00000126, 0x00000101,
             0x000000DF, 0x000000C0, 0x000000A4, 0x0000008C, 0x00000076,
             0x00000063, 0x00000052, 0x00000044, 0x00000038, 0x0000002E,
             0x00000025, /*16-31*/
             0x0000001E, 0x00000018, 0x00000013, 0x0000000F, 0x0000000C,
             0x00000009, 0x00000007, 0x00000005, 0x00000004, 0x00000003,
             0x00000002, 0x00000002, 0x00000001, 0x00000001, 0x00000001,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param14.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x2E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x26,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01C2,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0004,
                /*addback_clip_min*/
                0xFFFC,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x18,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01AE,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x28,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01F4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0D,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01E0,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x0F,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x0C,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01C2,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x06,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0226,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0212,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x19,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x00C8,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x05,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FC, 0x000003F2, 0x000003E2, 0x000003CC,
             0x000003B0, 0x0000038F, 0x0000036A, 0x00000341, 0x00000314,
             0x000002E6, 0x000002B5, 0x00000284, 0x00000252, 0x00000221,
             0x000001F0, /*0-15*/
             0x000001C1, 0x00000194, 0x00000169, 0x00000140, 0x0000011B,
             0x000000F8, 0x000000D8, 0x000000BB, 0x000000A0, 0x00000089,
             0x00000074, 0x00000062, 0x00000052, 0x00000044, 0x00000039,
             0x0000002F, /*16-31*/
             0x00000026, 0x0000001F, 0x00000019, 0x00000014, 0x00000010,
             0x0000000D, 0x0000000A, 0x00000008, 0x00000006, 0x00000005,
             0x00000004, 0x00000003, 0x00000002, 0x00000002, 0x00000001,
             0x00000001, /*32-47*/
             0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param15.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0172,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x28,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01D6,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0004,
                /*addback_clip_min*/
                0xFFFC,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x19,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01C2,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0xF6,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x28,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01A4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x1E,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0208,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x0C,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x01F4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x0A,
               /*addback30*/
               0x30,
               /*addback31*/
               0x30,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01D6,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x05,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x023A,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0226,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x003E,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x14,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x00C8,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x04,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FC, 0x000003F3, 0x000003E5, 0x000003D0,
             0x000003B7, 0x00000399, 0x00000377, 0x00000351, 0x00000328,
             0x000002FD, 0x000002CF, 0x000002A1, 0x00000272, 0x00000242,
             0x00000214, /*0-15*/
             0x000001E6, 0x000001B9, 0x0000018F, 0x00000166, 0x0000013F,
             0x0000011C, 0x000000FA, 0x000000DB, 0x000000BF, 0x000000A6,
             0x0000008F, 0x0000007B, 0x00000069, 0x00000059, 0x0000004B,
             0x0000003E, /*16-31*/
             0x00000034, 0x0000002B, 0x00000023, 0x0000001D, 0x00000018,
             0x00000013, 0x0000000F, 0x0000000C, 0x0000000A, 0x00000008,
             0x00000006, 0x00000005, 0x00000004, 0x00000003, 0x00000002,
             0x00000002, /*32-47*/
             0x00000001, 0x00000001, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x01,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param16.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x00,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x00C8,
         /*lum_thr1*/
         0x01F4,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0190,
                /*addback0*/
                0x0012,
                /*addback1*/
                0x0012,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01F4,
                /*addback0*/
                0x0014,
                /*addback1*/
                0x0014,
                /*addback_clip_max*/
                0x0004,
                /*addback_clip_min*/
                0xFFFC,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0258,
                /*addback0*/
                0x001A,
                /*addback1*/
                0x001A,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0xEC,
               /*addback30*/
               0x1A,
               /*addback31*/
               0x1A,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x36,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01C2,
                /*addback0*/
                0x0016,
                /*addback1*/
                0x0016,
                /*addback_clip_max*/
                0x0008,
                /*addback_clip_min*/
                0xFFF8,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x28,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0226,
                /*addback0*/
                0x001C,
                /*addback1*/
                0x0016,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x18,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0212,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0xF6,
               /*addback30*/
               0x24,
               /*addback31*/
               0x24,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0008,
               /*addback_clip_min*/
               0xFFF8,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x0A,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x01F4,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x0016,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x05,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x0258,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x0016,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x10,
                /*flat_thresh*/
                0x0244,
                /*addback0*/
                0x003E,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x000A,
                /*addback_clip_min*/
                0xFFF6,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x14,
               /*addback30*/
               0x3C,
               /*addback31*/
               0x3C,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x000A,
               /*addback_clip_min*/
               0xFFF6,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x00C8,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x02,
            /*simple_bpc_lum_thr*/
            0x012C,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003FC, 0x000003F4, 0x000003E6, 0x000003D3,
             0x000003BB, 0x0000039F, 0x0000037F, 0x0000035B, 0x00000334,
             0x0000030A, 0x000002DF, 0x000002B2, 0x00000284, 0x00000256,
             0x00000229, /*0-15*/
             0x000001FC, 0x000001D0, 0x000001A6, 0x0000017D, 0x00000156,
             0x00000132, 0x00000110, 0x000000F1, 0x000000D4, 0x000000B9,
             0x000000A1, 0x0000008B, 0x00000078, 0x00000066, 0x00000057,
             0x0000004A, /*16-31*/
             0x0000003E, 0x00000034, 0x0000002B, 0x00000024, 0x0000001E,
             0x00000018, 0x00000014, 0x00000010, 0x0000000D, 0x0000000A,
             0x00000008, 0x00000007, 0x00000005, 0x00000004, 0x00000003,
             0x00000002, /*32-47*/
             0x00000002, 0x00000001, 0x00000001, 0x00000001, 0x00000001,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param17.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param18.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param19.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param20.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param21.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param22.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param23.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
    /*param24.&BasePoint=1&*/
    /*v21_sensor_nlm_level*/
    {
        /*first_lum*/
        {/*nlm_flat_opt_bypass*/
         0x00,
         /*flat_opt_mode*/
         0x00,
         /*first_lum_bypass*/
         0x01,
         /*reserved*/
         0x00,
         /*lum_thr0*/
         0x0000,
         /*lum_thr1*/
         0x0000,
         /*nlm_lum*/
         {  /*[0x0]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x3F,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x00FA,
                /*addback0*/
                0x0020,
                /*addback1*/
                0x0020,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x30,
                /*flat_match_cnt*/
                0x15,
                /*flat_thresh*/
                0x015E,
                /*addback0*/
                0x0028,
                /*addback1*/
                0x0028,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x20,
                /*flat_match_cnt*/
                0x12,
                /*flat_thresh*/
                0x012C,
                /*addback0*/
                0x003F,
                /*addback1*/
                0x003F,
                /*addback_clip_max*/
                0x03FF,
                /*addback_clip_min*/
                0xFC00,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x3F,
               /*addback30*/
               0x3F,
               /*addback31*/
               0x3F,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x03FF,
               /*addback_clip_min*/
               0xFC00,

           }},
          /*[0x1]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }},
          /*[0x2]*/
          { /*nlm_flat*/
           {/*[0x0]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x1]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            },
            /*[0x2]*/
            {
                /*flat_inc_str*/
                0x00,
                /*flat_match_cnt*/
                0x00,
                /*flat_thresh*/
                0x0000,
                /*addback0*/
                0x0000,
                /*addback1*/
                0x0000,
                /*addback_clip_max*/
                0x0000,
                /*addback_clip_min*/
                0x0000,

            }},
           /*nlm_texture*/
           {
               /*texture_dec_str*/
               0x00,
               /*addback30*/
               0x00,
               /*addback31*/
               0x00,
               /*reserved*/
               0x00,
               /*addback_clip_max*/
               0x0000,
               /*addback_clip_min*/
               0x0000,

           }}}},
        /*nlm_dic*/
        {
            /*direction_mode_bypass*/
            0x00,
            /*dist_mode*/
            0x02,
            /*w_shift*/
            {
                0x02, 0x02, 0x03 /*0-2*/
            },
            /*cnt_th*/
            0x02,
            /*reserved*/
            {
                0x00, 0x00 /*0-1*/
            },
            /*diff_th*/
            0x003C,
            /*tdist_min_th*/
            0x008C,

        },
        /*simple_bpc*/
        {
            /*simple_bpc_bypass*/
            0x00,
            /*simple_bpc_thr*/
            0x08,
            /*simple_bpc_lum_thr*/
            0x0000,

        },
        /*lut_w*/
        {/*lut_w*/
         {
             0x000003FF, 0x000003F6, 0x000003DC, 0x000003B1, 0x00000378,
             0x00000335, 0x000002E9, 0x00000298, 0x00000246, 0x000001F5,
             0x000001A8, 0x00000160, 0x0000011F, 0x000000E6, 0x000000B6,
             0x0000008D, /*0-15*/
             0x0000006B, 0x00000050, 0x0000003B, 0x0000002A, 0x0000001E,
             0x00000015, 0x0000000E, 0x0000000A, 0x00000006, 0x00000004,
             0x00000003, 0x00000002, 0x00000001, 0x00000001, 0x00000000,
             0x00000000, /*16-31*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*32-47*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, /*48-63*/
             0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000 /*64-71*/
         }},
        /*nlm_den_strenth*/
        0x00,
        /*imp_opt_bypass*/
        0x00,
        /*vst_bypass*/
        0x00,
        /*nlm_bypass*/
        0x00,
    },
