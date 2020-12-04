/*
*    Header file with Custom Nextion #defines and object declarations 
*    To be included in main and also Nex_hardware.cpp so that page numbers can be 
*    determined on the fly in nex_listen function
*/

 //   uint8_t pg = 0;  // flag to track page changes to minimize writes to the sliders and other fields
    
    // Nextion Page ID, not the object ID
    #define page0_ID 0  // Main
    #define page1_ID 1  // Coupler - set up the attenuator values per band
    #define page2_ID 2  // Set1 
    #define page3_ID 3  // Power graphing page
    #define page4_ID 4  // PowerCal - calibrate ADC fwd and ref
    #define page5_ID 5  // Band Select Quick buttons

    // Nextion page 0 Main object IDs
    #define Main_ID 0
    #define fwdpwr_ID 8
    #define refpwr_ID 9
    #define swr_ID 10
    #define hv_ID 11
    #define v14_ID 12
    #define temp_ID 13
    #define curr_ID 14
    #define toConfig_ID 15
    #define FdBm_ID 16
    #define RdBm_ID 17
    #define band_ID 18
    #define Aux1_ID 19
    #define Aux2_ID 21
    #define PTT_CW_ID 23

    // Page 1 Coupler object IDs
    #define Coupler_ID 0
    #define savecfg_btn_1_ID 1
    #define fwd_cal_ID 2
    #define ref_cal_ID 3
    #define fwd_cal_num_ID 12
    #define ref_cal_num_ID 13
    #define f_att_minus_ID 7
    #define f_att_plus_ID 8
    #define r_att_minus_ID 10
    #define r_att_plus_ID 9
    #define FactoryRst1_ID 15
    #define FactoryRst2_ID 14
    #define band_cal_ID 16
    #define toSet1_ID 6

    // Page 2 Setpoints object IDs
    #define Set1_ID 0 
    #define hv_adj_ID 12
    #define hv_max_ID 2
    #define v14_adj_ID 15
    #define v14_max_ID 1
    #define curr_adj_ID 16
    #define curr_max_ID 3
    #define temp_adj_ID 17
    #define temp_max_ID 4
    #define meterID_adj_ID 19
    #define meterID_ID 5
    #define band_set_adj_ID 21
    #define band_set_ID 6
    #define toPwrGraph_ID 18
    // global set up in Set1 page
    #define Set1_Bandvar_ID 23    // Set1.Bandvar.txt
    #define savecfg_btn_2_ID 24
    
    // Page 3 PwrGraph object IDs
    #define PwrGraph_ID 0
    #define fPwrGraph_ID 1
    #define swrGraph_ID 15
    #define fPwr_scale_ID 6
    #define fPwrNum_ID 9
    #define rPwrNum_ID 10
    #define swrNum_ID 17  
    #define fscale_ID 7
    #define rscale_ID 8
    #define Graph_Timer_ID 19
    #define toPowerCal_ID 5
    
    // Page 4 PowerCal object IDs
    #define PowerCal_ID 0
    #define toMain_ID 1    
    #define Measure_hi_ID 2    
    #define Measure_lo_ID 4
    #define Units_ID 7
    #define savecfg_btn_4_ID 10
    #define HP_F_VDC_ID 11
    #define HP_R_VDC_ID 12
    #define LP_F_VDC_ID 13
    #define LP_R_VDC_ID 14
    #define HPPwrTarget_ID 24
    #define LPPwrTarget_ID 25    
    #define CalcFwd_ID 26
    #define CalcRef_ID 27
    
    // Page 5 BandSelect object IDs
    #define BandSelect_ID 0
    #define B_HF_ID 1
    #define B_50_ID 2
    #define B_144_ID 3
    #define B_222_ID 4
    #define B_432_ID 5
    #define B_902_ID 6
    #define B_1296_ID 7
    #define B_2_3G_ID 8
    #define B_3_4G_ID 9
    #define B_5_7G_ID 10
    #define B_10G_ID 11
    #define toMain1_ID 12
    #define BandSel_ID 14  // variable to pass along band from each button

    NexButton savecfg_btn_1 = NexButton(page1_ID, savecfg_btn_1_ID, "savecfg_btn_1");
    NexButton savecfg_btn_2 = NexButton(page2_ID, savecfg_btn_2_ID, "savecfg_btn_2");
    NexButton savecfg_btn_4 = NexButton(page4_ID, savecfg_btn_4_ID, "savecfg_btn_4");
    NexSlider fwd_cal = NexSlider(page1_ID, fwd_cal_ID, "fwd_cal");
    NexSlider ref_cal = NexSlider(page1_ID, ref_cal_ID, "ref_cal");
    NexText fwd_cal_num = NexText(page1_ID, fwd_cal_num_ID, "fwd_cal_num");  // slider
    NexText ref_cal_num = NexText(page1_ID, ref_cal_num_ID, "ref_cal_num");  // slider
    NexButton f_att_minus = NexButton(page1_ID, f_att_minus_ID, "f_att_minus");
    NexButton f_att_plus = NexButton(page1_ID, f_att_plus_ID, "f_att_plus");
    NexButton r_att_minus = NexButton(page1_ID, r_att_minus_ID, "r_att_minus");
    NexButton r_att_plus = NexButton(page1_ID, r_att_plus_ID, "r_att_plus");
    NexButton toConfig = NexButton(page0_ID, toConfig_ID, "toConfig");
    NexButton toSet1 = NexButton(page1_ID, toSet1_ID, "toSet1");
    NexButton toPwrGraph = NexButton(page2_ID, toPwrGraph_ID, "toPwrGraph");
    NexButton toPowerCal = NexButton(page3_ID, toPowerCal_ID, "toPowerCal");
    NexButton toMain = NexButton(page4_ID, toMain_ID, "toMain");
    NexPage Main = NexPage(page0_ID, Main_ID, "Main");
    NexPage Coupler = NexPage(page1_ID, Coupler_ID, "Coupler");
    NexPage Set1 = NexPage(page2_ID, Set1_ID, "Set1");
    NexPage PwrGraph = NexPage(page3_ID,PwrGraph_ID, "PwrGraph");
    NexPage PowerCal = NexPage(page4_ID,PowerCal_ID, "PowerCal");
    NexButton FactoryRst1 = NexButton(page1_ID, FactoryRst1_ID, "FactoryRst1");
    NexButton FactoryRst2 = NexButton(page1_ID, FactoryRst2_ID, "FactoryRst2");
    NexVariable Set1_Bandvar = NexVariable(page2_ID,Set1_Bandvar_ID,"Set1.Bandvar");
    NexSlider hv_adj= NexSlider(page2_ID,hv_adj_ID,"hv_adj");  // slider
    NexSlider v14_adj= NexSlider(page2_ID,v14_adj_ID,"v14_adj"); // slider
    NexSlider curr_adj= NexSlider(page2_ID,curr_adj_ID,"curr_adj"); // slider
    NexSlider temp_adj= NexSlider(page2_ID,temp_adj_ID,"temp_adj"); // slider
    NexSlider hv_max= NexSlider(page2_ID,hv_max_ID,"hv_max");  // slider
    NexSlider v14_max= NexSlider(page2_ID,v14_max_ID,"v14_max"); // slider
    NexSlider curr_max= NexSlider(page2_ID,curr_max_ID,"curr_max"); // slider
    NexSlider temp_max= NexSlider(page2_ID,temp_max_ID,"temp_max");// slider
    NexNumber meterID = NexNumber(page2_ID,meterID_ID,"meterID"); 
    NexSlider meterID_adj= NexSlider(page2_ID,meterID_adj_ID,"meterID_adj"); // slider
    NexText band_set = NexText(page2_ID,band_set_ID,"band_set"); 
    NexSlider band_set_adj= NexSlider(page2_ID,band_set_adj_ID,"band_set_adj"); // slider
    NexButton band = NexButton(page0_ID, band_ID, "band");
    NexText band_cal = NexText(page1_ID,band_cal_ID,"band_cal"); 
    NexWaveform fPwrGraph = NexWaveform(page3_ID,fPwrGraph_ID,"fPwrGraph"); 
    NexWaveform swrGraph = NexWaveform(page3_ID,swrGraph_ID,"swrGraph");
    NexButton fPwr_scale = NexButton(page3_ID,fPwr_scale_ID,"fPwr_scale"); 
    NexText fscale = NexText(page3_ID,fscale_ID,"fscale"); 
    NexText rscale = NexText(page3_ID,rscale_ID,"rscale"); 
    NexNumber fPwrNum = NexNumber(page3_ID,fPwrNum_ID,"fPwrNum"); 
    NexNumber rPwrNum = NexNumber(page3_ID,rPwrNum_ID,"rPwrNum"); 
    NexNumber swrNum = NexNumber(page3_ID,swrNum_ID,"swrNum");
    NexTimer Graph_Timer = NexTimer(page3_ID,Graph_Timer_ID,"Graph_Timer");
    NexNumber HPPwrTarget = NexNumber(page4_ID, HPPwrTarget_ID,"HPPwrTarget");
    NexNumber LPPwrTarget = NexNumber(page4_ID, LPPwrTarget_ID,"LPPwrTarget");
    NexNumber HP_F_VDC = NexNumber(page4_ID, HP_F_VDC_ID,"HP_F_VDC");
    NexNumber HP_R_VDC = NexNumber(page4_ID, HP_R_VDC_ID,"HP_R_VDC");
    NexNumber LP_F_VDC = NexNumber(page4_ID, LP_F_VDC_ID,"LP_F_VDC");
    NexNumber LP_R_VDC = NexNumber(page4_ID, LP_R_VDC_ID,"LP_R_VDC");
    NexButton Measure_hi = NexButton(page4_ID, Measure_hi_ID,"Measure_hi");
    NexButton Measure_lo = NexButton(page4_ID, Measure_lo_ID,"Measure_lo");
    NexDSButton Units = NexDSButton(page4_ID, Units_ID,"Units");
    NexButton CalcFwd = NexButton(page4_ID, CalcFwd_ID,"CalcFwd");
    NexButton CalcRef = NexButton(page4_ID, CalcRef_ID,"CalcRef");
    NexButton B_HF = NexButton(page5_ID, B_HF_ID, "B_HF");
    NexButton B_50 = NexButton(page5_ID, B_50_ID,"B_50");
    NexButton B_144 = NexButton(page5_ID, B_144_ID,"B_144");
    NexButton B_222 = NexButton(page5_ID, B_222_ID,"B_222");
    NexButton B_432 = NexButton(page5_ID, B_432_ID,"B_432");
    NexButton B_902 = NexButton(page5_ID, B_902_ID,"B_902");
    NexButton B_1296 = NexButton(page5_ID, B_1296_ID,"B_1296");
    NexButton B_2_3G = NexButton(page5_ID, B_2_3G_ID,"B_2_3G");
    NexButton B_3_4G = NexButton(page5_ID, B_3_4G_ID,"B_3_4G");
    NexButton B_5_7G = NexButton(page5_ID, B_5_7G_ID,"B_5_7G");
    NexButton B_10G = NexButton(page5_ID, B_10G_ID,"B_10G");
    NexText BandSel = NexText(page5_ID, BandSel_ID,"BandSel");
    NexButton toMain1 = NexButton(page5_ID, toMain1_ID, "toMain1");
    NexNumber Aux1 = NexNumber(page0_ID, Aux1_ID,"Aux1");
    NexNumber Aux2 = NexNumber(page0_ID, Aux2_ID,"Aux2");
    NexText PTT_CW = NexText(page0_ID, PTT_CW_ID,"PTT_CW");
                        
    NexTouch *nex_listen_list[] = {
        &savecfg_btn_1, &fwd_cal, &ref_cal, &f_att_minus, &f_att_plus, &r_att_minus, &r_att_plus, &band, &toMain, &toMain1, \
        &toConfig, &toSet1, &toPwrGraph, &FactoryRst1, &FactoryRst2, &Main, &Coupler, &Set1, \
        &hv_adj, &v14_adj, &temp_adj, &band_set_adj, &meterID_adj, &Set1_Bandvar, &curr_adj, \
        &savecfg_btn_2, &savecfg_btn_4, &Measure_hi, &Measure_lo, &CalcFwd, &CalcRef, \
        &B_HF, &B_50, &B_144, &B_222, &B_432, &B_902, &B_1296, &B_2_3G, &B_3_4G, &B_5_7G, &B_10G, NULL};
  
    uint32_t value;
