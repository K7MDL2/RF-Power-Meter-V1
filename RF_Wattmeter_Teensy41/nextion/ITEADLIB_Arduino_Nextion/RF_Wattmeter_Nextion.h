/*
*    Header file with Custom Nextion #defines and object declarations 
*    To be included in main and also Nex_hardware.cpp so that page numbers can be 
*    determined on the fly in nex_listen function


_____________________________________________________________________________________________________________
12/2020 K7MDL

This is the standard Arduino Nextion library with a few changes to suit the RF Wattmeter program usage.

1. Unused variable warnings and missing return statement warnings resolved.

2. The main change is in NexHardware.cpp where I added some lines in the nex_listen function to extract the page number
on the fly. When  touch event arrives it looks to see if it is from one of the lower right corner navigation buttons 
so it is dependent on knowing the object ID.  #include RF_Wattmeter_Nextion.h for the #define to help.  
If you change the Nav buttons you will need to update this also.  This was done to minimize the need the chance of a
missed page change event and reduce the query traffic to ask before every screen update.  Writing data to an inactive
page results in error messages and clogs the serial line with traffic.  This became important when running the Nextion
display directly over a LoRa transvert wireless link at 9600bps Air speed. Excess traffic causes long delays and missed
responses.

_____________________________________________________________________________________________________________
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

   