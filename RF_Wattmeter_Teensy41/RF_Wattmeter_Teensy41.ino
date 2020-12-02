#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h> 
#include "RF_Wattmeter_Arduino.h"

/*
 *
 * RF Power Meter by K7MDL 12/1/2020   - Remote (Headless) Edition for Testing on Arduino Teensy 4.1
 *
 * 12/1/2020 -  Ported from PSoC5. Nextion is mostly working, OLED and band decoder features note working yet.  Lots of 
 *  warnings to sift through, bugs to be found, lots more testing to do.
 *
*/

//struct Band_Cal Band_Cal_Table[10];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#define RESET_EEPROM 0

void setup(void) 
{
  pinMode(13,OUTPUT); 
#ifdef SSD1306_OLED
    //I2COLED_Start(); 
    display_init(DISPLAY_ADDRESS); // This line will initialize your display using the address you specified before.
    //gfx_setRotation(1);
    display_clear();    
    display_update();
#endif
    SerialUSB1.begin(9600);  // open port for OTRSP serial port command input
    Serial.begin(115200); // For debug or data output
    Serial.println(" ");   // Clear our output text from CPU init text
    write_Cal_Table_from_Default();  // Copy default values into memory in case EEPROM not yet initialized
      /*   initialize EEPROM storage if not done before - will overwrite default memory table values from EEPROM if EEPROM was written to before */    

  if (EEPROM.read(0) == 'G') 
  {
    Serial.println("EEPROM Data is Valid");
    EEPROM_Init_Read();
    //get_config_EEPROM();  // set last used values for CouplerSetNum (Band) and op_mode if there is data in the EEPROM
  }
 
  if (EEPROM.read(0) != 'G' || RESET_EEPROM == 1) 
  {    // Test if EEPROM has been initialized with table data yet
    Serial.println("Write EEPROM");
    write_Cal_Table_from_Default();  // Copy default values into memory
    write_Cal_Table_to_EEPROM(); // Copy memory into EEPROM if EEPROM is not initialized yet. Byte 0 will get marked with a 'G'
    EE_Save_State();   // use default values to populate state storage area in EEPROM (first 16 bytes reserved for state variables)
    ser_data_out = 1;
    toggle_ser_data_output(1);   // force set data output on
    // Read EEPROM
    //EEPROM_Init_Read();  // read stored state cal data from EEPROM into memory
  }  // end initilization write calls  

  Cal_Table();   // Load current Band values from Table
 
  updateTime = millis(); // Next update time
  // initialize all the readings to 0:  Used to smooth AD values
  for (int thisReading = 0; thisReading < NUMREADINGS; thisReading++) {
    readings_Fwd[thisReading] = 0;
    readings_Ref[thisReading] = 0;
  }
  //delay(1000);
  // ensure the band index is in proper range in case of bad input or memory operation
  if (CouplerSetNum < 0)   // constrain newBand
      CouplerSetNum = 0;   // something wrong if you end up here
  if (CouplerSetNum >= NUM_SETS)
      CouplerSetNum = NUM_SETS-1; // something wrong if you end up here
  NewBand = CouplerSetNum;

  #ifdef NEXTION
    nexInit();
    // Set up objects to monitor touch controls from Nextion Display
    CreateNexObject(savecfg_btn_1, page1_ID, savecfg_btn_1_ID, "savecfg_btn_1");
    CreateNexObject(savecfg_btn_2, page2_ID, savecfg_btn_2_ID, "savecfg_btn_2");
    CreateNexObject(savecfg_btn_4, page4_ID, savecfg_btn_4_ID, "savecfg_btn_4");
    CreateNexObject(fwd_cal, page1_ID, fwd_cal_ID, "fwd_cal");
    CreateNexObject(ref_cal, page1_ID, ref_cal_ID, "ref_cal");
    CreateNexObject(fwd_cal_num, page1_ID, fwd_cal_num_ID, "fwd_cal_num");  // slider
    CreateNexObject(ref_cal_num, page1_ID, ref_cal_num_ID, "ref_cal_num");  // slider
    CreateNexObject(f_att_minus, page1_ID, f_att_minus_ID, "f_att_minus");
    CreateNexObject(f_att_plus, page1_ID, f_att_plus_ID, "f_att_plus");
    CreateNexObject(r_att_minus, page1_ID, r_att_minus_ID, "r_att_minus");
    CreateNexObject(r_att_plus, page1_ID, r_att_plus_ID, "r_att_plus");
    CreateNexObject(toConfig, page0_ID, toConfig_ID, "toConfig");
    CreateNexObject(toSet1, page1_ID, toSet1_ID, "toSet1");
    CreateNexObject(toPwrGraph, page2_ID, toPwrGraph_ID, "toPwrGraph");
    CreateNexObject(toPowerCal, page3_ID, toPowerCal_ID, "toPowerCal");
    CreateNexObject(toMain, page4_ID, toMain_ID, "toMain");
    CreateNexObject(Main, page0_ID, Main_ID, "Main");
    CreateNexObject(Coupler, page1_ID, Coupler_ID, "Coupler");
    CreateNexObject(Set1, page2_ID, Set1_ID, "Set1");
    CreateNexObject(PwrGraph, page3_ID,PwrGraph_ID, "PwrGraph");
    CreateNexObject(PowerCal, page4_ID,PowerCal_ID, "PowerCal");
    CreateNexObject(FactoryRst1, page1_ID, FactoryRst1_ID, "FactoryRst1");
    CreateNexObject(FactoryRst2, page1_ID, FactoryRst2_ID, "FactoryRst2");
    CreateNexObject(Set1_Bandvar,page2_ID,Set1_Bandvar_ID,"Set1.Bandvar");
    CreateNexObject(hv_adj,page2_ID,hv_adj_ID,"hv_adj");  // slider
    CreateNexObject(v14_adj,page2_ID,v14_adj_ID,"v14_adj"); // slider
    CreateNexObject(curr_adj,page2_ID,curr_adj_ID,"curr_adj"); // slider
    CreateNexObject(temp_adj,page2_ID,temp_adj_ID,"temp_adj"); // slider
    CreateNexObject(hv_max,page2_ID,hv_max_ID,"hv_max");  
    CreateNexObject(v14_max,page2_ID,v14_max_ID,"v14_max"); 
    CreateNexObject(curr_max,page2_ID,curr_max_ID,"curr_max"); 
    CreateNexObject(temp_max,page2_ID,temp_max_ID,"temp_max");
    CreateNexObject(meterID,page2_ID,meterID_ID,"meterID"); 
    CreateNexObject(meterID_adj,page2_ID,meterID_adj_ID,"meterID_adj"); // slider
    CreateNexObject(band_set,page2_ID,band_set_ID,"band_set"); 
    CreateNexObject(band_set_adj,page2_ID,band_set_adj_ID,"band_set_adj"); // slider
    CreateNexObject(band, page0_ID, band_ID, "band");
    CreateNexObject(band_cal,page1_ID,band_cal_ID,"band_cal"); 
    CreateNexObject(fPwrGraph,page3_ID,fPwrGraph_ID,"fPwrGraph"); 
    CreateNexObject(fPwr_scale,page3_ID,fPwr_scale_ID,"fPwr_scale"); 
    CreateNexObject(fscale,page3_ID,fscale_ID,"fscale"); 
    CreateNexObject(rscale,page3_ID,rscale_ID,"rscale"); 
    CreateNexObject(fPwrNum,page3_ID,fPwrNum_ID,"fPwrNum"); 
    CreateNexObject(rPwrNum,page3_ID,rPwrNum_ID,"rPwrNum"); 
    CreateNexObject(swrNum,page3_ID,swrNum_ID,"swrNum");
    CreateNexObject(Graph_Timer,page3_ID,Graph_Timer_ID,"Graph_Timer");
    CreateNexObject(HPPwrTarget, page4_ID, HPPwrTarget_ID,"HPPwrTarget");
    CreateNexObject(LPPwrTarget, page4_ID, LPPwrTarget_ID,"LPPwrTarget");
    CreateNexObject(HP_F_VDC, page4_ID, HP_F_VDC_ID,"HP_F_VDC");
    CreateNexObject(HP_R_VDC, page4_ID, HP_R_VDC_ID,"HP_R_VDC");
    CreateNexObject(LP_F_VDC, page4_ID, LP_F_VDC_ID,"LP_F_VDC");
    CreateNexObject(LP_R_VDC, page4_ID, LP_R_VDC_ID,"LP_R_VDC");
    CreateNexObject(Measure_hi, page4_ID, Measure_hi_ID,"Measure_hi");
    CreateNexObject(Measure_lo, page4_ID, Measure_lo_ID,"Measure_lo");
    CreateNexObject(Units, page4_ID, Units_ID,"Units");
    CreateNexObject(CalcFwd, page4_ID, CalcFwd_ID,"CalcFwd");
    CreateNexObject(CalcRef, page4_ID, CalcRef_ID,"CalcRef");
    CreateNexObject(B_HF, page5_ID, B_HF_ID,"B_HF");
    CreateNexObject(B_50, page5_ID, B_50_ID,"B_50");
    CreateNexObject(B_144, page5_ID, B_144_ID,"B_144");
    CreateNexObject(B_222, page5_ID, B_222_ID,"B_222");
    CreateNexObject(B_432, page5_ID, B_432_ID,"B_432");
    CreateNexObject(B_902, page5_ID, B_902_ID,"B_902");
    CreateNexObject(B_1296, page5_ID, B_1296_ID,"B_1296");
    CreateNexObject(B_2_3G, page5_ID, B_2_3G_ID,"B_2_3G");
    CreateNexObject(B_3_4G, page5_ID, B_3_4G_ID,"B_3_4G");
    CreateNexObject(B_5_7G, page5_ID, B_5_7G_ID,"B_5_7G");
    CreateNexObject(B_10G, page5_ID, B_10G_ID,"B_10G");
    CreateNexObject(BandSel, page5_ID, BandSel_ID,"BandSel");
    CreateNexObject(toMain1, page5_ID, toMain1_ID, "toMain1");
    CreateNexObject(Aux1, page0_ID, Aux1_ID,"Aux1");
    CreateNexObject(Aux2, page0_ID, Aux2_ID,"Aux2");
    CreateNexObject(PTT_CW, page0_ID, PTT_CW_ID,"PTT_CW");
    
    // Set up Callback mappings
    NexTouch_attachPop(&savecfg_btn_1, savecfg_btn_1_pop_Callback, 0);   // all 3 page save cfg buttons do the same thing
    NexTouch_attachPush(&savecfg_btn_1, savecfg_btn_1_push_Callback, 0);  // so call the same functions
    NexTouch_attachPop(&savecfg_btn_2, savecfg_btn_1_pop_Callback, 0);
    NexTouch_attachPush(&savecfg_btn_2, savecfg_btn_1_push_Callback, 0);
    NexTouch_attachPop(&savecfg_btn_4, savecfg_btn_1_pop_Callback, 0);
    NexTouch_attachPush(&savecfg_btn_4, savecfg_btn_1_push_Callback, 0);
    NexTouch_attachPop(&fwd_cal, fwd_cal_pop_Callback, 0);
    NexTouch_attachPop(&ref_cal, ref_cal_pop_Callback, 0);
    NexTouch_attachPop(&toMain, toMain_push_Callback, 0);
    NexTouch_attachPop(&toMain1, toMain_push_Callback, 0);
    NexTouch_attachPop(&toConfig, toConfig_pop_Callback, 0);
    NexTouch_attachPop(&toSet1, Set1_Callback, 0); 
    NexTouch_attachPop(&toPwrGraph, toPwrGraph_Callback, 0); 
    NexTouch_attachPop(&f_att_minus, f_att_minus_pop_Callback, 0);
    NexTouch_attachPop(&f_att_plus, f_att_plus_pop_Callback, 0);
    NexTouch_attachPop(&r_att_minus, r_att_minus_pop_Callback, 0);
    NexTouch_attachPop(&r_att_plus, r_att_plus_pop_Callback, 0);
    NexTouch_attachPop(&FactoryRst1, FactoryRst1_pop_Callback, 0);
    NexTouch_attachPop(&FactoryRst2, FactoryRst2_pop_Callback, 0);
    NexTouch_attachPop(&meterID_adj, meterID_adj_pop_Callback, 0);
    NexTouch_attachPop(&band_set_adj, band_set_adj_pop_Callback, 0);
    //NexTouch_attachPop(&band, band_pop_Callback, 0);
    NexTouch_attachPop(&hv_adj, hv_adj_pop_Callback, 0);
    NexTouch_attachPop(&v14_adj, v14_adj_pop_Callback, 0);
    NexTouch_attachPop(&curr_adj, curr_adj_pop_Callback, 0);
    NexTouch_attachPop(&temp_adj, temp_adj_pop_Callback, 0);
    NexTouch_attachPop(&Measure_hi, Measure_hi_pop_Callback, 0);
    NexTouch_attachPop(&Measure_lo, Measure_lo_pop_Callback, 0);
    NexTouch_attachPop(&CalcFwd, CalcFwd_pop_Callback, 0);
    NexTouch_attachPop(&CalcRef, CalcRef_pop_Callback, 0);
    NexTouch_attachPop(&B_HF, BandSelect_HF_pop_Callback, 0);
    NexTouch_attachPop(&B_50, BandSelect_50_pop_Callback, 0);
    NexTouch_attachPop(&B_144, BandSelect_144_pop_Callback, 0);
    NexTouch_attachPop(&B_222, BandSelect_222_pop_Callback, 0);
    NexTouch_attachPop(&B_432, BandSelect_432_pop_Callback, 0);
    NexTouch_attachPop(&B_902, BandSelect_902_pop_Callback, 0);
    NexTouch_attachPop(&B_1296, BandSelect_1296_pop_Callback, 0);
    NexTouch_attachPop(&B_2_3G, BandSelect_2_3G_pop_Callback, 0);
    NexTouch_attachPop(&B_3_4G, BandSelect_3_4G_pop_Callback, 0);
    NexTouch_attachPop(&B_5_7G, BandSelect_5_7G_pop_Callback, 0);
    NexTouch_attachPop(&B_10G, BandSelect_10G_pop_Callback, 0);
    
    digitalWrite(Nextion_Switch, 0);   // Switches the Nextion display serial port between the CPU and the USB converter (For display uploads via USB)
    // Ensure we are on Page 0 in cas we start and the display is already running such as during dev and test
    //UART1_ClearRxBuffer();
    pg=0;
    NexPage_show(&Main);
    strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
    NexText_setText(&Set1_Bandvar);    // Update Nextion display for new values
    NexButton_setText(&band, Band_Cal_Table[CouplerSetNum].BandName);
    sprintf(cmd, " ");  // Clear the RX/TX/CW satus field until we get a valid OTRSP command otherwise
    NexText_setText(&PTT_CW);  // Get PTT first.  If CW present then next line will overwrite
    NexText_Set_font_color_pco(&PTT_CW, 2016);  // Set text to LT Green for RX
    NexText_Set_background_color_bco(&PTT_CW, 0);  // Set background color to black for RX
    NexSlider_setMaxval(&band_set_adj, (long) NUM_SETS);  // Ensure the control know the same num bands the host does.
    Set1_Callback(0);  // initialize our setpoints and meter id so it does not get read back as 0 values later.
    NexPage_show(&Main);
    NexNumber_setValue(&Graph_Timer, 500);   // 300msec for graph update rate.  300 set in display, can override here.
#endif // end Nextion setup section 
  
  adRead(); //get and calculate power + SWR values and display them  
}

// Return the supply voltage in volts.
float read_vcc()
{
    //const float V_BAND_GAP = 1.1;     // typical
    //ADMUX  = _BV(REFS0)    // ref = Vcc
    //       | 14;           // channel 14 is the bandgap reference
    //ADCSRA |= _BV(ADSC);   // start conversion
    //loop_until_bit_is_clear(ADCSRA, ADSC);  // wait until complete
    //return V_BAND_GAP * 1024 / ADC;
    return 3.3;  // ToDo Fix this for Teensy
}

float ADC_RF_Power_CountsTo_Volts(unsigned long counts)
{
  float Volts, VRef;
  VRef = 3.3;      
  Volts = (counts/ADC_COUNTS) * VRef;
  //Serial.println(Volts);
  return Volts;
}

void adRead(void)   // A/D converter read function.  Normalize the AD output to 100%.
{
    float a;
    float b;
    unsigned int c;
    float tmp, retry=0;
    unsigned long ad_counts=0;

    // Throttle the time the CPU spends here.   Offset the timing os teh data acquired is just before it is sent out
    Timer_X00ms_InterruptCnt = millis(); 
    if ((Timer_X00ms_InterruptCnt - Timer_X00ms_Last_AD) < 200)   // will use our own timestamp.
        return;   // skip out until greater than 100ms since our last visit here
    Timer_X00ms_Last_AD = Timer_X00ms_InterruptCnt;  // time stamp our visit here.  Do not want to come back too soon.  

__reread:   // jump label to reread values in case of odd result or hi SWR
    
    // Get Reflected Power     
        total_Ref -= readings_Ref[readIndex_Ref];// subtract the last reading:    
    c = 1; // read from the sensor:
    a = 0;
    for (int i = 0; i < c; ++i)  {
        // ADC_RF_Power_SetGain(1);  // can be used to tweak in the ADC                    
        ad_counts = analogRead(ad_Ref);  // single read but has locked up with N1MM commands at times               
        RefVal = ADC_RF_Power_CountsTo_Volts(ad_counts);         
        a += RefVal;
    }
    a /= c; // calculate the average then use result in a running average
    readings_Ref[readIndex_Ref] = a;   // get from the latest average above and track in this runnign average
    total_Ref += readings_Ref[readIndex_Ref];// add the reading to the total:
    readIndex_Ref += 1;       // advance to the next position in the array:
    //if (NUMREADINGS >1)  // Average less than the Fwd readings to minimize chance of ref being higher than Fwd on transient downward.
    //{
    //    if (readIndex_Ref >= NUMREADINGS-1)  // if we're at the end of the array...        
    //        readIndex_Ref = 0;// ...wrap around to the beginning:
    //}
    //else 
    //{
        if (readIndex_Ref >= NUMREADINGS)  // if we're at the end of the array...        
            readIndex_Ref = 0;// ...wrap around to the beginning:
    //}
    // caclulate dB value for digital display section
    b = total_Ref / NUMREADINGS;// calculate the average:        
    b -= Offset_R;   // adjust to 0V reference point
    b /= Slope_R;   // will be negative for detectors like AD8318
    b += CouplingFactor_Ref;
    b += 0.0; // fudge factor for frequency independent factors like cabling
    
    Ref_dBm = b;
    // 0dBm is max. = 1W fullscale on 1W scale.
    RefPwr = pow(10.0,(b-30.0)/10.0);    // convert to linear value for meter using 1KW @ 0dBm as reference 
    if (RefPwr > 999.9) 
        RefPwr = 999.9999;

    // subtract the last reading for Fwd Power
    total_Fwd -= readings_Fwd[readIndex_Fwd];
    // read from the sensor:
    c = 1;   // short term smaples that feed into running average
    a = 0;
    for (int i = 0; i < c; ++i) {                   
        // ADC_RF_Power_SetGain(1);  // can be used to tweak in the ADC
        ad_counts = analogRead(ad_Fwd);  // single read but has locked up with N1MM commands       
        FwdVal = ADC_RF_Power_CountsTo_Volts(ad_counts);    
        a += FwdVal;
    }
    a /= c; // calculate the average then use result in a running average
    readings_Fwd[readIndex_Fwd] = a;   // get from the latest average above and track in this runnign average
    total_Fwd += readings_Fwd[readIndex_Fwd];    // add the reading to the total: 
    readIndex_Fwd += 1;     // advance to the next position in the array:
    if (readIndex_Fwd >= NUMREADINGS) {     // if we're at the end of the array...      
        readIndex_Fwd = 0;  // ...wrap around to the beginning:
    }     
    b = total_Fwd / NUMREADINGS;    // calculate the average:
    // caclulate dB value for digital display section
    b -= Offset_F;   // adjusts to 0V reference point - calculated during cal
    b /= Slope_F;   // will be negative for detectors like AD8318 - calculated during cal
    b += CouplingFactor_Fwd;
    b += 0.0; // Fudge factor for frequency independent factors like cabling
    Fwd_dBm = b;    // Now have calibrated Forward Value in dBm.
    // 0dBm is max. = 1W fullscale on 1W scale for example
    FwdPwr =  pow(10.0,(b-30.0)/10.0);    // convert to linear value for meter using 1KW @ 0dBm as reference. Multiply by scale value.
    if (FwdPwr > 9999.9)
        FwdPwr = 9999.9999;    
    
    if (RefPwr < FwdPwr) 
        tmp = sqrt(RefPwr/FwdPwr);
    else 
        tmp = sqrt(FwdPwr-0.1/FwdPwr);
        //tmp = .99999;   // if Ref > Fwd, then SWR is sky high, limit number range < 99
        
    SWRVal = ((1 + tmp) / (1 - tmp));  // now have SWR in range of 1.0 to infinity.  
    if (RefPwr > FwdPwr && FwdPwr < 0.2) // Remove false SWR values if no Fwd Pwr such as TX turns off and Fwd goes to 0 before Ref.
        SWRVal = 0;
    else if (RefPwr <= 0.00001 || FwdPwr <= 0.2)
        SWRVal = 0;   // remove misleading SWR numbers when input are floating around in RX mode.
    else if (SWRVal > 9.9)
        SWRVal = 10;
    else if(SWRVal < 0)
        SWRVal = 0;
 
    if (retry < 2 && SWRVal > 2.0) // if SWR is high re read to verify.  Getting occasional false trips when Ref is Higher than Fwd on TX turning off.
    {
        retry++;  // prevent endless loop in hi SWR scenario - we just want to wait out any transient such as transition for RX to TX and TX to RX.
        delay(10);
        //adRead();
        goto __reread;        
    }
    retry = 0;
    
#ifdef SWR_ANALOG  // Update SWR Analog output for Amplifier protection when using KitProg board in an amplifier
    float SWR_val_temp = 0;
    unsigned char SWR_analog_value = 0;
    
    // DAC is configured for 0-4.0VDC output.  Amp board will be adjusted to suit this.
    // SWR_Val should be 0-100 type of number.  We care about 0 to 5 since the amp trip point will be about 3.0/
    SWR_val_temp = SWRVal;   // bounds check uing like size/type vars
    if (SWR_val_temp < 0.0)
        SWR_val_temp = 0.0;
    if (SWR_val_temp > 5.0)
        SWR_val_temp = 5.0;       
    // Actual trip point is set by pot on Amp Control board reading DAC voltage
    SWR_analog_value = SWR_val_temp * (255/5);  // Convert to 0-4.0 range (DAC value 0-255).
    if (SWR_analog_value < 0)
        SWR_analog_value = 0;
    if (SWR_analog_value > 255)
        SWR_analog_value = 255;                
    SWR_Fail_Analog_Output_SetValue(SWR_analog_value);     
#endif
    
    SWR_Serial_Val = SWRVal;
    FwdPwr_last = FwdPwr;  // update memory to minimize screen update and flicker on digital number
    sendSerialData();   // send this data to the serial port for remote monitoring
}

void loop() { 

    unsigned char ret1;
    
    while (1) 
    {
        // Listen for remote computer commands                
        serial_usb_read();  // fetch latest data in buffer                                      
        if (rx_count!=0)
            get_remote_cmd();       // scan buffer for command strings
            
        if (Button_A == YES) {   // Do a press and hold to display reflected power on analog meter. 
            Button_A = NO; //reset flag
            if (op_mode != PWR) {
                // coming from some other op_mode, redraw the screen for PWR op_mode
                op_mode = PWR;
            }                  
            // Save to EEPROM             
            EE_Save_State();            
        }
        if (Button_B == YES) {      // Select Cal Band
            //++CouplerSetNum;   // increment for manual button pushes
            if (NewBand < 0)   // constrain newBand            
            {
                NewBand = 0;   // something wrong if you end up here                        
                delay(1);
            }
            if (NewBand >= NUM_SETS)
                NewBand = NUM_SETS-1; // something wrong if you end up here
            CouplerSetNum = NewBand;    // set to commanded band.  If a Button B remote cmd, NewBand wil lbe incremented before here                            
            if (CouplerSetNum > NUM_SETS)
                CouplerSetNum = 0;                      
            Button_B = NO; //reset flag
            Cal_Table();  
            
#ifdef NEXTION
            strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
            NexText_setText(&Set1_Bandvar);    // Update Nextion display for new values            
            toConfig_pop_Callback(0);  // when on the Config pages will update them
            Set1_Callback(0);   
            update_Nextion(1);                        
#endif       

            EE_Save_State();
        }
        if (Button_C == YES) 
        {
            Button_C = NO; //reset flag
            if (op_mode != SWR)  {
                op_mode = SWR;              
                EE_Save_State();
            }
        }
        
        adRead(); //get and calculate power + SWR values and display them        
        
#ifdef SSD1306_OLED
        // Process OLED Display if used
        OLED();  // Update display for all params
#endif 
        
#ifdef NEXTION
        // Process Nextion Display events
        //Serial_PullMsg(0);                  
        //if (EVT_FLAG) 
        if (Serial_Available()) 
        {
          nexLoop(nex_listen_list);  // Process Nextion Display  
          Serial.println(pg);
        }
        if (NewBand != CouplerSetNum)
            update_Nextion(1);
        else if (!WAIT)   // skip if waiting for response from a diaplsy query to reduce traffic
            update_Nextion(0);    
#endif

/*
        unsigned int ret16;
        // N1MM CW and PTT and AUX message handling
        //ret16 = Serial_USB_GetLineControl();    // look for DTR and RTS for N1MM control of CW and PTT. 
                                                // Can only happen on USB serial since the external USB UART has only TX and RX lines connected to CPU
        // Check state of USB Serial Port DTR register for N1MM CW keying state
        if ((unsigned char)ret16 & Serial_USB_LINE_CONTROL_DTR)                    
            CW_Key_Out_Write(1);          
        else
            CW_Key_Out_Write(0);                       
         Check state of USB Serial Port RTS register for N1MM PTT state
        if ((unsigned char)ret16 & Serial_USB_LINE_CONTROL_RTS)     
            PTT_Out_Write(1);        
        else
            PTT_Out_Write(0);       
  */
        ret1 = OTRSP();   // set Aux output pins and change bands to match
        if (ret1)
        {
            Button_B = YES;  // Process any N1M Aux port commands on UART2
            NewBand = AuxNum1;
        }                
        // Process any radio band decoder changes
        Band_Decoder();
        
    }   // end of while
}

void Band_Decoder(void)
{     
    // Convert BCD to binary and update the (non-BCD) binary Port for amps and meter selection
    unsigned char decoder_band;
    unsigned char decoder_band_binary;

    //decoder_band = Band_BCD_In_Reg_Read();   // Get current radio band value
    decoder_band = digitalRead(2);   // Get current radio band value
    if (decoder_band != decoder_band_last)
    {
        decoder_band_last = decoder_band;
        if (decoder_band == 0) // if 0 it is HF
        {
            digitalWrite(Band_Decode_Control, 0);  // Deselect the amps and antennas if possible and avold divide by zero
            digitalWrite(Antenna_Select, 0);
        }   
        else
        {   // not zero (not HF) pick a VHF band now
            decoder_band = BCDToDecimal(decoder_band);   
            if (decoder_band > 6)
                decoder_band = 6;
            decoder_band_binary = (1 << (decoder_band-1));
            digitalWrite(Band_Decode_Control, decoder_band_binary);   // Convert BCD to parallel    
            // Based on band selected, also send out teh antenna select pattern. 
            // This is hard coded today for 3 SPDT relays to select one of 4 RF sources to connect to an antenna 
            switch(decoder_band)
            {   // writing a 1 will cause a ground (logic 0) to be applied to the latching relay board to move it to position 2.
                case 0: digitalWrite(Antenna_Select, 0); 
                    break;        // HF  - move all switches to position #1
                case 1: digitalWrite(Antenna_Select, 0); 
                    break;        // 50  - move all switches to position #1
                case 2: digitalWrite(Antenna_Select, 0); 
                    break;        // 144 - move pair A to #1, B to #1
                case 3: digitalWrite(Antenna_Select, 1); 
                    break;        // 222 - move pair A to #2, B to #1
                case 4: digitalWrite(Antenna_Select, 2); 
                    break;        // 432 - move pair A to #1, B to #2
                case 5: digitalWrite(Antenna_Select, 3); 
                    break;        // 903 - move pair A to #2, B to #2
                case 6: digitalWrite(Antenna_Select, 0); 
                    break;        // 1296 - move all switches to position #1
                default: digitalWrite(Antenna_Select, 0);
                    break;                
            }
        }
    }
}   
        
unsigned char BCDToDecimal(unsigned char BCD)
{
   return (((BCD>>4)*10) + (BCD & 0xF));
}

#ifdef NEXTION
    
void hv_adj_pop_Callback(void *ptr)
{
    long number;    
    //ptr += 1; // get rid of compiler error about unused ptr    
    NexSlider_getValue(&hv_adj, &number);
    // Change meter ID with this info
    set_hv_max = number;
    set_hv_max /= 10.0;
    NexSlider_setValue(&hv_adj, (uint16_t) (set_hv_max*10));
}

void v14_adj_pop_Callback(void *ptr)
{
    long number;
    //ptr += 1; // get rid of compiler error about unused ptr    
    NexSlider_getValue(&v14_adj, &number);
    // Change meter ID with this info
    set_v14_max = number;
    set_v14_max /= 10.0;
    NexSlider_setValue(&v14_adj, (uint16_t) (set_v14_max*10));
}

void curr_adj_pop_Callback(void *ptr)
{
    long number;    
    //ptr += 1; // get rid of compiler error about unused ptr    
    NexSlider_getValue(&curr_adj, &number);
    // Change meter ID with this info
    set_curr_max = number;
    set_curr_max /= 10;
    NexSlider_setValue(&curr_adj, (uint16_t) (set_curr_max*10));

}  

void temp_adj_pop_Callback(void *ptr)
{
    long number;    
    
    //ptr += 1; // get rid of compiler error about unused ptr    
    NexSlider_getValue(&temp_adj, &number);
    // Change meter ID with this info
    set_temp_max = (unsigned char)number;
    NexSlider_setValue(&temp_adj, (unsigned char) (set_temp_max));
}  
  
void band_set_adj_pop_Callback(void *ptr)
{
    long number;    
    //ptr += 1; // get rid of compiler error about unused ptr    
    NexSlider_getValue(&band_set_adj, &number); 
    NewBand = (unsigned char) number;   // get input for new band from Set1 page
    Button_B = YES; // set flag for main loop to process band change request.  
    // If there is a connection to radio (via serial or wireless) then it will override this at each update.                                                               
}

void band_pop_Callback(void *ptr) 
{
    char buf[32];    
    //ptr += 1; // get rid of compiler error about unused ptr        
    //CouplerSetNum = constrain(CouplerSetNum, 0, NUM_SETS);  
    //NewBand = CouplerSetNum +1;    // Update Newband to current value.  Will be incrmented in button function                          
    //if (NewBand >= NUM_SETS) 
   //     NewBand = 0;  // cycle back to lowest band
    //Button_B = YES; // set flag for main loop to process band change request.  
    // If there is a connection to radio (via serial or wireless) then it will override this at each update.   
    NexButton_setText(&band, itoa(CouplerSetNum, buf, 10)); 
    pg=5;
}

void BandSelect_HF_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr 
    BandSelect(0);
}

void BandSelect_50_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(1);
}

void BandSelect_144_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(2);
}

void BandSelect_222_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(3);
}

void BandSelect_432_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(4);
}

void BandSelect_902_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(5);
}

void BandSelect_1296_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(6);
}

void BandSelect_2_3G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(7);
}

void BandSelect_3_4G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(8);
}

void BandSelect_5_7G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr     
    BandSelect(9);
}

void BandSelect_10G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    //ptr += 1; // get rid of compiler error about unused ptr 
    BandSelect(10);
}

void BandSelect(unsigned char sband)   // Get var BandSel assigned by each band button to set NewBand var
{
    //long number;
    char buf[32];
    //NexNumber_getValue(&BandSel,&number);        
    NewBand = sband;    // Update Newband to current value.  Will be incremented in button function
    if (NewBand >= NUM_SETS) 
        NewBand = 0;  // cycle back to lowest band
    Button_B = YES; // set flag for main loop to process band change request.  
    // If there is a connection to radio (via serial or wireless) then it will override this at each update.   
    NexButton_setText(&band, itoa(NewBand, buf, 10));
    pg=0;
    update_Nextion(1);
}

void Measure_hi_pop_Callback(void *ptr)
{   
    long number; 
    //ptr += 1; // get rid of compiler error about unused ptr
    //NexButton_setText(&savecfg_button, "Saved Config");
    NexNumber_getValue(&HPPwrTarget, &number);
    Pwr_hi = number;
    NexNumber_getValue(&Units, &number);
    if (number==0)  // convert to dBm if in Watts (0 value)
    {
        // Get target lo power level from user in Watts, convert to dBm then measure and save ADC                                
           Pwr_hi = 10 * log10(1000*(Pwr_hi/1));
    }  
    // Now value is in dBm proceed
    adRead();  // update our high power readings for forward and reflected power for the current band
    FwdVal_hi = FwdVal; // save for calc     
    NexNumber_setValue(&HP_F_VDC, (int)(FwdVal_hi*100000));  // display ADC raw voltage on screen
    RefVal_hi = RefVal;  // save for calc    
    NexNumber_setValue(&HP_R_VDC, (int)(RefVal_hi*100000));
    print_Cal_Table_progress(1);  // end of loop   
}

void Measure_lo_pop_Callback(void *ptr)
{   
    long number;  
    //ptr += 1; // get rid of compiler error about unused ptr
    //NexButton_setText(&savecfg_button, "Saved Config");
    NexNumber_getValue(&LPPwrTarget, &number);
    Pwr_lo = number;        
    NexNumber_getValue(&Units, &number);
    if (number==0)
    {
        // Get target lo power level from user in Watts, convert to dBm then measure and save ADC                                
           Pwr_lo = 10 * log10(1000*(Pwr_lo/1));
    }
    // Now value is in dBm proceed
    adRead();  // update our high power readings for foprward and reflected power for the current band
    FwdVal_lo = FwdVal; // save for calc  
    NexNumber_setValue(&LP_F_VDC, (int)(FwdVal_lo*100000));
    RefVal_lo = RefVal;  // save for calc    
    NexNumber_setValue(&LP_R_VDC, (int)(RefVal_lo*100000));
    print_Cal_Table_progress(1);  // end of loop
}

void CalcFwd_pop_Callback(void *ptr)
{
    //ptr += 1; // get rid of compiler error about unused ptr 
    if (Pwr_hi != 0) // ensure we have a valid high power number before calculating.
    {                      
        // Calculate Slope and Offset
        Slope_F = (FwdVal_hi - FwdVal_lo)/(Pwr_hi - Pwr_lo);
        Offset_F = FwdVal_hi + ((CouplingFactor_Fwd - Pwr_hi) * Slope_F);
        Cal_Table_write();
        Cal_Table();  // read back to be sure it save correctly                                
        print_Cal_Table_progress(0);  // end of loop
    }
}

void CalcRef_pop_Callback(void *ptr)
{
    //ptr += 1; // get rid of compiler error about unused ptr 
    if (Pwr_hi != 0) // ensure we have a valid high power number before calculating.
    {                             
        // Calculate Slope and Offset
        Slope_R = (RefVal_hi - RefVal_lo)/(Pwr_hi - Pwr_lo);
        Offset_R = RefVal_hi + ((CouplingFactor_Ref - Pwr_hi) * Slope_R);
        Cal_Table_write();
        Cal_Table();  // read back to be sure it save correctly
        print_Cal_Table_progress(0);  // end of loop
    }
}

void meterID_adj_pop_Callback(void *ptr)
{
    long number;    
    //ptr += 1; // get rid of compiler error about unused ptr    
    NexSlider_getValue(&meterID, &number);
    // Change meter ID with this info
    //METERID = number;
}   
    
void FactoryRst1_pop_Callback(void *ptr)
{    
    //ptr += 1; // get rid of compiler error about unused ptr    
    // Sequenctionally pressing thse adn Rst2 hotspot areas is the same as 2 button reset of M5Stack or remote commands 193+195
    Reset_Flag = 1;
}

void FactoryRst2_pop_Callback(void *ptr)
{
    //ptr += 1; // get rid of compiler error about unused ptr
    if (Reset_Flag == 1) 
        reset_EEPROM();
        delay(1000);
        toConfig_pop_Callback(0);
        update_Nextion(1);
    Reset_Flag = 0;
}

void savecfg_btn_1_push_Callback(void *ptr)
{    
    //ptr += 1; // get rid of compiler error about unused ptr
    //NexButton_setText(&savecfg_button, "Saving");
    /*
    LED_Write(0);
    LED_1_Write(0);
    CyDelay(200);
    LED_Write(1);
    LED_1_Write(1);
    */
    EE_Save_State();  // push the button to commit the cal changes - Watch LED to see how long the EEPROM really takes.
    // There is 2 seconds total delay on the Nextion save config button to allow time for the CPU to finish loaded.    
    /*
    LED_Write(0);    // write to both ports for KitProg and Main Board onboard LEDs
    LED_1_Write(0);
    */
}

void savecfg_btn_1_pop_Callback(void *ptr)
{   
    //ptr += 1; // get rid of compiler error about unused ptr
    //NexButton_setText(&savecfg_button, "Saved Config");
    //LED_Write(1);    // write to both ports for KitProg and Main Board onboard LEDs
    //LED_1_Write(1);
}

void fwd_cal_pop_Callback(void *ptr) // Slider for Fwd Coupler Cal Value
{      
    //ptr += 1; // get rid of compiler error about unused ptr   
    // Read value of slider sent in event and set in Cal table then echo it back to textbox
    NexSlider_getValue(&fwd_cal, &rcv_num); ;
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd = rcv_num;
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update lable with decimal value text
    NexText_setText(&fwd_cal_num);    
    Cal_Table();
    toConfig_pop_Callback(0);
}

void ref_cal_pop_Callback(void *ptr) // Slider for Ref Coupler Cal Value
{
    //ptr += 1; // get rid of compiler error about unused ptr
    // Read value of slider sent in event and set in Cal table then echo it back to textbox
    NexSlider_getValue(&ref_cal, &rcv_num);  
    Band_Cal_Table[CouplerSetNum].Cpl_Ref = rcv_num;    
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update lable with decimal value text
    NexText_setText(&ref_cal_num);    
    Cal_Table();
    toConfig_pop_Callback(0);
}

void f_att_minus_pop_Callback(void *ptr) // fine tune slider by 0.1dB
{
    //ptr += 1; // get rid of compiler error about unused ptr
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd -= 0.1;
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update lable with decimal value text
    NexText_setText(&fwd_cal_num);     
    Cal_Table();
}

void f_att_plus_pop_Callback(void *ptr)
{    
    //ptr += 1; // get rid of compiler error about unused ptr          
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd += 0.1;    
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update lable with decimal value text
    NexText_setText(&fwd_cal_num);    
    Cal_Table();
}

void r_att_minus_pop_Callback(void *ptr)
{      
    //ptr += 1; // get rid of compiler error about unused ptr    
    Band_Cal_Table[CouplerSetNum].Cpl_Ref -= 0.1;
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update lable with decimal value text
    NexText_setText(&ref_cal_num); 
    Cal_Table();
}

void r_att_plus_pop_Callback(void *ptr)
{
    //ptr += 1; // get rid of compiler error about unused ptr   
    Band_Cal_Table[CouplerSetNum].Cpl_Ref += 0.1;    
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update lable with decimal value text
    NexText_setText(&ref_cal_num); 
    Cal_Table();
}

void toMain_push_Callback(void *ptr)
{
    //ptr += 1; // get rid of compiler error about unused ptr
    pg = 0;   // flag Update function to only update when page is active 
    update_Nextion(1);
    //NexPage_show(&page0);
}

void toConfig_pop_Callback(void *ptr)   // Got event to change to Config Page from somewhere
{
    //unsigned char number;
    //unsigned char ret1;    
    //ptr += 1; // get rid of compiler error about unused ptr
    
    // Update the sliders to current values. SLider callbacks wil update after here.
    //strcpy(cmd, "sendme");
    //sendCommand(cmd);
    //CyDelay(2);
    //ret1 = recvPageNumber(&number);    
    //if ((ret1 == 1) && (number = 1))  // ensure we are on the correct page)
    if (1 == 1)  // force to true, should only be in this fucntion if a page event sent us to here
    {       
        rcv_num = (long)(Band_Cal_Table[CouplerSetNum].Cpl_Fwd);
        NexSlider_setValue(&fwd_cal, rcv_num);  // Update slider knob position  0-100
         
        snprintf(cmd, 21, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update label with decimal value text
        NexText_setText(&fwd_cal_num);
      
        // Initialize the sliders to the current band value for Reflected
        rcv_num = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
        NexSlider_setValue(&ref_cal, rcv_num);
        
        snprintf(cmd, 20, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update label with decimal value text
        NexText_setText(&ref_cal_num);       
    }
}

/*
 *      Setpoints page on graphics display where alarm levels for things like voltage and temp are set.  
 *      These are globals so in theory do nto have to be on teh page to write to them
 *      Includes the unused *ptr so this can be used as a callback function if needed.
*/
void Set1_Callback(void *ptr)
{   
    //unsigned char number;
    //ptr += 1; // get rid of compiler error about unused ptr
    
    /*
    if (pg != 1)
    {
        strcpy(cmd, "sendme");
        sendCommand(cmd);    
        //CyDelay(10);
        if (!recvPageNumber(&number))
            return;
    }
    */
    //if (number == 2 || pg == 2) // only send this group of data while on page 2
    if (pg == 1)  // should only be here because a touch event called us.
    {
        //NexPage_show(&Set1);
        delay(25);
        //ptr += 1; // get rid of compiler error about unused ptr
        // The XYZ_max fields are considered the source for the slider to get its initial data from through the     
        //    the display's "Setpoints" page (page2) preinitialization section. After that the slider sends changes real time to the display
        //    not bothering the CPU until movement stops when an event is sent to the CPU where the slider callback will 
        //    get the XYZ_max value and store it in EEPROM.
        // For xfloat type object in Nextion, multiply the float x10 to get an integer it can use with teh slider
        // The display will format it with decimal point according to its config (num places to left and right of decimal point)
        NexSlider_setValue(&hv_adj, (uint16_t) (set_hv_max*10));
        NexNumber_setValue(&hv_max, (uint16_t) (set_hv_max*10));

        NexSlider_setValue(&v14_adj, (uint16_t) (set_v14_max*10));
        NexNumber_setValue(&v14_max, (uint16_t) (set_v14_max*10));

        NexSlider_setValue(&curr_adj, (uint16_t) (set_curr_max*10));
        NexNumber_setValue(&curr_max, (uint16_t) (set_curr_max*10));

        NexSlider_setValue(&temp_adj, (uint16_t) set_temp_max);
        NexNumber_setValue(&temp_max, (uint16_t) set_temp_max);
       
        NexSlider_setValue(&meterID_adj, METERID);
        NexNumber_setValue(&meterID, (uint16_t) METERID);
        
        // fix up initial position of slider knob for Band Slider
        NexSlider_setValue(&band_set_adj, CouplerSetNum);
        strcpy(cmd, Band_Cal_Table[CouplerSetNum].BandName);
        NexText_setText(&band_set);    
        strcpy(cmd, Band_Cal_Table[CouplerSetNum].BandName);
        NexText_setText(&band_cal);    
    }
}

void toPwrGraph_Callback(void *ptr)
{
    //ptr += 1; // get rid of compiler error about unused ptr
    pg=3;
    return;
}

unsigned char update_Nextion(unsigned char force_update)
{
    static float d;
    unsigned char temp_value_old = 0, set_temp_max_old = 0;
    float value;
    long number32;
    //unsigned char ret;
    static float FwdPwr_old, RefPwr_old, SWRVal_old, hv_value_old, V14_value_old, curr_value_old, Fwd_dBm_old, Ref_dBm_old;
    static unsigned char AuxNum1_old, AuxNum2_old, NewBand_old;
    static float set_curr_max_old, set_hv_max_old, set_v14_max_old;
      
    if (!force_update)
    {
        // throttle update rate
        if (DLY < 5)
            d = 5;
        else 
            d= DLY/5;
        if ((Timer_X00ms_InterruptCnt - Timer_X00ms_Last_Nex) < d)
           return 0;   // skip out until greater than 100ms since our last visit here
        Timer_X00ms_Last_Nex = Timer_X00ms_InterruptCnt;
  /*      if (pg != number)
        {
            strcpy(cmd, "sendme");
            sendCommand(cmd);
        
            CyDelay(5);
            ret = recvPageNumber(&number);
            if (ret == 1)  // good page num returned
                pg = number;    // else just skip this and use the pg value
        }
        */
    }
    else
    {
        // force all status tracking variables to 0 forcing them to update on this pass.
        //FwdPwr_old=RefPwr_old=SWRVal_old=hv_value_old=V14_value_old=temp_value_old=curr_value_old=NewBand_old=0;
    }
    
    // use page number to refresh each applicable page
    if (force_update || pg == 0) // only send this group of data while on page 0 every 250 ms at most
    { 
        // only send these Page 0 commands if we are on page 0 else we will get invalid data return messages
        // Update float fields on display        
        if (round(FwdPwr*16) != round(FwdPwr_old*16) && !WAIT)
        {
            FwdPwr_old = FwdPwr;
            if (FwdPwr > 9999.99)
                sprintf(cmd, "fwdpwr.txt=\"OVER\"");
            else if (FwdPwr > 99.99)
                sprintf(cmd, "fwdpwr.txt=\"%.0fW\"", FwdPwr);
            else
                sprintf(cmd, "fwdpwr.txt=\"%.1fW\"", FwdPwr);
            //NexText_setText(FwdPwr, cmd);   // alternative way to update val;ue using the library commands
            sendCommand(cmd);
        }
        if (round(RefPwr*16) != round(RefPwr_old*16) && !WAIT)
        {   
            RefPwr_old = RefPwr;
            if (RefPwr > 999.99)
                sprintf(cmd, "refpwr.txt=\"OVER\"");
            else if (RefPwr > 99.9)
                sprintf(cmd, "refpwr.txt=\"%.0fW\"", RefPwr);
            else
                sprintf(cmd, "refpwr.txt=\"%.1fW\"", RefPwr);
            sendCommand(cmd);
        }    
        
        if (round(Fwd_dBm*16) != round(Fwd_dBm_old*16) && !WAIT)
        {   
            Fwd_dBm_old = Fwd_dBm;
            sprintf(cmd, "FdBm.txt=\"%.2fdBm\"", Fwd_dBm);
            sendCommand(cmd);
        }
        
        if (round(Ref_dBm*16) != round(Ref_dBm_old*16) && !WAIT)
        {   
            Ref_dBm_old = Ref_dBm;
            sprintf(cmd, "RdBm.txt=\"%.2fdBm\"", Ref_dBm);
            sendCommand(cmd);        
        }
        
        if  (SWRVal != SWRVal_old && !WAIT)
        {
            SWRVal_old = SWRVal;
            value = SWRVal;
            if (value > 99.9)      // Display Overrange
            {
                sprintf(cmd, "SWR.txt=\"%s\"", "OVR");
                sendCommand(cmd);
                sprintf(cmd, "SWR.pco=63488");   // Red Text
                sendCommand(cmd);
            }
            else if (value > 2.0)      // Display Overrange
            {
                sprintf(cmd, "SWR.txt=\"%.1f\"", value);
                sendCommand(cmd);
                sprintf(cmd, "SWR.pco=63488");   // Red Text
                sendCommand(cmd);
            }
            else if (value < 0.002)    // Display greyed out NA
            {
                sprintf(cmd, "SWR.txt=\"%s\"", "NA");
                sendCommand(cmd);
                sprintf(cmd, "SWR.pco=50712");   // Flip value to grey
                sendCommand(cmd);
            }   
            else   // Display normal value
            {
                sprintf(cmd, "SWR.txt=\"%.1f\"", value);
                sendCommand(cmd);
                sprintf(cmd, "SWR.pco=65504");   // Flip value to normal yellow    
                sendCommand(cmd);  
            }
        }

        value = hv_read();   // convert to full ADC read function - this is temp for now.   
        value *= hv_cal_factor;
        if ((round(value*16) != round(hv_value_old*16) || set_hv_max_old != set_hv_max) && !WAIT)
        {
            hv_value_old = value;
            set_hv_max_old = set_hv_max;      
            if (value > set_hv_max)       
            {
                sprintf(cmd, "hv.pco=63488");   // Flip value to Red for high alert            
                sendCommand(cmd);        
            }
            else
            {
                sprintf(cmd, "hv.pco=2016");   // Flip value to normal yellow
                sendCommand(cmd);
            }
            sprintf(cmd, "hv.txt=\"%.1fV\"", value);
            sendCommand(cmd); 
        }
        
        value = v14_read();   
        value *= v14_cal_factor;
        if ((round(value*16) != round(V14_value_old*16) || set_v14_max_old != set_v14_max) && !WAIT)
        {
            V14_value_old = value;
            set_v14_max_old = set_v14_max;
            if (value > set_v14_max)
            {
                sprintf(cmd, "v14.pco=63488");   // Flip value to Red for high alert            
                sendCommand(cmd);
            }
            else
            {
                sprintf(cmd, "v14.pco=2016");   // Flip value to normal Yellow #65504                  
                sendCommand(cmd);
            }
            sprintf(cmd, "v14.txt=\"%.1fV\"", value);        
            sendCommand(cmd);
        }
           
        value = curr_read();
        value -= curr_zero_offset;
        value *= curr_cal_factor;
        if  ((round(value*16) != round(curr_value_old*16) || set_curr_max_old != set_curr_max) && !WAIT)
        {
            curr_value_old = value;
            set_curr_max_old = set_curr_max;
            if (value > set_curr_max)
            {        
                sprintf(cmd, "curr.pco=63488");   // Flip value to Red for high alert            
                sendCommand(cmd); 
            } 
            else
            {
                sprintf(cmd, "curr.pco=2016");   // Flip value to normal Yellow #65504                  
                sendCommand(cmd); 
            }
            sprintf(cmd, "curr.txt=\"%.1fA\"", value);
            sendCommand(cmd); 
        }
        
        value = temp_read();
        value *= temp_cal_factor;
        if  ((round(value*16) != round(temp_value_old*16) || set_temp_max_old != set_temp_max) && !WAIT)
        {
            temp_value_old = value;
            set_temp_max_old = set_temp_max;
            if (value > set_temp_max)
            {        
                sprintf(cmd, "temp.pco=63488");   // Flip value to Red for high alert            
                sendCommand(cmd); 
            } 
            else
            {
                sprintf(cmd, "temp.pco=2016");   // Flip value to normal Yellow #65504                  
                sendCommand(cmd); 
            }
            sprintf(cmd, "temp.txt=\"%.1fF\"", value);
            sendCommand(cmd); 
        }              
             
        if (force_update == 1)
        {
            sprintf(cmd, "band.txt=\"%s\"", Band_Cal_Table[NewBand].BandName);
            sendCommand(cmd); 
            
            strcpy(cmd,Band_Cal_Table[NewBand].BandName);
            NexText_setText(&Set1_Bandvar);      
        }
        else
        {
            sprintf(cmd, "band.txt=\"%s\"", Band_Cal_Table[CouplerSetNum].BandName);
            sendCommand(cmd); 
            
            strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
            NexText_setText(&Set1_Bandvar);      
        }
        
        if (NewBand_old != CouplerSetNum || force_update == 1)
        {
            NewBand_old = CouplerSetNum;            
            sprintf(cmd, "band.pco=65535");   // Flip value white
            sendCommand(cmd);            
            sprintf(cmd, "band.bco=20");   // Flip value to dark blue = background
            sendCommand(cmd);
            if (CouplerSetNum == 0)
                delay(1);
        }
        
        // Update Display for state of AUX 1 and 2 ports
        if ((AuxNum1 != AuxNum1_old || force_update == 1) && !WAIT)
        {
            AuxNum1_old = AuxNum1;
            number32 = (long) AuxNum1;
            NexNumber_setValue(&Aux1, number32);
        }
        if ((AuxNum2 != AuxNum2_old || force_update == 1) && !WAIT)
        {
            AuxNum2_old = AuxNum2;    
            number32 = (long) AuxNum2;
            NexNumber_setValue(&Aux2, number32);
        } 
 #ifdef USBSTUFF       
        // Update display for state of PTT and CW IO lines
        unsigned char state;
        state = Serial_USB_IsLineChanged();              /* Check for Line settings changed */
        if(state != 0u)
        {  
            if(state & Serial_USB_LINE_CONTROL_CHANGED)  /* Show new settings */
            {   
                state = Serial_USB_GetLineControl();
                if (((state & Serial_USB_LINE_CONTROL_RTS) == 2) && ((state & Serial_USB_LINE_CONTROL_DTR) == 0)) // PTT on and CW off
                {
                    sprintf(cmd, "TX");
                    NexText_setText(&PTT_CW);  // Get PTT first.  If CW present then next line will overwrite
                    NexText_Set_font_color_pco(&PTT_CW, 65504);  // Set text to Yellow for tx                    
                    NexText_Set_background_color_bco(&PTT_CW, 63488);  // Set bkgnd to RED for tx                    
                }
                else if (((state & Serial_USB_LINE_CONTROL_RTS) == 0) && ((state & Serial_USB_LINE_CONTROL_DTR) == 0)) // PTT on and CW off
                {
                    sprintf(cmd, "RX");
                    NexText_setText(&PTT_CW);  // Get PTT first.  If CW present then next line will overwrite
                    NexText_Set_font_color_pco(&PTT_CW, 2016);  // Set text to LT Green for RX
                    NexText_Set_background_color_bco(&PTT_CW, 0);  // Set background color to black for RX
                }                                        
                if (((state & Serial_USB_LINE_CONTROL_RTS) == 2) && ((state & Serial_USB_LINE_CONTROL_DTR) == 1)) // PTT on and CW off
                {
                    sprintf(cmd, "CW");
                    NexText_setText(&PTT_CW);  // Get PTT first.  If CW present then next line will overwrite
                    NexText_Set_font_color_pco(&PTT_CW, 2047);  // Set text to RED for tx  
                    NexText_Set_background_color_bco(&PTT_CW, 63488);  // Set background color to black for RX                  
                }
                else if (((state & Serial_USB_LINE_CONTROL_RTS) == 0) && ((state & Serial_USB_LINE_CONTROL_DTR) == 1)) // PTT on and CW off
                {
                    sprintf(cmd, "CW");
                    NexText_setText(&PTT_CW);  // Get PTT first.  If CW present then next line will overwrite
                    NexText_Set_font_color_pco(&PTT_CW, 2047);  // Set text to REd for TX.  Then check PTT again for RX state
                    NexText_Set_background_color_bco(&PTT_CW, 0);  // Set background color to black for RX
                }
            }
        }  
#endif
        return 1;
    }
    else if (pg == 3) // only send this group of data while on page 3
    {   // Page 3 is the Graphing page
        // Post up power and SWR values.  Display will read the values on a timer and update teh graph.  
        // It will also read the scale and cap the values to the scale max and scale the graph with its own math        
        
        if (FwdPwr > 1999.9)        
        {                                 
            NexNumber_setValue(&fPwrNum,1999); 
        }
        else
            NexNumber_setValue(&fPwrNum,FwdPwr);
            
        if (RefPwr > 999.9)        
        {   // graphing only accepts integer betwen 0 and 255. using 1000W max                    
            NexNumber_setValue(&rPwrNum,999); 
        }
        else
            NexNumber_setValue(&rPwrNum,RefPwr);             
                   
        // Just post up the value, the display will handle the math and graphing in a timer
        NexNumber_setValue(&swrNum,SWRVal*10); // uses xfloat so mult by 10 to shift decimal point      
    }    
    else if (pg == 4) // only send this group of data while on page 4
    {   
        // Page 4 is ADC calibration 
        if (!recvRetNumber(&number32))
            return 0;
        else
            return 1;
    }
    else if (pg == 5) // only send this group of data while on page 5
    {   
        // Page 5 is Quick Band Select
        if (!recvRetNumber(&number32))
            return 0;
        else
            return 1;
    }
    else if (pg == 1)  // on the Config Page #1
    {   
        // should not be here.
        return 0;
    }    
   return 1;
}
#endif  // end Nextion section

#ifdef SSD1306_OLED
/*                    
    Print to to 128x64 OLED graphics screen.  SSD1306 I2C type.
*/        
void OLED(void)
{ 
    char s[24]; 
    float value;
        
    display_clear();    
    gfx_drawRect( 0, 17, 127, 47, WHITE);
    
    // Top row is Yellow with Power in Watts, size 2 font
    gfx_setTextColor(WHITE);
    snprintf(s, 12, "  %*.1fW%c", 6, FwdPwr, '\0');
    gfx_setTextSize(2);    
    gfx_setCursor(4,0);
    gfx_println(s);
    
    //sprintf(s,"  Fwd    Ref    SWR");    
    gfx_setTextSize(1);    
    gfx_setCursor(16,20);
    gfx_println("Fwd");
    gfx_setCursor(58,20);
    gfx_println("Ref");
    gfx_setCursor(95,20);
    gfx_println("SWR");
    
    if (FwdPwr <= 0.1)
        sprintf(s, " %*.1f%*.1f%*s%c", 5, 0.0, 7, 0.0, 5, "NA", '\0'); 
    else
        sprintf(s, " %*.1f%*.1f%*.1f%c", 5, Fwd_dBm, 7, Ref_dBm, 6, SWRVal, '\0');    
    gfx_setTextSize(1);
    gfx_setCursor(1,30);
    gfx_println(s);
    
    value = temp_read();
    value *= temp_cal_factor;
    sprintf(s,"%*.0fF%c", 3, value, '\0');   // temp from the ADL5519 board (not the PA heat sink, maybe later from amp control board)
    gfx_setTextSize(1);
    gfx_setCursor(5,49);
    gfx_println(s);
        
    value = v14_read();
    value *= v14_cal_factor;
    sprintf(s,"%*.1fVDC%c", 4, value, '\0');   // 28V via Voltage divider
    gfx_setTextSize(1);
    gfx_setCursor(33,49);
    gfx_println(s);
    
    value = hv_read();
    value *= hv_cal_factor;
    sprintf(s,"%*.1fVDC%c", 4, value, '\0');   // 28V via Voltage divider
    gfx_setTextSize(1);
    gfx_setCursor(81,49);
    gfx_println(s);
    
    display_update();    // NOTE: You should remember to update the display in order to see the results on the oled. 
}
#endif // end SSD1306_OLED section

/*******************************************************************************
* Function Name: serial_usb_read
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Waits until VBUS becomes valid and starts the USBFS component which is
*      enumerated as virtual Com port.
*   2. Waits until the device is enumerated by the host.
*   3. Waits for data coming from the hyper terminal and sends it back.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
unsigned int serial_usb_read(void)
{
     rx_count = 0;
     int count = 0;
     int i=0;  
     /* Check for input data from host. */
     count = Serial.available();
     rx_buffer[0] = NULL;
     if (count > 0)
     {
          while (i < count)
          {
              rx_buffer[i++] = Serial.read();
              rx_buffer[i] = '\0';
          }
          rx_count = i;          
          //Serial.println(rx_count);
          //Serial.println((char *) rx_buffer);
          // initially p1 = p2.  parser will move p1 up to p2 and when they are equal, buffer is empty, parser will reset p1 and p2 back to start of sData         
          memcpy(pSdata2, rx_buffer, rx_count+1);   // append the new buffer data to current end marked by pointer 2        
          pSdata2 += rx_count;   // Update the end pointer position. The function processing chars will update the p1 and p2 pointer             
          rx_count = pSdata2 - pSdata1;  // update count for total unread chars. 
          //Serial.println(rx_count);  
     }
     rx_buffer[0] = '\0';
     return rx_count;
}

/*******************************************************************************
* Function Name: serial_usb_write
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. send 1 character ou thte usb virtual comm port.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void serial_usb_write(void)
{      
    tx_count = strlen((char *) tx_buffer);
    
    if (0u != tx_count)
    {
        Serial.write(tx_buffer, tx_count);         /* Send data back to host. */
    }
}

float hv_read(void)
{
    long ad_counts=0;
    
    // Read voltage by ADC via MUX (If connected)
    delay(5);
    ad_counts = analogRead(MUX_HV);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float v14_read(void)
{
    long ad_counts=0;
    
    // Read voltage by ADC via MUX (If connected)
    delay(5);
    ad_counts = analogRead(MUX_14V);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float curr_read(void)
{
    long ad_counts=0;
    
    // Read ADC via MUX (If connected)
    delay(5);
    ad_counts = analogRead(MUX_CURR);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float temp_read(void)
{
#ifdef DETECTOR_TEMP_CONNECTED
  float tmp;
    long ad_counts=0;
    // Read detector temperature (If connected)
    delay(5);
    ad_counts = analogRead(MUX_TEMP);  // single read but has locked up with N1MM commands   
    tmp = ADC_RF_Power_CountsTo_Volts(ad_counts);      // store the detector temp reading for cal optimization if desired.  For now jsut display it on the screen
    tmp -= 1.36;   // ADL5519 is 4.48mV/C at 27C which is typically 1.36VDC.  Convert to F.  
    tmp /= 0.00448;   // mV/C
    tmp += 27;   //1.36V at 27C (80.6F)
    tmp *= 9;
    tmp /= 5;    // convert to F
    tmp += 32;    
    TempVal = tmp;  
    return TempVal;
#endif 
    return 0;   // if not temp connected just return 0;
}

/*
 *   Send out dBm, Watts, and SWR values to data channel - serial, WiFi, or Bluetooth
*/
void sendSerialData()
{
    Timer_X00ms_InterruptCnt = millis();
    if ((Timer_X00ms_InterruptCnt - Timer_X00ms_Last) > 200 && ser_data_out == 1); //&& EEPROM.read(SER_DATA_OUT_OFFSET) == 'Y' )
    {
        Timer_X00ms_Last_USB = Timer_X00ms_InterruptCnt;       
        sprintf((char *) tx_buffer,"%d,%s,%s,%.2f,%.2f,%.1f,%.1f,%.1f\r\n%c", METERID, "170", Band_Cal_Table[CouplerSetNum].BandName, Fwd_dBm, Ref_dBm, FwdPwr, RefPwr, SWR_Serial_Val, '\0');       
        serial_usb_write();
        sprintf((char *) tx_buffer,"%d,%s,%.1f,%.1f,%.1f,%.1f\r\n%c", METERID, "171", hv_read()*hv_cal_factor, v14_read()*v14_cal_factor, (curr_read()-curr_zero_offset)*curr_cal_factor, temp_read()*temp_cal_factor, '\0');       
        serial_usb_write();   
    }
}

void get_remote_cmd()
{
    unsigned char cmd1;
    float cmd2;
    unsigned char cmd_str_len;
    unsigned char i = 0; 
    unsigned char j = 0;
    char cmd_str[BUF_LEN] = {};
    
    if (rx_count >=0) 
    { 
        pSdata = strchr(pSdata1, '\n');   // find string terminator position 
        if (pSdata) { 
            *pSdata = '\0';
            cmd_str_len = pSdata - pSdata1;                     
            strncpy(cmd_str, pSdata1, cmd_str_len);   // copy chars between p1 and the terminator
            pSdata1 += (cmd_str_len+1);  // reset ch pointer back to strat of string for char parsing
            if (pSdata1 >= pSdata2 || cmd_str_len > BUF_LEN)  // if we have caught up to the end p2 then reset to beginning of buffer position.
                pSdata1 = pSdata2 = sdata;           
            if (strlen(sdata) >= BUF_LEN-1) {
                //pSdata = sdata;
                sdata[0] = '\0';
                //printf("BUFFER OVERRRUN\n");
                //Serial_ClearRxBuffer();            
                return;
            }             
            // filter out unwanted characters             
            // Now we have a full comma delimited command string - validate and break it down           
            j = 0;
            for (i=0; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                if (isalnum(sdata[i]))
                    cmd_str[j++] = (sdata[i]);                 
            }
            cmd_str[j] = '\0';  
            //Serial.print(" Meter ID  ");
            //Serial.println("%s\n",cmd_str);
            if (atoi(cmd_str) == METERID) {
                if (i < cmd_str_len) {
                    j = 0;
                    // Look for Msg_Type now
                    for (i +=1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {   // i+1 to skip over the comma the var 'i' was left pointing at
                        cmd_str[j++] = sdata[i];                 
                    }
                    cmd_str[j] = '\0';
                    //printf(" Msg Type:  ");
                    //printf("%s",cmd_str);
                  
                    if (atoi(cmd_str) == 120)  {   // Vaidate this message type for commands.  120 = coammand, 170 is data output                              
                        j = 0;
                        if (i < cmd_str_len) {                               
                            for (i += 1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                                cmd_str[j++] = sdata[i];
                            }  
                            cmd_str[j] = '\0';
                            //printf(" CMD1:  ");
                            //printf("%s",cmd_str);
                            cmd1 = atoi(cmd_str);
                        }
                        else 
                            cmd1 = 0;
                        
                        j = 0;
                        if (i < cmd_str_len) {                                                
                            for (i += 1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                                cmd_str[j++] = sdata[i];                          
                            }
                            cmd_str[j] = '\0';
                            //printf(" CMD2:  ");
                            //printf("%s",cmd_str);
                            cmd2 = atof(cmd_str);
                        }
                        else 
                            cmd2 = 0.0;
                  
                        // Now do the commands     
                        // add code to limit variables received to allowed range
                        if (cmd1 == 255) Button_A = YES;   // switch scale - not really useful if you cannot see the meter face, data is not changed
                        if (cmd1 == 254) {
                            Button_B = YES;   // switch bands
                            //CouplerSetNum = constrain(CouplerSetNum, 0, NUM_SETS);  
                            NewBand = CouplerSetNum +1;    // Update Newband to current value.  Will be incrmented in button function                          
                            if (NewBand > NUM_SETS) 
                                NewBand = 0;  // cycle back to lowest band
                        }
                        if (cmd1 == 253) Button_C = YES;   // Switch op_modes betweem SWR and PWR - same as scale, not useful lif you cannot seethe meter face.
                        if (cmd1 == 252) print_cal_table();  //Speed up or slow down the Serial line output info rate
                        if (cmd1 == 251) ;  // Not Used yet
                        if (cmd1 == 250) {   // dump current cal table to remote  (Was Scale GUI button)
                            Button_B = YES;
                            NewBand = 10;  
                        }   
                        if (cmd1 == 249) {     // Jump to Band 5.7G
                            Button_B = YES;
                            NewBand = 9;  
                        }
                        if (cmd1 == 248) {     // Jump to Band 3.4G
                            Button_B = YES;
                            NewBand = 8;  
                        }
                        if (cmd1 == 247) {     // Jump to Band 2.3G
                            Button_B = YES;
                            NewBand = 7;  
                        }
                        if (cmd1 == 246) {     // Jump to Band 1296
                            Button_B = YES;
                            NewBand = 6;  
                        }
                        if (cmd1 == 245) {     // Jump to Band 902
                            Button_B = YES;
                            NewBand = 5;  
                        }
                        if (cmd1 == 244) {     // Jump to Band 432
                            Button_B = YES;
                            NewBand = 4;  
                        }
                        if (cmd1 == 243) {     // Jump to Band 222
                            Button_B = YES;
                            NewBand = 3;  
                        }
                        if (cmd1 == 242) {     // Jump to Band 144
                            Button_B = YES;
                            NewBand = 2;  
                        }
                        if (cmd1 == 241) {     // Jump to Band 50
                            Button_B = YES;
                            NewBand = 1;  
                        }
                        if (cmd1 == 240) {     // Jump to Band HF
                            Button_B = YES;
                            NewBand = 0;  
                        }
                        if (cmd1 == 239) {     // Toggle Serial power data outpout.  Other serial functions remain available.
                            toggle_ser_data_output(0);
                        }
                        if (cmd1 == 193) {    // Set up for potential EEPROM Reset if followed by 5 second press on Button C
                            Reset_Flag = 1;
                            print_Cmd_Progress(1);   // Use when there is a start and stop with delay the remote needs to know about
                        }
                        if (cmd1 == 194) {     // Set Reset EEPROM flag.  Will repopulate after CPU reset
                            if (Reset_Flag == 1) 
                                reset_EEPROM();
                            Reset_Flag = 0;   
                            print_Cmd_Progress(0);
                        }
                        if (cmd1 == 195) {    // Save to EEPROM - Usually following cal changes from Remote meter app that are not committed yet                          
                            Cal_Table_write();
                            EEPROM_Init_Write();    // save to eeprom on changes.  
                            EEPROM_Init_Read();     // load the values into memory to check them
                            Cal_Table();  // read back to be sure it save correctly                          
                           // Probably use a sw wd to do a software remote reset for PSoC5.
                        }                            
                        // Handle remote command to change stored coupling factor to support headless ops.
                        // TODO: Need to write into EEPROM, either here or by changing bands.                          
                        if (cmd1 >= 100 && cmd1 < 120) {     // Change Fwd coupling factor for Port X
                            int index;
                            index = cmd1 - 100;
                            Band_Cal_Table[index].Cpl_Fwd = cmd2;       // cmd2 is second byte in 2 byte payload.                              
                            EEPROM_Init_Write();    // save to eeprom
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();
                        }
                        if (cmd1 >= 120 && cmd1 < 140) {     // Change Ref coupling factor for Port X
                            int index;
                            index = cmd1 - 120;
                            Band_Cal_Table[index].Cpl_Ref = cmd2;       // cmd2 is second byte in 2 byte payload.                              
                            EEPROM_Init_Write();    // save to eeprom on changes.  
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();
                        }  
                        if (cmd1 >= 140 && cmd1 < 160) {     // Change Ref coupling factor for Port X by + cmd2 value
                            int index;
                            index = cmd1 - 140;
                            Band_Cal_Table[index].Cpl_Ref = Band_Cal_Table[index].Cpl_Ref + cmd2;       // cmd2 is second byte in 2 byte payload.                              
                            EEPROM_Init_Write();    // save to eeprom on changes.  
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();
                        }                                                                                                  
                        if (cmd1 >= 160 && cmd1 < 180) {     // Change Ref coupling factor for Port X by - cmd2 value
                            int index;
                            index = cmd1 - 160;
                            Band_Cal_Table[index].Cpl_Ref = Band_Cal_Table[index].Cpl_Ref - cmd2;       // cmd2 is second byte in 2 byte payload.                              
                            EEPROM_Init_Write();    // save to eeprom on changes.  
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();
                        }
                        if (cmd1 == 96) {     // Jump to Band HF
                           digitalWrite(Nextion_Switch, 1);    // Switch serial from display to USB UART converter                                                                      
                        }
                        if (cmd1 == 95) {     // Jump to Band HF
                           digitalWrite(Nextion_Switch, 0);    // Switch serial from display to CPU                                                        
                        }
                        if (cmd1 == 94) {    // This is cal routine. cmd is target Fwd value.  uses Attenuaor value adjustment to set cal.  Depricated.
                            //if higher than cmd 2 decrement, do adread(), check and adj again until a near match.
                            // Challenge might be impossible to get an exact match, so need close enough decision logic.
                            for (i = 0; i< 100; i++)   // put a limit on to ensure we do not have an endless loop if we cannot get a match within the window
                            {
                                if (FwdPwr > cmd2+4.0 && FwdPwr > 10.0) // Coarse tune
                                {
                                    // decrement 
                                    CouplingFactor_Fwd -= 0.5;
                                    adRead();  // get new values                                
                                    print_Cal_Table_progress(1);
                                }
                                else if (FwdPwr > cmd2+0.1) // coarse tune range
                                {
                                    // decrement 
                                    CouplingFactor_Fwd -= 0.01;
                                    adRead();  // get new values                                
                                    print_Cal_Table_progress(1);
                                }
                                else if (FwdPwr < cmd2-4.0 && FwdPwr > 10.0)
                                {
                                    // increment
                                    CouplingFactor_Fwd += 0.5;
                                    adRead();  // get new values                                
                                    print_Cal_Table_progress(1);
                                }  
                                else if (FwdPwr < cmd2-0.1)
                                {
                                    // increment
                                    CouplingFactor_Fwd += 0.01;
                                    adRead();  // get new values                                
                                    print_Cal_Table_progress(1);
                                }  
                                else    
                                {
                                    i=100;   // exit the loop                                 
                                    // if neither of the above then assume we are close enough and move on using the current value
                                    //Cal_Table();   // update the current band values for adRead to use 
                                    print_Cal_Table_progress(0);
                                    sprintf((char *) tx_buffer,"*** FwdPwr=%.2f   Loop count =%d ***\r\n", FwdPwr, i);   // debug message to Serial USB
                                    serial_usb_write();
                                    Cal_Table_write(); 
                                    //EEPROM_Init_Write();    // save to eeprom
                                    //EEPROM_Init_Read();     // load the values into memory
                                    Cal_Table();  // read it back                                   
                                }  
                                adRead();
                            }
                        }
                        if (cmd1 == 93) {    // This is an alternative auto cal routine changing the attenuation only. cmd is target Ref value.                         
                            for (i = 0; i< 100; i++)   // put a limit on to ensure we do not have an endless loop if we cannot get a match within the window
                            {
                                if (RefPwr > cmd2+1.0) // Fine tune
                                {
                                    // decrement 
                                    CouplingFactor_Ref -= 0.5;
                                    adRead();  // get new values                                                                   
                                    serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }
                                else if (RefPwr > cmd2+0.1) // coarse tune range
                                {
                                    // decrement 
                                    CouplingFactor_Ref -= 0.01;
                                    adRead();  // get new values                                                                    
                                    serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }
                                else if (RefPwr < cmd2-1.0)
                                {
                                    // increment
                                    CouplingFactor_Ref += 0.5;
                                    adRead();  // get new values                                
                                    serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }  
                                else if (RefPwr < cmd2-0.1)
                                {
                                    // increment
                                    CouplingFactor_Ref += 0.01;
                                    adRead();  // get new values                                
                                    serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }  
                                else    
                                {
                                    i=100;   // exit the loop                                 
                                    // if neither of the above then assume we are close enough and move on using the current value
                                    print_Cal_Table_progress(0);  // end of loop
                                    sprintf((char *) tx_buffer,"*** RefPwr=%.2f   Loop count =%d ***\r\n", RefPwr, i);   // debug message to Serial USB                                    
                                    serial_usb_write();
                                    Cal_Table_write(); 
                                    //EEPROM_Init_Write();    // save to eeprom
                                    //EEPROM_Init_Read();     // load the values into memory
                                    Cal_Table();  // read it back                                    
                                }                               
                            }
                        }
                        if (cmd1 == 92) 
                        {    // Get Slope from user (if needed, normally calculated)
                            Slope_F = cmd2;                            
                        }
                        if (cmd1 == 91) 
                        {    // Get Slope from user (if needed, normally calculated)
                            Slope_R = cmd2;                                           
                        }                                                                                     
                        if (cmd1 == 90) 
                        {    // Get Offset from user (if needed, normally calculated)
                            Offset_F = cmd2;                            
                        }
                        if (cmd1 == 89) 
                        {    // Get Offset from user (if needed, normally calculated)
                            Offset_R = cmd2;
                        }  
                        if (cmd1 == 88) 
                        {    // Get hv target value from user
                            float hv_target;
                            hv_target = cmd2;
                            if (hv_target == 0)
                                hv_cal_factor = hv_read();
                            else
                            hv_cal_factor = hv_target/hv_read();
                        }  
                        if (cmd1 == 87) 
                        {    // Get Offset from user
                            float v14_target;
                            v14_target = cmd2;
                            if (v14_target == 0)
                                v14_cal_factor = v14_read();
                            else
                                v14_cal_factor = v14_target/v14_read();
                        }  
                        if (cmd1 == 86)
                        {    // Get actual externally measured curr value from user (if needed, normally calculated)
                            float curr_target;
                            curr_target = cmd2;
                            if (curr_target == 0)
                                curr_cal_factor = curr_read() - curr_zero_offset;
                            else
                                curr_cal_factor = curr_target/(curr_read() - curr_zero_offset);
                        }
                        if (cmd1 == 85) 
                        {   // Set the 0 current reference point. ACS712 is a AC/DC sensor so it will be 2.5V at no load, 0-2.5
                            // for negative and 2.5 to 5V for positive current flow.
                            // Cmd2 value is used to fine tune result if needed for a accurate 0, normally it should be 0 and measured with no load or power off.
                            // Other sensors might be like the ACS715 whcih are DC only and output 0-5V range.  This will handle either type.
                            float curr0_target;
                            curr0_target = cmd2;
                            curr_zero_offset = curr_read() + curr0_target;  // Read the sensor voltage when there is 0 load on the current source sensor to establish the 0 point.
                        }   
                        if (cmd1 == 84) 
                        {    // Get Offset from user (if needed, normally calculated)
                            float temp_target;
                            temp_target = cmd2;
                            if (temp_target > 0)
                                temp_cal_factor = temp_target/temp_read();
                            else
                                temp_cal_factor = 1;  // a value of 0 is signal to reset to no correction factor
                        }  
                        if (cmd1 == 79) 
                        {    // Get target hi power level from user in Watts, convert to dBm then measure and save ADC                            
                            Pwr_hi = cmd2;    
                            Pwr_hi = 10 * log10(1000*(Pwr_hi/1));
                            adRead();  // update our high power readings for foprward and reflected power for the current band
                            FwdVal_hi = FwdVal; // save for calc                            
                            RefVal_hi = RefVal;  // save for calc    
                            print_Cal_Table_progress(1);  // end of loop
                        }
                        if (cmd1 == 78) 
                        {    // Get target hi power level from user in dBm then measure and save ADC                            
                            Pwr_hi = cmd2;                                
                            adRead();  // update our high power readings for foprward and reflected power for the current band
                            FwdVal_hi = FwdVal; // save for calc                            
                            RefVal_hi = RefVal;  // save for calc    
                            print_Cal_Table_progress(1);  // end of loop
                        }
                        if (cmd1 == 77) 
                        {    // Get target lo power level from user in Watts, convert to dBm then measure and save ADC                            
                            Pwr_lo = cmd2;    
                            Pwr_lo = 10 * log10(1000*(Pwr_lo/1));
                            adRead();  // update our high power readings for foprward and reflected power for the current band
                            FwdVal_lo = FwdVal; // save for calc                            
                            RefVal_lo = RefVal;  // save for calc    
                            print_Cal_Table_progress(1);  // end of loop
                        }
                        if (cmd1 == 76) 
                        {    // Get target power lo level from user in dBm then measure and save ADC                            
                            Pwr_lo = cmd2;                            
                            adRead();  // update our high power readings for foprward and reflected power for the current band
                            FwdVal_lo = FwdVal; // save for calc                            
                            RefVal_lo = RefVal;  // save for calc    
                            print_Cal_Table_progress(1);  // end of loop
                        }
                        if (cmd1 == 75) // Fwd Direction Calc
                        {   // Use stored values of Hi and lo power for Fwd direction
                            if (Pwr_hi != 0) // ensure we have a valid high power number before calculating.
                            {                      
                                // Calculate Slope and Offset
                                Slope_F = (FwdVal_hi - FwdVal_lo)/(Pwr_hi - Pwr_lo);
                                Offset_F = FwdVal_hi + ((CouplingFactor_Fwd - Pwr_hi) * Slope_F);
                                Cal_Table_write();
                                //EEPROM_Init_Write();    // save to eeprom on changes.  
                                //EEPROM_Init_Read();     // load the values into memory to check them
                                Cal_Table();  // read back to be sure it save correctly                                
                                print_Cal_Table_progress(0);  // end of loop
                            }
                        }
                        if (cmd1 == 74) //Ref Direction Calc
                        {    // Use stored values of Hi and lo power for Ref direction
                            if (Pwr_hi != 0) // ensure we have a valid high power number before calculating.
                            {                             
                                // Calculate Slope and Offset
                                Slope_R = (RefVal_hi - RefVal_lo)/(Pwr_hi - Pwr_lo);
                                Offset_R = RefVal_hi + ((CouplingFactor_Ref - Pwr_hi) * Slope_R);
                                Cal_Table_write();
                                //EEPROM_Init_Write();    // save to eeprom on changes.  
                                //EEPROM_Init_Read();     // load the values into memory to check them
                                Cal_Table();  // read back to be sure it save correctly
                                print_Cal_Table_progress(0);  // end of loop
                            }
                        }
                    } // end of msg_type 120                                      
                } // end of msg_type length check
            } // end of meter ID OK    
        } // end of \n found
    } //end rx_count not 0
} // end get_remote_cmd function

void Cal_Table()
{
    // Each coupler has unique coupling factor. 
    CouplingFactor_Fwd = Band_Cal_Table[CouplerSetNum].Cpl_Fwd;  // value in dB from coupler specs.  
    CouplingFactor_Ref = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
    Slope_F = Band_Cal_Table[CouplerSetNum].Slope_F;
    Offset_F = Band_Cal_Table[CouplerSetNum].Offset_F;
    Slope_R = Band_Cal_Table[CouplerSetNum].Slope_R;
    Offset_R = Band_Cal_Table[CouplerSetNum].Offset_R;
}
    
void Cal_Table_write(void)
{
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd = CouplingFactor_Fwd;  // value in dB from coupler specs.  
    Band_Cal_Table[CouplerSetNum].Cpl_Ref = CouplingFactor_Ref;    
    Band_Cal_Table[CouplerSetNum].Slope_F = Slope_F;
    Band_Cal_Table[CouplerSetNum].Offset_F = Offset_F;
    Band_Cal_Table[CouplerSetNum].Slope_R = Slope_R;
    Band_Cal_Table[CouplerSetNum].Offset_R = Offset_R;
}

void print_cal_table()
{
    unsigned char   i;

    // example output format : "101,150,TEXT,55.4,35.4,3.3,2.2"
    // #150 for msg_type field to signal this is data dump, not power levels or other type messages.
    // meterid with msg_type = 150 to signal different data set than the normal output. 120 inbound cmd, 170, power out
    for (i=0; i < NUM_SETS; i++) {        
        sprintf((char *) tx_buffer, "%d,%s,%s,%.1f,%.5f,%.5f,%.1f,%.5f,%.5f\r\n", METERID, "150", Band_Cal_Table[i].BandName, Band_Cal_Table[i].Cpl_Fwd, Band_Cal_Table[i].Slope_F, Band_Cal_Table[i].Offset_F, Band_Cal_Table[i].Cpl_Ref, Band_Cal_Table[i].Slope_R, Band_Cal_Table[i].Offset_R);
        tx_count = strlen((char *) tx_buffer);        
        serial_usb_write();   // Output table text to serial port              
    }
}

/*
Start or end of cal message output to serial port.  1 is start, 0 is end.  Message type is 161 and 160.
 used to adjust dBm levels to match watts target using existing slope and offset.
*/
void print_Cal_Table_progress(unsigned char flag)
{   // uses current Offset and slope
    flag +=160;   // translate to start or end of cal procedure message type.   161 start, 160 end
    // example output format : "101,150,TEXT,55.4,35.4,3.3,2.2"
    // #161/160 for msg_type field to signal this is a cal calculation in progress
    //sprintf((char *) tx_buffer, "%d,%d,%s,%.2f,%.4f,%.2f,%.4f\r\n", METERID, flag, Band_Cal_Table[CouplerSetNum].BandName, CouplingFactor_Fwd, FwdVal, CouplingFactor_Ref, RefVal);        
    sprintf((char *) tx_buffer, "%d,%d,%s,%.5f,%.5f,%.5f,%.5f\r\n", METERID, flag, Band_Cal_Table[CouplerSetNum].BandName, FwdVal_hi, FwdVal_lo, RefVal_hi, RefVal_lo);
    tx_count = strlen((char *) tx_buffer);
    serial_usb_write();   // Output table text to serial port                  
}

/*
Start or end of cal message output to serial port.  1 is start, 0 is end.  Message type is 161 and 160.
 used to adjust dBm levels to match watts target using existing slope and offset.
*/
void print_Cmd_Progress(unsigned char flag)
{   // uses current Offset and slope
    flag +=162;   // translate to start or end of cal procedure message type.   163 start, 162 end
    // example output format : "101,162,TEXT"
    // #163/162 for msg_type field to signal this is data dump, not power levels or other type messages.
    // Send out raw ADC voltage readings
    sprintf((char *) tx_buffer, "%d,%d,%s,%.5f,%.5f,%.5f,%.5f\r\n", METERID, flag, Band_Cal_Table[CouplerSetNum].BandName, FwdVal_hi, FwdVal_lo, RefVal_hi, RefVal_lo);        
    tx_count = strlen((char *) tx_buffer);
    serial_usb_write();   // Output table text to serial port                  
}

// read cal data from EEPROM (state is another function)
void read_Cal_Table_from_EEPROM()
{
    int   i;
    int   j;  
    unsigned char  eepromArray[sizeof Band_Cal_Table];
    int   Arr_Size;
    char  setpoint_buf[SETPOINT_LEN+1];
    int   len_ee;
   
    Serial.println("read_Cal_Table_from_EEPROM - Start");

    for(j=0; j<EEADDR; j++)
        {              
             EEPROM.get(j, eepromArray[j]);             //  address relative to 0x0000
        }  
        // Now extract into each variable
                                                                                // byte 1 of 0-15 is reserved guard space
        op_mode = eepromArray[OP_MODE_OFFSET];                                  // byte 2 of 0-15
        CouplerSetNum = eepromArray[COUPLERSETNUM_OFFSET];                      // byte 3
        ser_data_out = eepromArray[SER_DATA_OUT_OFFSET];                        // byte 4
        memcpy(setpoint_buf,&eepromArray[HV_MAX_OFFSET],SETPOINT_LEN);          // byte 5-8  
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string
        set_hv_max = atof(setpoint_buf);                                        // convert ascii to float
        memcpy(setpoint_buf,&eepromArray[V14_MAX_OFFSET],SETPOINT_LEN);         // byte 9-12       
        setpoint_buf[SETPOINT_LEN] = '\0';
        set_v14_max = atof(setpoint_buf);                                       // convert ascii to float              
        set_temp_max = eepromArray[TEMP_MAX_OFFSET];                            // byte 13 of 0-15
        METERID = eepromArray[METERID_OFFSET];                                  // byte 14 of 15
        //xxx = eepromArray[SPARE0_OFFSET];                                     // byte 15 of 15 

        // start row 1
        memcpy(Callsign,&eepromArray[CALLSIGN_OFFSET],CALLSIGN_LEN);            // byte 0-6 -  first 7 bytes of row 1
        memcpy(setpoint_buf,&eepromArray[CURR_MAX_OFFSET],SETPOINT_LEN);        // byte 7-10 of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminaite string
        set_curr_max = atof(setpoint_buf);                                      // convert ascii to float 
        memcpy(setpoint_buf,&eepromArray[HV_CAL_OFFSET],SETPOINT_LEN);          // byte 1B to 1E of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminaite string
        hv_cal_factor = atof(setpoint_buf);                                     // convert ascii to float 
                
        // start row 2
        memcpy(setpoint_buf,&eepromArray[V14_CAL_OFFSET],SETPOINT_LEN);         // byte 7-10 of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminaite string
        v14_cal_factor = atof(setpoint_buf);                                    // convert ascii to float 
        memcpy(setpoint_buf,&eepromArray[CURR_CAL_OFFSET],SETPOINT_LEN);        // byte 1B to 1E of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminaite string
        curr_cal_factor = atof(setpoint_buf);                                   // convert ascii to float         
        memcpy(setpoint_buf,&eepromArray[TEMP_CAL_OFFSET],SETPOINT_LEN);        // byte 1B to 1E of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminaite string
        temp_cal_factor = atof(setpoint_buf);                                   // convert ascii to float 
        memcpy(setpoint_buf,&eepromArray[CURR_0_OFFSET],SETPOINT_LEN);        // byte 1B to 1E of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminaite string
        curr_zero_offset = atof(setpoint_buf);  

  // Now get the band table struct data   
   for (i=0; i<NUM_SETS; i++) {
      if (i < EEPROM.length())
      {
         len_ee = EEADDR+(sizeof(Band_Cal_Table)*i);
         EEPROM.get(len_ee, Band_Cal_Table[i]);  
         //Serial.print(EEPROM.length());       
         //Serial.println("EEPROM Read");
         Serial.println(Band_Cal_Table[i].BandName);
         Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(10); 
      }
   }
   Serial.println("read_Cal_Table_from_EEPROM - End");
}

// Copy cal data in memory to EEPROM to preserve user changes (save state is another function)
void write_Cal_Table_to_EEPROM()
{
    unsigned int i;
    unsigned int len_ee;
    unsigned char   ee_row;
    unsigned char   c1_array[16]; /* stores 1 row of 16 bytes for eeprom write and reads */
    unsigned int    j;    
    unsigned char   eepromArray[NUM_SETS * (sizeof Band_Cal_Table)];  // 396 bytes
    unsigned int    Arr_Size;
    unsigned char   result;
    unsigned char   setpoint_buf[SETPOINT_LEN+1];

    Serial.println(" Write Cal Table to EEPROM - Start " );
    c1_array[0] = 'S';   /* what we just wrote, will overwrite as a block of 16 bytes) */
    c1_array[EE_RESERVED_BYTE1] = NULL;
    c1_array[OP_MODE_OFFSET] = op_mode;  
    c1_array[COUPLERSETNUM_OFFSET] = CouplerSetNum;
    c1_array[SER_DATA_OUT_OFFSET] = ser_data_out;                     
    // Need to test effect of leading 0s or not
    sprintf(setpoint_buf, "%*.1f", SETPOINT_LEN, set_hv_max);                            // convert float to 4 byte ascii
    memcpy(&c1_array[HV_MAX_OFFSET],setpoint_buf, SETPOINT_LEN);
    sprintf(setpoint_buf, "%*.1f",SETPOINT_LEN, set_v14_max);                           // convert float to 4 byte ascii
    memcpy(&c1_array[V14_MAX_OFFSET],setpoint_buf, SETPOINT_LEN);                                                   
    c1_array[TEMP_MAX_OFFSET] = set_temp_max;                              // byte 13 of 0-15
    c1_array[METERID_OFFSET] = METERID;                                     // byte 14 of 15
    c1_array[SPARE0_OFFSET] = '\0';                                        // byte 15 of 15  
    
    // copy row 0 to EEPROM now     
    ee_row = 0;
    for(j=0; j<16; j++)
    {              
         EEPROM.put(j+(ee_row*16), c1_array[j]);               
    }

    /*
    // Read it back to display for test
    unsigned char tempvar4[20];
    for(j=0; j<16; j++)
    {              
         EEPROM.get(j+(ee_row*16),tempvar4[j]);     
         Serial.print(" Row 0 BYTES  = ");
         Serial.println(tempvar4[j]);     
    }
    */
    memcpy(c1_array,Callsign, CALLSIGN_LEN);   /*  7 bytes */     
 
    sprintf(setpoint_buf, "%02.1f", set_curr_max);                           // convert float to 4 byte ascii
    memcpy(&c1_array[CURR_MAX_OFFSET-0x0010],setpoint_buf, SETPOINT_LEN);
    
    if (hv_cal_factor == 0) 
        hv_cal_factor = 1.0;  
    sprintf(setpoint_buf, "%01.2f", hv_cal_factor);                           // convert float to 4 byte ascii
    memcpy(&c1_array[HV_CAL_OFFSET-0x0010],setpoint_buf, SETPOINT_LEN);
    
    // bytes 15 of 0-15 is spare        
    // Now copy row 1 to EEPROM
    ee_row = 1;
    for(j=0; j<16; j++)
    {              
         EEPROM.put(j+(ee_row*16), c1_array[j]);               
    }
/*
    // Read it back to display for test
   //unsigned char tempvar4[20];
    for(j=0; j<16; j++)
    {              
         EEPROM.get(j+(ee_row*16),tempvar4[j]);     
         Serial.print(" Row 1 BYTES  = ");
         Serial.println(tempvar4[j]);     
    }
*/    
    // Prep and write Row 2 (0x0020 to 2F)
    if (v14_cal_factor == 0) 
        v14_cal_factor = 1.0;
    sprintf(setpoint_buf, "%01.2f", v14_cal_factor);                           // convert float to 4 byte ascii
    memcpy(&c1_array[V14_CAL_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN);
    
    if (curr_cal_factor == 0) 
        curr_cal_factor = 1.0;
    sprintf(setpoint_buf, "%01.2f", curr_cal_factor);                           // convert float to 4 byte ascii
    memcpy(&c1_array[CURR_CAL_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN); 
         
    if (temp_cal_factor == 0) 
        temp_cal_factor = 1.0;
    sprintf(setpoint_buf, "%01.2f", temp_cal_factor);                           // convert float to 4 byte ascii
    memcpy(&c1_array[TEMP_CAL_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN);      
      
    sprintf(setpoint_buf, "%01.2f", curr_zero_offset);    // fill spare byte with test pattern for debug
    memcpy(&c1_array[CURR_0_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN); 
    
    // Now copy row 2 to EEPROM
    ee_row = 2;
    for(j=0; j<16; j++)
    {              
         EEPROM.put(j+(ee_row*16), c1_array[j]);               
    }
/*
    // Read it back to display for test
    for(j=0; j<16; j++)
    {              
         EEPROM.get(j+(ee_row*16),tempvar4[j]);     
         Serial.print(" Row 2 BYTES  = ");
         Serial.println(tempvar4[j]);     
    }
*/  
   for (i=0; i< NUM_SETS; i++) {
      if (i < EEPROM.length()) {
         len_ee = EEADDR+(sizeof(Band_Cal_Table)*i);
         EEPROM.put(len_ee, Band_Cal_Table[i]);       
         Serial.println("EEPROM Written");
         Serial.println(Band_Cal_Table[i].BandName);
         Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(10);
         EEPROM.update(0,'G');
      }
   }    
   Serial.println("Write Cal Table to EEPROM - End " );
}

// Copy hard coded al data to memory
void write_Cal_Table_from_Default()
{
    unsigned int i;
  
    for (i=0;i<NUM_SETS;i++) 
    {
        // Populate initial database scructure in RAM.  Ultimately it needs to be read from EEPROM once initialized
        strcpy(Band_Cal_Table[i].BandName, Band_Cal_Table_Def[i].BandName);
        Band_Cal_Table[i].Cpl_Fwd = Band_Cal_Table_Def[i].Cpl_Fwd;
        Band_Cal_Table[i].Slope_F = Band_Cal_Table_Def[i].Slope_F;
        Band_Cal_Table[i].Offset_F = Band_Cal_Table_Def[i].Offset_F;
        Band_Cal_Table[i].Cpl_Ref = Band_Cal_Table_Def[i].Cpl_Ref;   
        Band_Cal_Table[i].Slope_R = Band_Cal_Table_Def[i].Slope_R;
        Band_Cal_Table[i].Offset_R = Band_Cal_Table_Def[i].Offset_R;
    }  
    //printf("Copied default data info Band Cal Table");
    set_hv_max = HV_MAX; // reset the setpoints and mterid and band
    set_v14_max = V14_MAX;      
    set_curr_max = CURR_MAX;
    set_temp_max = TEMP_MAX;
    METERID = default_METERID;
    CouplerSetNum = 0;
    op_mode=PWR;
    ser_data_out = 1;
    hv_cal_factor=1;          
    v14_cal_factor=1;
    curr_cal_factor=1;        
    temp_cal_factor=1;
    curr_zero_offset=2.5;
    /*  for debug
    Serial.println(" Write Cal From Default");
    Serial.println(op_mode);
    Serial.println(CouplerSetNum); 
    Serial.println(ser_data_out);
    Serial.println(set_hv_max);            
    Serial.println(set_v14_max);
    Serial.println(set_temp_max);
    Serial.println(METERID);  
    Serial.println(Callsign);
    Serial.println(set_curr_max);
    Serial.println(hv_cal_factor);          
    Serial.println(v14_cal_factor);
    Serial.println(curr_cal_factor);        
    Serial.println(temp_cal_factor);
    Serial.println(curr_zero_offset);
    */
    Serial.println("write_Cal_Table_from_Default - Completed");
}

// Read state in EEPROM
void get_config_EEPROM()
{
    CouplerSetNum = EEPROM.read(COUPLERSETNUM_OFFSET);
    //Serial.print("CouplerSetNum Read = ");
    //Serial.println(CouplerSetNum);
    op_mode = EEPROM.read(OP_MODE_OFFSET);
    //Serial.print("op_mode Read = ");
    //Serial.println(op_mode);
    //EEPROM.commit();
}

// Save state in EEPROM
unsigned char EE_Save_State()
{
  return 1;
    //char buf[3];
    Serial.println("EE_Save_State - Start");
    CouplerSetNum = constrain(CouplerSetNum,0,NUM_SETS);
    EEPROM.update(COUPLERSETNUM_OFFSET, CouplerSetNum);
    Serial.print("CouplerSetNum Write =");
    Serial.println(CouplerSetNum);
    op_mode = constrain(op_mode,1,3);
    if (op_mode == 1 || op_mode == 2) {
        EEPROM.update(OP_MODE_OFFSET, op_mode);
        Serial.print("op_mode Write = ");
        Serial.println(op_mode);
    }
    EEPROM.update(SER_DATA_OUT_OFFSET, ser_data_out);
    Serial.print("ser_data_out Write =");
    Serial.println(ser_data_out);
    Cal_Table();
    Serial.println("EE_Save_State - End");
    return 1;
}

// Mark EEPROM for overwrite
void reset_EEPROM() 
{
    // Mark EEPROM for overwrite
    if (Reset_Flag ==1 || RESET_EEPROM == 1) 
    {
        // erase byte 0 to force init process for testing.  Can also use to "factory" reset data     
        set_hv_max = HV_MAX;
        set_v14_max = V14_MAX;
        set_curr_max = CURR_MAX;
        set_temp_max = TEMP_MAX;
        hv_cal_factor = 8.18;
        v14_cal_factor = 2.12;
        curr_cal_factor = 1.0;
        temp_cal_factor = 1.0;
        curr_zero_offset = 2.5;
        // erase byte 0 to force init process for testing.  Can also use to "factory" reset data
        EEPROM.update(0, '0');
        write_Cal_Table_from_Default();
        //EEPROM_Init(EE_SAVE_YES);
        printf("Erased Byte 0");
        EEPROM_Init_Read();     // load the values into m
        Cal_Table();
    }
}

// Toggle USB serial output data
void toggle_ser_data_output(char force_on)
{
      if (ser_data_out == 0 || force_on == 1)
      {
        EEPROM.update(SER_DATA_OUT_OFFSET, 1);
        Serial.println("Enabled Serial Data Output");
        ser_data_out = 1;
      }
      else { 
        EEPROM.update(SER_DATA_OUT_OFFSET, 0);
        Serial.println("Disabled Serial Data Output");
        ser_data_out = 0;
      } 
}

unsigned char EEPROM_Init_Write(void)
{
    write_Cal_Table_to_EEPROM();
    return 1;
}

unsigned char EEPROM_Init_Read(void)
{
    read_Cal_Table_from_EEPROM();
    return 1;
}

/* ========================================
 *  OTRSP parsing
 *  Parses serial port commands from N1MM Logging program to control antennas and transverter
 *  via BCD or parallel GPIO port pins.
 *  Uses UART2 which shares the USB UART with the Nextion display if there is one. Desktop app
 *  config page switches between USB com port between the display and UART2 (for N1MM commands)
 *  Only using the Aux commands not the whole SO2R list of commands so not using the whole SO2R
 *  list of possible commands and queries
 *  Created by K7MDL July 27, 2020 for RF Wattmeter to enable antenna switching , transveter
 *  selection, PTT, CW, and band labeling on the LCD, especially for radios with native tranverter support. 
 * ========================================
*/

//-------------------------------------------------------------------

#define FALSE 0
#define TRUE 1
#define AUXCMDLEN 4
#define BANDCMDLEN 12

// Function declarations
void OTRSP_setup(void);
uint8_t OTRSP(void);
uint8_t OTRSP_Process(void);

/*
Convert AUX command from N1MM to 4 bit BCD
Command format is AUXxnn fixed width. x is the radio number, usually 1.   Example is AUX103 for command 03 for BCD output of 0x03.
*/
uint8_t OTRSP(void)
{     
    char c;
    static int i;
    static char AuxCmd0[20];
    
    //AuxNum1 = AuxNum2 = 0;  // Global values also used to update status on LCD
    if (SerialUSB1.available() > 0)
    {
        c = SerialUSB1.read();
        if ( (c == 'A' && AuxCmd0[0] != 'B') || c == 'B')   // AUXxYY\r or BANDxYYYYY...\r
        {           
            i = 0;
        }
        AuxCmd0[i++] = c;
        AuxCmd0[i] = '\0';
        //SerialUSB1.print("AuxCmd0 = ");
        //SerialUSB1.println(AuxCmd0);
          
        if ( (i == 7) && ((AuxCmd0[0] == 'A') || (AuxCmd0[0] == 'B')) && (AuxCmd0[6] == '\r') )
        {
            AuxCmd0[6]='\0';  // Terminate the string (no \r)
            //SerialUSB1.println(AuxCmd0);
            OTRSP_Process(AuxCmd0);         // Got a proper string start and end
            i=8;
        }
        else if ( i > 7 || ( i <= 7 && c == '\r') || ( i == 7 && c != '\r')  )   // bail and reset string capture if too long or has a CR too early
        {
            //SerialUSB1.println(i);  
            i = 0;   // reset capture string
            AuxCmd0[0] = '\0';            
            return 0;
        }
        else if ( i < 7 )   // still collecting chars, go back to the well for more chars.
        {
            return 0;
        }
        else 
        {
            SerialUSB1.print(" Should not be here! i = ");
            SerialUSB1.print(i);
        }
    }
    return 0;
}

uint8_t OTRSP_Process(char * AuxCmd0)
{
    //char AuxCmd0[20];
    char AuxCmd1[AUXCMDLEN], AuxCmd2[AUXCMDLEN];
    char BndCmd1[BANDCMDLEN], BndCmd2[BANDCMDLEN];
    uint8_t i;
    
    // Now have a full 6 char string starting with A or B
    SerialUSB1.print(" OTRSP Processing string : ");
    SerialUSB1.println(AuxCmd0); 
    if ((AuxCmd0[0] == 'A' || AuxCmd0[0] == 'B') && AuxCmd0[6] == '\0' && strlen(AuxCmd0)==6) // double validate
    {      
        i=0;
        // Looking only for 2 commands, BAND and AUX.   
        if (strncmp(AuxCmd0,"AUX1",4) == 0)   // process AUX1 where 1 is the radio number.  Each radio has a 4 bit BCD value
        {
            AuxCmd1[0] = AuxCmd0[4];
            AuxCmd1[1] = AuxCmd0[5];
            AuxCmd1[2] = AuxCmd0[6];
            AuxCmd1[3] = '\0';                    
            AuxNum1 = atoi(AuxCmd1);   // Convert text 0-255 ASCII to int
            //Aux1_Write(AuxNum1);  // write out to the Control register which in turn writes to the GPIO ports assigned.                      
            for (i=0; i < 20; i++)
            {
                AuxCmd0[i] = '\0';
                delay(1);
            }
            return(1);  // 1 signals a change
        }
        else if (strncmp(AuxCmd0,"AUX2",4) == 0)   // process AUX comand for radio 2.
        {
            AuxCmd2[0] = AuxCmd0[4];
            AuxCmd2[1] = AuxCmd0[5];
            AuxCmd1[2] = AuxCmd0[6];
            AuxCmd2[3] = '\0';
            AuxNum2 = atoi(AuxCmd2);   // Convert 0-15 ASCII to int
            //Aux2_Write(AuxNum2);  // write out to the Control register which in turn writes to the GPIO ports assigned.                                        
            for (i=0; i< 20; i++)
                AuxCmd0[i] = '\0' ;
            return(1);  // AuxCmd2 now has a translated value - return for band change at meter
        } // Look for BAND commands from N1MM - so far have not seen any - these are just for catching them, they cause no changes
        else if (strncmp(AuxCmd0,"BAND1",5) == 0)   // process AUX1 where 1 is the radio number.  Each radio has a 4 bit BCD value
        {
            // This will be the bottom frequency (in MHz) of the current radio band.  ie 3.5MHz for 3875KHz
            sprintf(BndCmd1,"%s", &AuxCmd0[5]);
            AuxCmd0[0]='\0';
            SerialUSB1.print(" OTRSP Processing BAND 1 string : ");
            SerialUSB1.println(BndCmd1); 
            return(0);  // TODO = passing band MHZ to a CouplerNUM  Search Band values
        }
        else if (strncmp(AuxCmd0,"BAND2",5) == 0)   // process AUX comand for radio 2.
        {
            sprintf(BndCmd2,"%s", &AuxCmd0[5]);
            AuxCmd0[0] = '\0' ;
            SerialUSB1.print(" OTRSP Processing BAND 2 string : ");
            SerialUSB1.println(BndCmd2); 
            return(0); 
        }
    }  
    return 0;   // nothing processed 0 is a valid band number so using 255.
}

/* [] END OF FILE */