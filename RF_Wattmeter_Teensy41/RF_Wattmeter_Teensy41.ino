#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h> 
//#include <usb_serial.h>   // not required for Teensy4.1 but maybe other Arduinos?
#include "RF_Wattmeter_Arduino.h"
/*
 *
 * RF Power Meter by K7MDL 12/4/2020   - Remote (Headless) Edition for Testing on Arduino Teensy 4.1
 *
 * 12/4/2020 -  Ported from PSoC5 C code. All seems to be working except OLED and band decoder output features which need to be finished porting over. 
 * Switched to standard Arduino Nextion Library with a few mods for nex_listen function to detect page number on nav button presses
 * and to resolve compiler warnings. OTRSP uses SerialUSB1 obbained by setting the Teensy USB port to Dual Ports setting.
 *
*/

//struct Band_Cal Band_Cal_Table[10];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#define RESET_EEPROM 0

// #define SerialUSB1 SerialUSB1  // assign as needed for your CPU type. 
                                  // For Teesny set ports to Dual or Triple and Use Serial USB1 and Serial USB2 for 2 and 3rd ports

void setup(void) 
{
  // Set up our input pins
  pinMode(ADC_FWD,INPUT);
  pinMode(ADC_REF,INPUT);
  pinMode(ADC_TEMP,INPUT); 
  pinMode(ADC_CURR,INPUT);
  pinMode(ADC_14V,INPUT);
  pinMode(ADC_HV,INPUT);
  pinMode(BAND_DEC_IN_0, INPUT);
  pinMode(BAND_DEC_IN_1, INPUT);
  pinMode(BAND_DEC_IN_2, INPUT);
  pinMode(BAND_DEC_IN_3, INPUT);
  pinMode(BAND_DEC_IN_4, INPUT);
  pinMode(BAND_DEC_IN_5, INPUT);

  // Set up our output pins, starting with initializing the CW and PTT control pins
  #ifdef TEENSY4_CW_PTT
  pinMode(CW_KEY_OUT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(CW_KEY_OUT), CW_KEY_OUT_ISR, CHANGE);
  CW_KEY_OUT_state = 0; 
  digitalWrite(CW_KEY_OUT, LOW);
  
  pinMode(PTT_OUT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PTT_OUT), PTT_OUT_ISR, CHANGE);
  PTT_OUT_state = 0;
  digitalWrite(PTT_OUT, LOW);
  #endif
  
  // now our other IO pins
  pinMode(BAND_DEC_A_0, OUTPUT);   // Band Decoder bank A pin (bit) 0
  pinMode(BAND_DEC_A_1, OUTPUT);   // Band Decoder bank A pin (bit) 1
  pinMode(BAND_DEC_A_2, OUTPUT);
  pinMode(BAND_DEC_A_3, OUTPUT);
  pinMode(BAND_DEC_A_4, OUTPUT);
  pinMode(BAND_DEC_A_5, OUTPUT);
  pinMode(BAND_DEC_A_6, OUTPUT);
  pinMode(BAND_DEC_A_7, OUTPUT);
  pinMode(BAND_DEC_B_0, OUTPUT);   // Band Decoder bank B pin (bit) 0
  pinMode(BAND_DEC_B_1, OUTPUT);
  pinMode(BAND_DEC_B_2, OUTPUT);
  pinMode(BAND_DEC_B_3, OUTPUT);
  pinMode(BAND_DEC_B_4, OUTPUT);
  pinMode(BAND_DEC_B_5, OUTPUT);
  pinMode(BAND_DEC_B_6, OUTPUT);
  pinMode(BAND_DEC_B_7, OUTPUT);
  pinMode(BAND_DEC_C_0, OUTPUT);  // Band Decoder bank C pin (bit) 0
  pinMode(BAND_DEC_C_1, OUTPUT); 
  pinMode(BAND_DEC_C_2, OUTPUT);
  pinMode(BAND_DEC_C_3, OUTPUT); 
  pinMode(BAND_DEC_C_4, OUTPUT); 
  pinMode(BAND_DEC_C_5, OUTPUT); 
  pinMode(BAND_DEC_C_6, OUTPUT); 
  pinMode(BAND_DEC_C_7, OUTPUT);

  // Initial our serial ports
#ifdef NEXTION
  pinMode(SERIAL1_RX_PIN, INPUT);   // initializing for Nextion or other usage
  pinMode(SERIAL1_TX_PIN, OUTPUT);
#endif  
  OTRSP_Serial.begin(9600);  // open port for OTRSP serial port command input    
  RFWM_Serial.begin(115200); // For debug or data output
  RFWM_Serial.println(" ");   // Clear our output text from CPU init text

#ifdef SSD1306_OLED
    //display_init(DISPLAY_ADDRESS); // This line will initialize your display using the address you specified before.

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))   // Address 0x3D for 128x64
    { 
      for(;;) // Don't proceed, loop forever
        RFWM_Serial.println("SSD1306 allocation failed");
    }
    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds
    // display.display() is NOT necessary after every single drawing command,
    // unless that's what you want...rather, you can batch up a bunch of
    // drawing operations and then update the screen all at once by calling
    // display.display().

    // Clear the buffer
    display.clearDisplay();
#endif

  write_Cal_Table_from_Default();  // Copy default values into memory in case EEPROM not yet initialized
      /*   initialize EEPROM storage if not done before - will overwrite default memory table values from EEPROM if EEPROM was written to before */    
  if (EEPROM.read(0) == 'G') 
  {
    RFWM_Serial.println("EEPROM Data is Valid");
    EEPROM_Init_Read();
    //get_config_EEPROM();  // set last used values for CouplerSetNum (Band) and op_mode if there is data in the EEPROM
  }
 
  if (EEPROM.read(0) != 'G' || RESET_EEPROM == 1) 
  {    // Test if EEPROM has been initialized with table data yet
    RFWM_Serial.println("Write EEPROM");
    write_Cal_Table_from_Default();  // Copy default values into memory
    EEPROM_Init_Write(); // Copy memory into EEPROM if EEPROM is not initialized yet. Byte 0 will get marked with a 'G'
    EE_Save_State();   // use default values to populate state storage area in EEPROM (first 16 bytes reserved for state variables)
    toggle_ser_data_output(1);   // force set data output on
    // Read EEPROM
    EEPROM_Init_Read();  // read stored state cal data from EEPROM into memory
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
    nexSeriale.begin(NexSerialBAUD);   // set in RF wattmter_Arduino.h - must match the Nextion display BAUD or BAUDS parameter set on Main Page
    nexInit();
    /*
    // Set up objects to monitor touch controls from Nextion Display 
    */
    // Set up Callback mappings
    savecfg_btn_1.attachPop(savecfg_btn_1_pop_Callback, 0);   // all 3 page save cfg buttons do the same thing
    savecfg_btn_1.attachPush(savecfg_btn_1_push_Callback, 0);  // so call the same functions
    savecfg_btn_2.attachPop(savecfg_btn_1_pop_Callback, 0);
    savecfg_btn_2.attachPush(savecfg_btn_1_push_Callback, 0);
    savecfg_btn_4.attachPop(savecfg_btn_1_pop_Callback, 0);
    savecfg_btn_4.attachPush(savecfg_btn_1_push_Callback, 0);
    fwd_cal.attachPop(fwd_cal_pop_Callback, 0);
    ref_cal.attachPop(ref_cal_pop_Callback, 0);
    toMain.attachPop(toMain_push_Callback, 0);
    toMain1.attachPop(toMain_push_Callback, 0); // go to same callback as Main
    toConfig.attachPop(toConfig_pop_Callback, 0);
    toSet1.attachPop(Set1_Callback, 0); 
    toPwrGraph.attachPop(toPwrGraph_Callback, 0); 
    f_att_minus.attachPop(f_att_minus_pop_Callback, 0);
    f_att_plus.attachPop(f_att_plus_pop_Callback, 0);
    r_att_minus.attachPop(r_att_minus_pop_Callback, 0);
    r_att_plus.attachPop(r_att_plus_pop_Callback, 0);
    FactoryRst1.attachPop(FactoryRst1_pop_Callback, 0);
    FactoryRst2.attachPop(FactoryRst2_pop_Callback, 0);
    meterID_adj.attachPop(meterID_adj_pop_Callback, 0);
    band_set_adj.attachPop(band_set_adj_pop_Callback, 0);
    //band.attachPop(band_pop_Callback, 0);
    hv_adj.attachPop(hv_adj_pop_Callback, 0);
    v14_adj.attachPop(v14_adj_pop_Callback, 0);
    curr_adj.attachPop(curr_adj_pop_Callback, 0);
    temp_adj.attachPop(temp_adj_pop_Callback, 0);
    Measure_hi.attachPop(Measure_hi_pop_Callback, 0);
    Measure_lo.attachPop(Measure_lo_pop_Callback, 0);
    CalcFwd.attachPop(CalcFwd_pop_Callback, 0);
    CalcRef.attachPop(CalcRef_pop_Callback, 0);
    B_HF.attachPop(BandSelect_HF_pop_Callback, 0);
    B_50.attachPop(BandSelect_50_pop_Callback, 0);
    B_144.attachPop(BandSelect_144_pop_Callback, 0);
    B_222.attachPop(BandSelect_222_pop_Callback, 0);
    B_432.attachPop(BandSelect_432_pop_Callback, 0);
    B_902.attachPop(BandSelect_902_pop_Callback, 0);
    B_1296.attachPop(BandSelect_1296_pop_Callback, 0);
    B_2_3G.attachPop(BandSelect_2_3G_pop_Callback, 0);
    B_3_4G.attachPop(BandSelect_3_4G_pop_Callback, 0);
    B_5_7G.attachPop(BandSelect_5_7G_pop_Callback, 0);
    B_10G.attachPop(BandSelect_10G_pop_Callback, 0);
   
    pg=0;
    Main.show();
    delay(100);
    sprintf(cmd, " ");  // Clear the RX/TX/CW satus field until we get a valid OTRSP command otherwise
    PTT_CW.setText(cmd);  // Get PTT first.  If CW present then next line will overwrite
    PTT_CW.Set_font_color_pco(2016);  // Set text to LT Green for RX
    PTT_CW.Set_background_color_bco(0);  // Set background color to black for RX
    band_set_adj.setMaxval((uint32_t) NUM_SETS);  // Ensure the control know the same num bands the host does.
    //Set1_Callback(0);  // initialize our setpoints and meter id so it does not get read back as 0 values later.
    band.setText(Band_Cal_Table[CouplerSetNum].BandName);
    strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
    Set1_Bandvar.setText(cmd);    // Update Nextion display for new values
    //Graph_Timer.setCycle(500);   // 300msec for graph update rate.  300 set in display, can override here.
#endif // end Nextion setup section 
  
  adRead(); //get and calculate power + SWR values and display them  
}

// Return the supply voltage in volts.  For ESP32 (M5Stack) 
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

float ADC_RF_Power_CountsTo_Volts(uint32_t counts)
{
  float Volts, VRef;
  VRef = 3.3;      
  Volts = (counts/ADC_COUNTS) * VRef;
  //RFWM_Serial.println(Volts);
  return Volts;
}

void adRead(void)   // A/D converter read function.  Normalize the AD output to 100%.
{
    float a;
    float b;
    uint16_t c;
    float tmp, retry=0;
    uint32_t ad_counts=0;
    uint16_t i;

    // Throttle the time the CPU spends here.   Offset the timing os teh data acquired is just before it is sent out
    Timer_X00ms_InterruptCnt = millis(); 
    if ((Timer_X00ms_InterruptCnt - Timer_X00ms_Last_AD) < 100)   // will use our own timestamp.
        return;   // skip out until greater than 100ms since our last visit here
    Timer_X00ms_Last_AD = Timer_X00ms_InterruptCnt;  // time stamp our visit here.  Do not want to come back too soon.  

__reread:   // jump label to reread values in case of odd result or hi SWR
    
    // Get Reflected Power     
        total_Ref -= readings_Ref[readIndex_Ref];// subtract the last reading:    
    c = 1; // read from the sensor:
    a = 0;
    for (i = 0; i < c; ++i)  {
        // ADC_RF_Power_SetGain(1);  // can be used to tweak in the ADC                    
        ad_counts = analogRead(ADC_REF);  // single read but has locked up with N1MM commands at times               
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
        ad_counts = analogRead(ADC_FWD);  // single read but has locked up with N1MM commands       
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
    uint8_t SWR_analog_value = 0;
    
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
    analogWrite(A11, SWR_analog_value);     // Dummy call for now until Band ecoder and DAC outputs are sorted after port from PSoC5 code.
#endif
    SWR_Serial_Val = SWRVal;
    FwdPwr_last = FwdPwr;  // update memory to minimize screen update and flicker on digital number
    sendSerialData();   // send this data to the serial port for remote monitoring
}

void loop() { 

    uint8_t ret1;
    
    while (1) 
    {
        // Listen for remote computer commands                
        serial_usb_read();  // fetch latest data in buffer                                      
        if (rx_count!=0)
            get_remote_cmd();       // scan buffer for command strings

        ret1 = OTRSP();   // set Aux output pins and change bands to match
        if (ret1)  // True if we got commands
        {
            ret1 = OTRSP_Process();  
            if (ret1)   // True if we got a valid AUX or BAND command
            {
                Button_B = YES;  // Process any N1MM Aux port commands on UART
                NewBand = AuxNum1;  // Use AUX 1 to change the wattmeter band
                //Band_Decode_A_Output(AuxNum1);
                //RFWM_Serial.print(" New Band # = ");
                //RFWM_Serial.println(NewBand);              
                //Band_Decode_B_Output(AuxNum1);   // zero the output pins for port A
                //RFWM_Serial.print(" Auxnum1 = ");
                //RFWM_Serial.println(AuxNum1);
                //Band_Decode_C_Output(AuxNum2);   // zero the output pins for port B
                //RFWM_Serial.print(" Auxnum2 = ");
                //RFWM_Serial.println(AuxNum2);
            }
        }     
            
        if (Band_Decoder())       // Process any radio band decoder changes 
        {
            Button_B = YES;
            NewBand = Band_Dec_In_Byte;
        }
        
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
            // Now set the the Group A output pins according to the stored pattern
            Band_Decode_A_Output(CouplerSetNum);   // Set outputs for the new band
            Band_Decode_B_Output(AuxNum1);
            Band_Decode_C_Output(AuxNum2);
            RFWM_Serial.print(" CouplerSetNum = ");
            RFWM_Serial.println(CouplerSetNum);
            
#ifdef NEXTION
            strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
            Set1_Bandvar.setText(cmd);    // Update Nextion display for new values            
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
        
        if (CW_KEY_OUT_state != CW_KEY_OUT_state_last)
        {
            if (CW_KEY_OUT_state)
                RFWM_Serial.write("ISR - CW KEY ON");
            else
                RFWM_Serial.write("ISR - CW KEY OFF");
            CW_KEY_OUT_state_last = CW_KEY_OUT_state;
        }   
        
        if (PTT_OUT_state != PTT_OUT_state_last)   
        {
          if (PTT_OUT_state)
                RFWM_Serial.write("ISR - PTT ON");
            else
                RFWM_Serial.write("ISR - PTT OFF");
          PTT_OUT_state_last = PTT_OUT_state;
        }
        adRead(); //get and calculate power + SWR values and display them        
        
#ifdef SSD1306_OLED
        // Process OLED Display if used
        OLED();  // Update display for all params
#endif 
        
#ifdef NEXTION
        // Process Nextion Display events
        if (nexSerial.available()) 
          nexLoop(nex_listen_list);  // Process Nextion Display  
        if (NewBand != CouplerSetNum)
            update_Nextion(1);
        else if (!WAIT)   // skip if waiting for response from a diaplsy query to reduce traffic
            update_Nextion(0);    
#endif
                       
    }   // end of while
}

#ifdef TEENSY4_CW_PTT
// N1MM CW and PTT and AUX message handling
// look for DTR and RTS for N1MM control of CW and PTT. 
// Can only happen on USB serial since the external USB UART has only TX and RX lines connected to CPU

// Interrupt Service Routine for CW Key Out pin from OTRSP
void CW_KEY_OUT_ISR(void)
{
    // Check state of USB Serial Port DTR register for N1MM CW keying state
    if (OTRSP_Serial.dtr())                    
    {   
        digitalWrite(CW_KEY_OUT, HIGH);
        CW_KEY_OUT_state = 1;
    }          
    else
    {
       digitalWrite(CW_KEY_OUT, LOW);     // follow DTR state. The Arduino IDE Serial Monitor will raise DTR when open (on the Serial port, not this one.)                       
       CW_KEY_OUT_state = 0;
    }
}

// Interrupt Servivce Routine for PTT pin from OTRSP
void PTT_OUT_ISR(void)
{
    //Check state of USB Serial Port RTS register for N1MM PTT state
    if (OTRSP_Serial.rts())     
    {
        digitalWrite(PTT_OUT, HIGH);  
        PTT_OUT_state = 1;
    }
    else
    {
        digitalWrite(PTT_OUT, LOW);     // Follow RTS state      
        PTT_OUT_state = 0;
    }
}
#endif

#ifdef NEXTION    
void hv_adj_pop_Callback(void *ptr)
{
    uint32_t number;    
 
    hv_adj.getValue(&number);
    // Change meter ID with this info
    set_hv_max = number;
    set_hv_max /= 10.0;
    hv_adj.setValue((uint16_t) (set_hv_max*10));
}

void v14_adj_pop_Callback(void *ptr)
{
    uint32_t number;
    //ptr += 1; // get rid of compiler error about unused ptr    
    v14_adj.getValue(&number);
    // Change meter ID with this info
    set_v14_max = number;
    set_v14_max /= 10.0;
    v14_adj.setValue((uint16_t) (set_v14_max*10));
}

void curr_adj_pop_Callback(void *ptr)
{
    uint32_t number;    
    //ptr += 1; // get rid of compiler error about unused ptr    
    curr_adj.getValue(&number);
    // Change meter ID with this info
    set_curr_max = number;
    set_curr_max /= 10;
    curr_adj.setValue((uint16_t) (set_curr_max*10));

}  

void temp_adj_pop_Callback(void *ptr)
{
    uint32_t number;    
    
    //ptr += 1; // get rid of compiler error about unused ptr    
    temp_adj.getValue(&number);
    // Change meter ID with this info
    set_temp_max = (uint8_t)number;
    temp_adj.setValue((uint8_t) (set_temp_max));
}  
  
void band_set_adj_pop_Callback(void *ptr)
{
    uint32_t number;    
  
    band_set_adj.getValue(&number); 
    NewBand = (uint8_t) number;   // get input for new band from Set1 page
    Button_B = YES; // set flag for main loop to process band change request.  
    // If there is a connection to radio (via serial or wireless) then it will override this at each update.                                                               
}

void band_pop_Callback(void *ptr) 
{
    char buf[32];     
    
    band.setText(itoa(CouplerSetNum, buf, 10)); 
    pg=5;
}

void BandSelect_HF_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    BandSelect(0);
}

void BandSelect_50_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{    
    BandSelect(1);
}

void BandSelect_144_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{     
    BandSelect(2);
}

void BandSelect_222_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{     
    BandSelect(3);
}

void BandSelect_432_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{    
    BandSelect(4);
}

void BandSelect_902_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{    
    BandSelect(5);
}

void BandSelect_1296_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    BandSelect(6);
}

void BandSelect_2_3G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    BandSelect(7);
}

void BandSelect_3_4G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    BandSelect(8);
}

void BandSelect_5_7G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    BandSelect(9);
}

void BandSelect_10G_pop_Callback(void *ptr)   // Get var BandSel assigned by each band button to set NewBand var
{
    BandSelect(10);
}

void BandSelect(uint8_t sband)   // Get var BandSel assigned by each band button to set NewBand var
{
    char buf[32];
    //BandSel.getValue(&number);        
    NewBand = sband;    // Update Newband to current value.  Will be incremented in button function
    if (NewBand >= NUM_SETS) 
        NewBand = 0;  // cycle back to lowest band
    Button_B = YES; // set flag for main loop to process band change request.  
    // If there is a connection to radio (via serial or wireless) then it will override this at each update.   
    band.setText(itoa(NewBand, buf, 10));
    pg=0;
    update_Nextion(1);
}

void Measure_hi_pop_Callback(void *ptr)
{   
    uint32_t number; 

    //NexButton_setText(&savecfg_button, "Saved Config");
    HPPwrTarget.getValue(&number);
    Pwr_hi = number;
    Units.getValue(&number);
    if (number==0)  // convert to dBm if in Watts (0 value)
    {
        // Get target lo power level from user in Watts, convert to dBm then measure and save ADC                                
           Pwr_hi = 10 * log10(1000*(Pwr_hi/1));
    }  
    // Now value is in dBm proceed
    adRead();  // update our high power readings for forward and reflected power for the current band
    FwdVal_hi = FwdVal; // save for calc     
    HP_F_VDC.setValue((int)(FwdVal_hi*100000));  // display ADC raw voltage on screen
    RefVal_hi = RefVal;  // save for calc    
    HP_R_VDC.setValue((int)(RefVal_hi*100000));
    print_Cal_Table_progress(1);  // end of loop   
}

void Measure_lo_pop_Callback(void *ptr)
{   
    uint32_t number;  

    //NexButton_setText(&savecfg_button, "Saved Config");
    LPPwrTarget.getValue(&number);
    Pwr_lo = number;        
    Units.getValue(&number);
    if (number==0)
    {
        // Get target lo power level from user in Watts, convert to dBm then measure and save ADC                                
           Pwr_lo = 10 * log10(1000*(Pwr_lo/1));
    }
    // Now value is in dBm proceed
    adRead();  // update our high power readings for foprward and reflected power for the current band
    FwdVal_lo = FwdVal; // save for calc  
    LP_F_VDC.setValue((int)(FwdVal_lo*100000));
    RefVal_lo = RefVal;  // save for calc    
    LP_R_VDC.setValue((int)(RefVal_lo*100000));
    print_Cal_Table_progress(1);  // end of loop
}

void CalcFwd_pop_Callback(void *ptr)
{
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
    uint32_t number;    
 
    meterID.getValue(&number);
    // Change meter ID with this info
    //METERID = number;
}   
    
void FactoryRst1_pop_Callback(void *ptr)
{      
    // Sequenctionally pressing thse adn Rst2 hotspot areas is the same as 2 button reset of M5Stack or remote commands 193+195
    Reset_Flag = 1;
}

void FactoryRst2_pop_Callback(void *ptr)
{
    if (Reset_Flag == 1) 
        reset_EEPROM();
        delay(1000);
        toConfig_pop_Callback(0);
        update_Nextion(1);
    Reset_Flag = 0;
}

void savecfg_btn_1_push_Callback(void *ptr)
{    
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
    //NexButton_setText(&savecfg_button, "Saved Config");
    //LED_Write(1);    // write to both ports for KitProg and Main Board onboard LEDs
    //LED_1_Write(1);
}

void fwd_cal_pop_Callback(void *ptr)    // Slider for Fwd Coupler Cal Value
{
    // Read value of slider sent in event and set in Cal table then echo it back to textbox
    fwd_cal.getValue(&rcv_num);
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd = rcv_num;
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update lable with decimal value text
    fwd_cal_num.setText(cmd);    
    Cal_Table();
    toConfig_pop_Callback(0);
}

void ref_cal_pop_Callback(void *ptr) // Slider for Ref Coupler Cal Value
{
    // Read value of slider sent in event and set in Cal table then echo it back to textbox
    ref_cal.getValue(&rcv_num);  
    Band_Cal_Table[CouplerSetNum].Cpl_Ref = rcv_num;    
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update lable with decimal value text
    ref_cal_num.setText(cmd);    
    Cal_Table();
    toConfig_pop_Callback(0);
}

void f_att_minus_pop_Callback(void *ptr) // fine tune slider by 0.1dB
{
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd -= 0.1;
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update lable with decimal value text
    fwd_cal_num.setText(cmd);     
    Cal_Table();
}

void f_att_plus_pop_Callback(void *ptr)
{        
    Band_Cal_Table[CouplerSetNum].Cpl_Fwd += 0.1;    
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update lable with decimal value text
    fwd_cal_num.setText(cmd);    
    Cal_Table();
}

void r_att_minus_pop_Callback(void *ptr)
{  
    Band_Cal_Table[CouplerSetNum].Cpl_Ref -= 0.1;
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update lable with decimal value text
    ref_cal_num.setText(cmd); 
    Cal_Table();
}

void r_att_plus_pop_Callback(void *ptr)
{
    Band_Cal_Table[CouplerSetNum].Cpl_Ref += 0.1;    
    sprintf(cmd, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update lable with decimal value text
    ref_cal_num.setText(cmd); 
    Cal_Table();
}

void toMain_push_Callback(void *ptr)
{
    pg = 0;   // flag Update function to only update when page is active 
    update_Nextion(1);
    //NexPage_show(&page0);
}

void toConfig_pop_Callback(void *ptr)   // Got event to change to Config Page from somewhere
{
    uint32_t number;
    uint8_t ret1;    
    
    // Update the sliders to current values. SLider callbacks wil update after here.
    if (pg!=1)
    {
        strcpy(cmd, "sendme");
        sendCommand(cmd);
        ret1 = recvPageNumber(&number);  
        if (ret1)
          pg = number;  
    }
    else  // ensure we are on the correct page
    {       
        rcv_num = (uint32_t)(Band_Cal_Table[CouplerSetNum].Cpl_Fwd);
        fwd_cal.setValue(rcv_num);  // Update slider knob position  0-100
         
        snprintf(cmd, 21, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Fwd, '\0');  // Update label with decimal value text
        fwd_cal_num.setText(cmd);
      
        // Initialize the sliders to the current band value for Reflected
        rcv_num = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
        ref_cal.setValue(rcv_num);
        
        snprintf(cmd, 20, "%.1fdB%c", Band_Cal_Table[CouplerSetNum].Cpl_Ref, '\0');  // Update label with decimal value text
        ref_cal_num.setText(cmd);   

        // Refresh Band info in case it changes.
        band_cal.setText(Band_Cal_Table[CouplerSetNum].BandName);
    }
}

/*
 *      Setpoints page on graphics display where alarm levels for things like voltage and temp are set.  
 *      These are globals so in theory do nto have to be on teh page to write to them
 *      Includes the unused *ptr so this can be used as a callback function if needed.
*/
void Set1_Callback(void *ptr)
{   
    uint32_t number32;
    
    if (pg != 2)
    {
        strcpy(cmd, "sendme");
        sendCommand(cmd);    
        if (recvPageNumber(&number32))
          pg = (uint8_t) number32;
        else
        {
          sendCommand("sendme");   //retry once first failure         
          if (recvPageNumber(&number32))    // good page num returned
              pg = (uint8_t) number32;    // else just skip this and use the pg value   
        }
    }  
    else if (pg == 2)
    {
        delay(25);
        // The XYZ_max fields are considered the source for the slider to get its initial data from through the     
        //    the display's "Setpoints" page (page2) preinitialization section. After that the slider sends changes real time to the display
        //    not bothering the CPU until movement stops when an event is sent to the CPU where the slider callback will 
        //    get the XYZ_max value and store it in EEPROM.
        // For xfloat type object in Nextion, multiply the float x10 to get an integer it can use with the slider
        // The display will format it with decimal point according to its config (num places to left and right of decimal point)
        hv_adj.setValue((uint16_t) (set_hv_max*10));
        hv_max.setValue((uint16_t) (set_hv_max*10));

        v14_adj.setValue((uint16_t) (set_v14_max*10));
        v14_max.setValue((uint16_t) (set_v14_max*10));

        curr_adj.setValue((uint16_t) (set_curr_max*10));
        curr_max.setValue((uint16_t) (set_curr_max*10));

        temp_adj.setValue((uint16_t) set_temp_max);
        temp_max.setValue((uint16_t) set_temp_max);
       
        meterID_adj.setValue(METERID);
        meterID.setValue((uint16_t) METERID);
        
        // fix up initial position of slider knob for Band Slider
        band_set_adj.setValue(CouplerSetNum);
        strcpy(cmd, Band_Cal_Table[CouplerSetNum].BandName);
        band_set.setText(cmd);     
    }
}

void toPwrGraph_Callback(void *ptr)
{
    pg=3;
    return;
}

uint8_t update_Nextion(uint8_t force_update)
{
    uint8_t temp_value_old = 0, set_temp_max_old = 0, ret;
    float value;
    uint32_t number32;
    static float FwdPwr_old, RefPwr_old, SWRVal_old, hv_value_old, V14_value_old, curr_value_old, Fwd_dBm_old, Ref_dBm_old;
    static uint8_t AuxNum1_old, AuxNum2_old, NewBand_old;
    static float set_curr_max_old, set_hv_max_old, set_v14_max_old;
    static uint8_t pg_last;
      
    if (!force_update)
    {
        // throttle update rate
        if ((Timer_X00ms_InterruptCnt - Timer_X00ms_Last_Nex) < 300)
           return 0;   // skip out until greater than 100ms since our last visit here
        Timer_X00ms_Last_Nex = Timer_X00ms_InterruptCnt;
    }
    else
    {
        // force all status tracking variables to 0 forcing them to update on this pass.
        //FwdPwr_old=RefPwr_old=SWRVal_old=hv_value_old=V14_value_old=temp_value_old=curr_value_old=NewBand_old=0;
    }
    if (pg != pg_last)
    {
        sendCommand("sendme");
        ret = recvPageNumber(&number32);
        if (ret == 1)  // good page num returned
          pg = (uint8_t) number32;    // else just skip this and use the pg value   
        else 
        { 
            sendCommand("sendme");   //retry once first failure
            ret = recvPageNumber(&number32);
            if (ret == 1)  // good page num returned
                pg = (uint8_t) number32;    // else just skip this and use the pg value   
        }
        pg_last = pg;
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
            //xx.setText(FwdPwr, cmd);   // alternative way to update val;ue using the library commands
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
            Set1_Bandvar.setText(cmd);      
        }
        else
        {
            sprintf(cmd, "band.txt=\"%s\"", Band_Cal_Table[CouplerSetNum].BandName);
            sendCommand(cmd); 
            
            strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
            Set1_Bandvar.setText(cmd);      
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
            number32 = (uint32_t) AuxNum1;
            Aux1.setValue(number32);
        }
        if ((AuxNum2 != AuxNum2_old || force_update == 1) && !WAIT)
        {
            AuxNum2_old = AuxNum2;    
            number32 = (uint32_t) AuxNum2;
            Aux2.setValue(number32);
        } 
 #ifdef USBSTUFF       
        // Update display for state of PTT and CW IO lines
        uint8_t state;
        state = Serial_USB_IsLineChanged();              /* Check for Line settings changed */
        if(state != 0u)
        {  
            if(state & Serial_USB_LINE_CONTROL_CHANGED)  /* Show new settings */
            {   
                state = Serial_USB_GetLineControl();
                if (((state & Serial_USB_LINE_CONTROL_RTS) == 2) && ((state & Serial_USB_LINE_CONTROL_DTR) == 0)) // PTT on and CW off
                {
                    sprintf(cmd, "TX");
                    PTT_CW.setText(cmd);  // Get PTT first.  If CW present then next line will overwrite
                    PTT_CW.Set_font_color_pco(65504);  // Set text to Yellow for tx                    
                    PTT_CW.background_color_bco(63488);  // Set bkgnd to RED for tx                    
                }
                else if (((state & Serial_USB_LINE_CONTROL_RTS) == 0) && ((state & Serial_USB_LINE_CONTROL_DTR) == 0)) // PTT on and CW off
                {
                    sprintf(cmd, "RX");
                    PTT_CW.setText(cmd);  // Get PTT first.  If CW present then next line will overwrite
                    PTT_CW.Set_font_color_pco(2016);  // Set text to LT Green for RX
                    PTT_CW.background_color_bco(0);  // Set background color to black for RX
                }                                        
                if (((state & Serial_USB_LINE_CONTROL_RTS) == 2) && ((state & Serial_USB_LINE_CONTROL_DTR) == 1)) // PTT on and CW off
                {
                    sprintf(cmd, "CW");
                    PTT_CW.setText(cmd);  // Get PTT first.  If CW present then next line will overwrite
                    PTT_CW.Set_font_color_pco(2047);  // Set text to RED for tx  
                    PTT_CW.background_color_bco(63488);  // Set background color to black for RX                  
                }
                else if (((state & Serial_USB_LINE_CONTROL_RTS) == 0) && ((state & Serial_USB_LINE_CONTROL_DTR) == 1)) // PTT on and CW off
                {
                    sprintf(cmd, "CW");
                    PTT_CW.setText(cmd));  // Get PTT first.  If CW present then next line will overwrite
                    PTT_CW.Set_font_color_pco(2047);  // Set text to REd for TX.  Then check PTT again for RX state
                    PTT_CW.background_color_bco(0);  // Set background color to black for RX
                }
            }
        }  
#endif
        return 1;
    }
    else if (pg == 3) // only send this group of data while on page 3
    {   
        // Page 3 is the Graphing page
        // Post up power and SWR values.  Display will read the values on a timer and update teh graph.  
        // It will also read the scale and cap the values to the scale max and scale the graph with its own math        
        
        if (FwdPwr > 1999.9)        
        {                                 
            fPwrNum.setValue(1999); 
        }
        else
            fPwrNum.setValue(FwdPwr);
            
        if (RefPwr > 999.9)        
        {   // graphing only accepts integer betwen 0 and 255. using 1000W max                    
            rPwrNum.setValue(999); 
        }
        else
            rPwrNum.setValue(RefPwr);             
                   
        // Just post up the value, the display will handle the math and graphing in a timer
        swrNum.setValue(SWRVal*10); // uses xfloat so mult by 10 to shift decimal point   
        
        Curr_band.setText(Band_Cal_Table[CouplerSetNum].BandName); 
    }    
    else if (pg == 4) // only send this group of data while on page 4
    {   
        // Page 4 is ADC calibration 
        Curr_band.setText(Band_Cal_Table[CouplerSetNum].BandName); 
                
        //if (!recvRetNumber(&number32))
        //    return 0;
        //else
            return 1;
    }
    else if (pg == 5) // only send this group of data while on page 5
    {   
        // Page 5 is Quick Band Select
        //if (!recvRetNumber(&number32))
        //    return 0;
        //else
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
        
    display.clearDisplay();    
    display.drawRect( 0, 17, 127, 47, SSD1306_WHITE);
    
    // Top row is Yellow with Power in Watts, size 2 font
    display.setTextColor(SSD1306_WHITE);
    snprintf(s, 12, "  %*.1fW%c", 6, FwdPwr, '\0');
    display.setTextSize(2);    
    display.setCursor(4,0);
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.println(s);
    
    //sprintf(s,"  Fwd    Ref    SWR");    
    display.setTextSize(1);    
    display.setCursor(16,20);
    display.println("Fwd");
    display.setCursor(58,20);
    display.println("Ref");
    display.setCursor(95,20);
    display.println("SWR");
    
    if (FwdPwr <= 0.1)
        sprintf(s, " %*.1f%*.1f%*s%c", 5, 0.0, 7, 0.0, 5, "NA", '\0'); 
    else
        sprintf(s, " %*.1f%*.1f%*.1f%c", 5, Fwd_dBm, 7, Ref_dBm, 6, SWRVal, '\0');    
    display.setTextSize(1);
    display.setCursor(1,30);
    display.println(s);
    
    value = temp_read();
    value *= temp_cal_factor;
    sprintf(s,"%*.0fF%c", 3, value, '\0');   // temp from the ADL5519 board (not the PA heat sink, maybe later from amp control board)
    display.setTextSize(1);
    display.setCursor(5,49);
    display.println(s);
        
    value = v14_read();
    value *= v14_cal_factor;
    sprintf(s,"%*.1fVDC%c", 4, value, '\0');   // 28V via Voltage divider
    display.setTextSize(1);
    display.setCursor(33,49);
    display.println(s);
    
    value = hv_read();
    value *= hv_cal_factor;
    sprintf(s,"%*.1fVDC%c", 4, value, '\0');   // 28V via Voltage divider
    display.setTextSize(1);
    display.setCursor(81,49);
    display.println(s);
    
    display.display();    // NOTE: You should remember to update the display in order to see the results on the oled. 
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
uint16_t serial_usb_read(void)
{
     rx_count = 0;
     int count = 0;
     int i=0;  
     /* Check for input data from host. */
     count = RFWM_Serial.available();
     rx_buffer[0] = _NULL;
     if (count > 0)
     {
          while (i < count)
          {
              rx_buffer[i++] = RFWM_Serial.read();
              rx_buffer[i] = '\0';
          }
          rx_count = i;          
          RFWM_Serial.println(rx_count);
          RFWM_Serial.println((char *) rx_buffer);
          // initially p1 = p2.  parser will move p1 up to p2 and when they are equal, buffer is empty, parser will reset p1 and p2 back to start of sData         
          memcpy(pSdata2, rx_buffer, rx_count+1);   // append the new buffer data to current end marked by pointer 2        
          pSdata2 += rx_count;   // Update the end pointer position. The function processing chars will update the p1 and p2 pointer             
          rx_count = pSdata2 - pSdata1;  // update count for total unread chars. 
          //RFWM_Serial.println(rx_count);  
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
        RFWM_Serial.write(tx_buffer, tx_count);         /* Send data back to host. */
    }
}

float hv_read(void)
{
    uint32_t ad_counts=0;
    
    // Read voltage by ADC via MUX (If connected)
    delay(5);
    ad_counts = analogRead(ADC_HV);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float v14_read(void)
{
    uint32_t ad_counts=0;
    
    // Read voltage by ADC via MUX (If connected)
    delay(5);
    ad_counts = analogRead(ADC_14V);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float curr_read(void)
{
    uint32_t ad_counts=0;
    
    // Read ADC via MUX (If connected)
    delay(5);
    ad_counts = analogRead(ADC_CURR);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float temp_read(void)
{
#ifdef DETECTOR_TEMP_CONNECTED
  float tmp;
    uint32_t ad_counts=0;
    // Read detector temperature (If connected)
    delay(5);
    ad_counts = analogRead(ADC_TEMP);  // single read but has locked up with N1MM commands   
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
 *   Send out dBm, Watts, and SWR values to data channel
*/
void sendSerialData()
{
    Timer_X00ms_InterruptCnt = millis();
    if (((Timer_X00ms_InterruptCnt - Timer_X00ms_Last) > 1000)) // && (EEPROM.read(SER_DATA_OUT_OFFSET) != 0))
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
    uint8_t cmd1;
    float cmd2;
    uint8_t cmd_str_len, i = 0, j = 0;
    char cmd_str[BUF_LEN] = {};
    
    if (rx_count >=0) 
    { 
        pSdata = (uint8_t *) strchr((char *) pSdata1, '\n');   // find string terminator position 
        if (pSdata) { 
            *pSdata = '\0';
            cmd_str_len = pSdata - pSdata1;                     
            strncpy(cmd_str, (char *) pSdata1, cmd_str_len);   // copy chars between p1 and the terminator
            pSdata1 += (cmd_str_len+1);  // reset ch pointer back to strat of string for char parsing
            if (pSdata1 >= pSdata2 || cmd_str_len > BUF_LEN)  // if we have caught up to the end p2 then reset to beginning of buffer position.
                pSdata1 = pSdata2 = sdata;           
            if (strlen((char *) sdata) >= BUF_LEN-1) {
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
            //RFWM_Serial.print(" Meter ID  ");
            //RFWM_Serial.println("%s\n",cmd_str);
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
                            //RFWM_Serial.println("Call Serial Data Output Toggle");
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
                        if (cmd1 == 96) {     // Not used on Arduino
                           //digitalWrite(Nextion_Switch, 1);    // Switch serial from display to USB UART converter                                                                      
                        }
                        if (cmd1 == 95) {    // Not used on Arduino
                           //digitalWrite(Nextion_Switch, 0);    // Switch serial from display to CPU                                                        
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
                        if (cmd1 == 69) // Get Custom Band Decoder Input port pattern from user for current band 
                        {                              
                            Band_Cal_Table[CouplerSetNum].band_input_pattern = cmd2;
                        }  
                        if (cmd1 == 68) // Get Custom Band Decoder Output port pattern from user for current band 
                        {                            
                            Band_Cal_Table[CouplerSetNum].band_A_output_pattern = cmd2;
                        }
                        if (cmd1 == 67) // Get Custom Band Decoder Output port pattern from user for current band 
                        {                              
                            Band_Cal_Table[CouplerSetNum].band_B_output_pattern = cmd2;
                        }
                        if (cmd1 == 66) // Get Custom Band Decoder Output port pattern from user for current band 
                        {                              
                            Band_Cal_Table[CouplerSetNum].band_C_output_pattern = cmd2;
                        }
                        if (cmd1 == 65)  // Set or Clear Band Decoder Output translate options
                        {                          
                            EEPROM.update(TRANS_INPUT,cmd2);  // 
                        }
                        if (cmd1 == 64)  // Set or Clear Band Decoder Input translate mode
                        {                          
                            EEPROM.update(TRANS_A,cmd2);  // 
                        }
                        if (cmd1 == 63)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(TRANS_B,cmd2);  // 
                        }
                        if (cmd1 == 62)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(TRANS_C,cmd2);  // 
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
    uint8_t   i;

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
void print_Cal_Table_progress(uint8_t flag)
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
void print_Cmd_Progress(uint8_t flag)
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
void read_Arduino_EEPROM()
{
    int   i;
    int   j;  
    uint8_t  eepromArray[sizeof Band_Cal_Table];
    //int   Arr_Size;
    char  setpoint_buf[SETPOINT_LEN+1];
    int   len_ee;
   
    RFWM_Serial.println("read_Cal_Table_from_EEPROM - Start");

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
        // Start row 3
        //Translate options using first 4 bytes of row 3 - read and write directly to EEPROM, no variable used.
        // They are initiallzed in write_Cal_Table_from_Default() fucntion.
    
  // Now get the band table struct data   
   for (i=0; i<NUM_SETS; i++) 
   {
      if (i < EEPROM.length())
      {
         len_ee = EEADDR+(sizeof(Band_Cal)*i);
         EEPROM.get(len_ee, Band_Cal_Table[i]);  
         //RFWM_Serial.print(EEPROM.length());       
         //RFWM_Serial.println("EEPROM Read");
         RFWM_Serial.println(Band_Cal_Table[i].BandName);
         RFWM_Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         RFWM_Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(10); 
      }
   }
   if (len_ee > EEPROM_SIZE)
   {       
     RFWM_Serial.print("ERROR! - Exceed EEPROM Storage limit - Bytes used = ");
     RFWM_Serial.println(len_ee);
   }   
   RFWM_Serial.print("EEPROM Max Size is : ");
   RFWM_Serial.print(EEPROM.length());    //EEPROM_SIZE);
   RFWM_Serial.print("  - EEPROM Bytes used : ");
   RFWM_Serial.print(len_ee);
   RFWM_Serial.print("  - EEPROM Bytes remaining = ");
   RFWM_Serial.println(EEPROM_SIZE - len_ee);
   RFWM_Serial.print("Number of table rows = ");
   RFWM_Serial.print(NUM_SETS);
   RFWM_Serial.print("  - Size of each Cal_Table row = ");
   RFWM_Serial.println(sizeof(Band_Cal));
   RFWM_Serial.println("read_Cal_Table_from_EEPROM - End");
}

// Copy cal data in memory to EEPROM to preserve user changes (save state is another function)
void write_Arduino_EEPROM()
{
    uint16_t    i;
    uint16_t    len_ee;
    uint8_t   ee_row;
    uint8_t   c1_array[16]; /* stores 1 row of 16 bytes for eeprom write and reads */
    uint16_t    j;    
    //uint8_t   eepromArray[NUM_SETS * (sizeof Band_Cal_Table)];  // 396 bytes
    //uint16_t    Arr_Size;
    //uint8_t   result;
    char   setpoint_buf[SETPOINT_LEN+1];

    RFWM_Serial.println(" Write Cal Table to EEPROM - Start " );
    c1_array[0] = 'S';   /* what we just wrote, will overwrite as a block of 16 bytes) */
    c1_array[EE_RESERVED_BYTE1] = 0;
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
    uint8_t tempvar4[20];
    for(j=0; j<16; j++)
    {              
         EEPROM.get(j+(ee_row*16),tempvar4[j]);     
         RFWM_Serial.print(" Row 0 BYTES  = ");
         RFWM_Serial.println(tempvar4[j]);     
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
   //uint8_t tempvar4[20];
    for(j=0; j<16; j++)
    {              
         EEPROM.get(j+(ee_row*16),tempvar4[j]);     
         RFWM_Serial.print(" Row 1 BYTES  = ");
         RFWM_Serial.println(tempvar4[j]);     
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
    sprintf(setpoint_buf, "%01.2f", (float)temp_cal_factor);                           // convert float to 4 byte ascii
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
         RFWM_Serial.print(" Row 2 BYTES  = ");
         RFWM_Serial.println(tempvar4[j]);     
    }
*/  
    // First 4 bytes of Row 3 are reserved for Translate options and initialized in write_Cal_Table_from_Default() function
    // They are read and written direct to EEPROM so nothing to do here for these values

   // Now write the Cal Table array 
   for (i=0; i< NUM_SETS; i++) {
      if (i < EEPROM.length()) {
         len_ee = EEADDR+(sizeof(Band_Cal)*i);
         RFWM_Serial.print("len_ee = ");
         RFWM_Serial.println(len_ee);
         EEPROM.put(len_ee, Band_Cal_Table[i]);       
         RFWM_Serial.println("EEPROM Written");
         RFWM_Serial.println(Band_Cal_Table[i].BandName);
         RFWM_Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         RFWM_Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(10);
         EEPROM.update(0,'G');
      }
      if (len_ee > EEPROM_SIZE)
      {       
         RFWM_Serial.print("ERROR! - Exceed EEPROM Storage limit - Bytes used = ");
         RFWM_Serial.println(len_ee);
      }
   }    
   RFWM_Serial.print("EEPROM Max Size is : ");
   RFWM_Serial.print(EEPROM.length());    //EEPROM_SIZE);
   RFWM_Serial.print("  -  EEPROM Bytes used : ");
   RFWM_Serial.print(len_ee);
   RFWM_Serial.print("  -  EEPROM Bytes remaining = ");
   RFWM_Serial.println(EEPROM_SIZE - len_ee);
   RFWM_Serial.println("Write Cal Table to EEPROM - End " );
}

// Copy hard coded al data to memory
void write_Cal_Table_from_Default()
{
    uint16_t i;
  
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
        Band_Cal_Table[i].band_input_pattern = Band_Cal_Table_Def[i].band_input_pattern;
        Band_Cal_Table[i].band_A_output_pattern = Band_Cal_Table_Def[i].band_A_output_pattern;
        Band_Cal_Table[i].band_B_output_pattern = Band_Cal_Table_Def[i].band_B_output_pattern;
        Band_Cal_Table[i].band_C_output_pattern = Band_Cal_Table_Def[i].band_C_output_pattern;
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
    // Initialize the mode of tranlation for each port
    EEPROM.update(TRANS_INPUT,0); // used to change bands from radio
    EEPROM.update(TRANS_A, 0);    // Used for pass thorugh to 5 band transverter
    EEPROM.update(TRANS_B, 2);    // Used to select antennas or amps
    EEPROM.update(TRANS_C, 2);    // Used to select antennas or amps
    /*  for debug
    RFWM_Serial.println(" Write Cal From Default");
    RFWM_Serial.println(op_mode);
    RFWM_Serial.println(CouplerSetNum); 
    RFWM_Serial.println(ser_data_out);
    RFWM_Serial.println(set_hv_max);            
    RFWM_Serial.println(set_v14_max);
    RFWM_Serial.println(set_temp_max);
    RFWM_Serial.println(METERID);  
    RFWM_Serial.println(Callsign);
    RFWM_Serial.println(set_curr_max);
    RFWM_Serial.println(hv_cal_factor);          
    RFWM_Serial.println(v14_cal_factor);
    RFWM_Serial.println(curr_cal_factor);        
    RFWM_Serial.println(temp_cal_factor);
    RFWM_Serial.println(curr_zero_offset);
    */
    RFWM_Serial.println("write_Cal_Table_from_Default - Completed");
}

// Read state in EEPROM
void get_config_EEPROM()
{
    CouplerSetNum = EEPROM.read(COUPLERSETNUM_OFFSET);
    //RFWM_Serial.print("CouplerSetNum Read = ");
    //RFWM_Serial.println(CouplerSetNum);
    op_mode = EEPROM.read(OP_MODE_OFFSET);
    //RFWM_Serial.print("op_mode Read = ");
    //RFWM_Serial.println(op_mode);
    //EEPROM.commit();
}

// Save state in EEPROM
uint8_t EE_Save_State()
{
  return 1;
    //char buf[3];
    RFWM_Serial.println("EE_Save_State - Start");
    CouplerSetNum = constrain(CouplerSetNum,0,NUM_SETS);
    EEPROM.update(COUPLERSETNUM_OFFSET, CouplerSetNum);
    RFWM_Serial.print("CouplerSetNum Write =");
    RFWM_Serial.println(CouplerSetNum);
    op_mode = constrain(op_mode,1,3);
    if (op_mode == 1 || op_mode == 2) {
        EEPROM.update(OP_MODE_OFFSET, op_mode);
        RFWM_Serial.print("op_mode Write = ");
        RFWM_Serial.println(op_mode);
    }
    EEPROM.update(SER_DATA_OUT_OFFSET, ser_data_out);
    RFWM_Serial.print("ser_data_out Write =");
    RFWM_Serial.println(ser_data_out);
    Cal_Table();
    RFWM_Serial.println("EE_Save_State - End");
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
void toggle_ser_data_output(uint8_t force_on)
{
      if (ser_data_out == 0 || force_on == 1)
      {
        EEPROM.update(SER_DATA_OUT_OFFSET, 1);
        RFWM_Serial.println("Enabled Serial Data Output");
        ser_data_out = 1;
      }
      else { 
        EEPROM.update(SER_DATA_OUT_OFFSET, 0);
        RFWM_Serial.println("Disabled Serial Data Output");
        ser_data_out = 0;
      } 
}

uint8_t EEPROM_Init_Write(void)
{
    write_Arduino_EEPROM();
    return 1;
}

uint8_t EEPROM_Init_Read(void)
{
    read_Arduino_EEPROM();
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

/*
Convert AUX command from N1MM to 4 bit BCD
Command format is AUXxnn fixed width. x is the radio number, usually 1.   Example is AUX103 for command 03 for BCD output of 0x03.
*/
uint8_t OTRSP(void)
{     
    char c;
    static int i;
    //static char AuxCmd0[20];   // Made a global
    static char buf[20];

    //AuxNum1 = AuxNum2 = 0;  // Global values also used to update status on LCD
    if (OTRSP_Serial.available() > 0)
    {
        c = OTRSP_Serial.read();  // Accept AUXyZZ where y is 1 or 2 and ZZ is 00-FF.
        buf[i] = c;
        buf[i+1] = '\0';
        //RFWM_Serial.print("buf char added = ");
        RFWM_Serial.print(c);   //echo input chars
        if (c=='\r')
          RFWM_Serial.print('\n');
        if (i >= 19)
              i = 0;  // prevent buffer overrun
        if (c == '\r' && i > 5)  // look for end of string then look back 6 chars for string parse
        {
            //OTRSP_Serial.println(i);
            strncpy(AuxCmd0, &buf[i-6], 6);
            AuxCmd0[6] = '\0';
            i=0; 
            // AUXxYY\r or BANDxZ...\r  YY is 00-99 (Should be 00-FF) and Z is 0-9, should be 0-F.
            if (strncmp(AuxCmd0,"AUX",3) == 0 || strncmp(AuxCmd0,"BAND",4) == 0)               
            {
                RFWM_Serial.print("AuxCmd0 = ");
                RFWM_Serial.println(AuxCmd0);
                return 1;   // Signal we have a good string
            }         
        }
        i++;  
    }
    return 0;
}

uint8_t OTRSP_Process(void)
{
    //static char AuxCmd0[20];
    char AuxCmd1[AUXCMDLEN], AuxCmd2[AUXCMDLEN];
    char BndCmd1[BANDCMDLEN], BndCmd2[BANDCMDLEN];
    uint8_t i;
    
    // Now have a full 6 char string starting with A or B
    RFWM_Serial.print(" OTRSP Process string : ");
    RFWM_Serial.println(AuxCmd0); 
    if ((AuxCmd0[0] == 'A' || AuxCmd0[0] == 'B') && AuxCmd0[6] == '\0' && strlen(AuxCmd0)==6) // double validate
    {      
        // Looking only for 2 commands, BAND and AUX.   
        if (strncmp(AuxCmd0,"AUX1",4) == 0)   // process AUX1 where 1 is the radio number.  Each radio has a 4 bit BCD value
        {
            AuxCmd1[0] = AuxCmd0[4];
            AuxCmd1[1] = AuxCmd0[5];
            AuxCmd1[2] = AuxCmd0[6];
            AuxCmd1[3] = '\0'; 
            //RFWM_Serial.print(" OTRSP Process AUX1 string in BCD: ");
            //RFWM_Serial.println(AuxCmd1); 
            //AuxNum1 = BCDToDecimal(AuxCmd1);   // Convert text 0x00-0xFF HEX to Decimal
            AuxNum1 = atoi(AuxCmd1);   // Convert text 0-255 ASCII to Decimal
            RFWM_Serial.print(" OTRSP Process AUX1 string in Decimal: ");
            RFWM_Serial.println(AuxNum1);
            
            //Aux1_Write(AuxNum1);  // write out to the Control register which in turn writes to the GPIO ports assigned.                      
            for (i=0; i < 20; i++)
                AuxCmd0[i] = '\0';
            return(1);  // 1 signals a change
        }
        else if (strncmp(AuxCmd0,"AUX2",4) == 0)   // process AUX comand for radio 2.
        {
            AuxCmd2[0] = AuxCmd0[4];
            AuxCmd2[1] = AuxCmd0[5];
            AuxCmd2[2] = AuxCmd0[6];
            AuxCmd2[3] = '\0';
            //AuxNum2 = BCDToDecimal(AuxCmd2);   // Convert text 0x00-0xFF HEX to Decimal
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
            RFWM_Serial.print(" OTRSP Processing BAND 1 string : ");
            RFWM_Serial.println(BndCmd1); 
            return(0);  // TODO = passing band MHZ to a CouplerNUM  Search Band values
        }
        else if (strncmp(AuxCmd0,"BAND2",5) == 0)   // process AUX comand for radio 2.
        {
            sprintf(BndCmd2,"%s", &AuxCmd0[5]);
            AuxCmd0[0] = '\0' ;
            RFWM_Serial.print(" OTRSP Processing BAND 2 string : ");
            RFWM_Serial.println(BndCmd2); 
            return(0); 
        }
    }  
    return 0;   // nothing processed 0 is a valid band number so using 255.
}

uint8_t BCDToDecimal(char *hex)
{
    //char hex[17];
    uint8_t decimal;   //place;
    uint16_t i = 0, val = 0, len = 0;

    decimal = 0;
    //place = 1;
    /* Find the length of total number of hex digit */
    len = (uint16_t) strlen(hex);
    len--;
    /*
     * Iterate over each hex digit
     */
    for(i=0; hex[i]!='\0'; i++)
    {
        /* Find the decimal representation of hex[i] */
        if(hex[i]>='0' && hex[i]<='9')
        {
            val = hex[i] - 48;
        }
        else if(hex[i]>='a' && hex[i]<='f')
        {
            val = hex[i] - 97 + 10;
        }
        else if(hex[i]>='A' && hex[i]<='F')
        {
            val = hex[i] - 65 + 10;
        }

        decimal += val * pow(16, len);
        len--;
    }
   //RFWM.print("DEC = ");
   //RFWM.println(decimal);
   return decimal;
}

uint8_t Band_Decoder(void)   // return 1 for new band detected, 0 for none.  
{     
    // Convert BCD to binary and update the (non-BCD) binary Port for amps and meter selection
    static uint8_t decoder_input_last;
    uint8_t tempval = 0;
    uint8_t i;

    Band_Decoder_Get_Input();   // Read the state of all input pins and store into variable Band_Dec_In_Byte

    if (Band_Dec_In_Byte != decoder_input_last)
    {
        RFWM_Serial.print(" Band_Dec_In_Byte = 0x");
        RFWM_Serial.println(Band_Dec_In_Byte, HEX);
        decoder_input_last = Band_Dec_In_Byte;
        
        // Read the translation scheme flag in the band record (.translate) and convert to various formats
        if (EEPROM.read(TRANS_INPUT) == 1)   
        {    
            // This conversion scheme will look for the first high input and give a count for a one pin per band scheme
            for (i = 0; i < 8; i++)
            {
                if (bitRead(Band_Dec_In_Byte, i))
                    tempval = i+1;     // count the highest bit set to a 1 for of 8 priority decoding.  Could also do this with custom pattern
            }
            Band_Dec_In_Byte = tempval;
            RFWM_Serial.print(" 1 of 8 Decode Match Found = 0x");
            RFWM_Serial.println(Band_Dec_In_Byte, HEX);
            NewBand = Band_Dec_In_Byte;
            return 1;  
        }
        else if (EEPROM.read(TRANS_INPUT) == 2)
        {
            // Use a custom pattern to look for anything ting you want to represent a band choice.
            // Advantage of this is you can use the lower 3 bits for band and the upper for antenna if needed.
            for (i = 0; i < NUM_SETS; i++)
            {
                if (Band_Cal_Table[i].band_input_pattern == Band_Dec_In_Byte)  // search the table rows for a matching value
                {
                    NewBand = i;
                    Band_Dec_In_Byte = i;
                    RFWM_Serial.print(" Custom Pattern Match Found : ");
                    RFWM_Serial.println(Band_Cal_Table[i].BandName);
                    return 1;  
                }
            }
        }
        else
        {
            // if translate value not 1 or 2, then pass through the input pattern
            NewBand = Band_Dec_In_Byte;
            RFWM_Serial.print(" Band Input Received, New Band = 0x");
            RFWM_Serial.println(Band_Dec_In_Byte, HEX);
            return 1;
        }       
        RFWM_Serial.print(" ERROR: No Matching Band Found, Input Received = 0x");
        RFWM_Serial.println(Band_Dec_In_Byte, HEX);
    }
    return 0;   // no change detected
}

/*   Noe about OTRSP and the band decoding strategy used here
        At this point we need to consider how to interpret and act on multiple band input sources.
        1. BCD/Parallel pin inputs.  6 digital pins. (follows radio or manual switch whcih woudl nto follow radio)
        2. OTRSP serial commands from a logger (Follows N1MM which is assumed to be following or directing the radio band changes)
        3. WSJT-X status packet commands relayed though the Desktop app which sends a band change command (follows radio)
        4. Desktop App band change buttons (does nto change radio)
        5. Future Serial port commands from a direct connected radio or computer (running omni-rig or hamlib for example)
        
        The OTRSP AUXxYY commands are flexible and do not have to be set up as a band change command though I am currently using it that
        way in case there is not WSJTX source to follow the radio.  Ideally we woudl get a BAND command, or follow N1MM UDP info,
        or read the band decoder input pins.  
        Rereading the OTRSP spec the YY payload data is 0-255 specified in 1 to 3 ASCII digits. 
        Today I have it as a fixed length 2 char hex value.  N1MM currently only sends a fixed length 2 digit value in ASCII
        decimal form with leading 0.  The latest version N1MM (as of 12/2020) expanded the Antenna table to 64 rows but only 0-15 are sent out Serial.
         
        AUXx commands could just change the antennas while the band is not changed.
        N1MM should send the BANDxY only be used to send out the bands for Radio 1 and Radio 2.  That would free up the Aux to be truly any value
        The best case to get that is to not use Aux for band info and instead use the Input pins for band input and free the aAUx to output a pattern without changing
        our wattmeter band.  The workaround is to set a table that converts the Aux to a band.  Might just as well set the band output to what you want and keep N1MM linear (0-15 for 15 bands).
        
        // The output Ports A and B will map to Radio 1 and Radio 2 from N1MM+.  The XX value is BCD and will be used as a band indicator
        // Today the band indicator causes the wattmter to change bands in turn causing the programmed output pattern to be sent out for that band.
        // Ideally there is a working BANDxY command which this progream supports, but I do nto think N1MM can issue this commands
        // Not sure if it is a raal OTRSP command, have seen it in some code examples.

        Except from https://k1xm.org/OTRSP/OTRSP_Protocol.pdf
        AUX x n
              Sets the auxiliary port. x is the port number. The values of x are 0-9. The value of n is a
                one to three digit decimal number. The value of n is 0-255.
              If the auxiliary port does not exist the command is ignored. If the value is larger than the
                port supports the high order bits will be ignored.
              There is a convention that auxiliary port 1 contains band or antenna information for radio 1
                and auxiliary port 2 contains band or antenna information for radio 2.
              The ?AUXx command returns the value actually set for a port and will not include bits
                which were ignored. For example, if port 1 is 4 bits, the command AUX125 will set the
                value to 9 and ?AUX1 will return AUX19.
              If a port does not exist the response to a query for that port would be that the command does
                not exist. For example, ?AUX0 would return ?AUX0 if there is no port 0. 
              Auxiliary port 1 is equivalent to LPT port pins 2, 7, 8, 9.
        BAND x n
              Tells the SO2R device the band information for a radio. x is the radio number. The values of x are 1 or 2.
              The value of n is the frequency in MHz of the bottom of the band, for example 3.5 or 144.
              If the band is not set ?BAND would return BAND0. 
              Ths means the payload 'n' can be variable length up to some max with CR as the delimiter.  
    
    // Set the Aux ports to match the OTRSP commands last received.
    // This is a bit pattern so a 05 will result in bits 0 and 3 getting set to HIGH.
    // If you only want 1 bit at a time per command then set the number input appropriately
    // Example, if you want bit 2 then bit 4 send the number 2 then the number 8.

    // We need a option to translate to BCD or bitwise, and to set an output pattern (any pattern) per band
    // AuxNum, will then continue to change meter bands.  
    // This won't work for Alternate antennas in N1MM though, it would switch bands such as to 5.7G for a value of 9.
*/    

// Read the group of pins that together are a pattern stored in 1 byte for htge Band in group
void Band_Decoder_Get_Input()
{    
    bitWrite(Band_Dec_In_Byte, 0, digitalRead(BAND_DEC_IN_0));  // read in and store the state of all band decoder input pins
    bitWrite(Band_Dec_In_Byte, 1, digitalRead(BAND_DEC_IN_1));
    bitWrite(Band_Dec_In_Byte, 2, digitalRead(BAND_DEC_IN_2));
    bitWrite(Band_Dec_In_Byte, 3, digitalRead(BAND_DEC_IN_3));
    bitWrite(Band_Dec_In_Byte, 4, digitalRead(BAND_DEC_IN_4));
    bitWrite(Band_Dec_In_Byte, 5, digitalRead(BAND_DEC_IN_5));
    bitClear(Band_Dec_In_Byte, 6);   // clear the unused top 2 bits
    bitClear(Band_Dec_In_Byte, 7);
    //RFWM_Serial.print(" Band_Dec_In_Byte = 0x");
    //RFWM_Serial.println(Band_Dec_In_Byte, HEX);
}

// Group of pins that together are a pattern stored in 1 byte for Band Out A group
void Band_Decode_A_Output(uint8_t pattern)
{
    if (EEPROM.read(TRANS_A) == 0)  // bit 0 and 1 is for Group A translation (or not).
        {}  // do nothing, pass straight through
    else if (EEPROM.read(TRANS_A) == 1)  
        pattern = bit(pattern)>>1;    // Translate to 1 pin at a time only such as for amp or transverter selection
    else if (EEPROM.read(TRANS_A) == 2)  // 2 is not used here
        pattern = Band_Cal_Table[CouplerSetNum].band_A_output_pattern;  // Use stored pattern for this band 
    else 
    {
        RFWM_Serial.print("No Valid Matching Rule for Band A Output translation = 0b");   
        RFWM_Serial.println(EEPROM.read(TRANS_A), BIN);
        return;
    }
    RFWM_Serial.print("Band A pattern = 0x");   
    RFWM_Serial.println(pattern, BIN);
    
    digitalWrite(BAND_DEC_A_0, bitRead(pattern, 0));
    digitalWrite(BAND_DEC_A_1, bitRead(pattern, 1));
    digitalWrite(BAND_DEC_A_2, bitRead(pattern, 2));
    digitalWrite(BAND_DEC_A_3, bitRead(pattern, 3));
    digitalWrite(BAND_DEC_A_4, bitRead(pattern, 4));
    digitalWrite(BAND_DEC_A_5, bitRead(pattern, 5));
    digitalWrite(BAND_DEC_A_6, bitRead(pattern, 6));
    digitalWrite(BAND_DEC_A_7, bitRead(pattern, 7));
}

  // we have a byte that needs to translate from a number to a bit
  // AuxNum1 says what band we should be on, as does CouplerSetNum, but not the output pin pattern which could be a complex set of relay connections
  // We want to configure a pattern for each band then when called, set that pattern for that band.  
  // No translation required because the user has specified the pattern for each based on the needs.  

  // N1MM will only send out a value to 15 on Serial port today.  That means is is a BCD number in decimal format.
  // We will take OTRSP AUx number or Band Decode nnumber and convert to only 1 bit.
  // So 1 = bit 0=1 or 1dec, 2=bit 1=1 or 2dec, 3=bit 2=1 or 4dec, 4=bit 3=1 or 8dec, 5=bit 4=1 or 16dec, 6=bit 5=1 or 32dec, 7=bit 6=1 or 64,  8=bit7=1 or 128dec
  // All numbers above 7 will be ignored.
  // Need option for user to choose to translate or pass untouched (Leave BCD). 
// Write out the Group B pins that together are a pattern stored in 1 byte for Band Out B group

// Normally this will be used only for OTRSP AUX1 control
void Band_Decode_B_Output(uint8_t pattern)
{
    if (EEPROM.read(TRANS_B) == 0)  // bit 0 and 1 is for Group A translation (or not).
        {}  // do nothing, pass straight through
    else if (EEPROM.read(TRANS_B) == 1)  
        pattern = bit(pattern)>>1;    // Translate to 1 pin at a time only such as for amp or transverter selection
    else if (EEPROM.read(TRANS_B) == 2)
        pattern = Band_Cal_Table[CouplerSetNum].band_B_output_pattern;  // Use stored pattern for this band
    else if (EEPROM.read(TRANS_B) == 3)        
        pattern = Band_Cal_Table[AuxNum1].band_B_output_pattern;  // Use OTRSP Aux2 as index to Cal table for this band 
    else if (EEPROM.read(TRANS_B) == 4)        
        pattern = AuxNum1;  // Use OTRSP Aux1 number for this band 
    else return;
    
    RFWM_Serial.print("Band B pattern from Aux1 = 0b");
    RFWM_Serial.println(pattern, BIN);
    
    digitalWrite(BAND_DEC_B_0, bitRead(pattern, 0));
    digitalWrite(BAND_DEC_B_1, bitRead(pattern, 1));
    digitalWrite(BAND_DEC_B_2, bitRead(pattern, 2));
    digitalWrite(BAND_DEC_B_3, bitRead(pattern, 3));
    digitalWrite(BAND_DEC_B_4, bitRead(pattern, 4));
    digitalWrite(BAND_DEC_B_5, bitRead(pattern, 5));
    digitalWrite(BAND_DEC_B_6, bitRead(pattern, 6));
    digitalWrite(BAND_DEC_B_7, bitRead(pattern, 7)); 
}

// Write out the Group C pins that together are a pattern stored in 1 byte for Band Out C group
// Normally this will be used for OTRSP AUX2 control
void Band_Decode_C_Output(uint8_t pattern)
{      
    if (EEPROM.read(TRANS_C) == 0)  // bit 0 and 1 is for Group A translation (or not).
        {}  // do nothing, pass straight through
    else if (EEPROM.read(TRANS_C) == 1)  
        pattern = bit(pattern)>>1;    // Translate to 1 pin at a time only such as for amp or transverter selection
    else if (EEPROM.read(TRANS_C) == 2)
        pattern = Band_Cal_Table[CouplerSetNum].band_C_output_pattern;  // Use Cal_Table stored pattern for this band        
    else if (EEPROM.read(TRANS_C) == 3) 
        pattern = Band_Cal_Table[AuxNum2].band_C_output_pattern;  // Use OTRSP Aux2 as index to Cal_Table for this band 
    else if (EEPROM.read(TRANS_C) == 4) 
        pattern = AuxNum2;  // Use OTRSP Aux2 number for this band 
    else
      return;
    
    RFWM_Serial.print("Band C pattern from Aux2 = 0b");
    RFWM_Serial.println(pattern, BIN);
    
    digitalWrite(BAND_DEC_C_0, bitRead(pattern, 0));
    digitalWrite(BAND_DEC_C_1, bitRead(pattern, 1));
    digitalWrite(BAND_DEC_C_2, bitRead(pattern, 2));
    digitalWrite(BAND_DEC_C_3, bitRead(pattern, 3));
    digitalWrite(BAND_DEC_C_4, bitRead(pattern, 4));
    digitalWrite(BAND_DEC_C_5, bitRead(pattern, 5));
    digitalWrite(BAND_DEC_C_6, bitRead(pattern, 6));
    digitalWrite(BAND_DEC_C_7, bitRead(pattern, 7)); 
}

/* [] END OF FILE */
