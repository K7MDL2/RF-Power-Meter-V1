#include <M5Stack.h>
#include <EEPROM.h>
#include <ESP32AnalogRead.h>
/*
 * 
 *
 RF Power Meter by K7MDL 5/15/2020   
 *
 * 5/15/2020 - Updated cal table after more cal testing.  Still noisy ADC below 900MHz
 *             Merged changes from the Nano Headless versaion for complee remote command set and
 *             new serial message protocol.  This is not backward compatible with V1.01 or earlier.
 *  
 * 5/7/2020 - Added correction for AD non-linearity.  Using factory A/D calibration data now.
 *            Aded lots of serial port vaidiation and error recovery
 *            Network thread always runs.  Serial thread stops and starts with errors and/or ON/Off button.
 *            Band Data and Radio ID now do a failover for various conditions of missng serial and/or network data
 *
 Displays Forward Power in Watts on analog scale, displays Forward and Reflected power in Watts and dBm
 in digital form, and calculates SWR and displays in digital and analog meter form
 * 
 Has user edited Calibration sets.  Make one set for each coupler (with extra atenuators), and make 
 several sets for each coupler with couling factor and any attenuation fudge factor to bring it into cal 
 for a given frequency band.   --- For now only 1 set with 10 "bands" for frequency correction is created
 and used with values that can be edited via the UI ---

 In this code example I use a RLC .05-1000MHz coupler and created 10 cal bands toi cover 50M to 10G.
 A value for each dual directional coupler port combines the coupler facxtor, added attenuators, and any cal correction fudge factor.
 The values are edited via the device UI and stored in EEPROM.
 
 Based on an M5Stack code example for an analogue meter using a ILI9341 TFT LCD screen

 Needs Font 2 (also Font 4 if using large scale label

 Make sure all the display driver and pin comnenctions are correct by
 editing the User_Setup.h file in the TFT_eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################
 
Example meter scale code updated by Bodmer for variable meter size
 */

// Define meter size as 1 for M5.Lcd.rotation(0) or 1.3333 for M5.Lcd.rotation(1)
#define M_SIZE 1.3333

#define METERID 101 // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station
#define METER_RATE 2   // used to skip serial data output to a lower rate
#define TEST_EEPROM 0  // change to 1 and reprogram, then set back to 0 to reset EEPROM data to default
#define TFT_GREY 0x5AEB
#define SWR 2
#define PWR 1
#define MENU 3
#define EEADDR 16 // Start location to write data table structure in EEPROM.  Byte level data values will start at 2.  EEPROM status is byte 0
#define NO 0
#define YES 1
#define ADC_COUNTS 1024    // 4096 for ESP32 12bit, 1024 for 10 bit ESP32 and Nano.
//#define ad_Fwd "A1"    // Analog 35 pin for channel 0
//#define ad_Ref "A2"   // Analog 36 pin for channel 1
// Edit the Coupler Set data inb teh Cal_Table function.  Set the max number of sets here, and the default to load at startup
#define NUM_SETS 5 // 10 bands, 0 through 9 for example
float Vref = 5.0;        // 3.3VDC for Nano and ESP32 (M5stack uses ESP32)  ESP32 also has calibrated Vref curve
int CouplerSetNum = 0;   // 0 is the default set on power up.  
int ser_data_out = 0;
int Reset_Flag = 0;
int scale_PWR_Fwd = 5;
int scale_PWR_Ref = 5;
float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = M_SIZE*120, osy = M_SIZE*120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update
const int ad_Fwd = 35; // Analog 35 pin for channel 0
const int ad_Ref = 36; // Analog 36 pin for channel 1
int old_analog =  -999; // Value last displayed
float Fwd_dBm = 0;
float Ref_dBm = 0;
float FwdPwr = 0;
float RefPwr = 0;
float SWRVal = 0;
float SWR_Serial_Val = 0;
float FwdVal = 0;
float RefVal = 0;
float scale_value_fwd = 1;
float scale_value_ref = 1;
float needle_value = 0;
int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;
int Edit_Atten = 0;
int op_mode = SWR;
int counter1 = 0;
int Button_A = 0;
int Button_B = 0;
int Button_C = 0;
int NewBand = 0;
int Ser_Data_Rate = METER_RATE;
float CouplingFactor_Fwd = 0;
float CouplingFactor_Ref = 0;
float Offset = 0.500;  // AD8318 is 0.5 offset.  0.5 to about 2.1 volts for range.
float Slope = 0.025;  // AD8318 is 25mV per dB with temp compensation
char Coupler_friendly_name[80] {"Default\0"};
float FwdPwr_last = 0;
int Inverted = 1;  // 0=no, 1=Yes.  Inverted output will have max V = no input, min volt at max power input.
/* 
 *  AD8318 is an inverted with about 2.5V for no inoput and 0.5 for max input cover -65 to +5dBm range
 *  linear between -55 and 0dBm
*/ 
const int numReadings = 6;   // adjust this for longer or shorter smooting period as AD noise requires
float readings_Fwd[numReadings];      // the readings from the analog input
float readings_Ref[numReadings];      // the readings from the analog input
int readIndex_Fwd = 0;              // the index of the current reading
int readIndex_Ref = 0;              // the index of the current reading
float total_Fwd = 0;                  // the running total
float total_Ref = 0;                  // the running total

#define EEPROM_SIZE 3000

struct Band_Cal {
  char  BandName[12];
  float Cpl_Fwd;
  float Cpl_Ref;
  int sc_P_Fwd;
  int sc_P_Ref;
} Band_Cal_Table_Def[10] = {
      {"50MHz", 72.3, 52.3, 5, 4},
     {"144MHz", 63.9, 43.9, 5, 4},
     {"222MHz", 61.0, 41.0, 5, 4},
     {"432MHz", 59.9, 39.9, 5, 4},
     {"902MHz", 58.1, 38.1, 5, 4},
    {"1296MHz", 71.4, 51.4, 5, 4},
     {"2.3GHz", 60.1, 40.1, 5, 4},
     {"3.4GHz", 60.2, 40.2, 5, 4},
     {"5.7GHz", 60.3, 40.3, 5, 4},
      {"10GHz", 60.4, 40.4, 5, 4}
    };

struct Band_Cal Band_Cal_Table[10];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

ESP32AnalogRead adc1;    // Class to correct A/D read values against factory cal data burned to chip
ESP32AnalogRead adc2;    // for second A/D channel

void setup(void) {
  
  M5.begin(true, false, true); // Init LCD, not SD cArd, not Serial
  M5.Lcd.setBrightness(30);
  M5.Lcd.setTextDatum(CC_DATUM);
  dacWrite (25,0); // silence speaker interference
  // M5.Lcd.setRotation(0);
  if (EEPROM.read(4) =='Y')
      Serial.begin(115200); // For debug or data output
  char buf[80];
  Serial.println(" ");   // Clear our output text from CPU init text
  EEPROM.begin(EEPROM_SIZE);
 
  if (EEPROM.read(0) == 'G') {
    Serial.println("EEPROM Data is Valid");
    get_config_EEPROM();  // set last used values for CouplerSetNum (Band) and op_mode if there is data in the EEPROM
  }
 
  if (EEPROM.read(0) != 'G') {    // Test if EEPROM has been initialized with table data yet
    write_Cal_Table_from_Default();  // Copy default values into memory
    write_Cal_Table_to_EEPROM(); // Copy memory into EEPROM if EEPROM is noit initialized yet. Byte 0 will get marked with a 'G'
    save_config_EEPROM();   // use default values to populate state storage area in EEPROM (first 16 bytes reserved for state variables)
    ser_data_out = 1;
    toggle_ser_data_output();   // set data output on
    ser_data_out = 0;
  }  // end initilization write calls
  
  // Read EEPROM
  read_Cal_Table_from_EEPROM();   // read cal data from EEPROM into memory
  
  Cal_Table();   // Load current Band values from Table
  init_screen();
  analogMeter();
  updateTime = millis(); // Next update time
  // initialize all the readings to 0:  Used to smooth AD values
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings_Fwd[thisReading] = 0;
    readings_Ref[thisReading] = 0;
  }
  //Serial.println(op_mode);
   adc1.attach(ad_Fwd);
   adc2.attach(ad_Ref);
  delay(1000);
}

void init_screen(void) 
{
  M5.Lcd.fillScreen(TFT_BLACK);
  //needle_value = 1;
  analogMeter(); // Draw analog meter

  M5.Lcd.setTextColor(TFT_WHITE);  // Text colour
  M5.Lcd.drawLine(0, int(M_SIZE*159), int(M_SIZE*239), int(M_SIZE*159), TFT_GREY);
  M5.Lcd.drawLine(int(M_SIZE*79), int(M_SIZE*159), int(M_SIZE*79), int(M_SIZE*119), TFT_GREY);
  M5.Lcd.drawLine(int(M_SIZE*161), int(M_SIZE*159), int(M_SIZE*161), int(M_SIZE*119), TFT_GREY);

  M5.Lcd.drawString("Forward", int(M_SIZE*39), int(M_SIZE*132), 2); // Fwd Pwr
  M5.Lcd.drawString("Reflected", int(M_SIZE*120), int(M_SIZE*132), 2); // Rev Pwr
  M5.Lcd.drawString("SWR", int(M_SIZE*199), int(M_SIZE*132), 2); // SWR
  
  M5.Lcd.drawString("PWR", int(M_SIZE*50), int(M_SIZE*170), 2); // Button A label
  M5.Lcd.drawString("Menu", int(M_SIZE*120), int(M_SIZE*170), 2); // Button B label
  M5.Lcd.drawString("SWR", int(M_SIZE*190), int(M_SIZE*170), 2); // Button C label

  //plotNeedle(1,0); // It takes between 2 and 12ms to replot the needle with zero delay
}

/*
// Return the supply voltage in volts.
float read_vcc()
{
    const float V_BAND_GAP = 1.1;     // typical
    ADMUX  = _BV(REFS0)    // ref = Vcc
           | 14;           // channel 14 is the bandgap reference
    ADCSRA |= _BV(ADSC);   // start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);  // wait until complete
    return V_BAND_GAP * 1024 / ADC;
}
*/

float adRead()   // A/D converter read function.  Normalize the AD output to 100%.
{
  float a;
  float a1;
  float b;
  int c;
  char buf[12];
  float tmp;
  
  M5.Lcd.setTextColor(TFT_BLACK, TFT_WHITE);  // Text colour

  // subtract the last reading:
  total_Fwd -= readings_Fwd[readIndex_Fwd];
  // read from the sensor:
  c = 10;   // short term smaples that feed into running average
  a = 0;
  adc1.attach(ad_Fwd);
  for (int i = 0; i < c; ++i) {
      // correction for AD non-linearity using factory data and correction library call for ESP32 AD
      a1 = adc1.readVoltage();
      //Vref = read_vcc();
      a1 = constrain(a1, Offset, 2.200);      
      a += a1;
      delay(5);
  }
  a /= c; // calculate the average then use result in a running average
  readings_Fwd[readIndex_Fwd] = a;   // get from the latest average above and track in this runnign average
  // add the reading to the total:
  total_Fwd += readings_Fwd[readIndex_Fwd];
  // advance to the next position in the array:
  readIndex_Fwd += 1;   
  // if we're at the end of the array...
  if (readIndex_Fwd >= numReadings) {
    // ...wrap around to the beginning:
    readIndex_Fwd = 0;
  }     
  // calculate the average:
  b = total_Fwd / numReadings;

  // caclulate dB value for digital display section
  b -= Offset;   // adjust to 0V reference point
  b /= Slope;
  b *= -1; // less than 0dBm so sign negative
  b += CouplingFactor_Fwd;
  b += 3.0; // Fudge factor for frequency independent factors like cabling

  // Now have calibrated Forward Value in dBm.
  Fwd_dBm = b;
  // 0dBm is max. = 1W fullscale on 1W scale for example
  // convert to linear value for meter using 1KW @ 0dBm as reference. Multiply by scale value.
  FwdPwr =  pow(10.0,(b-30.0)/10.0);
  if (FwdPwr > 9999)
      FwdPwr = 9999;
  FwdVal = FwdPwr;
  
  dtostrf(Fwd_dBm, 4, 1, buf);
  strncat(buf, "dBm", 3);
  M5.Lcd.drawRightString("          ", (M_SIZE*50), (M_SIZE*105), 2); // Clear Field
  M5.Lcd.drawRightString(buf, M_SIZE*(60), M_SIZE*(105), 2); // Test 
  // Serial.println(buf);
 
  // Now get Reflected Power 
  // subtract the last reading:
  total_Ref -= readings_Ref[readIndex_Ref];
  // read from the sensor:
  c = 10;
  a = 0;
  adc2.attach(ad_Ref);
  for (int i = 0; i < c; ++i)  {
      // correction for AD non-linearity using factory data and correction library call for ESP32 AD
      a1 = adc2.readVoltage();
      //Vref = read_vcc();
      a1 = constrain(a1, Offset, 2.200);
      a += a1;
      delay(5);
  }
  a /= c; // calculate the average then use result in a running average
  readings_Ref[readIndex_Ref] = a;   // get from the latest average above and track in this runnign average
  // add the reading to the total:
  total_Ref += readings_Ref[readIndex_Ref];
  // advance to the next position in the array:
  readIndex_Ref += 1;   
  // if we're at the end of the array...
  if (readIndex_Ref >= numReadings) {
    // ...wrap around to the beginning:
    readIndex_Ref = 0;
  }  
  // calculate the average:
  b = total_Ref / numReadings;
  
  // caclulate dB value for digital display section
  b -= Offset;   // adjust to 0V reference point
  b /= Slope;
  b *= -1;
  b += CouplingFactor_Ref;
  b += 3.0; // fudge factor for frequency independent factors like cabling
  
  Ref_dBm = b;
  // 0dBm is max. = 1W fullscale on 1W scale.
  // convert to linear value for meter using 1KW @ 0dBm as reference 
  RefPwr = pow(10.0,(b-30.0)/10.0);
  if (RefPwr > 999)
      RefPwr = 999;
  RefVal = RefPwr;
      
  dtostrf(Ref_dBm, 4, 1, buf);
  strncat(buf, "dBm", 3);
  M5.Lcd.drawRightString("        ", M_SIZE*(210), M_SIZE*(105), 2); // Clear Field
  M5.Lcd.drawRightString(buf, (M_SIZE*220), (M_SIZE*105), 2); // Rev Pwr
  // Serial.println(buf);

   //M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);  // Text colour
   // dtostrf(RefPwr, 1, 0, buf1);
   // strcpy(buf, "Ref PWR");
   // strcat(buf, buf1);
   // M5.Lcd.drawString(buf, int(M_SIZE*10), int(M_SIZE*20), 2); 
  
  // Write our Digital Values to Sreen here in dBm and SWR ratio
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour
  
  if (FwdPwr != FwdPwr_last) {
    if (FwdPwr >= 9999) strcpy(buf, "OVR ");
    else if (FwdPwr < 100) dtostrf(FwdPwr, 4, 1, buf);
    else dtostrf(FwdPwr, 4, 0, buf);   // remove decimal for larger values
    strncat(buf, "W\r", 2);  // write out the value in W on digital display zone
    M5.Lcd.fillRect(0, M_SIZE*138, M_SIZE*75, M_SIZE*18, TFT_BLACK);
    M5.Lcd.drawString(buf, M_SIZE*(39), M_SIZE*(149), 4); // Fwd Pwr write toe digital display zone

    if (RefPwr >= 999) strcpy(buf, "OVR ");
    else if (RefPwr < 100) dtostrf(RefPwr, 4, 1, buf);   // same as for forward power, remove decimal for large vcalues and display result
    else dtostrf(RefPwr, 4, 0, buf);
    strncat(buf, "W\r", 2);
    M5.Lcd.fillRect(M_SIZE*85, M_SIZE*138, M_SIZE*70, M_SIZE*18, TFT_BLACK);
    M5.Lcd.drawString(buf, M_SIZE*(119), M_SIZE*(149), 4); // Rev Pwr
    
    //VSWR = 1+sqrt of Pr/Pf  / 1-sqrt of Pr/Pf
    //if (RefPwr > FwdPwr) RefPwr = FwdPwr; 
    tmp = sqrt(RefPwr/FwdPwr);
    SWRVal = ((1 + tmp) / (1 - tmp));  // now have SWR in range of 1.0 to infinity.  
    if (FwdPwr <= 0.1)
        SWRVal = 0;   // remove misleading SWR numbers when inpout are floating around in RX mode.
    if (SWRVal > 9.9)
        SWRVal = 10;
    SWR_Serial_Val = SWRVal;
    M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour
    if (SWRVal == 0) strcpy(buf, "NA"); 
    else if (SWRVal < 4) dtostrf(SWRVal, 1, 1, buf);
    else if (SWRVal > 9.9) strcpy(buf, "inf");
    M5.Lcd.fillRect(M_SIZE*165, M_SIZE*138, M_SIZE*65, M_SIZE*18, TFT_BLACK);
    M5.Lcd.drawString(buf, M_SIZE*(199), M_SIZE*(149), 4); // SWR
    // Serial.println(buf);
    if (op_mode == SWR) {
      // scale to make SWR 1.0  to 4 == 0 to 100%
      SWRVal -=1;
      SWRVal *= 34;   // scale so 3.8 = 100
      needle_value = int(SWRVal);
      if (needle_value > 105) needle_value = 105;
      if (needle_value < -10) needle_value = -10;
    }
    else {
           needle_value = int(FwdVal/(scale_value_fwd/100));   // Use non-scaled value - must be between 0-100
           if (needle_value > 105) needle_value = 105;
           if (needle_value < -10) needle_value = -10;
         }
  }
  FwdPwr_last = FwdPwr;  // update memory to minimize screen update and flicker on digital number
  sendSerialData();   // send this data to the serial port for remote monitoring
  plotNeedle(needle_value, 6); // It takes between 2 and 12ms to replot the needle with zero delay
}

/*
 *   Send out dBm, Watts, and SWR values to data channel - serial, WiFi, or Bluetooth
*/
void sendSerialData()
{
    char tempbuf[80];
    if (EEPROM.read(4) == 'Y'){
        counter1++;
        if (counter1 > 16000) counter1 = 0;
        if ((counter1/1000 % 100 == 0)) {
            sprintf(tempbuf,"%d,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f", METERID, "170", Band_Cal_Table[CouplerSetNum].BandName, Fwd_dBm, Ref_dBm, FwdPwr, RefPwr, SWR_Serial_Val);
            Serial.println(tempbuf);
        }
    }
}

#define BUF_LEN 30

void get_remote_cmd(){
  static char sdata[BUF_LEN], *pSdata=sdata;
  char ch;
  int cmd1;
  float cmd2;
  int cmd_str_len;
  int i = 0; 
  int j = 0;   
  char cmd_str[BUF_LEN] = {};
  
  while (Serial.available() > 0)  { 
          
      ch = Serial.read();  // read one at a time looking for start of command
                                // amid a sea of possible other data incoming from anywhere
      if ((pSdata - sdata) >= BUF_LEN-1) {
          pSdata = sdata;
          sdata[0] = '\0';
          Serial.print("BUFFER OVERRRUN\n");
          Serial.flush();
          break;
      }

      // filter out unwanted characters             
      int x = isAlphaNumeric(ch);   // located on own line as a workaround:  Including this test with the other OR tests below teh result becomes inverted.
      if (x == 1 || ch == ',' || ch == '.' || ch=='\r' || ch == '\n') {      
           *pSdata++ = (char)ch;
           //Serial.println(sdata);   //  echo back what we heard 
          if (ch=='\r' || ch == '\n') {       // Command received and ready.
              pSdata --;       // Don't add \r and the last comma if any to string.
              *pSdata = '\0';  // Null terminate the string.       
              //Serial.println(sdata);   //  echo back what we heard
              cmd_str_len = strlen(sdata);
   
    // Now we have a full comma delimited command string - validate and break it down
              pSdata = sdata;
        
              j = 0;
              i = 0;
              for (i; sdata[i] != ','; i++) {
                  cmd_str[j++] = sdata[i];                 
              }    
              cmd_str[j] = '\0';  
              Serial.print(" Meter ID  ");
              Serial.println(cmd_str);
              
              if (atoi(cmd_str) == METERID) {                                 
                  if (i < cmd_str_len) {
                      j = 0;
                      i += 1;    // Look for Msg_Type now
                      for (i; sdata[i] != ','; i++) {   // i+1 to skip over the comma the var 'i' was left pointing at
                            cmd_str[j++] = sdata[i];                 
                      }
                      cmd_str[j] = '\0';
                      Serial.print(" Msg Type:  ");
                      Serial.println(cmd_str);
                  
                      if (atoi(cmd_str) == 120)  {   // Vaidate this message type for commands.  120 = coammand, 170 is data output                              
                            j = 0;
                            if (i < cmd_str_len) {
                                i += 1;
                                for (i; sdata[i] != ','; i++) {
                                    cmd_str[j++] = sdata[i];
                                }  
                                cmd_str[j] = '\0';
                                Serial.print(" CMD1:  ");
                                Serial.println(cmd_str);
                                cmd1 = atoi(cmd_str);
                            }
                            else 
                                cmd1 = 1;
                            
                            j = 0;
                            if (i < cmd_str_len) {
                                i += 1;                    
                                for (i; sdata[i] != ','; i++) {
                                    cmd_str[j++] = sdata[i];                          
                                }
                                cmd_str[j] = '\0';
                                Serial.print(" CMD2:  ");
                                Serial.println(cmd_str);
                                cmd2 = atof(cmd_str);
                            }
                            else 
                                cmd2 = 1.0;
                      
                 // Now do the commands     
                            // Process received commands
                            // add code to limit variables received to allowed range
                            if (cmd1 == 255) Button_A = YES;   // switch scale - not really useful if you cannot see the meter face, data is not changed
                            if (cmd1 == 254) {
                                Button_B = YES;   // switch bands
                                CouplerSetNum = constrain(CouplerSetNum, 0, NUM_SETS);  
                                NewBand = CouplerSetNum +1;    // Update Newband to current value.  Will be incrmented in button function                          
                                if (NewBand > NUM_SETS) 
                                    NewBand = 0;  // cycle back to lowest band
                            }
                            if (cmd1 == 253) Button_C = YES;   // Switch op_modes betweem SWR and PWR - same as scale, not useful lif you cannot seethe meter face.
                            if (cmd1 == 252) ++Ser_Data_Rate;  //Speed up or slow down the Serial line output info rate
                            if (cmd1 == 251) --Ser_Data_Rate;  //Speed up or slow down the Serial line output info rate
                            if (cmd1 == 250) print_cal_table();   // dump current cal table to remote  (Was Scale GUI button)
                    
                            if (cmd1 == 249) {     // Jump to BandX
                                Button_B = YES;
                                NewBand = 9;  
                            }
                            if (cmd1 == 248) {     // Jump to BandX
                                Button_B = YES;
                                NewBand = 8;  
                            }
                            if (cmd1 == 247) {     // Jump to BandX
                                Button_B = YES;
                                NewBand = 7;  
                            }
                            if (cmd1 == 246) {     // Jump to BandX
                                Button_B = YES;
                                NewBand = 6;  
                            }
                            if (cmd1 == 245) {     // Jump to Band 1296
                                Button_B = YES;
                                NewBand = 5;  
                            }
                            if (cmd1 == 244) {     // Jump to Band 902
                                Button_B = YES;
                                NewBand = 4;  
                            }
                            if (cmd1 == 243) {     // Jump to Band 432
                                Button_B = YES;
                                NewBand = 3;  
                            }
                            if (cmd1 == 242) {     // Jump to Band 222
                                Button_B = YES;
                                NewBand = 2;  
                            }
                            if (cmd1 == 241) {     // Jump to Band 144
                                Button_B = YES;
                                NewBand = 1;  
                            }
                            if (cmd1 == 240) {     // Jump to Band 50
                                Button_B = YES;
                                NewBand = 0;  
                            }
                            if (cmd1 == 239) {     // Toggle Serial power data outpout.  Other serial functions remain available.
                                toggle_ser_data_output();
                            }
                            if (cmd1 == 193) {    // Set up for potential EEPROM Reset if followed by 5 second press on Button C
                                Reset_Flag = 1;
                            }
                            if (cmd1 == 194) {     // Set Reset EEPROM flag.  Will repopulate after CPU reset
                                if (Reset_Flag == 1) 
                                    reset_EEPROM();
                                Reset_Flag = 0;   
                            }
                            if (cmd1 == 195) {    // Set up for potential EEPROM Reset if followed by 5 second press on Button C
                                resetFunc();  //call reset
                            }                            
                            // Handle remote command to change stored coupling factor to support headless ops.
                            // TODO: Need to write into EEPROM, either here or by changing bands.                          
                            if (cmd1 >= 100 && cmd1 < 110) {     // Change Fwd coupling factor for Port X
                                int index;
                                index = cmd1 - 100;
                                Serial.print("Band: ");
                                Serial.print(Band_Cal_Table[index].BandName);
                                Serial.print(" --- Old Fwd Value: ");
                                Serial.print(Band_Cal_Table[index].Cpl_Fwd);
                                Band_Cal_Table[index].Cpl_Fwd = cmd2;       // cmd2 is second byte in 2 byte payload.                              
                                Serial.print(" +++ New Fwd Value: ");
                                Serial.println(Band_Cal_Table[index].Cpl_Fwd);
                                write_Cal_Table_to_EEPROM();  // save to eeprom
                            }
                            if (cmd1 >= 110 && cmd1 < 120) {     // Change Ref coupling factor for Port X
                                int index;
                                index = cmd1 - 110;
                                Serial.print("Band: ");
                                Serial.print(Band_Cal_Table[index].BandName);
                                Serial.print(" --- Old Ref Value: ");
                                Serial.print(Band_Cal_Table[index].Cpl_Ref);
                                Band_Cal_Table[index].Cpl_Ref = cmd2;       // cmd2 is second byte in 2 byte payload.                              
                                Serial.print(" +++ New Ref Value: ");
                                Serial.println(Band_Cal_Table[index].Cpl_Ref);
                                write_Cal_Table_to_EEPROM();   // save to eeprom on changes.  
                            }                        
                            // validate to acceptable range of values
                            Ser_Data_Rate = constrain(Ser_Data_Rate, 1, 20);
                            Button_A = constrain(Button_A, 0, 1);
                            Button_B = constrain(Button_B, 0, 1);
                            Button_C = constrain(Button_C, 0, 1);
                            NewBand = constrain(NewBand, 0, NUM_SETS);                                     
                        } // end of msg_type 120                                      
                    } // end of msg_type length check
                } // end of meter ID OK    
                //pSdata = sdata; // Reset pointer to start of string. 
            } // end '/r'  
        } // end if valid characters
        else {
            Serial.print(" Bad Character -  Reset Buffer = ");
            pSdata = sdata;
            sdata[0] = '\0';
            Serial.println(ch);
            Serial.flush();
        } // end else not valid characters
    } //while serial available
} // end get_remote_cmd function

void loop() { 
  
  char buf[50];
  char buf1[12];
  int tmp;

  // Listen for remote computer commands
  get_remote_cmd();
 
  M5.update();
  if (M5.BtnA.wasReleased() || Button_A == YES) {   // Do a press and hold to display reflected power on analog meter. 
    Button_A = NO; //reset flag
    if (op_mode != PWR) {
      // coming from some other op_mode, redraw the screen for PWR op_mode
      op_mode = PWR;  
      init_screen();
      analogMeter();
    }
    else {
      scale_PWR_Fwd += 1;   // If already in PWR op_mode, increment scale. Otherwise leave scale as it was.
      scale_PWR_Ref += 1;
    }
    op_mode= PWR;
    
    if (scale_PWR_Fwd > 7 || scale_PWR_Fwd < 1) scale_PWR_Fwd = 1;
    switch (scale_PWR_Fwd) {
      case 1: scale_value_fwd = 1;     break;
      case 2: scale_value_fwd = 5;     break;
      case 3: scale_value_fwd = 10;    break;
      case 4: scale_value_fwd = 50;    break;
      case 5: scale_value_fwd = 100;   break;
      case 6: scale_value_fwd = 500;   break;
      case 7: scale_value_fwd = 1000;  break;
      default: scale_value_fwd = 100;  break;
      //break;
    }   
    
    if (scale_PWR_Ref > 7 || scale_PWR_Ref < 1) scale_PWR_Ref = 1;
    switch (scale_PWR_Ref) {
      case 1: scale_value_ref =1;     break;
      case 2: scale_value_ref =5;     break;
      case 3: scale_value_ref =10;    break;
      case 4: scale_value_ref =50;    break;
      case 5: scale_value_ref =100;   break;
      case 6: scale_value_ref =500;   break;
      case 7: scale_value_ref =1000;  break;
      default: scale_value_ref = 100; break;
      //break;
    }  
    
    // Save to EEPROM s
    Band_Cal_Table[CouplerSetNum].sc_P_Fwd = scale_PWR_Fwd;
    Band_Cal_Table[CouplerSetNum].sc_P_Ref = scale_PWR_Ref;
    save_config_EEPROM();   
    write_Cal_Table_to_EEPROM();
    
    analogMeter(); // Draw analog meter
    //plotNeedle(1,0); // It takes between 2 and 12ms to replot the needle with zero delay
  }
  if (M5.BtnB.wasReleasefor(700)){
    save_config_EEPROM();
    op_mode = MENU;
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE);  // Text color
    Menu_Nav();
  }
  if (M5.BtnB.wasReleased() || Button_B == YES) {      // Select Cal Band
    ++CouplerSetNum;   // increment for manual button pushes
    if (CouplerSetNum > NUM_SETS) 
      CouplerSetNum = 0;
    CouplerSetNum = constrain(CouplerSetNum, 0, NUM_SETS);
    if (Button_B == YES)
      CouplerSetNum = NewBand;    // set to commanded band.  If a Button B remote cmd, NewBand wil lbe incremented before here
    Button_B = NO; //reset flag
    
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE);  // Text colour
    Cal_Table();
    save_config_EEPROM();
    init_screen();
    analogMeter();
  }
  if (M5.BtnC.wasReleased() || Button_C == YES) {
    Button_C = NO; //reset flag
    if (op_mode != SWR)  {
    op_mode = SWR;
    save_config_EEPROM();
    init_screen();
    analogMeter(); // Draw analogue meter
    //plotNeedle(1,0); // It takes between 2 and 12ms to replot the needle with zero delay
    }
  } 
  if (M5.BtnA.wasReleasefor(5000)){
       Serial.println(" Button A pressed for over 5 seconds");  
       // Set up for potential EEPROM Reset if followed by 5 second press on Button C
       Reset_Flag = 1;
  }  
  if (M5.BtnC.wasReleasefor(5000)){
       Serial.println(" Button C pressed for over 5 seconds");  
       // Set Reset EEPROM flag.  Will repopulate after CPU reset
       if (Reset_Flag == 1) 
          reset_EEPROM();
       Reset_Flag = 0;            
  }
  if (M5.BtnC.wasReleasefor(10000)){
       Serial.println("Toggle Serial Data Output"); 
       toggle_ser_data_output();
  }
  if (op_mode != MENU && updateTime <= millis()) {
    updateTime = millis() + 45; // Update timer every 35 milliseconds
 
    // Create a Sine wave for testing
    //d += 1; if (d >= 360) d = 0;
    //FwdPwr = 50 + 50 * sin((d + 0) * 0.0174532925);
    
    adRead(); //get and calculate power + SWR values and display them
  }
}

/*  
  Main Menu to handle calibration data entry and choosing saved couple sets.  Start with one then ad more sets once working good.
  Need to have multiple frequency points stored for each coupler port.
*/
void Menu_Nav()
{
    char buf1[80]; 
    char buf[80];
    int scale_FS_fwd;
    int scale_FS_ref;
    int Cal_Band = 0;
    int Selection = 0;    
    int i;
  
/*     
  Use right and left buttons to dial in calibration values basd on kown power levels.
       Example:
       long press press center button to enter Cal screen starting with list of bands
       Choose Band from list using up and down buttons - bands are predefined
       Press Center button to accept band selection and move to cal screen for that band
       Display Current values for Fwd adn Ref Attenuation.  Initially read from default in code
       Highlight Fwd Value
       Press up or down and increment value
       press center button to accept and move to other field
       press up or down to change value
       press center to accept and move to other field
       long press center to data entry fo rthe band and display list of bands
       choose another band if desired with short center button press
       long press to exit cal op_mode and default to PWR screen
*/
// Draw list of bands (Predefined to 10 bands - customize teh BandName lable in code only - Att values are editiable)
    M5.Lcd.drawString("Calibration Menu", int(M_SIZE*10), int(M_SIZE*10), 2); 
    M5.Lcd.drawRightString("Hold Menu Button 1 Sec to EXIT", int(M_SIZE*10), int(M_SIZE*160), 2);
    M5.Lcd.drawRightString(" BAND             FWD           REF    ", int(M_SIZE*230), int(M_SIZE*30), 2);
    for (i=0; i<10; i++) {
      sprintf(buf, "%12s          %3.1f          %3.1f", Band_Cal_Table[i].BandName, Band_Cal_Table[i].Cpl_Fwd, Band_Cal_Table[i].Cpl_Ref);
      M5.Lcd.drawRightString(buf, int(M_SIZE*210), int(M_SIZE*(40+(i*10))), 2);
    }

    // Highlight field to edit
    Cal_Band = CouplerSetNum*2;  // Set highlight to current band
    Draw_Cursor(Cal_Band, 1);

  // Loop around acting on edit op_mode navigation buttons until long press detected to exit edit op_mode
  do {
      M5.update();   //Scan for button presses
      if (M5.BtnC.wasReleased()) { 
          if (!Edit_Atten & Cal_Band < (NUM_SETS*2)+1) ++Cal_Band;
          Draw_Cursor(Cal_Band, 1);           
      }
      if (M5.BtnA.wasReleased()) { 
          if (!Edit_Atten & (Cal_Band > 0)) --Cal_Band;
          Draw_Cursor(Cal_Band, -1);
      }
      if (M5.BtnB.wasReleased()) { 
          if (!Edit_Atten) {
            Edit_Atten = 1;
            edit_ATT(Cal_Band);
          }
          else Edit_Atten = 0;
      }
      if (M5.BtnB.wasReleasefor(700)) {
          get_config_EEPROM();
          init_screen();
          analogMeter(); // Draw analogue meter
          return;   // All done inb Edit mode, exit this funtion
      }
    } while (1);  // end forever while loop.  Exit when Button B is long pressed
}

/*
 * While in Edit mode, this function makes the actual field change and stores whole structure in EEPROM at end
*/
void edit_ATT(int field)
{
  float new_Val;
  char buf[80];
    // Use the cursor keys to dial up and down the selected attennuation value.
    // Press button B (Center) to accept displayed value.
    // Use the current value as the starting point. 

    CouplerSetNum = field/2;
    if (field % 2){
       new_Val = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
       M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour 
       sprintf(buf, "Edit New Value: %3.1f", new_Val);
       M5.Lcd.drawRightString(buf, M_SIZE*(220), M_SIZE*(4), 2);
    }
    else {
       new_Val = Band_Cal_Table[CouplerSetNum].Cpl_Fwd;
       M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour 
       sprintf(buf, "Edit New Value: %3.1f", new_Val);
       M5.Lcd.drawRightString(buf, M_SIZE*(220), M_SIZE*(4), 2);
    }
    
    do {   // While in Edit mode, edit actual values here
         M5.update();   //Scan for button presses
         if (M5.BtnC.wasReleased()) { 
            new_Val += 0.1;
            sprintf(buf, "Edit New Value: %3.1f", new_Val);
            M5.Lcd.drawRightString(buf, M_SIZE*(220), M_SIZE*(4), 2);                 
         }
          
         if (M5.BtnA.wasReleased()) { 
            new_Val -= 0.1;
            sprintf(buf, "Edit New Value: %3.1f", new_Val);
            M5.Lcd.drawRightString(buf, M_SIZE*(220), M_SIZE*(4), 2);
         }
          
         if (M5.BtnB.wasReleased()) { 
            if (field % 2) {
              Band_Cal_Table[CouplerSetNum].Cpl_Ref = new_Val;
            }
            else {
              Band_Cal_Table[CouplerSetNum].Cpl_Fwd = new_Val;
            }
            // Store Updated Cal Table Structure in EEPROM
            //Serial.println(Band_Cal_Table[CouplerSetNum].BandName);
            //Serial.println(Band_Cal_Table[CouplerSetNum].Cpl_Fwd);
            //Serial.println(Band_Cal_Table[CouplerSetNum].Cpl_Ref);
            write_Cal_Table_to_EEPROM();
            // Update Table of displayed values
            sprintf(buf, "%12s          %3.1f          %3.1f", Band_Cal_Table[CouplerSetNum].BandName, Band_Cal_Table[CouplerSetNum].Cpl_Fwd, Band_Cal_Table[CouplerSetNum].Cpl_Ref);
            M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour
            M5.Lcd.drawRightString(buf, int(M_SIZE*210), int(M_SIZE*(40+(CouplerSetNum*10))), 2);
            Draw_Cursor(field, 1); 
            M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour
            M5.Lcd.drawRightString("              ACCEPT    ", M_SIZE*(220), M_SIZE*(4), 2);
            Edit_Atten = 0;   
            return;         
         }
         if (M5.BtnB.wasReleasefor(700)) {
            return;
         }
    } while (1);
}

// read cal data from EEPROM (state is another function)
void read_Cal_Table_from_EEPROM()
{
   int i;
   
   for (i=0; i<10; i++) {
   EEPROM.get(EEADDR+(sizeof(Band_Cal_Table)*i), Band_Cal_Table[i]);
   /*
    * Serial.println("EEPROM Read");
   Serial.println(Band_Cal_Table[i].BandName);
   Serial.println(Band_Cal_Table[i].Cpl_Fwd);
   Serial.println(Band_Cal_Table[i].Cpl_Ref);
   Serial.println(Band_Cal_Table[i].sc_P_Fwd);
   Serial.println(Band_Cal_Table[i].sc_P_Ref);
   */
   delay(10); 
   }
}

// Copy cal data in memory to EEPROM to preserve user changes (save state is another function)
void write_Cal_Table_to_EEPROM()
{
   int i;
  
   for (i=0; i< 10;i++) {
   EEPROM.put(EEADDR+(sizeof(Band_Cal_Table)*i), Band_Cal_Table[i]);
   /*
    * Serial.println("EEPROM Written");
   Serial.println(Band_Cal_Table[i].BandName);
   Serial.println(Band_Cal_Table[i].Cpl_Fwd);
   Serial.println(Band_Cal_Table[i].Cpl_Ref);
   Serial.println(Band_Cal_Table[i].sc_P_Fwd);
   Serial.println(Band_Cal_Table[i].sc_P_Ref);
   */
   delay(10);
   EEPROM.write(0,'G');
   }
   EEPROM.commit();
   // Serial.println("Wrote to EEPROM");
}

// Copy hard coded al data to memory
void write_Cal_Table_from_Default()
{
  int i;
  
  for (i=0;i<10;i++) {
    // Populate initial database scructure in RAM.  Ultimately it needs to be read from EEPROM once initialized
    strcpy(Band_Cal_Table[i].BandName, Band_Cal_Table_Def[i].BandName);
    Band_Cal_Table[i].Cpl_Fwd = Band_Cal_Table_Def[i].Cpl_Fwd;
    Band_Cal_Table[i].Cpl_Ref = Band_Cal_Table_Def[i].Cpl_Ref;
    Band_Cal_Table[i].sc_P_Fwd = Band_Cal_Table_Def[i].sc_P_Fwd;  // Use 100W scale for now
    Band_Cal_Table[i].sc_P_Ref = Band_Cal_Table_Def[i].sc_P_Ref;  // Use 10W scale for now - program automaitcally set to 1 less than the fwd scale for now
    //Serial.println("Copied default data info Band Cal Table");
  }
}

// Read state in EEPROM
void get_config_EEPROM()
{
    CouplerSetNum = EEPROM.read(2)-'0';
    //Serial.print("CouplerSetNum Read = ");
    //Serial.println(CouplerSetNum);
    op_mode = EEPROM.read(3)-'0';
    //Serial.print("op_mode Read = ");
    //Serial.println(op_mode);
    EEPROM.commit();
}

// Save state in EEPROM
void save_config_EEPROM()
{
    char buf[3];
    CouplerSetNum = constrain(CouplerSetNum,0,NUM_SETS);
    EEPROM.write(2, (CouplerSetNum+'0'));
    //Serial.print("CouplerSetNum Write =");
    //Serial.println(CouplerSetNum);
    op_mode = constrain(op_mode,1,3);
    if (op_mode == 1 || op_mode == 2) {
        EEPROM.write(3, byte(op_mode)+'0');
        //Serial.print("op_mode Write = ");
        //Serial.println(op_mode);
    }
    EEPROM.commit();
}

// Mark EEPROM for overwrite
void reset_EEPROM()
{
    if (Reset_Flag ==1) {
        // erase byte 0 to force init process for testing.  Can also use to "factory" reset data
        EEPROM.write(0, '0');
        EEPROM.commit();
        Serial.println("Erased Byte 0");
    }
}


// Toggle USB serial output data
void toggle_ser_data_output()
{
     if (EEPROM.read(4) != 'Y' || ser_data_out == 1){
        EEPROM.write(4, 'Y');
        EEPROM.commit();
        Serial.println("Enabled Serial Data Output");
        ser_data_out = 0;
     }
     else { 
        EEPROM.write(4, 'N');
        EEPROM.commit();
        Serial.println("Disabled Serial Data Output");
     }
}

void Draw_Cursor(int Index, int Direction)
{
  int Selection;
  
  if (Index % 2) {  
      Selection = (((Index/2))*10) + 40;
      M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour
      M5.Lcd.drawRightString("[", int(M_SIZE*10), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString("]", int(M_SIZE*85), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString("<", int(M_SIZE*186), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*215), int(M_SIZE*Selection), 2);
      M5.Lcd.setTextColor(TFT_BLACK,TFT_BLACK);  // Text colour
      M5.Lcd.drawRightString("<", int(M_SIZE*120), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*150), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString("[", int(M_SIZE*10), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString("]", int(M_SIZE*85), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString("<", int(M_SIZE*186), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*215), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString("<", int(M_SIZE*120), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*150), int(M_SIZE*(Selection-(Direction*10))), 2);
  }       
  else {   
      Selection = ((Index/2)*10) + 40;
      M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  // Text colour
      M5.Lcd.drawRightString("[", int(M_SIZE*10), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString("]", int(M_SIZE*85), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString("<", int(M_SIZE*120), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*150), int(M_SIZE*Selection), 2);
      M5.Lcd.setTextColor(TFT_BLACK,TFT_BLACK);  // Text colour
      M5.Lcd.drawRightString("<", int(M_SIZE*186), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*215), int(M_SIZE*Selection), 2);
      M5.Lcd.drawRightString("[", int(M_SIZE*10), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString("]", int(M_SIZE*85), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString("<", int(M_SIZE*120), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*150), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString("<", int(M_SIZE*186), int(M_SIZE*(Selection-(Direction*10))), 2);
      M5.Lcd.drawRightString(">", int(M_SIZE*215), int(M_SIZE*(Selection-(Direction*10))), 2);
  }
}

void Cal_Table()
{
  //
  // Each coupler has unique coupling factor.  The program assumes all math relative to 0dBm (1mW).  
  // So these values will slide the refence pioint up or down based on the net total of attenuator
  // and coupling factor for fwd and ref ports, per frequency band.  
  // The more frequency bands in the table the better but at more effort to code and to enter cal data.
  // The AD8318 detector is fairly linear over frequency and power betwen 0 and -60 so trying to
  // operate there and no correction required for that end of things, just hte coupler itself plus added attenuators.
  //
  // Copy Selected Band Cal Data from Table to working variables  
  strcpy(Coupler_friendly_name, Band_Cal_Table[CouplerSetNum].BandName);
  CouplingFactor_Fwd = Band_Cal_Table[CouplerSetNum].Cpl_Fwd;  // value in dB from coupler specs.  
      // Program should account for this by muliplying measured value by this amount.  
      // For example, Fwd_dBm + 30.  If dBm measures at -22dBm, coupling factor is 30, then the actual value
      // at the coupler input port is x1000 higher, so +8dBm  (30-22=+8) or nearly 10mW.  
      // 50W input to coupler would show 1000 50mW or +17dBm
  CouplingFactor_Ref = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
  scale_PWR_Fwd = Band_Cal_Table[CouplerSetNum].sc_P_Fwd;  // Use 100W scale for now
  scale_PWR_Ref = Band_Cal_Table[CouplerSetNum].sc_P_Ref;  // Use 10W scale for now - program automaitcally set to 1 less than the fwd scale for now
}

// #########################################################################
//  Draw the analogue SWR meter on the screen
// #########################################################################
void analogMeter() {

  char  buf[11];
 
  // Meter outline
  M5.Lcd.fillRect(0, 0, M_SIZE*239, M_SIZE*126, TFT_GREY);
  M5.Lcd.fillRect(5, 3, M_SIZE*230, M_SIZE*119, TFT_WHITE);
  M5.Lcd.setTextColor(TFT_BLACK);  // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    uint16_t y0 = sy * (M_SIZE*100 + tl) + M_SIZE*140;
    uint16_t x1 = sx * M_SIZE*100 + M_SIZE*120;
    uint16_t y1 = sy * M_SIZE*100 + M_SIZE*140;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE*100 + tl) + M_SIZE*120;
    int y2 = sy2 * (M_SIZE*100 + tl) + M_SIZE*140;
    int x3 = sx2 * M_SIZE*100 + M_SIZE*120;
    int y3 = sy2 * M_SIZE*100 + M_SIZE*140;

    // Choose SWR or Watts Display with L and R buttons in Run mode

    // Draw SWR  meter face and fixed scale
    // SWR is fixed at 5.  Adjust orange and red zones.
      if (op_mode == SWR) {  
        // Yellow zone limits
        if (i >= -25 && i < 20) {
          M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
          M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
        }
        
        // Green zone limitsforward or backward 
        //if (i >= -50 && i < 0) {
        //  M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
        //  M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
        //}
        
        // Red zone limits
        if (i >= 20 && i < 50) {
          M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
          M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
        }
      }
      
      // Short scale tick length
      if (i % 25 != 0) tl = 8;
      
      // Recalculate coords incase tick length changed
      x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
      y0 = sy * (M_SIZE*100 + tl) + M_SIZE*140;
      x1 = sx * M_SIZE*100 + M_SIZE*120;
      y1 = sy * M_SIZE*100 + M_SIZE*140;
      
      // Draw tick
      M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
      
      // Check if labels should be drawn, with position tweaks
      if (i % 25 == 0) {
        // Calculate label positions
        x0 = sx * (M_SIZE*100 + tl + 10) + M_SIZE*120;
        y0 = sy * (M_SIZE*100 + tl + 10) + M_SIZE*140;
        if (op_mode == PWR)  {
            switch (scale_PWR_Fwd) { 
              case 1: {   // 1W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("0.25", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("0.5", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("0.75", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("1", x0, y0 - 12, 2); break;           
                }  
                break;
              }   
              case 2: {   // 5W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("1.25", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("2.5", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("3.75", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("5", x0, y0 - 12, 2); break;              
                }   
                break;
              } 
              case 3: {   // 10W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("2.5", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("5", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("7.5", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("10", x0, y0 - 12, 2); break;                  
                }    
                break;           
              } 
              case 4: {   // 50W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("12.5", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("25", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("37.5", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("50", x0, y0 - 12, 2); break;
                }
                break;
              }                
              case 5: {   // 100W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("25", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("50", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("75", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("100", x0, y0 - 12, 2); break;
                }
                break;
              }     
              case 6: {   // 500W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("125", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("250", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("375", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("500", x0, y0 - 12, 2); break;
                }
                break;
              }            
              case 7: {   // 1000W scale
                switch (i / 25) {
                  case -2:  M5.Lcd.drawString("0", x0, y0 - 12, 2); break;
                  case -1:  M5.Lcd.drawString("250", x0, y0 - 9, 2); break;
                  case 0:   M5.Lcd.drawString("500", x0, y0 - 7, 2); break;
                  case 1:   M5.Lcd.drawString("750", x0, y0 - 9, 2); break;
                  case 2:   M5.Lcd.drawString("1000", x0, y0 - 12, 2); break;
                }
                break;                         
              } 
            } 
        }
        if (op_mode == SWR)  {   
            switch (i / 25) {
              case -2:  M5.Lcd.drawCentreString("1", x0, y0 - 12, 2);  break;
              case -1:  M5.Lcd.drawCentreString("1.8", x0, y0 - 9, 2); break;
              case 0:   M5.Lcd.drawString("2.4", x0, y0 - 7, 2); break;
              case 1:   M5.Lcd.drawString("3.2", x0, y0 - 9, 2); break;
              case 2:   M5.Lcd.drawString("~", x0, y0 - 12, 2); break;
              }            
        }
      }

      // Now draw the arc of the scale
      sx = cos((i + 5 - 90) * 0.0174532925);
      sy = sin((i + 5 - 90) * 0.0174532925);
      x0 = sx * M_SIZE*100 + M_SIZE*120;
      y0 = sy * M_SIZE*100 + M_SIZE*140;
      // Draw scale arc, don't draw the last part
      if (i < 50) M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }
     
  if (op_mode == PWR) M5.Lcd.drawString("Watts", M_SIZE*120, M_SIZE*70, 4); // Comment out to avoid font 4
  if (op_mode == SWR) M5.Lcd.drawString("SWR", M_SIZE*119, M_SIZE*70, 4); // Comment out to avoid font 4
  
  // Show Band/Cal Set Selected on the display meter face 
  if (op_mode == SWR || op_mode == PWR) {
    strncpy(buf, Coupler_friendly_name, 10);
    buf[10] = '\0';
    M5.Lcd.drawString(buf, int(M_SIZE*130), int(M_SIZE*90), 2);  
  }
    
  M5.Lcd.drawRect(5, 3, M_SIZE*230, M_SIZE*119, TFT_BLACK); // Draw bezel line

  plotNeedle(1, 0); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(float value, byte ms_delay)
{
  char  buf[11];
  
  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);

  if (value < 0) value = 0; // Limit value to emulate needle end stops
  if (value > 105) value = 105;

  // Move the needle until new value reached
  while (!(value == old_analog)) {
    if (old_analog < value) old_analog++;
    else old_analog--;

    if (ms_delay == 0) old_analog = value; // Update immediately if delay is 0

    float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
    // Calculate tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    M5.Lcd.drawLine(M_SIZE*(120 + 20 * ltx - 1), M_SIZE*(140-20), osx - 1, osy, TFT_WHITE);
    M5.Lcd.drawLine(M_SIZE*(120 + 20 * ltx), M_SIZE*(140-20), osx, osy, TFT_WHITE);
    M5.Lcd.drawLine(M_SIZE*(120 + 20 * ltx + 1), M_SIZE*(140-20), osx + 1, osy, TFT_WHITE);

    // Re-plot text under needle
    M5.Lcd.setTextColor(TFT_BLACK);
    if (op_mode == PWR) M5.Lcd.drawString("Watts", M_SIZE*120, M_SIZE*70, 4); // // Comment out to avoid font 4
    if (op_mode == SWR) M5.Lcd.drawString("SWR", M_SIZE*120, M_SIZE*70, 4); // // Comment out to avoid font 4
    // Show Band/Cal Set Selected on the display meter face 
    if (op_mode == SWR || op_mode == PWR) {
      strncpy(buf, Coupler_friendly_name, 10);
      buf[10] = '\0';
      M5.Lcd.drawString(buf, int(M_SIZE*130), int(M_SIZE*90), 2);  
    }
    
    // Store new needle end coords for next erase
    ltx = tx;
    osx = M_SIZE*(sx * 98 + 120);
    osy = M_SIZE*(sy * 98 + 140);

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    M5.Lcd.drawLine(M_SIZE*(120 + 20 * ltx - 1), M_SIZE*(140-20), osx - 1, osy, TFT_RED);
    M5.Lcd.drawLine(M_SIZE*(120 + 20 * ltx), M_SIZE*(140-20), osx, osy, TFT_MAGENTA);
    M5.Lcd.drawLine(M_SIZE*(120 + 20 * ltx + 1), M_SIZE*(140-20), osx + 1, osy, TFT_RED);

    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

    // Wait before next update
    delay(ms_delay);
  }
}

void print_cal_table()
{
    int i;
    char buf[80];

    // example ouput format : "101,150,TEXT,55.4,35.4,3.3,2.2"
    // #150 for msg_type field to signal this is data dump, not power levels or other type messages.
    for (i=0; i <= NUM_SETS; i++) {
        //sprintf(buf, "%d,%s,%s,%f.2,%f.2",    // meterid with msg_type = 150 to signal different data set than the normal output. 120 inbound cmd, 170, power out
        //METERID,
        //"150",  //msg_tyope for cmd reply
        //Band_Cal_Table[i].BandName,
        //Band_Cal_Table[i].Cpl_Fwd,  // value in dB from coupler specs.  
        //Band_Cal_Table[i].Cpl_Ref
        //);
        //Serial.println(buf);   // Output table text to serial port    
        //sprintf not working on Nano for some reason.  So doing it the hard way.
        Serial.print(METERID);
        Serial.print(",150,");
        Serial.print(Band_Cal_Table[i].BandName);
        Serial.print(",");
        Serial.print(Band_Cal_Table[i].Cpl_Fwd);
        Serial.print(",");
        Serial.println(Band_Cal_Table[i].Cpl_Ref);                
    }
}
