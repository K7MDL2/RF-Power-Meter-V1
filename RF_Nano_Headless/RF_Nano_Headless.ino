#include <EEPROM.h>

/*
 *
 * RF Power Meter by K7MDL 5/8/2020   - Remote (Headless) Edition for Testing on Arduino Nano 
 *
 * 5/10/2020 - Stripped out physical button tests (kept code as virtual buttons).  Removed rest of M5/ESP32 dependencies,
 *        added standard Arduino replacements where needed.
 *
 * 5/8/2020 - Expanded remote commands to support dumping the cal table and writing to individual coupling
 *    factor cal values and saving to EEPROM.  Switch from a one byte command to a string with similar structure
 *    as the power level out.  Changed sequence number to msg_type fo future expansion and to help validate the 
 *    incoming message better from random data/noise/CPU status messages.
 *    Thre is still some screen drawing mostly in the cal area yet to be removed.  Considering leaving the digital
 *    values screen writing in for adaption to a 2x16/4x20 LCD or small graphics OLED embedded in an RFampifier as
 *    the main meter with SWR shutdowna nd other feature (including remote monitoring).
 *    One concern for getting Wi-Fi to work is the possibility the ESP32 int eh M5stack I am using may take over ADC2 pins 
 *    which would be a problem.  Also the internal noise coudl be improved using an external I2C connected A/D.
 *
 * 5/7/2020 -  Builds on RF Power Meter dated 5/7.  
 *    Begin strippoing the Display  and M5 specific components to allow for 
 *      standard Arduino and headless operation
 *
 Has user edited Calibration sets. Have 10 "bands" for frequency correction used with values that can be edited via the UI ---

 In this code example I use a RLC .05-1000MHz coupler and created 10 cal bands toi cover 50M to 10G.
 A value for each dual directional coupler port combines the coupler facxtor, added attenuators, and any cal correction fudge factor.
 The values are edited via the device UI and stored in EEPROM.
*/

// Define meter size as 1 for M5.Lcd.rotation(0) or 1.3333 for M5.Lcd.rotation(1)
#define M_SIZE 1.3333

#define METERID 101 // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station
#define METER_RATE 2   // used to skip serial data output to a lower rate
#define TEST_EEPROM 0  // change to 1 and reprogram, then set back to 0 to reset EEPROM data to default
#define SWR 2
#define PWR 1
#define MENU 3
#define EEADDR 16 // Start location to write data table structure in EEPROM.  Byte level data values will start at 2.  EEPROM status is byte 0
#define NO 0
#define YES 1

// Edit the Coupler Set data inb teh Cal_Table function.  Set the max number of sets here, and the default to load at startup
#define NUM_SETS 5 // 10 bands, 0 through 9 for example
int CouplerSetNum = 0;   // 0 is the default set on power up.  
float Vref = 5.0;           // 3.3VDC for Nano and ESP32 (M5stack uses ESP32)  ESP32 also has calibrated Vref curve
int adc_counts = 1024;      // 4095 for ESP32 12bit, 1023 for 10 bit ESP32 and Nano.

int ser_data_out = 0;
int Reset_Flag = 0;
uint32_t updateTime = 0;       // time for next update
const int ad_Fwd = A1; // Analog 35 pin for channel 0
const int ad_Ref = A2; // Analog 36 pin for channel 1
float Fwd_dBm = 0;
float Ref_dBm = 0;
float FwdPwr = 0;
float RefPwr = 0;
float SWRVal = 0;
float SWR_Serial_Val = 0;
float FwdVal = 0;
float RefVal = 0;
int d = 0;
int Edit_Atten = 0;
int op_mode = SWR;
int counter1 = 0;
int Button_A = 0;
int Button_B = 0;
int Button_C = 0;
int NewBand;
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
const int numReadings = 12;   // adjust this for longer or shorter smooting period as AD noise requires
float readings_Fwd[numReadings];      // the readings from the analog input
float readings_Ref[numReadings];      // the readings from the analog input
int readIndex_Fwd = 0;              // the index of the current reading
int readIndex_Ref = 0;              // the index of the current reading
float total_Fwd = 0;                  // the running total
float total_Ref = 0;                  // the running total

#define EEPROM_SIZE 1024    //1024 for ATMega328P.  ESP32 is in Flash so can be any reasonable size.

struct Band_Cal {
  char  BandName[12];
  float Cpl_Fwd;
  float Cpl_Ref;
} Band_Cal_Table_Def[10] = {
      {"50MHz", 74.3, 54.6},
     {"144MHz", 64.1, 44.9},
     {"222MHz", 61.4, 41.8},
     {"432MHz", 59.5, 39.8},
     {"902MHz", 58.6, 40.2},
    {"1296MHz", 72.6, 52.4},
     {"2.3GHz", 60.1, 40.1},
     {"3.4GHz", 60.2, 40.2},
     {"5.7GHz", 60.3, 40.3},
      {"10GHz", 60.4, 40.4}
    };
/*   Original calibration values dumped on remote command as an example before remotely issuing value changes.  
101-1,50MHz,74.3,54.599895.2
101-1,144MHz,64.099991.2,44.899979.2
101-1,222MHz,61.399986.2,41.799980.2
101-1,432MHz,59.499992.2,39.799988.2
101-1,902MHz,58.600006.2,40.199982.2
101-1,1296MHz,72.599937.2,52.399986.2
*/

struct Band_Cal Band_Cal_Table[10];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup(void) {
  pinMode(13,OUTPUT);
  //pinMode(A1,INPUT_PULLUP);
  //pinMode(A2,INPUT_PULLUP);
// Test tone to create voltage for testing on my Nano
  tone(13,1000);
  
  //write_Cal_Table_from_Default();  // Copy default values into memory
  //write_Cal_Table_to_EEPROM();
  //save_config_EEPROM();
  analogReference(DEFAULT);  // NA for ESP32
  //analogReadResolution(12);  // 10 or 12 for ESP32, 10 for most Arduinos.  Will automatically use 10Bit for 10bit CPU models

  Serial.begin(115200); // For debug or data output
  if (EEPROM.read(4*2) =='Y') {
     ser_data_out == 1;
     toggle_ser_data_output();   // set data output on
  }
  char buf[80];
  Serial.println(" ");   // Clear our output text from CPU init text
 
  if (EEPROM.read(0) == 'G') {
    Serial.println("EEPROM Data is Valid");
    //get_config_EEPROM();  // set last used values for CouplerSetNum (Band) and op_mode if there is data in the EEPROM
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
 
  updateTime = millis(); // Next update time
  // initialize all the readings to 0:  Used to smooth AD values
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings_Fwd[thisReading] = 0;
    readings_Ref[thisReading] = 0;
  }
  delay(1000);
}

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

float adRead()   // A/D converter read function.  Normalize the AD output to 100%.
{
    float a;
    float a1;
    float b;
    int c;
    char buf[12];
    float tmp;

    // subtract the last reading:
    total_Fwd -= readings_Fwd[readIndex_Fwd];
    // read from the sensor:
    c = 10;   // short term smaples that feed into running average
    a = 0;
    for (int i = 0; i < c; ++i) {
            a1 = analogRead(ad_Fwd);
            //Serial.print(a1);
            Vref = read_vcc();
            //Serial.print("  VCC = ");
            //Serial.print(Vref);
            a1 = a1 * (4.65/adc_counts);           // convert to voltage
            //Serial.print("  Fwd: ");
            //Serial.println(a1);
            a1 = constrain(a1, Offset, 2.200);      
            a += a1;
            delay(30);
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
    b += 1.5; // Fudge factor for frequency independent factors like cabling

    // Now have calibrated Forward Value in dBm.
    Fwd_dBm = b;
    // 0dBm is max. = 1W fullscale on 1W scale for example
    // convert to linear value for meter using 1KW @ 0dBm as reference. Multiply by scale value.
    FwdPwr =  pow(10.0,(b-30.0)/10.0);
    if (FwdPwr > 9999)
        FwdPwr = 9999;
    FwdVal = FwdPwr;
    
    //dtostrf(Fwd_dBm, 4, 1, buf);
    //strncat(buf, "dBm", 3);
    //Serial.println(buf);
    
    // Now get Reflected Power 
    // subtract the last reading:
    total_Ref -= readings_Ref[readIndex_Ref];
    // read from the sensor:
    c = 10;
    a = 0;
    for (int i = 0; i < c; ++i)  {
            a1 = analogRead(ad_Ref);
            Vref = read_vcc();
            //Serial.print("VCC = ");
            //Serial.println(Vref);
            a1 = a1 * (Vref/adc_counts);  // 10bit wit 3.3 Vref on Nano
            //Serial.print("Ref: ");
            //Serial.println(a1);
            //a1 = constrain(a1, Offset, 2.200);
            a += a1;
            delay(30);
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
    b += 1.5; // fudge factor for frequency independent factors like cabling
    
    Ref_dBm = b;
    // 0dBm is max. = 1W fullscale on 1W scale.
    // convert to linear value for meter using 1KW @ 0dBm as reference 
    RefPwr = pow(10.0,(b-30.0)/10.0);
    if (RefPwr > 999)
        RefPwr = 999;
    RefVal = RefPwr;
      
    //dtostrf(Ref_dBm, 4, 1, buf);
    //strncat(buf, "dBm", 3);
    //Serial.println(buf);

    // dtostrf(RefPwr, 1, 0, buf1);
    // strcpy(buf, "Ref PWR");
    // strcat(buf, buf1);
    // Serial.println(buf);
    
    // Write our Digital Values to Sreen here in dBm and SWR ratio
    
    //if (FwdPwr != FwdPwr_last) {
        //if (FwdPwr >= 9999) strcpy(buf, "OVR ");
        //else if (FwdPwr < 100) dtostrf(FwdPwr, 4, 1, buf);
        //else dtostrf(FwdPwr, 4, 0, buf);   // remove decimal for larger values
        //strncat(buf, "W\r", 2);  // write out the value in W on digital display zone
        
        //if (RefPwr >= 999) strcpy(buf, "OVR ");
        //else if (RefPwr < 100) dtostrf(RefPwr, 4, 1, buf);   // same as for forward power, remove decimal for large vcalues and display result
        //else dtostrf(RefPwr, 4, 0, buf);
        //strncat(buf, "W\r", 2);
        
    //VSWR = 1+sqrt of Pr/Pf  / 1-sqrt of Pr/Pf
    //if (RefPwr > FwdPwr) RefPwr = FwdPwr; 
    tmp = sqrt(RefPwr/FwdPwr);
    SWRVal = ((1 + tmp) / (1 - tmp));  // now have SWR in range of 1.0 to infinity.  
    if (FwdPwr <= 0.1)
        SWRVal = 0;   // remove misleading SWR numbers when inpout are floating around in RX mode.
    if (SWRVal > 9.9)
        SWRVal = 10;
    SWR_Serial_Val = SWRVal;

    FwdPwr_last = FwdPwr;  // update memory to minimize screen update and flicker on digital number
    sendSerialData();   // send this data to the serial port for remote monitoring
}

/*
 *   Send out dBm, Watts, and SWR values to data channel - serial, WiFi, or Bluetooth
*/
void sendSerialData()
{
    char tempbuf[50];

    if ((counter1/1000) % 100 == 0) {   // slow down the data rate.  Ideally do at at the AD read command
        //if (EEPROM.read(4*2) == 'Y'){   // only print if allowed
            Serial.print(METERID);
            Serial.print(",170,");
            Serial.print(Band_Cal_Table[CouplerSetNum].BandName);
            Serial.print(",");
            Serial.print(Fwd_dBm);
            Serial.print(",");
            Serial.print(Ref_dBm);
            Serial.print(",");
            Serial.print(FwdPwr);
            Serial.print(",");
            Serial.print(RefPwr);
            Serial.print(",");                       
            Serial.println(SWR_Serial_Val);
                        
            //sprintf(tempbuf,"%d,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f", METERID, "170", Band_Cal_Table[CouplerSetNum].BandName, Fwd_dBm, Ref_dBm, FwdPwr, RefPwr, SWR_Serial_Val);
            //Serial.println(tempbuf);
        //}    
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
          pSdata == sdata;
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
  
  // Listen for remote computer commands
    get_remote_cmd();

    if (Button_A == YES) {   // Do a press and hold to display reflected power on analog meter. 
        Button_A = NO; //reset flag
        if (op_mode != PWR) {
        // coming from some other op_mode, redraw the screen for PWR op_mode
        op_mode = PWR;  
        }                  
        // Save to EEPROM 
        save_config_EEPROM();   
        write_Cal_Table_to_EEPROM();
    }

    if (Button_B == YES) {      // Select Cal Band
        ++CouplerSetNum;   // increment for manual button pushes
        if (CouplerSetNum > NUM_SETS) 
        CouplerSetNum = 0;
        CouplerSetNum = constrain(CouplerSetNum, 0, NUM_SETS);
        if (Button_B == YES)
        CouplerSetNum = NewBand;    // set to commanded band.  If a Button B remote cmd, NewBand wil lbe incremented before here
        Button_B = NO; //reset flag
        Cal_Table();
        save_config_EEPROM();
    }
    if (Button_C == YES) {
        Button_C = NO; //reset flag
        if (op_mode != SWR)  {
        op_mode = SWR;
        save_config_EEPROM();
        }
    } 

    if (op_mode != MENU && updateTime <= millis()) {
        updateTime = millis() + 45; // Update timer every 35 milliseconds
    
        // Create a Sine wave for testing the AD
        //d += 1; if (d >= 360) d = 0;
        //FwdPwr = 50 + 50 * sin((d + 0) * 0.0174532925);
        
        adRead(); //get and calculate power + SWR values and display them
    }
}

/*
 * While in Edit mode, this function makes the actual field change and stores whole structure in EEPROM at end
*/

// read cal data from EEPROM (state is another function)
void read_Cal_Table_from_EEPROM()
{
   int i;
   int len_ee;
   
   for (i=0; i<10; i++) {
      if (i < EEPROM.length())
      {
         len_ee = EEADDR+(sizeof(Band_Cal_Table)*i);
         EEPROM.get(len_ee, Band_Cal_Table[i]);         
         //Serial.println("EEPROM Read");
         //Serial.println(Band_Cal_Table[i].BandName);
         //Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         //Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(10); 
      }
   }
}

// Copy cal data in memory to EEPROM to preserve user changes (save state is another function)
void write_Cal_Table_to_EEPROM()
{
   int i;
   int len_ee;
    
   for (i=0; i< 10;i++) {
      if (i < EEPROM.length()) {
         len_ee = EEADDR+(sizeof(Band_Cal_Table)*i);
         EEPROM.put(len_ee, Band_Cal_Table[i]);       
         //Serial.println("EEPROM Written");
         //Serial.println(Band_Cal_Table[i].BandName);
         //Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         //Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(10);
         EEPROM.update(0,'G');
      }
   }
   //EEPROM.commit();
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
    Serial.println("Copied default data info Band Cal Table");
  }
}

// Read state in EEPROM
void get_config_EEPROM()
{
    CouplerSetNum = EEPROM.read(2*2)-'0';
    //Serial.print("CouplerSetNum Read = ");
    //Serial.println(CouplerSetNum);
    op_mode = EEPROM.read(3*2)-'0';
    //Serial.print("op_mode Read = ");
    //Serial.println(op_mode);
    //EEPROM.commit();
}

// Save state in EEPROM
void save_config_EEPROM()
{
    char buf[3];
    CouplerSetNum = constrain(CouplerSetNum,0,NUM_SETS);
    EEPROM.update(2*2, (CouplerSetNum+'0'));
    //Serial.print("CouplerSetNum Write =");
    //Serial.println(CouplerSetNum);
    op_mode = constrain(op_mode,1,3);
    if (op_mode == 1 || op_mode == 2) {
        EEPROM.update(3*2, byte(op_mode)+'0');
        //Serial.print("op_mode Write = ");
        //Serial.println(op_mode);
    }
    //EEPROM.commit();
}

// Mark EEPROM for overwrite
void reset_EEPROM()
{
    if (Reset_Flag ==1) {
        // erase byte 0 to force init process for testing.  Can also use to "factory" reset data
        EEPROM.update(0, '0');
        //EEPROM.commit();
        Serial.println("Erased Byte 0");
    }
}

// Toggle USB serial output data
void toggle_ser_data_output()
{
     if (EEPROM.read(4*2) != 'Y' || ser_data_out == 1){
        EEPROM.update(4*2, 'Y');
        //EEPROM.commit();
        Serial.println("Enabled Serial Data Output");
        ser_data_out = 0;
     }
     else { 
        EEPROM.update(4*2, 'N');
        //EEPROM.commit();
        Serial.println("Disabled Serial Data Output");
     }
}

void Cal_Table()
{
    //
    // Each coupler has unique coupling factor. 
    // The more frequency bands in the table the better but at more effort to code and to enter cal data.
    // The AD8318 detector is fairly linear over frequency, temperature, and power betwen 0 and -60dBm so goal
    // is to land the attenuated coupler port output levels below 1mW at full power. 
    
    // Copy Selected Band Cal Data from Table to working variables  
    strcpy(Coupler_friendly_name, Band_Cal_Table[CouplerSetNum].BandName);
    CouplingFactor_Fwd = Band_Cal_Table[CouplerSetNum].Cpl_Fwd;  // value in dB from coupler specs.  
        // Program should account for this by muliplying measured value by this amount.  
        // For example, Fwd_dBm + 30.  If dBm measures at -22dBm, coupling factor is 30, then the actual value
        // at the coupler input port is x1000 higher, so +8dBm  (30-22=+8) or nearly 10mW.  
        // 50W input to coupler would show 1000 50mW or +17dBm
    CouplingFactor_Ref = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
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
