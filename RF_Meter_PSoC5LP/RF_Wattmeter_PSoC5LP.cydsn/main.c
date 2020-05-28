/* ========================================
 *
 * Copyright K7MDL, 2020
 * All Rights Reserved
 * Distribution allowed per MIT License
 *
 * ========================================
*/
/*  TODO:
    Vendor ID – This is a 16-bit number used to uniquely identify USB devices belonging to a 
    specific vendor/manufacturer to a USB host. Vendor IDs are assigned by the USB Implementers 
    Forum (USB-IF). The following link in the USB-IF webpage explains the method to obtain a
    vendor ID for your company:      
    http://www.usb.org/developers/vendor/ 
    Note Vendor ID 0x4B4 is a Cypress-only VID and may be used for development purposes only. 
    Products cannot be released using this VID; you must obtain your own VID. 
*/
    
/*
 *
 * RF Watmeter_PSoc5LP by K7MDL 5/27/2020   - Remote (Headless) Edition for PSoc5LP 
 *
 * 5/27/2020 -  Headless port from Arduino Nano to PSoC5LP CY8CKIT-059 dev module now fully working.
 *
 * 5/10/2020 -  Stripped out physical button tests (kept code as virtual buttons).  
 *              Removed rest of M5/ESP32 dependencies,
 *              Using standard Arduino functions (vs original M5Stack/esp32).  
 *              Added CPU reset and default EEPROM commands. 
 *              Trying out read_vcc() to measure VCC folowing each AD read to increase accuracy since
 *                  the Default is bus power (from USB or external).  Internal Vref is too low at 1.1V to use
 *                  Without adding an external voltage divider.
 *
 * 5/8/2020 - Expanded remote commands to support dumping the cal table and writing to individual coupling
 *    factor cal values and saving to EEPROM.  Switch from a one byte command to a string with similar structure
 *    as the power level out.  Changed sequence number to msg_type fo future expansion and to help validate the 
 *    incoming message better from random data/noise/CPU status messages.
 *    There is still some screen drawing mostly in the cal area yet to be removed.  Considering leaving the digital
 *    values screen writing in for adaption to a 2x16/4x20 LCD or small graphics OLED embedded in an RFampifier as
 *    the main meter with SWR shutdowna nd other feature (including remote monitoring).
 *    One concern for getting Wi-Fi to work is the possibility the ESP32 in the M5stack I am using may take over ADC2 pins 
 *    which would be a problem.  Also the internal noise coudl be improved using an external I2C connected A/D.
 *
 * 5/7/2020 -  Builds on RF Power Meter dated 5/7.  
 *    Begin stripping the Display and M5 specific components to allow for 
 *      standard Arduino and headless operation
 *
 Has user edited Calibration sets. Have 10 "bands" for frequency correction used with values that can be edited via the UI ---

 In this code example I use a RLC .05-1000MHz coupler and created 10 cal bands toi cover 50M to 10G.
 A value for each dual directional coupler port combines the coupler facxtor, added attenuators, and any cal correction fudge factor.
 The values are edited via the device UI and stored in EEPROM.
*/
#include <project.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow usage of the floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
 //asm (".global _printf_float");
#endif

/*
  User edited values here for Callsign and Meter ID number
*/
#define CALLSIGN_LEN            (7u)
char    Callsign[CALLSIGN_LEN] = "K7MDL\0";
#define METERID 102 // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station

// Rest of program...
#define METER_RATE 2   // used to skip serial data output to a lower rate
#define TEST_EEPROM 0  // change to 1 and reprogram, then set back to 0 to reset EEPROM data to default
#define SWR 2
#define PWR 1
#define MENU 3
#define NO 0
#define YES 1
#define ADC_COUNTS 1024    // 4096 for ESP32 12bit, 1024 for 10 bit ESP32 and Nano.
#define ad_Fwd "A1"    // Analog 35 pin for channel 0
#define ad_Ref "A2"   // Analog 36 pin for channel 1
// Edit the Coupler Set data inb teh Cal_Table function.  Set the max number of sets here, and the default to load at startup
#define NUM_SETS 10 // 10 bands, 0 through 9 for example
#define EE_STATE_BASE_ADDR      (0x0000)   /* row 0 */
#define EE_RESERVED_BYTE1       (0x0001)
#define OP_MODE_OFFSET          (0x0002)
#define COUPLERSETNUM_OFFSET    (0x0003)
#define SER_DATA_OUT_OFFSET     (0x0004)
#define CALLSIGN_OFFSET         (0x0010)  /* first 7 bytes of row 1*/
#define CAL_TBL_ARR_OFFSET      (0x0020)  /* Row 2  */
#define EE_SAVE_YES             (65u)
#define TEST_LEN                (14u)
#define CFG_ARR_TEST_STRING     "01234567890123"
#define Serial_USB_DEVICE       (0u)
/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define Serial_USB_BUFFER_SIZE  (64u)
#define LINE_STR_LENGTH         (20u)
#define BUF_LEN  Serial_USB_BUFFER_SIZE

char8* parity[] = {"None", "Odd", "Even", "Mark", "Space"};
char8* stop[]   = {"1", "1.5", "2"};
float32 Vref = 5.0;        // 3.3VDC for Nano and ESP32 (M5stack uses ESP32)  ESP32 also has calibrated Vref curve
int CouplerSetNum = 0;   // 0 is the default set on power up.  
int ser_data_out = 0;
int Reset_Flag = 0;
uint32_t updateTime = 0;       // time for next update
float32 Fwd_dBm = 0;
float32 Ref_dBm = 0;
float32 FwdPwr = 0;
float32 RefPwr = 0;
float32 SWRVal = 0;
float32 SWR_Serial_Val = 0;
float32 FwdVal = 0;
float32 RefVal = 0;
//int d = 0;
int Edit_Atten = 0;
int op_mode = SWR;
int counter1 = 0;
int Button_A = 0;
int Button_B = 0;
int Button_C = 0;
int NewBand;
int Ser_Data_Rate = METER_RATE;
float32 CouplingFactor_Fwd = 0;
float32 CouplingFactor_Ref = 0;
float32 Offset = 0.500;  // AD8318 is 0.5 offset.  0.5 to about 2.1 volts for range.
float32 Slope = 0.025;  // AD8318 is 25mV per dB with temp compensation
float32 FwdPwr_last = 0;
int Inverted = 1;  // 0=no, 1=Yes.  Inverted output will have max V = no input, min volt at max power input.
/* 
 *  AD8318 is an inverted with about 2.5V for no inoput and 0.5 for max input cover -65 to +5dBm range
 *  linear between -55 and 0dBm
*/ 
#define NUMREADINGS 2   // adjust this for longer or shorter smooting period as AD noise requires
float32 readings_Fwd[NUMREADINGS];      // the readings from the analog input
float32 readings_Ref[NUMREADINGS];      // the readings from the analog input
int readIndex_Fwd = 0;              // the index of the current reading
int readIndex_Ref = 0;              // the index of the current reading
float32 total_Fwd = 0;                  // the running total
float32 total_Ref = 0;                  // the running total
uint8   EE_PGM_Status = 0u;
uint8   EE_PGM_Failed;
uint8 rx_buffer[Serial_USB_BUFFER_SIZE];
uint8 tx_buffer[Serial_USB_BUFFER_SIZE];
uint16 rx_count = 0;
uint16 tx_count = 0;
static char sdata[Serial_USB_BUFFER_SIZE], *pSdata = sdata, *pSdata1=sdata, *pSdata2=sdata;

// Function declarations
void toggle_ser_data_output();
void setup(void);
void adRead(void);
void read_Cal_Table_from_EEPROM();
void sendSerialData();
void print_cal_table();
void Cal_Table();
void reset_EEPROM();
void write_Cal_Table_to_EEPROM();
void save_config_EEPROM();
void write_Cal_Table_from_Default();
void get_remote_cmd();
uint16 serial_usb_read(void);
void serial_usb_write();
uint8 EEPROM_Init(uint8); /* function to initially copy the config table array to EEPROM on first use.  Calls Read or Write */
uint8 EEPROM_Init_Write(void); /* function to initially copy the config table array TO the EEPROM on first use */
uint8 EEPROM_Init_Read(void); /* function to initially copy the config table array FROM the EEPROM on first use */
uint8 Copy_to_EE(uint8 e_row, uint8_t c_array[16],uint8 print);  /*  called by EEPROM_Init_Write */
uint8_t EE_Save_State(void);

struct Band_Cal {
  char  BandName[12];
  float32 Cpl_Fwd;
  float32 Cpl_Ref;
} Band_Cal_Table_Def[NUM_SETS] = {
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

void setup(void) {

    Serial_Start();
    /* Start USBFS operation with 5-V operation. */
    Serial_USB_Start(Serial_USB_DEVICE, Serial_USB_5V_OPERATION);  
    write_Cal_Table_from_Default();  // Copy default values into memory in case EEPROM not yet initialized
    ADC_RF_Power_Start();
    ADC_RF_Power_AMux_Start();
    
    EEPROM_Start(); /*  Start the EEPROM storage System             */
    CyDelay(7);     /* wait at least 5ms to power up EEPROM       */
   
    /*   initialize EEPROM storage if not done before - will overwrite default memory table values from EEPROM if EEPROM was written to before */    
    EE_PGM_Failed = EEPROM_Init(0);

    Cal_Table();   // Load current Band values from Table

    // initialize all the readings to 0:  Used to smooth AD values
        for (int thisReading = 0; thisReading < NUMREADINGS; thisReading++) {
        readings_Fwd[thisReading] = 0;
        readings_Ref[thisReading] = 0;
    }
    CyDelay(1000);
}

int main(void)
{   
    CyGlobalIntEnable; /* Enable global interrupts. */

    setup();

    while (1) {
        // Listen for remote computer commands
        serial_usb_read();  // festch latest data in buffer
              
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
            ++CouplerSetNum;   // increment for manual button pushes
            if (CouplerSetNum > NUM_SETS) 
                CouplerSetNum = 0;
            if (Button_B == YES)
                CouplerSetNum = NewBand;    // set to commanded band.  If a Button B remote cmd, NewBand wil lbe incremented before here
            Button_B = NO; //reset flag
            Cal_Table();
            //save_config_EEPROM();           
            EE_Save_State();
        }
        if (Button_C == YES) {
            Button_C = NO; //reset flag
            if (op_mode != SWR)  {
                op_mode = SWR;
                //save_config_EEPROM();
                EE_Save_State();
            }
        } 
        adRead(); //get and calculate power + SWR values and display them
        CyDelay(200);
    }
}

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

uint16 serial_usb_read(void)
{
    /* Host can send double SET_INTERFACE request. */
    if (0u != Serial_USB_IsConfigurationChanged())
    {
        /* Initialize IN endpoints when device is configured. */
        if (0u != Serial_USB_GetConfiguration())
        {
            /* Enumeration is done, enable OUT endpoint to receive data 
             * from host. */
            Serial_USB_CDC_Init();
        }
    }
    
    rx_count = 0;    
    /* Service USB CDC when device is configured. */
    if (0u != Serial_USB_GetConfiguration())
    {
        /* Check for input data from host. */
        if (0u != Serial_USB_DataIsReady())
        {
            /* Read received data and re-enable OUT endpoint. */        
            rx_count = Serial_USB_GetAll(rx_buffer);
        }   
        // initially p1 = p2.  parser will move p1 up to p2 and when they are equal, buffer is empty, parser will reset p1 and p2 back to start of sData
        if (rx_count != 0){
            memcpy(pSdata2 , rx_buffer, rx_count);   // append the new buffer data to current end marked by pointer 2
            pSdata2 += (rx_count);   // Update the end pointer position. The function processing chars wil lupdate teh p1 and p2 pointers
            //*(pSdata2) = '\0';
            rx_count = pSdata2 - pSdata1;  // update count for total unread chars.
        }
    }
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

    if (0u != tx_count)
    {
        /* Wait until component is ready to send data to host. */
        while (0u == Serial_USB_CDCIsReady())
        {
        }

        /* Send data back to host. */
        Serial_USB_PutData(tx_buffer, tx_count);

        /* If the last sent packet is exactly the maximum packet 
        *  size, it is followed by a zero-length packet to assure
        *  that the end of the segment is properly identified by 
        *  the terminal.
        */
        if (Serial_USB_BUFFER_SIZE == tx_count)
        {
            /* Wait until component is ready to send data to PC. */
            while (0u == Serial_USB_CDCIsReady())
            {
            }

            /* Send zero-length packet to PC. */
            Serial_USB_PutData(NULL, 0u);
        }
    }
}

void adRead(void)   // A/D converter read function.  Normalize the AD output to 100%.
{
    float32 a;
    float32 a1;
    float32 b;
    int c;
    //char buf[12];
    float32 tmp;
    int16 ad_counts;

    // subtract the last reading:
    total_Fwd -= readings_Fwd[readIndex_Fwd];
    // read from the sensor:
    c = 2;   // short term smaples that feed into running average
    a = 0;
    for (int i = 0; i < c; ++i) {                   
            // ADC_RF_Power_SetGain(1);  // can be used to tweak in the ADC
            AMux_RF_Power_FastSelect(0);
            CyDelayUs(5);
            ad_counts = ADC_RF_Power_Read32();
            a1 = ADC_RF_Power_CountsTo_Volts(ad_counts);    
            a += a1;
            CyDelay(30);
    }
    a /= c; // calculate the average then use result in a running average
    readings_Fwd[readIndex_Fwd] = a;   // get from the latest average above and track in this runnign average
    // add the reading to the total:
    total_Fwd += readings_Fwd[readIndex_Fwd];
    // advance to the next position in the array:
    readIndex_Fwd += 1;   
    // if we're at the end of the array...
    if (readIndex_Fwd >= NUMREADINGS) {
        // ...wrap around to the beginning:
        readIndex_Fwd = 0;
    }     
    // calculate the average:
    b = total_Fwd / NUMREADINGS;

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
    //printf(buf);
    
    // Now get Reflected Power 
    // subtract the last reading:
    total_Ref -= readings_Ref[readIndex_Ref];
    // read from the sensor:
    c = 2;
    a = 0;
    for (int i = 0; i < c; ++i)  {
            // ADC_RF_Power_SetGain(1);  // can be used to tweak in the ADC
            AMux_RF_Power_FastSelect(1);
            CyDelayUs(5);
            ad_counts = ADC_RF_Power_Read32();
            a1 = ADC_RF_Power_CountsTo_Volts(ad_counts);
            a += a1;
            CyDelay(30);
    }
    a /= c; // calculate the average then use result in a running average
    readings_Ref[readIndex_Ref] = a;   // get from the latest average above and track in this runnign average
    // add the reading to the total:
    total_Ref += readings_Ref[readIndex_Ref];
    // advance to the next position in the array:
    readIndex_Ref += 1;   
    // if we're at the end of the array...
    if (readIndex_Ref >= NUMREADINGS) {
        // ...wrap around to the beginning:
        readIndex_Ref = 0;
    }  
    // calculate the average:
    b = total_Ref / NUMREADINGS;
    
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
    //printf(buf);

    // dtostrf(RefPwr, 1, 0, buf1);
    // strcpy(buf, "Ref PWR");
    // strcat(buf, buf1);
    // printf(buf);
    
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

    //if ((counter1/10) % 10 == 0) {   // slow down the data rate.  Ideally do at at the AD read command                
        sprintf(tx_buffer,"%d,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f\r\n", METERID, "170", Band_Cal_Table[CouplerSetNum].BandName, Fwd_dBm, Ref_dBm, FwdPwr, RefPwr, SWR_Serial_Val);
        tx_count = strlen(tx_buffer);
        serial_usb_write();
    //}    
}

void get_remote_cmd(){

    uint8 cmd1;
    float32 cmd2;
    uint8 cmd_str_len;
    uint8 i = 0; 
    uint8 j = 0; 
    char cmd_str[BUF_LEN] = {};
    
    if (rx_count !=0) {
        pSdata = strchr(pSdata1, '\n');   // find string terminator position
        if (pSdata) { 
            *pSdata = '\0';
            cmd_str_len = pSdata - pSdata1;
            strncpy(cmd_str, pSdata1, cmd_str_len);   // copy chars between p1 and the terminator
            pSdata1 += (cmd_str_len+1);  // reset ch pointer back to stat of string for char parsing
             
            if (pSdata1 >= pSdata2 || cmd_str_len > BUF_LEN)  // if we have caught up to the end p2 then reset to beginning of buffer position.
                pSdata1 = pSdata2 = sdata; 
        }
        printf("%s", cmd_str);
    }
    if (strlen(sdata) >= BUF_LEN-1) {
        //pSdata = sdata;
        sdata[0] = '\0';
        printf("BUFFER OVERRRUN\n");
        //Serial_ClearRxBuffer();            
    }
    else {
        // filter out unwanted characters             
        // Now we have a full comma delimited command string - validate and break it down           
        j = 0;
        for (i=0; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
            if (isalnum(sdata[i]))
                cmd_str[j++] = (sdata[i]);                 
        }
        cmd_str[j] = '\0';  
        printf(" Meter ID  ");
        printf("%s",cmd_str);

        if (atoi(cmd_str) == METERID) {
            if (i < cmd_str_len) {
                j = 0;
                // Look for Msg_Type now
                for (i +=1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {   // i+1 to skip over the comma the var 'i' was left pointing at
                    cmd_str[j++] = sdata[i];                 
                }
                cmd_str[j] = '\0';
                printf(" Msg Type:  ");
                printf("%s",cmd_str);
              
                if (atoi(cmd_str) == 120)  {   // Vaidate this message type for commands.  120 = coammand, 170 is data output                              
                    j = 0;
                    if (i < cmd_str_len) {                               
                        for (i += 1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                            cmd_str[j++] = sdata[i];
                        }  
                        cmd_str[j] = '\0';
                        printf(" CMD1:  ");
                        printf("%s",cmd_str);
                        cmd1 = atoi(cmd_str);
                    }
                    else 
                        cmd1 = 1;
                    
                    j = 0;
                    if (i < cmd_str_len) {                                                
                        for (i += 1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                            cmd_str[j++] = sdata[i];                          
                        }
                        cmd_str[j] = '\0';
                        printf(" CMD2:  ");
                        printf("%s",cmd_str);
                        cmd2 = atof(cmd_str);
                    }
                    else 
                        cmd2 = 1.0;
              
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
                    if (cmd1 == 252) NULL;  //Speed up or slow down the Serial line output info rate
                    if (cmd1 == 251) NULL;  //Speed up or slow down the Serial line output info rate
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
                       // resetFunc();  //call reset
                       // Probably use a sw wd to do a software remote reset.
                    }                            
                    // Handle remote command to change stored coupling factor to support headless ops.
                    // TODO: Need to write into EEPROM, either here or by changing bands.                          
                    if (cmd1 >= 100 && cmd1 < 110) {     // Change Fwd coupling factor for Port X
                        int index;
                        index = cmd1 - 100;
                        printf("Band: ");
                        printf("%s",Band_Cal_Table[index].BandName);
                        printf(" --- Old Fwd Value: ");
                        printf("%f",Band_Cal_Table[index].Cpl_Fwd);
                        Band_Cal_Table[index].Cpl_Fwd = cmd2;       // cmd2 is second byte in 2 byte payload.                              
                        printf(" +++ New Fwd Value: ");
                        printf("%f",Band_Cal_Table[index].Cpl_Fwd);
                        EEPROM_Init_Write();  // save to eeprom
                    }
                    if (cmd1 >= 110 && cmd1 < 120) {     // Change Ref coupling factor for Port X
                        int index;
                        index = cmd1 - 110;
                        printf("Band: ");
                        printf("%s",Band_Cal_Table[index].BandName);
                        printf(" --- Old Ref Value: ");
                        printf("%f",Band_Cal_Table[index].Cpl_Ref);
                        Band_Cal_Table[index].Cpl_Ref = cmd2;       // cmd2 is second byte in 2 byte payload.                              
                        printf(" +++ New Ref Value: ");
                        printf("%f",Band_Cal_Table[index].Cpl_Ref);
                        EEPROM_Init_Write();   // save to eeprom on changes.  
                    }                                      
                } // end of msg_type 120                                      
            } // end of msg_type length check
        } // end of meter ID OK    
    } //while serial available    
} // end get_remote_cmd function

// Copy hard coded cal data to memory
void write_Cal_Table_from_Default()
{
  uint i;
  
  for (i=0;i<NUM_SETS;i++) {
    // Populate initial database scructure in RAM.  Ultimately it needs to be read from EEPROM once initialized
    strcpy(Band_Cal_Table[i].BandName, Band_Cal_Table_Def[i].BandName);
    Band_Cal_Table[i].Cpl_Fwd = Band_Cal_Table_Def[i].Cpl_Fwd;
    Band_Cal_Table[i].Cpl_Ref = Band_Cal_Table_Def[i].Cpl_Ref;
    printf("Copied default data info Band Cal Table");
  }
}

// Read state in EEPROM
void get_config_EEPROM()
{
    CouplerSetNum = EEPROM_ReadByte(EE_STATE_BASE_ADDR+COUPLERSETNUM_OFFSET )-'0';
    //printf("CouplerSetNum Read = ");
    //printf(CouplerSetNum);
    op_mode = EEPROM_ReadByte(EE_STATE_BASE_ADDR+OP_MODE_OFFSET )-'0';
    //printf("op_mode Read = ");
    //printf(op_mode);
}

// Save state in EEPROM
void save_config_EEPROM()
{
    //CouplerSetNum = constrain(CouplerSetNum,0,NUM_SETS);
    EEPROM_WriteByte(EE_STATE_BASE_ADDR+COUPLERSETNUM_OFFSET, (CouplerSetNum+'0'));
    //printf("CouplerSetNum Write =");
    //printf(CouplerSetNum);
    //op_mode = constrain(op_mode,1,3);
    if (op_mode == 1 || op_mode == 2) {
        EEPROM_WriteByte(EE_STATE_BASE_ADDR+OP_MODE_OFFSET, (op_mode)+'0');
        //printf("op_mode Write = ");
        //printf(op_mode);
    }
}

// Mark EEPROM for overwrite
void reset_EEPROM()
{
    if (Reset_Flag ==1) {
        // erase byte 0 to force init process for testing.  Can also use to "factory" reset data
        //EEPROM_Init(EE_SAVE_YES);
        EEPROM_WriteByte(0,EE_STATE_BASE_ADDR);
        printf("Erased Byte 0");

    }
}

// Toggle USB serial output data
void toggle_ser_data_output()
{
     if (EEPROM_ReadByte(EE_STATE_BASE_ADDR+SER_DATA_OUT_OFFSET) != 'Y' || ser_data_out == 1){
        EEPROM_WriteByte(EE_STATE_BASE_ADDR+SER_DATA_OUT_OFFSET, 'Y');
        //printf("Enabled Serial Data Output");
        ser_data_out = 0;
     }
     else { 
        EEPROM_WriteByte(EE_STATE_BASE_ADDR+SER_DATA_OUT_OFFSET , 'N');
        //printf("Disabled Serial Data Output");
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
    //strcpy(Coupler_friendly_name, Band_Cal_Table[CouplerSetNum].BandName);
    CouplingFactor_Fwd = Band_Cal_Table[CouplerSetNum].Cpl_Fwd;  // value in dB from coupler specs.  
        // Program should account for this by muliplying measured value by this amount.  
        // For example, Fwd_dBm + 30.  If dBm measures at -22dBm, coupling factor is 30, then the actual value
        // at the coupler input port is x1000 higher, so +8dBm  (30-22=+8) or nearly 10mW.  
        // 50W input to coupler would show 1000 50mW or +17dBm
    CouplingFactor_Ref = Band_Cal_Table[CouplerSetNum].Cpl_Ref;
}

void print_cal_table()
{
    uint8   i;
    uint8   j;
    char    buf[80];

    // example ouput format : "101,150,TEXT,55.4,35.4,3.3,2.2"
    // #150 for msg_type field to signal this is data dump, not power levels or other type messages.
    for (i=0; i < NUM_SETS; i++) {
        sprintf(buf, "%d,%s,%s,%.2f,%.2f\r\n",    // meterid with msg_type = 150 to signal different data set than the normal output. 120 inbound cmd, 170, power out
        METERID,
        "150",  //msg_tyope for cmd reply
        Band_Cal_Table[i].BandName,
        Band_Cal_Table[i].Cpl_Fwd,  // value in dB from coupler specs.  
        Band_Cal_Table[i].Cpl_Ref
        );
        
        tx_count = strlen(buf);
        for (j = 0; j< tx_count; j++)
            tx_buffer[j] = buf[j];
        tx_buffer[j+1] = '\0';
        serial_usb_write();   // Output table text to serial port              
    }
}

/*******************************************************************************
* Function Name: EEPROM_Init
********************************************************************************
* Summary:
*   Called for Initialization, Check Byte 0 != G then copy into EEPROM space the needed variables
*   and the Band_Cal_Table structure. There can be other data in the rest of EEPROM.
*   Variables wil be reset to default after an overwrite.  A master reset menu option may be created to do this via the menu system
*
*  
*    Test Address 0.
*    Values for Address 0 (EEPROM_Cfg_Status)
*        0x00   Uninitialized or Erased
*        'S'  Initialization Started
*        'G'  Initialization Completed
*   
* Parameters:
*   None
*
* Return:
*   0 for success, 1 for FAIL
*
*******************************************************************************/

uint8 EEPROM_Init(uint8 ee_save)
{
    /*  If we find a 'G' character at the start of EEPROM then skip initialization to prevent user data overwrite */   
    EE_PGM_Status = EEPROM_ReadByte(EE_STATE_BASE_ADDR);    
    if(((EE_PGM_Failed == NO) && EE_PGM_Status != 'G') || ee_save == EE_SAVE_YES)   /*   EEPROM_Init will set byte 0, 'S' or on success 'G' */
    {  /* start writing or overwriting 
        LCD_Position(TOP,0u);
        LCD_PrintString(" EE Write START ");      
        LCD_Position(BOTTOM,0u);
        LCD_PrintString("                ");      
        */
        CyDelay(1000u);  
        if(EEPROM_Init_Write() != 0)    /*  call EEPROM init function and check for success = 0       */
        {   /* 1st Failure. Update display with code and Retry 1 time                                     
            LCD_Position(TOP,0u);
            LCD_PrintString(" EE Write FAIL! ");              
            LCD_Position(BOTTOM,0u);
            LCD_PrintString("#1 Fail Code:   ");
            LCD_Position(BOTTOM,13u); 
            */
            EE_PGM_Status = EEPROM_ReadByte(EE_STATE_BASE_ADDR);
            if(EE_PGM_Status <48 || EE_PGM_Status != 'G')  /* filter out G to prevent false code, convert any non printable chars */                       
            {   
                //LCD_PutChar(EE_PGM_Status+48);   /*  Should be S for started or 0x00 */
            }
            else 
            {   
                //LCD_PutChar(EE_PGM_Status);   /*  Should be S for started or 0x00 */
            }          
            CyDelay(5000u);   /*  give time to see the screen eror  */
            //LCD_Position(BOTTOM,0u);
            //LCD_PrintString(" Retrying 1 time");
            CyDelay(1000u);   
            if(EEPROM_Init_Write() != 0) /* Now try 1 more time  */
            {   /*  Retry failed also   */
                /*LCD_Position(BOTTOM,0u);
                LCD_PrintString("#2 Fail Code:   ");
                LCD_Position(BOTTOM,13u); */
                EE_PGM_Status = EEPROM_ReadByte(EE_STATE_BASE_ADDR);
                if(EE_PGM_Status <48 || EE_PGM_Status != 'G')  /* filter out G to prevent false code, convert any non printable chars */                       
                {   
                    // LCD_PutChar(EE_PGM_Status+48);   /*  Should be S for started or 0x00 */
                }
                else 
                {   
                    // LCD_PutChar(EE_PGM_Status);   /*  Should be S for started or 0x00 */
                }
                CyDelay(10000u);    
                EE_PGM_Failed = YES;    /* Set a flag to not retry until program is reset.  Continue with defaults */
                return 1;
            }
        }
        #ifdef DEBUG_EEPROM
        LCD_Position(TOP,0u);
        LCD_PrintString(" EE Write Done! ");
        LCD_Position(BOTTOM,0u);
        LCD_PrintString(" Success! C:   ");
        LCD_Position(BOTTOM,12u);
        EE_PGM_Status = EEPROM_ReadByte(EE_STATE_BASE_ADDR);               
        LCD_PutChar(EE_PGM_Status);   /*  Should be 'G' */
        CyDelay(3000u);   
        #endif 
        EE_PGM_Failed = NO;
        return 0;
    }
    if(!EEPROM_Init_Read())
        return 0;   /*   Already has good EEPROM config data or has write attempts have failed 2 times before since reset so skip out  */  
    else return 1;  /*  FAIL   */
}
/*******************************************************************************
* Function Name: EEPROM_Init_Write
********************************************************************************
* Summary:
*   Copy into EEPROM space the needed variables and the Band_Cal_Table data structure.  
*   There can be other data in the rest of EEPROM and this will overwrite it.
*   Variables wil be reset to default after an overwrite.  A master reset menu option may be created to do this
*   Called by EEPROM_Init. After writing is complete check Byte 0 == 'G'.
*
* Parameters:
*   None
*
* Return:
*   0 for success, 1 for FAIL
*
*******************************************************************************/
uint8 EEPROM_Init_Write(void)
{
    uint8       ee_row;
    uint8_t     c1_array[16]; /* stores 1 row of 16 bytes for eeprom write and reads */
    uint16      j;  
    uint8_t     eepromArray[(NUM_SETS*20)];  // 200 is the array size calculated by the sizeOf function below
    uint16      Arr_Size;
    
    
    EEPROM_UpdateTemperature();   
    /*  write 'S' to byte 0 to indicate we started writing bytes but not finished    */
    if(!EEPROM_ByteWritePos('S', 0, 0))
    {
        /*  Start writing all of our data to EEPROM now.  
            Everything is 1 byte in size so can serially copy each into EEPROM with no size or type conversion required
            Have 7 structs and some variables to do at the end.  Lets start with the Band_Cal_Table structure
        */
        
        /* Begin with our state variables  in row 1, starting with the 3rd byte.  0 == corrupt flag, 1 = reserved, 3 is first data. 
        *   2 rows in at byte 33 (32 0-31) will start writing rows of Array data.  It is at the end to avoid having to recomputer where the saved data is relative to the start of EEPROM.
        */          
        c1_array[0] = 'S';   /* what we just wrote, will overwrite as a block of 16 bytes) */
        c1_array[EE_RESERVED_BYTE1] = _NULL;
        c1_array[OP_MODE_OFFSET] = op_mode;  
        c1_array[COUPLERSETNUM_OFFSET] = CouplerSetNum;
        c1_array[SER_DATA_OUT_OFFSET] = ser_data_out;              
        
        /*   Insert additional new EEPROM variables here.  Have a few more bytes left.   */    
        c1_array[0x5] = '\0';
        c1_array[0x6] = '\0';
        c1_array[0x7] = '\0';
        c1_array[0x8] = '\0';
        c1_array[0x9] = '\0';        
        c1_array[0xA] = '\0';
        c1_array[0xB] = '\0';
        c1_array[0xC] = '\0';
        c1_array[0xD] = '\0';
        c1_array[0xE] = '\0';        
        c1_array[0xF] = '\0';

        ee_row = 0;
        Copy_to_EE(ee_row,c1_array,NO);     /* copy 1 row at a time to EEPROM */               
        /*  next row   */

        memcpy(c1_array,Callsign, CALLSIGN_LEN);   /*  7 bytes */      
        // append any oher strings like this example:
        //memcpy(&c1_array[CALLSIGN_LEN],Grid_Square, GRIDSQUARE_LEN); /* 9 bytes */
        
        ee_row = 1;
        Copy_to_EE(ee_row,c1_array,NO);     /* copy 1 row at a time to EEPROM */
        
        Arr_Size = (sizeof Band_Cal_Table);
        memcpy(eepromArray,Band_Cal_Table,Arr_Size);       /*  copy ARR_SIZE bytes back into the Band_Cal_Table structure (if it is not padded)  */                                 
      
        /*   start at 3rd row 1st byte (row 2) 0x0020)   */
        for(j=0; j<(Arr_Size); j++)  /* if we have t bands then we have 14 rows of EEPROM to read - or BANDS = 7 *32bytes, start at byte address  0x0010  */
        {              
             EEPROM_WriteByte(eepromArray[j], CAL_TBL_ARR_OFFSET+j);      /*  start at byte 16 and get ARR_SIZE bytes for whole structure */          
        }            
       
        /*If all went OK set first byte to 'G' and return */
        if(EEPROM_ByteWritePos('G', 0, 0) == 0)
        {
            return 0;   // all good, exit
        }
        else
        {
        /*  return fail (1) and leave the S in place */
            return 1;  /* write to EEPROM completed OK but the writing the marker byte failed  */      
        }
        
    }
    else
    {
        return 1;   /* failed to write the 'S' so abandon */
    }
    
}

/*******************************************************************************
* Function Name: Copy_to_EE
********************************************************************************
* Summary:
*   Takes data in an array of 16 bytes and writes it to the row requested by calling functions
*   EEPROM starts at 16 bit zero for these API functions.  We are reserving row 1 for flags to protect overwrite at startup.
    Writing in rows for now.  Callers have to gather up their data into a 16 byte array and pass that in along with the row.

* Parameters:
*   e_row  index to EEPROM row
*
* Return:
*   0 for success, 1 for FAIL
*
*******************************************************************************/
uint8 Copy_to_EE(uint8 e_row, uint8_t c_array[16], uint8 print)
{
    uint16      j;
    uint8_t     eepromArray[16];   /* copy of data from eeprom to compare with source for validation.  */
    
    /*  Use e_row to specify the data row */
    
    /*  print the result for validation */
    if(print)
    {
        //LCD_Display_Clear();
        
        //sniprintf(LCD_Display_Row1, 17, "%s     ","DataWrite Row:  ");
        //LCD_Display_Update(TOP);
        //LCD_Position(TOP,15u);
        //LCD_PutChar(e_row+64);
        //  
        //sniprintf(LCD_Display_Row2, 17, "%s     ", c_array);
       // LCD_Display_Update(BOTTOM);       
        CyDelay(300);
    }
    if(!EEPROM_Write(c_array,e_row))
    {                      
        if(print)
        {        
            //LCD_Position(BOTTOM,14u);
            //LCD_PrintString("OK");
            CyDelay(300u);  
        }
    }
    else
    {
        if(print)
        {
            /*   error visits here.  */
            //LCD_Position(BOTTOM,9u);
            //LCD_PrintString(" FAIL! ");
            CyDelay(2000u);  
        }
        return 1;
    };            

    /*  Clear bottom status line for next event */
    if(print)
    {
        //LCD_Display_Clear();
        CyDelay(100);    

        /*sniprintf(LCD_Display_Row1, 17, "%s     ","DataWrite Row:  ");
        LCD_Display_Update(TOP);
        LCD_Position(TOP,15u);
        LCD_PutChar(e_row+64);
        LCD_Position(TOP,15u);
        LCD_PutChar(e_row+64); */
    }
    /*  now read it back from EEPROM  and compare to the data written */
    for(j=0; j<16; j++)
    {              
        eepromArray[j] = EEPROM_ReadByte(j+(e_row*16));               
    }
    if(memcmp(eepromArray,c_array,16))
    {
        if(print)
        {
            //LCD_Position(BOTTOM,9u);
            //LCD_PrintString(" FAIL! ");
            CyDelay(2000u);    
        }
        return 1;  
    }
    else    /* verified!  */
    {
        if(print)
        {            
            /* sniprintf(LCD_Display_Row2, 17, "%-9.9s     ", eepromArray);
            LCD_Display_Update(BOTTOM);
            CyDelay(400);
            LCD_Position(BOTTOM,14u);
            LCD_PrintString("OK");  */
            CyDelay(300u);            
        }
    }
        if(print)
        {
            //LCD_Display_Clear();
            CyDelay(800);
        }
    /*  Now read from EEPROM using CYDEV_EE_BASE for absolute address to start of EEPROM
    CYDEV_EEPROM_ROW_SIZE for row size
    CY_DEV_EE_SIZE for size of EEPROM memory space
    
    Look into how EEPROM is mirror for regular read only access and how to map the structure over it.
    Else read it byte at a time or copy the whole thing to a temp structure and proceed normally.  Writes still have to be some byte by byte.
    
    */
    return 0;
}

/*******************************************************************************
* Function Name: EEPROM_Init_Read
********************************************************************************
* Summary:
*   Transfers the master copy of config settings in EEPROM and copies it intop Band_Cal_Table array structure.
*   EEPROM starts at 16 bit zero for these API functions.  We are reserving row1 for flags to protect overwrite at startup.
*
* Parameters:
*   e_row  index to EEPROM row
*
* Return:
*   0 for success, 1 for FAIL
*
*******************************************************************************/
uint8 EEPROM_Init_Read(void)
{
    uint16      j;  
    uint8_t     eepromArray[NUM_SETS*20];
    uint16      Arr_Size;
    
    EE_PGM_Status = EEPROM_ReadByte(EE_STATE_BASE_ADDR);    
    if(EE_PGM_Status != 'G')   /*   EEPROM_Init will set byte 0, 'S' or on success 'G' */
        return 1;
    else
    {
        /* start reading all of our data to EEPROM now.  
            Everything is 1 byte in size so can serially copy each into EEPROM with no size or type conversion required
        */ 
        /* Get 1 row of 16 bytes as we did to write this originally */
        for(j=0; j<32; j++)
        {              
            eepromArray[j] = EEPROM_ReadByte(j+EE_STATE_BASE_ADDR);             /*  address relative to 0x0000  */              
        }                          
        op_mode = eepromArray[EE_STATE_BASE_ADDR+OP_MODE_OFFSET];               /* byte 2 of 16 */
        CouplerSetNum = eepromArray[EE_STATE_BASE_ADDR+COUPLERSETNUM_OFFSET];   /* byte 3 of 16 */
        ser_data_out = eepromArray[EE_STATE_BASE_ADDR+SER_DATA_OUT_OFFSET];     /* byte 4 of 16 */

        /*  Add new variables here as there is space left */         
        
        memcpy(Callsign,&eepromArray[CALLSIGN_OFFSET],CALLSIGN_LEN);   /*  first 7 bytes of row 1 */
        // memcpy(Grid_Square,&eepromArray[GRIDSQUARE_OFFSET],GRIDSQUARE_LEN); /* next 9 bytes from byte #7 in the array  */ 
                
        /*  Using Arr_Size since it (likely) accurately returns the byte size of the CfgTblArr structure */          
        Arr_Size = (sizeof Band_Cal_Table);
        /*   start at 3rd row 1st byte (row 2) 0x0020)   */
        for(j=0; j<(Arr_Size); j++)  /* if we have t bands then we have 14 rows of EEPROM to read - or BANDS = 7 *32bytes, start at byte address  0x0010  */
        {              
            eepromArray[j] = EEPROM_ReadByte(CAL_TBL_ARR_OFFSET+j);      /*  start at byte 16 and get ARR_SIZE bytes for whole structure */          
        }                 
        return 1;
    }
    return 0;   /*  Now have restored user state fully.  Be sure to write updates as they happen */
}

/*******************************************************************************
* Function Name: EE_Save_State
********************************************************************************
* Summary:
*   Saves the state variables required to restore last known state after a power off.
*   Saving all state in one function call since there are only a few and time is not critical for us 
*
* Parameters:
*   None
*
* Return:
*   0 = Success
*   1 = FAIL
*
*******************************************************************************/    
uint8_t EE_Save_State(void) 
{
    uint8   i;
    
    /* save state to EEPROM   */
    /* Use these to save the state of these state variables when ever you change them*/ 
    
    /* These are stored in the first and 2nd rows.  Skipping the first byte though */
    EEPROM_WriteByte(op_mode, EE_STATE_BASE_ADDR+OP_MODE_OFFSET);                
    EEPROM_WriteByte(CouplerSetNum, EE_STATE_BASE_ADDR+COUPLERSETNUM_OFFSET); 
    EEPROM_WriteByte(ser_data_out, EE_STATE_BASE_ADDR+SER_DATA_OUT_OFFSET); 
    
    /* 2nd row variables */
    for(i=0;i<CALLSIGN_LEN;++i)  /*  offset 16 bytes */
    {   EEPROM_WriteByte(Callsign[i],EE_STATE_BASE_ADDR+CALLSIGN_OFFSET+i);   /* first 7 bytes of row */
    }    
    //for(i=0;i<GRIDSQUARE_LEN;++i)  /*  next 9 bytes to finish the 16 byte row */
    //{
    //    EEPROM_WriteByte(Grid_Square[i],EE_STATE_BASE_ADDR+GRIDSQUARE_OFFSET+i); /*  last 9 bytes of block */ 
    //}    
    
    return 0;
}

/* [] END OF FILE */
