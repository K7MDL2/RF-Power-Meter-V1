/* ========================================
 *  RF_Wattmeter.h
 * For the PSoC5LP version
 * K7MDL June 2020
 *
 * ========================================
*/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h> 

/******************************************************************************
  User edited values here for Callsign and Meter ID number and Some Setpoints
******************************************************************************/
#define SSD1306_OLED
#define OLED_COMBO_LAYOUT   // requires SSD1306 define active.
#define NEXTION           // OK to run OLED at same time
#define DETECTOR_TEMP_CONNECTED     // Tested with teh ADL5519 onboard temp output. 
//#define LORA   // This is set in Serial.c also
//#define SWR_ANALOG   // enables cal and SWR DAC output for embedded amplifier use, in this case a 1296 amp
//#define AMP1296    // enables specific hard coded cal values for voltages for 1296 amp
//#define TEENSY4_OTRSP_CW_PTT   // Include the PTT and CW pin operation from OTRSP commands. Can comment out if not using OTRSP to prevent unused port event triggers.

// On the Teensy 4.X these are USB Serial so no pin assignments needed.  Teensy 4.x can have up to 3 Serial USB ports, 8 hardware serial ports.
// Serial is main.  SerialUSB1 and SerialUSB2 are the others.
#define RFWM_Serial Serial   // RF Wattmeter data output.  Also accepts control commands and debug output in Serial Monitor.
#define OTRSP_Serial SerialUSB1    // OTRSP Serial protocol from programs like N1MM+ for transveter or antenna control
#define BAND_DEC_Serial SerialUSB2   // when and if serial band decoder methods used

#ifdef NEXTION
    //#define nexSerial Serial1   // defined in NexConfig.h
    #define SERIAL1_RX_PIN 0
    #define SERIAL1_TX_PIN 1
    //#define dbSerial Serial     // defined in NexConfig.h
    #include <Nextion.h>
    #define NexSerialBAUD 38400
#endif

#ifdef SSD1306_OLED
    //#include "ssd1306.h"
    //#define DISPLAY_ADDRESS 0x3C //  for OLED i2C - 011110+SA0+RW - 0x3C or 0x3D NOTE1
    // If you are using a 128x64 OLED screen, the address will be 0x3C or 0x3D, if one does not work try with the other one.
    #include <SPI.h>
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels    
    // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins OR SDA1, SCL1 pins with "Wire1" class)
    #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #define Wire Wire1   // Using 2nd I2C port pins for wiring convenience.  My OLED display has pullup resistors installed, check yours.
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

//#define METERID 102 // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station
#define CALLSIGN_LEN            (7u)
char   Callsign[CALLSIGN_LEN] = "K7MDL\0";
uint8_t default_METERID = 100; // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station
// range is 100 to 119 based on UI input method design limits
// Can be overriden by external commands if stored in EEPROM
const float HV_MAX = 28.5;
const float V14_MAX = 14.6;
const float CURR_MAX = 15.0;
const uint8_t TEMP_MAX = 135;

/*******************************************
Non-user edit section
*********************************************/
void Clear_buf(void);
void OTRSP_setup(void);
uint8_t OTRSP(void);

#ifdef DETECTOR_TEMP_CONNECTED
float TempVal = 0.0;    
#endif

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow usage of the floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif

/* Interrupt prototypes */
uint32_t Timer_X00ms_InterruptCnt;
uint32_t Timer_X00ms_Last;
uint32_t Timer_X00ms_Last_AD;
uint32_t Timer_X00ms_Last_USB;
uint32_t Timer_X00ms_Last_Nex;
uint32_t Timer_X00ms_Last_OLED;

// Rest of program...
#define METER_RATE 2        // used to skip serial data output to a lower rate
#define EEADDR CAL_TBL_ARR_OFFSET // Start location to write data table structure in EEPROM.  Byte level data values will start at 2.  EEPROM status is byte 0
//#define EEPROM_SIZE  4284   // 4284 for Teensy 4.1    //1024 for ATMega328P.  ESP32 is in Flash so can be any reasonable size.  Using sizeof eeprom function in code.

#define ADC_VREF (3.3)   // For Teensy4.1 which is a 3.3V chip  
// Define the Analog input pins   -- !!!! Thesde are 3.3VDC max on Teensy 4.X PUs!!!!
#define ADC_FWD A4        // These are the Analog Mux input assignments for Teensy 4.1
#define ADC_REF A5
#define ADC_TEMP A6       // temperature from detector for better calibration.  ADL5519 and some AD8318 modules.  This is nto the RF amp heat sink temp!
#define ADC_CURR A7
#define ADC_14V A8
#define ADC_HV A9

// Band Decoder Input Pins - Assign these according to your needs includng wiring convenience.  
// Each pin is mapped into bytes so there is no need to have them adjacent.  Easier when they are for wiring purposes.  See GitHUb Wiki pages for wiring example chart.
#define BAND_DEC_IN_0   3  // 6 input pins.  Can take any pattern to be translated to various ports.  Example, BCD on 3 pins for transverter/amp, last 3 BCD for antenna switch
#define BAND_DEC_IN_1   4  // This example is using 4 wire BCD in my install so not using all 6 defined pins, just 4 with PTT and Gnd on the same Band In connector
                           // Comment out lines in Band input read function to match, set the unused bit to 0.  Dont want floating bits.
#define BAND_DEC_IN_2   5
#define BAND_DEC_IN_3   6
//#define BAND_DEC_IN_4   99    // not using so set pin to somehting impossible
//#define BAND_DEC_IN_5   99    // only needed if not using BCD input otherwise need to trade pins with another port
#define BAND_DEC_PTT_IN 7       //  PTT input from radio.  Will pass through to PTT out, possibly with translations.

// Band Decoder Banks A and B and C
#define BAND_DEC_A_0    33      // Only using 5 pins for BCD transverter control so bits 5, 6, and 7 are disabled in this example.
#define BAND_DEC_A_1    34
#define BAND_DEC_A_2    35
#define BAND_DEC_A_3    36
#define BAND_DEC_A_4    37
#define BAND_DEC_A_5    99      //38      
#define BAND_DEC_A_6    99      // not using so set pin to somehting impossible
#define BAND_DEC_A_7    99      // not using so set pin to somehting impossible

#define BAND_DEC_B_0    24
#define BAND_DEC_B_1    25
#define BAND_DEC_B_2    26
#define BAND_DEC_B_3    27
#define BAND_DEC_B_4    28
#define BAND_DEC_B_5    29
#define BAND_DEC_B_6    30
#define BAND_DEC_B_7    31

#define BAND_DEC_C_0    8
#define BAND_DEC_C_1    9
#define BAND_DEC_C_2    10
#define BAND_DEC_C_3    11
#define BAND_DEC_C_4    12
#define BAND_DEC_C_5    99    // not using so set pin to somehting impossible
#define BAND_DEC_C_6    99    // not using so set pin to somehting impossible
#define BAND_DEC_C_7    99    // not using so set pin to somehting impossible

// I2C pins for use with OLED or other IO
#define SDA1_PIN 17
#define SCL1_PIN 16

// Arduino Band Decoder and CW/PTT output Pin Assignments
#define PTT_OUT         39    // Follows PTT IN from Band decoder input and/or OTRSP serial line DTR/RTS.  May have translations applied
#define PTT_OUT2        40    // Follows PTT IN from Band decoder input and/or OTRSP serial line DTR/RTS.  May have translations applied
#define CW_KEY_OUT      13    // CW is from OTRSP serial line RTS/DTR 

// 4 pins unused --->  41/a17, 14/A0, 15/A1, and 32 are unused still in this example tailered to one of my installations.

uint8_t Band_Dec_In_Byte;       // Byte storing decoder input pattern
uint8_t Band_Dec_OutA_Byte;     // Byte representing pattern for Port A (which is a collection of pins changed by Bit Set commands)
uint8_t Band_Dec_OutB_Byte;
uint8_t Antenna_Select;         // byte value pattern overlaid on any nnumber of port pins.  Set bit commands break out which pins are used
uint8_t SEQ_Delay = 25;         // milliseconds delay for sequencing transveter and amps PTT

#define RX 0
#define TX 1
#define SWR 2
#define PWR 1
#define MENU 3
#define NO 0
#define YES 1
#define ADC_COUNTS 1024.0       // 4096 for ESP32 12bit, 1024 for 10 bit ESP32 and Nano.

// Edit the Coupler Set data in the Cal_Table function.  Set the max number of sets here, and the default to load at startup
#define NUM_SETS 11 // 11 bands, 0 through 10 for example

#define SETPOINT_LEN            (0x0004)
#define TEMPERATURE_LEN         (0x0001)
#define METERID_LEN             (0x0001)
#define EE_STATE_BASE_ADDR      (0x0000)  // row 0
#define EE_RESERVED_BYTE1       (0x0001)
#define OP_MODE_OFFSET          (0x0002)
#define COUPLERSETNUM_OFFSET    (0x0003)
#define SER_DATA_OUT_OFFSET     (0x0004)  
#define HV_MAX_OFFSET           (0x0005)  // 5, 6, 7, 8 - 4 bytes each needed for ASCII version of floats  ex: 113 degrees or 78.2 dB or 28.6 VDC
#define V14_MAX_OFFSET          (0x0009)  // 9, A, B, C - 4 bytes each needed for ASCII version of floats  ex: 113 degrees or 78.2 dB or 28.6 VDC
#define TEMP_MAX_OFFSET         (0x000D)  // 1 byte - temp is 1 byte ex: 113
#define METERID_OFFSET          (0x000E)  // 1 byte 100-119 range
#define SPARE0_OFFSET           (0x000F)  // byte F spare
/* end row 0, start row 1  */
#define CALLSIGN_OFFSET         (0x0010)  // bytes 10 to 16 in row 1   7 bytes
#define CURR_MAX_OFFSET         (0x0017)  // 17, 18, 19, 1A - 4 bytes each needed for ASCII version of floats  ex: 113 degrees or 78.2 dB or 28.6 VDC
#define HV_CAL_OFFSET           (0x001B)  // byte 1B, 1C, 1D, 1E
#define SPARE1_OFFSET           (0x001F)  // byte 1F spare
// end row 1 data start row 2
#define V14_CAL_OFFSET          (0x0020)  // byte 20, 21, 22, 23
#define CURR_CAL_OFFSET         (0x0024)  // byte 24, 25, 26, 27
#define TEMP_CAL_OFFSET         (0x0028)  // byte 28, 29, 2A, 2B
#define CURR_0_OFFSET           (0x002C)  // byte 2C, 2D, 2E, 2F
// Start Row 3
#define TRANS_INPUT             (0x0030)  // byte 30
#define TRANS_A                 (0x0031)  // byte 31
#define TRANS_B                 (0x0032)  // byte 32
#define TRANS_C                 (0x0033)  // byte 33
#define DIS_OTRSP_BAND_CHANGE   (0x0034)  // byte 34
#define PTT_IN_POLARITY         (0x0035)  // byte 35
#define PTT_OUT_POLARITY        (0x0036)  // byte 36
#define CW_KEY_OUT_POLARITY     (0x0037)  // byte 37
#define PORTA_IS_PTT            (0x0038)  // byte 38
#define PORTB_IS_PTT            (0x0039)  // byte 39
#define PORTC_IS_PTT            (0x003A)  // byte 3A

// start row 4 data
#define CAL_TBL_ARR_OFFSET      (0x0040)  /* start row 2 and beyond  */
#define EE_SAVE_YES             (65u)
#define TEST_LEN                (14u)
#define CFG_ARR_TEST_STRING     "01234567890123"
/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.*/
#define Serial_USB_BUFFER_SIZE  (64)
#define LINE_STR_LENGTH         (20u)
#define BUF_LEN  Serial_USB_BUFFER_SIZE

/* Global Variables */
//char* parity[] = {"None", "Odd", "Even", "Mark", "Space"};
//char* stop[]   = {"1", "1.5", "2"};
float Vref = 5.0;        // 3.3VDC for Nano and ESP32 (M5stack uses ESP32)  ESP32 also has calibrated Vref curve
uint8_t METERID = 100;    // tracks current Meter ID number   Resets to default_METERID.
uint8_t CouplerSetNum = 0;   // 0 is the default set on power up.  
uint8_t ser_data_out = 0;
uint8_t Reset_Flag = 0;
uint32_t updateTime = 0;       // time for next update
float Fwd_dBm = 0;
float Ref_dBm = 0;
float FwdPwr = 0;
float RefPwr = 0;
float SWRVal = 0;
float SWR_Serial_Val = 0;
float FwdVal = 0;
float RefVal = 0;
uint8_t Edit_Atten = 0;
uint8_t op_mode = SWR;
uint8_t counter1 = 0;
uint8_t Button_A = 0;
uint8_t Button_B = 0;
uint8_t Button_C = 0;
uint8_t NewBand;
float CouplingFactor_Fwd = 0;
float CouplingFactor_Ref = 0;
float Offset_F = 0.6455;  // AD8318 is 0.5 offset for around +5dBm to +10dBm (nonlinear range) for 0.5V to about 2.2 volts for whole range.  We want to max at 0dBm
float Slope_F = 0.02521;  // AD8318 is about 25mV per dB with temp compensation
float Offset_R = 0.6455;  
float Slope_R = 0.02521;  
float FwdPwr_last = 0;
float Pwr_hi = 0, Pwr_lo = 0;
float FwdVal_hi = 0, FwdVal_lo = 0; // used for auto cal
float RefVal_hi = 0, RefVal_lo = 0; // used for auto cal
#define NUMREADINGS 1   // adjust this for longer or shorter smooting period as AD noise requires
float readings_Fwd[NUMREADINGS];      // the readings from the analog input
float readings_Ref[NUMREADINGS];      // the readings from the analog input
uint8_t readIndex_Fwd = 0;              // the index of the current reading
uint8_t readIndex_Ref = 0;              // the index of the current reading
float total_Fwd = 0;                  // the running total
float total_Ref = 0;                  // the running total
uint8_t EE_PGM_Status = 0u;
uint8_t EE_PGM_Failed;
uint8_t rx_buffer[Serial_USB_BUFFER_SIZE];
uint8_t tx_buffer[Serial_USB_BUFFER_SIZE];
uint8_t rx_count = 0;
uint8_t tx_count = 0;
float set_hv_max = 28.5;   // max values for VDC, Current and Temp if used in an amplifier wit hthese sensors
float set_v14_max = 14.8;
float set_curr_max = 10;
uint8_t set_temp_max = 132;
float hv_cal_factor;
float v14_cal_factor;
float curr_cal_factor;
float curr_zero_offset;
float temp_cal_factor;
static uint8_t sdata[Serial_USB_BUFFER_SIZE], *pSdata=sdata, *pSdata1=sdata, *pSdata2=sdata;
float value1_last;
char AuxCmd0[20];
uint8_t AuxNum1, AuxNum2;  // N1MM OTRSP serial protocol control for 4 2 4 bit IO ports for antenna and transverter control.  See OTRSP.C
uint8_t decoder_band_last = 200;
uint8_t PTT_IN_state = 0;                 // This is the state of PTT Input RX or TX independent of polarity. 1 is TX, 0 is RX.
                                          // Polarity is used to interpret the input signal to set state to RX or TX.                                        
uint8_t PTT_IN_state_last = 200;          // Track changes
//uint8_t PTT_IN_polarity = 0;            // 1 is ACTIVE HIGH, 0 is ACTIVE LOW.   If a input pin is low and Polarity is 0, then we have TX.
                                          // If an input pin is is high and polarity is 1, we have TX (PTT_INI_state = 1).  All other states are RX.
                                        
volatile uint8_t PTT_OUT_state = 0;       // This is the state of PTT output for RX or TX independent of polarity.
uint8_t PTT_OUT_state_last = 200;         // Track changes
//uint8_t PTT_OUT_polarity = 0;           // 1 is ACTIVE HIGH, 0 is ACTIVE LOW. 

volatile uint8_t CW_KEY_OUT_state = 0;  // This is the state of CW output RX or TX independent of polarity. 1 is TX, 0 is RX.
uint8_t CW_KEY_OUT_state_last = 200;      // Track changes
//uint8_t CW_KEY_OUT_polarity = 0;        // 1 is ACTIVE HIGH, 0 is ACTIVE LOW. 
// Generally this will follow OTRSP and and state will be ignored, but polarity will still apply.  
// If an input pin is ever assigned to a CW key input then state will be needed like for PTT input
unsigned long PTT_IN_debounce_timestamp = 0;
uint8_t PTT_IN_pin = 0;
uint8_t PTT_IN_pin_last = 200;  // Force process to set proper state at startup
uint8_t PTT_IN_changed = 200;   // Force process to set proper state at startup
uint8_t PortA_state = 0;
uint8_t PortB_state = 0;
uint8_t PortC_state = 0;

// Function declarations
void toggle_ser_data_output(uint8_t);
void setup(void);
void adRead(void);
void read_Cal_Table_from_EEPROM(void);
void sendSerialData(void);
void print_cal_table(void);
void print_Cal_Table_progress(uint8_t);
void print_Cmd_Progress(uint8_t);
void Cal_Table();
void Cal_Table_write(void);
void reset_EEPROM();
void write_Cal_Table_to_EEPROM(void);
void write_Cal_Table_from_Default(void);
void get_remote_cmd(void);
uint16_t serial_usb_read(void);
void serial_usb_write(void);
uint8_t BCDToDecimal(char *hex);
uint8_t Band_Decoder(void);
float hv_read(void);
float v14_read(void);
float curr_read(void);
float temp_read(void);
uint8_t OTRSP(void);
uint8_t OTRSP_Process(void);

#ifdef SWR_ANALOG  // Update SWR Analog output for Amplifier protection when using KitProg board in an amplifier
float SWR_Fail(void);
#endif

#ifdef NEXTION
void savecfg_btn_1_push_Callback(void *);
void savecfg_btn_1_pop_Callback(void *);
void fwd_cal_pop_Callback(void *);
void ref_cal_pop_Callback(void *);
void f_att_minus_pop_Callback(void *);
void f_att_plus_pop_Callback(void *);
void r_att_minus_pop_Callback(void *);
void r_att_plus_pop_Callback(void *);
void toMain_push_Callback(void *);
void toConfig_pop_Callback(void *);
void FactoryRst1_pop_Callback(void *);
void FactoryRst2_pop_Callback(void *);
void Set1_Callback(void *ptr);
void toPwrGraph_Callback(void *);
void meterID_adj_pop_Callback(void *);
void band_set_adj_pop_Callback(void *);
void hv_adj_pop_Callback(void *ptr);
void v14_adj_pop_Callback(void *ptr);
void curr_adj_pop_Callback(void *ptr);
void temp_adj_pop_Callback(void *ptr);
void band_pop_Callback(void *ptr);
void Measure_hi_pop_Callback(void *ptr);
void Measure_lo_pop_Callback(void *ptr);
void CalcFwd_pop_Callback(void *ptr);
void CalcRef_pop_Callback(void *ptr);
void BandSelect(uint8_t);
void BandSelect_HF_pop_Callback(void *ptr);
void BandSelect_50_pop_Callback(void *ptr);
void BandSelect_144_pop_Callback(void *ptr);
void BandSelect_222_pop_Callback(void *ptr);
void BandSelect_432_pop_Callback(void *ptr);
void BandSelect_902_pop_Callback(void *ptr);
void BandSelect_1296_pop_Callback(void *ptr);
void BandSelect_2_3G_pop_Callback(void *ptr);
void BandSelect_3_4G_pop_Callback(void *ptr);
void BandSelect_5_7G_pop_Callback(void *ptr);
void BandSelect_10G_pop_Callback(void *ptr);
uint8_t update_Nextion(uint8_t);
uint8_t pg;
uint8_t WAIT = 0;  // traffic flag - if a function is waiting for a response for the display set this flag to hold off display updates
#ifdef LORA
uint16_t DLY = 600;  // Set the value to prevent missed messages. 50 for wired, 500 for LoRa set in LoRaCfg
#endif
#ifndef LORA
uint16_t DLY = 50;  // Set the value to prevent missed messages. 50 for wired, 500 for LoRa set in LoRaCfg    
#endif

#endif

// EEPROM related functions
uint8_t EEPROM_Init(uint8_t); /* function to initially copy the config table array to EEPROM on first use.  Calls Read or Write */
uint8_t EEPROM_Init_Write(void); /* function to initially copy the config table array TO the EEPROM on first use */
uint8_t EEPROM_Init_Read(void); /* function to initially copy the config table array FROM the EEPROM on first use */
uint8_t Copy_to_EE(uint8_t e_row, uint8_t c_array[16],uint8_t print);  /*  called by EEPROM_Init_Write */
uint8_t EE_Save_State(void);

struct Band_Cal {
    char  BandName[12];
    float Cpl_Fwd;
    float Slope_F;
    float Offset_F;
    float Cpl_Ref;
    float Slope_R;
    float Offset_R;
    uint8_t band_input_pattern;  // pattern for band decoder input pins to find match against and change bands if there is a match
    uint8_t band_A_output_pattern;  // pattern per band for band decode output Port A pins
    uint8_t band_B_output_pattern;  // pattern per band for band decode output Port B pins
    uint8_t band_C_output_pattern;  // pattern per band for band decode output Port C pins
} Band_Cal_Table_Def[NUM_SETS] = {   // set up defaults for EEPROM
     {"HF",     72.0, -0.02145, 0.48840, 73.6, -0.02145, 0.48840, 0x00, 0x00, 1, 0},
     {"50MHz",  72.6, -0.02148, 0.48102, 72.6, -0.02154, 0.51408, 0x01, 0x01, 2, 1},
     {"144MHz", 63.4, -0.02149, 0.48256, 63.4, -0.02202, 0.50459, 0x02, 0x02, 3, 2},
     {"222MHz", 60.5, -0.02160, 0.48026, 60.5, -0.02192, 0.50488, 0x03, 0x03, 4, 3},
     {"432MHz", 59.6, -0.02154, 0.45122, 59.6, -0.02171, 0.48347, 0x04, 0x04, 5, 4},
     {"902MHz", 59.2, -0.02149, 0.43525, 59.2, -0.02291, 0.45729, 0x05, 0x05, 6, 5},
#ifdef SWR_ANALOG    
    {"1296MHz", 61.5, -0.02224, 0.40569, 51.5, -0.02607, 0.38766, 0x06, 0x06, 7, 6},  // for KitProg
#elif !SWR_ANALOG    
    {"1296MHz", 72.6, -0.02144, 0.43581, 72.6, -0.02174, 0.45506, 0x06, 0x06, 7, 6},  // for normal board
#endif    
     {"2.3GHz", 60.1, -0.02514, 0.644, 60.1, -0.02514, 0.644, 0x07, 0x07, 8, 7},
     {"3.4GHz", 60.2, -0.02514, 0.644, 60.2, -0.02514, 0.644, 0x08, 0x08, 9, 8},
     {"5.7GHz", 60.3, -0.02514, 0.644, 60.3, -0.02514, 0.644, 0x09, 0x09, 10, 9},
     {"10GHz",  60.4, -0.02514, 0.644, 60.4, -0.02514, 0.644, 0x0A, 0x0A, 11, 10}
    };
struct Band_Cal Band_Cal_Table[NUM_SETS];   // Calibration table, one for each band

#ifdef SSD1306_OLED
    void OLED(void);
#endif 

#ifdef NEXTION
  #include <RF_Wattmeter_Nextion.h>
// For Nextion Display usage
    uint32_t rcv_num;
    static char cmd[64];
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
    NexText Curr_band = NexText(page3_ID,Curr_band_ID,"Curr_band"); 
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
        
#endif   // end this section of NEXTION related var and defines
/* [] END OF FILE */
