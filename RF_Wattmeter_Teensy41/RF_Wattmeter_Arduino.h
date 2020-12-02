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
//#define SSD1306_OLED
#define NEXTION
#define DETECTOR_TEMP_CONNECTED
//#define LORA   // This is set in Serial.c also
//#define SWR_ANALOG   // enables cal and SWR DAC output for embedded amplifier use, in this case a 1296 amp
//#define AMP1296    // enables specific hartd coded cal values for voltages for 1296 amp

#ifdef NEXTION
  #define nexSerial Serial3
  #define dbSerial Serial
  #include <Nextion.h>
#endif

#ifdef SSD1306_OLED
    #include "ssd1306.h"
    #define DISPLAY_ADDRESS 0x3C //  for OLED i2C - 011110+SA0+RW - 0x3C or 0x3D NOTE1
    // If you are using a 128x64 OLED screen, the address will be 0x3C or 0x3D, if one does not work try with the other one.
#endif

#define CALLSIGN_LEN            (7u)
char   Callsign[CALLSIGN_LEN] = "K7MDL\0";
unsigned char default_METERID = 100; // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station
// range is 100 to 119 based on UI input method design limits
// Can be overriden by external commands if stored in EEPROM
const float HV_MAX = 28.5;
const float V14_MAX = 14.6;
const float CURR_MAX = 15.0;
const unsigned char TEMP_MAX = 135;

/*******************************************
Non-user edit section
*********************************************/
void Clear_buf(void);
void OTRSP_setup(void);
unsigned char OTRSP(void);
unsigned char Translate_Band(unsigned char);


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
unsigned long Timer_X00ms_InterruptCnt;
unsigned long Timer_X00ms_Last;
unsigned long Timer_X00ms_Last_AD;
unsigned long Timer_X00ms_Last_USB;
unsigned long Timer_X00ms_Last_Nex;
unsigned long Timer_X00ms_Last_OLED;

// Rest of program...
//#define METERID 102 // Set the ID for this meter to permit monitoring more than 1 meter unit on a remote station
#define METER_RATE 2   // used to skip serial data output to a lower rate
#define EEADDR 64 // Start location to write data table structure in EEPROM.  Byte level data values will start at 2.  EEPROM status is byte 0
#define MUX_FWD A10       // These are the Analog Mux input assignments
#define MUX_REF A11
#define MUX_TEMP A14      // temperature from detector for better calibration.  ADL5519 and some AD8318 modules.  This is nto the RF amp heat sink temp!
#define MUX_14V A16
#define MUX_CURR A15
#define MUX_HV A17
#define ad_Fwd A10    // Analog 35 pin for channel 0
#define ad_Ref A11   // Analog 36 pin for channel 1
#define EEPROM_SIZE  4284   // 4284 for Teensy 4.1    //1024 for ATMega328P.  ESP32 is in Flash so can be any reasonable size.

#define Band_Decode_Control 2
#define Antenna_Select 3
#define Nextion_Switch 4

#define TEST_EEPROM 0  // change to 1 and reprogram, then set back to 0 to reset EEPROM data to default
#define SWR 2
#define PWR 1
#define MENU 3
#define NO 0
#define YES 1
#define ADC_COUNTS 1024.0    // 4096 for ESP32 12bit, 1024 for 10 bit ESP32 and Nano.
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
// end row 1 data 
#define V14_CAL_OFFSET          (0x0020)  // byte 20, 21, 22, 23
#define CURR_CAL_OFFSET         (0x0024)  // byte 24, 25, 26, 27
#define TEMP_CAL_OFFSET         (0x0028)  // byte 28, 29, 2A, 2B
#define CURR_0_OFFSET           (0x002C)  // byte 2C, 2D, 2E, 2F spare

// bytes 2C to 3F spare 

// start row 5 data (whole table)
#define CAL_TBL_ARR_OFFSET      (0x0040)  /* start row 2 and beyond  */
#define EE_SAVE_YES             (65u)
#define TEST_LEN                (14u)
#define CFG_ARR_TEST_STRING     "01234567890123"
#define Serial_USB_DEVICE       (0u)
/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define Serial_USB_BUFFER_SIZE  (64)
#define LINE_STR_LENGTH         (20u)
#define BUF_LEN  Serial_USB_BUFFER_SIZE

/* Global Variables */
//char* parity[] = {"None", "Odd", "Even", "Mark", "Space"};
//char* stop[]   = {"1", "1.5", "2"};
float Vref = 5.0;        // 3.3VDC for Nano and ESP32 (M5stack uses ESP32)  ESP32 also has calibrated Vref curve
unsigned char METERID = 100;    // tracks current Meter ID number   Resets to default_METERID.
unsigned char CouplerSetNum = 0;   // 0 is the default set on power up.  
unsigned char ser_data_out = 0;
unsigned char Reset_Flag = 0;
long updateTime = 0;       // time for next update
float Fwd_dBm = 0;
float Ref_dBm = 0;
float FwdPwr = 0;
float RefPwr = 0;
float SWRVal = 0;
float SWR_Serial_Val = 0;
float FwdVal = 0;
float RefVal = 0;
unsigned char Edit_Atten = 0;
unsigned char op_mode = SWR;
unsigned char counter1 = 0;
unsigned char Button_A = 0;
unsigned char Button_B = 0;
unsigned char Button_C = 0;
unsigned char NewBand;
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
unsigned char readIndex_Fwd = 0;              // the index of the current reading
unsigned char readIndex_Ref = 0;              // the index of the current reading
float total_Fwd = 0;                  // the running total
float total_Ref = 0;                  // the running total
unsigned char EE_PGM_Status = 0u;
unsigned char EE_PGM_Failed;
unsigned char rx_buffer[Serial_USB_BUFFER_SIZE];
unsigned char tx_buffer[Serial_USB_BUFFER_SIZE];
unsigned char rx_count = 0;
unsigned char tx_count = 0;
float set_hv_max = 28.5;   // max values for VDC, Current and Temp if used in an amplifier wit hthese sensors
float set_v14_max = 14.8;
float set_curr_max = 10;
unsigned char set_temp_max = 132;
float hv_cal_factor;
float v14_cal_factor;
float curr_cal_factor;
float curr_zero_offset;
float temp_cal_factor;
static unsigned char sdata[Serial_USB_BUFFER_SIZE], *pSdata=sdata, *pSdata1=sdata, *pSdata2=sdata;
extern char cmd[64];
float value1_last;
unsigned char AuxNum1, AuxNum2;  // N1MM OTRSP serial protocol control for 4 2 4 bit IO ports for antenna and transverter control.  See OTRSP.C
unsigned char decoder_band_last = 0;

// Function declarations
void toggle_ser_data_output(char);
void setup(void);
void adRead(void);
void read_Cal_Table_from_EEPROM(void);
void sendSerialData(void);
void print_cal_table(void);
void print_Cal_Table_progress(unsigned char);
void print_Cmd_Progress(unsigned char);
void Cal_Table();
void Cal_Table_write(void);
void reset_EEPROM();
void write_Cal_Table_to_EEPROM(void);
void write_Cal_Table_from_Default(void);
void get_remote_cmd(void);
unsigned int serial_usb_read(void);
void serial_usb_write(void);
unsigned char BCDToDecimal(unsigned char BCD);
void Band_Decoder(void);
float hv_read(void);
float v14_read(void);
float curr_read(void);
float temp_read(void);

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
void BandSelect(unsigned char);
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
unsigned char update_Nextion(unsigned char);
unsigned char pg;
unsigned char WAIT = 0;  // traffic flag - if a function is waiting for a response for the display set this flag to hold off display updates
#ifdef LORA
unsigned int DLY = 600;  // Set the value to prevent missed messages. 50 for wired, 500 for LoRa set in LoRaCfg
#endif
#ifndef LORA
unsigned int DLY = 50;  // Set the value to prevent missed messages. 50 for wired, 500 for LoRa set in LoRaCfg    
#endif

#endif

// EEPROM related functions
unsigned char EEPROM_Init(unsigned char); /* function to initially copy the config table array to EEPROM on first use.  Calls Read or Write */
unsigned char EEPROM_Init_Write(void); /* function to initially copy the config table array TO the EEPROM on first use */
unsigned char EEPROM_Init_Read(void); /* function to initially copy the config table array FROM the EEPROM on first use */
unsigned char Copy_to_EE(unsigned char e_row, unsigned char c_array[16],unsigned char print);  /*  called by EEPROM_Init_Write */
unsigned char EE_Save_State(void);

struct Band_Cal {
    char  BandName[12];
    float Cpl_Fwd;
    float Slope_F;
    float Offset_F;
    float Cpl_Ref;
    float Slope_R;
    float Offset_R;
} Band_Cal_Table_Def[NUM_SETS] = {
     {"HF", 72.0, -0.02145, 0.48840, 73.6, -0.02145, 0.48840},
     {"50MHz", 72.6, -0.02148, 0.48102, 72.6, -0.02154, 0.51408},
     {"144MHz", 63.4, -0.02149, 0.48256, 63.4, -0.02202, 0.50459},
     {"222MHz", 60.5, -0.02160, 0.48026, 60.5, -0.02192, 0.50488},
     {"432MHz", 59.6, -0.02154, 0.45122, 59.6, -0.02171, 0.48347},
     {"902MHz", 59.2, -0.02149, 0.43525, 59.2, -0.02291, 0.45729},
#ifdef SWR_ANALOG    
    {"1296MHz",61.5, -0.02224, 0.40569, 51.5, -0.02607, 0.38766},  // for KitProg
#elif !SWR_ANALOG    
    {"1296MHz",72.6, -0.02144, 0.43581, 72.6, -0.02174, 0.45506},  // for normal board
#endif    
     {"2.3GHz", 60.1, -0.02514, 0.644, 60.1, -0.02514, 0.644},
     {"3.4GHz", 60.2, -0.02514, 0.644, 60.2, -0.02514, 0.644},
     {"5.7GHz", 60.3, -0.02514, 0.644, 60.3, -0.02514, 0.644},
     {"10GHz", 60.4, -0.02514, 0.644, 60.4, -0.02514, 0.644}
    };
struct Band_Cal Band_Cal_Table[NUM_SETS];

#ifdef SSD1306_OLED
    void OLED(void);
#endif 

// For Nextion Display usage on Serial3
#ifdef NEXTION
    long rcv_num;
 //   unsigned char pg = 0;  // flag to track page changes to minimize writes to the sliders and other fields
    struct NexObject savecfg_btn_1, fwd_cal, ref_cal, fwd_cal_num, band, ref_cal_num, f_att_minus, f_att_plus, \
                        r_att_minus, r_att_plus, toMain, toMain1, toConfig, toSet1, Main, Coupler, Set1, FactoryRst1, FactoryRst2, savecfg_btn_2,\
                        hv_adj, v14_adj, temp_adj, curr_adj, meterID_adj, meterID, band_set_adj, band_set, Set1_Bandvar, band_cal, Graph_Timer, \
                        hv_max, v14_max, temp_max, curr_max, FdBm, RdBm, toPwrGraph, PwrGraph, fPwrGraph, swrGraph, fPwr_scale, fscale, rscale, \
                        fPwrNum, rPwrNum, swrNum, HPPwrTarget, LPPwrTarget, Measure_hi, Measure_lo, HP_F_VDC, HP_R_VDC, LP_F_VDC, LP_R_VDC, \
                        savecfg_btn_4, Units, toPowerCal, PowerCal, CalcFwd, CalcRef, Aux1, Aux2, PTT_CW, B_HF, B_50, B_144, B_222, B_432, \
                        B_902, B_1296, B_2_3G, B_3_4G, B_5_7G, B_10G, BandSel;
    struct NexObject *nex_listen_list[] = {
        &savecfg_btn_1, &fwd_cal, &ref_cal, &f_att_minus, &f_att_plus, &r_att_minus, &band, &toMain, &toMain1, \
        &r_att_plus, &toMain, &toConfig, &toSet1, &toPwrGraph, &FactoryRst1, &FactoryRst2, &Main, &Coupler, &Set1, \
        &hv_adj, &v14_adj, &temp_adj, &band_set_adj, &meterID_adj, &Set1_Bandvar, &curr_adj, \
        &savecfg_btn_2, &savecfg_btn_4, &Measure_hi, &Measure_lo, &CalcFwd, &CalcRef, \
        &B_HF, &B_50, &B_144, &B_222, &B_432, &B_902, &B_1296, &B_2_3G, &B_3_4G, &B_5_7G, &B_10G, NULL};
  
    long value;

    // Nextion Page ID, not the object ID
    #define page0_ID 0  // Main
    #define page1_ID 1  // Coupler - set up the attenuator values per band
    #define page2_ID 2  // Set1 
    #define page3_ID 3  // Power graphing page
    #define page4_ID 4  // PowerCal - calibrate ADC fwd and ref
    #define page5_ID 5  // Band Select Quick buttons

    // Nextion Main object ID
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
    
#endif   // end this section of NEXTION related var and defines

/* [] END OF FILE */
