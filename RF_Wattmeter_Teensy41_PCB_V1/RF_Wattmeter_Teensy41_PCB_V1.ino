#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h> 

//#include <usb_serial.h>   // not required for Teensy4.1 but maybe other Arduinos?
#include "RF_Wattmeter_Teensy41_PCB_V1.h"
#include "Watchdog_t4.h"  // https://github.com/tonton81/WDT_T4 for internal watchdog
/*
 *
 * RF Wattmetter and Band Decoder on Arduino Teensy 4.1 with Ethernet option - PCB V1 version
 * by K7MDL  https://github.com/K7MDL2/RF-Power-Meter-V1
 * 
 * This version is tailored to work with the V1 PCB by K7MDL developed April 2022.
 * 
 * 10/29/2022
 * 1. Added last 2 bytes of IP adress to EEPROM.
 *  a. first 2 are fixed by define - ie 192.168.x.y
 *  b. x subnet is in EEPROM as ip_adr1.  Same value used in my_ip_adr and dst_ip_adr
 *  c. y client ip is EEPROM as my_ip_adr0 and dewt_ip_adr0.  Unique value used in my_ip_adr and dst_ip_adr
 *  example        ip(192, 168, ip_adr1, my_ip_adr0);
 *          remote_ip(192, 168, ip_adr1, dest_ip_adr0);
 *          
 *   Look for these defaults to get started in the .h file:
 *   #define DEF_NET_IP_ADR1       192   // Form of (192, 168, 2, 188) which is 192.168.2.188;
 *   #define DEF_NET_IP_ADR2       168
 *   #define DEF_SUBNET_IP_ADR1      2   // byte 3D - shared subnet byte ex: (192, 168, IP_ADR1, MY_IP_ADR0)
 *   #define DEF_MY_IP_ADR0        188   // byte 3E - My ipaddress static IP address byte (192, 168, IP_ADR1, MY_IP_ADR0)
 *   #define DEF_DEST_IP_ADR0       65   // byte 3F - Desination IP Address static IP (DEF_NET_IP_ADR1, DEF_NET_IP_ADR2, IP_ADR1, DEST_IP_ADR0)
 * 
 *  TO DO:  Add remote commands to update the new EEPROM IP values during operation
 * 
 * 4/16/2022 
 * 1. If there is was no enet cable attached and enet was enabled, after the first 10 minute retry, it kept 
 *    retrying every few seconds rather than every 10 minutes.   Fixed.
 * 2. Optimized PTT blocking more, freed up 6M band that was used as a workaround for all 1s case.  
 * 3. If all 1's or all 0's received, or any invalid band decode input, Port A, B and C are forced to 255.
 * 4. Added message 173 to the data streaming out to track PTT state.
 * 5. Changed data output timer from 600ms to 50ms to better capture fast moving power levels.  Increased ADC reads to 25ms.
 * 
 * 4/15/2022
 *  1. Fixed problem with outputs all turning on during startup, including PTT.
 *  2. Removed state inversions added for old proto hardware, helped fix the config and all-on relays issue.
 *  3. Fixed problem of PTT turning on when the radio providing the band decode is powered on or off. Now PTT
 *     checks band decode for a valid band and if the input is all 0's or all l's PTT output changing to TX is prevented.
 *  4. Housed the PCB in a metal box. Tested at 500W on 144 and 432 and other bands and no RFI issues.  The temperature reading, 
 *     often a sign of RFI when the value reported changes 10F-20F during TX, now barely moves 1 degree.  Plan to remove the 
 *     many snap-on ferrites from external cables and retest for RFI resistance.
 *  
 * 4/9/2022
 *  1. Added internal watchdog timer in hopes of removing the external WD card.
 *  2. Reassigned some pins to support a new PCB (V1) designed to hopefully 
 *    be highly RFI resistant.  
 *  3. Added a Combo Ethernet/USB2 jack to the PCB and routed lines from the Teensy to it.
 *  4. PCB is using opto-couplers so some signals are inverted.
 *  5. Both Port A and B are 8 ports each, Port C is 3, Input has 6.  1x PTT Input, 2x PTT outputs. 
 *  6. Input uses a floating ground (Input_Return) that should be ties to the band decoder source ground.  
 *  7. Each input should supply about 1-3ma to activate the optocoupler. Change the resistors as required.
 *
 * 1/7/2021 - Added ADS1115 4 channel ADC for 16 bit RF power resolution, Tuned up serial and ethernet comms.  
 *            Putting the Arduino Teensy Wattmeter and Band Decoder into service in my remote VHF+ transverter/amplifier/antenna switch boxes this week, it now seems ready to go.
 *  1. Measure Bird Line section output (is in the form of dBV) as a power detector but requires too many cal points to warrant effort 
 *      If close cal is not required then it does seem to work.  A significant UI effort would be needed to put that data in for every band.
 *  2. Measuring temperature, RF Fwd & Ref, all 3 on the ADS1115 ADC board.  
 *  3. The onboard ADC is still usable for more things like voltage, current, Icom CI-V.
 *  4. The ADS1115 is far higher resolution with a 16 bit SAR ADC and has PGA with more suitable input ranges (using 4V single ended inputs) and a decent onboard Vref source.  
 *  5. The PSoC5 version uses a 20bit SAR ADC.
 *  6. Using ADS1115 SingleShot mode. Continuous mode does not work well unless a delay is added between changing mux ports and measuring.
 *  7. Too short of delay also causes a startup hang.
 *  8. Increased the i2c bus speed from 100KHz to 400KHz (fast mode) for better ADC results.
 *  9. Found 8SPS on the ADS1115 was too slow.  64 or 128KSPS seem good.
 * 10. Degugged several ethernet and serial port issues arising from toggling on and off the power data output streams. Goals is to always function regardless of outside cpomms state.  
 * 11. Removed delays where possible.
 * 12. Using the ADS1115_WE library per MIT license.  Tried a few, I liked this one best.  https://github.com/wollewald/ADS1115_WE
 * 13. NOTE: Using the Teensy 4.1 alternate i2c pins. This requires using "Wire1" instead of Wire. Used #defines Wire Wire1 but 
 *       I modified the library .cpp and .h fle and those #defines seemed to be ignored at times.
 * 
 * 1/1/2021 - Added Ethernet UDP capability tht parallels the serial data control and montoring link.  Serial is always active.
 *   Ethernet support is a #define ENET.  If compiled there is the  (ENET_ENABLE) EEPROM flag to enable or disable it.  
 *   Used a 2nd Arduino on ethernet to test control/monitor comms.  This is a prep step to sending Nextion serial data to a remote location over the wire.  
 *   Up next is to update the Desktop App to use ethernet.  
 *   A NativeEthernet.cpp file is included in this project that has a patched Ethernet.begin class.  The patch inserts a delay and timeout counter to prevent 
 *      a hang in the case no ethernet is connected.  When calling Ethernet.begin(mac,ip) it succeeds except for the last line where 
 *      it waits for Link_status to be Good, but never will. Using DHCP it times oup properly.  
 *      If Enet option is complied in, and it is enabled (in EEPROM setting) and an Enet startup fails, a flag is set 
 *      and a 10 minute timer is started to restart the enet system. The enet_ready flag is set whe all is good. 
 *      Functions using enet read and write look for the enet_ready flag so they do not call a non working function.
 *   Created an Arduino Teensy test program that is decoding WSJT-X packets so the desktop app is nto required to glean band information
 *   If using the Band Decoder feature with wired Band inputs or N1MM OTRSP then WSJT-X is not required for auto band change.
 *   Changed all debug/info messages to use DBG_Serial #define to enable routing the messages to alternate serial ports.
 *   Also inserted a ">" character at the start of each debug message.  
 *   The Desktop App was updated to look for the ">" char at the first char position.  It then prints it to the screen and exists the parser function
 *   This enables a cleaner debug experience.
 *   
 *   ToDo: Update Desktop App for ethernet control
 *   ToDo: Build Arduino app to route Nextion serial data over ethernet to remote located Wattmeter/Decoder box. 
 *         This app wil lalso have NTP client to get latest time and display on a new Nexton Clock page to be added.
 *   ToDo: Add support for new Nextion page for new Band Decoder Configuration UI features (currently only in the Desktop App UI)   
 *         Full support for the remote config commands are already in place on the CPU here, this work is reatled only to Nextion UI work for a new page.
 *   
 * 12/23/2020 - Changed pin assignmetns to suit simpler decoder build wiring with ULN2803 drivers.  Accounted for driver inversion in Band In and PTT.  
 *   Need to do the same for outputs.  Not required if no inverting buffers used or flip the Polarity settings.
 *   
 * 
 * 12/18/2020 - Changed OLED Watts line to "OFF" on band 0.  Fixed bug where band decoder was not reading input on boot up.
 * 
 * 12/17/2020 -  Ported from PSoC5 C code. Now has full featured Band Decoder with 1 input port and 3 ouput ports, PTT in and out with polarity. 
 * OLED and Nextion display and headless operation supported.  Can run both displays at the same time.
 * 
 * Switched to standard Arduino Nextion Library with a few mods for nex_listen function to detect page number on nav button presses
 * and to resolve compiler warnings. 
 * 
 * OTRSP uses SerialUSB1 obtained by setting the Teensy USB port to Dual Ports setting.  
 * Use triple ports setting for SerialUSB2 port to send the Nextion display to remote PC where it can be bridged to a local hardware 
 * port and displayed.  
 * 
 * Band Decoder ports have many features, see GitHub site and Wiki pages for how to configure.
 * 
 * 
 * ToDo: 
 * 1. Create Config screen on Nextion for voltage, current, temp, and all the Band Decoder features.
 * 2. Finish OTRSP PTT and CW debug.  Add polarity to the CW output pin.
 * 
*/
/*
    Icom CIV and ACC Voltage decoding sections from Band Decoder 2 project 
      https://remoteqth.com/wiki/index.php?page=Band+decoder+MK2
      GNU V2 applies
*/

//struct Band_Cal Band_Cal_Table[10];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#define RESET_EEPROM 0    // Set this to 1 to force a EEPROM reset on program startup - Needed if you change IP adress of desktop monitor also

// #define SerialUSB1 SerialUSB1  // assign as needed for your CPU type. 
                                  // For Teensy set ports to Dual or Triple and Use Serial USB1 and Serial USB2 for 2 and 3rd ports
WDT_T4<WDT1> wdt;   // internal watchdog functionality.
//#define EXT_WD

void setup(void) 
{ 
  // Set up our input pins
  #ifndef ADS1115_ADC
  pinMode(ADC_FWD,INPUT);
  pinMode(ADC_REF,INPUT);
  pinMode(ADC_TEMP,INPUT);   // If nothing is connected to these pins then setting to INPUT_PULLUP will pin them to Vcc and prevent floating around.
  #endif
  pinMode(ADC_CURR,INPUT_PULLUP);
  pinMode(ADC_14V,INPUT);
  pinMode(ADC_HV,INPUT_PULLUP);
  pinMode(ADC_SPARE_A6,INPUT_PULLUP);
  pinMode(BAND_DEC_IN_0, INPUT_PULLUP);
  pinMode(BAND_DEC_IN_1, INPUT_PULLUP);
  pinMode(BAND_DEC_IN_2, INPUT_PULLUP);
  pinMode(BAND_DEC_IN_3, INPUT_PULLUP);
  pinMode(BAND_DEC_IN_4, INPUT_PULLUP);
  pinMode(BAND_DEC_IN_5, INPUT_PULLUP);
  
  pinMode(BAND_DEC_PTT_IN, INPUT_PULLUP);   // Interrupt handler setup for PTT input
  PTT_IN_state = 0;   // 0 is RX mode, 1 is TX mode
  PTT_IN_state_last = 0;

// For the PCB version all opto-couplers are connected such that grounding the input turns ON the
//    output transistor connecting the output to ground. For ouputs controlling relays this is normal (active LOW control).
// In the off and power up state this usually works best since the powered off state and CPU startup states will
//    not cuse the output transistors to turn on causing unwanted external relays activation before full program control
  
  // Set up our output pins, starting with initializing the CW and PTT control pins
  #ifdef TEENSY4_CW_PTT
      pinMode(CW_KEY_OUT, OUTPUT);
      CW_KEY_OUT_state = 0; 
      CW_KEY_OUT_state_last = 200;
      digitalWrite(CW_KEY_OUT, HIGH);    // need to initialize these according to polarity config
  #endif  
     
      pinMode(PTT_OUT, OUTPUT);
      PTT_OUT_state = 0;
      PTT_OUT_state_last = 200;
      digitalWrite(PTT_OUT, HIGH);    // need to initialize these according to polarity config but set to a OFF state here for optocouplers.
      pinMode(PTT_OUT2, OUTPUT);
      digitalWrite(PTT_OUT2, HIGH);    // need to initialize these according to polarity config but set to a OFF state here for optocouplers.
 
  // now our other IO pins
  pinMode(BAND_DEC_A_0, OUTPUT);   // Band Decoder bank A pin (bit) 0
  digitalWrite(BAND_DEC_A_0, HIGH);
  pinMode(BAND_DEC_A_1, OUTPUT);   // Band Decoder bank A pin (bit) 1
  digitalWrite(BAND_DEC_A_1, HIGH);
  pinMode(BAND_DEC_A_2, OUTPUT);
  digitalWrite(BAND_DEC_A_2, HIGH);
  pinMode(BAND_DEC_A_3, OUTPUT);
  digitalWrite(BAND_DEC_A_3, HIGH);
  pinMode(BAND_DEC_A_4, OUTPUT);
  digitalWrite(BAND_DEC_A_4, HIGH);
  pinMode(BAND_DEC_A_5, OUTPUT);
  digitalWrite(BAND_DEC_A_5, HIGH);
  pinMode(BAND_DEC_A_6, OUTPUT);
  digitalWrite(BAND_DEC_A_6, HIGH);
  pinMode(BAND_DEC_A_7, OUTPUT);
  digitalWrite(BAND_DEC_A_7, HIGH);

  pinMode(BAND_DEC_B_0, OUTPUT);   // Band Decoder bank B pin (bit) 0
  digitalWrite(BAND_DEC_B_0, HIGH);
  pinMode(BAND_DEC_B_1, OUTPUT);
  digitalWrite(BAND_DEC_B_1, HIGH);
  pinMode(BAND_DEC_B_2, OUTPUT);
  digitalWrite(BAND_DEC_B_2, HIGH);
  pinMode(BAND_DEC_B_3, OUTPUT);
  digitalWrite(BAND_DEC_B_3, HIGH);
  pinMode(BAND_DEC_B_4, OUTPUT);
  digitalWrite(BAND_DEC_B_4, HIGH);
  pinMode(BAND_DEC_B_5, OUTPUT);
  digitalWrite(BAND_DEC_B_5, HIGH);
  pinMode(BAND_DEC_B_6, OUTPUT);
  digitalWrite(BAND_DEC_B_6, HIGH);
  pinMode(BAND_DEC_B_7, OUTPUT);
  digitalWrite(BAND_DEC_B_7, HIGH);
  
  pinMode(BAND_DEC_C_0, OUTPUT);  // Band Decoder bank C pin (bit) 0
  digitalWrite(BAND_DEC_C_0, HIGH);
  pinMode(BAND_DEC_C_1, OUTPUT); 
  digitalWrite(BAND_DEC_C_1, HIGH);
  pinMode(BAND_DEC_C_2, OUTPUT);
  digitalWrite(BAND_DEC_C_2, HIGH);
  
  // Remaining not used on the PCB version, at least not for "port C" usage.
  //pinMode(BAND_DEC_C_3, OUTPUT); 
  //digitalWrite(BAND_DEC_C_3, HIGH);
  //pinMode(BAND_DEC_C_4, OUTPUT); 
  //digitalWrite(BAND_DEC_C_4, HIGH);
  //pinMode(BAND_DEC_C_5, OUTPUT); 
  //digitalWrite(BAND_DEC_C_5, HIGH);
  //pinMode(BAND_DEC_C_6, OUTPUT); 
  //digitalWrite(BAND_DEC_C_6, HIGH);
  //pinMode(BAND_DEC_C_7, OUTPUT);
  //digitalWrite(BAND_DEC_C_7, HIGH);

  // Initial our serial ports
#ifdef NEXTION
  pinMode(SERIAL1_RX_PIN, INPUT);   // initializing for Nextion or other usage
  pinMode(SERIAL1_TX_PIN, OUTPUT);
#endif  
#ifdef OTRSP_Serial
  OTRSP_Serial.begin(9600);  // open port for OTRSP serial port command input    
#endif
  RFWM_Serial.begin(115200); // For debug or data output
  RFWM_Serial.println(" ");   // Clear our output text from CPU init text

 #if defined(ICOM_ACC)
    pinMode(ADPin, INPUT);
#endif

#ifdef ADS1115_ADC
  #ifdef USE_WIRE1
    Wire1.begin();   // Note Wire1. is used instead of Wire. to use teh Teensy alternate I2C bus port pins.
    Wire1.setClock(400000);
  #else
    Wire.begin();   // Teensy I2C bus port pins.
    Wire.setClock(400000);
  #endif
    if(!adc.init()){
        RFWM_Serial.println("ADS1115 not connected!");
    }
    else 
    {
        //uint8_t success = 1;
        //success = Wire1.endTransmission();
        RFWM_Serial.print("ADS1115 board found at I2C address: ");
        RFWM_Serial.println(I2C_ADDRESS, HEX);
    }
  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_4096); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  //adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel

  /* Set number of conversions after which the alert pin will be active
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_DISABLE_ALERT); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  // adc.setConvRate(ADS1115_64_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  #ifdef ADS1115_SINGLE_MODE
      adc.setMeasureMode(ADS1115_SINGLE); //comment line/change parameter to change mode
  #else
      adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode
  #endif
   /* Choose maximum limit or maximum and minimum alert limit (window)in Volt - alert pin will 
   *  be active when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin will be deactivated (if 
   *  not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alarm pin will be active until the
   * conversion register is read (getResult functions). If disabled the alarm pin will be
   * deactivated with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * Enable or disable latch. If latch is enabled the alarm pin will be active until the
   * conversion register is read (getResult functions). If disabled the alarm pin will be
   * deactivated with next value within limits. 
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will be active, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default
  #ifdef ADS1115_SINGLE_MODE
      RefVal = readChannel(ADS1115_COMP_2_GND);        
      RFWM_Serial.print("ADS1115 Running in Single Mode, ADC Ch2 Voltage = ");
  #else   // continuous mode locks up without 20ms+ delay
      adc.setCompareChannels(ADS1115_COMP_2_GND); //comment line/change parameter to change channel           
      delay(20);
      RefVal = adc.getResult_V();    // for ADS1115 module  
      RFWM_Serial.print("ADS1115 Running in Continuous Mode, ADC Ch2 Voltage = ");      
  #endif

  RFWM_Serial.println(RefVal); 
#endif

#ifdef SSD1306_OLED
    //display_init(DISPLAY_ADDRESS); // This line will initialize your display using the address you specified before.

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))   // Address 0x3C for 128x64, if not working try 0x3D
    { 
      for(;;) // Don't proceed, loop forever
        RFWM_Serial.println(">SSD1306 allocation failed");
    }
    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    //delay(100); // Pause x seconds
    // display.display() is NOT necessary after every single drawing command,
    // unless that's what you want...rather, you can batch up a bunch of
    // drawing operations and then update the screen all at once by calling
    // display.display().

    // Clear the buffer
    display.clearDisplay();
#endif

#ifdef ENET
  if (EEPROM.read(ENET_ENABLE))
  { 
      enet_start(); 
      if (!enet_ready)
      {
          enet_start_fail_time = millis();      // set timer for 10 minute self recovery in main loop
          DBG_Serial.println(">Ethernet System Startup Failed, setting retry timer (10 minutes)");
      }
      DBG_Serial.println(">Ethernet System Startup");
  }
#endif

#ifdef EXT_WD
    // External watchdog board reset pins Just using #2 for now
    pinMode(WD1_PIN, OUTPUT);
    digitalWrite( WD1_PIN, HIGH);   //high is normal state, pulse low to reset timer
    pinMode(WD2_PIN, OUTPUT);       // This is connected to reset the timer.  
    digitalWrite(WD2_PIN, HIGH);
    // The board output drives a relay, pulses it high for 300ms to power cycle this CPU.   
    // Need to reset timer before this happens in main loop.
#else
    // Internal Watchdog (to replace the external card)
    WDT_timings_t config;
    config.trigger = 5; /* in seconds, 0->128 */
    config.timeout = 10; /* in seconds, 0->128 */
    //config.callback = myCallback;
    wdt.begin(config);
    pinMode(WD1_PIN, OUTPUT);
#endif
  
  write_Cal_Table_from_Default();  // Copy default values into memory in case EEPROM not yet initialized
      /*   initialize EEPROM storage if not done before - will overwrite default memory table values from EEPROM if EEPROM was written to before */    
  if (EEPROM.read(0) == 'G') 
  {
    DBG_Serial.println(">EEPROM Data is Valid");
    EEPROM_Init_Read();
    //get_config_EEPROM();  // set last used values for CouplerSetNum (Band) and op_mode if there is data in the EEPROM
  }
 
  if (EEPROM.read(0) != 'G' || RESET_EEPROM == 1) 
  {    // Test if EEPROM has been initialized with table data yet
    DBG_Serial.println(">Write EEPROM");
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
  // ensure the band index is in proper range in case of bad input or memory operation
  if (CouplerSetNum < 0)   // constrain newBand
      CouplerSetNum = 0;   // something wrong if you end up here
  if (CouplerSetNum >= NUM_SETS)
      CouplerSetNum = NUM_SETS-1; // something wrong if you end up here
  NewBand = CouplerSetNum;

  #ifdef NEXTION
    nexSerial.begin(NexSerialBAUD);   // set in RF wattmeter_Arduino.h - must match the Nextion display BAUD or BAUDS parameter set on Main Page
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

void loop() 
{ 
    uint8_t ret1;

    // Listen for remote computer commands                
    serial_usb_read();  // fetch latest data in buffer                                      
    if (rx_count!=0)
        get_remote_cmd();       // scan buffer for command strings

    #ifdef OTRSP_Serial
    ret1 = OTRSP();   // set Aux output pins and change bands to match
    if (ret1)  // True if we got commands
    {
        ret1 = OTRSP_Process();  
        if (ret1)   // True if we got a valid AUX or BAND command
        { 
            if (!EEPROM.read(DIS_OTRSP_BAND_CHANGE))
            {
                // If 1 then Disable is active.   Default is 0, change bands with AUX1 changes.
                Button_B = YES;  // Process any N1MM Aux port commands on UART                  
                NewBand = AuxNum1;  // Use AUX 1 to change the wattmeter band
                DBG_Serial.print("> New Band # = ");
                DBG_Serial.println(NewBand);              
            }
            else
                DBG_Serial.println("> OTRSP sourced band changes are Disabled");
        }
    }  
    #endif // OTRSP   

#ifdef ENET     // remove this code if no ethernet usage intended
    if (EEPROM.read(ENET_ENABLE))   // only process enet if enabled.
    {
        if (!enet_ready)
            if ((millis() - enet_start_fail_time) >  600000)  // check every 10 minutes (600K ms) and attempt a restart.
                {
                  enet_start();
                  enet_start_fail_time = millis();  // reset timer for another 10 minutes
                }
        enet_read();
        if (rx_count!=0)
          get_remote_cmd();       // scan buffer for command strings
    }
#endif
    
    if (Band_Decoder() && PTT_IN_state != TX)       // Process any radio band decoder changes. 
    {                                                    // if in transmit, skip.  Don't want RFI or band change operating relays hot.    
        Button_B = YES;
        NewBand = Band_Dec_In_Byte;
    }
    
    if (Button_B == YES && PTT_IN_state != TX) {      // Select Cal Band
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
        DBG_Serial.print("> CouplerSetNum = ");
        DBG_Serial.println(CouplerSetNum);
        
#ifdef NEXTION
        strcpy(cmd,Band_Cal_Table[CouplerSetNum].BandName);
        Set1_Bandvar.setText(cmd);    // Update Nextion display for new values            
        toConfig_pop_Callback(0);  // when on the Config pages will update them
        Set1_Callback(0);   
        update_Nextion(1);  
        DBG_Serial.println(">Nextion Update");                      
#endif       
        EE_Save_State();
    }    
    
    if (CW_KEY_OUT_state != CW_KEY_OUT_state_last)
    {
        if (CW_KEY_OUT_state)
            DBG_Serial.println(">CW KEY ON");
        else
            DBG_Serial.println(">CW KEY OFF");
        CW_KEY_OUT_state_last = CW_KEY_OUT_state;
    }   

    // If PTT input changed, the flag is set and a timestamp made.  
    PTT_IN_pin = digitalRead(BAND_DEC_PTT_IN);    // We do not know if 1 or 0 is TX yet, determined in PTT_IN_handler()
    //DBG_Serial.print(">\n>PTT input IO pin hardware state = ");DBG_Serial.println(PTT_IN_pin);
    //PTT_IN_pin = ~PTT_IN_pin & 0x01;  // flip if using an inverting buffer
    if (PTT_IN_pin != PTT_IN_pin_last)
    {   // reset the timer until it stops changing.
        PTT_IN_debounce_timestamp = millis();   // record time of change.  Looking for no change for for maybe 15ms
        PTT_IN_pin_last = PTT_IN_pin;
        PTT_IN_changed = 0;
    }
    else   // state has stayed the same since last look
    {
        if (((millis() - PTT_IN_debounce_timestamp) > 3) && PTT_IN_changed == 0)
        {   // change the state of the T/R status
            DBG_Serial.print(">\n>PTT input IO pin hardware state = ");DBG_Serial.println(PTT_IN_pin);
            PTT_IN_changed = 1;   // reset changed flag
            PTT_IN_handler(PTT_IN_pin);  // Set RX or TX state corrected for configured polarity
            PTT_OUT_handler();   // Set PTT Output state corrected for polarity and set PTT ouput pins
        } 
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
    else   // skip if waiting for response from a diaplsy query to reduce traffic
        update_Nextion(0);    
#endif

  #ifdef EXT_WD
    // Reset External WD Timer
    if (millis() - wd_timestamp > 8000 && WD_reset_flag != 2)
    // if flag == 2 skip wd timer reset and force a WD timeout and power relay cycle, usually from remote command cmd=192.
    {     
        // Strobe low for 200ms  This part starts it only.
        wd_timestamp = millis();
        WD_reset_flag = 1;
        //Serial.println("Reset WD Timer");
        digitalWrite(WD1_PIN, LOW);
        digitalWrite(WD2_PIN, LOW);
    }
    if ((millis() - wd_timestamp > 200) && WD_reset_flag == 1)   // forms a non-blocking delay of 200ms to ensure a good WD timer reset
    {        
        // now release the low pulse allowing WD to count again
        digitalWrite(WD1_PIN, HIGH);
        digitalWrite(WD2_PIN, HIGH);
        //Serial.println("End Reset WD Timer");
        WD_reset_flag = 0;
    }
  #else  // reset internal WD Timer
      // if flag == 2 skip wd timer reset and force a WD timeout, usually from remote command cmd=192.
      if (WD_reset_flag != 2)
        wdt.feed(); /* uncomment to feed the watchdog */
      //DBG_Serial.println("Reset WD Timer");
  #endif
} // ------ End of Main Loop() --------------------

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
    //analogReference(DEFAULT);
    VRef = ADC_VREF;
    Volts = (counts/ADC_COUNTS) * VRef;
    //DBG_Serial.println(Volts);
    return Volts;
}

void adRead(void)   // A/D converter read function.  Normalize the AD output to 100%.
{
    float a;
    float b;
    uint16_t c;
    float tmp, retry=0;
    uint16_t i;

    // Throttle the time the CPU spends here.   Offset the timing of the data acquired is just before it is sent out
    Timer_X00ms_InterruptCnt = millis(); 
    if ((Timer_X00ms_InterruptCnt - Timer_X00ms_Last_AD) < 50)   // will use our own timestamp.
        return;   // skip out until greater than 25ms since our last visit here
    Timer_X00ms_Last_AD = Timer_X00ms_InterruptCnt;  // time stamp our visit here.  Do not want to come back too soon.  

__reread:   // jump label to reread values in case of odd result or hi SWR

// Get Reflected Power  
// ____________________ Ref Power Capture ___________________________
//   
        total_Ref -= readings_Ref[readIndex_Ref];// subtract the last reading:    
    c = 1; // read from the sensor:
    a = 0;
    for (i = 0; i < c; ++i)  
    {
        #ifdef ADS1115_ADC  
            #ifdef ADS1115_SINGLE_MODE
                RefVal = readChannel(ADC_REF);        
            #else   // continuous mode locks up without 20ms+ delay
                adc.setCompareChannels(ADC_REF); //comment line/change parameter to change channel           
                delay(20);
                RefVal = adc.getResult_V();    // for ADS1115 module        
            #endif
            //RefVal = constrain(RefVal, 0.000, 3300.0);
        #else    // internal ADC           
           uint32_t ad_counts=0;    
           ad_counts = analogRead(ADC_REF);              
           RefVal = ADC_RF_Power_CountsTo_Volts(ad_counts);         
        #endif  
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
    RefVal = b = total_Ref / NUMREADINGS;// calculate the average:        
    b -= Offset_R;   // adjust to 0V reference point
    b /= Slope_R;   // will be negative for detectors like AD8318
    b += CouplingFactor_Ref;
    b += 0.0; // fudge factor for frequency independent factors like cabling

    Ref_dBm = b;
    // 0dBm is max. = 1W fullscale on 1W scale.
    RefPwr = pow(10.0,(b-30.0)/10.0);    // convert to linear value for meter using 1KW @ 0dBm as reference 
    if (RefPwr > 999.9) 
        RefPwr = 999.9999;
        
// ____________________ Fwd Power Capture ___________________________
//
// subtract the last reading for Fwd Power
    total_Fwd -= readings_Fwd[readIndex_Fwd];
    // read from the sensor:
    c = 6;   // short term samples that feed into running average
    a = 0;

    for (int i = 0; i < c; ++i) 
    {
        #ifdef ADS1115_ADC   
            #ifdef ADS1115_SINGLE_MODE
                FwdVal = readChannel(ADC_FWD);   
            #else   // continuous mode locks up without 20ms+ delay
                adc.setCompareChannels(ADC_FWD); //comment line/change parameter to change channel
                delay(20);           
                FwdVal = adc.getResult_V();    // for ADS1115 module
            #endif
        #else
            uint32_t ad_counts=0;
            ad_counts = analogRead(ADC_FWD);  // single read               
            FwdVal = ADC_RF_Power_CountsTo_Volts(ad_counts); 
        #endif   
        a += FwdVal;
    }
    a /= c; // calculate the average then use result in a running average
    readings_Fwd[readIndex_Fwd] = a;   // get from the latest average above and track in this running average
    total_Fwd += readings_Fwd[readIndex_Fwd];    // add the reading to the total: 
    readIndex_Fwd += 1;     // advance to the next position in the array:
    if (readIndex_Fwd >= NUMREADINGS) {     // if we're at the end of the array...      
        readIndex_Fwd = 0;  // ...wrap around to the beginning:
    }     
    FwdVal = b = (float) total_Fwd / NUMREADINGS;    // calculate the average:

    // caclulate dB value for digital display section
    b -= Offset_F;   // adjusts to 0V reference point - calculated during cal
    b /= Slope_F;   // will be negative for detectors like AD8318 - calculated during cal
    b += CouplingFactor_Fwd;
    b += 0.0; // Fudge factor for frequency independent factors like cabling   
    Fwd_dBm = b;    // Now have calibrated Forward Value in dBm.   
    
    // 0dBm is max. = 1W fullscale on 1W scale for example
    FwdPwr =  pow(10.0,(b-30.0)/10.0);    // convert to linear value for meter using 1KW @ 0dBm as reference. Multiply by scale value.
    
    // When the measured range is larger than the log detector (<60dB), such as for 1KW range, can never get 0 Watts.  Filter out readings below X Watts
    if (FwdPwr < FwdPwr_Cutoff)
    {
      FwdPwr = 0.0;
      Fwd_dBm = 0.0;
      Ref_dBm = 0.0;
      RefPwr = 0.0;
    }

    if (FwdPwr > 9999.9)
        FwdPwr = 9999.9999;    
    
    if (RefPwr < FwdPwr) 
        tmp = sqrt(RefPwr/FwdPwr);
    else 
        tmp = sqrt(FwdPwr-0.1/FwdPwr);
        //tmp = .99999;   // if Ref > Fwd, then SWR is sky high, limit number range < 99

//        
// ------------------------ SWR -----------------------------------------------------------------------        
//
    SWRVal = ((1 + tmp) / (1 - tmp));  // now have SWR in range of 1.0 to infinity.  
    if ((RefPwr > FwdPwr) && (FwdPwr < FwdPwr_Cutoff)) //  0.2 for <400W range.  Remove false SWR values if no Fwd Pwr such as TX turns off and Fwd goes to 0 before Ref.
        SWRVal = 0;
    else if ((RefPwr <= 0.00001) || (FwdPwr <= FwdPwr_Cutoff))  // Adjust this number. 3 to 5 good for 1KW ranges.  0.2 for <400W range. This is because the detector range is 55dB and 1KW is 60dB.  Will never hit true zero.
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

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

#ifdef TEENSY4_OTRSP_CW_PTT
// N1MM CW and PTT and AUX message handling
// look for DTR and RTS for N1MM control of CW and PTT. 
// Can only happen on USB serial since the external USB UART has only TX and RX lines connected to CPU

// CW Key Out pin from OTRSP
void CW_KEY_OUT_OTRSP(void)
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

// PTT Input pin from OTRSP Serial port
void PTT_OTRSP(void)
{
    //Check state of USB Serial Port RTS register for N1MM PTT state
    if (OTRSP_Serial.rts())     
    {
        PTT_IN_handler(HIGH);
        PTT_OUT_Handler();      
    }
    else
    {
        PTT_IN_handler(LOW);
        PTT_OUT_Handler();       
    }
}
#endif

void PTT_IN_handler(uint8_t pin_state)   // Normally called from ISR or OTRSP PTT signal
{    
    uint8_t polarity ;

    polarity = EEPROM.read(PTT_IN_POLARITY);
    DBG_Serial.print(">Input PTT Polarity is ");
    if (polarity)
        DBG_Serial.println("HIGH");
    else
        DBG_Serial.println("LOW");
    DBG_Serial.println(">If Polarity = Hardware IO pin state then we are in TX Mode");

    if (pin_state == LOW)
    {
        if (polarity == LOW)   // 1 is Active HIGH, 0 is ACTIVE LOW.   IF a input pin is low and Polarity is 0, then we have TX.
        {
            PTT_IN_state = TX;   // Set to TX
            DBG_Serial.println(">PTT Input = TX"); 
        }
        else
        {
            PTT_IN_state = RX;   // Set to RX
            DBG_Serial.println(">PTT Input = RX"); 
        }
    }
    else
    {
        if (polarity == HIGH)   // 1 is Active HIGH, 0 is ACTIVE LOW.   IF a input pin is low and Polarity is 0, then we have TX.
        { 
            PTT_IN_state = TX;   // Set to TX
            DBG_Serial.println(">PTT Input = TX"); 
        }
        else
        {
            PTT_IN_state = RX;   // Set to RX
            DBG_Serial.println(">PTT Input = RX"); 
        }
    }    
    //if (PTT_IN_state == RX)
    //    DBG_Serial.println(">PTT OFF (RX Mode)");
    //else
    //    DBG_Serial.println(">PTT ON (TX Mode)");    
    PTT_IN_state_last = PTT_IN_state;
}

void PTT_OUT_handler(void)   // Uses polarity corrected T/R state tracked in PTT_IN_state.  Set PTT Output to follow (observing it's own Polarity setting)
{
    // Set block_PTT flag if we have invalid or specific patterns that are invalid
    // One case is radio powered on and off toggles the band decoder input lines to all 0's
    //   and all 1's until the actual band is initialized in the radio.  This could leave a Xvtr keyed up.
    Band_Decoder_Get_Input();
    if (Band_Dec_In_Byte == 0x3F || Band_Dec_In_Byte == 0x30 || Band_Dec_In_Byte == 0x00 || Band_Dec_In_Byte == 0xFF)
    {
      block_PTT = 1;
      DBG_Serial.print("PTT Blocked - TX not permitted on Band: ");DBG_Serial.println(Band_Dec_In_Byte, HEX);
      // In most cases the opto-coupler outputs will be turned on to pull a load to ground to activate it, such as a relay.
      // With that assumption will force all outpout portst to OFF state when an invalid band is received.
      // Was using a unused band as a workaround but that wasted 1 or 2 bands (HF and 6M in my test config).
      // The most common reasons for an invalid band are radio power on and off, and during the initial conifguration period.
      Band_Decode_A_Output(255);   // set all opto outputs OFF on PCB.
      Band_Decode_B_Output(255);
      Band_Decode_C_Output(255);
      DBG_Serial.print("All outputs forced to OFF");
      return;
    }
    block_PTT = 0;
    DBG_Serial.println(">All outputs returned to normal");
    if (PTT_IN_state == TX)   // We are in TX Mode
    {
        // Block PTT for invalid bands.  This is intended to cover the observation that when the K3 is turned off then back on, 
        //    combined with the opto-coupler interface box at the K3 end effects, we see all 0's sent on the input 
        //    followed by all 1's and PTT line flips to high (TX due to inversion of K3 opto box).
        //    When the K3 is turned back on, the PTT flips from 1 to 0 (back to RX), Input all 0's, then the 
        //    input updates to the actual band pattern.  
        // The bad side effect of PTT on when power is off it keys the PTT relay (amps are off hopefully so no harm) 
        //    but the Xvtr is always on and it is sent to TX mode.  If any RF flows it would send RF into the output
        //    of a LNA.  Plus the Xvtr would be stuck in TX until the K3 is back on some day.  Not good.
        // The long term answer is to remove the inversion of the PTT line at the K3 box. The PTT blocking logic added
        //    wil handle it though for now.
        // Hook up the DigOut1 signal at that time and use bit 4 (DigOut) to disable the Xvtr.
        // The Xvtr can also be disabled by lowering setting either B3 and B4 bits to opposite of normal causing an invalid band input
        
        if (EEPROM.read(PTT_OUT_POLARITY) == LOW) // Change this to LOW if NOT using an inverting driver
        {            
            digitalWrite(PTT_OUT2, LOW);    // RX is on, TX is ACTIVE LOW so send out a 0 to PTT2 OUT pin )(sequencer for Amps first)
            delay(SEQ_Delay);
            digitalWrite(PTT_OUT, LOW);     // TX is on, TX is ACTIVE LOW so send out a 0 to PTT OUT pin(now Xvtr)
            PTT_OUT_state = LOW;            // this is used when calling Port (A,B,C) update functions in case they follow PTT
            DBG_Serial.println(">In TX Mode: PTT Output Polarity is LOW");
        }
        else
        {        
            digitalWrite(PTT_OUT2, HIGH);    // RX is on, TX is ACTIVE LOW so send out a 1 to PTT OUT pin (sequencer for Amps first)
            delay(SEQ_Delay);
            digitalWrite(PTT_OUT, HIGH);    // TX is on, TX is ACTIVE LOW so send out a 1 to PTT OUT pin (Now Xvtr)
            PTT_OUT_state = HIGH;           // this is used when calling Port (A,B,C) update functions in case they follow PTT            
            DBG_Serial.println(">In TX Mode: PTT Output Polarity is HIGH");
        }
        DBG_Serial.print(">In TX Mode: PTT Output ON - IO Pin is ");  DBG_Serial.println(PTT_OUT_state); 
    }
    else   // We are in RX mode
    {
        if (EEPROM.read(PTT_OUT_POLARITY) == LOW) // Change this to LOW if NOT using an inverting driver
        {
            digitalWrite(PTT_OUT, HIGH);    // RX is on, TX is ACTIVE LOW so send out a 1 to PTT OUT pin (Sequence Xvtr release first)
            PTT_OUT_state = HIGH;           // this is used when calling Port (A,B,C) update functions in case they follow PTT
            delay(SEQ_Delay);
            digitalWrite(PTT_OUT2, HIGH);    // RX is on, TX is ACTIVE LOW so send out a 1 to PTT OUT pin (Now amps)
            DBG_Serial.println(">In RX Mode: PTT Output Polarity is LOW");
        }
        else
        {
            digitalWrite(PTT_OUT, LOW);     // RX is on, TX is ACTIVE HIGH so send out a 0 to PTT OUT pin  (Sequence Xvtr release first)
            PTT_OUT_state = LOW;            // this is used when calling Port (A,B,C) update functions in case they follow PTT_OUT_state
            delay(SEQ_Delay);
            digitalWrite(PTT_OUT2, LOW);    // RX is on, TX is ACTIVE LOW so send out a 1 to PTT OUT pin (now Amps)
            DBG_Serial.println(">In RX Mode: PTT Output Polarity is HIGH");
        }
        DBG_Serial.print(">PTT Output OFF - IO Pin is ");  DBG_Serial.println(PTT_OUT_state); 
    }
        
    // If any of the output ports (ABC, others) are configured to follow PTT then we need to update those ports now.
    Band_Decode_A_Output(CouplerSetNum); 
    Band_Decode_B_Output(AuxNum1);
    Band_Decode_C_Output(AuxNum2);
}

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
        //delay(25);
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
        if (round(FwdPwr*16) != round(FwdPwr_old*16))
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
        if (round(RefPwr*16) != round(RefPwr_old*16))
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
        
        if (round(Fwd_dBm*16) != round(Fwd_dBm_old*16))
        {   
            Fwd_dBm_old = Fwd_dBm;
            sprintf(cmd, "FdBm.txt=\"%.2fdBm\"", Fwd_dBm);
            sendCommand(cmd);
        }
        
        if (round(Ref_dBm*16) != round(Ref_dBm_old*16))
        {   
            Ref_dBm_old = Ref_dBm;
            sprintf(cmd, "RdBm.txt=\"%.2fdBm\"", Ref_dBm);
            sendCommand(cmd);        
        }
        
        if  (SWRVal != SWRVal_old)
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
        if ((round(value*16) != round(hv_value_old*16) || set_hv_max_old != set_hv_max))
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
        if ((round(value*16) != round(V14_value_old*16) || set_v14_max_old != set_v14_max))
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
        if  ((round(value*16) != round(curr_value_old*16) || set_curr_max_old != set_curr_max))
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
        if  ((round(value*16) != round(temp_value_old*16) || set_temp_max_old != set_temp_max))
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
        if ((AuxNum1 != AuxNum1_old || force_update == 1))        
        {
            AuxNum1_old = AuxNum1;
            number32 = (uint32_t) AuxNum1;
            Aux1.setValue(number32);
        }
        if ((AuxNum2 != AuxNum2_old || force_update == 1))
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
    char TxRx[5];

    if (PTT_IN_state)
       strcpy(TxRx,"TX\0");
    else
       strcpy(TxRx,"RX\0");
        
    display.clearDisplay();    
    display.drawRect( 0, 17, 127, 47, SSD1306_WHITE);
    
    // Top row is Yellow with Power in Watts, size 2 font
    display.setTextColor(SSD1306_WHITE);
    if (CouplerSetNum && FwdPwr <100)  // show decimal when < 100W
        snprintf(s, 15, "%*s %*.1fW%c", 2, TxRx, 6, FwdPwr, '\0');
    else if (CouplerSetNum)   // hide decimal when over 100W
        snprintf(s, 15, "%*s %*.0fW%c", 2, TxRx, 6, FwdPwr, '\0');
    else
        snprintf(s, 12, "  %*s%c", 6, "OFF", '\0');
    display.setTextSize(2);    
    display.setCursor(0,0);
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.println(s);

#ifdef OLED_COMBO_LAYOUT
    //sprintf(s," Band    Fwd    SWR");    
    display.setTextSize(1);    
    display.setCursor(15,20);
    display.println("Band");
    display.setCursor(62,20);
    display.println("Fwd");
    display.setCursor(98,20);
    display.println("SWR");
    //display.setCursor(100,20);
    //display.println("T/R");
       
    if (FwdPwr <= 0.1)
        sprintf(s, " %*s%*.1f%*s%c", 7, Band_Cal_Table[CouplerSetNum].BandName, 6, 0.0, 5, "", '\0'); 
    else
    {
        sprintf(s, " %*s%*.1f%*.1f%c", 7, Band_Cal_Table[CouplerSetNum].BandName, 6, Fwd_dBm, 5, SWRVal, '\0');
        //sprintf(s, " %*s%*.1f%*s%c", 7, Band_Cal_Table[CouplerSetNum].BandName, 6, Fwd_dBm, 5, TxRx, '\0');
    }
    
    if (CouplerSetNum == 0)
    {
        sprintf(s, " %*s%*d%*s%c", 3, Band_Cal_Table[CouplerSetNum].BandName, 7, 0, 7, "", '\0');   
        display.setCursor(7,31);
    }   
    else
    {
      display.setCursor(1,31); 
    }
    display.setTextSize(1); 
    display.println(s);
    
    sprintf(s,"In:%02X A:%02X B:%02X C:%02X%c", Band_Dec_In_Byte, PortA_state, PortB_state, PortC_state, '\0');
    display.setTextSize(1);
    display.setCursor(3,49);
    display.println(s);
#else    
    float value;
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
#endif       
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
          DBG_Serial.print(">RX Count = ");       
          DBG_Serial.println(rx_count);
          DBG_Serial.print(">RX Buffer = ");       
          DBG_Serial.println((char *) rx_buffer);
          
          // initially p1 = p2.  parser will move p1 up to p2 and when they are equal, buffer is empty, parser will reset p1 and p2 back to start of sData         
          memcpy(pSdata2, rx_buffer, rx_count+1);   // append the new buffer data to current end marked by pointer 2        
          pSdata2 += rx_count;                      // Update the end pointer position. The function processing chars will update the p1 and p2 pointer             
          rx_count = pSdata2 - pSdata1;             // update count for total unread chars. 
          DBG_Serial.print(">RX Count after adding to buffer ring = "); 
          //DBG_Serial.println(rx_count);  
     }
     rx_buffer[0] = '\0';
     return rx_count;
}

/*
 *   Send out dBm, Watts, and SWR values to data channel
*/
void sendSerialData()
{
    Timer_X00ms_InterruptCnt = millis();
    if (Timer_X00ms_InterruptCnt - Timer_X00ms_Last_USB_PWR > PWR_MSG_DELAY)
    {          
        Timer_X00ms_Last_USB_PWR = Timer_X00ms_InterruptCnt;       
        sprintf((char *) tx_buffer,"%d,%s,%s,%.2f,%.2f,%.1f,%.1f,%.1f\r\n%c", METERID, "170", Band_Cal_Table[CouplerSetNum].BandName, Fwd_dBm, Ref_dBm, FwdPwr, RefPwr, SWR_Serial_Val, '\0');       
        if (ser_data_out)
            RFWM_Serial.print((char*) tx_buffer);  //, tx_count); 
        #ifdef ENET
        if (enet_data_out)
            enet_write(tx_buffer, tx_count);   // mirror out to the ethernet connection
        #endif    
    }
    if (Timer_X00ms_InterruptCnt - Timer_X00ms_Last_USB_VOLTS > VOLTS_MSG_DELAY)
    {        
        Timer_X00ms_Last_USB_VOLTS = Timer_X00ms_InterruptCnt;   
        sprintf((char *) tx_buffer,"%d,%s,%.1f,%.1f,%.1f,%.1f\r\n%c", METERID, "171", (hv_read()*hv_cal_factor), (v14_read()*v14_cal_factor), ((curr_read()-curr_zero_offset)*curr_cal_factor), (temp_read()*temp_cal_factor), '\0');       
        if (ser_data_out)
            RFWM_Serial.print((char *) tx_buffer);   //, tx_count); 
        #ifdef ENET            
        if (enet_data_out)
            enet_write(tx_buffer, tx_count);   // mirror out to the ethernet connection
        #endif
    }
    if (Timer_X00ms_InterruptCnt - Timer_X00ms_Last_USB_PTT > PTT_MSG_DELAY)
    {          
        Timer_X00ms_Last_USB_PTT = Timer_X00ms_InterruptCnt;       
        sprintf((char *) tx_buffer,"%d,%s,%s,%d\r\n%c", METERID, "173", "PTT", PTT_IN_state, '\0');       
        if (ser_data_out)
            RFWM_Serial.print((char *) tx_buffer);   //, tx_count); 
        #ifdef ENET            
        if (enet_data_out)
            enet_write(tx_buffer, tx_count);   // mirror out to the ethernet connection
        #endif
    }
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
        #ifdef ENET
          enet_write(tx_buffer, tx_count);   // mirror out to the ethernet connection
        #endif        
    }
}

float hv_read(void)
{
    uint32_t ad_counts=0;
    
    // Read voltage by ADC via MUX (If connected)
    delay(0);
    ad_counts = analogRead(ADC_HV);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float v14_read(void)
{
    uint32_t ad_counts=0;
    
    // Read voltage by ADC via MUX (If connected)
    delay(0);
    ad_counts = analogRead(ADC_14V);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float curr_read(void)
{
    uint32_t ad_counts=0;
    
    // Read ADC via MUX (If connected)
    delay(0);
    ad_counts = analogRead(ADC_CURR);  // single read but has locked up with N1MM commands
    return ad_counts;
}

float temp_read(void)
{
#ifdef DETECTOR_TEMP_CONNECTED
  float tmp;
    // Read detector temperature (If connected)
    #if defined ADS1115_ADC && defined ADS1115_ADC_TEMPERATURE  // Use external ADC
        #ifdef ADS1115_SINGLE_MODE
            tmp = readChannel(ADC_TEMP);   
        #else 
            adc.setCompareChannels(ADC_TEMP); //comment line/change parameter to change channel  
            delay(20);// continuous mode locks up without 20ms+ delay           
            tmp = adc.getResult_V();    // for ADS1115 module 
        #endif 
    #else  // use internal ADC
        uint32_t ad_counts=0;
        ad_counts = analogRead(ADC_TEMP);               // single read but has locked up with N1MM commands   
        tmp = ADC_RF_Power_CountsTo_Volts(ad_counts);   // store the detector temp reading for cal optimization if desired.  For now jsut display it on the screen
    #endif
    tmp -= 1.36;                                    // ADL5519 is 4.48mV/C at 27C which is typically 1.36VDC.  Convert to F.  
    tmp /= 0.00448;                                 // mV/C
    tmp += 27;                                      //1.36V at 27C (80.6F)
    tmp *= 9;
    tmp /= 5;                                       // convert to F
    tmp += 32;    
    TempVal = tmp;  
    return TempVal;
#endif 
    return 0;   // if not temp connected just return 0;
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
            //DBG_Serial.print("> Meter ID  ");
            //DBG_Serial.println("%s\n",cmd_str);
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
                        if (cmd1 == 251) ;  // dump current cal table to remote  (Was Scale GUI button)

                        //Cover 20 bands plus generic HF (band 0) for undefined band needs.
                        if (cmd1 == 250) {     // Jump to Band 10G (3cm)
                            Button_B = YES;
                            NewBand = 20;  
                        }   
                        if (cmd1 == 249) {     // Jump to Band 5.7G (6cm)
                            Button_B = YES;
                            NewBand = 19;  
                        }
                        if (cmd1 == 248) {     // Jump to Band 3.4G (3cm)
                            Button_B = YES;
                            NewBand = 18;  
                        }
                        if (cmd1 == 247) {     // Jump to Band 2.3G (12cm)
                            Button_B = YES;
                            NewBand = 17;  
                        }
                        if (cmd1 == 246) {     // Jump to Band 1296 (23cm)
                            Button_B = YES;
                            NewBand = 16;  
                        }
                        if (cmd1 == 245) {     // Jump to Band 902 (33cm)
                            Button_B = YES;
                            NewBand = 15;  
                        }
                        if (cmd1 == 244) {     // Jump to Band 70cm
                            Button_B = YES;
                            NewBand = 14;  
                        }
                        if (cmd1 == 243) {     // Jump to Band 1.25M
                            Button_B = YES;
                            NewBand = 13;  
                        }
                        if (cmd1 == 242) {     // Jump to Band 2M
                            Button_B = YES;
                            NewBand = 12;  
                        }
                        if (cmd1 == 241) {     // Jump to Band 6M
                            Button_B = YES;
                            NewBand = 11;                          
                        }
                        if (cmd1 == 240) {     // Jump to Band 10M
                            Button_B = YES;
                            NewBand = 10;  
                        }   
                        if (cmd1 == 239) {     // Jump to Band 12M
                            Button_B = YES;
                            NewBand = 9;  
                        }
                        if (cmd1 == 238) {     // Jump to Band 15M
                            Button_B = YES;
                            NewBand = 8;  
                        }
                        if (cmd1 == 237) {     // Jump to Band 17M
                            Button_B = YES;
                            NewBand = 7;  
                        }
                        if (cmd1 == 236) {     // Jump to Band 20M
                            Button_B = YES;
                            NewBand = 6;  
                        }
                        if (cmd1 == 235) {     // Jump to Band 30M
                            Button_B = YES;
                            NewBand = 5;  
                        }
                        if (cmd1 == 234) {     // Jump to Band 40M
                            Button_B = YES;
                            NewBand = 4;  
                        }
                        if (cmd1 == 233) {     // Jump to Band 60M
                            Button_B = YES;
                            NewBand = 3;  
                        }
                        if (cmd1 == 232) {     // Jump to Band 80M
                            Button_B = YES;
                            NewBand = 2;  
                        }
                        if (cmd1 == 231) {     // Jump to Band 160M
                            Button_B = YES;
                            NewBand = 1;  
                        }
                        if (cmd1 == 230) {     // Jump to Band HF
                            Button_B = YES;
                            NewBand = 0;
                        }
                        if (cmd1 == 209) {     // Toggle Serial power data outpout.  Other serial functions remain available.
                            //DBG_Serial.println(">Call Serial Data Output Toggle");
                            toggle_ser_data_output(cmd2);
                        }
                        if (cmd1 == 192 && cmd2 == 1) {     // Trigger watchdog timeout to reset on command.                                                      
                            WD_reset_flag = 2;                                       
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
                        if (cmd1 == 94) {    // This is cal routine. cmd is target Fwd value.  uses Attenuator value adjustment to set cal.  Depricated.
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
                                    //serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }
                                else if (RefPwr > cmd2+0.1) // coarse tune range
                                {
                                    // decrement 
                                    CouplingFactor_Ref -= 0.01;
                                    adRead();  // get new values                                                                    
                                    //serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }
                                else if (RefPwr < cmd2-1.0)
                                {
                                    // increment
                                    CouplingFactor_Ref += 0.5;
                                    adRead();  // get new values                                
                                    //serial_usb_write();
                                    print_Cal_Table_progress(1);
                                }  
                                else if (RefPwr < cmd2-0.1)
                                {
                                    // increment
                                    CouplingFactor_Ref += 0.01;
                                    adRead();  // get new values                                
                                    //serial_usb_write();
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
                                temp_cal_factor = 1.0;  // a value of 0 is signal to reset to no correction factor
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
                            EEPROM_Init_Write();    // save to eeprom
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();  // read it back  
                        }  
                        if (cmd1 == 68) // Get Custom Band Decoder Output port pattern from user for current band 
                        {                            
                            Band_Cal_Table[CouplerSetNum].band_A_output_pattern = cmd2;
                            EEPROM_Init_Write();    // save to eeprom
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();  // read it back  
                        }
                        if (cmd1 == 67) // Get Custom Band Decoder Output port pattern from user for current band 
                        {                              
                            Band_Cal_Table[CouplerSetNum].band_B_output_pattern = cmd2;
                            EEPROM_Init_Write();    // save to eeprom
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();  // read it back  
                        }
                        if (cmd1 == 66) // Get Custom Band Decoder Output port pattern from user for current band 
                        {                              
                            Band_Cal_Table[CouplerSetNum].band_C_output_pattern = cmd2;
                            EEPROM_Init_Write();    // save to eeprom
                            EEPROM_Init_Read();     // load the values into memory
                            Cal_Table();  // read it back  
                        }
                        if (cmd1 == 65)  // Set or Clear Band Decoder Output translate options
                        {                          
                            EEPROM.update(TRANS_INPUT,cmd2);
                        }
                        if (cmd1 == 64)  // Set or Clear Band Decoder Input translate mode
                        {                          
                            EEPROM.update(TRANS_A,cmd2);
                        }
                        if (cmd1 == 63)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(TRANS_B,cmd2);
                        }
                        if (cmd1 == 62)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(TRANS_C,cmd2);
                        }
                        if (cmd1 == 61)  // Set Disable Band Change by OTRSP command
                        {              
                            if (cmd2 == 0)          
                              EEPROM.update(DIS_OTRSP_BAND_CHANGE, 0);    // 0 = Enable (Default)
                            if (cmd2 == 1)
                              EEPROM.update(DIS_OTRSP_BAND_CHANGE, 1);    // 1 = Disable
                        }
                        if (cmd1 == 60)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            sprintf((char *) tx_buffer,"%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n%c", METERID, "172", \
                                EEPROM.read(DIS_OTRSP_BAND_CHANGE), \
                                EEPROM.read(TRANS_INPUT), \
                                EEPROM.read(TRANS_A), \
                                EEPROM.read(TRANS_B), \
                                EEPROM.read(TRANS_C), \
                                Band_Cal_Table[CouplerSetNum].band_input_pattern, \
                                Band_Cal_Table[CouplerSetNum].band_A_output_pattern,  \
                                Band_Cal_Table[CouplerSetNum].band_B_output_pattern, \
                                Band_Cal_Table[CouplerSetNum].band_C_output_pattern, \
                                EEPROM.read(PTT_IN_POLARITY), \
                                EEPROM.read(PTT_OUT_POLARITY), \
                                EEPROM.read(CW_KEY_OUT_POLARITY), \
                                EEPROM.read(PORTA_IS_PTT), \
                                EEPROM.read(PORTB_IS_PTT), \
                                EEPROM.read(PORTC_IS_PTT), \
                                '\0');       
                            serial_usb_write();  
                        }
                        if (cmd1 == 59)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(PTT_IN_POLARITY,cmd2);
                            PTT_IN_pin_last = ~PTT_IN_pin_last;
                        }
                        if (cmd1 == 58)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(PTT_OUT_POLARITY,cmd2);
                            PTT_IN_pin_last = ~PTT_IN_pin_last;
                        }
                        if (cmd1 == 57)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(CW_KEY_OUT_POLARITY,cmd2);
                        }
                        if (cmd1 == 56)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(PORTA_IS_PTT,cmd2);
                        }
                        if (cmd1 == 55)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(PORTB_IS_PTT,cmd2);
                        }
                        if (cmd1 == 54)  // Set or Clear Band Decoder Output translate mode
                        {                          
                            EEPROM.update(PORTC_IS_PTT,cmd2);
                        }
                        if (cmd1 == 53)  // 1 is enable enet, 0 is disable
                        {   
                          #ifdef ENET                       
                            EEPROM.update(ENET_ENABLE,cmd2);
                            if (cmd2)
                                enet_start();   // start up ethernet system
                          #endif
                        } 
                        if (cmd1 == 52)  // 1 is enable enet data, 0 is disable, 2 is toggle
                        {   
                          #ifdef ENET                       
                            if (cmd2)
                                toggle_enet_data_out(cmd2);   // turn on or off the UDP power and voltage data stream                        #endif
                          #endif
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
        DBG_Serial.println((char  *) tx_buffer);
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
   
    DBG_Serial.println(">read_Cal_Table_from_EEPROM - Start");

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
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string
        set_curr_max = atof(setpoint_buf);                                      // convert ascii to float 
        
        memcpy(setpoint_buf,&eepromArray[HV_CAL_OFFSET],SETPOINT_LEN);          // byte 1B to 1E of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string        
        hv_cal_factor = atof(setpoint_buf)/100;                                     // convert ascii to float 
                
        // start row 2
        memcpy(setpoint_buf,&eepromArray[V14_CAL_OFFSET],SETPOINT_LEN);         // byte 0-3 of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string
        v14_cal_factor = atof(setpoint_buf)/100;                                    // convert ascii to float 
        
        memcpy(setpoint_buf,&eepromArray[CURR_CAL_OFFSET],SETPOINT_LEN);        // byte 4-7 of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string
        curr_cal_factor = atof(setpoint_buf)/100;                                   // convert ascii to float         
        
        memcpy(setpoint_buf,&eepromArray[TEMP_CAL_OFFSET],SETPOINT_LEN);        // byte 8 to 0B of 0-15
        DBG_Serial.println();
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string
        temp_cal_factor = atof(setpoint_buf)/1000;                                   // convert ascii to float 
        
        memcpy(setpoint_buf,&eepromArray[CURR_0_OFFSET],SETPOINT_LEN);        // byte 0C to 0F of 0-15
        setpoint_buf[SETPOINT_LEN] = '\0';                                      // null terminate string
        curr_zero_offset = atof(setpoint_buf);  
        
        // Start row 3
        // The above variables are stored in EEPROM and copied to RAM.  
        // Other variables are read and written directly from EEPROM only so do not appear here.
        //Translate options using first 5 bytes of row 3 - read and write directly to EEPROM, no variable used.
        // They are initialized in write_Cal_Table_from_Default() function.
        enet_data_out = EEPROM.read(ENET_DATA_OUT_OFFSET);    // this one we want a regular variable to read
      #ifdef ENET  
        ip_adr1      = EEPROM.read(IP_ADR1);        // byte 3D - shared subnet byte ex: (192, 168, IP_ADR1, MY_IP_ADR0)
        my_ip_adr0   = EEPROM.read(MY_IP_ADR0);     // byte 3E - My ip address - static IP address byte (192, 168, IP_ADR1, MY_IP_ADR0)
        dest_ip_adr0 = EEPROM.read(DEST_IP_ADR0);   // byte 3F - Destination IP Address - static IP (192, 168, IP_ADR1, DEST_IP_ADR0)
      #endif

  // Now get the band table struct data   
   for (i=0; i<NUM_SETS; i++) 
   {
      if (i < EEPROM.length())
      {
         len_ee = EEADDR+(sizeof(Band_Cal)*i);
         EEPROM.get(len_ee, Band_Cal_Table[i]);  
         //DBG_Serial.print(EEPROM.length());       
         //DBG_Serial.println(">EEPROM Read");
         DBG_Serial.println(Band_Cal_Table[i].BandName);
         DBG_Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         DBG_Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(1); 
      }
   }
   if (len_ee > EEPROM.length())  //EEPROM_SIZE)
   {       
     DBG_Serial.print(">ERROR! - Exceed EEPROM Storage limit - Bytes used = ");
     DBG_Serial.println(len_ee);
   }   
   DBG_Serial.print(">EEPROM Max Size is : ");
   DBG_Serial.print(EEPROM.length());    //EEPROM_SIZE);
   DBG_Serial.print(">  - EEPROM Bytes used : ");
   DBG_Serial.print(len_ee);
   DBG_Serial.print(">  - EEPROM Bytes remaining = ");
   DBG_Serial.println(EEPROM.length() - len_ee);    //EEPROM_SIZE - len_ee);
   DBG_Serial.print(">Number of table rows = ");
   DBG_Serial.print(NUM_SETS);
   DBG_Serial.print(">  - Size of each Cal_Table row = ");
   DBG_Serial.println(sizeof(Band_Cal));
   DBG_Serial.println(">read_Cal_Table_from_EEPROM - End");
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

    DBG_Serial.println("> Write Cal Table to EEPROM - Start " );
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
         DBG_Serial.print("> Row 0 BYTES  = ");
         DBG_Serial.println(tempvar4[j]);     
    }
    */
    memcpy(c1_array,Callsign, CALLSIGN_LEN);   /*  7 bytes */     
 
    sprintf(setpoint_buf, "%02.1f", set_curr_max);                           // convert float to 4 byte ascii
    memcpy(&c1_array[CURR_MAX_OFFSET-0x0010],setpoint_buf, SETPOINT_LEN);
    
    if (hv_cal_factor == 0) 
        hv_cal_factor = 1.0;  
    sprintf(setpoint_buf, "%01.2f", hv_cal_factor*100);                           // convert float to 4 byte ascii
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
         DBG_Serial.print("> Row 1 BYTES  = ");
         DBG_Serial.println(tempvar4[j]);     
    }
*/    
    // Prep and write Row 2 (0x0020 to 2F)
    if (v14_cal_factor == 0) 
        v14_cal_factor = 1.0;
    sprintf(setpoint_buf, "%02.2f", v14_cal_factor*100);                           // convert float to 4 byte ascii
    memcpy(&c1_array[V14_CAL_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN);
    
    if (curr_cal_factor == 0) 
        curr_cal_factor = 1.0;
    sprintf(setpoint_buf, "%02.2f", curr_cal_factor*100);                           // convert float to 4 byte ascii
    memcpy(&c1_array[CURR_CAL_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN); 
         
    if (temp_cal_factor == 0) 
        temp_cal_factor = 1.0;
    sprintf(setpoint_buf, "%02.2f", temp_cal_factor*1000);                           // convert float to 4 byte ascii
    memcpy(&c1_array[TEMP_CAL_OFFSET-0x0020],setpoint_buf, SETPOINT_LEN);      
      
    sprintf(setpoint_buf, "%02.2f", curr_zero_offset);   
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
         DBG_Serial.print("> Row 2 BYTES  = ");
         DBG_Serial.println(tempvar4[j]);     
    }
*/  
    // Byte in Row 3 are reserved for Translate and other lesser used options and initialized in write_Cal_Table_from_Default() function
    // They are read and written direct to EEPROM so nothing to do here for these values, same for some other EEPROM only vars
    EEPROM.update(ENET_DATA_OUT_OFFSET, enet_data_out);    // this one we want a regular variable or
    #ifdef ENET
        EEPROM.update(IP_ADR1, ip_adr1);             // byte 3D - shared subnet byte ex: (192, 168, IP_ADR1, MY_IP_ADR0)
        EEPROM.update(MY_IP_ADR0, my_ip_adr0);       // byte 3E - My ip address -  static IP address byte (192, 168, IP_ADR1, MY_IP_ADR0)
        EEPROM.update(DEST_IP_ADR0, dest_ip_adr0);   // byte 3F - Destination IP Address - static IP (192, 168, IP_ADR1, DEST_IP_ADR0)
    #endif

   // Now write the Cal Table array 
   for (i=0; i< NUM_SETS; i++) {
      if (i < EEPROM.length()) {
         len_ee = EEADDR+(sizeof(Band_Cal)*i);
         DBG_Serial.print(">len_ee = ");
         DBG_Serial.println(len_ee);
         EEPROM.put(len_ee, Band_Cal_Table[i]);       
         DBG_Serial.println(">EEPROM Written");
         DBG_Serial.println(Band_Cal_Table[i].BandName);
         DBG_Serial.println(Band_Cal_Table[i].Cpl_Fwd);
         DBG_Serial.println(Band_Cal_Table[i].Cpl_Ref);        
         delay(1);
         EEPROM.update(0,'G');
      }
      if (len_ee > EEPROM.length())    // EEPROM_SIZE)
      {       
         DBG_Serial.print(">ERROR! - Exceed EEPROM Storage limit - Bytes used = ");
         DBG_Serial.println(len_ee);
      }
   }    
   DBG_Serial.print(">EEPROM Max Size is : ");
   DBG_Serial.print(EEPROM.length());    //EEPROM_SIZE);
   DBG_Serial.print("  -  EEPROM Bytes used : ");
   DBG_Serial.print(len_ee);
   DBG_Serial.print("  -  EEPROM Bytes remaining = ");
   DBG_Serial.println(EEPROM.length() - len_ee);     //(EEPROM_SIZE - len_ee);
   DBG_Serial.println(">Write Cal Table to EEPROM - End " );
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
    enet_data_out = 1;
    hv_cal_factor=1;          
    v14_cal_factor=1;
    curr_cal_factor=1;        
    temp_cal_factor=1;
    curr_zero_offset=2.5;

    /*  for debug
    DBG_Serial.println("> Write Cal From Default");
    DBG_Serial.println(op_mode);
    DBG_Serial.println(CouplerSetNum); 
    DBG_Serial.println(ser_data_out);
    DBG_Serial.println(set_hv_max);            
    DBG_Serial.println(set_v14_max);
    DBG_Serial.println(set_temp_max);
    DBG_Serial.println(METERID);  
    DBG_Serial.println(Callsign);
    DBG_Serial.println(set_curr_max);
    DBG_Serial.println(hv_cal_factor);          
    DBG_Serial.println(v14_cal_factor);
    DBG_Serial.println(curr_cal_factor);        
    DBG_Serial.println(temp_cal_factor);
    DBG_Serial.println(curr_zero_offset);
    */
    DBG_Serial.println(">write_Cal_Table_from_Default - Completed");
}

// Read state in EEPROM
void get_config_EEPROM()
{
    CouplerSetNum = EEPROM.read(COUPLERSETNUM_OFFSET);
    //DBG_Serial.print(">CouplerSetNum Read = ");
    //DBG_Serial.println(CouplerSetNum);
    op_mode = EEPROM.read(OP_MODE_OFFSET);
    //DBG_Serial.print(">op_mode Read = ");
    //DBG_Serial.println(op_mode);
    //EEPROM.commit();
}

// Save state in EEPROM
uint8_t EE_Save_State()
{
    //char buf[3];
    DBG_Serial.println(">EE_Save_State");
    CouplerSetNum = constrain(CouplerSetNum,0,NUM_SETS);
    EEPROM.update(COUPLERSETNUM_OFFSET, CouplerSetNum);
    DBG_Serial.print(">CouplerSetNum Write =");
    DBG_Serial.println(CouplerSetNum);
    op_mode = constrain(op_mode,1,3);
    if (op_mode == 1 || op_mode == 2) {
        EEPROM.update(OP_MODE_OFFSET, op_mode);
        DBG_Serial.print(">op_mode Write = ");
        DBG_Serial.println(op_mode);
    }
    EEPROM.update(SER_DATA_OUT_OFFSET, ser_data_out);
    DBG_Serial.print(">ser_data_out Write =");
    DBG_Serial.println(ser_data_out);
    EEPROM.update(ENET_DATA_OUT_OFFSET, enet_data_out);
    DBG_Serial.print(">enet_data_out Write =");
    DBG_Serial.println(enet_data_out);
    Cal_Table();
    DBG_Serial.println(">EE_Save_State - End");
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
        // Initialize the mode of translation for each port    
        EEPROM.update(TRANS_INPUT,0); // used to change bands from radio
        EEPROM.update(TRANS_A, 0);    // Used for pass through to 5 band transverter
        EEPROM.update(TRANS_B, 0);    // Used to select antennas or amps
        EEPROM.update(TRANS_C, 0);    // Used to select antennas or amps
        EEPROM.update(DIS_OTRSP_BAND_CHANGE, 0);    // 1 = Disable, 0 = Enable (Default)
        EEPROM.update(PTT_IN_POLARITY, 0);
        EEPROM.update(PTT_OUT_POLARITY, 0);
        EEPROM.update(CW_KEY_OUT_POLARITY, 0);
        EEPROM.update(PORTA_IS_PTT, 0);
        EEPROM.update(PORTB_IS_PTT, 0);
        EEPROM.update(PORTC_IS_PTT, 0);   // 0 is NO or off.  Decoder output ports can check this to see if the pin should follow PTT        
        EEPROM.update(ENET_ENABLE, 1);   // if enet code is not compiled this is jsut ignored.  Enet is enabled by default otherwise
        EEPROM.update(ENET_DATA_OUT_OFFSET, 1);  // if enet is enabled this will toggle the power and voltage data stream output.  Does nto stop comand responses or debug/info messages
        EEPROM.update(SER_DATA_OUT_OFFSET, 1);  // same as for enet but for serial port power and voltage info only
        #ifdef ENET
            EEPROM.update(IP_ADR1, DEF_SUBNET_IP_ADR1);     // byte 3D - shared subnet byte ex: (192, 168, IP_ADR1, MY_IP_ADR0)
            EEPROM.update(MY_IP_ADR0, DEF_MY_IP_ADR0);      // byte 3E - My ipadress static IP address byte (192, 168, IP_ADR1, MY_IP_ADR0)
            EEPROM.update(DEST_IP_ADR0, DEF_DEST_IP_ADR0);  // byte 3F - Desination IP Address static IP (192, 168, IP_ADR1, DEST_IP_ADR0)
        #endif

        //EEPROM_Init(EE_SAVE_YES);
        printf("Erased Byte 0");
        EEPROM_Init_Read();     // load the values into m
        Cal_Table();
    }
}

#ifdef ENET                       
// Toggle UDP output data
void toggle_enet_data_out(uint8_t mode)
{
      if (mode == 1)
        enet_data_out = 1;
      if (mode ==0)
        enet_data_out = 0;
      if (mode ==2){
          if (enet_data_out == 0)
              enet_data_out = 1;
          else         
              enet_data_out = 0; 
      }
      if (enet_data_out == 1){
          EEPROM.update(ENET_DATA_OUT_OFFSET, 1);      
          DBG_Serial.println(">Enabled UDP Data Output");
      }
      else {
          EEPROM.update(ENET_DATA_OUT_OFFSET, 0);      
          DBG_Serial.println(">Disabled UDP Data Output");
      }
}
#endif

// Toggle USB serial output data
void toggle_ser_data_output(uint8_t mode)
{
       if (mode == 1)
        ser_data_out = 1;
     if (mode ==0)
        ser_data_out = 0;
      if (mode ==2){
          if (ser_data_out == 0)
              ser_data_out = 1;
          else         
              ser_data_out = 0; 
      }
      if (ser_data_out == 1){
          EEPROM.update(SER_DATA_OUT_OFFSET, 1);      
          DBG_Serial.println(">Enabled Serial Data Output");
      }
      else {
          EEPROM.update(SER_DATA_OUT_OFFSET, 0);      
          DBG_Serial.println(">Disabled Serial Data Output");
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


#ifdef OTRSP_Serial
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
#define BANDCMDLEN 20

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
        //DBG_Serial.print(">buf char added = ");
        DBG_Serial.print(c);   //echo input chars
        if (c=='\r')
          DBG_Serial.print('\n');
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
                DBG_Serial.print(">AuxCmd0 = ");
                DBG_Serial.println(AuxCmd0);
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
    DBG_Serial.print(">OTRSP Process string : ");
    DBG_Serial.println(AuxCmd0); 
    if ((AuxCmd0[0] == 'A' || AuxCmd0[0] == 'B') && AuxCmd0[6] == '\0' && strlen(AuxCmd0)==6) // double validate
    {      
        // Looking only for 2 commands, BAND and AUX.   
        if (strncmp(AuxCmd0,"AUX1",4) == 0)   // process AUX1 where 1 is the radio number.  Each radio has a 4 bit BCD value
        {
            AuxCmd1[0] = AuxCmd0[4];
            AuxCmd1[1] = AuxCmd0[5];
            AuxCmd1[2] = AuxCmd0[6];
            AuxCmd1[3] = '\0'; 
            //DBG_Serial.print(">OTRSP Process AUX1 string in BCD: ");
            //DBG_Serial.println(AuxCmd1); 
            //AuxNum1 = BCDToDecimal(AuxCmd1);   // Convert text 0x00-0xFF HEX to Decimal
            AuxNum1 = atoi(AuxCmd1);   // Convert text 0-255 ASCII to Decimal
            DBG_Serial.print(">OTRSP Process AUX1 string in Decimal: ");
            DBG_Serial.println(AuxNum1);
            
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
            DBG_Serial.print(">OTRSP Processing BAND 1 string : ");
            DBG_Serial.println(BndCmd1); 
            return(0);  // TODO = passing band MHZ to a CouplerNUM  Search Band values
        }
        else if (strncmp(AuxCmd0,"BAND2",5) == 0)   // process AUX comand for radio 2.
        {
            sprintf(BndCmd2,"%s", &AuxCmd0[5]);
            AuxCmd0[0] = '\0' ;
            DBG_Serial.print(">OTRSP Processing BAND 2 string : ");
            DBG_Serial.println(BndCmd2); 
            return(0); 
        }
    }  
    return 0;   // nothing processed 0 is a valid band number so using 255.
}
#endif // OTRSP

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
    static uint8_t decoder_input_last = 200;        // Force process to set proper state at startup
    uint8_t tempval = 0;
    uint8_t i;
    uint8_t trans_in;

    // if in transmit, skip.  Don't want RFI or band change operating relays hot.
    if (PTT_IN_state == TX)
        return 0;
    
    trans_in = EEPROM.read(TRANS_INPUT);

    Band_Decoder_Get_Input();   // Read the state of all input pins and store into variable Band_Dec_In_Byte  

    if (Band_Dec_In_Byte != decoder_input_last)
    {      
        DBG_Serial.print(">\n>Band_Dec_In_Byte = 0x"); DBG_Serial.print(Band_Dec_In_Byte, HEX);
        DBG_Serial.print("  b"); DBG_Serial.print(Band_Dec_In_Byte, BIN);
        DBG_Serial.print("  d"); DBG_Serial.println(Band_Dec_In_Byte, DEC);
        decoder_input_last = Band_Dec_In_Byte;

        block_PTT = 0;  // reset for normal bands
        // Read the translation scheme flag in the band record (.translate) and convert to various formats
        if (trans_in == 1)   
        {    
            // This conversion scheme will look for the first high input and give a count for a one pin per band scheme
            for (i = 0; i < 8; i++)
            {
                if (bitRead(Band_Dec_In_Byte, i))
                    tempval = i+1;     // count the highest bit set to a 1 for of 8 priority decoding.  Could also do this with custom pattern
            }
            Band_Dec_In_Byte = tempval;
            DBG_Serial.print("> 1 of 8 Decode Match Found = 0x");
            DBG_Serial.println(Band_Dec_In_Byte, HEX);
            NewBand = Band_Dec_In_Byte;
            return 1;  
        }
        else if (trans_in == 2)
        {
            // Use a custom pattern to look for anything you want to represent a band choice.
            // Advantage of this is you can use the lower 3 bits for band and the upper for antenna if needed.
            for (i = 0; i < NUM_SETS; i++)
            {
                if (Band_Cal_Table[i].band_input_pattern == Band_Dec_In_Byte)  // search the table rows for a matching value
                {
                    NewBand = i;
                    Band_Dec_In_Byte = i;
                    DBG_Serial.print("> Custom Pattern Match Found : ");
                    DBG_Serial.println(Band_Cal_Table[i].BandName);
                    return 1;  
                }
            }
        }
        else
        {
            // if translate value not 1 or 2, then pass through the input pattern
            NewBand = Band_Dec_In_Byte;
            DBG_Serial.print("> Band Input Received, New Band = 0x");
            DBG_Serial.println(Band_Dec_In_Byte, HEX);
            return 1;
        }       
        DBG_Serial.print("> ERROR: No Matching Band or Invalid Band Found, Input Received = 0x");
        DBG_Serial.println(Band_Dec_In_Byte, HEX);
        block_PTT = 1;  // invalid band so prevent TX
        DBG_Serial.print("PTT Blocked by Invalid Band: ");DBG_Serial.println(Band_Dec_In_Byte, HEX);
        // In most cases the opto-coupler outputs will be turned on to pull a load to ground to activate it, such as a relay.
        // With that assumption will force all outpout portst to OFF state when an invalid band is received.
        // Was using a unused band as a workaround but that wasted 1 or 2 bands (HF and 6M in my test config).
        // The most common reasons for an invalid band are radio power on and off, and during the initial conifguration period.
        Band_Decode_A_Output(255);   // set all opto outputs OFF on PCB.
        Band_Decode_B_Output(255);
        Band_Decode_C_Output(255);
        DBG_Serial.println("> All outputs forced to OFF\n");
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

// Read the group of pins that together are a pattern stored in 1 byte for the Band in group
void Band_Decoder_Get_Input()
{    
    unsigned long Band_Dec_loop_timestamp;
    uint8_t Band_Dec_In_Byte_changed_flag;    
    uint8_t Band_Dec_In_Byte_last;
    uint8_t Band_Dec_loop_count;
    uint8_t Band_Dec_In_Byte_temp;
    
    Band_Dec_In_Byte_last = Band_Dec_In_Byte;
    Band_Dec_In_Byte_changed_flag = 0;
    Band_Dec_loop_timestamp = 0;
    Band_Dec_loop_count = 0;
    Band_Dec_In_Byte_temp = 0;
    do
    {    
        // read our input pins      
        bitWrite(Band_Dec_In_Byte_temp, 0, digitalRead(BAND_DEC_IN_0));  // read in and store the state of all band decoder input pins
        bitWrite(Band_Dec_In_Byte_temp, 1, digitalRead(BAND_DEC_IN_1));
        bitWrite(Band_Dec_In_Byte_temp, 2, digitalRead(BAND_DEC_IN_2));
        bitWrite(Band_Dec_In_Byte_temp, 3, digitalRead(BAND_DEC_IN_3));
        bitWrite(Band_Dec_In_Byte_temp, 4, digitalRead(BAND_DEC_IN_4));  
        bitWrite(Band_Dec_In_Byte_temp, 5, digitalRead(BAND_DEC_IN_5));   
        //Band_Dec_In_Byte_temp = ~Band_Dec_In_Byte_temp & 0x3F;   // Uncomment if lines are inverted.  They are not for the PCB.
        Band_Dec_In_Byte_temp = Band_Dec_In_Byte_temp & 0x3F;   // Use this line for no inverted inputs.  Mask to the 5 actual bits.
        //DBG_Serial.print("> Band_Dec_In_Byte_temp = 0x");   // for debug
        //DBG_Serial.println(Band_Dec_In_Byte_temp, HEX);
        
        // debounce
        if (Band_Dec_In_Byte_changed_flag == 0 && Band_Dec_In_Byte_temp != Band_Dec_In_Byte_last) // input has changed?
        {   // yes it has
            Band_Dec_In_Byte_changed_flag = 1;   // set changed flag
            Band_Dec_loop_timestamp = millis();    // mark start of event
            //Band_Dec_In_Byte_last = Band_Dec_In_Byte; 
        }

        if (Band_Dec_In_Byte_changed_flag == 0)      // nothing happened, exit
            return;                              
        delay(1);  // has not changed so wait and check for stable input
        Band_Dec_loop_count++;  // this is an escape plan if input never stops changing              
        
    } while(((millis()-Band_Dec_loop_timestamp) < 30) || (Band_Dec_loop_count < 50));   // wait a max of XXms for input to settle else exit
    Band_Dec_In_Byte = Band_Dec_In_Byte_temp;     // only get here if it changed so set our new value
}

// Group of pins that together are a pattern stored in 1 byte for Band Out A group
void Band_Decode_A_Output(uint8_t pattern)
{
    uint8_t trans_a;
    
    trans_a = EEPROM.read(TRANS_A);
    
    if (block_PTT)
        pattern = 255;
    else if ( trans_a == 0)  // bit 0 and 1 is for Group A translation (or not).
        {}  // do nothing, pass straight through. Normally will be last OTRSP AuxNum1 value
    else if (trans_a == 1)  
        pattern = bit(pattern)>>1;    // Translate to 1 of 8 pins at a time only such as for amp or transverter selection
    else if (trans_a == 2)  // Ignore OTRSP, follow Current  Band
        pattern = Band_Cal_Table[CouplerSetNum].band_A_output_pattern;  // Use stored pattern for this band 
    else if (trans_a == 3)        
        pattern = Band_Cal_Table[AuxNum1].band_A_output_pattern;  // Use OTRSP Aux1 as index to Cal table for this band 
    else if (trans_a == 4)        
        pattern = AuxNum1;  // Use OTRSP Aux1 number for this band 
    else 
    {
        DBG_Serial.print(">No Valid Matching Rule for Band A Output translation = 0b");   
        DBG_Serial.println(trans_a, BIN);
        return;
    }
    // Toggle Toggle the state of the pins to follow PTT RX and TX.  High pins in variable 'pattern' are raise high or low depending on Mode setting
    // If PortX is set to Active HIGH then high pins in pattern var are toggled to follow PTT.  In RX all pins will be LOW.  (aka high side drive, +V to operate)
    // If PortX is set to Active LOW then high pins in pattern var are toggled to follow PTT and are set to LOW.  In RX all pins or HIGH (aka low side drive, Ground to operate)
    if (EEPROM.read(PORTA_IS_PTT) > 0)   // Look to see if this port is operating as PTT. 
    {                                    // Each port has it's own polarity.  Mode 1 is TX = ACTIVE LOW, Mode 2 is TX = ACTIVE HIGH. Mode 0 is Ignore (not in PTT mode)
        DBG_Serial.print("> Port_A pattern var before PTT Mode conversion = ");
        DBG_Serial.println(pattern, BIN);
        if (EEPROM.read(PORTA_IS_PTT) == 1) // All pins at 1 in RX, high pins in pattern go LOW in TX only
        {            
            if (PTT_IN_state == RX)    // PTT_IN_state is RX or TX virtual state set by the Band Decode PTT Input pins or OTRSP
                pattern = 0xFF;    // Mode 1 is all pins high in RX.  For TX lower the high pins marked in variable pattern
            else 
                pattern = ~pattern & 0xFF;
        }
        else if (EEPROM.read(PORTA_IS_PTT) == 2)   // All pins at 0 in RX, high pins in pattern go HIGH in TX only
        {
            if (PTT_IN_state == RX)
                pattern = 0;          // PTT_IN_state is the state of RX or TX.                
        }
    }
    DBG_Serial.print(">Port_A pattern = 0x"); DBG_Serial.print(pattern, HEX); 
    DBG_Serial.print("  b");DBG_Serial.print(pattern, BIN); 
    DBG_Serial.print("  d");DBG_Serial.println(pattern, DEC);
    PortA_state = pattern;
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
//-----------------------------------------------------------------------------------------------------------------
// Normally this will be used only for OTRSP AUX1 control
void Band_Decode_B_Output(uint8_t pattern)
{
    uint8_t trans_b;
    
    trans_b = EEPROM.read(TRANS_B);

    if (block_PTT)
        pattern = 255;
    else if (trans_b == 0)  // bit 0 and 1 is for Group B translation (or not).
        {}  // do nothing, pass straight through
    else if (trans_b == 1)  
        pattern = bit(pattern)>>1;    // Translate to 1 pin at a time only such as for amp or transverter selection (1-of-8)
    else if (trans_b == 2)
        pattern = Band_Cal_Table[CouplerSetNum].band_B_output_pattern;  // Use stored pattern for this band, Ignore AuxNum1 (effectively ignore OTRSP)
    else if (trans_b == 3)        
        pattern = Band_Cal_Table[AuxNum1].band_B_output_pattern;  // Use OTRSP Aux1 as index to Cal table for this band 
    else if (trans_b == 4)        
        pattern = AuxNum1;  // Use OTRSP Aux1 number for this band 
    else
    {
        DBG_Serial.print(">No Valid Matching Rule for Band B Output translation = 0b");   
        DBG_Serial.println(trans_b, BIN);
        return;
    }

    // Toggle Toggle the state of the pins to follow PTT RX and TX.  High pins in variable 'pattern' are raise high or low depending on Mode setting
    // If PortX is set to Active HIGH then high pins in pattern var are toggled to follow PTT.  In RX all pins will be LOW.  (aka high side drive, +V to operate)
    // If PortX is set to Active LOW then high pins in pattern var are toggled to follow PTT and are set to LOW.  In RX all pins or HIGH (aka low side drive, Ground to operate)
    if (EEPROM.read(PORTB_IS_PTT) > 0)   // Look to see if this port is operating as PTT. 
    {                                    // Each port has it's own polarity.  Mode 1 is TX = ACTIVE LOW, Mode 2 is TX = ACTIVE HIGH. Mode 0 is Ignore (not in PTT mode)
        DBG_Serial.print("> Port_B pattern var before PTT Mode conversion = ");
        DBG_Serial.println(pattern, BIN);
        if (EEPROM.read(PORTB_IS_PTT) == 1) // All pins at 1 in RX, high pins in pattern go LOW in TX only
        {            
            if (PTT_IN_state == RX)    // PTT_IN_state is RX or TX virtual state set by the Band Decode PTT Input pins or OTRSP
                pattern = 0xFF;    // Mode 1 is all pins high in RX.  For TX lower the high pins marked in variable pattern
            else 
                pattern = ~pattern & 0xFF;
        }
        else if (EEPROM.read(PORTB_IS_PTT) == 2)   // All pins at 0 in RX, high pins in pattern go HIGH in TX only
        {
            if (PTT_IN_state == RX)
                pattern = 0;          // PTT_IN_state is the state of RX or TX.                
        }
    }     
    DBG_Serial.print(">Port_B pattern = 0x"); DBG_Serial.print(pattern, HEX); 
    DBG_Serial.print("  b");DBG_Serial.print(pattern, BIN);
    DBG_Serial.print("  d");DBG_Serial.println(pattern, DEC);
    PortB_state = pattern;
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
    uint8_t trans_c;

    trans_c = EEPROM.read(TRANS_C);

    if (block_PTT)
        pattern = 255;
    else if (trans_c == 0)  // bit 0 and 1 is for Group C translation (or not).
        {}  // do nothing, pass straight through
    else if (trans_c == 1)  
        pattern = bit(pattern)>>1;    // Translate to 1 of 8 pins at a time only such as for amp or transverter selection
    else if (trans_c == 2)
        pattern = Band_Cal_Table[CouplerSetNum].band_C_output_pattern;  // Use custom pattern for this band, Ignore AuxNum1 (effectively ignore OTRSP)     
    else if (trans_c == 3) 
        pattern = Band_Cal_Table[AuxNum2].band_C_output_pattern;  // Use OTRSP Aux2 as index to Cal_Table for this band 
    else if (trans_c == 4) 
        pattern = AuxNum2;  // Use OTRSP Aux2 number for this band
    else
    {
        DBG_Serial.print(">No Valid Matching Rule for Band C Output translation = 0b");   
        DBG_Serial.println(trans_c, BIN);
        return;
    }

    // Toggle Toggle the state of the pins to follow PTT RX and TX.  High pins in variable 'pattern' are raise high or low depending on Mode setting
    // If PortX is set to Active HIGH then high pins in pattern var are toggled to follow PTT.  In RX all pins will be LOW.  (aka high side drive, +V to operate)
    // If PortX is set to Active LOW then high pins in pattern var are toggled to follow PTT and are set to LOW.  In RX all pins or HIGH (aka low side drive, Ground to operate)
    if (EEPROM.read(PORTC_IS_PTT) > 0 && trans_c < 3)   // Look to see if this port is operating as PTT. 
    {                                    // Each port has it's own polarity.  Mode 1 is TX = ACTIVE LOW, Mode 2 is TX = ACTIVE HIGH. Mode 0 is Ignore (not in PTT mode)
        DBG_Serial.print("> Port C pattern var before PTT Mode conversion = ");
        DBG_Serial.println(pattern, BIN);
        if (EEPROM.read(PORTC_IS_PTT) == 1) // All pins at 1 in RX, high pins in pattern go LOW in TX only
        {            
            if (PTT_IN_state == RX)    // PTT_IN_state is RX or TX virtual state set by the Band Decode PTT Input pin or OTRSP
                pattern = 0xFF;    // Mode 1 is all pins high in RX.  For TX lower the high pins marked in variable pattern
            else 
                pattern = ~pattern & 0xFF;
        }
        else if (EEPROM.read(PORTC_IS_PTT) == 2)   // All pins at 0 in RX, high pins in pattern go HIGH in TX only
        {
            if (PTT_IN_state == RX)
                pattern = 0;          // PTT_IN_state is the state of RX or TX.                
        }
    }   
    DBG_Serial.print(">Port_C pattern = 0x"); DBG_Serial.print(pattern, HEX);
    DBG_Serial.print("  b");DBG_Serial.print(pattern, BIN);
    DBG_Serial.print("  d");DBG_Serial.println(pattern, DEC);DBG_Serial.println("");
    PortC_state = pattern;
    digitalWrite(BAND_DEC_C_0, bitRead(pattern, 0));
    digitalWrite(BAND_DEC_C_1, bitRead(pattern, 1));
    digitalWrite(BAND_DEC_C_2, bitRead(pattern, 2));
    //digitalWrite(BAND_DEC_C_3, bitRead(pattern, 3));
    //digitalWrite(BAND_DEC_C_4, bitRead(pattern, 4));
    //digitalWrite(BAND_DEC_C_5, bitRead(pattern, 5));
    //digitalWrite(BAND_DEC_C_6, bitRead(pattern, 6));
    //digitalWrite(BAND_DEC_C_7, bitRead(pattern, 7)); 
}

#ifdef ENET
void teensyMAC(uint8_t *mac) 
{
  static char teensyMac[23];
  
  #if defined (HW_OCOTP_MAC1) && defined(HW_OCOTP_MAC0)
    Serial.println("using HW_OCOTP_MAC* - see https://forum.pjrc.com/threads/57595-Serial-amp-MAC-Address-Teensy-4-0");
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;

    #define MAC_OK

  #else
    
    mac[0] = 0x04;
    mac[1] = 0xE9;
    mac[2] = 0xE5;

    uint32_t SN=0;
    __disable_irq();
    
    #if defined(HAS_KINETIS_FLASH_FTFA) || defined(HAS_KINETIS_FLASH_FTFL)
      Serial.println("using FTFL_FSTAT_FTFA - vis teensyID.h - see https://github.com/sstaub/TeensyID/blob/master/TeensyID.h");
      
      FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
      FTFL_FCCOB0 = 0x41;
      FTFL_FCCOB1 = 15;
      FTFL_FSTAT = FTFL_FSTAT_CCIF;
      while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
      SN = *(uint32_t *)&FTFL_FCCOB7;

      #define MAC_OK
      
    #elif defined(HAS_KINETIS_FLASH_FTFE)
      Serial.println("using FTFL_FSTAT_FTFE - vis teensyID.h - see https://github.com/sstaub/TeensyID/blob/master/TeensyID.h");
      
      kinetis_hsrun_disable();
      FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
      *(uint32_t *)&FTFL_FCCOB3 = 0x41070000;
      FTFL_FSTAT = FTFL_FSTAT_CCIF;
      while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
      SN = *(uint32_t *)&FTFL_FCCOBB;
      kinetis_hsrun_enable();

      #define MAC_OK
      
    #endif
    
    __enable_irq();

    for(uint8_t by=0; by<3; by++) mac[by+3]=(SN >> ((2-by)*8)) & 0xFF;

  #endif

  #ifdef MAC_OK
    sprintf(teensyMac, "MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(teensyMac);
  #else
    Serial.println("ERROR: could not get MAC");
  #endif
}

uint8_t enet_read(void)
{
  //  experiment with this -->   udp.listen( true );           // and wait for incoming message

     if (!enet_ready)   // skip if no enet connection
         return 0;
     
     rx_count = 0;
     int count = 0; 

     // if there's data available, read a packet
     count = Udp.parsePacket();      
     rx_buffer[0] = _NULL;
     if (count > 0)
     {
          Udp.read(rx_buffer, Serial_USB_BUFFER_SIZE);
          rx_buffer[count] = '\0';
          rx_count = count;          
          DBG_Serial.println(rx_count);
          DBG_Serial.println((char *) rx_buffer);
          
          // initially p1 = p2.  parser will move p1 up to p2 and when they are equal, buffer is empty, parser will reset p1 and p2 back to start of sData         
          memcpy(pSdata2, rx_buffer, rx_count+1);   // append the new buffer data to current end marked by pointer 2        
          pSdata2 += rx_count;                      // Update the end pointer position. The function processing chars will update the p1 and p2 pointer             
          rx_count = pSdata2 - pSdata1;             // update count for total unread chars. 
          //DBG_Serial.println(rx_count);  
     }
     rx_buffer[0] = '\0';
     return rx_count;
}

uint8_t enet_write(uint8_t *tx_buffer, uint8_t tx_count)
{   
    IPAddress remote_ip(192, 168, ip_adr1, dest_ip_adr0); // use EEPROM stored values

    if (enet_ready & EEPROM.read(ENET_ENABLE))   // skip if no enet connection
    {
        //DBG_Serial.print("ENET Write: ");
        //DBG_Serial.println((char *) tx_buffer);
        Udp.beginPacket(HostIP, remoteport);
        Udp.write((char *) tx_buffer);
        if (Udp.endPacket())
        {
          return 1;
        }
        else
          DBG_Serial.println("ENET Write Failed");
    }
    return 0;
} 

void enet_start(void)
{
    uint8_t mac[6];

    // Ensure EEPROM is read first
    IPAddress ip(192, 168, ip_adr1, my_ip_adr0); // use EEPROM stored values

    teensyMAC(mac);   
    delay(1000);
    // start the Ethernet  
    // If using DHCP (leave off the ip arg) works but more difficult to configure the desktop and remote touchscreen clients
    Ethernet.begin(mac, ip);
    // Check for Ethernet hardware present
    enet_ready = 0;
    if (Ethernet.hardwareStatus() == EthernetNoHardware) 
    {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        //while (true) {
        //  delay(1); // do nothing, no point running without Ethernet hardware
        //}
        enet_ready = 0;  // shut down usage of enet
    }
    else
    {
        delay(1000);
        Serial.print("Ethernet Address = ");
        Serial.println(Ethernet.localIP());
        delay(5000);
        if (Ethernet.linkStatus() == LinkOFF) 
        {
        Serial.println("Ethernet cable is not connected.");
        enet_ready = 0;
        }
        else
        {  
        enet_ready = 1;
        delay(100);
        Serial.println("Ethernet cable connected.");
        // start UDP
        Udp.begin(localPort);
        #ifdef Nex_UDP
            //Udp_Nex.begin(localPort+2);
        #endif
        }
    }
}

/*  Mod required for NativeEthernet.cpp file in Ethernet.begin class.  
 *   At end of the function is a statement that hangs if no ethernet cable is connected.  
 *   
 *   while(!link_status){
 *      return;
 *   }
 *   
 *   You can never progress to use the link status function to query if a cable is connected and the program is halted.
 *    
 *   Add the below to let it escape.  Use the enet_ready flag to signal if enet started OK or not.  
 *   
    uint16_t escape_counter = 0;
    while(!link_status && escape_counter < 200){
        escape_counter++;
        Serial.println("Waiting for Link Status");
        delay(10);
        return;
    }  
 *
 */
#endif

#if defined(ICOM_CIV) || defined(ICOM_CIV_OUT)

void FrequencyRequest(){
  #if defined(REQUEST)
  if(REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])){

    #if defined(ICOM_CIV)
      txCIV(3, 0, CIV_ADRESS);  // ([command], [freq]) 3=read
    #endif

    RequestTimeout[0]=millis();
  }
  #endif
}

//-------------------------------------------------------------------------------------------------------

unsigned char hexToDecBy4bit(unsigned char hex)
// convert a character representation of a hexidecimal digit into the actual hexidecimal value
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}
//-------------------------------------------------------------------------------------------------------
float volt(int raw, float divider) {
  // float voltage = (raw * 5.0) / 1024.0 * ResistorCoeficient;
  float voltage = float(raw) * ArefVoltage * divider / 1023.0;
  #if defined(SERIAL_debug)
    Serial.print(F("Voltage "));
    Serial.println(voltage);
  #endif
  return voltage;
}

//-------------------------------------------------------------------------------------------------------

void BandDecoderOutput()
{
  //=====[ Output Icom CIV ]=======================
  #if defined(ICOM_CIV_OUT)
      if(freq!= freqPrev1){                    // if change
          txCIV(0, freq, CIV_ADR_OUT);         // 0 - set freq
          freqPrev1 = freq;
      }
  #endif
}
//---------------------------------------------------------------------------------------------------------


void ICOM_Decoder(void)
{
  //----------------------------------- Icom ACC
  #if defined(ICOM_ACC)
    AccVoltage = volt(analogRead(ADPin), 1.0);
    if (counter == 5) {
        // AccVoltage = float(AccVoltage) * ArefVoltage * Divider / 1023.0;

        //=====[ Icom ACC voltage range ]===========================================================
        if (AccVoltage > 0.73 && AccVoltage < 1.00 ) {BAND=10;}  //   6m   * * * * * * * * * * * * * * * *
        if (AccVoltage > 1.00 && AccVoltage < 1.09 ) {BAND=9;}   //  10m   *           Need              *
        if (AccVoltage > 1.09 && AccVoltage < 1.32 ) {BAND=8;}   //  12m   *    calibrated to your       *
        if (AccVoltage > 1.32 && AccVoltage < 1.55 ) {BAND=7;}   //  15m   *         own ICOM            *
        if (AccVoltage > 1.55 && AccVoltage < 1.77 ) {BAND=6;}   //  17m   *     ----------------        *
        if (AccVoltage > 1.77 && AccVoltage < 2.24 ) {BAND=5;}   //  20m   *    (These values have       *
        if (AccVoltage > 0.10 && AccVoltage < 0.50 ) {BAND=4;}   //  30m   *   been measured by any)     *
        if (AccVoltage > 2.24 && AccVoltage < 2.73 ) {BAND=3;}   //  40m   *          ic-746             *
        if (AccVoltage > 2.73 && AccVoltage < 2.99 ) {BAND=2;}   //  80m   *                             *
        if (AccVoltage > 2.99 && AccVoltage < 4.00 ) {BAND=1;}   // 160m   * * * * * * * * * * * * * * * *
        if (AccVoltage > 0.00 && AccVoltage < 0.10 ) {BAND=0;}   // parking

        //==========================================================================================

        //bandSET();                                // set outputs
        delay (20);
    }else{
        if (abs(prevAccVoltage-AccVoltage)>10) {            // average
            //means change or spurious number
            prevAccVoltage=AccVoltage;
        }else {
            counter++;
            prevAccVoltage=AccVoltage;
        }
    }
    #if defined(SERIAL_echo)
        serialEcho();
        Serial.print(AccVoltage);
        Serial.println(F(" V"));
        Serial.flush();
    #endif

    delay(500);                                   // refresh time
  #endif

  //----------------------------------- Icom CIV
  #if defined(ICOM_CIV)
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        #if defined(DEBUG)
          Serial.print(incomingByte);
          Serial.print(F("|"));
          Serial.println(incomingByte, HEX);
        #endif
        icomSM(incomingByte);
        rdIS="";
        // if(rdI[10]==0xFD){    // state machine end
        if(StateMachineEnd == true){    // state machine end
          StateMachineEnd = false;
          for (int i=9; i>=5; i-- ){
              if (rdI[i] < 10) {            // leading zero
                  rdIS = rdIS + 0;
              }
              rdIS = rdIS + String(rdI[i], HEX);  // append BCD digit from HEX variable to string
          }
          freq = rdIS.toInt();
          // Serial.println(freq);
          // Serial.println("-------");
          FreqToBandRules();
          //bandSET();

          #if defined(SERIAL_echo)
              serialEcho();
          #endif
          RequestTimeout[0]=millis();
        }
    }
  #endif
}

void CIV_BandDecoderOutput(){
  //=====[ Output Icom CIV ]=======================
  #if defined(ICOM_CIV_OUT)
    if(freq!= freqPrev1)
    {                    // if change
        txCIV(0, freq, CIV_ADR_OUT);         // 0 - set freq
        freqPrev1 = freq;
    }
  #endif
} 

/*
FE|FE|0|56|0|70|99|99|52|0|FD
FE|FE|0|56|0|30| 0| 0|53|0|FD
*/
void icomSM(byte b){      // state machine
    // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 address used by software
    // Serial.print(b, HEX);
    // Serial.print(" | ");
    // Serial.println(state);
    switch (state) {
        case 1: if( b == 0xFE ){ state = 2; rdI[0]=b; rdI[10]=0x00; }; break;
        case 2: if( b == 0xFE ){ state = 3; rdI[1]=b; }else{ state = 1;}; break;
        // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
        case 3: if( b == 0x00 || b == 0xE0 || b == 0x0E || b == 0xF1 ){ state = 4; rdI[2]=b;                       // choose command $03
        }else if( b == CIV_ADRESS ){ state = 6; rdI[2]=b;
                }else if( b == 0xFE ){ state = 3; rdI[1]=b;      // FE (3x reduce to 2x)
                }else{ state = 1;}; break;                       // or $05
  
        case 4: if( b == CIV_ADRESS ){ state = 5; rdI[3]=b; }else{ state = 1;}; break;                      // select command $03
        case 5: if( b == 0x00 || b == 0x03 ){state = 8; rdI[4]=b;  // freq
                }else if( b == 0x04 ){state = 14; rdI[4]=b;        // mode
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;        // FE
                }else{ state = 1;}; break;
  
        case 6: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 7; rdI[3]=b; }else{ state = 1;}; break;  // select command $05
        case 7: if( b == 0x00 || b == 0x05 ){ state = 8; rdI[4]=b; }else{ state = 1;}; break;
  
        case 8: if( b <= 0x99 ){state = 9; rdI[5]=b;             // 10Hz 1Hz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
        case 9: if( b <= 0x99 ){state = 10; rdI[6]=b;            // 1kHz 100Hz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 10: if( b <= 0x99 ){state = 11; rdI[7]=b;            // 100kHz 10kHz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 11: if( b <= 0x52 ){state = 12; rdI[8]=b;            // 10MHz 1Mhz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 12: if( b <= 0x01 || b == 0x04){state = 13; rdI[9]=b; // 1GHz 100MHz  <-- 1xx/4xx MHz limit
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 13: if( b == 0xFD ){state = 1; rdI[10]=b; StateMachineEnd = true;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1; rdI[10] = 0x00;}; break;
  
       case 14: if( b <= 0x12 ){state = 15; rdI[5]=b;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;   // Mode
       case 15: if( b <= 0x03 ){state = 16; rdI[6]=b;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;   // Filter
       case 16: if( b == 0xFD ){state = 1; rdI[7]=b;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1; rdI[7] = 0;}; break;
    }
}

//---------------------------------------------------------------------------------------------------------
bool CheckPartFreqInRange(byte FRQ, int PART){
  bool result=false;
  for (int i=0; i<16; i++){
    if( PartFreqToByte(Freq2Band[i][0],PART)<FRQ && FRQ<PartFreqToByte(Freq2Band[i][1],PART)){
      result=true;
    }
  }
  return result;
}
//---------------------------------------------------------------------------------------------------------
byte PartFreqToByte(long FREQ, int PART)
{
  String StrFreq=String(FREQ);
  StrFreq.reserve(10);
  while (StrFreq.length() < 10){                 // leding zeros
    StrFreq = 0 + StrFreq;
  }
  StrFreq=StrFreq.substring(PART*2,PART*2+2);
  byte PartFreq = StrFreq.toInt();
  return PartFreq;
}
//---------------------------------------------------------------------------------------------------------

int txCIV(int commandCIV, long dataCIVtx, int toAddress) 
{
    //Serial.flush();
    Serial.write(254);                                    // FE
    Serial.write(254);                                    // FE
    Serial.write(toAddress);                              // to adress
    Serial.write(fromAdress);                             // from OE
    Serial.write(commandCIV);                             // data
    if (dataCIVtx != 0){
        String freqCIVtx = String(dataCIVtx);             // to string
        freqCIVtx.reserve(11);
        String freqCIVtxPart;
        freqCIVtxPart.reserve(11);
        while (freqCIVtx.length() < 10) {                 // leding zeros
            freqCIVtx = 0 + freqCIVtx;
        }
        for (int x=8; x>=0; x=x-2){                       // loop for 5x2 char [xx xx xx xx xx]
            freqCIVtxPart = freqCIVtx.substring(x,x+2);   // cut freq to five part
                Serial.write(hexToDec(freqCIVtxPart));    // HEX to DEC, because write as DEC format from HEX variable
        }
    }
    Serial.write(253);                                    // FD
    // Serial.flush();
    while(Serial.available()){        // clear buffer
      Serial.read();
    }
}
  
//---------------------------------------------------------------------------------------------------------

unsigned int hexToDec(String hexString) 
{
    hexString.reserve(2);
    unsigned int decValue = 0;
    int nextInt;
    for (int i = 0; i < hexString.length(); i++) {
        nextInt = int(hexString.charAt(i));
        if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
        if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
        if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
        nextInt = constrain(nextInt, 0, 15);
        decValue = (decValue * 16) + nextInt;
    }
    return decValue;
}

//---------------------------------------------------------------------------------------------------------

void FreqToBandRules(){
         if (freq >=Freq2Band[0][0] && freq <=Freq2Band[0][1] )  {BAND=1;}  // 160m
    else if (freq >=Freq2Band[1][0] && freq <=Freq2Band[1][1] )  {BAND=2;}  //  80m
    else if (freq >=Freq2Band[2][0] && freq <=Freq2Band[2][1] )  {BAND=3;}  //  40m
    else if (freq >=Freq2Band[3][0] && freq <=Freq2Band[3][1] )  {BAND=4;}  //  30m
    else if (freq >=Freq2Band[4][0] && freq <=Freq2Band[4][1] )  {BAND=5;}  //  20m
    else if (freq >=Freq2Band[5][0] && freq <=Freq2Band[5][1] )  {BAND=6;}  //  17m
    else if (freq >=Freq2Band[6][0] && freq <=Freq2Band[6][1] )  {BAND=7;}  //  15m
    else if (freq >=Freq2Band[7][0] && freq <=Freq2Band[7][1] )  {BAND=8;}  //  12m
    else if (freq >=Freq2Band[8][0] && freq <=Freq2Band[8][1] )  {BAND=9;}  //  10m
    else if (freq >=Freq2Band[9][0] && freq <=Freq2Band[9][1] ) {BAND=10;}  //   6m
    else if (freq >=Freq2Band[10][0] && freq <=Freq2Band[10][1] ) {BAND=11;}  // 2m
    else if (freq >=Freq2Band[11][0] && freq <=Freq2Band[11][1] ) {BAND=12;}  // 70cm
    else {BAND=0;}                                                // out of range
}
#endif

#ifdef Nex_UDP
// pass through Nextion serial port commands to UDP fore remote display
void Nex_Redirect_to_UDP(uint8_t *nex_cmd)
{
  Serial.println("Data to Nextion");
  Nex_enet_write(nex_cmd);
}

// pass through responses from Nextion serial port to UDP
unsigned int Nex_Redirect_from_UDP(char * rx_string, uint8_t len)
{
  Serial.println("Data From Remote Nextion Display over UDP");
   int packetSize = Udp_Nex.parsePacket();
  if (packetSize) 
  {
    Serial.print("#1 Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    
    // read the packet into packetBufffer
    Udp_Nex.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    //Serial.println("Contents:");
    Serial.println(packetBuffer);
    return 1;
  }  
  return 0;
}

uint8_t Nex_enet_write(uint8_t *tx_buffer)
{  
    IPAddress remote_ip(192, 168, ip_adr1, dest_ip_adr0); // use EEPROM stored values

    if (enet_ready & EEPROM.read(ENET_ENABLE))   // skip if no enet connection
    {
        DBG_Serial.print("Data to Remote Nextion Display via UDP: ");
        DBG_Serial.println((char *) tx_buffer);
        Udp_Nex.beginPacket(remote_ip, remoteport_Nex);
        Udp_Nex.write((char *) tx_buffer);
        Udp_Nex.endPacket();
        return 1;
    }
    return 0;
}
#endif
/* [] END OF FILE */
