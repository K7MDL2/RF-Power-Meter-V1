/*
 Rotor-Controller-UDP-Server:
 This sketch receives UDP message commands from a client to turn a rotator and sends rotator position
 information back to the client.

January 2021
K7MDL

 This code is in the public domain.
 */


// This is a test program to send and receive data with the the K7MDL RF Wattmeter and Band Decoder
// Use with UDP-CPU-1 program.  Set up your IP addresses to your network.
// Jan 1, 2021

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
//#include <EEPROM.h>
//#include <stdio.h>
//#include <math.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stddef.h> 
#include "Rotor-Controller-UDP-Server.h"
#include <avr/wdt.h>

void(* resetFunc) (void) = 0;   // set up software reset function - causes a hang on Teensy 4.1
// 
//____________________________________ SETUP _________________________________________________
//
void setup()
{
  DBG = 0;
  // set pinMode here to prevent all relays from turning on at once.
  set_pins();
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  digitalWrite(LED, OFF);
  // set pinMode here to prevent all relays from turning on at once.
  send_pin_Status();
  init_relay_state();
  enet_start();
  
  if (EEPROM.read(0) != 'G') 
  {
      Serial.print("RTR1-Initializing EEPROM to default values - Byte 0 = ");
      Serial.println(EEPROM.read(0));
      set_EE_Vars();  // 'G' is good EEPROM otherwise reset EEPROM values to default
      EEPROM.update(0,'G');  // mark EEPROM as programmed
  }   
  else 
  {
      get_EE_Vars();   // found a 'G' in byte 0, read it, esle skip and use the defaults in memory.
      Serial.println("RTR1-EEPROM Read completed");
  }
  
  //load_presets();  can use this function if desired.  When commented out, uses the values initialized in the .h file
      
  /*    Available time periods:      
      ATMega 8, 168, 328, 1280, 2560:
      WDTO_15MS (15 ms )
      WDTO_30MS (30 ms)
      WDTO_60MS (60 ms)
      WDTO_120MS (120 ms)
      WDTO_250MS (250 ms)
      WDTO_500MS (500 ms)
      WDTO_1S (1 sec)
      WDTO_2S (2 sec)
  */    
  //wdt_enable(WDTO_2S);  // (30 ms)   // does not seem to work on the Teensy 4.1.
    Serial.println("RTR1-Enabled Watchdog Timer");
    
    Serial.println("RTR1-Completed Setup()");
}
// 
//____________________________________ LOOP _________________________________________________
//
void loop()
{
    static unsigned long rotor_state_time;
  
    timer = millis() + 10000UL;                                 // length of time we will reset WDT
    
    if (!enet_ready){
        if ((millis() - enet_start_fail_time) >  600000)  // check every 10 minutes (600K ms) and attempt a restart.
            enet_start();
    }
    enet_read();   // sets rx_count if there is data
    if (rx_count!=0)
      get_remote_cmd();       // scan buffer for command strings
    
    #ifdef STRESS_TEST_RELAYS
        stress_test_relays();
    #endif
  
    // Periodically look at rotor position and move rotor to target. The move part wil likey change to only move on user command, for now it is following target vs simulated position
    if ((millis() - rotor_state_time) > 200)  
    {
        rotor_state_time = millis();
        Rotor_state();   // reads the positions updating sposition variables
        stall_detect();   // check for stall during movement and stop with error msg if timeout exceeded      
        compute_rotor_move();  // mnually or auto move rotor
        if (!MovetoPreset && !MoveRotor)
              RotorTargetAZ = 0;
        if (DBG==1) Serial.print("RTR1-Rotor AZ = ");
        if (DBG==1) Serial.print(RotorAZ);
        if (DBG==1) Serial.print(" Target AZ = ");
        if (RotorTargetAZ > 360){if (DBG==1) Serial.print(RotorTargetAZ-360);}
        else {if (DBG==1) Serial.println(RotorTargetAZ);}
    }
    //wdt_reset();
}
// 
//____________________________________ rotor_position_counts _________________________________________________
//
// read the ADC voltage and store counts returned in global var
void rotor_position_counts(void)
{
    RotorPosCounts_raw = analogRead(ROTOR_ANALOG_AZ_PIN); 
    RotorPosCounts = map(RotorPosCounts_raw, map_pos_low_Counts, map_pos_high_Counts, 0, 1023);
    if (DBG==4)
    {
        Serial.print("RTR1-Rotor Pos Counts =");
        Serial.println(RotorPosCounts);
    }
}
// 
//____________________________________ rotor_position_volts _________________________________________________
//
// Convert ADC voltage to compass heading
void rotor_position_volts(void)
{
    RotorPosV = RotorPosCounts_raw * (Vref/1024);
    if (DBG==4)
    {
        Serial.print("RTR1-Rotor Pos Volts =");
        Serial.println(RotorPosV);
    }
}
// 
//____________________________________ rotor_position_AZ _________________________________________________
//
// Convert counts to degrees using 360 scale
void rotor_position_AZ(void)
{
    rotor_position_counts();
    RotorAZ_raw = ((360*RotorPosCounts)/ADC_COUNTS);
    RotorAZ = (float) RotorAZ_raw + RotorAZ_Offset + RotorAZ_StartPos;
    if (RotorAZ > 360)
        RotorAZ -= 360;
    if (RotorAZ < 0)
        RotorAZ += 360; 
    RotorAZ_raw += RotorAZ_StartPos;
    if (DBG==4)
    {
      Serial.print("RTR1-RotorAZ_raw =");
      Serial.print(RotorAZ_raw);
      Serial.print("  RotorAZ =");
      Serial.println(RotorAZ);      
    }
}
// 
//____________________________________ load_presets _________________________________________________
//       
void load_presets(void)
{
    // set up test presets
    Preset[0] = 30.0;
    Preset[1] = 60.0;
    Preset[2] = 90.0;
    Preset[3] = 120.0;
    Preset[4] = 150.0;
    Preset[5] = 180.0;
    Preset[6] = 240.0;
    Preset[7] = 285.0;
    Preset[8] = 330.0;
    Preset[9] = 355.0;
}

void send_status(void)
{
    Serial.println((char*) tx_buffer);
    enet_write(tx_buffer, 1);
}

void reboot(void) 
{
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while(1) {}
}
