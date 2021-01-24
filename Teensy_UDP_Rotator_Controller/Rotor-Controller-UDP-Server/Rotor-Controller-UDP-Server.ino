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
  load_presets();  
  enet_start();
  Serial.println("RTR1-Completed Setup()");
}
// 
//____________________________________ LOOP _________________________________________________
//
void loop()
{
  static unsigned long rotor_state_time;
  
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
      if (RotorTargetAZ > 360)
          if (DBG==1) Serial.print(RotorTargetAZ-360);
      else
          if (DBG==1) Serial.println(RotorTargetAZ);
  }
}
// 
//____________________________________ rotor_position_counts _________________________________________________
//
// read the ADC voltage and store voltage in global var
void rotor_position_counts(void)
{
    RotorPosCounts = analogRead(ROTOR_ANALOG_AZ_PIN); 
    if (DBG==2)
    {
      if (DBG==2) Serial.print("RTR1-Rotor Pos Counts =");
      if (DBG==2) Serial.println(RotorPosCounts);
    }
}
// 
//____________________________________ rotor_position_volts _________________________________________________
//
// Convert ADC vpoltage to compass heading
void rotor_position_volts(void)
{
    RotorPosV = RotorPosCounts * (Vref/1024);
    if (DBG==2)
    {
      if (DBG==2) Serial.print("RTR1-Rotor Pos Volts =");
      if (DBG==2) Serial.println(RotorPosV);
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
    RotorAZ = RotorAZ_raw + RotorAZ_Offset + RotorAZ_StartPos;
    if (RotorAZ > 360)
        RotorAZ -= 360;
    if (RotorAZ < 0)
        RotorAZ += 360; 
    RotorAZ_raw += RotorAZ_StartPos;
    if (DBG==2)
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
