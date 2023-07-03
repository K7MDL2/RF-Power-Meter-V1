/*
 Rotor-Controller-UDP-Server:
 This sketch receives UDP message commands from a client to turn a rotator and sends rotator position
 information back to the client.

January 2021
K7MDL

 This code is in the public domain.
 */

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <EEPROM.h>
// 
//____________________________________ Defines _________________________________________________
//
#define LED 13
#define ROTOR_AC_POWER_PIN 28
#define LEFT_FAST_PIN 29
#define LEFT_SLOW_PIN 30
#define RIGHT_FAST_PIN 31
#define RIGHT_SLOW_PIN 32
#define ROTOR_ANALOG_AZ_PIN A0
#define BUFFER_SIZE  (512)
#define LINE_STR_LENGTH         (20u)
#define BUF_LEN  BUFFER_SIZE
#define ON 1
#define OFF 0
#define STOP 0
#define CW 1
#define CCW 2
// 
//____________________________________ IP Settings _________________________________________________
//
// Enter a MAC address and IP address for your controller below. MAC not required for Teensy cause we are using TeensyMAC function.
// The IP address will be dependent on your local network:  don't need this since we can automatically figure ou tthe mac
//byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC
//};
IPAddress ip(192, 168, 2, 189);    // Our static IP address.  Could use DHCP but preferring static address.
unsigned int localPort = 7946;     // local port to LISTEN for the remote display/Desktop app
//unsigned int localPort_Nex = 7945;     // local port to LISTEN for the remote display/Desktop app

// buffers for receiving and sending data
char packetBuffer[BUFFER_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "Random Reply";        // a string to send back

//Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
IPAddress remote_ip(192, 168, 2, 65);  // destination  IP (desktop app or remote display Arduino
unsigned int remoteport = 7947;    // the destination port to SENDTO (a remote display or Desktop app)
//unsigned int remoteport_Nex = 7949;    // the destination port to SENDTO (a remote display or Desktop app)

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
// 
//____________________________________ Variables _________________________________________________
//
float ADC_COUNTS=1024;
float Vref = 3.3f;        // 3.3VDC for Nano and ESP32 (M5stack uses ESP32)  ESP32 also has calibrated Vref curve
uint8_t METERID = 100;    // tracks current Meter ID number Resets to default_METERID.
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t tx_buffer[BUFFER_SIZE];
uint8_t rx_count = 0;
uint8_t tx_count = 0;
uint8_t sdata[BUFFER_SIZE], *pSdata=sdata, *pSdata1=sdata, *pSdata2=sdata;
uint8_t enet_ready = 0;
unsigned long enet_start_fail_time = 0;
uint8_t DBG = 0;
uint16_t RotorTargetAZ;
float RotorAZ;
int16_t RotorAZ_Offset = -130;
float Preset[10] = {300.0, 359.0, 20.0, 40.0, 60.0, 70.0, 100.0, 140.0, 170.0, 185.0};
uint16_t RotorAZ_StartPos = 302;
float RotorAZ_raw;
uint8_t Rotor_StopBand = 3;
uint16_t manual_limit_CCW = 310;
uint16_t manual_limit_CW = 200; 
float RotorPosV;
uint16_t RotorPosCounts;
uint16_t RotorPosCounts_raw;
uint8_t SlowdownDegrees = 10;
uint8_t MoveRotor = STOP;
uint8_t MovetoPreset = 0;
unsigned long stall_detect_timer = 0;
uint8_t STALL_DETECT_DISTANCE = 4;  // number of degrees rotor needs to move to reset timeout timer
uint16_t STALL_TIMEOUT = 4000;  // number in milliseconds
unsigned long RotorAZ_raw_last = 0;
uint16_t map_pos_low_Counts = 231;  // A 33 ohm resistor is between the pot and ground and also the pot and 3.3V.  
                                    //There is a 1.756V range in the middle with 0.756 to ground on the bottom of the pot
                                    // and 2.52V on the top of the pot as measured in my install with HD-73 rotor with 75ohm pot.
uint16_t map_pos_high_Counts = 782;  // Adjust these number for actual install.  A map function wil expand this to 1024 count range 
                                      // as if the pot was swigning a full 0 to 3.3V.
unsigned long timer;   // For watchdog timer on Teensy
