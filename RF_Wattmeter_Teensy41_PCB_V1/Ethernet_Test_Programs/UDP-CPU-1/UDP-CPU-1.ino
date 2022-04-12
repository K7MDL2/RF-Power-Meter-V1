/*
UDP-CPU-1:

1. This program is a Nextion serial gateway seerver.  
The RF remote Wattmeter is the Client side with the host CPU redirecting serial commands over the ethernet with UDP.  
This server will convert the messages back to serial for a lcoal display.
Uses a static IP assigment for the client and server.

 This code is in the public domain.
 */

// This is a test program to send and receive data with the the K7MDL RF Wattmeter and Band Decoder
// Jan 1, 2021
#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

// Our address and listening port number
IPAddress ip(192, 168, 2, 186);
unsigned int localPort = 7944;      // local port to listen on

// Band Decoder/Wattmeter Address and it's listening port number
IPAddress ip_decoder(192, 168, 2, 188);
uint16_t decoder_port = 7945;

#define Nex_Serial Serial1

// buffers for receiving and sending data
char packetBuffer[512];  // buffer to hold incoming packet,
char ReplyBuffer[] = "100,120,239,1\r\n";        // Turn on and off the data output stream from the Wattmeter/Decoder.
char ReplyBuffer2[] =  "100,120,239,0\r\n";
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Enter a MAC address and IP address for your controller below.   Commented out to use the teensyMAC function to get the MAC addr.
// The IP address will be dependent on your local network:
//.byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};

// Get the Mac Addr in the Teensy.  Can still manually set if you want.
void teensyMAC(uint8_t *mac) {

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

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

  uint8_t mac[6];
  teensyMAC(mac);
  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  Serial1.begin(115200);
  //while (!Nex_Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort+2);
  Serial.println("Ethernet UDP Started, Waiting for Rx Packets from Decoder/Wattmeter");
}

void loop() 
{
    uint8_t i;
    //uint8_t tx_count;
    //unsigned long time_stamp = 0, time_stamp2 = 0;
    uint8_t temp[50] = {0};
    
    //if (millis() - time_stamp > 5000)
    //{
    // Read Nextion serial port RX line and mirror it out via UDP
    //tx_count = Nex_Serial.available();
    //for (i=0; i< tx_count; i++)
    //{
        Nex_Serial.setTimeout(0.1);
        Nex_Serial.readBytes((char *)temp, sizeof(temp));     
        Udp.beginPacket(ip_decoder, decoder_port);
        Udp.write((char *)temp);   // send out data received from the Nextion serial port
        Udp.endPacket();
        //time_stamp = millis();
    //}
   
    // Receive commands from the Host over UDP and send them to the the Nextion via the serial port.
    //if there's data available, read a packet
    int packetSize = Udp.parsePacket();
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
      Udp.read(packetBuffer, 512);
      Serial.println("Contents:");
      Serial.println(packetBuffer);
      Nex_Serial.print(packetBuffer); // send to Nextion on Hardware Serial port #1
    }
    delay(1);
}
