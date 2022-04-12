/*
   Ethenet UDP functions, part of Rotor-Controller-UDP-Server
   
 This sketch receives UDP message commands from a client to turn a rotator and sends rotator position
 information back to the client.

January 2021
K7MDL

 This code is in the public domain.
 */
// 
//____________________________________ enet_read _________________________________________________
//
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
          Udp.read(rx_buffer, BUFFER_SIZE);
          rx_buffer[count] = '\0';
          rx_count = count;          
          Serial.println(rx_count);
          Serial.println((char *) rx_buffer);
          
          // initially p1 = p2.  parser will move p1 up to p2 and when they are equal, buffer is empty, parser will reset p1 and p2 back to start of sData         
          memcpy(pSdata2, rx_buffer, rx_count+1);   // append the new buffer data to current end marked by pointer 2        
          pSdata2 += rx_count;                      // Update the end pointer position. The function processing chars will update the p1 and p2 pointer             
          rx_count = pSdata2 - pSdata1;             // update count for total unread chars. 
          //DBG_Serial.println(rx_count);  
     }
     rx_buffer[0] = '\0';
     return rx_count;
}
// 
//____________________________________ enet_write _________________________________________________
//
uint8_t enet_write(uint8_t *tx_buffer, uint8_t tx_count)
{   
   if (enet_ready)   // skip if no enet connection
   {
       if (DBG==6) Serial.print("ENET Write: ");
       if (DBG==6) Serial.println((char *) tx_buffer);
       Udp.beginPacket(remote_ip, remoteport);
       Udp.write((char *) tx_buffer);
       Udp.endPacket();
       return 1;
   }
   return 0;
} 
// 
//____________________________________ enet_start _________________________________________________
//
void enet_start(void)
{
  uint8_t mac[6];
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
// 
//____________________________________ teensyMAC _________________________________________________
//
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
// 
//____________________________________ generic_UDP_Read _________________________________________________
//
void generic_UDP_Read(void)
{
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("#2 Received packet of size from client ");
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
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
  }    
    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
}
