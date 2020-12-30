/*
WSJTX_Decode_Arduino

 created 12/26/2020
 last updated 12/29/2020
 by K7MDL

 This code is in the public domain.

This program is a test program to learn UDP messaging over ethernet on a Teensy 4.1 which has native ethernet chip.
The code below is a hodgepodge of examples and various observations of WSJTX decoding programs, most of them in Python. 
Most of the GitHub examples I looked at are outdated as new fields have been added to many messages as of WSJT-X 2.3.0 rc2.
Also looked through the WSJT-X source code to get the updated fields.   

No points for style here, just hacked my way through to get packets to decode correctly.  

--> Unresolved still as of 12/29/2020 - Still figuring out how to handle the date/time values.

My ultimate goal is to apply this to my Band Decoder project which is currently serial based. Will listen to
WSJT-X, N1MM, or other logger's UDP messages to gleen current radio band information and other useful band decoding
related info such as the N1MM antenna table, recently extended from 16 rows to 64 rows. 17-64 are only available over UDP.

The 2nd goal for my Band Decoder/RF Wattmeter is to have a local CPU with Nextion touchscreen display talk over ethernet
to a remote band decoder/wattmeter that is actually measuring RF power and controlling equipment.  
The current capability is over USB Serial which requires 1 to 3 USB serial ports.  Converting to ethernet should make
things easier and faster and take advantage of the latest logging programs and WSJTX.

*/

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <TimeLib.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};
IPAddress ip(192, 168, 2, 187);  // insert your devices static IP

#define UDP_RX_BUFFER
unsigned int localPort = 2237;      // local port to listen on
// buffers for receiving and sending data
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet, 24bytes is not enought for this app.
char packetBuffer[1500];  // larger buffer to hold incoming packet.
char ReplyBuffer[] = "acknowledged";        // a string to send back
volatile uint32_t ptr_pos;
uint32_t max_ptr_pos;
char str_decode[255];

//Status Message
long the_type;
char wsjtx_id[20];
float dial_frequency;
char mode[20];
char dx_call[20];
char report[20];
char tx_mode[20];
bool tx_enabled;
bool transmitting;
bool decoding;
uint32_t rx_df;
uint32_t tx_df;
char de_call[20];
char de_grid[20];
char dx_grid[20];
bool tx_watchdog;
char sub_mode[20];
bool fast_mode;
uint8_t special_op_mode;
int32_t frequency_tolerance;
int32_t tr_period;
char configuration_name[40];

//Heartbeat Message
long heartbeat_maximum_schema_number;
char heartbeat_version[10];
char heartbeat_revision[10];

// Decode Message
bool decode_is_new;
time_t decode_time; //:: DiffTime    QTime(int h, int m, int s = 0, int ms = 0)
int16_t h;  // QTime class components
int16_t m;
int16_t s;
int16_t ms;
int32_t decode_snr;
float decode_delta_time;
time_t decode_delta_frequency;
char decode_mode[20];
char decode_message[100];
bool low_confidence;
bool off_air;

double logged_date_time_off;
char logged_dx_call[40];
char logged_dx_grid[40];
float logged_dial_frequency;
char logged_mode[20];
char logged_report_sent[20];
char logged_report_received[20];
char logged_tx_power[80];
char logged_comments[80];
char logged_name[80];
double logged_date_time_on;
char operator_call[20];
char my_call[20];
char my_grid[20];
char exchange_sent[20];
char exchange_rcvd[20];
char prop_mode[20];

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

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
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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
  Udp.begin(localPort);
  Serial.print("Listening on UDP Port: ");
  Serial.println(localPort);
}

void loop() 
{
  int i;

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {
    Serial.print("\n\nReceived packet of size ");
    Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    //for (int i=0; i < 4; i++) {
    //  Serial.print(remote[i], DEC);
    //  if (i < 3) {
    //    Serial.print(".");
     // }
  //}
    //Serial.print(", port ");
   //Serial.println(Udp.remotePort());

    // read the packet into packetBuffer
    //Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Udp.read(packetBuffer, 1500);
    // decode contents of wsjtx and n1mm here
        Serial.print("\nRaw packet Content: SOP 0x");
        packetBuffer[packetSize+1] = '\0';
        for (i=0; i< packetSize; i++)
        {   
            Serial.print("i");
            Serial.print(i);       
            Serial.print("-x");
            Serial.print(packetBuffer[i], HEX);                      
            Serial.print(",");
        }
        Serial.println("|| EOP\n");
            
        Serial.print("Raw packet Text View Content: ");
        for (i=11; i< packetSize; i++)
        {          
            Serial.print(packetBuffer[i]);
            //Serial.print("");
        }
        Serial.println("|| packet end\n");
        
// TYPE_VALUE = 1 = Status
// 0 = heartbeat
// 2 = decode
        PacketReader(packetSize);
        the_type = QInt32();
        Serial.print("UDP Message Type is ");
        Serial.println(the_type, HEX);
        QString();                  
        strcpy(wsjtx_id, str_decode);
        Serial.print("WSJTX ID is:");
        Serial.println(wsjtx_id);

        // Now process by type of packet
        if (the_type == 0)  // Heartbeat
        {
            heartbeat_maximum_schema_number = QUint32();
            Serial.print("heartbeat_maximum_schema_number:");
            Serial.println(heartbeat_maximum_schema_number);
            QString();                  
            strcpy(heartbeat_version, str_decode);
            Serial.print("heartbeat_version:");
            Serial.println(heartbeat_version);
            QString();                  
            strcpy(heartbeat_revision, str_decode);  
            Serial.print("heartbeat_revision:");
            Serial.println(heartbeat_version);
        }
        if (the_type == 1)        
        {
            StatusPacket(packetSize);
        }
        if (the_type == 2)
        {
            Decode(packetSize);
        }
        if (the_type == 5)
        {       
            QSOLogged(packetSize);
        }
        if (the_type == 12)
        {
            Serial.print("ADIF Record:");                    
            for (i=16; i< packetSize; i++)
            {          
                Serial.print(packetBuffer[i]);
                //Serial.print("");
            }
            Serial.println("|| packet end\n");
            Serial.println('\n');       
        }
    // send a reply to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
  }
  delay(10);
}

void Decode(int packetSize)
{
    decode_is_new = QBool();
    Serial.print("decode_is_new:");
    Serial.println(decode_is_new);

    decode_time = QUint32();          //Time    4 byte value   h, m, s is 6 bytes 8 with ms
    Serial.print("decode_time:");
    Serial.println(decode_time);
    time_t t = decode_time;
    //breakTime(decode_time, tm);
    Serial.println(hour(t));
    Serial.println(minute(t));
    Serial.println(second(t));    

    decode_snr = QInt32();
    Serial.print("decode_snr:");
    Serial.println(decode_snr);
    
    decode_delta_time = QFloat();
    Serial.print("decode_delta_time:");
    Serial.println(decode_delta_time);
    
    decode_delta_frequency = QUint32();
    Serial.print("decode_delta_frequency:");
    Serial.println(decode_delta_frequency);

    QString();
    strcpy(decode_mode, str_decode);
    Serial.print("decode_mode:");
    Serial.println(decode_mode);
          
    QString();
    Serial.println(str_decode); 
    strcpy(decode_message, str_decode);
    Serial.print("decode_message:");
    Serial.println(decode_message);            
    
    bool low_confidence = QBool();
    Serial.print("low_confidence:");
    Serial.println(low_confidence);            
    
    bool off_air = QBool();
    Serial.print("off_air:");
    Serial.println(off_air);  
}

void QSOLogged(int packetSize)
{         
    logged_date_time_off = QUint8();
    logged_date_time_off = QUint32();
    logged_date_time_off = QUint64();
    Serial.print("logged_date_time_off:");
    Serial.println(logged_date_time_off);

    QString();        
    strcpy(logged_dx_call, str_decode);
    Serial.print("logged_dx_call:");
    Serial.println(logged_dx_call);
   
    QString();
    strcpy(logged_dx_grid, str_decode);
    Serial.print("logged_dx_grid:");
    Serial.println(logged_dx_grid);
    
    logged_dial_frequency = QUint64();                         
    Serial.print("logged_dial_frequency:");
    Serial.println(logged_dial_frequency);

    QString();
    strcpy(logged_mode, str_decode);
    Serial.print("logged_mode:");
    Serial.println(logged_mode);
   
    QString();
    strcpy(logged_report_sent, str_decode);
    Serial.print("logged_report_sent:");
    Serial.println(logged_report_sent);
    
    QString();
    strcpy(logged_report_received, str_decode);
    Serial.print("logged_report_received:");
    Serial.println(logged_report_received);
   
    QString();
    strcpy(logged_tx_power, str_decode);
    Serial.print("logged_tx_power:");
    Serial.println(logged_tx_power);

    QString();    
    strcpy(logged_comments, str_decode);
    Serial.print("logged_comments:");
    Serial.println(logged_comments);
  
    QString();
    strcpy(logged_name, str_decode);
    Serial.print("logged_name:");
    Serial.println(logged_name);
    
    logged_date_time_off = QUint8();
    logged_date_time_off = QUint32();
    logged_date_time_off = QUint64();                      
    Serial.print("logged_date_time_on:");
    Serial.println(logged_date_time_on);
    
    QString();            
    strcpy(operator_call, str_decode);
    Serial.print("operator_call:");
    Serial.println(operator_call);
    
    QString();            
    strcpy(my_call, str_decode);
    Serial.print("my_call:");
    Serial.println(my_call);
             
    QString();          
    strcpy(my_grid, str_decode);
    Serial.print("my_grid:");
    Serial.println(my_grid);
    
    QString();            
    strcpy(exchange_sent, str_decode);
    Serial.print("exchange_sent:");
    Serial.println(exchange_sent);

    QString();
    strcpy(exchange_rcvd, str_decode);
    Serial.print("exchange_rcvd:");
    Serial.println(exchange_rcvd);
         
    QString();
    strcpy(prop_mode, str_decode);
    Serial.print("prop_mode:");
    Serial.println(prop_mode);
}

void StatusPacket(int packetSize)     // process packetBuffer string
{
    dial_frequency = QUint64();
    Serial.print("Dial Frequency is:");
    Serial.println(dial_frequency);
    
    QString();
    strcpy(mode, str_decode);
    Serial.print("Mode is:");
    Serial.println(mode);
    
    QString();
    strcpy(dx_call, str_decode);
    Serial.print("Dx Call is:");
    Serial.println(dx_call);
    
    QString();
    strcpy(report, str_decode);
    Serial.print("Report is:");
    Serial.println(report);
    
    QString();
    strcpy(tx_mode, str_decode);
    Serial.print("TX Mode is:");
    Serial.println(tx_mode);
    
    tx_enabled = QBool();
    Serial.print("TX Enabled is:");
    Serial.println(tx_enabled);
    
    transmitting = QBool();
    Serial.print("Transmitting is:");
    Serial.println(transmitting);
    
    decoding = QBool();
    Serial.print("Decoding is:");
    Serial.println(decoding);
              
    rx_df = QUint32();
    Serial.print("rx_df is:");
    Serial.println(rx_df);
    
    tx_df = QUint32();
    Serial.print("tx_df is:");
    Serial.println(tx_df);
    
    QString();
    strcpy(de_call, str_decode);
    Serial.print("De Call is:");
    Serial.println(de_call);

    QString();
    strcpy(de_grid, str_decode);
    Serial.print("De Grid is:");
    Serial.println(de_grid);          
    
    QString();
    strcpy(dx_grid, str_decode);
    Serial.print("DX Grid is:");
    Serial.println(dx_grid);
    
    tx_watchdog = QBool();
    Serial.print("TX Watchdog is:");
    Serial.println(tx_watchdog);

    QString();
    strcpy(sub_mode, str_decode);
    Serial.print("Sub_Mode is:");
    Serial.println(sub_mode);

    fast_mode = QBool();
    Serial.print("Fast_mode is:");
    Serial.println(fast_mode);

    special_op_mode = QUint8();
    Serial.print("Special_op_mode is:");
    Serial.println(special_op_mode);

    frequency_tolerance = QInt32();
    Serial.print("frequency_tolerance:");
    Serial.println(frequency_tolerance);

    tr_period = QInt32();
    Serial.print("tr_period:");
    Serial.println(tr_period);

    QString();           
    strcpy(configuration_name, str_decode);
    Serial.print("configuration_name:");
    Serial.println(configuration_name);
              
    sprintf(str_decode, "Decoded packet = %lu-%s-%f-%s-%s-%s-%s-%d-%d-%d", the_type, wsjtx_id, dial_frequency, mode, dx_call, report, tx_mode, tx_enabled, transmitting, decoding);
    Serial.print(str_decode);
    str_decode[0] = '\0';
    sprintf(str_decode, "-%lu-%lu-%s-%s-%s-%d-%s-%d-%X-%#lX-%#lX-%s-EOP", rx_df, tx_df, de_call, de_grid, dx_grid, tx_watchdog, sub_mode, fast_mode, special_op_mode, frequency_tolerance, tr_period, configuration_name);
    Serial.println(str_decode);
    str_decode[0] = '\0';
}
void PacketReader(int packetSize)
{  
  ptr_pos = 0;
  max_ptr_pos = packetSize - 1;
  skip_header(); 
}

void skip_header(void)
{
  if (max_ptr_pos < 8)  // skip header 8 bytes
      Serial.println("Not enough data to skip header");
  ptr_pos += 8;
}

uint8_t at_eof(void)
{
  if (ptr_pos > max_ptr_pos)
  {
      Serial.print("at_eof");   
      return 1;
  }
  else
  {
      Serial.print("NOT at_eof");   
      return 0;
  }
}

uint32_t QUint32(void)  // get the next 4 bytes and return as an unsigned int32
{
  uint8_t i;
  uint32_t value;
  char sval[5] = {};

  for (i=0; i<4; i++)
  {
      sval[i] = packetBuffer[ptr_pos+i];
  }
  sval[4] = '\0';

  // little endian
  //uint8_t *sval;
  //value = sval[0] + (sval[1] << 8) + (sval[2] << 16) + (sval[3] << 24);
  //Serial.print("\nQUint32 (little endian) = ");
  //Serial.println(val1);
  // big endian  
  value = (sval[0] << 24) + (sval[1] << 16) + (sval[2] << 8) + sval[3];
  //Serial.print("\nQUint32 (big endian) = ");
  //Serial.println(value);
  ptr_pos += 4;
  return value;
}

int32_t QInt32(void)  // get the next 4 bytes and return as an unsigned int32
{
  uint8_t i;
  int32_t value;
  char sval[5] = {};
  //Serial.print("ptr_pos = ");
  //Serial.println(ptr_pos);
  for (i=0; i<4; i++)
  {
      sval[i] = packetBuffer[ptr_pos+i];
  }
  sval[4] = '\0';
  value = (sval[0] << 24) + (sval[1] << 16) + (sval[2] << 8) + sval[3];
  //Serial.print("QInt32 = ");
  //Serial.println(value);
  ptr_pos += 4;
  return value;
}

uint16_t QUint16(void)
{
  uint8_t i;
  uint16_t value;
  char sval[3] = {};

  for (i=0; i<2; i++)
  {
      sval[i] = packetBuffer[ptr_pos+i]+48;
  }
  sval[2] = '\0';
  value = (sval[0] << 8) + sval[1];
  //Serial.print("QUint16 = ");
  //Serial.println(value);
  ptr_pos += 2;
  return value;
}

uint8_t QUint8(void)
{
    char value;
    value = packetBuffer[ptr_pos];
    //Serial.print("QUint8 = ");
    //Serial.println(value);
    ptr_pos += 1;
    return value;
}

int8_t QInt8(void)
{
    char value;
    value = packetBuffer[ptr_pos];
    //Serial.print("QInt8 = ");
    //Serial.println(value);
    ptr_pos += 1;
    return value;
}

int32_t QString(void)
{
    int8_t str_len =0;
    
    str_decode[0] = '\0';
    str_len = QInt32();
    if (str_len == -1)
    {
      Serial.println(" String length is -1");
      str_decode[0] = '\0';
      return 0;
    }
    //Serial.print(" String length is ");
    //Serial.println(str_len);
    strncpy(str_decode, &packetBuffer[ptr_pos], str_len);
    str_decode[str_len] = '\0';
    //Serial.print("QString = ");
    //Serial.println(str_decode);
    ptr_pos += str_len;
    return str_len;
}

bool QBool(void)
{
    bool value;
    value = packetBuffer[ptr_pos];
    ptr_pos += 1;
    if (value == 1)
      return true;
    else
      return false;
}

uint64_t QUint64(void)  // get next 8 bytes, return as a double
{
  uint8_t i;
  uint64_t value;
  uint64_t sval[9];

  for (i=0; i<8; i++)
  {
      sval[i] = packetBuffer[ptr_pos+i];
  }
  sval[8] = '\0';
  value = (sval[0] << 56) + (sval[1] << 48) + (sval[2] << 40) + (sval[3] << 32) + (sval[4] << 24) + (sval[5] << 16) + (sval[6] << 8) + sval[7];
  ptr_pos += 8;
  return value;
}

float QFloat(void)  // get next 8 bytes, return as a float
{
  float value;
  uint64_t sval[9];
  uint8_t i;

  //Serial.print("ptr_pos = ");
  //Serial.println(ptr_pos);
  for (i=0; i<8; i++)
  {
      sval[i] = packetBuffer[ptr_pos+i];
  }
  sval[9] = '\0';
  value = (sval[0] << 56) + (sval[1] << 48) + (sval[2] << 40) + (sval[3] << 32) + (sval[4] << 24) + (sval[5] << 16) + (sval[6] << 8) + sval[7];
  ptr_pos += 8;
  return value;
}
