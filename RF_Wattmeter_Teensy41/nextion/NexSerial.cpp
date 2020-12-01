//#include <stdio.h>
//#include <string.h>
//#include <stddef.h>
//#include <stdlib.h>
#include "NexConfig.h"
#include "NexSerial.h"
#include <Utilities.h>

//#define LORA
#define RxBUFFLEN 64
unsigned char rxBuff[RxBUFFLEN];
volatile unsigned char rxHead, rxTail;
extern char cmd[RxBUFFLEN];
extern unsigned char WAIT;  // traffic flag - if a function is waiting for a response for the display set this flag to hold off display updates

void Serial_Init()
{
    rxHead = 0;
    rxTail = 0;
#ifdef LORA
    LoRaCfg();
#endif
    nexSerial.begin(9600);
}

unsigned char Serial_Write(unsigned char c)
{
#ifdef LORA    
    LoRaAuxReady();   // pin connected to CTS of UART1 now through an inverter.  Should not need this function for write.
#endif
    //while (WAIT)
      //  delay(1);
    nexSerial.write(c);  // PSoC API
    return 1;
}

unsigned char Serial_Read()
{
    unsigned char c;   
    
    if (rxTail < rxHead)
    {        
        c = rxBuff[rxTail++];   // extract next char not read yet by functions
        if (rxTail == rxHead)   // if we are caught up, reset buffer
        {
            rxHead = 0;
            rxTail = 0;
        }
    }
    else
    {
        rxHead = 0;     // reset buffer if empy or tail > head for some reason
        rxTail = 0;
        c = -1;         // -1 indicates something wrong, ignore C value
    }
    return c;
}

unsigned char Serial_Available()
{   
    //  Shift to Interrupt routine to fill buffer    
    unsigned char c; 
      
    while ((nexSerial.available()) != 0)
    {       
        c = nexSerial.read();
        rxBuff[rxHead++] = c;  // fill buffer with next char
        if (rxHead > RxBUFFLEN-1)  // prevent overrun
            rxHead = RxBUFFLEN;
        if (rxTail > RxBUFFLEN-1)
            rxTail = RxBUFFLEN;
    } 
    return rxHead - rxTail;
}

unsigned char Serial_ReadBytes(char *buf, unsigned char len)
{
    unsigned char cnt = 0;
    
    Serial_Available();
    if (len < rxHead - rxTail)
    {
        //ArrayCopy(buf, &rxBuff[rxTail], len);
        ClearArray(buf);
        memcpy(buf, &rxBuff[rxTail], len);
        rxTail += len;
        cnt = len;
    }
    else if (len == rxHead - rxTail)
    {
        //ArrayCopy(buf, &rxBuff[rxTail], len);
        ClearArray(buf);
        memcpy(buf, &rxBuff[rxTail], len);
        rxTail = 0;
        rxHead = 0;
        cnt = len;
    }
    else
    {
        //ArrayCopy(buf, &rxBuff[rxTail], rxHead - rxTail);
        ClearArray(buf);
        cnt = rxHead - rxTail;
        if (cnt > strlen(buf)) // On initial power up (not resets) the memecpy was copyung more bytes then the buffer.
            cnt = strlen(buf);  // this appears to prevent that and make for reliable startup
        if (cnt !=0 && rxTail !=0)
            memcpy(buf, &rxBuff[rxTail], cnt);                
        rxTail = 0;
        rxHead = 0;
    }
    return cnt;
}

void Serial_Print(char *scmd )
{
    // using global cmd_data as passing string is not working for some reason, only getting first char of strings.
#ifdef LORA    
    LoRaAuxReady();
#endif
    //while (WAIT)
      //  delay(1);
    nexSerial.write(cmd);
    //dbSerial.write(cmd);
}

#ifdef LORA
uint8 LoRaAuxReady(void)
{
    #define LOW 0
    #define HIGH 1
    #define TIME_OUT_CNT 100
    uint8_t cnt = 0;
    // uint8_t data_buf[100], data_len;

    while(LoRa_Aux_Read()==LOW && (cnt++<TIME_OUT_CNT)) {
        //Serial.print(".");
        delay(2);
    }

    if(cnt==0) 
    {
    } 
    else if(cnt>=TIME_OUT_CNT) 
    {
        //STATUS = RET_TIMEOUT;
        delay(10);
        //Serial.println(" TimeOut");
    } 
    else 
    {
        //Serial.println("");
        delay(2);
    }
    return 1;
}
    
void LoRaCfg(void)
{
    unsigned char Readcnt, idx, buf_len;
    buf_len=6;
    unsigned char Readbuf[10];
    
    // When configuring, uses 9600baud.  Need to figure out how to change UART speed programatically
    //LoRaAuxReady();
    LoRaControl_Write(0x03);   // program mode = 3
    delay(5);
    //UART1_ClearRxBuffer();
    LoRaAuxReady();
    nexSerial.write(0xC3);   // C1 3x is module info
    nexSerial.write(0xC3);
    nexSerial.write(0xC3); 
    delay(4);
    Readcnt = nexSerial.available();
    if (Readcnt == 4) {
        for(idx=0;idx<4;idx++) 
        {
            Readbuf[idx] = nexSerial.read();
            if (Readbuf[idx] == 0)
                    delay(1); // This is just to suppress compiler unused var warning, may use later
        }
    }
    //else
        //UART1_ClearRxBuffer();  // Size not match
        
    // default is C0 00 00 1A 17 44
    //LoRaAuxReady();
    LoRaAuxReady();
    nexSerial.write(0xC0);    // write 6 cfg bytes an save to NVRAM.  USe C2 to not save.
    nexSerial.write(0x00);   // Address
    nexSerial.write(0x00);   // Address
    nexSerial.write(0x1D);   // 25 is 19200 wire and 19200 air speed
                           // 1D is 9600 wire and 19200 air
                           // 1C is 9600 wire 9600 air
                           // 1A is 9600 wire and 2400 air
                           // 24 is 19200 wire and 9600 air
    nexSerial.write(0x0F);   // 0F is 915MHz. 00-1F is 90 to 931MHz range. Top 3 bits reseved. This is the Channel
    nexSerial.write(0x46);   // 44 is transparent mode with TX and RX pullups, 250ms wakeup, FEC on.  Bottom 2 bits is power level.  4+0 = 20Bm, 4+2 = 14dBm for 46 total    
    delay(50);
    
    LoRaAuxReady();
    nexSerial.write(0xC4);   // C4 3x is modulke reset
    nexSerial.write(0xC4);
    nexSerial.write(0xC4); 
    delay(1200);
    
    LoRaAuxReady();
    //UART1_ClearRxBuffer();
    nexSerial.write(0xC3);   // C1 3x is save parms 
    nexSerial.write(0xC3);
    nexSerial.write(0xC3); 
    delay(40);
    Readcnt = nexSerial.available();
    if (Readcnt == buf_len) 
    {
        for(idx=0;idx<buf_len;idx++) 
        {
        Readbuf[idx] = nexSerial.read();
        }
    }
    
    LoRaAuxReady();
    LoRaControl_Write(0x00);   // Operational Mode = 0
    LoRaAuxReady();
    delay(10);
}
#endif
