/**
 * @file NexHardware.cpp
 *
 * The implementation of base API for using Nextion. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/11
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#include "NexHardware.h"
#include "RF_Wattmeter_Nextion.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#define NEX_RET_CMD_FINISHED            (0x01)
#define NEX_RET_EVENT_LAUNCHED          (0x88)
#define NEX_RET_EVENT_UPGRADED          (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)     
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_INVALID_CMD             (0x00)
#define NEX_RET_INVALID_COMPONENT_ID    (0x02)
#define NEX_RET_INVALID_PAGE_ID         (0x03)
#define NEX_RET_INVALID_PICTURE_ID      (0x04)
#define NEX_RET_INVALID_FONT_ID         (0x05)
#define NEX_RET_INVALID_BAUD            (0x11)
#define NEX_RET_INVALID_VARIABLE        (0x1A)
#define NEX_RET_INVALID_OPERATION       (0x1B)


// added for K7MDL version 12/2020 - some of this is carry over from PSoC5, others are for LoRA from the PSoC5 so far.
//extern unsigned char WAIT;  // traffic flag - if a function is waiting for a response for the display set this flag to hold off display updates
extern unsigned char pg;
//extern unsigned int DLY;  // // Set the value to prevent missed messages.  
// Delay value is critical.   LoRa at 9600 500 is good. 50 for wired connection.
// Less and you start missing events, numbers come back 0.  After a while LoRa data gets backed up, missing events
// Need flow control on the Nextion end likely.

//#define NEX_UDP

#ifdef NEX_UDP
    extern void Nex_Redirect_from_UDP(char *);
    extern void Nex_Redirect_to_UDP(char *);
#endif


/*
 * Receive page number. 
 * 
 * @param number - save uint32_t data. 
 * @param timeout - set timeout time. 
 *
 * @retval true - success. 
 * @retval false - failed.
 *
 */
bool recvPageNumber(uint32_t *number, uint32_t timeout)
{
    bool ret = false;
    uint8_t temp[5] = {0};

    if (!number)
    {
        goto __return;
    }
    
    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_CURRENT_PAGE_ID_HEAD
        && temp[2] == 0xFF
        && temp[3] == 0xFF
        && temp[4] == 0xFF
        )
    {
        *number = ((uint32_t)temp[1]);
        ret = true;
    }

__return:

    if (ret) 
    {
        dbSerialPrint("recvPageNumber :");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvPageNumber err");
        dbSerialPrintln(temp[1]);
    }
    
    return ret;
}

/*
 * Receive uint32_t data. 
 * 
 * @param number - save uint32_t data. 
 * @param timeout - set timeout time. 
 *
 * @retval true - success. 
 * @retval false - failed.
 *
 */
bool recvRetNumber(uint32_t *number, uint32_t timeout)
{
    bool ret = false;
    uint8_t temp[8] = {0};

    if (!number)
    {
        goto __return;
    }
    
    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_NUMBER_HEAD
        && temp[5] == 0xFF
        && temp[6] == 0xFF
        && temp[7] == 0xFF
        )
    {
        *number = ((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | (temp[2] << 8) | (temp[1]);
        ret = true;
    }

__return:

    if (ret) 
    {
        dbSerialPrint("recvRetNumber :");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvRetNumber err");
    }
    
    return ret;
}


/*
 * Receive string data. 
 * 
 * @param buffer - save string data. 
 * @param len - string buffer length. 
 * @param timeout - set timeout time. 
 *
 * @return the length of string buffer.
 *
 */
uint16_t recvRetString(char *buffer, uint16_t len, uint32_t timeout)
{
    uint16_t ret = 0;
    bool str_start_flag = false;
    uint8_t cnt_0xff = 0;
    String temp = String("");
    uint8_t c = 0;
    long start;

    if (!buffer || len == 0)
    {
        goto __return;
    }
    
    start = millis();
    while (millis() - start <= timeout)
    {
        while (nexSerial.available())
        {
            c = nexSerial.read();
            if (str_start_flag)
            {
                if (0xFF == c)
                {
                    cnt_0xff++;                    
                    if (cnt_0xff >= 3)
                    {
                        break;
                    }
                }
                else
                {
                    temp += (char)c;
                }
            }
            else if (NEX_RET_STRING_HEAD == c)
            {
                str_start_flag = true;
            }
        }
        
        if (cnt_0xff >= 3)
        {
            break;
        }
    }

    ret = temp.length();
    ret = ret > len ? len : ret;
    strncpy(buffer, temp.c_str(), ret);
    
__return:

    dbSerialPrint("recvRetString[");
    dbSerialPrint(temp.length());
    dbSerialPrint(",");
    dbSerialPrint(temp);
    dbSerialPrintln("]");

    return ret;
}

/*
 * Send command to Nextion.
 *
 * @param cmd - the string of command.
 */
void sendCommand(const char* cmd)
{
    while (nexSerial.available())
    {
        nexSerial.read();
    }
#ifdef NEX_UDP
    Nex_Redirect_to_UDP(cmd);
    Nex_Redirect_to_UDP(0xFF);
    Nex_Redirect_to_UDP(0xFF);
    Nex_Redirect_to_UDP(0xFF);
    dbSerialPrintln(cmd);
#endif
    nexSerial.print(cmd);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    dbSerialPrintln(cmd);
}


/*
 * Command is executed successfully. 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool recvRetCommandFinished(uint32_t timeout)
{    
    bool ret = false;
    uint8_t temp[4] = {0};
    
    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    {
        ret = false;
    }

    if (temp[0] == NEX_RET_CMD_FINISHED
        && temp[1] == 0xFF
        && temp[2] == 0xFF
        && temp[3] == 0xFF
        )
    {
        ret = true;
    }

    if (ret) 
    {
        dbSerialPrintln("recvRetCommandFinished ok");
    }
    else
    {
        dbSerialPrintln("recvRetCommandFinished err");
    }
    
    return ret;
}


bool nexInit(void)
{
    bool ret1 = false;
    bool ret2 = false;
    
    dbSerialBegin(115200);
    //nexSerial.begin(38400);
    nexSerial.begin(115200);
    delay(5);
    sendCommand("");
    sendCommand("bkcmd=3");
    ret1 = recvRetCommandFinished();
    sendCommand("page 0");
    ret2 = recvRetCommandFinished();
    return ret1 && ret2;
}

void nexLoop(NexTouch *nex_listen_list[])
{
    static uint8_t __buffer[10];
    
    uint16_t i;
    uint8_t c;  
    
    while (nexSerial.available() > 0)
    {   
        delay(10);
        c = nexSerial.read();
        
        if (NEX_RET_EVENT_TOUCH_HEAD == c)
        {
            if (nexSerial.available() >= 6)
            {
                __buffer[0] = c;  
                for (i = 1; i < 7; i++)
                {
                    __buffer[i] = nexSerial.read();
                }
                __buffer[i] = 0x00;
                
                
                if (__buffer[1] == page4_ID && __buffer[2] == toMain_ID)  // this is from last page, will change if a page is added or removed.
                    pg = 0;  // pick up the page numbers on events to remove the need to ask for page number for screen refreshes and minimize serial traffic
                else if (__buffer[1] == page0_ID && __buffer[2] == toConfig_ID)  // from page 0 toConfig_ID = 15
                    pg = 1; 
                else if (__buffer[1] == page1_ID && __buffer[2] == toSet1_ID)  // from page 1 toSet1_ID = 6
                    pg = 2; 
                else if (__buffer[1] == page2_ID && __buffer[2] == toPwrGraph_ID)  // from page 2 toPwrGraph_ID = 18
                    pg = 3;  
                else if (__buffer[1] == page3_ID && __buffer[2] == toPowerCal_ID)  // from page 3 toPowerCal_ID = 5
                    pg = 4;                             
                else
                    pg = __buffer[1];  // any other event should be from current page
                
                dbSerialPrint("\n **** Page = ");
                dbSerialPrintln(pg);
                
                if (0xFF == __buffer[4] && 0xFF == __buffer[5] && 0xFF == __buffer[6])
                {
                    NexTouch::iterate(nex_listen_list, __buffer[1], __buffer[2], (int32_t)__buffer[3]);
                }
                
            }
        }
    }
}

