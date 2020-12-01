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
#include "Utilities.h"
//#include "NexSerial.h"
#include <stdio.h>
#include <string.h>

#define NEX_RET_CMD_FINISHED (0x01)
#define NEX_RET_EVENT_LAUNCHED (0x88)
#define NEX_RET_EVENT_UPGRADED (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD (0x65)
#define NEX_RET_EVENT_POSITION_HEAD (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD (0x66)
#define NEX_RET_STRING_HEAD (0x70)
#define NEX_RET_NUMBER_HEAD (0x71)
#define NEX_RET_INVALID_CMD (0x00)
#define NEX_RET_INVALID_COMPONENT_ID (0x02)
#define NEX_RET_INVALID_PAGE_ID (0x03)
#define NEX_RET_INVALID_PICTURE_ID (0x04)
#define NEX_RET_INVALID_FONT_ID (0x05)
#define NEX_RET_INVALID_BAUD (0x11)
#define NEX_RET_INVALID_VARIABLE (0x1A)
#define NEX_RET_INVALID_OPERATION (0x1B)

extern unsigned int DLY;  // Set the value in LoRaCfg so only used if LoRa config is used
// Delay value is critical. For 19.2K 80-100 works good.  9600 500 is good.
// Less and you start missing events, numbers come back 0.  After a while LoRa data gets backed up, missing events
// Need flow control on the Nextion end likely.

char txt[32];
char cmd[64];
char buf[12];
char msg[32];
unsigned char wrData;
extern unsigned char WAIT;  // traffic flag - if a function is waiting for a response for the display set this flag to hold off display updates
extern unsigned char pg;
extern unsigned int DLY;  // Set the value to prevent missed messages. 50 for wired, 500 for LoRa set in LoRaCfg

/*
 * Receive current page number. 
 * 
 * @param number - save unsigned long data. 
 * @param 100 - set 100 time. 
 *
 * @retval 1 - success. 
 * @retval 0 - failed.
 *
 */
unsigned char recvPageNumber(unsigned char *number)
{
    unsigned char ret = 0;
    char temp[5];

    if (!*number)
    {
        goto __return;
    }

    nexDelay(DLY);    // 20 is good for direct connect display.  Some other value needed for slow link like LoRa radio 
   
    if (5 != nexSerial_readBytes((char *)temp, 5))  // set the number of bytes for message length we are looking for
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_CURRENT_PAGE_ID_HEAD && temp[2] == 0xFF && temp[3] == 0xFF && temp[4] == 0xFF)
    {
        *number = temp[1];
        ret = 1;
    }

__return:

    /*if (ret) 
    {
        dbSerialPrint("recvRetPage :");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvRetPage err");
    }*/

    return ret;
}


/*
 * Receive unsigned long data. 
 * 
 * @param number - save unsigned long data. 
 * @param 100 - set 100 time. 
 *
 * @retval 1 - success. 
 * @retval 0 - failed.
 *
 */
unsigned char recvRetNumber(unsigned long *number)
{
    unsigned char ret = 0;
    char temp[8];

    if (!number)
    {
        goto __return;
    }
    //WAIT = 1;
    nexDelay(DLY);      // This delay is very sensitive.  20 seems to be about right for direct connected display. 
                        // for a slower update rate such as over LoRa radio might need something else.
    if (8 != nexSerial_readBytes((char *)temp, 8))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_NUMBER_HEAD && temp[5] == 0xFF && temp[6] == 0xFF && temp[7] == 0xFF)
    {
        *number = ((unsigned long)temp[4] << 24) | ((unsigned long)temp[3] << 16) | (temp[2] << 8) | (temp[1]);
        ret = 1;
    }

__return:

    /*if (ret) 
    {
        dbSerialPrint("recvRetNumber :");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvRetNumber err");
    }*/
    WAIT = 0;
    return ret;
}

/*
 * Receive string data. 
 * 
 * @param buffer - save string data. 
 * @param len - string buffer length. 
 * @param 100 - set 100 time. 
 *
 * @return the length of string buffer.
 *
 */
unsigned int recvRetString(char *buffer, unsigned int len)
{
    unsigned int ret = 0;
    unsigned char str_start_flag = 0;
    unsigned char cnt_0xff = 0;
    char arr[32];
    char *temp = arr;
    unsigned char c = 0;
    long start;

    if (!buffer || len == 0)
    {
        goto __return;
    }

    nexDelay(DLY);      // This delay is very sensitive.  20 seems to be about right for direct connected display. 
                        // for a slower update rate such as over LoRa radio might need something else.
    start = 500;
    while (start)
    {
        while (nexSerial_available())
        {
            c = nexSerial_read();
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
                str_start_flag = 1;
            }
        }

        if (cnt_0xff >= 3)
        {
            break;
        }
        --start;
    }

    ret = ArrayLength(temp);
    ret = ret > len ? len : ret;
    strncpy(buffer, temp, ret);

__return:
    /*
    dbSerialPrint("recvRetString[");
    dbSerialPrint(temp.length());
    dbSerialPrint(",");
    dbSerialPrint(temp);
    dbSerialPrintln("]");
     */
    return ret;
}

/*
 * Command is executed successfully. 
 *
 * @param 100 - set 100 time.
 *
 * @retval 1 - success.
 * @retval 0 - failed. 
 *
 */
unsigned char recvRetCommandFinished()
{
    unsigned char ret = 0;
    char temp[4] = {0};
    
    return 1;  // if Result messages are turned off with bkcmd = 0 in the Nextion then no success msg will ever come so just return success.
        
    // nexSerial.set100(100);
    nexDelay(DLY);
    WAIT = 1;
    
    if (ArrayLength(temp) != nexSerial_readBytes((char *)temp, ArrayLength(temp)))
    {
        ret = 0;
    }

    if (temp[0] == NEX_RET_CMD_FINISHED && temp[1] == 0xFF && temp[2] == 0xFF && temp[3] == 0xFF)
    {
        ret = 1;
    }
    /*
    if (ret) 
    {
        dbSerialPrintln("recvRetCommandFinished ok");
    }
    else
    {
        dbSerialPrintln("recvRetCommandFinished err");
    }
     */
    WAIT = 0;
    return ret;
}

void sendCommand(char *scmd)
{
    scmd += 1;  // get rid of unused parm compiler message.
    // not using command var, maybe a compiler bug resulting in truncating passed strings to the first char only.
    sprintf(cmd, "%s%c%c%c", cmd, 255, 255, 255);
    Serial_Print(cmd);
}

unsigned char nexInit(void)
{
    unsigned char ret1 = 0;
    unsigned char ret2 = 0;

    //dbSerialBegin(9600);
    nexSerial_init();
    sendCommand("");
    sendCommand("bkcmd=1");
    ret1 = recvRetCommandFinished();
    sendCommand("page 0");
    ret2 = recvRetCommandFinished();
    return ret1 && ret2;
}

void nexLoop(struct NexObject *nex_listen_list[])
{
    static unsigned char __buffer[10];

    unsigned int i;
    unsigned char c;

    while (nexSerial_available() > 0)
    {
        c = nexSerial_read();
        if (c == (unsigned char) -1)
            return;   
        
        if (NEX_RET_EVENT_TOUCH_HEAD == c)
        {
            if (nexSerial_available() >= 6)
            {
                __buffer[0] = c;
                for (i = 1; i < 7; i++)
                {
                    __buffer[i] = nexSerial_read();
                }
                __buffer[i] = 0x00;
                
                if (__buffer[1] == 0x04 && __buffer[2] == 0x01)  // this is from last page, will chage if a page is added or removed.
                    pg = 0;  // pick up the page numbers on events to remove the need to ask for page number for screen refreshes and minimize serial traffic
                else if (__buffer[1] == 0x00 && __buffer[2] == 0x01)
                    pg = 1;  // left page 0
                else
                    pg = __buffer[1];   // all pages between 0 and 4

                if (0xFF == __buffer[4] && 0xFF == __buffer[5] && 0xFF == __buffer[6])
                {
                    NexTouch_iterate(nex_listen_list, __buffer[1], __buffer[2], (int32_t)__buffer[3]);
                }
            }
        }
    }
}