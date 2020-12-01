/**
 * @file NexButton.cpp
 *
 * The implementation of class NexButton. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/13
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software); you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation); either version 2 of
 * the License, or (at your option) any later version.
 */

#include "NexButton.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Utilities.h"

extern char cmd[64];
extern char buf[12];

uint16_t NexButton_getText(struct NexObject *button, char *buffer, uint16_t len)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".txt");
    sendCommand(cmd);
    return recvRetString(buffer, len);
}

uint8_t NexButton_setText(struct NexObject *button, const char *buffer)
{
    ClearString(cmd);
    strcat(cmd, button->__name);
    strcat(cmd, ".txt=\"");
    strcat(cmd, (char*)buffer);
    strcat(cmd, "\"");
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_background_color_bco(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".bco");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_background_color_bco(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".bco=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_press_background_color_bco2(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".bco2");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_press_background_color_bco2(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".bco2=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_font_color_pco(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".pco");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_font_color_pco(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".pco=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_press_font_color_pco2(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".pco2");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_press_font_color_pco2(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".pco2=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_place_xcen(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".xcen");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_place_xcen(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf,10);
    strcat(cmd, button->__name);
    strcat(cmd, ".xcen=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_place_ycen(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".ycen");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_place_ycen(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".ycen=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_getFont(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".font");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_setFont(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".font=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_background_cropi_picc(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".picc");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_background_crop_picc(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".picc=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_press_background_crop_picc2(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".picc2");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_press_background_crop_picc2(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".picc2=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_background_image_pic(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".pic");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_background_image_pic(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".pic=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexButton_Get_press_background_image_pic2(struct NexObject *button, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, button->__name);
    strcat(cmd, ".pic2");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexButton_Set_press_background_image_pic2(struct NexObject *button, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, button->__name);
    strcat(cmd, ".pic2=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, button->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}