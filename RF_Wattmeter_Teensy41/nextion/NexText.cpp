#include "NexText.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Utilities.h"

extern char buf[32];
extern char cmd[64];

unsigned int NexText_getText(struct NexObject *text, char *buffer, unsigned int len)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".txt");
    sendCommand(cmd);
    return recvRetString(buffer, len);
}

// For this command, put yoour text into the global string cmd[] and this fntion will use it instead of the passed pointer to buffer which is not working right.
unsigned char NexText_setText(struct NexObject *text)         //, char *buffer)
{    
    strcpy(buf, cmd);  // using *buffer results in endless strcat loop and crash. Thsi is a workaround
    ClearString(cmd);
    strcat(cmd, text->__name);
    strcat(cmd, ".txt=\"");
    strcat(cmd, buf);
    strcat(cmd, "\"");
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_Get_background_color_bco(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".bco");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_Set_background_color_bco(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".bco=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, text->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_Get_font_color_pco(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".pco");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_Set_font_color_pco(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".pco=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, text->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_Get_place_xcen(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".xcen");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_Set_place_xcen(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".xcen=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, text->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_Get_place_ycen(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".ycen");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_Set_place_ycen(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".ycen=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, text->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_getFont(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".font");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_setFont(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".font=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, text->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_Get_background_crop_picc(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".picc");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_Set_background_crop_picc(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".picc=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, text->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

unsigned long NexText_Get_background_image_pic(struct NexObject *text, unsigned long *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, text->__name);
    strcat(cmd, ".pic");
    sendCommand(cmd);
    return recvRetNumber(number);
}

unsigned char NexText_Set_background_image_pic(struct NexObject *text, unsigned long number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, text->__name);
    strcat(cmd, ".pic=");
    strcat(cmd, buf);

    sendCommand(cmd);
    return recvRetCommandFinished();
}