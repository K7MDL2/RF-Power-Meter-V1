#include "NexSlider.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Utilities.h"

extern char buf[12];
extern char cmd[64];

uint8_t NexSlider_getValue(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".val");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_setValue(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".val=");
    strcat(cmd, buf);

    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexSlider_Get_background_color_bco(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".bco");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_Set_background_color_bco(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".bco=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, slider->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexSlider_Get_font_color_pco(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".pco");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_Set_font_color_pco(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".pco=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, slider->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexSlider_Get_pointer_thickness_wid(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".wid");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_Set_pointer_thickness_wid(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".wid=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, slider->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexSlider_Get_cursor_height_hig(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".hig");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_Set_cursor_height_hig(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".hig=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, slider->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexSlider_getMaxval(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".maxval");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_setMaxval(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".maxval=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, slider->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}

uint32_t NexSlider_getMinval(struct NexObject *slider, uint32_t *number)
{
    ClearString(cmd);
    strcat(cmd, "get ");
    strcat(cmd, slider->__name);
    strcat(cmd, ".minval");
    sendCommand(cmd);
    return recvRetNumber(number);
}

uint8_t NexSlider_setMinval(struct NexObject *slider, uint32_t number)
{
    ClearString(buf);
    ClearString(cmd);

    utoa(number, buf, 10);
    strcat(cmd, slider->__name);
    strcat(cmd, ".minval=");
    strcat(cmd, buf);
    sendCommand(cmd);

    ClearString(cmd);
    strcat(cmd, "ref ");
    strcat(cmd, slider->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}