#include "NexPage.h"
#include <string.h>
#include "Utilities.h"

char buffer[5];
extern char cmd[64];

uint8_t NexPage_show(struct NexObject *page)
{

    //ClearString(buffer);
    if (!page->__name)
    {
        return 0;
    }

    ClearString(cmd);
    strcat(cmd, "page ");
    strcat(cmd, page->__name);
    sendCommand(cmd);
    return recvRetCommandFinished();
}