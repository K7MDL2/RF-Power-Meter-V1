#ifndef __NEXHARDWARE_H__
#define __NEXHARDWARE_H__
#include "NexConfig.h"
#include "NexTouch.h"
#include "NexObject.h"
/**
 * Init Nextion.  
 * 
 * @return true if success, false for failure. 
 */
unsigned char nexInit(void);

/**
 * Listen touch event and calling callbacks attached before.
 * 
 * Supports push and pop at present. 
 *
 * @param nex_listen_list - index to Nextion Components list. 
 * @return none. 
 *
 * @warning This function must be called repeatedly to response touch events
 *  from Nextion touch panel. Actually, you should place it in your loop function. 
 */
void nexLoop(struct NexObject *nex_listen_list[]);

/**
 * @}
 */
//timeout=100
unsigned char recvPageNumber(unsigned char *number);
unsigned char recvRetNumber(unsigned long *number);
unsigned int recvRetString(char *buffer, unsigned int len);
unsigned char recvRetCommandFinished();
void sendCommand(char *command);

#define CreateNexObject(obj, pid, id, name) \
    obj.__pid = pid;                        \
    obj.__cid = id;                         \
    StringCopy(obj.__name, name)

#endif /* #ifndef __NEXHARDWARE_H__ */