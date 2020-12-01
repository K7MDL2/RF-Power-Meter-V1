#ifndef __NEXTOUCH_H__
#define __NEXTOUCH_H__

#include "NexConfig.h"
#include "NexObject.h"
#include <string.h>
#include "..\util\Utilities.h"

#define NEX_EVENT_PUSH (0x01)
#define NEX_EVENT_POP (0x00)

void NexTouch_iterate(struct NexObject **list, unsigned char pid, unsigned char cid, unsigned long event);

void NexTouch_attachPush(struct NexObject *touch, NexTouchEventCb push, void *ptr);

void NexTouch_detachPush(struct NexObject *touch);

void NexTouch_attachPop(struct NexObject *touch, NexTouchEventCb pop, void *ptr);

void NexTouch_detachPop(struct NexObject *touch);

void NexTouch_push(struct NexObject *touch);
void NexTouch_pop(struct NexObject *touch);

#endif