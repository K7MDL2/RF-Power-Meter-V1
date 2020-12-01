#include "NexTouch.h"
#include "Utilities.h"

NexTouchEventCb __cb_push;
void *__cbpush_ptr;
NexTouchEventCb __cb_pop;
void *__cbpop_ptr;

void NexTouch_attachPush(struct NexObject *touch, NexTouchEventCb push, void *ptr)
{
    touch->__cb_push = push;
    touch->__cbpush_ptr = ptr;
}

void NexTouch_detachPush(struct NexObject *touch)
{
    touch->__cb_push = 0;
    touch->__cbpush_ptr = 0;
}

void NexTouch_attachPop(struct NexObject *touch, NexTouchEventCb pop, void *ptr)
{
    touch->__cb_pop = pop;
    touch->__cbpop_ptr = ptr;
}

void NexTouch_detachPop(struct NexObject *touch)
{
    touch->__cb_pop = 0;
    touch->__cbpop_ptr = 0;
}

void NexTouch_push(struct NexObject *touch)
{
    if (touch->__cb_push)
    {
        touch->__cb_push(__cbpush_ptr);
    }
}

void NexTouch_pop(struct NexObject *touch)
{
    if (touch->__cb_pop)
    {
        touch->__cb_pop(__cbpop_ptr);
    }
}

void NexTouch_iterate(struct NexObject **list, unsigned char pid, unsigned char cid, unsigned long event)
{
    struct NexObject *e = 0;
    uint16_t i = 0;

    if (0 == list)
    {
        return;
    }

    for (i = 0; (e = list[i]) != 0; i++)
    {
        if (e->__pid == pid && e->__cid == cid)
        {
            //e->printObjInfo();
            if (NEX_EVENT_PUSH == event)
            {
                if (e->__cb_push)
                {
                    e->__cb_push(e->__cbpush_ptr);
                }
            }
            else if (NEX_EVENT_POP == event)
            {
                if (e->__cb_pop)
                {
                    e->__cb_pop(e->__cbpop_ptr);
                }
            }

            break;
        }
    }
}