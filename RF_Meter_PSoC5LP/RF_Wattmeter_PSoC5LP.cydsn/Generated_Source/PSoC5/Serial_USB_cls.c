/***************************************************************************//**
* \file Serial_USB_cls.c
* \version 3.20
*
* \brief
*  This file contains the USB Class request handler.
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial_USB_pvt.h"
#include "cyapicallbacks.h"

#if(Serial_USB_EXTERN_CLS == Serial_USB_FALSE)

/***************************************
* User Implemented Class Driver Declarations.
***************************************/
/* `#START USER_DEFINED_CLASS_DECLARATIONS` Place your declaration here */

/* `#END` */


/*******************************************************************************
* Function Name: Serial_USB_DispatchClassRqst
****************************************************************************//**
*  This routine dispatches class specific requests depend on interface class.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_DispatchClassRqst(void) 
{
    uint8 interfaceNumber;
    uint8 requestHandled = Serial_USB_FALSE;

    /* Get interface to which request is intended. */
    switch (Serial_USB_bmRequestTypeReg & Serial_USB_RQST_RCPT_MASK)
    {
        case Serial_USB_RQST_RCPT_IFC:
            /* Class-specific request directed to interface: wIndexLoReg 
            * contains interface number.
            */
            interfaceNumber = (uint8) Serial_USB_wIndexLoReg;
            break;
        
        case Serial_USB_RQST_RCPT_EP:
            /* Class-specific request directed to endpoint: wIndexLoReg contains 
            * endpoint number. Find interface related to endpoint. 
            */
            interfaceNumber = Serial_USB_EP[Serial_USB_wIndexLoReg & Serial_USB_DIR_UNUSED].interface;
            break;
            
        default:
            /* Default interface is zero. */
            interfaceNumber = 0u;
            break;
    }
    
    /* Check that interface is within acceptable range */
    if (interfaceNumber <= Serial_USB_MAX_INTERFACES_NUMBER)
    {
    #if (defined(Serial_USB_ENABLE_HID_CLASS)   || \
         defined(Serial_USB_ENABLE_AUDIO_CLASS) || \
         defined(Serial_USB_ENABLE_CDC_CLASS)   || \
         Serial_USB_ENABLE_MSC_CLASS)

        /* Handle class request depends on interface type. */
        switch (Serial_USB_interfaceClass[interfaceNumber])
        {
        #if defined(Serial_USB_ENABLE_HID_CLASS)
            case Serial_USB_CLASS_HID:
                requestHandled = Serial_USB_DispatchHIDClassRqst();
                break;
        #endif /* (Serial_USB_ENABLE_HID_CLASS) */
                
        #if defined(Serial_USB_ENABLE_AUDIO_CLASS)
            case Serial_USB_CLASS_AUDIO:
                requestHandled = Serial_USB_DispatchAUDIOClassRqst();
                break;
        #endif /* (Serial_USB_CLASS_AUDIO) */
                
        #if defined(Serial_USB_ENABLE_CDC_CLASS)
            case Serial_USB_CLASS_CDC:
                requestHandled = Serial_USB_DispatchCDCClassRqst();
                break;
        #endif /* (Serial_USB_ENABLE_CDC_CLASS) */
            
        #if (Serial_USB_ENABLE_MSC_CLASS)
            case Serial_USB_CLASS_MSD:
            #if (Serial_USB_HANDLE_MSC_REQUESTS)
                /* MSC requests are handled by the component. */
                requestHandled = Serial_USB_DispatchMSCClassRqst();
            #elif defined(Serial_USB_DISPATCH_MSC_CLASS_RQST_CALLBACK)
                /* MSC requests are handled by user defined callbcak. */
                requestHandled = Serial_USB_DispatchMSCClassRqst_Callback();
            #else
                /* MSC requests are not handled. */
                requestHandled = Serial_USB_FALSE;
            #endif /* (Serial_USB_HANDLE_MSC_REQUESTS) */
                break;
        #endif /* (Serial_USB_ENABLE_MSC_CLASS) */
            
            default:
                /* Request is not handled: unknown class request type. */
                requestHandled = Serial_USB_FALSE;
                break;
        }
    #endif /* Class support is enabled */
    }
    
    /* `#START USER_DEFINED_CLASS_CODE` Place your Class request here */

    /* `#END` */

#ifdef Serial_USB_DISPATCH_CLASS_RQST_CALLBACK
    if (Serial_USB_FALSE == requestHandled)
    {
        requestHandled = Serial_USB_DispatchClassRqst_Callback(interfaceNumber);
    }
#endif /* (Serial_USB_DISPATCH_CLASS_RQST_CALLBACK) */

    return (requestHandled);
}


/*******************************************************************************
* Additional user functions supporting Class Specific Requests
********************************************************************************/

/* `#START CLASS_SPECIFIC_FUNCTIONS` Place any additional functions here */

/* `#END` */

#endif /* Serial_USB_EXTERN_CLS */


/* [] END OF FILE */
