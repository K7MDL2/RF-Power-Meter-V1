/***************************************************************************//**
* \file Serial_USB_vnd.c
* \version 3.20
*
* \brief
*  This file contains the  USB vendor request handler.
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

#if(Serial_USB_EXTERN_VND == Serial_USB_FALSE)

/***************************************
* Vendor Specific Declarations
***************************************/

/* `#START VENDOR_SPECIFIC_DECLARATIONS` Place your declaration here */

/* `#END` */


/*******************************************************************************
* Function Name: Serial_USB_HandleVendorRqst
****************************************************************************//**
*
*  This routine provide users with a method to implement vendor specific
*  requests.
*
*  To implement vendor specific requests, add your code in this function to
*  decode and disposition the request.  If the request is handled, your code
*  must set the variable "requestHandled" to TRUE, indicating that the
*  request has been handled.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_HandleVendorRqst(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;

    /* Check request direction: D2H or H2D. */
    if (0u != (Serial_USB_bmRequestTypeReg & Serial_USB_RQST_DIR_D2H))
    {
        /* Handle direction from device to host. */
        
        switch (Serial_USB_bRequestReg)
        {
            case Serial_USB_GET_EXTENDED_CONFIG_DESCRIPTOR:
            #if defined(Serial_USB_ENABLE_MSOS_STRING)
                Serial_USB_currentTD.pData = (volatile uint8 *) &Serial_USB_MSOS_CONFIGURATION_DESCR[0u];
                Serial_USB_currentTD.count = Serial_USB_MSOS_CONFIGURATION_DESCR[0u];
                requestHandled  = Serial_USB_InitControlRead();
            #endif /* (Serial_USB_ENABLE_MSOS_STRING) */
                break;
            
            default:
                break;
        }
    }

    /* `#START VENDOR_SPECIFIC_CODE` Place your vendor specific request here */

    /* `#END` */

#ifdef Serial_USB_HANDLE_VENDOR_RQST_CALLBACK
    if (Serial_USB_FALSE == requestHandled)
    {
        requestHandled = Serial_USB_HandleVendorRqst_Callback();
    }
#endif /* (Serial_USB_HANDLE_VENDOR_RQST_CALLBACK) */

    return (requestHandled);
}


/*******************************************************************************
* Additional user functions supporting Vendor Specific Requests
********************************************************************************/

/* `#START VENDOR_SPECIFIC_FUNCTIONS` Place any additional functions here */

/* `#END` */


#endif /* Serial_USB_EXTERN_VND */


/* [] END OF FILE */
