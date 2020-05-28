/***************************************************************************//**
* \file Serial_USB_cdc.c
* \version 3.20
*
* \brief
*  This file contains the USB MSC Class request handler and global API for MSC 
*  class.
*
* Related Document:
*  Universal Serial Bus Class Definitions for Communication Devices Version 1.1
*
********************************************************************************
* \copyright
* Copyright 2012-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial_USB_msc.h"
#include "Serial_USB_pvt.h"
#include "cyapicallbacks.h"

#if (Serial_USB_HANDLE_MSC_REQUESTS)

/***************************************
*          Internal variables
***************************************/

static uint8 Serial_USB_lunCount = Serial_USB_MSC_LUN_NUMBER;


/*******************************************************************************
* Function Name: Serial_USB_DispatchMSCClassRqst
****************************************************************************//**
*   
*  \internal 
*  This routine dispatches MSC class requests.
*
* \return
*  Status of request processing: handled or not handled.
*
* \globalvars
*  Serial_USB_lunCount - stores number of LUN (logical units).
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_DispatchMSCClassRqst(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;
    
    /* Get request data. */
    uint16 value  = Serial_USB_GET_UINT16(Serial_USB_wValueHiReg,  Serial_USB_wValueLoReg);
    uint16 dataLength = Serial_USB_GET_UINT16(Serial_USB_wLengthHiReg, Serial_USB_wLengthLoReg);
       
    /* Check request direction: D2H or H2D. */
    if (0u != (Serial_USB_bmRequestTypeReg & Serial_USB_RQST_DIR_D2H))
    {
        /* Handle direction from device to host. */
        
        if (Serial_USB_MSC_GET_MAX_LUN == Serial_USB_bRequestReg)
        {
            /* Check request fields. */
            if ((value  == Serial_USB_MSC_GET_MAX_LUN_WVALUE) &&
                (dataLength == Serial_USB_MSC_GET_MAX_LUN_WLENGTH))
            {
                /* Reply to Get Max LUN request: setup control read. */
                Serial_USB_currentTD.pData = &Serial_USB_lunCount;
                Serial_USB_currentTD.count =  Serial_USB_MSC_GET_MAX_LUN_WLENGTH;
                
                requestHandled  = Serial_USB_InitControlRead();
            }
        }
    }
    else
    {
        /* Handle direction from host to device. */
        
        if (Serial_USB_MSC_RESET == Serial_USB_bRequestReg)
        {
            /* Check request fields. */
            if ((value  == Serial_USB_MSC_RESET_WVALUE) &&
                (dataLength == Serial_USB_MSC_RESET_WLENGTH))
            {
                /* Handle to Bulk-Only Reset request: no data control transfer. */
                Serial_USB_currentTD.count = Serial_USB_MSC_RESET_WLENGTH;
                
            #ifdef Serial_USB_DISPATCH_MSC_CLASS_MSC_RESET_RQST_CALLBACK
                Serial_USB_DispatchMSCClass_MSC_RESET_RQST_Callback();
            #endif /* (Serial_USB_DISPATCH_MSC_CLASS_MSC_RESET_RQST_CALLBACK) */
                
                requestHandled = Serial_USB_InitNoDataControlTransfer();
            }
        }
    }
    
    return (requestHandled);
}


/*******************************************************************************
* Function Name: Serial_USB_MSC_SetLunCount
****************************************************************************//**
*
*  This function sets the number of logical units supported in the application. 
*  The default number of logical units is set in the component customizer.
*
*  \param lunCount: Count of the logical units. Valid range is between 1 and 16.
*
*
* \globalvars
*  Serial_USB_lunCount - stores number of LUN (logical units).
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_MSC_SetLunCount(uint8 lunCount) 
{
    Serial_USB_lunCount = (lunCount - 1u);
}


/*******************************************************************************
* Function Name: Serial_USB_MSC_GetLunCount
****************************************************************************//**
*
*  This function returns the number of logical units.
*
* \return
*   Number of the logical units.
*
* \globalvars
*  Serial_USB_lunCount - stores number of LUN (logical units).
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_MSC_GetLunCount(void) 
{
    return (Serial_USB_lunCount + 1u);
}   

#endif /* (Serial_USB_HANDLE_MSC_REQUESTS) */


/* [] END OF FILE */
