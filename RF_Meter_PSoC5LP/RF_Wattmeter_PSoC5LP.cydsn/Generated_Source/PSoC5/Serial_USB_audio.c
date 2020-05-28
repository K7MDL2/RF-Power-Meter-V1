/***************************************************************************//**
* \file Serial_USB_audio.c
* \version 3.20
*
* \brief
*  This file contains the USB AUDIO Class request handler.
*
* Related Document:
*  Universal Serial Bus Device Class Definition for Audio Devices Release 1.0
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial_USB_audio.h"
#include "Serial_USB_pvt.h"
#include "cyapicallbacks.h"

#if defined(Serial_USB_ENABLE_AUDIO_CLASS)

/***************************************
* Custom Declarations
***************************************/

/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */


#if !defined(USER_SUPPLIED_AUDIO_HANDLER)

/***************************************
*    AUDIO Variables
***************************************/

#if defined(Serial_USB_ENABLE_AUDIO_STREAMING)
    /** Contains the current audio sample frequency. It is set by the host using a SET_CUR request to the endpoint.*/
    volatile uint8 Serial_USB_currentSampleFrequency[Serial_USB_MAX_EP][Serial_USB_SAMPLE_FREQ_LEN];
    /** Used as a flag for the user code, to inform it that the host has been sent a request 
     * to change the sample frequency. The sample frequency will be sent on the next OUT transaction. 
     * It contains the endpoint address when set. The following code is recommended for 
     * detecting new sample frequency in main code:
     * \snippet  /USBFS_sut_02.cydsn/main.c Detecting new Sample Frequency
     *     
     * The USBFS_transferState variable is checked to make sure that the transfer completes. */
    volatile uint8 Serial_USB_frequencyChanged;
    /** Contains the mute configuration set by the host.*/
    volatile uint8 Serial_USB_currentMute;
    /** Contains the volume level set by the host.*/
    volatile uint8 Serial_USB_currentVolume[Serial_USB_VOLUME_LEN];
    volatile uint8 Serial_USB_minimumVolume[Serial_USB_VOLUME_LEN] = {Serial_USB_VOL_MIN_LSB,
                                                                                  Serial_USB_VOL_MIN_MSB};
    volatile uint8 Serial_USB_maximumVolume[Serial_USB_VOLUME_LEN] = {Serial_USB_VOL_MAX_LSB,
                                                                                  Serial_USB_VOL_MAX_MSB};
    volatile uint8 Serial_USB_resolutionVolume[Serial_USB_VOLUME_LEN] = {Serial_USB_VOL_RES_LSB,
                                                                                     Serial_USB_VOL_RES_MSB};
#endif /*  Serial_USB_ENABLE_AUDIO_STREAMING */


/*******************************************************************************
* Function Name: Serial_USB_DispatchAUDIOClassRqst
****************************************************************************//**
*
*  This routine dispatches class requests
*
* \return
*  Results of Audio Class request handling: 
*  - Serial_USB_TRUE  - request was handled without errors
*  - Serial_USB_FALSE - error occurs during handling of request     
*
* \globalvars
*   Serial_USB_currentSampleFrequency: Contains the current audio Sample
*       Frequency. It is set by the Host using SET_CUR request to the endpoint.
*   Serial_USB_frequencyChanged: This variable is used as a flag for the
*       user code, to be aware that Host has been sent request for changing
*       Sample Frequency. Sample frequency will be sent on the next OUT
*       transaction. It is contains endpoint address when set. The following
*       code is recommended for detecting new Sample Frequency in main code:
*       
*  \snippet  /USBFS_sut_02.cydsn/main.c Detecting new Sample Frequency
*      
*   Serial_USB_transferState variable is checked to be sure that transfer
*              completes.
*   Serial_USB_currentMute: Contains mute configuration set by Host.
*   Serial_USB_currentVolume: Contains volume level set by Host.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_DispatchAUDIOClassRqst(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;
    
    uint8 RqstRcpt = (uint8)(Serial_USB_bmRequestTypeReg & Serial_USB_RQST_RCPT_MASK);
#if defined(Serial_USB_ENABLE_AUDIO_STREAMING)
    uint8 wValueHi = (uint8) Serial_USB_wValueHiReg;
    uint8 epNumber = (uint8) Serial_USB_wIndexLoReg & Serial_USB_DIR_UNUSED;
#endif /* (Serial_USB_ENABLE_AUDIO_STREAMING) */
    
    /* Check request direction: D2H or H2D. */
    if (0u != (Serial_USB_bmRequestTypeReg & Serial_USB_RQST_DIR_D2H))
    {
        /* Handle direction from device to host. */
        
        if (Serial_USB_RQST_RCPT_EP == RqstRcpt)
        {
            /* Request recipient is to endpoint. */
            switch (Serial_USB_bRequestReg)
            {
                case Serial_USB_GET_CUR:
                #if defined(Serial_USB_ENABLE_AUDIO_STREAMING)
                    if (wValueHi == Serial_USB_SAMPLING_FREQ_CONTROL)
                    {
                         /* point Control Selector is Sampling Frequency */
                        Serial_USB_currentTD.wCount = Serial_USB_SAMPLE_FREQ_LEN;
                        Serial_USB_currentTD.pData  = Serial_USB_currentSampleFrequency[epNumber];
                        
                        requestHandled   = Serial_USB_InitControlRead();
                    }
                #endif /* (Serial_USB_ENABLE_AUDIO_STREAMING) */
                
                    /* `#START AUDIO_READ_REQUESTS` Place other request handler here */

                    /* `#END` */
                
                #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_READ_REQUESTS_CALLBACK    
                    Serial_USB_DispatchAUDIOClass_AUDIO_READ_REQUESTS_Callback();
                #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_READ_REQUESTS_CALLBACK) */                   
                break;
                
                default:
                    /* Do not handle this request unless callback is defined. */
                    break;
            }
        
        }
        else if (Serial_USB_RQST_RCPT_IFC == RqstRcpt)
        {
            /* Request recipient is interface or entity ID. */
            switch (Serial_USB_bRequestReg)
            {
                case Serial_USB_GET_CUR:
                #if defined(Serial_USB_ENABLE_AUDIO_STREAMING)
                    if (wValueHi == Serial_USB_MUTE_CONTROL)
                    {
                        /* `#START MUTE_CONTROL_GET_REQUEST` Place multi-channel handler here */

                        /* `#END` */

                    #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_MUTE_CONTROL_GET_REQUEST_CALLBACK
                        Serial_USB_DispatchAUDIOClass_MUTE_CONTROL_GET_REQUEST_Callback();
                    #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_MUTE_CONTROL_GET_REQUEST_CALLBACK) */

                        /* Entity ID Control Selector is MUTE */
                        Serial_USB_currentTD.wCount = 1u;
                        Serial_USB_currentTD.pData  = &Serial_USB_currentMute;
                        
                        requestHandled = Serial_USB_InitControlRead();
                    }
                    else if (wValueHi == Serial_USB_VOLUME_CONTROL)
                    {
                        /* `#START VOLUME_CONTROL_GET_REQUEST` Place multi-channel handler here */

                        /* `#END` */

                    #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_VOLUME_CONTROL_GET_REQUEST_CALLBACK
                        Serial_USB_DispatchAUDIOClass_VOLUME_CONTROL_GET_REQUEST_Callback();
                    #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_VOLUME_CONTROL_GET_REQUEST_CALLBACK) */

                        /* Entity ID Control Selector is VOLUME, */
                        Serial_USB_currentTD.wCount = Serial_USB_VOLUME_LEN;
                        Serial_USB_currentTD.pData  = Serial_USB_currentVolume;
                        
                        requestHandled = Serial_USB_InitControlRead();
                    }
                    else
                    {
                        /* `#START OTHER_GET_CUR_REQUESTS` Place other request handler here */

                        /* `#END` */

                    #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_OTHER_GET_CUR_REQUESTS_CALLBACK
                        Serial_USB_DispatchAUDIOClass_OTHER_GET_CUR_REQUESTS_Callback();
                    #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_OTHER_GET_CUR_REQUESTS_CALLBACK) */
                    }
                    break;
                    
                case Serial_USB_GET_MIN:
                    if (wValueHi == Serial_USB_VOLUME_CONTROL)
                    {
                        /* Entity ID Control Selector is VOLUME, */
                        Serial_USB_currentTD.wCount = Serial_USB_VOLUME_LEN;
                        Serial_USB_currentTD.pData  = &Serial_USB_minimumVolume[0];
                        
                        requestHandled = Serial_USB_InitControlRead();
                    }
                    break;
                    
                case Serial_USB_GET_MAX:
                    if (wValueHi == Serial_USB_VOLUME_CONTROL)
                    {
                        /* Entity ID Control Selector is VOLUME, */
                        Serial_USB_currentTD.wCount = Serial_USB_VOLUME_LEN;
                        Serial_USB_currentTD.pData  = &Serial_USB_maximumVolume[0];
                        
                        requestHandled = Serial_USB_InitControlRead();
                    }
                    break;
                    
                case Serial_USB_GET_RES:
                    if (wValueHi == Serial_USB_VOLUME_CONTROL)
                    {
                         /* Entity ID Control Selector is VOLUME, */
                        Serial_USB_currentTD.wCount = Serial_USB_VOLUME_LEN;
                        Serial_USB_currentTD.pData  = &Serial_USB_resolutionVolume[0];
                        
                        requestHandled   = Serial_USB_InitControlRead();
                    }
                    break;
                    
                /* The contents of the status message is reserved for future use.
                * For the time being, a null packet should be returned in the data stage of the
                * control transfer, and the received null packet should be ACKed.
                */
                case Serial_USB_GET_STAT:
                    Serial_USB_currentTD.wCount = 0u;    
                    
                    requestHandled = Serial_USB_InitControlWrite();

                #endif /* (Serial_USB_ENABLE_AUDIO_STREAMING) */
                
                    /* `#START AUDIO_WRITE_REQUESTS` Place other request handler here */

                    /* `#END` */
                
                #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_WRITE_REQUESTS_CALLBACK
                    Serial_USB_DispatchAUDIOClass_AUDIO_WRITE_REQUESTS_Callback();
                #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_WRITE_REQUESTS_CALLBACK) */
                    break;
                
                default:
                    /* Do not handle this request. */
                    break;
            }
        }
        else
        {   
            /* Do not handle other requests recipients. */
        }
    }
    else
    {
        /* Handle direction from host to device. */
        
        if (Serial_USB_RQST_RCPT_EP == RqstRcpt)
        {
            /* Request recipient is endpoint. */
            switch (Serial_USB_bRequestReg)
            {
                case Serial_USB_SET_CUR:
                #if defined(Serial_USB_ENABLE_AUDIO_STREAMING)
                    if (wValueHi == Serial_USB_SAMPLING_FREQ_CONTROL)
                    {
                         /* point Control Selector is Sampling Frequency */
                        Serial_USB_currentTD.wCount = Serial_USB_SAMPLE_FREQ_LEN;
                        Serial_USB_currentTD.pData  = Serial_USB_currentSampleFrequency[epNumber];
                        Serial_USB_frequencyChanged = (uint8) epNumber;
                        
                        requestHandled   = Serial_USB_InitControlWrite();
                    }
                #endif /* (Serial_USB_ENABLE_AUDIO_STREAMING) */

                    /* `#START AUDIO_SAMPLING_FREQ_REQUESTS` Place other request handler here */

                    /* `#END` */

                #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_SAMPLING_FREQ_REQUESTS_CALLBACK
                    Serial_USB_DispatchAUDIOClass_AUDIO_SAMPLING_FREQ_REQUESTS_Callback();
                #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_SAMPLING_FREQ_REQUESTS_CALLBACK) */
                    break;
                
                default:
                    /* Do not handle this request. */
                    break;
            }
        }
        else if(Serial_USB_RQST_RCPT_IFC == RqstRcpt)
        {
            /* Request recipient is interface or entity ID. */
            switch (Serial_USB_bRequestReg)
            {
                case Serial_USB_SET_CUR:
                #if defined(Serial_USB_ENABLE_AUDIO_STREAMING)
                    if (wValueHi == Serial_USB_MUTE_CONTROL)
                    {
                        /* `#START MUTE_SET_REQUEST` Place multi-channel handler here */

                        /* `#END` */

                    #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_MUTE_SET_REQUEST_CALLBACK
                        Serial_USB_DispatchAUDIOClass_MUTE_SET_REQUEST_Callback();
                    #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_MUTE_SET_REQUEST_CALLBACK) */

                        /* Entity ID Control Selector is MUTE */
                        Serial_USB_currentTD.wCount = 1u;
                        Serial_USB_currentTD.pData  = &Serial_USB_currentMute;
                        
                        requestHandled = Serial_USB_InitControlWrite();
                    }
                    else if (wValueHi == Serial_USB_VOLUME_CONTROL)
                    {
                        /* `#START VOLUME_CONTROL_SET_REQUEST` Place multi-channel handler here */

                        /* `#END` */

                    #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_VOLUME_CONTROL_SET_REQUEST_CALLBACK
                        Serial_USB_DispatchAUDIOClass_VOLUME_CONTROL_SET_REQUEST_Callback();
                    #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_VOLUME_CONTROL_SET_REQUEST_CALLBACK) */

                        /* Entity ID Control Selector is VOLUME */
                        Serial_USB_currentTD.wCount = Serial_USB_VOLUME_LEN;
                        Serial_USB_currentTD.pData  = Serial_USB_currentVolume;
                        
                        requestHandled = Serial_USB_InitControlWrite();
                    }
                    else
                    {
                        /* `#START OTHER_SET_CUR_REQUESTS` Place other request handler here */

                        /* `#END` */

                    #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_OTHER_SET_CUR_REQUESTS_CALLBACK
                        Serial_USB_DispatchAUDIOClass_OTHER_SET_CUR_REQUESTS_Callback();
                    #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_OTHER_SET_CUR_REQUESTS_CALLBACK) */
                    }
                #endif /*  Serial_USB_ENABLE_AUDIO_STREAMING */
                
                
                    /* `#START AUDIO_CONTROL_SEL_REQUESTS` Place other request handler here */

                    /* `#END` */
                    
                #ifdef Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_CONTROL_SEL_REQUESTS_CALLBACK
                    Serial_USB_DispatchAUDIOClass_AUDIO_CONTROL_SEL_REQUESTS_Callback();
                #endif /* (Serial_USB_DISPATCH_AUDIO_CLASS_AUDIO_CONTROL_SEL_REQUESTS_CALLBACK) */
                break;

                default:
                    /* Do not handle this request. */
                break;
            }
        }
        else
        {
            /* Do not handle other requests recipients. */
        }
    }

    return (requestHandled);
}
#endif /* (USER_SUPPLIED_AUDIO_HANDLER) */


/*******************************************************************************
* Additional user functions supporting AUDIO Requests
********************************************************************************/

/* `#START AUDIO_FUNCTIONS` Place any additional functions here */

/* `#END` */

#endif  /* (Serial_USB_ENABLE_AUDIO_CLASS) */


/* [] END OF FILE */
