/***************************************************************************//**
* \file Serial_USB_std.c
* \version 3.20
*
* \brief
*  This file contains the USB Standard request handler.
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial_USB_pvt.h"

/***************************************
*   Static data allocation
***************************************/

#if defined(Serial_USB_ENABLE_FWSN_STRING)
    static volatile uint8* Serial_USB_fwSerialNumberStringDescriptor;
    static volatile uint8  Serial_USB_snStringConfirm = Serial_USB_FALSE;
#endif  /* (Serial_USB_ENABLE_FWSN_STRING) */

#if defined(Serial_USB_ENABLE_FWSN_STRING)
    /***************************************************************************
    * Function Name: Serial_USB_SerialNumString
    ************************************************************************//**
    *
    *  This function is available only when the User Call Back option in the 
    *  Serial Number String descriptor properties is selected. Application 
    *  firmware can provide the source of the USB device serial number string 
    *  descriptor during run time. The default string is used if the application 
    *  firmware does not use this function or sets the wrong string descriptor.
    *
    *  \param snString:  Pointer to the user-defined string descriptor. The 
    *  string descriptor should meet the Universal Serial Bus Specification 
    *  revision 2.0 chapter 9.6.7
    *  Offset|Size|Value|Description
    *  ------|----|------|---------------------------------
    *  0     |1   |N     |Size of this descriptor in bytes
    *  1     |1   |0x03  |STRING Descriptor Type
    *  2     |N-2 |Number|UNICODE encoded string
    *  
    * *For example:* uint8 snString[16]={0x0E,0x03,'F',0,'W',0,'S',0,'N',0,'0',0,'1',0};
    *
    * \reentrant
    *  No.
    *
    ***************************************************************************/
    void  Serial_USB_SerialNumString(uint8 snString[]) 
    {
        Serial_USB_snStringConfirm = Serial_USB_FALSE;
        
        if (snString != NULL)
        {
            /* Check descriptor validation */
            if ((snString[0u] > 1u) && (snString[1u] == Serial_USB_DESCR_STRING))
            {
                Serial_USB_fwSerialNumberStringDescriptor = snString;
                Serial_USB_snStringConfirm = Serial_USB_TRUE;
            }
        }
    }
#endif  /* Serial_USB_ENABLE_FWSN_STRING */


/*******************************************************************************
* Function Name: Serial_USB_HandleStandardRqst
****************************************************************************//**
*
*  This Routine dispatches standard requests
*
*
* \return
*  TRUE if request handled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_HandleStandardRqst(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;
    uint8 interfaceNumber;
    uint8 configurationN;
    uint8 bmRequestType = Serial_USB_bmRequestTypeReg;

#if defined(Serial_USB_ENABLE_STRINGS)
    volatile uint8 *pStr = 0u;
    #if defined(Serial_USB_ENABLE_DESCRIPTOR_STRINGS)
        uint8 nStr;
        uint8 descrLength;
    #endif /* (Serial_USB_ENABLE_DESCRIPTOR_STRINGS) */
#endif /* (Serial_USB_ENABLE_STRINGS) */
    
    static volatile uint8 Serial_USB_tBuffer[Serial_USB_STATUS_LENGTH_MAX];
    const T_Serial_USB_LUT CYCODE *pTmp;

    Serial_USB_currentTD.count = 0u;

    if (Serial_USB_RQST_DIR_D2H == (bmRequestType & Serial_USB_RQST_DIR_MASK))
    {
        /* Control Read */
        switch (Serial_USB_bRequestReg)
        {
            case Serial_USB_GET_DESCRIPTOR:
                if (Serial_USB_DESCR_DEVICE ==Serial_USB_wValueHiReg)
                {
                    pTmp = Serial_USB_GetDeviceTablePtr();
                    Serial_USB_currentTD.pData = (volatile uint8 *)pTmp->p_list;
                    Serial_USB_currentTD.count = Serial_USB_DEVICE_DESCR_LENGTH;
                    
                    requestHandled  = Serial_USB_InitControlRead();
                }
                else if (Serial_USB_DESCR_CONFIG == Serial_USB_wValueHiReg)
                {
                    pTmp = Serial_USB_GetConfigTablePtr((uint8) Serial_USB_wValueLoReg);
                    
                    /* Verify that requested descriptor exists */
                    if (pTmp != NULL)
                    {
                        Serial_USB_currentTD.pData = (volatile uint8 *)pTmp->p_list;
                        Serial_USB_currentTD.count = (uint16)((uint16)(Serial_USB_currentTD.pData)[Serial_USB_CONFIG_DESCR_TOTAL_LENGTH_HI] << 8u) | \
                                                                            (Serial_USB_currentTD.pData)[Serial_USB_CONFIG_DESCR_TOTAL_LENGTH_LOW];
                        requestHandled  = Serial_USB_InitControlRead();
                    }
                }
                
            #if(Serial_USB_BOS_ENABLE)
                else if (Serial_USB_DESCR_BOS == Serial_USB_wValueHiReg)
                {
                    pTmp = Serial_USB_GetBOSPtr();
                    
                    /* Verify that requested descriptor exists */
                    if (pTmp != NULL)
                    {
                        Serial_USB_currentTD.pData = (volatile uint8 *)pTmp;
                        Serial_USB_currentTD.count = ((uint16)((uint16)(Serial_USB_currentTD.pData)[Serial_USB_BOS_DESCR_TOTAL_LENGTH_HI] << 8u)) | \
                                                                             (Serial_USB_currentTD.pData)[Serial_USB_BOS_DESCR_TOTAL_LENGTH_LOW];
                        requestHandled  = Serial_USB_InitControlRead();
                    }
                }
            #endif /*(Serial_USB_BOS_ENABLE)*/
            
            #if defined(Serial_USB_ENABLE_STRINGS)
                else if (Serial_USB_DESCR_STRING == Serial_USB_wValueHiReg)
                {
                /* Descriptor Strings */
                #if defined(Serial_USB_ENABLE_DESCRIPTOR_STRINGS)
                    nStr = 0u;
                    pStr = (volatile uint8 *) &Serial_USB_STRING_DESCRIPTORS[0u];
                    
                    while ((Serial_USB_wValueLoReg > nStr) && (*pStr != 0u))
                    {
                        /* Read descriptor length from 1st byte */
                        descrLength = *pStr;
                        /* Move to next string descriptor */
                        pStr = &pStr[descrLength];
                        nStr++;
                    }
                #endif /* (Serial_USB_ENABLE_DESCRIPTOR_STRINGS) */
                
                /* Microsoft OS String */
                #if defined(Serial_USB_ENABLE_MSOS_STRING)
                    if (Serial_USB_STRING_MSOS == Serial_USB_wValueLoReg)
                    {
                        pStr = (volatile uint8 *)& Serial_USB_MSOS_DESCRIPTOR[0u];
                    }
                #endif /* (Serial_USB_ENABLE_MSOS_STRING) */
                
                /* SN string */
                #if defined(Serial_USB_ENABLE_SN_STRING)
                    if ((Serial_USB_wValueLoReg != 0u) && 
                        (Serial_USB_wValueLoReg == Serial_USB_DEVICE0_DESCR[Serial_USB_DEVICE_DESCR_SN_SHIFT]))
                    {
                    #if defined(Serial_USB_ENABLE_IDSN_STRING)
                        /* Read DIE ID and generate string descriptor in RAM */
                        Serial_USB_ReadDieID(Serial_USB_idSerialNumberStringDescriptor);
                        pStr = Serial_USB_idSerialNumberStringDescriptor;
                    #elif defined(Serial_USB_ENABLE_FWSN_STRING)
                        
                        if(Serial_USB_snStringConfirm != Serial_USB_FALSE)
                        {
                            pStr = Serial_USB_fwSerialNumberStringDescriptor;
                        }
                        else
                        {
                            pStr = (volatile uint8 *)&Serial_USB_SN_STRING_DESCRIPTOR[0u];
                        }
                    #else
                        pStr = (volatile uint8 *)&Serial_USB_SN_STRING_DESCRIPTOR[0u];
                    #endif  /* (Serial_USB_ENABLE_IDSN_STRING) */
                    }
                #endif /* (Serial_USB_ENABLE_SN_STRING) */
                
                    if (*pStr != 0u)
                    {
                        Serial_USB_currentTD.count = *pStr;
                        Serial_USB_currentTD.pData = pStr;
                        requestHandled  = Serial_USB_InitControlRead();
                    }
                }
            #endif /*  Serial_USB_ENABLE_STRINGS */
                else
                {
                    requestHandled = Serial_USB_DispatchClassRqst();
                }
                break;
                
            case Serial_USB_GET_STATUS:
                switch (bmRequestType & Serial_USB_RQST_RCPT_MASK)
                {
                    case Serial_USB_RQST_RCPT_EP:
                        Serial_USB_currentTD.count = Serial_USB_EP_STATUS_LENGTH;
                        Serial_USB_tBuffer[0u]     = Serial_USB_EP[Serial_USB_wIndexLoReg & Serial_USB_DIR_UNUSED].hwEpState;
                        Serial_USB_tBuffer[1u]     = 0u;
                        Serial_USB_currentTD.pData = &Serial_USB_tBuffer[0u];
                        
                        requestHandled  = Serial_USB_InitControlRead();
                        break;
                    case Serial_USB_RQST_RCPT_DEV:
                        Serial_USB_currentTD.count = Serial_USB_DEVICE_STATUS_LENGTH;
                        Serial_USB_tBuffer[0u]     = Serial_USB_deviceStatus;
                        Serial_USB_tBuffer[1u]     = 0u;
                        Serial_USB_currentTD.pData = &Serial_USB_tBuffer[0u];
                        
                        requestHandled  = Serial_USB_InitControlRead();
                        break;
                    default:    /* requestHandled is initialized as FALSE by default */
                        break;
                }
                break;
                
            case Serial_USB_GET_CONFIGURATION:
                Serial_USB_currentTD.count = 1u;
                Serial_USB_currentTD.pData = (volatile uint8 *) &Serial_USB_configuration;
                requestHandled  = Serial_USB_InitControlRead();
                break;
                
            case Serial_USB_GET_INTERFACE:
                Serial_USB_currentTD.count = 1u;
                Serial_USB_currentTD.pData = (volatile uint8 *) &Serial_USB_interfaceSetting[Serial_USB_wIndexLoReg];
                requestHandled  = Serial_USB_InitControlRead();
                break;
                
            default: /* requestHandled is initialized as FALSE by default */
                break;
        }
    }
    else
    {
        /* Control Write */
        switch (Serial_USB_bRequestReg)
        {
            case Serial_USB_SET_ADDRESS:
                /* Store address to be set in Serial_USB_NoDataControlStatusStage(). */
                Serial_USB_deviceAddress = (uint8) Serial_USB_wValueLoReg;
                requestHandled = Serial_USB_InitNoDataControlTransfer();
                break;
                
            case Serial_USB_SET_CONFIGURATION:
                configurationN = Serial_USB_wValueLoReg;
                
                /* Verify that configuration descriptor exists */
                if(configurationN > 0u)
                {
                    pTmp = Serial_USB_GetConfigTablePtr((uint8) configurationN - 1u);
                }
                
                /* Responds with a Request Error when configuration number is invalid */
                if (((configurationN > 0u) && (pTmp != NULL)) || (configurationN == 0u))
                {
                    /* Set new configuration if it has been changed */
                    if(configurationN != Serial_USB_configuration)
                    {
                        Serial_USB_configuration = (uint8) configurationN;
                        Serial_USB_configurationChanged = Serial_USB_TRUE;
                        Serial_USB_Config(Serial_USB_TRUE);
                    }
                    requestHandled = Serial_USB_InitNoDataControlTransfer();
                }
                break;
                
            case Serial_USB_SET_INTERFACE:
                if (0u != Serial_USB_ValidateAlternateSetting())
                {
                    /* Get interface number from the request. */
                    interfaceNumber = Serial_USB_wIndexLoReg;
                    Serial_USB_interfaceNumber = (uint8) Serial_USB_wIndexLoReg;
                     
                    /* Check if alternate settings is changed for interface. */
                    if (Serial_USB_interfaceSettingLast[interfaceNumber] != Serial_USB_interfaceSetting[interfaceNumber])
                    {
                        Serial_USB_configurationChanged = Serial_USB_TRUE;
                    
                        /* Change alternate setting for the endpoints. */
                    #if (Serial_USB_EP_MANAGEMENT_MANUAL && Serial_USB_EP_ALLOC_DYNAMIC)
                        Serial_USB_Config(Serial_USB_FALSE);
                    #else
                        Serial_USB_ConfigAltChanged();
                    #endif /* (Serial_USB_EP_MANAGEMENT_MANUAL && Serial_USB_EP_ALLOC_DYNAMIC) */
                    }
                    
                    requestHandled = Serial_USB_InitNoDataControlTransfer();
                }
                break;
                
            case Serial_USB_CLEAR_FEATURE:
                switch (bmRequestType & Serial_USB_RQST_RCPT_MASK)
                {
                    case Serial_USB_RQST_RCPT_EP:
                        if (Serial_USB_wValueLoReg == Serial_USB_ENDPOINT_HALT)
                        {
                            requestHandled = Serial_USB_ClearEndpointHalt();
                        }
                        break;
                    case Serial_USB_RQST_RCPT_DEV:
                        /* Clear device REMOTE_WAKEUP */
                        if (Serial_USB_wValueLoReg == Serial_USB_DEVICE_REMOTE_WAKEUP)
                        {
                            Serial_USB_deviceStatus &= (uint8)~Serial_USB_DEVICE_STATUS_REMOTE_WAKEUP;
                            requestHandled = Serial_USB_InitNoDataControlTransfer();
                        }
                        break;
                    case Serial_USB_RQST_RCPT_IFC:
                        /* Validate interfaceNumber */
                        if (Serial_USB_wIndexLoReg < Serial_USB_MAX_INTERFACES_NUMBER)
                        {
                            Serial_USB_interfaceStatus[Serial_USB_wIndexLoReg] &= (uint8) ~Serial_USB_wValueLoReg;
                            requestHandled = Serial_USB_InitNoDataControlTransfer();
                        }
                        break;
                    default:    /* requestHandled is initialized as FALSE by default */
                        break;
                }
                break;
                
            case Serial_USB_SET_FEATURE:
                switch (bmRequestType & Serial_USB_RQST_RCPT_MASK)
                {
                    case Serial_USB_RQST_RCPT_EP:
                        if (Serial_USB_wValueLoReg == Serial_USB_ENDPOINT_HALT)
                        {
                            requestHandled = Serial_USB_SetEndpointHalt();
                        }
                        break;
                        
                    case Serial_USB_RQST_RCPT_DEV:
                        /* Set device REMOTE_WAKEUP */
                        if (Serial_USB_wValueLoReg == Serial_USB_DEVICE_REMOTE_WAKEUP)
                        {
                            Serial_USB_deviceStatus |= Serial_USB_DEVICE_STATUS_REMOTE_WAKEUP;
                            requestHandled = Serial_USB_InitNoDataControlTransfer();
                        }
                        break;
                        
                    case Serial_USB_RQST_RCPT_IFC:
                        /* Validate interfaceNumber */
                        if (Serial_USB_wIndexLoReg < Serial_USB_MAX_INTERFACES_NUMBER)
                        {
                            Serial_USB_interfaceStatus[Serial_USB_wIndexLoReg] &= (uint8) ~Serial_USB_wValueLoReg;
                            requestHandled = Serial_USB_InitNoDataControlTransfer();
                        }
                        break;
                    
                    default:    /* requestHandled is initialized as FALSE by default */
                        break;
                }
                break;
                
            default:    /* requestHandled is initialized as FALSE by default */
                break;
        }
    }
    
    return (requestHandled);
}


#if defined(Serial_USB_ENABLE_IDSN_STRING)
    /***************************************************************************
    * Function Name: Serial_USB_ReadDieID
    ************************************************************************//**
    *
    *  This routine read Die ID and generate Serial Number string descriptor.
    *
    *  \param descr:  pointer on string descriptor. This string size has to be equal or
    *          greater than Serial_USB_IDSN_DESCR_LENGTH.
    *
    *
    * \reentrant
    *  No.
    *
    ***************************************************************************/
    void Serial_USB_ReadDieID(uint8 descr[]) 
    {
        const char8 CYCODE hex[] = "0123456789ABCDEF";
        uint8 i;
        uint8 j = 0u;
        uint8 uniqueId[8u];

        if (NULL != descr)
        {
            /* Initialize descriptor header. */
            descr[0u] = Serial_USB_IDSN_DESCR_LENGTH;
            descr[1u] = Serial_USB_DESCR_STRING;
            
            /* Unique ID size is 8 bytes. */
            CyGetUniqueId((uint32 *) uniqueId);

            /* Fill descriptor with unique device ID. */
            for (i = 2u; i < Serial_USB_IDSN_DESCR_LENGTH; i += 4u)
            {
                descr[i]      = (uint8) hex[(uniqueId[j] >> 4u)];
                descr[i + 1u] = 0u;
                descr[i + 2u] = (uint8) hex[(uniqueId[j] & 0x0Fu)];
                descr[i + 3u] = 0u;
                ++j;
            }
        }
    }
#endif /* (Serial_USB_ENABLE_IDSN_STRING) */


/*******************************************************************************
* Function Name: Serial_USB_ConfigReg
****************************************************************************//**
*
*  This routine configures hardware registers from the variables.
*  It is called from Serial_USB_Config() function and from RestoreConfig
*  after Wakeup.
*
*******************************************************************************/
void Serial_USB_ConfigReg(void) 
{
    uint8 ep;

#if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
    uint8 epType = 0u;
#endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */

    /* Go thought all endpoints and set hardware configuration */
    for (ep = Serial_USB_EP1; ep < Serial_USB_MAX_EP; ++ep)
    {
        Serial_USB_ARB_EP_BASE.arbEp[ep].epCfg = Serial_USB_ARB_EPX_CFG_DEFAULT;
        
    #if (Serial_USB_EP_MANAGEMENT_DMA)
        /* Enable arbiter endpoint interrupt sources */
        Serial_USB_ARB_EP_BASE.arbEp[ep].epIntEn = Serial_USB_ARB_EPX_INT_MASK;
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA) */
    
        if (Serial_USB_EP[ep].epMode != Serial_USB_MODE_DISABLE)
        {
            if (0u != (Serial_USB_EP[ep].addr & Serial_USB_DIR_IN))
            {
                Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_NAK_IN;
                
            #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO && CY_PSOC4)
                /* Clear DMA_TERMIN for IN endpoint. */
                Serial_USB_ARB_EP_BASE.arbEp[ep].epIntEn &= (uint32) ~Serial_USB_ARB_EPX_INT_DMA_TERMIN;
            #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO && CY_PSOC4) */
            }
            else
            {
                Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_NAK_OUT;

            #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
                /* (CY_PSOC4): DMA_TERMIN for OUT endpoint is set above. */
                
                /* Prepare endpoint type mask. */
                epType |= (uint8) (0x01u << (ep - Serial_USB_EP1));
            #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
            }
        }
        else
        {
            Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_STALL_DATA_EP;
        }
        
    #if (!Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        #if (CY_PSOC4)
            Serial_USB_ARB_EP16_BASE.arbEp[ep].rwRa16  = (uint32) Serial_USB_EP[ep].buffOffset;
            Serial_USB_ARB_EP16_BASE.arbEp[ep].rwWa16  = (uint32) Serial_USB_EP[ep].buffOffset;
        #else
            Serial_USB_ARB_EP_BASE.arbEp[ep].rwRa    = LO8(Serial_USB_EP[ep].buffOffset);
            Serial_USB_ARB_EP_BASE.arbEp[ep].rwRaMsb = HI8(Serial_USB_EP[ep].buffOffset);
            Serial_USB_ARB_EP_BASE.arbEp[ep].rwWa    = LO8(Serial_USB_EP[ep].buffOffset);
            Serial_USB_ARB_EP_BASE.arbEp[ep].rwWaMsb = HI8(Serial_USB_EP[ep].buffOffset);
        #endif /* (CY_PSOC4) */
    #endif /* (!Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
    }

#if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
     /* BUF_SIZE depend on DMA_THRESS value:0x55-32 bytes  0x44-16 bytes 0x33-8 bytes 0x22-4 bytes 0x11-2 bytes */
    Serial_USB_BUF_SIZE_REG = Serial_USB_DMA_BUF_SIZE;

    /* Configure DMA burst threshold */
#if (CY_PSOC4)
    Serial_USB_DMA_THRES16_REG   = Serial_USB_DMA_BYTES_PER_BURST;
#else
    Serial_USB_DMA_THRES_REG     = Serial_USB_DMA_BYTES_PER_BURST;
    Serial_USB_DMA_THRES_MSB_REG = 0u;
#endif /* (CY_PSOC4) */
    Serial_USB_EP_ACTIVE_REG = Serial_USB_DEFAULT_ARB_INT_EN;
    Serial_USB_EP_TYPE_REG   = epType;
    
    /* Cfg_cmp bit set to 1 once configuration is complete. */
    /* Lock arbiter configtuation */
    Serial_USB_ARB_CFG_REG |= (uint8)  Serial_USB_ARB_CFG_CFG_CMP;
    /* Cfg_cmp bit set to 0 during configuration of PFSUSB Registers. */
    Serial_USB_ARB_CFG_REG &= (uint8) ~Serial_USB_ARB_CFG_CFG_CMP;

#endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */

    /* Enable interrupt SIE interurpt source from EP0-EP1 */
    Serial_USB_SIE_EP_INT_EN_REG = (uint8) Serial_USB_DEFAULT_SIE_EP_INT_EN;
}


/*******************************************************************************
* Function Name: Serial_USB_EpStateInit
****************************************************************************//**
*
*  This routine initialize state of Data end points based of its type: 
*   IN  - Serial_USB_IN_BUFFER_EMPTY (Serial_USB_EVENT_PENDING)
*   OUT - Serial_USB_OUT_BUFFER_EMPTY (Serial_USB_NO_EVENT_PENDING)
*
*******************************************************************************/
void Serial_USB_EpStateInit(void) 
{
    uint8 i;

    for (i = Serial_USB_EP1; i < Serial_USB_MAX_EP; i++)
    { 
        if (0u != (Serial_USB_EP[i].addr & Serial_USB_DIR_IN))
        {
            /* IN Endpoint */
            Serial_USB_EP[i].apiEpState = Serial_USB_EVENT_PENDING;
        }
        else
        {
            /* OUT Endpoint */
            Serial_USB_EP[i].apiEpState = Serial_USB_NO_EVENT_PENDING;
        }
    }
                    
}


/*******************************************************************************
* Function Name: Serial_USB_Config
****************************************************************************//**
*
*  This routine configures endpoints for the entire configuration by scanning
*  the configuration descriptor.
*
*  \param clearAltSetting: It configures the bAlternateSetting 0 for each interface.
*
* Serial_USB_interfaceClass - Initialized class array for each interface.
*   It is used for handling Class specific requests depend on interface class.
*   Different classes in multiple Alternate settings does not supported.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_Config(uint8 clearAltSetting) 
{
    uint8 ep;
    uint8 curEp;
    uint8 i;
    uint8 epType;
    const uint8 *pDescr;
    
    #if (!Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        uint16 buffCount = 0u;
    #endif /* (!Serial_USB_EP_MANAGEMENT_DMA_AUTO) */

    const T_Serial_USB_LUT CYCODE *pTmp;
    const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE *pEP;

    /* Clear endpoints settings */
    for (ep = 0u; ep < Serial_USB_MAX_EP; ++ep)
    {
        Serial_USB_EP[ep].attrib     = 0u;
        Serial_USB_EP[ep].hwEpState  = 0u;
        Serial_USB_EP[ep].epToggle   = 0u;
        Serial_USB_EP[ep].bufferSize = 0u;
        Serial_USB_EP[ep].interface  = 0u;
        Serial_USB_EP[ep].apiEpState = Serial_USB_NO_EVENT_PENDING;
        Serial_USB_EP[ep].epMode     = Serial_USB_MODE_DISABLE;   
    }

    /* Clear Alternate settings for all interfaces. */
    if (0u != clearAltSetting)
    {
        for (i = 0u; i < Serial_USB_MAX_INTERFACES_NUMBER; ++i)
        {
            Serial_USB_interfaceSetting[i]     = 0u;
            Serial_USB_interfaceSettingLast[i] = 0u;
        }
    }

    /* Init Endpoints and Device Status if configured */
    if (Serial_USB_configuration > 0u)
    {
        #if defined(Serial_USB_ENABLE_CDC_CLASS)
            uint8 cdcComNums = 0u;
        #endif  /* (Serial_USB_ENABLE_CDC_CLASS) */  

        pTmp = Serial_USB_GetConfigTablePtr(Serial_USB_configuration - 1u);
        
        /* Set Power status for current configuration */
        pDescr = (const uint8 *)pTmp->p_list;
        if ((pDescr[Serial_USB_CONFIG_DESCR_ATTRIB] & Serial_USB_CONFIG_DESCR_ATTRIB_SELF_POWERED) != 0u)
        {
            Serial_USB_deviceStatus |= (uint8)  Serial_USB_DEVICE_STATUS_SELF_POWERED;
        }
        else
        {
            Serial_USB_deviceStatus &= (uint8) ~Serial_USB_DEVICE_STATUS_SELF_POWERED;
        }
        
        /* Move to next element */
        pTmp = &pTmp[1u];
        ep = pTmp->c;  /* For this table, c is the number of endpoints configurations  */

        #if (Serial_USB_EP_MANAGEMENT_MANUAL && Serial_USB_EP_ALLOC_DYNAMIC)
            /* Configure for dynamic EP memory allocation */
            /* p_list points the endpoint setting table. */
            pEP = (T_Serial_USB_EP_SETTINGS_BLOCK *) pTmp->p_list;
            
            for (i = 0u; i < ep; i++)
            {     
                /* Compare current Alternate setting with EP Alt */
                if (Serial_USB_interfaceSetting[pEP->interface] == pEP->altSetting)
                {                                                          
                    curEp  = pEP->addr & Serial_USB_DIR_UNUSED;
                    epType = pEP->attributes & Serial_USB_EP_TYPE_MASK;
                    
                    Serial_USB_EP[curEp].addr       = pEP->addr;
                    Serial_USB_EP[curEp].attrib     = pEP->attributes;
                    Serial_USB_EP[curEp].bufferSize = pEP->bufferSize;

                    if (0u != (pEP->addr & Serial_USB_DIR_IN))
                    {
                        /* IN Endpoint */
                        Serial_USB_EP[curEp].epMode     = Serial_USB_GET_ACTIVE_IN_EP_CR0_MODE(epType);
                        Serial_USB_EP[curEp].apiEpState = Serial_USB_EVENT_PENDING;
                    
                    #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && (Serial_USB_MIDI_IN_BUFF_SIZE > 0))
                        if ((pEP->bMisc == Serial_USB_CLASS_AUDIO) && (epType == Serial_USB_EP_TYPE_BULK))
                        {
                            Serial_USB_midi_in_ep = curEp;
                        }
                    #endif  /* (Serial_USB_ENABLE_MIDI_STREAMING) */
                    }
                    else
                    {
                        /* OUT Endpoint */
                        Serial_USB_EP[curEp].epMode     = Serial_USB_GET_ACTIVE_OUT_EP_CR0_MODE(epType);
                        Serial_USB_EP[curEp].apiEpState = Serial_USB_NO_EVENT_PENDING;
                        
                    #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && (Serial_USB_MIDI_OUT_BUFF_SIZE > 0))
                        if ((pEP->bMisc == Serial_USB_CLASS_AUDIO) && (epType == Serial_USB_EP_TYPE_BULK))
                        {
                            Serial_USB_midi_out_ep = curEp;
                        }
                    #endif  /* (Serial_USB_ENABLE_MIDI_STREAMING) */
                    }

                #if(defined (Serial_USB_ENABLE_CDC_CLASS))
                    if((pEP->bMisc == Serial_USB_CLASS_CDC_DATA) ||(pEP->bMisc == Serial_USB_CLASS_CDC))
                    {
                        cdcComNums = Serial_USB_Cdc_EpInit(pEP, curEp, cdcComNums);
                    }
                #endif  /* (Serial_USB_ENABLE_CDC_CLASS) */
                }
                
                pEP = &pEP[1u];
            }
            
        #else
            for (i = Serial_USB_EP1; i < Serial_USB_MAX_EP; ++i)
            {
                /* p_list points the endpoint setting table. */
                pEP = (const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE *) pTmp->p_list;
                /* Find max length for each EP and select it (length could be different in different Alt settings) */
                /* but other settings should be correct with regards to Interface alt Setting */
                
                for (curEp = 0u; curEp < ep; ++curEp)
                {
                    if (i == (pEP->addr & Serial_USB_DIR_UNUSED))
                    {
                        /* Compare endpoint buffers size with current size to find greater. */
                        if (Serial_USB_EP[i].bufferSize < pEP->bufferSize)
                        {
                            Serial_USB_EP[i].bufferSize = pEP->bufferSize;
                        }
                        
                        /* Compare current Alternate setting with EP Alt */
                        if (Serial_USB_interfaceSetting[pEP->interface] == pEP->altSetting)
                        {                            
                            Serial_USB_EP[i].addr = pEP->addr;
                            Serial_USB_EP[i].attrib = pEP->attributes;
                            
                            epType = pEP->attributes & Serial_USB_EP_TYPE_MASK;
                            
                            if (0u != (pEP->addr & Serial_USB_DIR_IN))
                            {
                                /* IN Endpoint */
                                Serial_USB_EP[i].epMode     = Serial_USB_GET_ACTIVE_IN_EP_CR0_MODE(epType);
                                Serial_USB_EP[i].apiEpState = Serial_USB_EVENT_PENDING;
                                
                            #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && (Serial_USB_MIDI_IN_BUFF_SIZE > 0))
                                if ((pEP->bMisc == Serial_USB_CLASS_AUDIO) && (epType == Serial_USB_EP_TYPE_BULK))
                                {
                                    Serial_USB_midi_in_ep = i;
                                }
                            #endif  /* (Serial_USB_ENABLE_MIDI_STREAMING) */
                            }
                            else
                            {
                                /* OUT Endpoint */
                                Serial_USB_EP[i].epMode     = Serial_USB_GET_ACTIVE_OUT_EP_CR0_MODE(epType);
                                Serial_USB_EP[i].apiEpState = Serial_USB_NO_EVENT_PENDING;
                                
                            #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && (Serial_USB_MIDI_OUT_BUFF_SIZE > 0))
                                if ((pEP->bMisc == Serial_USB_CLASS_AUDIO) && (epType == Serial_USB_EP_TYPE_BULK))
                                {
                                    Serial_USB_midi_out_ep = i;
                                }
                            #endif  /* (Serial_USB_ENABLE_MIDI_STREAMING) */
                            }

                        #if (defined(Serial_USB_ENABLE_CDC_CLASS))
                            if((pEP->bMisc == Serial_USB_CLASS_CDC_DATA) ||(pEP->bMisc == Serial_USB_CLASS_CDC))
                            {
                                cdcComNums = Serial_USB_Cdc_EpInit(pEP, i, cdcComNums);
                            }
                        #endif  /* (Serial_USB_ENABLE_CDC_CLASS) */

                            #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
                                break;  /* Use first EP setting in Auto memory management */
                            #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
                        }
                    }
                    
                    pEP = &pEP[1u];
                }
            }
        #endif /*  (Serial_USB_EP_MANAGEMENT_MANUAL && Serial_USB_EP_ALLOC_DYNAMIC) */

        /* Init class array for each interface and interface number for each EP.
        *  It is used for handling Class specific requests directed to either an
        *  interface or the endpoint.
        */
        /* p_list points the endpoint setting table. */
        pEP = (const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE *) pTmp->p_list;
        for (i = 0u; i < ep; i++)
        {
            /* Configure interface number for each EP */
            Serial_USB_EP[pEP->addr & Serial_USB_DIR_UNUSED].interface = pEP->interface;
            pEP = &pEP[1u];
        }
        
        /* Init pointer on interface class table */
        Serial_USB_interfaceClass = Serial_USB_GetInterfaceClassTablePtr();
        
    /* Set the endpoint buffer addresses */
    #if (!Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        buffCount = 0u;
        for (ep = Serial_USB_EP1; ep < Serial_USB_MAX_EP; ++ep)
        {
            Serial_USB_EP[ep].buffOffset = buffCount;        
            buffCount += Serial_USB_EP[ep].bufferSize;
            
        #if (Serial_USB_GEN_16BITS_EP_ACCESS)
            /* Align EP buffers to be event size to access 16-bits DR register. */
            buffCount += (0u != (buffCount & 0x01u)) ? 1u : 0u;
        #endif /* (Serial_USB_GEN_16BITS_EP_ACCESS) */            
        }
    #endif /* (!Serial_USB_EP_MANAGEMENT_DMA_AUTO) */

        /* Configure hardware registers */
        Serial_USB_ConfigReg();
    }
}


/*******************************************************************************
* Function Name: Serial_USB_ConfigAltChanged
****************************************************************************//**
*
*  This routine update configuration for the required endpoints only.
*  It is called after SET_INTERFACE request when Static memory allocation used.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_ConfigAltChanged(void) 
{
    uint8 ep;
    uint8 curEp;
    uint8 epType;
    uint8 i;
    uint8 interfaceNum;

    const T_Serial_USB_LUT CYCODE *pTmp;
    const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE *pEP;

    /* Init Endpoints and Device Status if configured */
    if (Serial_USB_configuration > 0u)
    {
        /* Get number of endpoints configurations (ep). */
        pTmp = Serial_USB_GetConfigTablePtr(Serial_USB_configuration - 1u);
        pTmp = &pTmp[1u];
        ep = pTmp->c;

        /* Get pointer to endpoints setting table (pEP). */
        pEP = (const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE *) pTmp->p_list;
        
        /* Look through all possible endpoint configurations. Find endpoints 
        * which belong to current interface and alternate settings for 
        * re-configuration.
        */
        interfaceNum = Serial_USB_interfaceNumber;
        for (i = 0u; i < ep; i++)
        {
            /* Find endpoints which belong to current interface and alternate settings. */
            if ((interfaceNum == pEP->interface) && 
                (Serial_USB_interfaceSetting[interfaceNum] == pEP->altSetting))
            {
                curEp  = ((uint8) pEP->addr & Serial_USB_DIR_UNUSED);
                epType = ((uint8) pEP->attributes & Serial_USB_EP_TYPE_MASK);
                
                /* Change the SIE mode for the selected EP to NAK ALL */
                Serial_USB_EP[curEp].epToggle   = 0u;
                Serial_USB_EP[curEp].addr       = pEP->addr;
                Serial_USB_EP[curEp].attrib     = pEP->attributes;
                Serial_USB_EP[curEp].bufferSize = pEP->bufferSize;

                if (0u != (pEP->addr & Serial_USB_DIR_IN))
                {
                    /* IN Endpoint */
                    Serial_USB_EP[curEp].epMode     = Serial_USB_GET_ACTIVE_IN_EP_CR0_MODE(epType);
                    Serial_USB_EP[curEp].apiEpState = Serial_USB_EVENT_PENDING;
                }
                else
                {
                    /* OUT Endpoint */
                    Serial_USB_EP[curEp].epMode     = Serial_USB_GET_ACTIVE_OUT_EP_CR0_MODE(epType);
                    Serial_USB_EP[curEp].apiEpState = Serial_USB_NO_EVENT_PENDING;
                }
                
                /* Make SIE to NAK any endpoint requests */
                Serial_USB_SIE_EP_BASE.sieEp[curEp].epCr0 = Serial_USB_MODE_NAK_IN_OUT;

            #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
                /* Clear IN data ready. */
                Serial_USB_ARB_EP_BASE.arbEp[curEp].epCfg &= (uint8) ~Serial_USB_ARB_EPX_CFG_IN_DATA_RDY;

                /* Select endpoint number of reconfiguration */
                Serial_USB_DYN_RECONFIG_REG = (uint8) ((curEp - 1u) << Serial_USB_DYN_RECONFIG_EP_SHIFT);
                
                /* Request for dynamic re-configuration of endpoint. */
                Serial_USB_DYN_RECONFIG_REG |= Serial_USB_DYN_RECONFIG_ENABLE;
                
                /* Wait until block is ready for re-configuration */
                while (0u == (Serial_USB_DYN_RECONFIG_REG & Serial_USB_DYN_RECONFIG_RDY_STS))
                {
                }
                
                /* Once DYN_RECONFIG_RDY_STS bit is set, FW can change the EP configuration. */
                /* Change EP Type with new direction */
                if (0u != (pEP->addr & Serial_USB_DIR_IN))
                {
                    /* Set endpoint type: 0 - IN and 1 - OUT. */
                    Serial_USB_EP_TYPE_REG &= (uint8) ~(uint8)((uint8) 0x01u << (curEp - 1u));
                    
                #if (CY_PSOC4)
                    /* Clear DMA_TERMIN for IN endpoint */
                    Serial_USB_ARB_EP_BASE.arbEp[curEp].epIntEn &= (uint32) ~Serial_USB_ARB_EPX_INT_DMA_TERMIN;
                #endif /* (CY_PSOC4) */
                }
                else
                {
                    /* Set endpoint type: 0 - IN and 1- OUT. */
                    Serial_USB_EP_TYPE_REG |= (uint8) ((uint8) 0x01u << (curEp - 1u));
                    
                #if (CY_PSOC4)
                    /* Set DMA_TERMIN for OUT endpoint */
                    Serial_USB_ARB_EP_BASE.arbEp[curEp].epIntEn |= (uint32) Serial_USB_ARB_EPX_INT_DMA_TERMIN;
                #endif /* (CY_PSOC4) */
                }
                
                /* Complete dynamic re-configuration: all endpoint related status and signals 
                * are set into the default state.
                */
                Serial_USB_DYN_RECONFIG_REG &= (uint8) ~Serial_USB_DYN_RECONFIG_ENABLE;

            #else
                Serial_USB_SIE_EP_BASE.sieEp[curEp].epCnt0 = HI8(Serial_USB_EP[curEp].bufferSize);
                Serial_USB_SIE_EP_BASE.sieEp[curEp].epCnt1 = LO8(Serial_USB_EP[curEp].bufferSize);
                
                #if (CY_PSOC4)
                    Serial_USB_ARB_EP16_BASE.arbEp[curEp].rwRa16  = (uint32) Serial_USB_EP[curEp].buffOffset;
                    Serial_USB_ARB_EP16_BASE.arbEp[curEp].rwWa16  = (uint32) Serial_USB_EP[curEp].buffOffset;
                #else
                    Serial_USB_ARB_EP_BASE.arbEp[curEp].rwRa    = LO8(Serial_USB_EP[curEp].buffOffset);
                    Serial_USB_ARB_EP_BASE.arbEp[curEp].rwRaMsb = HI8(Serial_USB_EP[curEp].buffOffset);
                    Serial_USB_ARB_EP_BASE.arbEp[curEp].rwWa    = LO8(Serial_USB_EP[curEp].buffOffset);
                    Serial_USB_ARB_EP_BASE.arbEp[curEp].rwWaMsb = HI8(Serial_USB_EP[curEp].buffOffset);
                #endif /* (CY_PSOC4) */                
            #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
            }
            
            pEP = &pEP[1u]; /* Get next EP element */
        }
        
        /* The main loop has to re-enable DMA and OUT endpoint */
    }
}


/*******************************************************************************
* Function Name: Serial_USB_GetConfigTablePtr
****************************************************************************//**
*
*  This routine returns a pointer a configuration table entry
*
*  \param confIndex:  Configuration Index
*
* \return
*  Device Descriptor pointer or NULL when descriptor does not exist.
*
*******************************************************************************/
const T_Serial_USB_LUT CYCODE *Serial_USB_GetConfigTablePtr(uint8 confIndex)
                                                        
{
    /* Device Table */
    const T_Serial_USB_LUT CYCODE *pTmp;

    pTmp = (const T_Serial_USB_LUT CYCODE *) Serial_USB_TABLE[Serial_USB_device].p_list;

    /* The first entry points to the Device Descriptor,
    *  the second entry point to the BOS Descriptor
    *  the rest configuration entries.
    *  Set pointer to the first Configuration Descriptor
    */
    pTmp = &pTmp[2u];
    /* For this table, c is the number of configuration descriptors  */
    if(confIndex >= pTmp->c)   /* Verify that required configuration descriptor exists */
    {
        pTmp = (const T_Serial_USB_LUT CYCODE *) NULL;
    }
    else
    {
        pTmp = (const T_Serial_USB_LUT CYCODE *) pTmp[confIndex].p_list;
    }

    return (pTmp);
}


#if (Serial_USB_BOS_ENABLE)
    /*******************************************************************************
    * Function Name: Serial_USB_GetBOSPtr
    ****************************************************************************//**
    *
    *  This routine returns a pointer a BOS table entry
    *
    *  
    *
    * \return
    *  BOS Descriptor pointer or NULL when descriptor does not exist.
    *
    *******************************************************************************/
    const T_Serial_USB_LUT CYCODE *Serial_USB_GetBOSPtr(void)
                                                            
    {
        /* Device Table */
        const T_Serial_USB_LUT CYCODE *pTmp;

        pTmp = (const T_Serial_USB_LUT CYCODE *) Serial_USB_TABLE[Serial_USB_device].p_list;

        /* The first entry points to the Device Descriptor,
        *  the second entry points to the BOS Descriptor
        */
        pTmp = &pTmp[1u];
        pTmp = (const T_Serial_USB_LUT CYCODE *) pTmp->p_list;
        return (pTmp);
    }
#endif /* (Serial_USB_BOS_ENABLE) */


/*******************************************************************************
* Function Name: Serial_USB_GetDeviceTablePtr
****************************************************************************//**
*
*  This routine returns a pointer to the Device table
*
* \return
*  Device Table pointer
*
*******************************************************************************/
const T_Serial_USB_LUT CYCODE *Serial_USB_GetDeviceTablePtr(void)
                                                            
{
    /* Device Table */
    return( (const T_Serial_USB_LUT CYCODE *) Serial_USB_TABLE[Serial_USB_device].p_list );
}


/*******************************************************************************
* Function Name: USB_GetInterfaceClassTablePtr
****************************************************************************//**
*
*  This routine returns Interface Class table pointer, which contains
*  the relation between interface number and interface class.
*
* \return
*  Interface Class table pointer.
*
*******************************************************************************/
const uint8 CYCODE *Serial_USB_GetInterfaceClassTablePtr(void)
                                                        
{
    const T_Serial_USB_LUT CYCODE *pTmp;
    const uint8 CYCODE *pInterfaceClass;
    uint8 currentInterfacesNum;

    pTmp = Serial_USB_GetConfigTablePtr(Serial_USB_configuration - 1u);
    if (pTmp != NULL)
    {
        currentInterfacesNum  = ((const uint8 *) pTmp->p_list)[Serial_USB_CONFIG_DESCR_NUM_INTERFACES];
        /* Third entry in the LUT starts the Interface Table pointers */
        /* The INTERFACE_CLASS table is located after all interfaces */
        pTmp = &pTmp[currentInterfacesNum + 2u];
        pInterfaceClass = (const uint8 CYCODE *) pTmp->p_list;
    }
    else
    {
        pInterfaceClass = (const uint8 CYCODE *) NULL;
    }

    return (pInterfaceClass);
}


/*******************************************************************************
* Function Name: Serial_USB_TerminateEP
****************************************************************************//**
*
*  This function terminates the specified USBFS endpoint.
*  This function should be used before endpoint reconfiguration.
*
*  \param ep Contains the data endpoint number.
*
*  \reentrant
*  No.
*
* \sideeffect
* 
* The device responds with a NAK for any transactions on the selected endpoint.
*   
*******************************************************************************/
void Serial_USB_TerminateEP(uint8 epNumber) 
{
    /* Get endpoint number */
    epNumber &= Serial_USB_DIR_UNUSED;

    if ((epNumber > Serial_USB_EP0) && (epNumber < Serial_USB_MAX_EP))
    {
        /* Set the endpoint Halt */
        Serial_USB_EP[epNumber].hwEpState |= Serial_USB_ENDPOINT_STATUS_HALT;

        /* Clear the data toggle */
        Serial_USB_EP[epNumber].epToggle = 0u;
        Serial_USB_EP[epNumber].apiEpState = Serial_USB_NO_EVENT_ALLOWED;

        if ((Serial_USB_EP[epNumber].addr & Serial_USB_DIR_IN) != 0u)
        {   
            /* IN Endpoint */
            Serial_USB_SIE_EP_BASE.sieEp[epNumber].epCr0 = Serial_USB_MODE_NAK_IN;
        }
        else
        {
            /* OUT Endpoint */
            Serial_USB_SIE_EP_BASE.sieEp[epNumber].epCr0 = Serial_USB_MODE_NAK_OUT;
        }
    }
}


/*******************************************************************************
* Function Name: Serial_USB_SetEndpointHalt
****************************************************************************//**
*
*  This routine handles set endpoint halt.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_SetEndpointHalt(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;
    uint8 ep;
    
    /* Set endpoint halt */
    ep = Serial_USB_wIndexLoReg & Serial_USB_DIR_UNUSED;

    if ((ep > Serial_USB_EP0) && (ep < Serial_USB_MAX_EP))
    {
        /* Set the endpoint Halt */
        Serial_USB_EP[ep].hwEpState |= (Serial_USB_ENDPOINT_STATUS_HALT);

        /* Clear the data toggle */
        Serial_USB_EP[ep].epToggle = 0u;
        Serial_USB_EP[ep].apiEpState |= Serial_USB_NO_EVENT_ALLOWED;

        if ((Serial_USB_EP[ep].addr & Serial_USB_DIR_IN) != 0u)
        {
            /* IN Endpoint */
            Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = (Serial_USB_MODE_STALL_DATA_EP | 
                                                            Serial_USB_MODE_ACK_IN);
        }
        else
        {
            /* OUT Endpoint */
            Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = (Serial_USB_MODE_STALL_DATA_EP | 
                                                            Serial_USB_MODE_ACK_OUT);
        }
        requestHandled = Serial_USB_InitNoDataControlTransfer();
    }

    return (requestHandled);
}


/*******************************************************************************
* Function Name: Serial_USB_ClearEndpointHalt
****************************************************************************//**
*
*  This routine handles clear endpoint halt.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_ClearEndpointHalt(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;
    uint8 ep;

    /* Clear endpoint halt */
    ep = Serial_USB_wIndexLoReg & Serial_USB_DIR_UNUSED;

    if ((ep > Serial_USB_EP0) && (ep < Serial_USB_MAX_EP))
    {
        /* Clear the endpoint Halt */
        Serial_USB_EP[ep].hwEpState &= (uint8) ~Serial_USB_ENDPOINT_STATUS_HALT;

        /* Clear the data toggle */
        Serial_USB_EP[ep].epToggle = 0u;
        
        /* Clear toggle bit for already armed packet */
        Serial_USB_SIE_EP_BASE.sieEp[ep].epCnt0 &= (uint8) ~(uint8)Serial_USB_EPX_CNT_DATA_TOGGLE;
        
        /* Return API State as it was defined before */
        Serial_USB_EP[ep].apiEpState &= (uint8) ~Serial_USB_NO_EVENT_ALLOWED;

        if ((Serial_USB_EP[ep].addr & Serial_USB_DIR_IN) != 0u)
        {
            /* IN Endpoint */
            if(Serial_USB_EP[ep].apiEpState == Serial_USB_IN_BUFFER_EMPTY)
            {       
                /* Wait for next packet from application */
                Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_NAK_IN;
            }
            else    /* Continue armed transfer */
            {
                Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_ACK_IN;
            }
        }
        else
        {
            /* OUT Endpoint */
            if (Serial_USB_EP[ep].apiEpState == Serial_USB_OUT_BUFFER_FULL)
            {       
                /* Allow application to read full buffer */
                Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_NAK_OUT;
            }
            else    /* Mark endpoint as empty, so it will be reloaded */
            {
                Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_MODE_ACK_OUT;
            }
        }
        
        requestHandled = Serial_USB_InitNoDataControlTransfer();
    }

    return(requestHandled);
}


/*******************************************************************************
* Function Name: Serial_USB_ValidateAlternateSetting
****************************************************************************//**
*
*  Validates (and records) a SET INTERFACE request.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_ValidateAlternateSetting(void) 
{
    uint8 requestHandled = Serial_USB_FALSE;
    
    uint8 interfaceNum;
    uint8 curInterfacesNum;
    const T_Serial_USB_LUT CYCODE *pTmp;
    
    /* Get interface number from the request. */
    interfaceNum = (uint8) Serial_USB_wIndexLoReg;
    
    /* Get number of interfaces for current configuration. */
    pTmp = Serial_USB_GetConfigTablePtr(Serial_USB_configuration - 1u);
    curInterfacesNum  = ((const uint8 *) pTmp->p_list)[Serial_USB_CONFIG_DESCR_NUM_INTERFACES];

    /* Validate that interface number is within range. */
    if ((interfaceNum <= curInterfacesNum) || (interfaceNum <= Serial_USB_MAX_INTERFACES_NUMBER))
    {
        /* Save current and new alternate settings (come with request) to make 
        * desicion about following endpoint re-configuration.
        */
        Serial_USB_interfaceSettingLast[interfaceNum] = Serial_USB_interfaceSetting[interfaceNum];
        Serial_USB_interfaceSetting[interfaceNum]     = (uint8) Serial_USB_wValueLoReg;
        
        requestHandled = Serial_USB_TRUE;
    }

    return (requestHandled);
}


/* [] END OF FILE */
