/***************************************************************************//**
* \file Serial_USB_drv.c
* \version 3.20
*
* \brief
*  This file contains the Endpoint 0 Driver for the USBFS Component.  
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


/***************************************
* Global data allocation
***************************************/

volatile T_Serial_USB_EP_CTL_BLOCK Serial_USB_EP[Serial_USB_MAX_EP];

/** Contains the current configuration number, which is set by the host using a 
 * SET_CONFIGURATION request. This variable is initialized to zero in 
 * USBFS_InitComponent() API and can be read by the USBFS_GetConfiguration() 
 * API.*/
volatile uint8 Serial_USB_configuration;

/** Contains the current interface number.*/
volatile uint8 Serial_USB_interfaceNumber;

/** This variable is set to one after SET_CONFIGURATION and SET_INTERFACE 
 *requests. It can be read by the USBFS_IsConfigurationChanged() API */
volatile uint8 Serial_USB_configurationChanged;

/** Contains the current device address.*/
volatile uint8 Serial_USB_deviceAddress;

/** This is a two-bit variable that contains power status in the bit 0 
 * (DEVICE_STATUS_BUS_POWERED or DEVICE_STATUS_SELF_POWERED) and remote wakeup 
 * status (DEVICE_STATUS_REMOTE_WAKEUP) in the bit 1. This variable is 
 * initialized to zero in USBFS_InitComponent() API, configured by the 
 * USBFS_SetPowerStatus() API. The remote wakeup status cannot be set using the 
 * API SetPowerStatus(). */
volatile uint8 Serial_USB_deviceStatus;

volatile uint8 Serial_USB_interfaceSetting[Serial_USB_MAX_INTERFACES_NUMBER];
volatile uint8 Serial_USB_interfaceSetting_last[Serial_USB_MAX_INTERFACES_NUMBER];
volatile uint8 Serial_USB_interfaceStatus[Serial_USB_MAX_INTERFACES_NUMBER];

/** Contains the started device number. This variable is set by the 
 * USBFS_Start() or USBFS_InitComponent() APIs.*/
volatile uint8 Serial_USB_device;

/** Initialized class array for each interface. It is used for handling Class 
 * specific requests depend on interface class. Different classes in multiple 
 * alternate settings are not supported.*/
const uint8 CYCODE *Serial_USB_interfaceClass;


/***************************************
* Local data allocation
***************************************/

volatile uint8  Serial_USB_ep0Toggle;
volatile uint8  Serial_USB_lastPacketSize;

/** This variable is used by the communication functions to handle the current 
* transfer state.
* Initialized to TRANS_STATE_IDLE in the USBFS_InitComponent() API and after a 
* complete transfer in the status stage.
* Changed to the TRANS_STATE_CONTROL_READ or TRANS_STATE_CONTROL_WRITE in setup 
* transaction depending on the request type.
*/
volatile uint8  Serial_USB_transferState;
volatile T_Serial_USB_TD Serial_USB_currentTD;
volatile uint8  Serial_USB_ep0Mode;
volatile uint8  Serial_USB_ep0Count;
volatile uint16 Serial_USB_transferByteCount;


/*******************************************************************************
* Function Name: Serial_USB_ep_0_Interrupt
****************************************************************************//**
*
*  This Interrupt Service Routine handles Endpoint 0 (Control Pipe) traffic.
*  It dispatches setup requests and handles the data and status stages.
*
*
*******************************************************************************/
CY_ISR(Serial_USB_EP_0_ISR)
{
    uint8 tempReg;
    uint8 modifyReg;

#ifdef Serial_USB_EP_0_ISR_ENTRY_CALLBACK
    Serial_USB_EP_0_ISR_EntryCallback();
#endif /* (Serial_USB_EP_0_ISR_ENTRY_CALLBACK) */
    
    tempReg = Serial_USB_EP0_CR_REG;
    if ((tempReg & Serial_USB_MODE_ACKD) != 0u)
    {
        modifyReg = 1u;
        if ((tempReg & Serial_USB_MODE_SETUP_RCVD) != 0u)
        {
            if ((tempReg & Serial_USB_MODE_MASK) != Serial_USB_MODE_NAK_IN_OUT)
            {
                /* Mode not equal to NAK_IN_OUT: invalid setup */
                modifyReg = 0u;
            }
            else
            {
                Serial_USB_HandleSetup();
                
                if ((Serial_USB_ep0Mode & Serial_USB_MODE_SETUP_RCVD) != 0u)
                {
                    /* SETUP bit set: exit without mode modificaiton */
                    modifyReg = 0u;
                }
            }
        }
        else if ((tempReg & Serial_USB_MODE_IN_RCVD) != 0u)
        {
            Serial_USB_HandleIN();
        }
        else if ((tempReg & Serial_USB_MODE_OUT_RCVD) != 0u)
        {
            Serial_USB_HandleOUT();
        }
        else
        {
            modifyReg = 0u;
        }
        
        /* Modify the EP0_CR register */
        if (modifyReg != 0u)
        {
            
            tempReg = Serial_USB_EP0_CR_REG;
            
            /* Make sure that SETUP bit is cleared before modification */
            if ((tempReg & Serial_USB_MODE_SETUP_RCVD) == 0u)
            {
                /* Update count register */
                tempReg = (uint8) Serial_USB_ep0Toggle | Serial_USB_ep0Count;
                Serial_USB_EP0_CNT_REG = tempReg;
               
                /* Make sure that previous write operaiton was successful */
                if (tempReg == Serial_USB_EP0_CNT_REG)
                {
                    /* Repeat until next successful write operation */
                    do
                    {
                        /* Init temporary variable */
                        modifyReg = Serial_USB_ep0Mode;
                        
                        /* Unlock register */
                        tempReg = (uint8) (Serial_USB_EP0_CR_REG & Serial_USB_MODE_SETUP_RCVD);
                        
                        /* Check if SETUP bit is not set */
                        if (0u == tempReg)
                        {
                            /* Set the Mode Register  */
                            Serial_USB_EP0_CR_REG = Serial_USB_ep0Mode;
                            
                            /* Writing check */
                            modifyReg = Serial_USB_EP0_CR_REG & Serial_USB_MODE_MASK;
                        }
                    }
                    while (modifyReg != Serial_USB_ep0Mode);
                }
            }
        }
    }

    Serial_USB_ClearSieInterruptSource(Serial_USB_INTR_SIE_EP0_INTR);
	
#ifdef Serial_USB_EP_0_ISR_EXIT_CALLBACK
    Serial_USB_EP_0_ISR_ExitCallback();
#endif /* (Serial_USB_EP_0_ISR_EXIT_CALLBACK) */
}


/*******************************************************************************
* Function Name: Serial_USB_HandleSetup
****************************************************************************//**
*
*  This Routine dispatches requests for the four USB request types
*
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_HandleSetup(void) 
{
    uint8 requestHandled;
    
    /* Clear register lock by SIE (read register) and clear setup bit 
    * (write any value in register).
    */
    requestHandled = (uint8) Serial_USB_EP0_CR_REG;
    Serial_USB_EP0_CR_REG = (uint8) requestHandled;
    requestHandled = (uint8) Serial_USB_EP0_CR_REG;

    if ((requestHandled & Serial_USB_MODE_SETUP_RCVD) != 0u)
    {
        /* SETUP bit is set: exit without mode modification. */
        Serial_USB_ep0Mode = requestHandled;
    }
    else
    {
        /* In case the previous transfer did not complete, close it out */
        Serial_USB_UpdateStatusBlock(Serial_USB_XFER_PREMATURE);

        /* Check request type. */
        switch (Serial_USB_bmRequestTypeReg & Serial_USB_RQST_TYPE_MASK)
        {
            case Serial_USB_RQST_TYPE_STD:
                requestHandled = Serial_USB_HandleStandardRqst();
                break;
                
            case Serial_USB_RQST_TYPE_CLS:
                requestHandled = Serial_USB_DispatchClassRqst();
                break;
                
            case Serial_USB_RQST_TYPE_VND:
                requestHandled = Serial_USB_HandleVendorRqst();
                break;
                
            default:
                requestHandled = Serial_USB_FALSE;
                break;
        }
        
        /* If request is not recognized. Stall endpoint 0 IN and OUT. */
        if (requestHandled == Serial_USB_FALSE)
        {
            Serial_USB_ep0Mode = Serial_USB_MODE_STALL_IN_OUT;
        }
    }
}


/*******************************************************************************
* Function Name: Serial_USB_HandleIN
****************************************************************************//**
*
*  This routine handles EP0 IN transfers.
*
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_HandleIN(void) 
{
    switch (Serial_USB_transferState)
    {
        case Serial_USB_TRANS_STATE_IDLE:
            break;
        
        case Serial_USB_TRANS_STATE_CONTROL_READ:
            Serial_USB_ControlReadDataStage();
            break;
            
        case Serial_USB_TRANS_STATE_CONTROL_WRITE:
            Serial_USB_ControlWriteStatusStage();
            break;
            
        case Serial_USB_TRANS_STATE_NO_DATA_CONTROL:
            Serial_USB_NoDataControlStatusStage();
            break;
            
        default:    /* there are no more states */
            break;
    }
}


/*******************************************************************************
* Function Name: Serial_USB_HandleOUT
****************************************************************************//**
*
*  This routine handles EP0 OUT transfers.
*
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_HandleOUT(void) 
{
    switch (Serial_USB_transferState)
    {
        case Serial_USB_TRANS_STATE_IDLE:
            break;
        
        case Serial_USB_TRANS_STATE_CONTROL_READ:
            Serial_USB_ControlReadStatusStage();
            break;
            
        case Serial_USB_TRANS_STATE_CONTROL_WRITE:
            Serial_USB_ControlWriteDataStage();
            break;
            
        case Serial_USB_TRANS_STATE_NO_DATA_CONTROL:
            /* Update the completion block */
            Serial_USB_UpdateStatusBlock(Serial_USB_XFER_ERROR);
            
            /* We expect no more data, so stall INs and OUTs */
            Serial_USB_ep0Mode = Serial_USB_MODE_STALL_IN_OUT;
            break;
            
        default:    
            /* There are no more states */
            break;
    }
}


/*******************************************************************************
* Function Name: Serial_USB_LoadEP0
****************************************************************************//**
*
*  This routine loads the EP0 data registers for OUT transfers. It uses the
*  currentTD (previously initialized by the _InitControlWrite function and
*  updated for each OUT transfer, and the bLastPacketSize) to determine how
*  many uint8s to transfer on the current OUT.
*
*  If the number of uint8s remaining is zero and the last transfer was full,
*  we need to send a zero length packet.  Otherwise we send the minimum
*  of the control endpoint size (8) or remaining number of uint8s for the
*  transaction.
*
*
* \globalvars
*  Serial_USB_transferByteCount - Update the transfer byte count from the
*     last transaction.
*  Serial_USB_ep0Count - counts the data loaded to the SIE memory in
*     current packet.
*  Serial_USB_lastPacketSize - remembers the USBFS_ep0Count value for the
*     next packet.
*  Serial_USB_transferByteCount - sum of the previous bytes transferred
*     on previous packets(sum of USBFS_lastPacketSize)
*  Serial_USB_ep0Toggle - inverted
*  Serial_USB_ep0Mode  - prepare for mode register content.
*  Serial_USB_transferState - set to TRANS_STATE_CONTROL_READ
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_LoadEP0(void) 
{
    uint8 ep0Count = 0u;

    /* Update the transfer byte count from the last transaction */
    Serial_USB_transferByteCount += Serial_USB_lastPacketSize;

    /* Now load the next transaction */
    while ((Serial_USB_currentTD.count > 0u) && (ep0Count < 8u))
    {
        Serial_USB_EP0_DR_BASE.epData[ep0Count] = (uint8) *Serial_USB_currentTD.pData;
        Serial_USB_currentTD.pData = &Serial_USB_currentTD.pData[1u];
        ep0Count++;
        Serial_USB_currentTD.count--;
    }

    /* Support zero-length packet */
    if ((Serial_USB_lastPacketSize == 8u) || (ep0Count > 0u))
    {
        /* Update the data toggle */
        Serial_USB_ep0Toggle ^= Serial_USB_EP0_CNT_DATA_TOGGLE;
        /* Set the Mode Register  */
        Serial_USB_ep0Mode = Serial_USB_MODE_ACK_IN_STATUS_OUT;
        /* Update the state (or stay the same) */
        Serial_USB_transferState = Serial_USB_TRANS_STATE_CONTROL_READ;
    }
    else
    {
        /* Expect Status Stage Out */
        Serial_USB_ep0Mode = Serial_USB_MODE_STATUS_OUT_ONLY;
        /* Update the state (or stay the same) */
        Serial_USB_transferState = Serial_USB_TRANS_STATE_CONTROL_READ;
    }

    /* Save the packet size for next time */
    Serial_USB_ep0Count =       (uint8) ep0Count;
    Serial_USB_lastPacketSize = (uint8) ep0Count;
}


/*******************************************************************************
* Function Name: Serial_USB_InitControlRead
****************************************************************************//**
*
*  Initialize a control read transaction. It is used to send data to the host.
*  The following global variables should be initialized before this function
*  called. To send zero length packet use InitZeroLengthControlTransfer
*  function.
*
*
* \return
*  requestHandled state.
*
* \globalvars
*  Serial_USB_currentTD.count - counts of data to be sent.
*  Serial_USB_currentTD.pData - data pointer.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_InitControlRead(void) 
{
    uint16 xferCount;

    if (Serial_USB_currentTD.count == 0u)
    {
        (void) Serial_USB_InitZeroLengthControlTransfer();
    }
    else
    {
        /* Set up the state machine */
        Serial_USB_transferState = Serial_USB_TRANS_STATE_CONTROL_READ;
        
        /* Set the toggle, it gets updated in LoadEP */
        Serial_USB_ep0Toggle = 0u;
        
        /* Initialize the Status Block */
        Serial_USB_InitializeStatusBlock();
        
        xferCount = ((uint16)((uint16) Serial_USB_lengthHiReg << 8u) | ((uint16) Serial_USB_lengthLoReg));

        if (Serial_USB_currentTD.count > xferCount)
        {
            Serial_USB_currentTD.count = xferCount;
        }
        
        Serial_USB_LoadEP0();
    }

    return (Serial_USB_TRUE);
}


/*******************************************************************************
* Function Name: Serial_USB_InitZeroLengthControlTransfer
****************************************************************************//**
*
*  Initialize a zero length data IN transfer.
*
* \return
*  requestHandled state.
*
* \globalvars
*  Serial_USB_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*  Serial_USB_ep0Mode  - prepare for mode register content.
*  Serial_USB_transferState - set to TRANS_STATE_CONTROL_READ
*  Serial_USB_ep0Count - cleared, means the zero-length packet.
*  Serial_USB_lastPacketSize - cleared.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_InitZeroLengthControlTransfer(void)
                                                
{
    /* Update the state */
    Serial_USB_transferState = Serial_USB_TRANS_STATE_CONTROL_READ;
    
    /* Set the data toggle */
    Serial_USB_ep0Toggle = Serial_USB_EP0_CNT_DATA_TOGGLE;
    
    /* Set the Mode Register  */
    Serial_USB_ep0Mode = Serial_USB_MODE_ACK_IN_STATUS_OUT;
    
    /* Save the packet size for next time */
    Serial_USB_lastPacketSize = 0u;
    
    Serial_USB_ep0Count = 0u;

    return (Serial_USB_TRUE);
}


/*******************************************************************************
* Function Name: Serial_USB_ControlReadDataStage
****************************************************************************//**
*
*  Handle the Data Stage of a control read transfer.
*
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_ControlReadDataStage(void) 

{
    Serial_USB_LoadEP0();
}


/*******************************************************************************
* Function Name: Serial_USB_ControlReadStatusStage
****************************************************************************//**
*
*  Handle the Status Stage of a control read transfer.
*
*
* \globalvars
*  Serial_USB_USBFS_transferByteCount - updated with last packet size.
*  Serial_USB_transferState - set to TRANS_STATE_IDLE.
*  Serial_USB_ep0Mode  - set to MODE_STALL_IN_OUT.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_ControlReadStatusStage(void) 
{
    /* Update the transfer byte count */
    Serial_USB_transferByteCount += Serial_USB_lastPacketSize;
    
    /* Go Idle */
    Serial_USB_transferState = Serial_USB_TRANS_STATE_IDLE;
    
    /* Update the completion block */
    Serial_USB_UpdateStatusBlock(Serial_USB_XFER_STATUS_ACK);
    
    /* We expect no more data, so stall INs and OUTs */
    Serial_USB_ep0Mode = Serial_USB_MODE_STALL_IN_OUT;
}


/*******************************************************************************
* Function Name: Serial_USB_InitControlWrite
****************************************************************************//**
*
*  Initialize a control write transaction
*
* \return
*  requestHandled state.
*
* \globalvars
*  Serial_USB_USBFS_transferState - set to TRANS_STATE_CONTROL_WRITE
*  Serial_USB_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*  Serial_USB_ep0Mode  - set to MODE_ACK_OUT_STATUS_IN
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_InitControlWrite(void) 
{
    uint16 xferCount;

    /* Set up the state machine */
    Serial_USB_transferState = Serial_USB_TRANS_STATE_CONTROL_WRITE;
    
    /* This might not be necessary */
    Serial_USB_ep0Toggle = Serial_USB_EP0_CNT_DATA_TOGGLE;
    
    /* Initialize the Status Block */
    Serial_USB_InitializeStatusBlock();

    xferCount = ((uint16)((uint16) Serial_USB_lengthHiReg << 8u) | ((uint16) Serial_USB_lengthLoReg));

    if (Serial_USB_currentTD.count > xferCount)
    {
        Serial_USB_currentTD.count = xferCount;
    }

    /* Expect Data or Status Stage */
    Serial_USB_ep0Mode = Serial_USB_MODE_ACK_OUT_STATUS_IN;

    return(Serial_USB_TRUE);
}


/*******************************************************************************
* Function Name: Serial_USB_ControlWriteDataStage
****************************************************************************//**
*
*  Handle the Data Stage of a control write transfer
*       1. Get the data (We assume the destination was validated previously)
*       2. Update the count and data toggle
*       3. Update the mode register for the next transaction
*
*
* \globalvars
*  Serial_USB_transferByteCount - Update the transfer byte count from the
*    last transaction.
*  Serial_USB_ep0Count - counts the data loaded from the SIE memory
*    in current packet.
*  Serial_USB_transferByteCount - sum of the previous bytes transferred
*    on previous packets(sum of USBFS_lastPacketSize)
*  Serial_USB_ep0Toggle - inverted
*  Serial_USB_ep0Mode  - set to MODE_ACK_OUT_STATUS_IN.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_ControlWriteDataStage(void) 
{
    uint8 ep0Count;
    uint8 regIndex = 0u;

    ep0Count = (Serial_USB_EP0_CNT_REG & Serial_USB_EPX_CNT0_MASK) - Serial_USB_EPX_CNTX_CRC_COUNT;

    Serial_USB_transferByteCount += (uint8)ep0Count;

    while ((Serial_USB_currentTD.count > 0u) && (ep0Count > 0u))
    {
        *Serial_USB_currentTD.pData = (uint8) Serial_USB_EP0_DR_BASE.epData[regIndex];
        Serial_USB_currentTD.pData = &Serial_USB_currentTD.pData[1u];
        regIndex++;
        ep0Count--;
        Serial_USB_currentTD.count--;
    }
    
    Serial_USB_ep0Count = (uint8)ep0Count;
    
    /* Update the data toggle */
    Serial_USB_ep0Toggle ^= Serial_USB_EP0_CNT_DATA_TOGGLE;
    
    /* Expect Data or Status Stage */
    Serial_USB_ep0Mode = Serial_USB_MODE_ACK_OUT_STATUS_IN;
}


/*******************************************************************************
* Function Name: Serial_USB_ControlWriteStatusStage
****************************************************************************//**
*
*  Handle the Status Stage of a control write transfer
*
* \globalvars
*  Serial_USB_transferState - set to TRANS_STATE_IDLE.
*  Serial_USB_USBFS_ep0Mode  - set to MODE_STALL_IN_OUT.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_ControlWriteStatusStage(void) 
{
    /* Go Idle */
    Serial_USB_transferState = Serial_USB_TRANS_STATE_IDLE;
    
    /* Update the completion block */    
    Serial_USB_UpdateStatusBlock(Serial_USB_XFER_STATUS_ACK);
    
    /* We expect no more data, so stall INs and OUTs */
    Serial_USB_ep0Mode = Serial_USB_MODE_STALL_IN_OUT;
}


/*******************************************************************************
* Function Name: Serial_USB_InitNoDataControlTransfer
****************************************************************************//**
*
*  Initialize a no data control transfer
*
* \return
*  requestHandled state.
*
* \globalvars
*  Serial_USB_transferState - set to TRANS_STATE_NO_DATA_CONTROL.
*  Serial_USB_ep0Mode  - set to MODE_STATUS_IN_ONLY.
*  Serial_USB_ep0Count - cleared.
*  Serial_USB_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 Serial_USB_InitNoDataControlTransfer(void) 
{
    Serial_USB_transferState = Serial_USB_TRANS_STATE_NO_DATA_CONTROL;
    Serial_USB_ep0Mode       = Serial_USB_MODE_STATUS_IN_ONLY;
    Serial_USB_ep0Toggle     = Serial_USB_EP0_CNT_DATA_TOGGLE;
    Serial_USB_ep0Count      = 0u;

    return (Serial_USB_TRUE);
}


/*******************************************************************************
* Function Name: Serial_USB_NoDataControlStatusStage
****************************************************************************//**
*  Handle the Status Stage of a no data control transfer.
*
*  SET_ADDRESS is special, since we need to receive the status stage with
*  the old address.
*
* \globalvars
*  Serial_USB_transferState - set to TRANS_STATE_IDLE.
*  Serial_USB_ep0Mode  - set to MODE_STALL_IN_OUT.
*  Serial_USB_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*  Serial_USB_deviceAddress - used to set new address and cleared
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_NoDataControlStatusStage(void) 
{
    if (0u != Serial_USB_deviceAddress)
    {
        /* Update device address if we got new address. */
        Serial_USB_CR0_REG = (uint8) Serial_USB_deviceAddress | Serial_USB_CR0_ENABLE;
        Serial_USB_deviceAddress = 0u;
    }

    Serial_USB_transferState = Serial_USB_TRANS_STATE_IDLE;
    
    /* Update the completion block. */
    Serial_USB_UpdateStatusBlock(Serial_USB_XFER_STATUS_ACK);
    
    /* Stall IN and OUT, no more data is expected. */
    Serial_USB_ep0Mode = Serial_USB_MODE_STALL_IN_OUT;
}


/*******************************************************************************
* Function Name: Serial_USB_UpdateStatusBlock
****************************************************************************//**
*
*  Update the Completion Status Block for a Request.  The block is updated
*  with the completion code the Serial_USB_transferByteCount.  The
*  StatusBlock Pointer is set to NULL.
*
*  completionCode - status.
*
*
* \globalvars
*  Serial_USB_currentTD.pStatusBlock->status - updated by the
*    completionCode parameter.
*  Serial_USB_currentTD.pStatusBlock->length - updated.
*  Serial_USB_currentTD.pStatusBlock - cleared.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_UpdateStatusBlock(uint8 completionCode) 
{
    if (Serial_USB_currentTD.pStatusBlock != NULL)
    {
        Serial_USB_currentTD.pStatusBlock->status = completionCode;
        Serial_USB_currentTD.pStatusBlock->length = Serial_USB_transferByteCount;
        Serial_USB_currentTD.pStatusBlock = NULL;
    }
}


/*******************************************************************************
* Function Name: Serial_USB_InitializeStatusBlock
****************************************************************************//**
*
*  Initialize the Completion Status Block for a Request.  The completion
*  code is set to USB_XFER_IDLE.
*
*  Also, initializes Serial_USB_transferByteCount.  Save some space,
*  this is the only consumer.
*
* \globalvars
*  Serial_USB_currentTD.pStatusBlock->status - set to XFER_IDLE.
*  Serial_USB_currentTD.pStatusBlock->length - cleared.
*  Serial_USB_transferByteCount - cleared.
*
* \reentrant
*  No.
*
*******************************************************************************/
void Serial_USB_InitializeStatusBlock(void) 
{
    Serial_USB_transferByteCount = 0u;
    
    if (Serial_USB_currentTD.pStatusBlock != NULL)
    {
        Serial_USB_currentTD.pStatusBlock->status = Serial_USB_XFER_IDLE;
        Serial_USB_currentTD.pStatusBlock->length = 0u;
    }
}


/* [] END OF FILE */
