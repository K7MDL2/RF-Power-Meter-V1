/***************************************************************************//**
* \file Serial_USB_cydmac.h
* \version 3.20
*
* \brief
*  This file provides macros implemenation of DMA_P4 functions.
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_USBFS_Serial_USB_cydmac_H)
#define CY_USBFS_Serial_USB_cydmac_H

#include "Serial_USB_pvt.h"

/*******************************************************************************
* Function Name: Serial_USB_CyDmaSetConfiguration
****************************************************************************//**
*
*  Sets configuration information for the specified descriptor.
*
*  \param ch:    DMA ch modified by this function.
*  \param descr: Descriptor (0 or 1) modified by this function.
*  \param cfg:   Descriptor control register.
*
* \sideeffect
*   The status register associated with the specified descriptor is reset to 
*   zero after this function call. This function should not be called while 
*   the descriptor is active. This can be checked by calling CyDmaGetStatus().
*
*******************************************************************************/
#define Serial_USB_CyDmaSetConfiguration(ch, descr, cfg) \
    do{ \
        CYDMA_DESCR_BASE.descriptor[ch][descr].ctl = (cfg); \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaSetInterruptMask
****************************************************************************//**
*
*  Enables the DMA channel interrupt.
*
*  \param ch: Channel used by this function.
*
*
*******************************************************************************/
#define Serial_USB_CyDmaSetInterruptMask(ch) \
    do{ \
        CYDMA_INTR_MASK_REG |= ((uint32)(1UL << (ch))); \
    }while(0)


/*******************************************************************************
* Function Name:Serial_USB_CyDmaSetDescriptor0Next
****************************************************************************//**
*
*  Sets the descriptor 0 that should be run the next time the channel is
*  triggered.
*
*  \param channel:    Channel used by this function.
*
*
*******************************************************************************/
#define Serial_USB_CyDmaSetDescriptor0Next(ch) \
    do{ \
        CYDMA_CH_CTL_BASE.ctl[(ch)] &= (uint32) ~CYDMA_DESCRIPTOR; \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaSetNumDataElements
****************************************************************************//**
*
*  Sets the number of data elements to transfer for specified descriptor.
*
*  \param ch:    Channel used by this function.
*  \param descr: Descriptor (0 or 1) modified by this function.
*  \param numEl: Total number of data elements this descriptor transfers - 1u.
*         Valid ranges are 0 to 65535.
*
*
* \sideeffect
*   This function should not be called when the specified descriptor is active 
*   in the DMA transfer engine. This can be checked by calling CyDmaGetStatus().
*
*******************************************************************************/
#define Serial_USB_CyDmaSetNumDataElements(ch, descr, numEl) \
    do{ \
        CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].ctl = \
            ((CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].ctl & (uint32) ~CYDMA_DATA_NR) | ((uint32) (numEl))); \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaGetSrcAddress
****************************************************************************//**
*
*  Returns the source address for the specified descriptor.
*
*  \param ch:    Channel used by this function.
*  \param descr: Specifies descriptor (0 or 1) used by this function.
*
* \return
*  Source address written to specified descriptor.
*
*******************************************************************************/
#define Serial_USB_CyDmaGetSrcAddress(ch, descr)    CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].src


/*******************************************************************************
* Function Name: Serial_USB_CyDmaSetSrcAddress
****************************************************************************//**
*
*  Configures the source address for the specified descriptor.
*
*  \param ch:         Channel used by this function.
*  \param descr:      Descriptor (0 or 1) modified by this function.
*  \param srcAddress: Address of DMA transfer source.
*
*
* \sideeffect
*   This function should not be called when the specified descriptor is active 
*   in the DMA transfer engine. This can be checked by calling CyDmaGetStatus().
*
*******************************************************************************/
#define Serial_USB_CyDmaSetSrcAddress(ch, descr, srcAddress) \
    do{ \
        CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].src = (srcAddress); \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaGetDstAddress
****************************************************************************//**
*
*  Returns the destination address for the specified descriptor, set by
*  CyDmaSetDstAddress().
*
*  \param ch:    Channel used by this function.
*  \param descr: Specifies descriptor (0 or 1) used by this function.
*
* \return
*  Destination address written to specified descriptor.
*
*******************************************************************************/
#define Serial_USB_CyDmaGetDstAddress(ch, descr)    CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].dst


/*******************************************************************************
* Function Name: Serial_USB_CyDmaSetDstAddress
****************************************************************************//**
*
*  Configures the destination address for the specified descriptor.
*
*  \param ch:         Channel used by this function.
*  \param descr:      Descriptor (0 or 1) modified by this function.
*  \param dstAddress: Address of DMA transfer destination.
*
*
* \sideeffect
*   This function should not be called when the specified descriptor is active 
*   in the DMA transfer engine. This can be checked by calling CyDmaGetStatus().
*
*******************************************************************************/
#define Serial_USB_CyDmaSetDstAddress(ch, descr, dstAddress) \
    do{ \
        CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].dst = (dstAddress); \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaValidateDescriptor
****************************************************************************//**
*
*  Validates the specified descriptor after it has been invalidated.
*
*  \param ch:    Channel used by this function.
*  \param descr: Descriptor (0 or 1) modified by this function.
*
*
* \sideeffect
*   The status register associated with the specified descriptor is reset to 
*   zero after this function call.
*   This function should not be called when the specified descriptor is active 
*   in the DMA transfer engine. This can be checked by calling CyDmaGetStatus().
*
*******************************************************************************/
#define Serial_USB_CyDmaValidateDescriptor(ch, descr) \
    do{ \
        CYDMA_DESCR_BASE.descriptor[(ch)][(descr)].status = CYDMA_VALID; \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaChEnable
****************************************************************************//**
*
*  Enables the DMA ch.
*
*  \param ch: Channel used by this function.
*
*
* \sideeffect
*   If this function is called before DMA is completely configured the operation 
*   of the DMA is undefined and could result in system data corruption.
*
*******************************************************************************/
#define Serial_USB_CyDmaChEnable(ch) \
    do{ \
        CYDMA_CH_CTL_BASE.ctl[(ch)] |= CYDMA_ENABLED; \
    }while(0)


/*******************************************************************************
* Function Name: CyDmaChDisable
****************************************************************************//**
*
*  Disables the DMA ch.
*
*  \param ch: Channel used by this function.
*
*
* \sideeffect
*  If this function is called during a DMA transfer the transfer is aborted.
*
*******************************************************************************/
#define Serial_USB_CyDmaChDisable(ch) \
    do{ \
        CYDMA_CH_CTL_BASE.ctl[(ch)] &= (uint32) ~CYDMA_ENABLED; \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaTriggerIn
****************************************************************************//**
*
*  Triggers the DMA channel to execute a transfer. The tr_in signal is 
*  triggered.
*
*  \param trSel: trigger to be activated.
*
*
*******************************************************************************/
#define Serial_USB_DMA_USB_REQ_TR_OUT (0xC0020100U)
#define Serial_USB_CyDmaTriggerIn(trSel) \
    do{ \
        CYDMA_TR_CTL_REG = Serial_USB_DMA_USB_REQ_TR_OUT | (uint32)(trSel); \
    }while(0)


/*******************************************************************************
* Function Name: Serial_USB_CyDmaTriggerOut
****************************************************************************//**
*
*  Triggers the DMA channel to generate a transfer completion signal without 
*  actual transfer executed. The tr_out signal is triggered.
*
*  \param trSel: trigger to be activated.
*
*
*******************************************************************************/
#define Serial_USB_DMA_USB_BURST_END_TR_OUT  (0xC0020300U)
#define Serial_USB_CyDmaTriggerOut(trSel) \
    do{ \
        CYDMA_TR_CTL_REG = Serial_USB_DMA_USB_BURST_END_TR_OUT | (uint32)(trSel); \
    }while(0)


#endif /* (CY_USBFS_Serial_USB_cydmac_H) */


/* [] END OF FILE */
