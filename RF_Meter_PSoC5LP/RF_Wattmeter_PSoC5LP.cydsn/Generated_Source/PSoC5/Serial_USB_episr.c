/***************************************************************************//**
* \file Serial_USB_episr.c
* \version 3.20
*
* \brief
*  This file contains the Data endpoint Interrupt Service Routines.
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial_USB_pvt.h"
#include "Serial_USB_cydmac.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */


#if (Serial_USB_EP1_ISR_ACTIVE)
    /******************************************************************************
    * Function Name: Serial_USB_EP_1_ISR
    ***************************************************************************//**
    *
    *  Endpoint 1 Interrupt Service Routine
    *
    ******************************************************************************/
    CY_ISR(Serial_USB_EP_1_ISR)
    {

    #ifdef Serial_USB_EP_1_ISR_ENTRY_CALLBACK
        Serial_USB_EP_1_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_1_ISR_ENTRY_CALLBACK) */

        /* `#START EP1_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    
        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP1_INTR);
            
        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to be read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP1].addr & Serial_USB_DIR_IN))
    #endif /* (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP1].epCr0;
            
            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP1) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP1].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }

            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP1].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if (Serial_USB_midi_out_ep == Serial_USB_EP1)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */
    
        /* `#START EP1_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_1_ISR_EXIT_CALLBACK
        Serial_USB_EP_1_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_1_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }

#endif /* (Serial_USB_EP1_ISR_ACTIVE) */


#if (Serial_USB_EP2_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_2_ISR
    ****************************************************************************//**
    *
    *  Endpoint 2 Interrupt Service Routine.
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_2_ISR)
    {
    #ifdef Serial_USB_EP_2_ISR_ENTRY_CALLBACK
        Serial_USB_EP_2_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_2_ISR_ENTRY_CALLBACK) */

        /* `#START EP2_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */

        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP2_INTR);

        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to be read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP2].addr & Serial_USB_DIR_IN))
    #endif /* (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {            
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP2].epCr0;
            
            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP2) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP2].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }

            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP2].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if (Serial_USB_midi_out_ep == Serial_USB_EP2)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */        
    
        /* `#START EP2_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_2_ISR_EXIT_CALLBACK
        Serial_USB_EP_2_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_2_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP2_ISR_ACTIVE) */


#if (Serial_USB_EP3_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_3_ISR
    ****************************************************************************//**
    *
    *  Endpoint 3 Interrupt Service Routine.
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_3_ISR)
    {
    #ifdef Serial_USB_EP_3_ISR_ENTRY_CALLBACK
        Serial_USB_EP_3_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_3_ISR_ENTRY_CALLBACK) */

        /* `#START EP3_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */

        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP3_INTR);    

        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to be read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP3].addr & Serial_USB_DIR_IN))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {            
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP3].epCr0;

            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP3) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP3].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }

            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP3].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if (Serial_USB_midi_out_ep == Serial_USB_EP3)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */        

        /* `#START EP3_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_3_ISR_EXIT_CALLBACK
        Serial_USB_EP_3_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_3_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP3_ISR_ACTIVE) */


#if (Serial_USB_EP4_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_4_ISR
    ****************************************************************************//**
    *
    *  Endpoint 4 Interrupt Service Routine.
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_4_ISR)
    {
    #ifdef Serial_USB_EP_4_ISR_ENTRY_CALLBACK
        Serial_USB_EP_4_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_4_ISR_ENTRY_CALLBACK) */

        /* `#START EP4_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */

        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP4_INTR);
        
        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP4].addr & Serial_USB_DIR_IN))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP4].epCr0;

            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP4) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP4].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }

            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP4].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if(Serial_USB_midi_out_ep == Serial_USB_EP4)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */        

        /* `#START EP4_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_4_ISR_EXIT_CALLBACK
        Serial_USB_EP_4_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_4_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP4_ISR_ACTIVE) */


#if (Serial_USB_EP5_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_5_ISR
    ****************************************************************************//**
    *
    *  Endpoint 5 Interrupt Service Routine
    *
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_5_ISR)
    {
    #ifdef Serial_USB_EP_5_ISR_ENTRY_CALLBACK
        Serial_USB_EP_5_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_5_ISR_ENTRY_CALLBACK) */

        /* `#START EP5_USER_CODE` Place your code here */

        /* `#END` */

    #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && \
                 Serial_USB_ISR_SERVICE_MIDI_OUT && CY_PSOC3)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */

        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP5_INTR);
    
        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP5].addr & Serial_USB_DIR_IN))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {            
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP5].epCr0;

            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP5) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP5].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }

            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP5].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))        
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if (Serial_USB_midi_out_ep == Serial_USB_EP5)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */

        /* `#START EP5_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_5_ISR_EXIT_CALLBACK
        Serial_USB_EP_5_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_5_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP5_ISR_ACTIVE) */


#if (Serial_USB_EP6_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_6_ISR
    ****************************************************************************//**
    *
    *  Endpoint 6 Interrupt Service Routine.
    *
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_6_ISR)
    {
    #ifdef Serial_USB_EP_6_ISR_ENTRY_CALLBACK
        Serial_USB_EP_6_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_6_ISR_ENTRY_CALLBACK) */

        /* `#START EP6_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */

        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP6_INTR);
        
        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP6].addr & Serial_USB_DIR_IN))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP6].epCr0;

            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP6) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP6].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }
            
            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP6].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if (Serial_USB_midi_out_ep == Serial_USB_EP6)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */

        /* `#START EP6_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_6_ISR_EXIT_CALLBACK
        Serial_USB_EP_6_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_6_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP6_ISR_ACTIVE) */


#if (Serial_USB_EP7_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_7_ISR
    ****************************************************************************//**
    *
    *  Endpoint 7 Interrupt Service Routine.
    *
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_7_ISR)
    {
    #ifdef Serial_USB_EP_7_ISR_ENTRY_CALLBACK
        Serial_USB_EP_7_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_7_ISR_ENTRY_CALLBACK) */

        /* `#START EP7_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    
        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP7_INTR);
        
        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP7].addr & Serial_USB_DIR_IN))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {           
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP7].epCr0;

            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP7) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP7].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }
            
            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP7].apiEpState = Serial_USB_EVENT_PENDING;
        }


    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if(Serial_USB_midi_out_ep == Serial_USB_EP7)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */

        /* `#START EP7_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_7_ISR_EXIT_CALLBACK
        Serial_USB_EP_7_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_7_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP7_ISR_ACTIVE) */


#if (Serial_USB_EP8_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_EP_8_ISR
    ****************************************************************************//**
    *
    *  Endpoint 8 Interrupt Service Routine
    *
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_EP_8_ISR)
    {
    #ifdef Serial_USB_EP_8_ISR_ENTRY_CALLBACK
        Serial_USB_EP_8_ISR_EntryCallback();
    #endif /* (Serial_USB_EP_8_ISR_ENTRY_CALLBACK) */

        /* `#START EP8_USER_CODE` Place your code here */

        /* `#END` */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        {
            uint8 intEn = EA;
            CyGlobalIntEnable;  /* Enable nested interrupts. */
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */

        Serial_USB_ClearSieEpInterruptSource(Serial_USB_SIE_INT_EP8_INTR);
        
        /* Notifies user that transfer IN or OUT transfer is completed.
        * IN endpoint: endpoint buffer can be reloaded, Host is read data.
        * OUT endpoint: data is ready to read from endpoint buffer. 
        */
    #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        if (0u != (Serial_USB_EP[Serial_USB_EP8].addr & Serial_USB_DIR_IN))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
        {
            /* Read CR0 register to clear SIE lock. */
            (void) Serial_USB_SIE_EP_BASE.sieEp[Serial_USB_EP8].epCr0;

            /* Toggle all endpoint types except ISOC. */
            if (Serial_USB_GET_EP_TYPE(Serial_USB_EP8) != Serial_USB_EP_TYPE_ISOC)
            {
                Serial_USB_EP[Serial_USB_EP8].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
            }

            /* EP_MANAGEMENT_DMA_AUTO (Ticket ID# 214187): For OUT endpoint this event is used to notify
            * user that DMA has completed copying data from OUT endpoint which is not completely true.
            * Because last chunk of data is being copied.
            * For CY_PSOC 3/5LP: it is acceptable as DMA is really fast.
            * For CY_PSOC4: this event is set in Arbiter interrupt (source is DMA_TERMIN).
            */
            Serial_USB_EP[Serial_USB_EP8].apiEpState = Serial_USB_EVENT_PENDING;
        }

    #if (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO))
        #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
            !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
            if (Serial_USB_midi_out_ep == Serial_USB_EP8)
            {
                Serial_USB_MIDI_OUT_Service();
            }
        #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
    #endif /* (!(CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)) */

        /* `#START EP8_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_EP_8_ISR_EXIT_CALLBACK
        Serial_USB_EP_8_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_8_ISR_EXIT_CALLBACK) */

    #if (CY_PSOC3 && defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
        !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
        
            EA = intEn; /* Restore nested interrupt configuration. */
        }
    #endif /* (CY_PSOC3 && Serial_USB_ISR_SERVICE_MIDI_OUT) */
    }
#endif /* (Serial_USB_EP8_ISR_ACTIVE) */


#if (Serial_USB_SOF_ISR_ACTIVE)
    /*******************************************************************************
    * Function Name: Serial_USB_SOF_ISR
    ****************************************************************************//**
    *
    *  Start of Frame Interrupt Service Routine.
    *
    *
    *******************************************************************************/
    CY_ISR(Serial_USB_SOF_ISR)
    {
    #ifdef Serial_USB_SOF_ISR_ENTRY_CALLBACK
        Serial_USB_SOF_ISR_EntryCallback();
    #endif /* (Serial_USB_SOF_ISR_ENTRY_CALLBACK) */

        /* `#START SOF_USER_CODE` Place your code here */

        /* `#END` */

        Serial_USB_ClearSieInterruptSource(Serial_USB_INTR_SIE_SOF_INTR);

    #ifdef Serial_USB_SOF_ISR_EXIT_CALLBACK
        Serial_USB_SOF_ISR_ExitCallback();
    #endif /* (Serial_USB_SOF_ISR_EXIT_CALLBACK) */
    }
#endif /* (Serial_USB_SOF_ISR_ACTIVE) */


#if (Serial_USB_BUS_RESET_ISR_ACTIVE)
/*******************************************************************************
* Function Name: Serial_USB_BUS_RESET_ISR
****************************************************************************//**
*
*  USB Bus Reset Interrupt Service Routine.  Calls _Start with the same
*  parameters as the last USER call to _Start
*
*
*******************************************************************************/
CY_ISR(Serial_USB_BUS_RESET_ISR)
{
#ifdef Serial_USB_BUS_RESET_ISR_ENTRY_CALLBACK
    Serial_USB_BUS_RESET_ISR_EntryCallback();
#endif /* (Serial_USB_BUS_RESET_ISR_ENTRY_CALLBACK) */

    /* `#START BUS_RESET_USER_CODE` Place your code here */

    /* `#END` */

    Serial_USB_ClearSieInterruptSource(Serial_USB_INTR_SIE_BUS_RESET_INTR);

    Serial_USB_ReInitComponent();

#ifdef Serial_USB_BUS_RESET_ISR_EXIT_CALLBACK
    Serial_USB_BUS_RESET_ISR_ExitCallback();
#endif /* (Serial_USB_BUS_RESET_ISR_EXIT_CALLBACK) */
}
#endif /* (Serial_USB_BUS_RESET_ISR_ACTIVE) */


#if (Serial_USB_LPM_ACTIVE)
/***************************************************************************
* Function Name: Serial_USB_INTR_LPM_ISR
************************************************************************//**
*
*   Interrupt Service Routine for LPM of the interrupt sources.
*
*
***************************************************************************/
CY_ISR(Serial_USB_LPM_ISR)
{
#ifdef Serial_USB_LPM_ISR_ENTRY_CALLBACK
    Serial_USB_LPM_ISR_EntryCallback();
#endif /* (Serial_USB_LPM_ISR_ENTRY_CALLBACK) */

    /* `#START LPM_BEGIN_USER_CODE` Place your code here */

    /* `#END` */

    Serial_USB_ClearSieInterruptSource(Serial_USB_INTR_SIE_LPM_INTR);

    /* `#START LPM_END_USER_CODE` Place your code here */

    /* `#END` */

#ifdef Serial_USB_LPM_ISR_EXIT_CALLBACK
    Serial_USB_LPM_ISR_ExitCallback();
#endif /* (Serial_USB_LPM_ISR_EXIT_CALLBACK) */
}
#endif /* (Serial_USB_LPM_ACTIVE) */


#if (Serial_USB_EP_MANAGEMENT_DMA && Serial_USB_ARB_ISR_ACTIVE)
    /***************************************************************************
    * Function Name: Serial_USB_ARB_ISR
    ************************************************************************//**
    *
    *  Arbiter Interrupt Service Routine.
    *
    *
    ***************************************************************************/
    CY_ISR(Serial_USB_ARB_ISR)
    {
        uint8 arbIntrStatus;
        uint8 epStatus;
        uint8 ep = Serial_USB_EP1;

    #ifdef Serial_USB_ARB_ISR_ENTRY_CALLBACK
        Serial_USB_ARB_ISR_EntryCallback();
    #endif /* (Serial_USB_ARB_ISR_ENTRY_CALLBACK) */

        /* `#START ARB_BEGIN_USER_CODE` Place your code here */

        /* `#END` */

        /* Get pending ARB interrupt sources. */
        arbIntrStatus = Serial_USB_ARB_INT_SR_REG;

        while (0u != arbIntrStatus)
        {
            /* Check which EP is interrupt source. */
            if (0u != (arbIntrStatus & 0x01u))
            {
                /* Get endpoint enable interrupt sources. */
                epStatus = (Serial_USB_ARB_EP_BASE.arbEp[ep].epSr & Serial_USB_ARB_EP_BASE.arbEp[ep].epIntEn);

                /* Handle IN endpoint buffer full event: happens only once when endpoint buffer is loaded. */
                if (0u != (epStatus & Serial_USB_ARB_EPX_INT_IN_BUF_FULL))
                {
                    if (0u != (Serial_USB_EP[ep].addr & Serial_USB_DIR_IN))
                    {
                        /* Clear data ready status. */
                        Serial_USB_ARB_EP_BASE.arbEp[ep].epCfg &= (uint8) ~Serial_USB_ARB_EPX_CFG_IN_DATA_RDY;

                    #if (CY_PSOC3 || CY_PSOC5LP)
                        #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO && (Serial_USB_EP_DMA_AUTO_OPT == 0u))
                            /* Set up common area DMA with rest of data. */
                            if(Serial_USB_inLength[ep] > Serial_USB_DMA_BYTES_PER_BURST)
                            {
                                Serial_USB_LoadNextInEP(ep, 0u);
                            }
                            else
                            {
                                Serial_USB_inBufFull[ep] = 1u;
                            }
                        #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO && (Serial_USB_EP_DMA_AUTO_OPT == 0u)) */
                    #endif /* (CY_PSOC3 || CY_PSOC5LP) */

                        /* Arm IN endpoint. */
                        Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0 = Serial_USB_EP[ep].epMode;

                    #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && Serial_USB_ISR_SERVICE_MIDI_IN)
                        if (ep == Serial_USB_midi_in_ep)
                        {
                            /* Clear MIDI input pointer. */
                            Serial_USB_midiInPointer = 0u;
                        }
                    #endif /* (Serial_USB_ENABLE_MIDI_STREAMING) */
                    }
                }

            #if (Serial_USB_EP_MANAGEMENT_DMA_MANUAL)
                /* Handle DMA completion event for OUT endpoints. */
                if (0u != (epStatus & Serial_USB_ARB_EPX_SR_DMA_GNT))
                {
                    if (0u == (Serial_USB_EP[ep].addr & Serial_USB_DIR_IN))
                    {
                        /* Notify user that data has been copied from endpoint buffer. */
                        Serial_USB_EP[ep].apiEpState = Serial_USB_NO_EVENT_PENDING;

                        /* DMA done coping data: OUT endpoint has to be re-armed by user. */
                    }
                }
            #endif /* (Serial_USB_EP_MANAGEMENT_DMA_MANUAL) */

            #if (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO)
                /* Handle DMA completion event for OUT endpoints. */
                if (0u != (epStatus & Serial_USB_ARB_EPX_INT_DMA_TERMIN))
                {
                    uint32 channelNum = Serial_USB_DmaChan[ep];

                    /* Restore burst counter for endpoint. */
                    Serial_USB_DmaEpBurstCnt[ep] = Serial_USB_DMA_GET_BURST_CNT(Serial_USB_DmaEpBurstCntBackup[ep]);

                    /* Disable DMA channel to restore descriptor configuration. The on-going transfer is aborted. */
                    Serial_USB_CyDmaChDisable(channelNum);

                    /* Generate DMA tr_out signal to notify USB IP that DMA is done. This signal is not generated
                    * when transfer was aborted (it occurs when host writes less bytes than buffer size).
                    */
                    Serial_USB_CyDmaTriggerOut(Serial_USB_DmaBurstEndOut[ep]);

                    /* Restore destination address for output endpoint. */
                    Serial_USB_CyDmaSetDstAddress(channelNum, Serial_USB_DMA_DESCR0, (void*) ((uint32) Serial_USB_DmaEpBufferAddrBackup[ep]));
                    Serial_USB_CyDmaSetDstAddress(channelNum, Serial_USB_DMA_DESCR1, (void*) ((uint32) Serial_USB_DmaEpBufferAddrBackup[ep] +
                                                                                                                   Serial_USB_DMA_BYTES_PER_BURST));

                    /* Restore number of data elements to transfer which was adjusted for last burst. */
                    if (0u != (Serial_USB_DmaEpLastBurstEl[ep] & Serial_USB_DMA_DESCR_REVERT))
                    {
                        Serial_USB_CyDmaSetNumDataElements(channelNum, (Serial_USB_DmaEpLastBurstEl[ep] >> Serial_USB_DMA_DESCR_SHIFT),
                                                                             Serial_USB_DMA_GET_MAX_ELEM_PER_BURST(Serial_USB_DmaEpLastBurstEl[ep]));
                    }

                    /* Validate descriptor 0 and 1 (also reset current state). Command to start with descriptor 0. */
                    Serial_USB_CyDmaValidateDescriptor(channelNum, Serial_USB_DMA_DESCR0);
                    if (Serial_USB_DmaEpBurstCntBackup[ep] > 1u)
                    {
                        Serial_USB_CyDmaValidateDescriptor(channelNum, Serial_USB_DMA_DESCR1);
                    }
                    Serial_USB_CyDmaSetDescriptor0Next(channelNum);

                    /* Enable DMA channel: configuration complete. */
                    Serial_USB_CyDmaChEnable(channelNum);
                    
                    
                    /* Read CR0 register to clear SIE lock. */
                    (void) Serial_USB_SIE_EP_BASE.sieEp[ep].epCr0;
                    
                    /* Toggle all endpoint types except ISOC. */
                    if (Serial_USB_GET_EP_TYPE(ep) != Serial_USB_EP_TYPE_ISOC)
                    {
                        Serial_USB_EP[ep].epToggle ^= Serial_USB_EPX_CNT_DATA_TOGGLE;
                    }
            
                    /* Notify user that data has been copied from endpoint buffer. */
                    Serial_USB_EP[ep].apiEpState = Serial_USB_EVENT_PENDING;
                    
                #if (defined(Serial_USB_ENABLE_MIDI_STREAMING) && \
                    !defined(Serial_USB_MAIN_SERVICE_MIDI_OUT) && Serial_USB_ISR_SERVICE_MIDI_OUT)
                    if (Serial_USB_midi_out_ep == ep)
                    {
                        Serial_USB_MIDI_OUT_Service();
                    }
                #endif /* (Serial_USB_ISR_SERVICE_MIDI_OUT) */
                }
            #endif /* (CY_PSOC4 && Serial_USB_EP_MANAGEMENT_DMA_AUTO) */


                /* `#START ARB_USER_CODE` Place your code here for handle Buffer Underflow/Overflow */

                /* `#END` */

            #ifdef Serial_USB_ARB_ISR_CALLBACK
                Serial_USB_ARB_ISR_Callback(ep, epStatus);
            #endif /* (Serial_USB_ARB_ISR_CALLBACK) */

                /* Clear serviced endpoint interrupt sources. */
                Serial_USB_ARB_EP_BASE.arbEp[ep].epSr = epStatus;
            }

            ++ep;
            arbIntrStatus >>= 1u;
        }

        /* `#START ARB_END_USER_CODE` Place your code here */

        /* `#END` */

    #ifdef Serial_USB_ARB_ISR_EXIT_CALLBACK
        Serial_USB_ARB_ISR_ExitCallback();
    #endif /* (Serial_USB_ARB_ISR_EXIT_CALLBACK) */
    }

#endif /*  (Serial_USB_ARB_ISR_ACTIVE && Serial_USB_EP_MANAGEMENT_DMA) */


#if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
#if (CY_PSOC4)

    /******************************************************************************
    * Function Name: Serial_USB_EPxDmaDone
    ***************************************************************************//**
    *
    * \internal
    *  Endpoint  DMA Done Interrupt Service Routine basic function .
    *  
    *  \param dmaCh
    *  number of DMA channel
    *  
    *  \param ep
    *  number of USB end point
    *  
    *  \param dmaDone
    *  transfer completion flag
    *  
    *  \return
    *   updated transfer completion flag
    *
    ******************************************************************************/
    CY_INLINE static void Serial_USB_EPxDmaDone(uint8 dmaCh, uint8 ep)
    {
        uint32 nextAddr;

        /* Manage data elements which remain to transfer. */
        if (0u != Serial_USB_DmaEpBurstCnt[ep])
        {
            if(Serial_USB_DmaEpBurstCnt[ep] <= 2u)
            {
                /* Adjust length of last burst. */
                Serial_USB_CyDmaSetNumDataElements(dmaCh,
                                                    ((uint32) Serial_USB_DmaEpLastBurstEl[ep] >> Serial_USB_DMA_DESCR_SHIFT),
                                                    ((uint32) Serial_USB_DmaEpLastBurstEl[ep] &  Serial_USB_DMA_BURST_BYTES_MASK));
            }
            

            /* Advance source for input endpoint or destination for output endpoint. */
            if (0u != (Serial_USB_EP[ep].addr & Serial_USB_DIR_IN))
            {
                /* Change source for descriptor 0. */
                nextAddr = (uint32) Serial_USB_CyDmaGetSrcAddress(dmaCh, Serial_USB_DMA_DESCR0);
                nextAddr += (2u * Serial_USB_DMA_BYTES_PER_BURST);
                Serial_USB_CyDmaSetSrcAddress(dmaCh, Serial_USB_DMA_DESCR0, (void *) nextAddr);

                /* Change source for descriptor 1. */
                nextAddr += Serial_USB_DMA_BYTES_PER_BURST;
                Serial_USB_CyDmaSetSrcAddress(dmaCh, Serial_USB_DMA_DESCR1, (void *) nextAddr);
            }
            else
            {
                /* Change destination for descriptor 0. */
                nextAddr  = (uint32) Serial_USB_CyDmaGetDstAddress(dmaCh, Serial_USB_DMA_DESCR0);
                nextAddr += (2u * Serial_USB_DMA_BYTES_PER_BURST);
                Serial_USB_CyDmaSetDstAddress(dmaCh, Serial_USB_DMA_DESCR0, (void *) nextAddr);

                /* Change destination for descriptor 1. */
                nextAddr += Serial_USB_DMA_BYTES_PER_BURST;
                Serial_USB_CyDmaSetDstAddress(dmaCh, Serial_USB_DMA_DESCR1, (void *) nextAddr);
            }

            /* Enable DMA to execute transfer as it was disabled because there were no valid descriptor. */
            Serial_USB_CyDmaValidateDescriptor(dmaCh, Serial_USB_DMA_DESCR0);
            
            --Serial_USB_DmaEpBurstCnt[ep];
            if (0u != Serial_USB_DmaEpBurstCnt[ep])
            {
                Serial_USB_CyDmaValidateDescriptor(dmaCh, Serial_USB_DMA_DESCR1);
                --Serial_USB_DmaEpBurstCnt[ep];
            }
            
            Serial_USB_CyDmaChEnable (dmaCh);
            Serial_USB_CyDmaTriggerIn(Serial_USB_DmaReqOut[ep]);
        }
        else
        {
            /* No data to transfer. False DMA trig. Ignore.  */
        }

    }

    #if (Serial_USB_DMA1_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP1_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 1 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP1_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP1_DMA_CH,
                                                  Serial_USB_EP1);
                
        }
    #endif /* (Serial_USB_DMA1_ACTIVE) */


    #if (Serial_USB_DMA2_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP2_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 2 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP2_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP2_DMA_CH,
                                                  Serial_USB_EP2);
        }
    #endif /* (Serial_USB_DMA2_ACTIVE) */


    #if (Serial_USB_DMA3_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP3_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 3 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP3_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP3_DMA_CH,
                                                  Serial_USB_EP3);
        }
    #endif /* (Serial_USB_DMA3_ACTIVE) */


    #if (Serial_USB_DMA4_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP4_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 4 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP4_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP4_DMA_CH,
                                                  Serial_USB_EP4);
        }
    #endif /* (Serial_USB_DMA4_ACTIVE) */


    #if (Serial_USB_DMA5_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP5_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 5 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP5_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP5_DMA_CH,
                                                  Serial_USB_EP5);
        }
    #endif /* (Serial_USB_DMA5_ACTIVE) */


    #if (Serial_USB_DMA6_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP6_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 6 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP6_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP6_DMA_CH,
                                                  Serial_USB_EP6);
        }
    #endif /* (Serial_USB_DMA6_ACTIVE) */


    #if (Serial_USB_DMA7_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP7_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 7 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP7_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP7_DMA_CH,
                                                  Serial_USB_EP7);
        }
    #endif /* (Serial_USB_DMA7_ACTIVE) */


    #if (Serial_USB_DMA8_ACTIVE)
        /******************************************************************************
        * Function Name: Serial_USB_EP8_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  Endpoint 8 DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        void Serial_USB_EP8_DMA_DONE_ISR(void)
        {

            Serial_USB_EPxDmaDone((uint8)Serial_USB_EP8_DMA_CH,
                                                  Serial_USB_EP8);
        }
    #endif /* (Serial_USB_DMA8_ACTIVE) */


#else
    #if (Serial_USB_EP_DMA_AUTO_OPT == 0u)
        /******************************************************************************
        * Function Name: Serial_USB_EP_DMA_DONE_ISR
        ***************************************************************************//**
        *
        *  DMA Done Interrupt Service Routine.
        *
        *
        ******************************************************************************/
        CY_ISR(Serial_USB_EP_DMA_DONE_ISR)
        {
            uint8 int8Status;
            uint8 int17Status;
            uint8 ep_status;
            uint8 ep = Serial_USB_EP1;

        #ifdef Serial_USB_EP_DMA_DONE_ISR_ENTRY_CALLBACK
            Serial_USB_EP_DMA_DONE_ISR_EntryCallback();
        #endif /* (Serial_USB_EP_DMA_DONE_ISR_ENTRY_CALLBACK) */

            /* `#START EP_DMA_DONE_BEGIN_USER_CODE` Place your code here */

            /* `#END` */

            /* Read clear on read status register with EP source of interrupt. */
            int17Status = Serial_USB_EP17_DMA_Done_SR_Read() & Serial_USB_EP17_SR_MASK;
            int8Status  = Serial_USB_EP8_DMA_Done_SR_Read()  & Serial_USB_EP8_SR_MASK;

            while (int8Status != 0u)
            {
                while (int17Status != 0u)
                {
                    if ((int17Status & 1u) != 0u)  /* If EpX interrupt present. */
                    {
                        /* Read Endpoint Status Register. */
                        ep_status = Serial_USB_ARB_EP_BASE.arbEp[ep].epSr;

                        if ((0u == (ep_status & Serial_USB_ARB_EPX_SR_IN_BUF_FULL)) &&
                            (0u ==Serial_USB_inBufFull[ep]))
                        {
                            /* `#START EP_DMA_DONE_USER_CODE` Place your code here */

                            /* `#END` */

                        #ifdef Serial_USB_EP_DMA_DONE_ISR_CALLBACK
                            Serial_USB_EP_DMA_DONE_ISR_Callback();
                        #endif /* (Serial_USB_EP_DMA_DONE_ISR_CALLBACK) */

                            /* Transfer again 2 last bytes into pre-fetch endpoint area. */
                            Serial_USB_ARB_EP_BASE.arbEp[ep].rwWaMsb = 0u;
                            Serial_USB_ARB_EP_BASE.arbEp[ep].rwWa = (Serial_USB_DMA_BYTES_PER_BURST * ep) - Serial_USB_DMA_BYTES_REPEAT;
                            Serial_USB_LoadNextInEP(ep, 1u);

                            /* Set Data ready status to generate DMA request. */
                            Serial_USB_ARB_EP_BASE.arbEp[ep].epCfg |= Serial_USB_ARB_EPX_CFG_IN_DATA_RDY;
                        }
                    }

                    ep++;
                    int17Status >>= 1u;
                }

                int8Status >>= 1u;

                if (int8Status != 0u)
                {
                    /* Prepare pointer for EP8. */
                    ep = Serial_USB_EP8;
                    int17Status = int8Status & 0x01u;
                }
            }

            /* `#START EP_DMA_DONE_END_USER_CODE` Place your code here */

            /* `#END` */

    #ifdef Serial_USB_EP_DMA_DONE_ISR_EXIT_CALLBACK
        Serial_USB_EP_DMA_DONE_ISR_ExitCallback();
    #endif /* (Serial_USB_EP_DMA_DONE_ISR_EXIT_CALLBACK) */
        }
    #endif /* (Serial_USB_EP_DMA_AUTO_OPT == 0u) */
#endif /* (CY_PSOC4) */
#endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */


#if (CY_PSOC4)
    /***************************************************************************
    * Function Name: Serial_USB_IntrHandler
    ************************************************************************//**
    *
    *   Interrupt handler for Hi/Mid/Low ISRs.
    *
    *  regCause - The cause register of interrupt. One of the three variants:
    *       Serial_USB_INTR_CAUSE_LO_REG - Low interrupts.
    *       Serial_USB_INTR_CAUSE_MED_REG - Med interrupts.
    *       Serial_USB_INTR_CAUSE_HI_REG - - High interrupts.
    *
    *
    ***************************************************************************/
    CY_INLINE static void Serial_USB_IntrHandler(uint32 intrCause)
    {
        /* Array of pointers to component interrupt handlers. */
        static const cyisraddress Serial_USB_isrCallbacks[] =
        {

        };

        uint32 cbIdx = 0u;

        /* Check arbiter interrupt source first. */
        if (0u != (intrCause & Serial_USB_INTR_CAUSE_ARB_INTR))
        {
            Serial_USB_isrCallbacks[Serial_USB_ARB_EP_INTR_NUM]();
        }

        /* Check all other interrupt sources (except arbiter and resume). */
        intrCause = (intrCause  & Serial_USB_INTR_CAUSE_CTRL_INTR_MASK) |
                    ((intrCause & Serial_USB_INTR_CAUSE_EP1_8_INTR_MASK) >>
                                  Serial_USB_INTR_CAUSE_EP_INTR_SHIFT);

        /* Call interrupt handlers for active interrupt sources. */
        while (0u != intrCause)
        {
            if (0u != (intrCause & 0x1u))
            {
                 Serial_USB_isrCallbacks[cbIdx]();
            }

            intrCause >>= 1u;
            ++cbIdx;
        }
    }


    /***************************************************************************
    * Function Name: Serial_USB_INTR_HI_ISR
    ************************************************************************//**
    *
    *   Interrupt Service Routine for the high group of the interrupt sources.
    *
    *
    ***************************************************************************/
    CY_ISR(Serial_USB_INTR_HI_ISR)
    {
        Serial_USB_IntrHandler(Serial_USB_INTR_CAUSE_HI_REG);
    }

    /***************************************************************************
    * Function Name: Serial_USB_INTR_MED_ISR
    ************************************************************************//**
    *
    *   Interrupt Service Routine for the medium group of the interrupt sources.
    *
    *
    ***************************************************************************/
    CY_ISR(Serial_USB_INTR_MED_ISR)
    {
       Serial_USB_IntrHandler(Serial_USB_INTR_CAUSE_MED_REG);
    }

    /***************************************************************************
    * Function Name: Serial_USB_INTR_LO_ISR
    ************************************************************************//**
    *
    *   Interrupt Service Routine for the low group of the interrupt sources.
    *
    *
    ***************************************************************************/
    CY_ISR(Serial_USB_INTR_LO_ISR)
    {
        Serial_USB_IntrHandler(Serial_USB_INTR_CAUSE_LO_REG);
    }
#endif /* (CY_PSOC4) */


/* [] END OF FILE */
