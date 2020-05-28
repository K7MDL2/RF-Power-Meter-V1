/*******************************************************************************
* File Name: SerialINT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (Serial_RX_INTERRUPT_ENABLED && (Serial_RX_ENABLED || Serial_HD_ENABLED))
    /*******************************************************************************
    * Function Name: Serial_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_rxBuffer - RAM buffer pointer for save received data.
    *  Serial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Serial_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Serial_rxBufferOverflow - software overflow flag. Set to one
    *     when Serial_rxBufferWrite index overtakes
    *     Serial_rxBufferRead index.
    *  Serial_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Serial_rxBufferWrite is equal to
    *    Serial_rxBufferRead
    *  Serial_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Serial_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Serial_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef Serial_RXISR_ENTRY_CALLBACK
        Serial_RXISR_EntryCallback();
    #endif /* Serial_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START Serial_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = Serial_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in Serial_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (Serial_RX_STS_BREAK | 
                            Serial_RX_STS_PAR_ERROR |
                            Serial_RX_STS_STOP_ERROR | 
                            Serial_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                Serial_errorStatus |= readStatus & ( Serial_RX_STS_BREAK | 
                                                            Serial_RX_STS_PAR_ERROR | 
                                                            Serial_RX_STS_STOP_ERROR | 
                                                            Serial_RX_STS_OVERRUN);
                /* `#START Serial_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef Serial_RXISR_ERROR_CALLBACK
                Serial_RXISR_ERROR_Callback();
            #endif /* Serial_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & Serial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = Serial_RXDATA_REG;
            #if (Serial_RXHW_ADDRESS_ENABLED)
                if(Serial_rxAddressMode == (uint8)Serial__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & Serial_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & Serial_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Serial_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Serial_rxAddressDetected = 0u;
                        }
                    }
                    if(Serial_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        Serial_rxBuffer[Serial_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    Serial_rxBuffer[Serial_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                Serial_rxBuffer[Serial_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (Serial_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(Serial_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        Serial_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    Serial_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(Serial_rxBufferWrite >= Serial_RX_BUFFER_SIZE)
                    {
                        Serial_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(Serial_rxBufferWrite == Serial_rxBufferRead)
                    {
                        Serial_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (Serial_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            Serial_RXSTATUS_MASK_REG  &= (uint8)~Serial_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(Serial_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (Serial_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & Serial_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START Serial_RXISR_END` */

        /* `#END` */

    #ifdef Serial_RXISR_EXIT_CALLBACK
        Serial_RXISR_ExitCallback();
    #endif /* Serial_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (Serial_RX_INTERRUPT_ENABLED && (Serial_RX_ENABLED || Serial_HD_ENABLED)) */


#if (Serial_TX_INTERRUPT_ENABLED && Serial_TX_ENABLED)
    /*******************************************************************************
    * Function Name: Serial_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_txBuffer - RAM buffer pointer for transmit data from.
    *  Serial_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  Serial_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Serial_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef Serial_TXISR_ENTRY_CALLBACK
        Serial_TXISR_EntryCallback();
    #endif /* Serial_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START Serial_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((Serial_txBufferRead != Serial_txBufferWrite) &&
             ((Serial_TXSTATUS_REG & Serial_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(Serial_txBufferRead >= Serial_TX_BUFFER_SIZE)
            {
                Serial_txBufferRead = 0u;
            }

            Serial_TXDATA_REG = Serial_txBuffer[Serial_txBufferRead];

            /* Set next pointer */
            Serial_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Serial_TXISR_END` */

        /* `#END` */

    #ifdef Serial_TXISR_EXIT_CALLBACK
        Serial_TXISR_ExitCallback();
    #endif /* Serial_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (Serial_TX_INTERRUPT_ENABLED && Serial_TX_ENABLED) */


/* [] END OF FILE */
