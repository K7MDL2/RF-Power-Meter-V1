/*******************************************************************************
* File Name: Serial.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Serial.h"
#if (Serial_INTERNAL_CLOCK_USED)
    #include "Serial_IntClock.h"
#endif /* End Serial_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Serial_initVar = 0u;

#if (Serial_TX_INTERRUPT_ENABLED && Serial_TX_ENABLED)
    volatile uint8 Serial_txBuffer[Serial_TX_BUFFER_SIZE];
    volatile uint8 Serial_txBufferRead = 0u;
    uint8 Serial_txBufferWrite = 0u;
#endif /* (Serial_TX_INTERRUPT_ENABLED && Serial_TX_ENABLED) */

#if (Serial_RX_INTERRUPT_ENABLED && (Serial_RX_ENABLED || Serial_HD_ENABLED))
    uint8 Serial_errorStatus = 0u;
    volatile uint8 Serial_rxBuffer[Serial_RX_BUFFER_SIZE];
    volatile uint8 Serial_rxBufferRead  = 0u;
    volatile uint8 Serial_rxBufferWrite = 0u;
    volatile uint8 Serial_rxBufferLoopDetect = 0u;
    volatile uint8 Serial_rxBufferOverflow   = 0u;
    #if (Serial_RXHW_ADDRESS_ENABLED)
        volatile uint8 Serial_rxAddressMode = Serial_RX_ADDRESS_MODE;
        volatile uint8 Serial_rxAddressDetected = 0u;
    #endif /* (Serial_RXHW_ADDRESS_ENABLED) */
#endif /* (Serial_RX_INTERRUPT_ENABLED && (Serial_RX_ENABLED || Serial_HD_ENABLED)) */


/*******************************************************************************
* Function Name: Serial_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  Serial_Start() sets the initVar variable, calls the
*  Serial_Init() function, and then calls the
*  Serial_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The Serial_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time Serial_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the Serial_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Serial_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(Serial_initVar == 0u)
    {
        Serial_Init();
        Serial_initVar = 1u;
    }

    Serial_Enable();
}


/*******************************************************************************
* Function Name: Serial_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call Serial_Init() because
*  the Serial_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Serial_Init(void) 
{
    #if(Serial_RX_ENABLED || Serial_HD_ENABLED)

        #if (Serial_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(Serial_RX_VECT_NUM, &Serial_RXISR);
            CyIntSetPriority(Serial_RX_VECT_NUM, Serial_RX_PRIOR_NUM);
            Serial_errorStatus = 0u;
        #endif /* (Serial_RX_INTERRUPT_ENABLED) */

        #if (Serial_RXHW_ADDRESS_ENABLED)
            Serial_SetRxAddressMode(Serial_RX_ADDRESS_MODE);
            Serial_SetRxAddress1(Serial_RX_HW_ADDRESS1);
            Serial_SetRxAddress2(Serial_RX_HW_ADDRESS2);
        #endif /* End Serial_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Serial_RXBITCTR_PERIOD_REG = Serial_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Serial_RXSTATUS_MASK_REG  = Serial_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Serial_RX_ENABLED || Serial_HD_ENABLED*/

    #if(Serial_TX_ENABLED)
        #if (Serial_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(Serial_TX_VECT_NUM, &Serial_TXISR);
            CyIntSetPriority(Serial_TX_VECT_NUM, Serial_TX_PRIOR_NUM);
        #endif /* (Serial_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (Serial_TXCLKGEN_DP)
            Serial_TXBITCLKGEN_CTR_REG = Serial_BIT_CENTER;
            Serial_TXBITCLKTX_COMPLETE_REG = ((Serial_NUMBER_OF_DATA_BITS +
                        Serial_NUMBER_OF_START_BIT) * Serial_OVER_SAMPLE_COUNT) - 1u;
        #else
            Serial_TXBITCTR_PERIOD_REG = ((Serial_NUMBER_OF_DATA_BITS +
                        Serial_NUMBER_OF_START_BIT) * Serial_OVER_SAMPLE_8) - 1u;
        #endif /* End Serial_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (Serial_TX_INTERRUPT_ENABLED)
            Serial_TXSTATUS_MASK_REG = Serial_TX_STS_FIFO_EMPTY;
        #else
            Serial_TXSTATUS_MASK_REG = Serial_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Serial_TX_INTERRUPT_ENABLED*/

    #endif /* End Serial_TX_ENABLED */

    #if(Serial_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Serial_WriteControlRegister( \
            (Serial_ReadControlRegister() & (uint8)~Serial_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Serial_PARITY_TYPE << Serial_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Serial_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Serial_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call Serial_Enable() because the Serial_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Serial_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Serial_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (Serial_RX_ENABLED || Serial_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        Serial_RXBITCTR_CONTROL_REG |= Serial_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        Serial_RXSTATUS_ACTL_REG  |= Serial_INT_ENABLE;

        #if (Serial_RX_INTERRUPT_ENABLED)
            Serial_EnableRxInt();

            #if (Serial_RXHW_ADDRESS_ENABLED)
                Serial_rxAddressDetected = 0u;
            #endif /* (Serial_RXHW_ADDRESS_ENABLED) */
        #endif /* (Serial_RX_INTERRUPT_ENABLED) */
    #endif /* (Serial_RX_ENABLED || Serial_HD_ENABLED) */

    #if(Serial_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!Serial_TXCLKGEN_DP)
            Serial_TXBITCTR_CONTROL_REG |= Serial_CNTR_ENABLE;
        #endif /* End Serial_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        Serial_TXSTATUS_ACTL_REG |= Serial_INT_ENABLE;
        #if (Serial_TX_INTERRUPT_ENABLED)
            Serial_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            Serial_EnableTxInt();
        #endif /* (Serial_TX_INTERRUPT_ENABLED) */
     #endif /* (Serial_TX_INTERRUPT_ENABLED) */

    #if (Serial_INTERNAL_CLOCK_USED)
        Serial_IntClock_Start();  /* Enable the clock */
    #endif /* (Serial_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Serial_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Serial_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (Serial_RX_ENABLED || Serial_HD_ENABLED)
        Serial_RXBITCTR_CONTROL_REG &= (uint8) ~Serial_CNTR_ENABLE;
    #endif /* (Serial_RX_ENABLED || Serial_HD_ENABLED) */

    #if (Serial_TX_ENABLED)
        #if(!Serial_TXCLKGEN_DP)
            Serial_TXBITCTR_CONTROL_REG &= (uint8) ~Serial_CNTR_ENABLE;
        #endif /* (!Serial_TXCLKGEN_DP) */
    #endif /* (Serial_TX_ENABLED) */

    #if (Serial_INTERNAL_CLOCK_USED)
        Serial_IntClock_Stop();   /* Disable the clock */
    #endif /* (Serial_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (Serial_RX_ENABLED || Serial_HD_ENABLED)
        Serial_RXSTATUS_ACTL_REG  &= (uint8) ~Serial_INT_ENABLE;

        #if (Serial_RX_INTERRUPT_ENABLED)
            Serial_DisableRxInt();
        #endif /* (Serial_RX_INTERRUPT_ENABLED) */
    #endif /* (Serial_RX_ENABLED || Serial_HD_ENABLED) */

    #if (Serial_TX_ENABLED)
        Serial_TXSTATUS_ACTL_REG &= (uint8) ~Serial_INT_ENABLE;

        #if (Serial_TX_INTERRUPT_ENABLED)
            Serial_DisableTxInt();
        #endif /* (Serial_TX_INTERRUPT_ENABLED) */
    #endif /* (Serial_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Serial_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 Serial_ReadControlRegister(void) 
{
    #if (Serial_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(Serial_CONTROL_REG);
    #endif /* (Serial_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: Serial_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  Serial_WriteControlRegister(uint8 control) 
{
    #if (Serial_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       Serial_CONTROL_REG = control;
    #endif /* (Serial_CONTROL_REG_REMOVED) */
}


#if(Serial_RX_ENABLED || Serial_HD_ENABLED)
    /*******************************************************************************
    * Function Name: Serial_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      Serial_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      Serial_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      Serial_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      Serial_RX_STS_BREAK            Interrupt on break.
    *      Serial_RX_STS_OVERRUN          Interrupt on overrun error.
    *      Serial_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      Serial_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void Serial_SetRxInterruptMode(uint8 intSrc) 
    {
        Serial_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Serial_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  Serial_rxBuffer - RAM buffer pointer for save received data.
    *  Serial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Serial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Serial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Serial_ReadRxData(void) 
    {
        uint8 rxData;

    #if (Serial_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        Serial_DisableRxInt();

        locRxBufferRead  = Serial_rxBufferRead;
        locRxBufferWrite = Serial_rxBufferWrite;

        if( (Serial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = Serial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= Serial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            Serial_rxBufferRead = locRxBufferRead;

            if(Serial_rxBufferLoopDetect != 0u)
            {
                Serial_rxBufferLoopDetect = 0u;
                #if ((Serial_RX_INTERRUPT_ENABLED) && (Serial_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( Serial_HD_ENABLED )
                        if((Serial_CONTROL_REG & Serial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            Serial_RXSTATUS_MASK_REG  |= Serial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        Serial_RXSTATUS_MASK_REG  |= Serial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end Serial_HD_ENABLED */
                #endif /* ((Serial_RX_INTERRUPT_ENABLED) && (Serial_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = Serial_RXDATA_REG;
        }

        Serial_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = Serial_RXDATA_REG;

    #endif /* (Serial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Serial_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  Serial_RX_STS_FIFO_NOTEMPTY.
    *  Serial_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  Serial_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   Serial_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Serial_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Serial_ReadRxStatus(void) 
    {
        uint8 status;

        status = Serial_RXSTATUS_REG & Serial_RX_HW_MASK;

    #if (Serial_RX_INTERRUPT_ENABLED)
        if(Serial_rxBufferOverflow != 0u)
        {
            status |= Serial_RX_STS_SOFT_BUFF_OVER;
            Serial_rxBufferOverflow = 0u;
        }
    #endif /* (Serial_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Serial_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. Serial_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  Serial_rxBuffer - RAM buffer pointer for save received data.
    *  Serial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Serial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Serial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Serial_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (Serial_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        Serial_DisableRxInt();

        locRxBufferRead  = Serial_rxBufferRead;
        locRxBufferWrite = Serial_rxBufferWrite;

        if( (Serial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = Serial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= Serial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            Serial_rxBufferRead = locRxBufferRead;

            if(Serial_rxBufferLoopDetect != 0u)
            {
                Serial_rxBufferLoopDetect = 0u;
                #if( (Serial_RX_INTERRUPT_ENABLED) && (Serial_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( Serial_HD_ENABLED )
                        if((Serial_CONTROL_REG & Serial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            Serial_RXSTATUS_MASK_REG |= Serial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        Serial_RXSTATUS_MASK_REG |= Serial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end Serial_HD_ENABLED */
                #endif /* Serial_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = Serial_RXSTATUS_REG;
            if((rxStatus & Serial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = Serial_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Serial_RX_STS_BREAK | Serial_RX_STS_PAR_ERROR |
                                Serial_RX_STS_STOP_ERROR | Serial_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        Serial_EnableRxInt();

    #else

        rxStatus =Serial_RXSTATUS_REG;
        if((rxStatus & Serial_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = Serial_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (Serial_RX_STS_BREAK | Serial_RX_STS_PAR_ERROR |
                            Serial_RX_STS_STOP_ERROR | Serial_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (Serial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Serial_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 Serial_GetByte(void) 
    {
        
    #if (Serial_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        Serial_DisableRxInt();
        locErrorStatus = (uint16)Serial_errorStatus;
        Serial_errorStatus = 0u;
        Serial_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | Serial_ReadRxData() );
    #else
        return ( ((uint16)Serial_ReadRxStatus() << 8u) | Serial_ReadRxData() );
    #endif /* Serial_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: Serial_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  Serial_rxBufferWrite - used to calculate left bytes.
    *  Serial_rxBufferRead - used to calculate left bytes.
    *  Serial_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 Serial_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (Serial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        Serial_DisableRxInt();

        if(Serial_rxBufferRead == Serial_rxBufferWrite)
        {
            if(Serial_rxBufferLoopDetect != 0u)
            {
                size = Serial_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(Serial_rxBufferRead < Serial_rxBufferWrite)
        {
            size = (Serial_rxBufferWrite - Serial_rxBufferRead);
        }
        else
        {
            size = (Serial_RX_BUFFER_SIZE - Serial_rxBufferRead) + Serial_rxBufferWrite;
        }

        Serial_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((Serial_RXSTATUS_REG & Serial_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (Serial_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Serial_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_rxBufferWrite - cleared to zero.
    *  Serial_rxBufferRead - cleared to zero.
    *  Serial_rxBufferLoopDetect - cleared to zero.
    *  Serial_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void Serial_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        Serial_RXDATA_AUX_CTL_REG |= (uint8)  Serial_RX_FIFO_CLR;
        Serial_RXDATA_AUX_CTL_REG &= (uint8) ~Serial_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (Serial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        Serial_DisableRxInt();

        Serial_rxBufferRead = 0u;
        Serial_rxBufferWrite = 0u;
        Serial_rxBufferLoopDetect = 0u;
        Serial_rxBufferOverflow = 0u;

        Serial_EnableRxInt();

    #endif /* (Serial_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: Serial_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Serial__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Serial__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Serial__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Serial__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Serial__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Serial_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Serial_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Serial_RXHW_ADDRESS_ENABLED)
            #if(Serial_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* Serial_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Serial_CONTROL_REG & (uint8)~Serial_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Serial_CTRL_RXADDR_MODE0_SHIFT);
                Serial_CONTROL_REG = tmpCtrl;

                #if(Serial_RX_INTERRUPT_ENABLED && \
                   (Serial_RXBUFFERSIZE > Serial_FIFO_LENGTH) )
                    Serial_rxAddressMode = addressMode;
                    Serial_rxAddressDetected = 0u;
                #endif /* End Serial_RXBUFFERSIZE > Serial_FIFO_LENGTH*/
            #endif /* End Serial_CONTROL_REG_REMOVED */
        #else /* Serial_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End Serial_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Serial_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Serial_SetRxAddress1(uint8 address) 
    {
        Serial_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Serial_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Serial_SetRxAddress2(uint8 address) 
    {
        Serial_RXADDRESS2_REG = address;
    }

#endif  /* Serial_RX_ENABLED || Serial_HD_ENABLED*/


#if( (Serial_TX_ENABLED) || (Serial_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: Serial_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   Serial_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   Serial_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   Serial_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   Serial_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void Serial_SetTxInterruptMode(uint8 intSrc) 
    {
        Serial_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Serial_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  Serial_txBuffer - RAM buffer pointer for save data for transmission
    *  Serial_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Serial_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Serial_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Serial_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Serial_initVar != 0u)
        {
        #if (Serial_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            Serial_DisableTxInt();

            if( (Serial_txBufferRead == Serial_txBufferWrite) &&
                ((Serial_TXSTATUS_REG & Serial_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                Serial_TXDATA_REG = txDataByte;
            }
            else
            {
                if(Serial_txBufferWrite >= Serial_TX_BUFFER_SIZE)
                {
                    Serial_txBufferWrite = 0u;
                }

                Serial_txBuffer[Serial_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                Serial_txBufferWrite++;
            }

            Serial_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            Serial_TXDATA_REG = txDataByte;

        #endif /*(Serial_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: Serial_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 Serial_ReadTxStatus(void) 
    {
        return(Serial_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Serial_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_txBuffer - RAM buffer pointer for save data for transmission
    *  Serial_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Serial_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Serial_PutChar(uint8 txDataByte) 
    {
    #if (Serial_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            Serial_DisableTxInt();
        #endif /* (Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = Serial_txBufferWrite;
            locTxBufferRead  = Serial_txBufferRead;

        #if ((Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            Serial_EnableTxInt();
        #endif /* (Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(Serial_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((Serial_TXSTATUS_REG & Serial_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            Serial_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= Serial_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            Serial_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            Serial_DisableTxInt();
        #endif /* (Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            Serial_txBufferWrite = locTxBufferWrite;

        #if ((Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            Serial_EnableTxInt();
        #endif /* (Serial_TX_BUFFER_SIZE > Serial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (Serial_TXSTATUS_REG & Serial_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                Serial_SetPendingTxInt();
            }
        }

    #else

        while((Serial_TXSTATUS_REG & Serial_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        Serial_TXDATA_REG = txDataByte;

    #endif /* Serial_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Serial_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void Serial_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(Serial_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                Serial_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Serial_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void Serial_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(Serial_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                Serial_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Serial_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Serial_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(Serial_initVar != 0u)
        {
            Serial_PutChar(txDataByte);
            Serial_PutChar(0x0Du);
            Serial_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Serial_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  Serial_txBufferWrite - used to calculate left space.
    *  Serial_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Serial_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (Serial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        Serial_DisableTxInt();

        if(Serial_txBufferRead == Serial_txBufferWrite)
        {
            size = 0u;
        }
        else if(Serial_txBufferRead < Serial_txBufferWrite)
        {
            size = (Serial_txBufferWrite - Serial_txBufferRead);
        }
        else
        {
            size = (Serial_TX_BUFFER_SIZE - Serial_txBufferRead) +
                    Serial_txBufferWrite;
        }

        Serial_EnableTxInt();

    #else

        size = Serial_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & Serial_TX_STS_FIFO_FULL) != 0u)
        {
            size = Serial_FIFO_LENGTH;
        }
        else if((size & Serial_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (Serial_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: Serial_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_txBufferWrite - cleared to zero.
    *  Serial_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void Serial_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        Serial_TXDATA_AUX_CTL_REG |= (uint8)  Serial_TX_FIFO_CLR;
        Serial_TXDATA_AUX_CTL_REG &= (uint8) ~Serial_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (Serial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        Serial_DisableTxInt();

        Serial_txBufferRead = 0u;
        Serial_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        Serial_EnableTxInt();

    #endif /* (Serial_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: Serial_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   Serial_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   Serial_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   Serial_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   Serial_SEND_WAIT_REINIT - Performs both options: 
    *      Serial_SEND_BREAK and Serial_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Serial_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with Serial_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The Serial_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void Serial_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Serial_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(Serial_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == Serial_SEND_BREAK) ||
                (retMode == Serial_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                Serial_WriteControlRegister(Serial_ReadControlRegister() |
                                                      Serial_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                Serial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = Serial_TXSTATUS_REG;
                }
                while((tmpStat & Serial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == Serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Serial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = Serial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & Serial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == Serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Serial_REINIT) ||
                (retMode == Serial_SEND_WAIT_REINIT) )
            {
                Serial_WriteControlRegister(Serial_ReadControlRegister() &
                                              (uint8)~Serial_CTRL_HD_SEND_BREAK);
            }

        #else /* Serial_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == Serial_SEND_BREAK) ||
                (retMode == Serial_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (Serial_PARITY_TYPE != Serial__B_UART__NONE_REVB) || \
                                    (Serial_PARITY_TYPE_SW != 0u) )
                    Serial_WriteControlRegister(Serial_ReadControlRegister() |
                                                          Serial_CTRL_HD_SEND_BREAK);
                #endif /* End Serial_PARITY_TYPE != Serial__B_UART__NONE_REVB  */

                #if(Serial_TXCLKGEN_DP)
                    txPeriod = Serial_TXBITCLKTX_COMPLETE_REG;
                    Serial_TXBITCLKTX_COMPLETE_REG = Serial_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = Serial_TXBITCTR_PERIOD_REG;
                    Serial_TXBITCTR_PERIOD_REG = Serial_TXBITCTR_BREAKBITS8X;
                #endif /* End Serial_TXCLKGEN_DP */

                /* Send zeros */
                Serial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = Serial_TXSTATUS_REG;
                }
                while((tmpStat & Serial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == Serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Serial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = Serial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & Serial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == Serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Serial_REINIT) ||
                (retMode == Serial_SEND_WAIT_REINIT) )
            {

            #if(Serial_TXCLKGEN_DP)
                Serial_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                Serial_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End Serial_TXCLKGEN_DP */

            #if( (Serial_PARITY_TYPE != Serial__B_UART__NONE_REVB) || \
                 (Serial_PARITY_TYPE_SW != 0u) )
                Serial_WriteControlRegister(Serial_ReadControlRegister() &
                                                      (uint8) ~Serial_CTRL_HD_SEND_BREAK);
            #endif /* End Serial_PARITY_TYPE != NONE */
            }
        #endif    /* End Serial_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Serial_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       Serial_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       Serial_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears Serial_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void Serial_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( Serial_CONTROL_REG_REMOVED == 0u )
            Serial_WriteControlRegister(Serial_ReadControlRegister() |
                                                  Serial_CTRL_MARK);
        #endif /* End Serial_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( Serial_CONTROL_REG_REMOVED == 0u )
            Serial_WriteControlRegister(Serial_ReadControlRegister() &
                                                  (uint8) ~Serial_CTRL_MARK);
        #endif /* End Serial_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndSerial_TX_ENABLED */

#if(Serial_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Serial_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void Serial_LoadRxConfig(void) 
    {
        Serial_WriteControlRegister(Serial_ReadControlRegister() &
                                                (uint8)~Serial_CTRL_HD_SEND);
        Serial_RXBITCTR_PERIOD_REG = Serial_HD_RXBITCTR_INIT;

    #if (Serial_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        Serial_SetRxInterruptMode(Serial_INIT_RX_INTERRUPTS_MASK);
    #endif /* (Serial_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: Serial_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void Serial_LoadTxConfig(void) 
    {
    #if (Serial_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        Serial_SetRxInterruptMode(0u);
    #endif /* (Serial_RX_INTERRUPT_ENABLED) */

        Serial_WriteControlRegister(Serial_ReadControlRegister() | Serial_CTRL_HD_SEND);
        Serial_RXBITCTR_PERIOD_REG = Serial_HD_TXBITCTR_INIT;
    }

#endif  /* Serial_HD_ENABLED */


/* [] END OF FILE */
