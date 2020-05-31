/*******************************************************************************
* File Name: I22OLED_INT.c
* Version 3.50
*
* Description:
*  This file provides the source code of Interrupt Service Routine (ISR)
*  for the I2C component.
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I22OLED_PVT.h"
#include "cyapicallbacks.h"


/*******************************************************************************
*  Place your includes, defines and code here.
********************************************************************************/
/* `#START I22OLED_ISR_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: I22OLED_ISR
********************************************************************************
*
* Summary:
*  The handler for the I2C interrupt. The slave and master operations are
*  handled here.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
CY_ISR(I22OLED_ISR)
{
#if (I22OLED_MODE_SLAVE_ENABLED)
   uint8  tmp8;
#endif  /* (I22OLED_MODE_SLAVE_ENABLED) */

    uint8  tmpCsr;
    
#ifdef I22OLED_ISR_ENTRY_CALLBACK
    I22OLED_ISR_EntryCallback();
#endif /* I22OLED_ISR_ENTRY_CALLBACK */
    

#if(I22OLED_TIMEOUT_FF_ENABLED)
    if(0u != I22OLED_TimeoutGetStatus())
    {
        I22OLED_TimeoutReset();
        I22OLED_state = I22OLED_SM_EXIT_IDLE;
        /* I22OLED_CSR_REG should be cleared after reset */
    }
#endif /* (I22OLED_TIMEOUT_FF_ENABLED) */


    tmpCsr = I22OLED_CSR_REG;      /* Make copy as interrupts clear */

#if(I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED)
    if(I22OLED_CHECK_START_GEN(I22OLED_MCSR_REG))
    {
        I22OLED_CLEAR_START_GEN;

        /* Set transfer complete and error flags */
        I22OLED_mstrStatus |= (I22OLED_MSTAT_ERR_XFER |
                                        I22OLED_GET_MSTAT_CMPLT);

        /* Slave was addressed */
        I22OLED_state = I22OLED_SM_SLAVE;
    }
#endif /* (I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED) */


#if(I22OLED_MODE_MULTI_MASTER_ENABLED)
    if(I22OLED_CHECK_LOST_ARB(tmpCsr))
    {
        /* Set errors */
        I22OLED_mstrStatus |= (I22OLED_MSTAT_ERR_XFER     |
                                        I22OLED_MSTAT_ERR_ARB_LOST |
                                        I22OLED_GET_MSTAT_CMPLT);

        I22OLED_DISABLE_INT_ON_STOP; /* Interrupt on Stop is enabled by write */

        #if(I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED)
            if(I22OLED_CHECK_ADDRESS_STS(tmpCsr))
            {
                /* Slave was addressed */
                I22OLED_state = I22OLED_SM_SLAVE;
            }
            else
            {
                I22OLED_BUS_RELEASE;

                I22OLED_state = I22OLED_SM_EXIT_IDLE;
            }
        #else
            I22OLED_BUS_RELEASE;

            I22OLED_state = I22OLED_SM_EXIT_IDLE;

        #endif /* (I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED) */
    }
#endif /* (I22OLED_MODE_MULTI_MASTER_ENABLED) */

    /* Check for master operation mode */
    if(I22OLED_CHECK_SM_MASTER)
    {
    #if(I22OLED_MODE_MASTER_ENABLED)
        if(I22OLED_CHECK_BYTE_COMPLETE(tmpCsr))
        {
            switch (I22OLED_state)
            {
            case I22OLED_SM_MSTR_WR_ADDR:  /* After address is sent, write data */
            case I22OLED_SM_MSTR_RD_ADDR:  /* After address is sent, read data */

                tmpCsr &= ((uint8) ~I22OLED_CSR_STOP_STATUS); /* Clear Stop bit history on address phase */

                if(I22OLED_CHECK_ADDR_ACK(tmpCsr))
                {
                    /* Setup for transmit or receive of data */
                    if(I22OLED_state == I22OLED_SM_MSTR_WR_ADDR)   /* TRANSMIT data */
                    {
                        /* Check if at least one byte to transfer */
                        if(I22OLED_mstrWrBufSize > 0u)
                        {
                            /* Load the 1st data byte */
                            I22OLED_DATA_REG = I22OLED_mstrWrBufPtr[0u];
                            I22OLED_TRANSMIT_DATA;
                            I22OLED_mstrWrBufIndex = 1u;   /* Set index to 2nd element */

                            /* Set transmit state until done */
                            I22OLED_state = I22OLED_SM_MSTR_WR_DATA;
                        }
                        /* End of buffer: complete writing */
                        else if(I22OLED_CHECK_NO_STOP(I22OLED_mstrControl))
                        {
                            /* Set write complete and master halted */
                            I22OLED_mstrStatus |= (I22OLED_MSTAT_XFER_HALT |
                                                            I22OLED_MSTAT_WR_CMPLT);

                            I22OLED_state = I22OLED_SM_MSTR_HALT; /* Expect ReStart */
                            I22OLED_DisableInt();
                        }
                        else
                        {
                            I22OLED_ENABLE_INT_ON_STOP; /* Enable interrupt on Stop, to catch it */
                            I22OLED_GENERATE_STOP;
                        }
                    }
                    else  /* Master receive data */
                    {
                        I22OLED_READY_TO_READ; /* Release bus to read data */

                        I22OLED_state  = I22OLED_SM_MSTR_RD_DATA;
                    }
                }
                /* Address is NACKed */
                else if(I22OLED_CHECK_ADDR_NAK(tmpCsr))
                {
                    /* Set Address NAK error */
                    I22OLED_mstrStatus |= (I22OLED_MSTAT_ERR_XFER |
                                                    I22OLED_MSTAT_ERR_ADDR_NAK);

                    if(I22OLED_CHECK_NO_STOP(I22OLED_mstrControl))
                    {
                        I22OLED_mstrStatus |= (I22OLED_MSTAT_XFER_HALT |
                                                        I22OLED_GET_MSTAT_CMPLT);

                        I22OLED_state = I22OLED_SM_MSTR_HALT; /* Expect RESTART */
                        I22OLED_DisableInt();
                    }
                    else  /* Do normal Stop */
                    {
                        I22OLED_ENABLE_INT_ON_STOP; /* Enable interrupt on Stop, to catch it */
                        I22OLED_GENERATE_STOP;
                    }
                }
                else
                {
                    /* Address phase is not set for some reason: error */
                    #if(I22OLED_TIMEOUT_ENABLED)
                        /* Exit interrupt to take chance for timeout timer to handle this case */
                        I22OLED_DisableInt();
                        I22OLED_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (I22OLED_TIMEOUT_ENABLED) */
                }
                break;

            case I22OLED_SM_MSTR_WR_DATA:

                if(I22OLED_CHECK_DATA_ACK(tmpCsr))
                {
                    /* Check if end of buffer */
                    if(I22OLED_mstrWrBufIndex  < I22OLED_mstrWrBufSize)
                    {
                        I22OLED_DATA_REG =
                                                 I22OLED_mstrWrBufPtr[I22OLED_mstrWrBufIndex];
                        I22OLED_TRANSMIT_DATA;
                        I22OLED_mstrWrBufIndex++;
                    }
                    /* End of buffer: complete writing */
                    else if(I22OLED_CHECK_NO_STOP(I22OLED_mstrControl))
                    {
                        /* Set write complete and master halted */
                        I22OLED_mstrStatus |= (I22OLED_MSTAT_XFER_HALT |
                                                        I22OLED_MSTAT_WR_CMPLT);

                        I22OLED_state = I22OLED_SM_MSTR_HALT;    /* Expect restart */
                        I22OLED_DisableInt();
                    }
                    else  /* Do normal Stop */
                    {
                        I22OLED_ENABLE_INT_ON_STOP;    /* Enable interrupt on Stop, to catch it */
                        I22OLED_GENERATE_STOP;
                    }
                }
                /* Last byte NAKed: end writing */
                else if(I22OLED_CHECK_NO_STOP(I22OLED_mstrControl))
                {
                    /* Set write complete, short transfer and master halted */
                    I22OLED_mstrStatus |= (I22OLED_MSTAT_ERR_XFER       |
                                                    I22OLED_MSTAT_ERR_SHORT_XFER |
                                                    I22OLED_MSTAT_XFER_HALT      |
                                                    I22OLED_MSTAT_WR_CMPLT);

                    I22OLED_state = I22OLED_SM_MSTR_HALT;    /* Expect ReStart */
                    I22OLED_DisableInt();
                }
                else  /* Do normal Stop */
                {
                    I22OLED_ENABLE_INT_ON_STOP;    /* Enable interrupt on Stop, to catch it */
                    I22OLED_GENERATE_STOP;

                    /* Set short transfer and error flag */
                    I22OLED_mstrStatus |= (I22OLED_MSTAT_ERR_SHORT_XFER |
                                                    I22OLED_MSTAT_ERR_XFER);
                }

                break;

            case I22OLED_SM_MSTR_RD_DATA:

                I22OLED_mstrRdBufPtr[I22OLED_mstrRdBufIndex] = I22OLED_DATA_REG;
                I22OLED_mstrRdBufIndex++;

                /* Check if end of buffer */
                if(I22OLED_mstrRdBufIndex < I22OLED_mstrRdBufSize)
                {
                    I22OLED_ACK_AND_RECEIVE;       /* ACK and receive byte */
                }
                /* End of buffer: complete reading */
                else if(I22OLED_CHECK_NO_STOP(I22OLED_mstrControl))
                {
                    /* Set read complete and master halted */
                    I22OLED_mstrStatus |= (I22OLED_MSTAT_XFER_HALT |
                                                    I22OLED_MSTAT_RD_CMPLT);

                    I22OLED_state = I22OLED_SM_MSTR_HALT;    /* Expect ReStart */
                    I22OLED_DisableInt();
                }
                else
                {
                    I22OLED_ENABLE_INT_ON_STOP;
                    I22OLED_NAK_AND_RECEIVE;       /* NACK and TRY to generate Stop */
                }
                break;

            default: /* This is an invalid state and should not occur */

            #if(I22OLED_TIMEOUT_ENABLED)
                /* Exit interrupt to take chance for timeout timer to handles this case */
                I22OLED_DisableInt();
                I22OLED_ClearPendingInt();
            #else
                /* Block execution flow: unexpected condition */
                CYASSERT(0u != 0u);
            #endif /* (I22OLED_TIMEOUT_ENABLED) */

                break;
            }
        }

        /* Catches Stop: end of transaction */
        if(I22OLED_CHECK_STOP_STS(tmpCsr))
        {
            I22OLED_mstrStatus |= I22OLED_GET_MSTAT_CMPLT;

            I22OLED_DISABLE_INT_ON_STOP;
            I22OLED_state = I22OLED_SM_IDLE;
        }
    #endif /* (I22OLED_MODE_MASTER_ENABLED) */
    }
    else if(I22OLED_CHECK_SM_SLAVE)
    {
    #if(I22OLED_MODE_SLAVE_ENABLED)

        if((I22OLED_CHECK_STOP_STS(tmpCsr)) || /* Stop || Restart */
           (I22OLED_CHECK_BYTE_COMPLETE(tmpCsr) && I22OLED_CHECK_ADDRESS_STS(tmpCsr)))
        {
            /* Catch end of master write transaction: use interrupt on Stop */
            /* The Stop bit history on address phase does not have correct state */
            if(I22OLED_SM_SL_WR_DATA == I22OLED_state)
            {
                I22OLED_DISABLE_INT_ON_STOP;

                I22OLED_slStatus &= ((uint8) ~I22OLED_SSTAT_WR_BUSY);
                I22OLED_slStatus |= ((uint8)  I22OLED_SSTAT_WR_CMPLT);

                I22OLED_state = I22OLED_SM_IDLE;
            }
        }

        if(I22OLED_CHECK_BYTE_COMPLETE(tmpCsr))
        {
            /* The address only issued after Start or ReStart: so check the address
               to catch these events:
                FF : sets an address phase with a byte_complete interrupt trigger.
                UDB: sets an address phase immediately after Start or ReStart. */
            if(I22OLED_CHECK_ADDRESS_STS(tmpCsr))
            {
            /* Check for software address detection */
            #if(I22OLED_SW_ADRR_DECODE)
                tmp8 = I22OLED_GET_SLAVE_ADDR(I22OLED_DATA_REG);

                if(tmp8 == I22OLED_slAddress)   /* Check for address match */
                {
                    if(0u != (I22OLED_DATA_REG & I22OLED_READ_FLAG))
                    {
                        /* Place code to prepare read buffer here                  */
                        /* `#START I22OLED_SW_PREPARE_READ_BUF_interrupt` */

                        /* `#END` */

                    #ifdef I22OLED_SW_PREPARE_READ_BUF_CALLBACK
                        I22OLED_SwPrepareReadBuf_Callback();
                    #endif /* I22OLED_SW_PREPARE_READ_BUF_CALLBACK */
                        
                        /* Prepare next operation to read, get data and place in data register */
                        if(I22OLED_slRdBufIndex < I22OLED_slRdBufSize)
                        {
                            /* Load first data byte from array */
                            I22OLED_DATA_REG = I22OLED_slRdBufPtr[I22OLED_slRdBufIndex];
                            I22OLED_ACK_AND_TRANSMIT;
                            I22OLED_slRdBufIndex++;

                            I22OLED_slStatus |= I22OLED_SSTAT_RD_BUSY;
                        }
                        else    /* Overflow: provide 0xFF on bus */
                        {
                            I22OLED_DATA_REG = I22OLED_OVERFLOW_RETURN;
                            I22OLED_ACK_AND_TRANSMIT;

                            I22OLED_slStatus  |= (I22OLED_SSTAT_RD_BUSY |
                                                           I22OLED_SSTAT_RD_ERR_OVFL);
                        }

                        I22OLED_state = I22OLED_SM_SL_RD_DATA;
                    }
                    else  /* Write transaction: receive 1st byte */
                    {
                        I22OLED_ACK_AND_RECEIVE;
                        I22OLED_state = I22OLED_SM_SL_WR_DATA;

                        I22OLED_slStatus |= I22OLED_SSTAT_WR_BUSY;
                        I22OLED_ENABLE_INT_ON_STOP;
                    }
                }
                else
                {
                    /*     Place code to compare for additional address here    */
                    /* `#START I22OLED_SW_ADDR_COMPARE_interruptStart` */

                    /* `#END` */

                #ifdef I22OLED_SW_ADDR_COMPARE_ENTRY_CALLBACK
                    I22OLED_SwAddrCompare_EntryCallback();
                #endif /* I22OLED_SW_ADDR_COMPARE_ENTRY_CALLBACK */
                    
                    I22OLED_NAK_AND_RECEIVE;   /* NACK address */

                    /* Place code to end of condition for NACK generation here */
                    /* `#START I22OLED_SW_ADDR_COMPARE_interruptEnd`  */

                    /* `#END` */

                #ifdef I22OLED_SW_ADDR_COMPARE_EXIT_CALLBACK
                    I22OLED_SwAddrCompare_ExitCallback();
                #endif /* I22OLED_SW_ADDR_COMPARE_EXIT_CALLBACK */
                }

            #else /* (I22OLED_HW_ADRR_DECODE) */

                if(0u != (I22OLED_DATA_REG & I22OLED_READ_FLAG))
                {
                    /* Place code to prepare read buffer here                  */
                    /* `#START I22OLED_HW_PREPARE_READ_BUF_interrupt` */

                    /* `#END` */
                    
                #ifdef I22OLED_HW_PREPARE_READ_BUF_CALLBACK
                    I22OLED_HwPrepareReadBuf_Callback();
                #endif /* I22OLED_HW_PREPARE_READ_BUF_CALLBACK */

                    /* Prepare next operation to read, get data and place in data register */
                    if(I22OLED_slRdBufIndex < I22OLED_slRdBufSize)
                    {
                        /* Load first data byte from array */
                        I22OLED_DATA_REG = I22OLED_slRdBufPtr[I22OLED_slRdBufIndex];
                        I22OLED_ACK_AND_TRANSMIT;
                        I22OLED_slRdBufIndex++;

                        I22OLED_slStatus |= I22OLED_SSTAT_RD_BUSY;
                    }
                    else    /* Overflow: provide 0xFF on bus */
                    {
                        I22OLED_DATA_REG = I22OLED_OVERFLOW_RETURN;
                        I22OLED_ACK_AND_TRANSMIT;

                        I22OLED_slStatus  |= (I22OLED_SSTAT_RD_BUSY |
                                                       I22OLED_SSTAT_RD_ERR_OVFL);
                    }

                    I22OLED_state = I22OLED_SM_SL_RD_DATA;
                }
                else  /* Write transaction: receive 1st byte */
                {
                    I22OLED_ACK_AND_RECEIVE;
                    I22OLED_state = I22OLED_SM_SL_WR_DATA;

                    I22OLED_slStatus |= I22OLED_SSTAT_WR_BUSY;
                    I22OLED_ENABLE_INT_ON_STOP;
                }

            #endif /* (I22OLED_SW_ADRR_DECODE) */
            }
            /* Data states */
            /* Data master writes into slave */
            else if(I22OLED_state == I22OLED_SM_SL_WR_DATA)
            {
                if(I22OLED_slWrBufIndex < I22OLED_slWrBufSize)
                {
                    tmp8 = I22OLED_DATA_REG;
                    I22OLED_ACK_AND_RECEIVE;
                    I22OLED_slWrBufPtr[I22OLED_slWrBufIndex] = tmp8;
                    I22OLED_slWrBufIndex++;
                }
                else  /* of array: complete write, send NACK */
                {
                    I22OLED_NAK_AND_RECEIVE;

                    I22OLED_slStatus |= I22OLED_SSTAT_WR_ERR_OVFL;
                }
            }
            /* Data master reads from slave */
            else if(I22OLED_state == I22OLED_SM_SL_RD_DATA)
            {
                if(I22OLED_CHECK_DATA_ACK(tmpCsr))
                {
                    if(I22OLED_slRdBufIndex < I22OLED_slRdBufSize)
                    {
                         /* Get data from array */
                        I22OLED_DATA_REG = I22OLED_slRdBufPtr[I22OLED_slRdBufIndex];
                        I22OLED_TRANSMIT_DATA;
                        I22OLED_slRdBufIndex++;
                    }
                    else   /* Overflow: provide 0xFF on bus */
                    {
                        I22OLED_DATA_REG = I22OLED_OVERFLOW_RETURN;
                        I22OLED_TRANSMIT_DATA;

                        I22OLED_slStatus |= I22OLED_SSTAT_RD_ERR_OVFL;
                    }
                }
                else  /* Last byte was NACKed: read complete */
                {
                    /* Only NACK appears on bus */
                    I22OLED_DATA_REG = I22OLED_OVERFLOW_RETURN;
                    I22OLED_NAK_AND_TRANSMIT;

                    I22OLED_slStatus &= ((uint8) ~I22OLED_SSTAT_RD_BUSY);
                    I22OLED_slStatus |= ((uint8)  I22OLED_SSTAT_RD_CMPLT);

                    I22OLED_state = I22OLED_SM_IDLE;
                }
            }
            else
            {
            #if(I22OLED_TIMEOUT_ENABLED)
                /* Exit interrupt to take chance for timeout timer to handle this case */
                I22OLED_DisableInt();
                I22OLED_ClearPendingInt();
            #else
                /* Block execution flow: unexpected condition */
                CYASSERT(0u != 0u);
            #endif /* (I22OLED_TIMEOUT_ENABLED) */
            }
        }
    #endif /* (I22OLED_MODE_SLAVE_ENABLED) */
    }
    else
    {
        /* The FSM skips master and slave processing: return to IDLE */
        I22OLED_state = I22OLED_SM_IDLE;
    }

#ifdef I22OLED_ISR_EXIT_CALLBACK
    I22OLED_ISR_ExitCallback();
#endif /* I22OLED_ISR_EXIT_CALLBACK */    
}


#if ((I22OLED_FF_IMPLEMENTED) && (I22OLED_WAKEUP_ENABLED))
    /*******************************************************************************
    * Function Name: I22OLED_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  The interrupt handler to trigger after a wakeup.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    CY_ISR(I22OLED_WAKEUP_ISR)
    {
    #ifdef I22OLED_WAKEUP_ISR_ENTRY_CALLBACK
        I22OLED_WAKEUP_ISR_EntryCallback();
    #endif /* I22OLED_WAKEUP_ISR_ENTRY_CALLBACK */
         
        /* Set flag to notify that matched address is received */
        I22OLED_wakeupSource = 1u;

        /* SCL is stretched until the I2C_Wake() is called */

    #ifdef I22OLED_WAKEUP_ISR_EXIT_CALLBACK
        I22OLED_WAKEUP_ISR_ExitCallback();
    #endif /* I22OLED_WAKEUP_ISR_EXIT_CALLBACK */
    }
#endif /* ((I22OLED_FF_IMPLEMENTED) && (I22OLED_WAKEUP_ENABLED)) */


/* [] END OF FILE */
