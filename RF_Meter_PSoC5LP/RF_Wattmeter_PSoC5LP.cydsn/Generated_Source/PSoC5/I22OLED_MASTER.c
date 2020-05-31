/*******************************************************************************
* File Name: I22OLED_MASTER.c
* Version 3.50
*
* Description:
*  This file provides the source code of APIs for the I2C component master mode.
*
*******************************************************************************
* Copyright 2012-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I22OLED_PVT.h"

#if(I22OLED_MODE_MASTER_ENABLED)

/**********************************
*      System variables
**********************************/

volatile uint8 I22OLED_mstrStatus;     /* Master Status byte  */
volatile uint8 I22OLED_mstrControl;    /* Master Control byte */

/* Transmit buffer variables */
volatile uint8 * I22OLED_mstrRdBufPtr;     /* Pointer to Master Read buffer */
volatile uint8   I22OLED_mstrRdBufSize;    /* Master Read buffer size       */
volatile uint8   I22OLED_mstrRdBufIndex;   /* Master Read buffer Index      */

/* Receive buffer variables */
volatile uint8 * I22OLED_mstrWrBufPtr;     /* Pointer to Master Write buffer */
volatile uint8   I22OLED_mstrWrBufSize;    /* Master Write buffer size       */
volatile uint8   I22OLED_mstrWrBufIndex;   /* Master Write buffer Index      */


/*******************************************************************************
* Function Name: I22OLED_MasterWriteBuf
********************************************************************************
*
* Summary:
*  Automatically writes an entire buffer of data to a slave device. Once the
*  data transfer is initiated by this function, further data transfer is handled
*  by the included ISR in byte by byte mode.
*
* Parameters:
*  slaveAddr: 7-bit slave address.
*  xferData:  Pointer to buffer of data to be sent.
*  cnt:       Size of buffer to send.
*  mode:      Transfer mode defines: start or restart condition generation at
*             begin of the transfer and complete the transfer or halt before
*             generating a stop.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  The included ISR will start a transfer after a start or restart condition is
*  generated.
*
* Global variables:
*  I22OLED_mstrStatus  - The global variable used to store a current
*                                 status of the I2C Master.
*  I22OLED_state       - The global variable used to store a current
*                                 state of the software FSM.
*  I22OLED_mstrControl - The global variable used to control the master
*                                 end of a transaction with or without Stop
*                                 generation.
*  I22OLED_mstrWrBufPtr - The global variable used to store a pointer
*                                  to the master write buffer.
*  I22OLED_mstrWrBufIndex - The global variable used to store current
*                                    index within the master write buffer.
*  I22OLED_mstrWrBufSize - The global variable used to store a master
*                                   write buffer size.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 I22OLED_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode)
      
{
    uint8 errStatus;

    errStatus = I22OLED_MSTR_NOT_READY;

    if(NULL != wrData)
    {
        /* Check I2C state to allow transfer: valid states are IDLE or HALT */
        if(I22OLED_SM_IDLE == I22OLED_state)
        {
            /* Master is ready for transaction: check if bus is free */
            if(I22OLED_CHECK_BUS_FREE(I22OLED_MCSR_REG))
            {
                errStatus = I22OLED_MSTR_NO_ERROR;
            }
            else
            {
                errStatus = I22OLED_MSTR_BUS_BUSY;
            }
        }
        else if(I22OLED_SM_MSTR_HALT == I22OLED_state)
        {
            /* Master is ready and waiting for ReStart */
            errStatus = I22OLED_MSTR_NO_ERROR;

            I22OLED_ClearPendingInt();
            I22OLED_mstrStatus &= (uint8) ~I22OLED_MSTAT_XFER_HALT;
        }
        else
        {
            /* errStatus = I22OLED_MSTR_NOT_READY was send before */
        }

        if(I22OLED_MSTR_NO_ERROR == errStatus)
        {
            /* Set state to start write transaction */
            I22OLED_state = I22OLED_SM_MSTR_WR_ADDR;

            /* Prepare write buffer */
            I22OLED_mstrWrBufIndex = 0u;
            I22OLED_mstrWrBufSize  = cnt;
            I22OLED_mstrWrBufPtr   = (volatile uint8 *) wrData;

            /* Set end of transaction flag: Stop or Halt (following ReStart) */
            I22OLED_mstrControl = mode;

            /* Clear write status history */
            I22OLED_mstrStatus &= (uint8) ~I22OLED_MSTAT_WR_CMPLT;

            /* Hardware actions: write address and generate Start or ReStart */
            I22OLED_DATA_REG = (uint8) (slaveAddress << I22OLED_SLAVE_ADDR_SHIFT);

            if(I22OLED_CHECK_RESTART(mode))
            {
                I22OLED_GENERATE_RESTART;
            }
            else
            {
                I22OLED_GENERATE_START;
            }

            /* Enable interrupt to complete transfer */
            I22OLED_EnableInt();
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: I22OLED_MasterReadBuf
********************************************************************************
*
* Summary:
*  Automatically writes an entire buffer of data to a slave device. Once the
*  data transfer is initiated by this function, further data transfer is handled
*  by the included ISR in byte by byte mode.
*
* Parameters:
*  slaveAddr: 7-bit slave address.
*  xferData:  Pointer to buffer where to put data from slave.
*  cnt:       Size of buffer to read.
*  mode:      Transfer mode defines: start or restart condition generation at
*             begin of the transfer and complete the transfer or halt before
*             generating a stop.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  The included ISR will start a transfer after start or restart condition is
*  generated.
*
* Global variables:
*  I22OLED_mstrStatus  - The global variable used to store a current
*                                 status of the I2C Master.
*  I22OLED_state       - The global variable used to store a current
*                                 state of the software FSM.
*  I22OLED_mstrControl - The global variable used to control the master
*                                 end of a transaction with or without
*                                 Stop generation.
*  I22OLED_mstrRdBufPtr - The global variable used to store a pointer
*                                  to the master write buffer.
*  I22OLED_mstrRdBufIndex - The global variable  used to store a
*                                    current index within the master
*                                    write buffer.
*  I22OLED_mstrRdBufSize - The global variable used to store a master
*                                   write buffer size.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I22OLED_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode)
      
{
    uint8 errStatus;

    errStatus = I22OLED_MSTR_NOT_READY;

    if(NULL != rdData)
    {
        /* Check I2C state to allow transfer: valid states are IDLE or HALT */
        if(I22OLED_SM_IDLE == I22OLED_state)
        {
            /* Master is ready to transaction: check if bus is free */
            if(I22OLED_CHECK_BUS_FREE(I22OLED_MCSR_REG))
            {
                errStatus = I22OLED_MSTR_NO_ERROR;
            }
            else
            {
                errStatus = I22OLED_MSTR_BUS_BUSY;
            }
        }
        else if(I22OLED_SM_MSTR_HALT == I22OLED_state)
        {
            /* Master is ready and waiting for ReStart */
            errStatus = I22OLED_MSTR_NO_ERROR;

            I22OLED_ClearPendingInt();
            I22OLED_mstrStatus &= (uint8) ~I22OLED_MSTAT_XFER_HALT;
        }
        else
        {
            /* errStatus = I22OLED_MSTR_NOT_READY was set before */
        }

        if(I22OLED_MSTR_NO_ERROR == errStatus)
        {
            /* Set state to start write transaction */
            I22OLED_state = I22OLED_SM_MSTR_RD_ADDR;

            /* Prepare read buffer */
            I22OLED_mstrRdBufIndex  = 0u;
            I22OLED_mstrRdBufSize   = cnt;
            I22OLED_mstrRdBufPtr    = (volatile uint8 *) rdData;

            /* Set end of transaction flag: Stop or Halt (following ReStart) */
            I22OLED_mstrControl = mode;

            /* Clear read status history */
            I22OLED_mstrStatus &= (uint8) ~I22OLED_MSTAT_RD_CMPLT;

            /* Hardware actions: write address and generate Start or ReStart */
            I22OLED_DATA_REG = ((uint8) (slaveAddress << I22OLED_SLAVE_ADDR_SHIFT) |
                                                  I22OLED_READ_FLAG);

            if(I22OLED_CHECK_RESTART(mode))
            {
                I22OLED_GENERATE_RESTART;
            }
            else
            {
                I22OLED_GENERATE_START;
            }

            /* Enable interrupt to complete transfer */
            I22OLED_EnableInt();
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: I22OLED_MasterSendStart
********************************************************************************
*
* Summary:
*  Generates Start condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I22OLED_state - The global variable used to store a current state of
*                           the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I22OLED_MasterSendStart(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = I22OLED_MSTR_NOT_READY;

    /* If IDLE, check if bus is free */
    if(I22OLED_SM_IDLE == I22OLED_state)
    {
        /* If bus is free, generate Start condition */
        if(I22OLED_CHECK_BUS_FREE(I22OLED_MCSR_REG))
        {
            /* Disable interrupt for manual master operation */
            I22OLED_DisableInt();

            /* Set address and read/write flag */
            slaveAddress = (uint8) (slaveAddress << I22OLED_SLAVE_ADDR_SHIFT);
            if(0u != R_nW)
            {
                slaveAddress |= I22OLED_READ_FLAG;
                I22OLED_state = I22OLED_SM_MSTR_RD_ADDR;
            }
            else
            {
                I22OLED_state = I22OLED_SM_MSTR_WR_ADDR;
            }

            /* Hardware actions: write address and generate Start */
            I22OLED_DATA_REG = slaveAddress;
            I22OLED_GENERATE_START_MANUAL;

            /* Wait until address is transferred */
            while(I22OLED_WAIT_BYTE_COMPLETE(I22OLED_CSR_REG))
            {
            }

        #if(I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED)
            if(I22OLED_CHECK_START_GEN(I22OLED_MCSR_REG))
            {
                I22OLED_CLEAR_START_GEN;

                /* Start condition was not generated: reset FSM to IDLE */
                I22OLED_state = I22OLED_SM_IDLE;
                errStatus = I22OLED_MSTR_ERR_ABORT_START_GEN;
            }
            else
        #endif /* (I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED) */

        #if(I22OLED_MODE_MULTI_MASTER_ENABLED)
            if(I22OLED_CHECK_LOST_ARB(I22OLED_CSR_REG))
            {
                I22OLED_BUS_RELEASE_MANUAL;

                /* Master lost arbitrage: reset FSM to IDLE */
                I22OLED_state = I22OLED_SM_IDLE;
                errStatus = I22OLED_MSTR_ERR_ARB_LOST;
            }
            else
        #endif /* (I22OLED_MODE_MULTI_MASTER_ENABLED) */

            if(I22OLED_CHECK_ADDR_NAK(I22OLED_CSR_REG))
            {
                /* Address has been NACKed: reset FSM to IDLE */
                I22OLED_state = I22OLED_SM_IDLE;
                errStatus = I22OLED_MSTR_ERR_LB_NAK;
            }
            else
            {
                /* Start was sent without errors */
                errStatus = I22OLED_MSTR_NO_ERROR;
            }
        }
        else
        {
            errStatus = I22OLED_MSTR_BUS_BUSY;
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: I22OLED_MasterSendRestart
********************************************************************************
*
* Summary:
*  Generates ReStart condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I22OLED_state - The global variable used to store a current state of
*                           the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I22OLED_MasterSendRestart(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = I22OLED_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(I22OLED_CHECK_MASTER_MODE(I22OLED_MCSR_REG))
    {
        /* Set address and read/write flag */
        slaveAddress = (uint8) (slaveAddress << I22OLED_SLAVE_ADDR_SHIFT);
        if(0u != R_nW)
        {
            slaveAddress |= I22OLED_READ_FLAG;
            I22OLED_state = I22OLED_SM_MSTR_RD_ADDR;
        }
        else
        {
            I22OLED_state = I22OLED_SM_MSTR_WR_ADDR;
        }

        /* Hardware actions: write address and generate ReStart */
        I22OLED_DATA_REG = slaveAddress;
        I22OLED_GENERATE_RESTART_MANUAL;

        /* Wait until address has been transferred */
        while(I22OLED_WAIT_BYTE_COMPLETE(I22OLED_CSR_REG))
        {
        }

    #if(I22OLED_MODE_MULTI_MASTER_ENABLED)
        if(I22OLED_CHECK_LOST_ARB(I22OLED_CSR_REG))
        {
            I22OLED_BUS_RELEASE_MANUAL;

            /* Master lost arbitrage: reset FSM to IDLE */
            I22OLED_state = I22OLED_SM_IDLE;
            errStatus = I22OLED_MSTR_ERR_ARB_LOST;
        }
        else
    #endif /* (I22OLED_MODE_MULTI_MASTER_ENABLED) */

        if(I22OLED_CHECK_ADDR_NAK(I22OLED_CSR_REG))
        {
            /* Address has been NACKed: reset FSM to IDLE */
            I22OLED_state = I22OLED_SM_IDLE;
            errStatus = I22OLED_MSTR_ERR_LB_NAK;
        }
        else
        {
            /* ReStart was sent without errors */
            errStatus = I22OLED_MSTR_NO_ERROR;
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: I22OLED_MasterSendStop
********************************************************************************
*
* Summary:
*  Generates I2C Stop condition on bus. Function do nothing if Start or Restart
*  condition was failed before call this function.
*
* Parameters:
*  None.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  Stop generation is required to complete the transaction.
*  This function does not wait until a Stop condition is generated.
*
* Global variables:
*  I22OLED_state - The global variable used to store a current state of
*                           the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I22OLED_MasterSendStop(void) 
{
    uint8 errStatus;

    errStatus = I22OLED_MSTR_NOT_READY;

    /* Check if master is active on bus */
    if(I22OLED_CHECK_MASTER_MODE(I22OLED_MCSR_REG))
    {
        I22OLED_GENERATE_STOP_MANUAL;
        I22OLED_state = I22OLED_SM_IDLE;

        /* Wait until stop has been generated */
        while(I22OLED_WAIT_STOP_COMPLETE(I22OLED_CSR_REG))
        {
        }

        errStatus = I22OLED_MSTR_NO_ERROR;

    #if(I22OLED_MODE_MULTI_MASTER_ENABLED)
        if(I22OLED_CHECK_LOST_ARB(I22OLED_CSR_REG))
        {
            I22OLED_BUS_RELEASE_MANUAL;

            /* NACK was generated by instead Stop */
            errStatus = I22OLED_MSTR_ERR_ARB_LOST;
        }
    #endif /* (I22OLED_MODE_MULTI_MASTER_ENABLED) */
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: I22OLED_MasterWriteByte
********************************************************************************
*
* Summary:
*  Sends one byte to a slave. A valid Start or ReStart condition must be
*  generated before this call this function. Function do nothing if Start or
*  Restart condition was failed before call this function.
*
* Parameters:
*  data:  The data byte to send to the slave.
*
* Return:
*  Status error - Zero means no errors.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I22OLED_state - The global variable used to store a current state of
*                           the software FSM.
*
*******************************************************************************/
uint8 I22OLED_MasterWriteByte(uint8 theByte) 
{
    uint8 errStatus;

    errStatus = I22OLED_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(I22OLED_CHECK_MASTER_MODE(I22OLED_MCSR_REG))
    {
        I22OLED_DATA_REG = theByte;   /* Write DATA register */
        I22OLED_TRANSMIT_DATA_MANUAL; /* Set transmit mode   */
        I22OLED_state = I22OLED_SM_MSTR_WR_DATA;

        /* Wait until data byte has been transmitted */
        while(I22OLED_WAIT_BYTE_COMPLETE(I22OLED_CSR_REG))
        {
        }

    #if(I22OLED_MODE_MULTI_MASTER_ENABLED)
        if(I22OLED_CHECK_LOST_ARB(I22OLED_CSR_REG))
        {
            I22OLED_BUS_RELEASE_MANUAL;

            /* Master lost arbitrage: reset FSM to IDLE */
            I22OLED_state = I22OLED_SM_IDLE;
            errStatus = I22OLED_MSTR_ERR_ARB_LOST;
        }
        /* Check LRB bit */
        else
    #endif /* (I22OLED_MODE_MULTI_MASTER_ENABLED) */

        if(I22OLED_CHECK_DATA_ACK(I22OLED_CSR_REG))
        {
            I22OLED_state = I22OLED_SM_MSTR_HALT;
            errStatus = I22OLED_MSTR_NO_ERROR;
        }
        else
        {
            I22OLED_state = I22OLED_SM_MSTR_HALT;
            errStatus = I22OLED_MSTR_ERR_LB_NAK;
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: I22OLED_MasterReadByte
********************************************************************************
*
* Summary:
*  Reads one byte from a slave and ACK or NACK the transfer. A valid Start or
*  ReStart condition must be generated before this call this function. Function
*  do nothing if Start or Restart condition was failed before call this
*  function.
*
* Parameters:
*  acknNack:  Zero, response with NACK, if non-zero response with ACK.
*
* Return:
*  Byte read from slave.
*
* Side Effects:
*  This function is entered without a "byte complete" bit set in the I2C_CSR
*  register. It does not exit until it is set.
*
* Global variables:
*  I22OLED_state - The global variable used to store a current
*                           state of the software FSM.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I22OLED_MasterReadByte(uint8 acknNak) 
{
    uint8 theByte;

    theByte = 0u;

    /* Check if START condition was generated */
    if(I22OLED_CHECK_MASTER_MODE(I22OLED_MCSR_REG))
    {
        /* When address phase needs to release bus and receive byte,
        * then decide ACK or NACK
        */
        if(I22OLED_SM_MSTR_RD_ADDR == I22OLED_state)
        {
            I22OLED_READY_TO_READ_MANUAL;
            I22OLED_state = I22OLED_SM_MSTR_RD_DATA;
        }

        /* Wait until data byte has been received */
        while(I22OLED_WAIT_BYTE_COMPLETE(I22OLED_CSR_REG))
        {
        }

        theByte = I22OLED_DATA_REG;

        /* Command ACK to receive next byte and continue transfer.
        *  Do nothing for NACK. The NACK will be generated by
        *  Stop or ReStart routine.
        */
        if(acknNak != 0u) /* Generate ACK */
        {
            I22OLED_ACK_AND_RECEIVE_MANUAL;
        }
        else              /* Do nothing for the follwong NACK */
        {
            I22OLED_state = I22OLED_SM_MSTR_HALT;
        }
    }

    return(theByte);
}


/*******************************************************************************
* Function Name: I22OLED_MasterStatus
********************************************************************************
*
* Summary:
*  Returns the master's communication status.
*
* Parameters:
*  None.
*
* Return:
*  Current status of I2C master.
*
* Global variables:
*  I22OLED_mstrStatus - The global variable used to store a current
*                                status of the I2C Master.
*
*******************************************************************************/
uint8 I22OLED_MasterStatus(void) 
{
    uint8 status;

    I22OLED_DisableInt(); /* Lock from interrupt */

    /* Read master status */
    status = I22OLED_mstrStatus;

    if (I22OLED_CHECK_SM_MASTER)
    {
        /* Set transfer in progress flag in status */
        status |= I22OLED_MSTAT_XFER_INP;
    }

    I22OLED_EnableInt(); /* Release lock */

    return (status);
}


/*******************************************************************************
* Function Name: I22OLED_MasterClearStatus
********************************************************************************
*
* Summary:
*  Clears all status flags and returns the master status.
*
* Parameters:
*  None.
*
* Return:
*  Current status of I2C master.
*
* Global variables:
*  I22OLED_mstrStatus - The global variable used to store a current
*                                status of the I2C Master.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I22OLED_MasterClearStatus(void) 
{
    uint8 status;

    I22OLED_DisableInt(); /* Lock from interrupt */

    /* Read and clear master status */
    status = I22OLED_mstrStatus;
    I22OLED_mstrStatus = I22OLED_MSTAT_CLEAR;

    I22OLED_EnableInt(); /* Release lock */

    return (status);
}


/*******************************************************************************
* Function Name: I22OLED_MasterGetReadBufSize
********************************************************************************
*
* Summary:
*  Returns the amount of bytes that has been transferred with an
*  I2C_MasterReadBuf command.
*
* Parameters:
*  None.
*
* Return:
*  Byte count of transfer. If the transfer is not yet complete, it will return
*  the byte count transferred so far.
*
* Global variables:
*  I22OLED_mstrRdBufIndex - The global variable stores current index
*                                    within the master read buffer.
*
*******************************************************************************/
uint8 I22OLED_MasterGetReadBufSize(void) 
{
    return (I22OLED_mstrRdBufIndex);
}


/*******************************************************************************
* Function Name: I22OLED_MasterGetWriteBufSize
********************************************************************************
*
* Summary:
*  Returns the amount of bytes that has been transferred with an
*  I2C_MasterWriteBuf command.
*
* Parameters:
*  None.
*
* Return:
*  Byte count of transfer. If the transfer is not yet complete, it will return
*  the byte count transferred so far.
*
* Global variables:
*  I22OLED_mstrWrBufIndex -  The global variable used to stores current
*                                     index within master write buffer.
*
*******************************************************************************/
uint8 I22OLED_MasterGetWriteBufSize(void) 
{
    return (I22OLED_mstrWrBufIndex);
}


/*******************************************************************************
* Function Name: I22OLED_MasterClearReadBuf
********************************************************************************
*
* Summary:
*  Resets the read buffer pointer back to the first byte in the buffer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  I22OLED_mstrRdBufIndex - The global variable used to stores current
*                                    index within master read buffer.
*  I22OLED_mstrStatus - The global variable used to store a current
*                                status of the I2C Master.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I22OLED_MasterClearReadBuf(void) 
{
    I22OLED_DisableInt(); /* Lock from interrupt */

    I22OLED_mstrRdBufIndex = 0u;
    I22OLED_mstrStatus    &= (uint8) ~I22OLED_MSTAT_RD_CMPLT;

    I22OLED_EnableInt(); /* Release lock */
}


/*******************************************************************************
* Function Name: I22OLED_MasterClearWriteBuf
********************************************************************************
*
* Summary:
*  Resets the write buffer pointer back to the first byte in the buffer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  I22OLED_mstrRdBufIndex - The global variable used to stote current
*                                    index within master read buffer.
*  I22OLED_mstrStatus - The global variable used to store a current
*                                status of the I2C Master.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I22OLED_MasterClearWriteBuf(void) 
{
    I22OLED_DisableInt(); /* Lock from interrupt */

    I22OLED_mstrWrBufIndex = 0u;
    I22OLED_mstrStatus    &= (uint8) ~I22OLED_MSTAT_WR_CMPLT;

    I22OLED_EnableInt(); /* Release lock */
}

#endif /* (I22OLED_MODE_MASTER_ENABLED) */


/* [] END OF FILE */
