/*******************************************************************************
* File Name: I2COLED_SLAVE.c
* Version 3.50
*
* Description:
*  This file provides the source code of APIs for the I2C component Slave mode.
*
* Note:
*
*******************************************************************************
* Copyright 2012-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I2COLED_PVT.h"

#if (I2COLED_MODE_SLAVE_ENABLED)

/**********************************
*      System variables
**********************************/

volatile uint8 I2COLED_slStatus;   /* Slave Status  */

/* Transmit buffer variables */
volatile uint8 * I2COLED_slRdBufPtr;   /* Pointer to Transmit buffer  */
volatile uint8   I2COLED_slRdBufSize;  /* Slave Transmit buffer size  */
volatile uint8   I2COLED_slRdBufIndex; /* Slave Transmit buffer Index */

/* Receive buffer variables */
volatile uint8 * I2COLED_slWrBufPtr;   /* Pointer to Receive buffer  */
volatile uint8   I2COLED_slWrBufSize;  /* Slave Receive buffer size  */
volatile uint8   I2COLED_slWrBufIndex; /* Slave Receive buffer Index */

#if (I2COLED_SW_ADRR_DECODE)
    volatile uint8 I2COLED_slAddress;  /* Software address variable */
#endif /* (I2COLED_SW_ADRR_DECODE) */


/*******************************************************************************
* Function Name: I2COLED_SlaveStatus
********************************************************************************
*
* Summary:
*  Returns I2C slave's communication status.
*
* Parameters:
*  None.
*
* Return:
*  Current status of I2C slave.
*
* Global variables:
*  I2COLED_slStatus - The global variable used to store a current
*                              status of the I2C slave.
*
*******************************************************************************/
uint8 I2COLED_SlaveStatus(void) 
{
    return (I2COLED_slStatus);
}


/*******************************************************************************
* Function Name: I2COLED_SlaveClearReadStatus
********************************************************************************
*
* Summary:
*  Clears the read status flags and returns they values.
*  The I2COLED_SSTAT_RD_BUSY flag is not effected by clear.
*
* Parameters:
*  None.
*
* Return:
*  Current read status of I2C slave.
*
* Global variables:
*  I2COLED_slStatus - The global variable used to store a current
*                              status of the I2C slave.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I2COLED_SlaveClearReadStatus(void) 
{
    uint8 status;

    I2COLED_DisableInt(); /* Lock from interrupt */

    /* Mask of transfer complete flag and Error status */
    status = (I2COLED_slStatus & I2COLED_SSTAT_RD_MASK);
    I2COLED_slStatus &= (uint8) ~I2COLED_SSTAT_RD_CLEAR;

    I2COLED_EnableInt(); /* Release lock */

    return (status);
}


/*******************************************************************************
* Function Name: I2COLED_SlaveClearWriteStatus
********************************************************************************
*
* Summary:
*  Clears the write status flags and returns they values.
*  The I2COLED_SSTAT_WR_BUSY flag is not effected by clear.
*
* Parameters:
*  None.
*
* Return:
*  Current write status of I2C slave.
*
* Global variables:
*  I2COLED_slStatus - The global variable used to store a current
*                              status of the I2C slave.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 I2COLED_SlaveClearWriteStatus(void) 
{
    uint8 status;

    I2COLED_DisableInt(); /* Lock from interrupt */

    /* Mask of transfer complete flag and Error status */
    status = (I2COLED_slStatus & I2COLED_SSTAT_WR_MASK);
    I2COLED_slStatus &= (uint8) ~I2COLED_SSTAT_WR_CLEAR;

    I2COLED_EnableInt(); /* Release lock */

    return (status);
}


/*******************************************************************************
* Function Name: I2COLED_SlaveSetAddress
********************************************************************************
*
* Summary:
*  Sets the I2C slave address.
*
* Parameters:
*  address: I2C slave address for the primary device. This value may be any
*  address between 0 and 127.
*
* Return:
*  None.
*
* Global variables:
*  I2COLED_Address  - The global variable used to store an I2C slave
*                              address for the primary device when the software
*                              address detect feature is used.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I2COLED_SlaveSetAddress(uint8 address) 
{
#if (I2COLED_SW_ADRR_DECODE)
    I2COLED_slAddress = (address & I2COLED_SLAVE_ADDR_MASK);
#else
    I2COLED_ADDR_REG  = (address & I2COLED_SLAVE_ADDR_MASK);
#endif /* (I2COLED_SW_ADRR_DECODE) */
}


/*******************************************************************************
* Function Name: I2COLED_SlaveInitReadBuf
********************************************************************************
*
* Summary:
*  Sets the buffer pointer and size of the read buffer. This function also
*  resets the transfer count returned with the I2C_SlaveGetReadBufSize function.
*
* Parameters:
*  readBuf:  Pointer to the data buffer to be read by the master.
*  bufSize:  Size of the read buffer exposed to the I2C master.
*
* Return:
*  None.
*
* Global variables:
*  I2COLED_slRdBufPtr   - The global variable used to store a pointer
*                                  to the slave read buffer.
*  I2COLED_slRdBufSize  - The global variable used to store a slave
*                                  read buffer size.
*  I2COLED_slRdBufIndex - The global variable used to store a current
*                                  index within the slave read buffer.
*
* Side Effects:
*  If this function is called during a bus transaction, data from the previous
*  buffer location and the beginning of the current buffer may be transmitted.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I2COLED_SlaveInitReadBuf(uint8 * rdBuf, uint8 bufSize)
     
{
    if (NULL != rdBuf)
    {
        I2COLED_DisableInt(); /* Lock from interrupt */

        /* Set buffer pointer */
        I2COLED_slRdBufPtr   = (volatile uint8 *) rdBuf;
        I2COLED_slRdBufSize  = bufSize;    /* Set buffer size     */
        I2COLED_slRdBufIndex = 0u;         /* Clears buffer index */

        I2COLED_EnableInt(); /* Release lock */
    }
}


/*******************************************************************************
* Function Name: I2COLED_SlaveInitWriteBuf
********************************************************************************
*
* Summary:
*  Sets the buffer pointer and size of the read buffer. This function also
*  resets the transfer count returned with the I2C_SlaveGetReadBufSize function.
*
* Parameters:
*  writeBuf:  Pointer to the data buffer to be read by the master.
*  bufSize:  Size of the buffer exposed to the I2C master.
*
* Return:
*  None.
*
* Global variables:
*  I2COLED_slWrBufPtr   - The global variable used to store a pointer
*                                  to the slave write buffer.
*  I2COLED_slWrBufSize  - The global variable used to store a slave
*                                  write buffer size.
*  I2COLED_slWrBufIndex - The global variable used to store a current
*                                  index within the slave write buffer.
*
* Side Effects:
*  If this function is called during a bus transaction, data from the previous
*  buffer location and the beginning of the current buffer may be transmitted.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I2COLED_SlaveInitWriteBuf(uint8 * wrBuf, uint8 bufSize)
     
{
    if (NULL != wrBuf)
    {
        I2COLED_DisableInt(); /* Lock from interrupt */

         /* Set buffer pointer */
        I2COLED_slWrBufPtr   = (volatile uint8 *) wrBuf;
        I2COLED_slWrBufSize  = bufSize;    /* Set buffer size     */
        I2COLED_slWrBufIndex = 0u;         /* Clears buffer index */

        I2COLED_EnableInt(); /* Release lock */
    }
}


/*******************************************************************************
* Function Name: I2COLED_SlaveGetReadBufSize
********************************************************************************
*
* Summary:
*  Returns the number of bytes read by the I2C master since an
*  I2C_SlaveInitReadBuf or I2C_SlaveClearReadBuf function was executed.
*  The maximum return value will be the size of the read buffer.
*
* Parameters:
*  None.
*
* Return:
*  Bytes read by master.
*
* Global variables:
*  I2COLED_slRdBufIndex - The global variable used to store a current
*                                  index within the slave read buffer.
*
*******************************************************************************/
uint8 I2COLED_SlaveGetReadBufSize(void) 
{
    return (I2COLED_slRdBufIndex);
}


/*******************************************************************************
* Function Name: I2COLED_SlaveGetWriteBufSize
********************************************************************************
*
* Summary:
*  Returns the number of bytes written by the I2C master since an
*  I2C_SlaveInitWriteBuf or I2C_SlaveClearWriteBuf function was executed.
*  The maximum return value will be the size of the write buffer.
*
* Parameters:
*  None.
*
* Return:
*  Bytes written by master.
*
* Global variables:
*  I2COLED_slWrBufIndex - The global variable used to store a current
*                                  index within the slave write buffer.
*
*******************************************************************************/
uint8 I2COLED_SlaveGetWriteBufSize(void) 
{
    return (I2COLED_slWrBufIndex);
}


/*******************************************************************************
* Function Name: I2COLED_SlaveClearReadBuf
********************************************************************************
*
* Summary:
*  Resets the read pointer to the first byte in the read buffer. The next byte
*  read by the master will be the first byte in the read buffer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  I2COLED_slRdBufIndex - The global variable used to store a current
*                                  index the within slave read buffer.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I2COLED_SlaveClearReadBuf(void) 
{
    I2COLED_slRdBufIndex = 0u;
}


/*******************************************************************************
* Function Name: I2COLED_SlaveClearRxBuf
********************************************************************************
*
* Summary:
*  Resets the write pointer to the first byte in the write buffer. The next byte
*  written by the master will be the first byte in the write buffer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  I2COLED_slWrBufIndex - The global variable used to store a current
*                                  index within the slave write buffer.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I2COLED_SlaveClearWriteBuf(void) 
{
    I2COLED_slWrBufIndex = 0u;
}

#endif /* (I2COLED_MODE_SLAVE_ENABLED) */


/* [] END OF FILE */
