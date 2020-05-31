/*******************************************************************************
* File Name: I22OLED.c
* Version 3.50
*
* Description:
*  This file provides the source code of APIs for the I2C component.
*  The actual protocol and operation code resides in the interrupt service
*  routine file.
*
*******************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I22OLED_PVT.h"


/**********************************
*      System variables
**********************************/

uint8 I22OLED_initVar = 0u; /* Defines if component was initialized */

volatile uint8 I22OLED_state;  /* Current state of I2C FSM */


/*******************************************************************************
* Function Name: I22OLED_Init
********************************************************************************
*
* Summary:
*  Initializes I2C registers with initial values provided from customizer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I22OLED_Init(void) 
{
#if (I22OLED_FF_IMPLEMENTED)
    /* Configure fixed function block */
    I22OLED_CFG_REG  = I22OLED_DEFAULT_CFG;
    I22OLED_XCFG_REG = I22OLED_DEFAULT_XCFG;
    I22OLED_ADDR_REG = I22OLED_DEFAULT_ADDR;
    I22OLED_CLKDIV1_REG = LO8(I22OLED_DEFAULT_DIVIDE_FACTOR);
    I22OLED_CLKDIV2_REG = HI8(I22OLED_DEFAULT_DIVIDE_FACTOR);

#else
    uint8 intState;

    /* Configure control and interrupt sources */
    I22OLED_CFG_REG      = I22OLED_DEFAULT_CFG;
    I22OLED_INT_MASK_REG = I22OLED_DEFAULT_INT_MASK;

    /* Enable interrupt generation in status */
    intState = CyEnterCriticalSection();
    I22OLED_INT_ENABLE_REG |= I22OLED_INTR_ENABLE;
    CyExitCriticalSection(intState);

    /* Configure bit counter */
    #if (I22OLED_MODE_SLAVE_ENABLED)
        I22OLED_PERIOD_REG = I22OLED_DEFAULT_PERIOD;
    #endif  /* (I22OLED_MODE_SLAVE_ENABLED) */

    /* Configure clock generator */
    #if (I22OLED_MODE_MASTER_ENABLED)
        I22OLED_MCLK_PRD_REG = I22OLED_DEFAULT_MCLK_PRD;
        I22OLED_MCLK_CMP_REG = I22OLED_DEFAULT_MCLK_CMP;
    #endif /* (I22OLED_MODE_MASTER_ENABLED) */
#endif /* (I22OLED_FF_IMPLEMENTED) */

#if (I22OLED_TIMEOUT_ENABLED)
    I22OLED_TimeoutInit();
#endif /* (I22OLED_TIMEOUT_ENABLED) */

    /* Configure internal interrupt */
    CyIntDisable    (I22OLED_ISR_NUMBER);
    CyIntSetPriority(I22OLED_ISR_NUMBER, I22OLED_ISR_PRIORITY);
    #if (I22OLED_INTERN_I2C_INTR_HANDLER)
        (void) CyIntSetVector(I22OLED_ISR_NUMBER, &I22OLED_ISR);
    #endif /* (I22OLED_INTERN_I2C_INTR_HANDLER) */

    /* Set FSM to default state */
    I22OLED_state = I22OLED_SM_IDLE;

#if (I22OLED_MODE_SLAVE_ENABLED)
    /* Clear status and buffers index */
    I22OLED_slStatus = 0u;
    I22OLED_slRdBufIndex = 0u;
    I22OLED_slWrBufIndex = 0u;

    /* Configure matched address */
    I22OLED_SlaveSetAddress(I22OLED_DEFAULT_ADDR);
#endif /* (I22OLED_MODE_SLAVE_ENABLED) */

#if (I22OLED_MODE_MASTER_ENABLED)
    /* Clear status and buffers index */
    I22OLED_mstrStatus = 0u;
    I22OLED_mstrRdBufIndex = 0u;
    I22OLED_mstrWrBufIndex = 0u;
#endif /* (I22OLED_MODE_MASTER_ENABLED) */
}


/*******************************************************************************
* Function Name: I22OLED_Enable
********************************************************************************
*
* Summary:
*  Enables I2C operations.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  None.
*
*******************************************************************************/
void I22OLED_Enable(void) 
{
#if (I22OLED_FF_IMPLEMENTED)
    uint8 intState;

    /* Enable power to block */
    intState = CyEnterCriticalSection();
    I22OLED_ACT_PWRMGR_REG  |= I22OLED_ACT_PWR_EN;
    I22OLED_STBY_PWRMGR_REG |= I22OLED_STBY_PWR_EN;
    CyExitCriticalSection(intState);
#else
    #if (I22OLED_MODE_SLAVE_ENABLED)
        /* Enable bit counter */
        uint8 intState = CyEnterCriticalSection();
        I22OLED_COUNTER_AUX_CTL_REG |= I22OLED_CNT7_ENABLE;
        CyExitCriticalSection(intState);
    #endif /* (I22OLED_MODE_SLAVE_ENABLED) */

    /* Enable slave or master bits */
    I22OLED_CFG_REG |= I22OLED_ENABLE_MS;
#endif /* (I22OLED_FF_IMPLEMENTED) */

#if (I22OLED_TIMEOUT_ENABLED)
    I22OLED_TimeoutEnable();
#endif /* (I22OLED_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: I22OLED_Start
********************************************************************************
*
* Summary:
*  Starts the I2C hardware. Enables Active mode power template bits or clock
*  gating as appropriate. It is required to be executed before I2C bus
*  operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  This component automatically enables its interrupt.  If I2C is enabled !
*  without the interrupt enabled, it can lock up the I2C bus.
*
* Global variables:
*  I22OLED_initVar - This variable is used to check the initial
*                             configuration, modified on the first
*                             function call.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I22OLED_Start(void) 
{
    if (0u == I22OLED_initVar)
    {
        I22OLED_Init();
        I22OLED_initVar = 1u; /* Component initialized */
    }

    I22OLED_Enable();
    I22OLED_EnableInt();
}


/*******************************************************************************
* Function Name: I22OLED_Stop
********************************************************************************
*
* Summary:
*  Disables I2C hardware and disables I2C interrupt. Disables Active mode power
*  template bits or clock gating as appropriate.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void I22OLED_Stop(void) 
{
    I22OLED_DisableInt();

#if (I22OLED_TIMEOUT_ENABLED)
    I22OLED_TimeoutStop();
#endif  /* End (I22OLED_TIMEOUT_ENABLED) */

#if (I22OLED_FF_IMPLEMENTED)
    {
        uint8 intState;
        uint16 blockResetCycles;

        /* Store registers effected by block disable */
        I22OLED_backup.addr    = I22OLED_ADDR_REG;
        I22OLED_backup.clkDiv1 = I22OLED_CLKDIV1_REG;
        I22OLED_backup.clkDiv2 = I22OLED_CLKDIV2_REG;

        /* Calculate number of cycles to reset block */
        blockResetCycles = ((uint16) ((uint16) I22OLED_CLKDIV2_REG << 8u) | I22OLED_CLKDIV1_REG) + 1u;

        /* Disable block */
        I22OLED_CFG_REG &= (uint8) ~I22OLED_CFG_EN_SLAVE;
        /* Wait for block reset before disable power */
        CyDelayCycles((uint32) blockResetCycles);

        /* Disable power to block */
        intState = CyEnterCriticalSection();
        I22OLED_ACT_PWRMGR_REG  &= (uint8) ~I22OLED_ACT_PWR_EN;
        I22OLED_STBY_PWRMGR_REG &= (uint8) ~I22OLED_STBY_PWR_EN;
        CyExitCriticalSection(intState);

        /* Enable block */
        I22OLED_CFG_REG |= (uint8) I22OLED_ENABLE_MS;

        /* Restore registers effected by block disable. Ticket ID#198004 */
        I22OLED_ADDR_REG    = I22OLED_backup.addr;
        I22OLED_ADDR_REG    = I22OLED_backup.addr;
        I22OLED_CLKDIV1_REG = I22OLED_backup.clkDiv1;
        I22OLED_CLKDIV2_REG = I22OLED_backup.clkDiv2;
    }
#else

    /* Disable slave or master bits */
    I22OLED_CFG_REG &= (uint8) ~I22OLED_ENABLE_MS;

#if (I22OLED_MODE_SLAVE_ENABLED)
    {
        /* Disable bit counter */
        uint8 intState = CyEnterCriticalSection();
        I22OLED_COUNTER_AUX_CTL_REG &= (uint8) ~I22OLED_CNT7_ENABLE;
        CyExitCriticalSection(intState);
    }
#endif /* (I22OLED_MODE_SLAVE_ENABLED) */

    /* Clear interrupt source register */
    (void) I22OLED_CSR_REG;
#endif /* (I22OLED_FF_IMPLEMENTED) */

    /* Disable interrupt on stop (enabled by write transaction) */
    I22OLED_DISABLE_INT_ON_STOP;
    I22OLED_ClearPendingInt();

    /* Reset FSM to default state */
    I22OLED_state = I22OLED_SM_IDLE;

    /* Clear busy statuses */
#if (I22OLED_MODE_SLAVE_ENABLED)
    I22OLED_slStatus &= (uint8) ~(I22OLED_SSTAT_RD_BUSY | I22OLED_SSTAT_WR_BUSY);
#endif /* (I22OLED_MODE_SLAVE_ENABLED) */
}


/* [] END OF FILE */
