/*******************************************************************************
* File Name: I2COLED_PM.c
* Version 3.50
*
* Description:
*  This file provides low power mode APIs for the I2C component.
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I2COLED_PVT.h"

I2COLED_BACKUP_STRUCT I2COLED_backup =
{
    I2COLED_DISABLE,

#if (I2COLED_FF_IMPLEMENTED)
    I2COLED_DEFAULT_XCFG,
    I2COLED_DEFAULT_CFG,
    I2COLED_DEFAULT_ADDR,
    LO8(I2COLED_DEFAULT_DIVIDE_FACTOR),
    HI8(I2COLED_DEFAULT_DIVIDE_FACTOR),
#else  /* (I2COLED_UDB_IMPLEMENTED) */
    I2COLED_DEFAULT_CFG,
#endif /* (I2COLED_FF_IMPLEMENTED) */

#if (I2COLED_TIMEOUT_ENABLED)
    I2COLED_DEFAULT_TMOUT_PERIOD,
    I2COLED_DEFAULT_TMOUT_INTR_MASK,
#endif /* (I2COLED_TIMEOUT_ENABLED) */
};

#if ((I2COLED_FF_IMPLEMENTED) && (I2COLED_WAKEUP_ENABLED))
    volatile uint8 I2COLED_wakeupSource;
#endif /* ((I2COLED_FF_IMPLEMENTED) && (I2COLED_WAKEUP_ENABLED)) */


/*******************************************************************************
* Function Name: I2COLED_SaveConfig
********************************************************************************
*
* Summary:
*  The Enable wakeup from Sleep Mode selection influences this function
*  implementation:
*   Unchecked: Stores the component non-retention configuration registers.
*   Checked:   Disables the master, if it was enabled before, and enables
*              backup regulator of the I2C hardware. If a transaction intended
*              for component executes during this function call, it waits until
*              the current transaction is completed and I2C hardware is ready
*              to enter sleep mode. All subsequent I2C traffic is NAKed until
*              the device is put into sleep mode.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  I2COLED_backup - The global variable used to save the component
*                            configuration and non-retention registers before
*                            entering the sleep mode.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I2COLED_SaveConfig(void) 
{
#if (I2COLED_FF_IMPLEMENTED)
    #if (I2COLED_WAKEUP_ENABLED)
        uint8 intState;
    #endif /* (I2COLED_WAKEUP_ENABLED) */

    /* Store registers before enter low power mode */
    I2COLED_backup.cfg     = I2COLED_CFG_REG;
    I2COLED_backup.xcfg    = I2COLED_XCFG_REG;
    I2COLED_backup.addr    = I2COLED_ADDR_REG;
    I2COLED_backup.clkDiv1 = I2COLED_CLKDIV1_REG;
    I2COLED_backup.clkDiv2 = I2COLED_CLKDIV2_REG;

#if (I2COLED_WAKEUP_ENABLED)
    /* Disable master */
    I2COLED_CFG_REG &= (uint8) ~I2COLED_ENABLE_MASTER;

    /* Enable backup regulator to keep block powered in low power mode */
    intState = CyEnterCriticalSection();
    I2COLED_PWRSYS_CR1_REG |= I2COLED_PWRSYS_CR1_I2C_REG_BACKUP;
    CyExitCriticalSection(intState);

    /* 1) Set force NACK to ignore I2C transactions;
    *  2) Wait unti I2C is ready go to Sleep; !!
    *  3) These bits are cleared on wake up.
    */
    /* Wait when block is ready for sleep */
    I2COLED_XCFG_REG |= I2COLED_XCFG_FORCE_NACK;
    while (0u == (I2COLED_XCFG_REG & I2COLED_XCFG_RDY_TO_SLEEP))
    {
    }

    /* Setup wakeup interrupt */
    I2COLED_DisableInt();
    (void) CyIntSetVector(I2COLED_ISR_NUMBER, &I2COLED_WAKEUP_ISR);
    I2COLED_wakeupSource = 0u;
    I2COLED_EnableInt();
#endif /* (I2COLED_WAKEUP_ENABLED) */

#else
    /* Store only address match bit */
    I2COLED_backup.control = (I2COLED_CFG_REG & I2COLED_CTRL_ANY_ADDRESS_MASK);
#endif /* (I2COLED_FF_IMPLEMENTED) */

#if (I2COLED_TIMEOUT_ENABLED)
    I2COLED_TimeoutSaveConfig();
#endif /* (I2COLED_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: I2COLED_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred method to prepare the component before device enters
*  sleep mode. The Enable wakeup from Sleep Mode selection influences this
*  function implementation:
*   Unchecked: Checks current I2C component state, saves it, and disables the
*              component by calling I2C_Stop() if it is currently enabled.
*              I2C_SaveConfig() is then called to save the component
*              non-retention configuration registers.
*   Checked:   If a transaction intended for component executes during this
*              function call, it waits until the current transaction is
*              completed. All subsequent I2C traffic intended for component
*              is NAKed until the device is put to sleep mode. The address
*              match event wakes up the device.
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
void I2COLED_Sleep(void) 
{
#if (I2COLED_WAKEUP_ENABLED)
    /* Do not enable block after exit low power mode if it is wakeup source */
    I2COLED_backup.enableState = I2COLED_DISABLE;

    #if (I2COLED_TIMEOUT_ENABLED)
        I2COLED_TimeoutStop();
    #endif /* (I2COLED_TIMEOUT_ENABLED) */

#else
    /* Store enable state */
    I2COLED_backup.enableState = (uint8) I2COLED_IS_ENABLED;

    if (0u != I2COLED_backup.enableState)
    {
        I2COLED_Stop();
    }
#endif /* (I2COLED_WAKEUP_ENABLED) */

    I2COLED_SaveConfig();
}


/*******************************************************************************
* Function Name: I2COLED_RestoreConfig
********************************************************************************
*
* Summary:
*  The Enable wakeup from Sleep Mode selection influences this function
*  implementation:
*   Unchecked: Restores the component non-retention configuration registers
*              to the state they were in before I2C_Sleep() or I2C_SaveConfig()
*              was called.
*   Checked:   Disables the backup regulator of the I2C hardware. Sets up the
*              regular component interrupt handler and generates the component
*              interrupt if it was wake up source to release the bus and
*              continue in-coming I2C transaction.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  I2COLED_backup - The global variable used to save the component
*                            configuration and non-retention registers before
*                            exiting the sleep mode.
*
* Reentrant:
*  No.
*
* Side Effects:
*  Calling this function before I2COLED_SaveConfig() or
*  I2COLED_Sleep() will lead to unpredictable results.
*
*******************************************************************************/
void I2COLED_RestoreConfig(void) 
{
#if (I2COLED_FF_IMPLEMENTED)
    uint8 intState;

    if (I2COLED_CHECK_PWRSYS_I2C_BACKUP)
    /* Low power mode was Sleep - backup regulator is enabled */
    {
        /* Enable backup regulator in active mode */
        intState = CyEnterCriticalSection();
        I2COLED_PWRSYS_CR1_REG &= (uint8) ~I2COLED_PWRSYS_CR1_I2C_REG_BACKUP;
        CyExitCriticalSection(intState);

        /* Restore master */
        I2COLED_CFG_REG = I2COLED_backup.cfg;
    }
    else
    /* Low power mode was Hibernate - backup regulator is disabled. All registers are cleared */
    {
    #if (I2COLED_WAKEUP_ENABLED)
        /* Disable power to block before register restore */
        intState = CyEnterCriticalSection();
        I2COLED_ACT_PWRMGR_REG  &= (uint8) ~I2COLED_ACT_PWR_EN;
        I2COLED_STBY_PWRMGR_REG &= (uint8) ~I2COLED_STBY_PWR_EN;
        CyExitCriticalSection(intState);

        /* Enable component in I2C_Wakeup() after register restore */
        I2COLED_backup.enableState = I2COLED_ENABLE;
    #endif /* (I2COLED_WAKEUP_ENABLED) */

        /* Restore component registers after Hibernate */
        I2COLED_XCFG_REG    = I2COLED_backup.xcfg;
        I2COLED_CFG_REG     = I2COLED_backup.cfg;
        I2COLED_ADDR_REG    = I2COLED_backup.addr;
        I2COLED_CLKDIV1_REG = I2COLED_backup.clkDiv1;
        I2COLED_CLKDIV2_REG = I2COLED_backup.clkDiv2;
    }

#if (I2COLED_WAKEUP_ENABLED)
    I2COLED_DisableInt();
    (void) CyIntSetVector(I2COLED_ISR_NUMBER, &I2COLED_ISR);
    if (0u != I2COLED_wakeupSource)
    {
        /* Generate interrupt to process incoming transaction */
        I2COLED_SetPendingInt();
    }
    I2COLED_EnableInt();
#endif /* (I2COLED_WAKEUP_ENABLED) */

#else
    I2COLED_CFG_REG = I2COLED_backup.control;
#endif /* (I2COLED_FF_IMPLEMENTED) */

#if (I2COLED_TIMEOUT_ENABLED)
    I2COLED_TimeoutRestoreConfig();
#endif /* (I2COLED_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: I2COLED_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred method to prepare the component for active mode
*  operation (when device exits sleep mode). The Enable wakeup from Sleep Mode
*  selection influences this function implementation:
*   Unchecked: Restores the component non-retention configuration registers
*              by calling I2C_RestoreConfig(). If the component was enabled
*              before the I2C_Sleep() function was called, I2C_Wakeup()
*              re-enables it.
*   Checked:   Enables  master functionality if it was enabled before sleep,
*              and disables the backup regulator of the I2C hardware.
*              The incoming transaction continues as soon as the regular
*              I2C interrupt handler is set up (global interrupts has to be
*              enabled to service I2C component interrupt).
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
* Side Effects:
*  Calling this function before I2COLED_SaveConfig() or
*  I2COLED_Sleep() will lead to unpredictable results.
*
*******************************************************************************/
void I2COLED_Wakeup(void) 
{
    I2COLED_RestoreConfig();

    /* Restore component enable state */
    if (0u != I2COLED_backup.enableState)
    {
        I2COLED_Enable();
        I2COLED_EnableInt();
    }
    else
    {
    #if (I2COLED_TIMEOUT_ENABLED)
        I2COLED_TimeoutEnable();
    #endif /* (I2COLED_TIMEOUT_ENABLED) */
    }
}


/* [] END OF FILE */
