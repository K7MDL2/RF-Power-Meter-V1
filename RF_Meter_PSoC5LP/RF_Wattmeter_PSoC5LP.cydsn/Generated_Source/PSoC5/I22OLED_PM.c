/*******************************************************************************
* File Name: I22OLED_PM.c
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

#include "I22OLED_PVT.h"

I22OLED_BACKUP_STRUCT I22OLED_backup =
{
    I22OLED_DISABLE,

#if (I22OLED_FF_IMPLEMENTED)
    I22OLED_DEFAULT_XCFG,
    I22OLED_DEFAULT_CFG,
    I22OLED_DEFAULT_ADDR,
    LO8(I22OLED_DEFAULT_DIVIDE_FACTOR),
    HI8(I22OLED_DEFAULT_DIVIDE_FACTOR),
#else  /* (I22OLED_UDB_IMPLEMENTED) */
    I22OLED_DEFAULT_CFG,
#endif /* (I22OLED_FF_IMPLEMENTED) */

#if (I22OLED_TIMEOUT_ENABLED)
    I22OLED_DEFAULT_TMOUT_PERIOD,
    I22OLED_DEFAULT_TMOUT_INTR_MASK,
#endif /* (I22OLED_TIMEOUT_ENABLED) */
};

#if ((I22OLED_FF_IMPLEMENTED) && (I22OLED_WAKEUP_ENABLED))
    volatile uint8 I22OLED_wakeupSource;
#endif /* ((I22OLED_FF_IMPLEMENTED) && (I22OLED_WAKEUP_ENABLED)) */


/*******************************************************************************
* Function Name: I22OLED_SaveConfig
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
*  I22OLED_backup - The global variable used to save the component
*                            configuration and non-retention registers before
*                            entering the sleep mode.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void I22OLED_SaveConfig(void) 
{
#if (I22OLED_FF_IMPLEMENTED)
    #if (I22OLED_WAKEUP_ENABLED)
        uint8 intState;
    #endif /* (I22OLED_WAKEUP_ENABLED) */

    /* Store registers before enter low power mode */
    I22OLED_backup.cfg     = I22OLED_CFG_REG;
    I22OLED_backup.xcfg    = I22OLED_XCFG_REG;
    I22OLED_backup.addr    = I22OLED_ADDR_REG;
    I22OLED_backup.clkDiv1 = I22OLED_CLKDIV1_REG;
    I22OLED_backup.clkDiv2 = I22OLED_CLKDIV2_REG;

#if (I22OLED_WAKEUP_ENABLED)
    /* Disable master */
    I22OLED_CFG_REG &= (uint8) ~I22OLED_ENABLE_MASTER;

    /* Enable backup regulator to keep block powered in low power mode */
    intState = CyEnterCriticalSection();
    I22OLED_PWRSYS_CR1_REG |= I22OLED_PWRSYS_CR1_I2C_REG_BACKUP;
    CyExitCriticalSection(intState);

    /* 1) Set force NACK to ignore I2C transactions;
    *  2) Wait unti I2C is ready go to Sleep; !!
    *  3) These bits are cleared on wake up.
    */
    /* Wait when block is ready for sleep */
    I22OLED_XCFG_REG |= I22OLED_XCFG_FORCE_NACK;
    while (0u == (I22OLED_XCFG_REG & I22OLED_XCFG_RDY_TO_SLEEP))
    {
    }

    /* Setup wakeup interrupt */
    I22OLED_DisableInt();
    (void) CyIntSetVector(I22OLED_ISR_NUMBER, &I22OLED_WAKEUP_ISR);
    I22OLED_wakeupSource = 0u;
    I22OLED_EnableInt();
#endif /* (I22OLED_WAKEUP_ENABLED) */

#else
    /* Store only address match bit */
    I22OLED_backup.control = (I22OLED_CFG_REG & I22OLED_CTRL_ANY_ADDRESS_MASK);
#endif /* (I22OLED_FF_IMPLEMENTED) */

#if (I22OLED_TIMEOUT_ENABLED)
    I22OLED_TimeoutSaveConfig();
#endif /* (I22OLED_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: I22OLED_Sleep
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
void I22OLED_Sleep(void) 
{
#if (I22OLED_WAKEUP_ENABLED)
    /* Do not enable block after exit low power mode if it is wakeup source */
    I22OLED_backup.enableState = I22OLED_DISABLE;

    #if (I22OLED_TIMEOUT_ENABLED)
        I22OLED_TimeoutStop();
    #endif /* (I22OLED_TIMEOUT_ENABLED) */

#else
    /* Store enable state */
    I22OLED_backup.enableState = (uint8) I22OLED_IS_ENABLED;

    if (0u != I22OLED_backup.enableState)
    {
        I22OLED_Stop();
    }
#endif /* (I22OLED_WAKEUP_ENABLED) */

    I22OLED_SaveConfig();
}


/*******************************************************************************
* Function Name: I22OLED_RestoreConfig
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
*  I22OLED_backup - The global variable used to save the component
*                            configuration and non-retention registers before
*                            exiting the sleep mode.
*
* Reentrant:
*  No.
*
* Side Effects:
*  Calling this function before I22OLED_SaveConfig() or
*  I22OLED_Sleep() will lead to unpredictable results.
*
*******************************************************************************/
void I22OLED_RestoreConfig(void) 
{
#if (I22OLED_FF_IMPLEMENTED)
    uint8 intState;

    if (I22OLED_CHECK_PWRSYS_I2C_BACKUP)
    /* Low power mode was Sleep - backup regulator is enabled */
    {
        /* Enable backup regulator in active mode */
        intState = CyEnterCriticalSection();
        I22OLED_PWRSYS_CR1_REG &= (uint8) ~I22OLED_PWRSYS_CR1_I2C_REG_BACKUP;
        CyExitCriticalSection(intState);

        /* Restore master */
        I22OLED_CFG_REG = I22OLED_backup.cfg;
    }
    else
    /* Low power mode was Hibernate - backup regulator is disabled. All registers are cleared */
    {
    #if (I22OLED_WAKEUP_ENABLED)
        /* Disable power to block before register restore */
        intState = CyEnterCriticalSection();
        I22OLED_ACT_PWRMGR_REG  &= (uint8) ~I22OLED_ACT_PWR_EN;
        I22OLED_STBY_PWRMGR_REG &= (uint8) ~I22OLED_STBY_PWR_EN;
        CyExitCriticalSection(intState);

        /* Enable component in I2C_Wakeup() after register restore */
        I22OLED_backup.enableState = I22OLED_ENABLE;
    #endif /* (I22OLED_WAKEUP_ENABLED) */

        /* Restore component registers after Hibernate */
        I22OLED_XCFG_REG    = I22OLED_backup.xcfg;
        I22OLED_CFG_REG     = I22OLED_backup.cfg;
        I22OLED_ADDR_REG    = I22OLED_backup.addr;
        I22OLED_CLKDIV1_REG = I22OLED_backup.clkDiv1;
        I22OLED_CLKDIV2_REG = I22OLED_backup.clkDiv2;
    }

#if (I22OLED_WAKEUP_ENABLED)
    I22OLED_DisableInt();
    (void) CyIntSetVector(I22OLED_ISR_NUMBER, &I22OLED_ISR);
    if (0u != I22OLED_wakeupSource)
    {
        /* Generate interrupt to process incoming transaction */
        I22OLED_SetPendingInt();
    }
    I22OLED_EnableInt();
#endif /* (I22OLED_WAKEUP_ENABLED) */

#else
    I22OLED_CFG_REG = I22OLED_backup.control;
#endif /* (I22OLED_FF_IMPLEMENTED) */

#if (I22OLED_TIMEOUT_ENABLED)
    I22OLED_TimeoutRestoreConfig();
#endif /* (I22OLED_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: I22OLED_Wakeup
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
*  Calling this function before I22OLED_SaveConfig() or
*  I22OLED_Sleep() will lead to unpredictable results.
*
*******************************************************************************/
void I22OLED_Wakeup(void) 
{
    I22OLED_RestoreConfig();

    /* Restore component enable state */
    if (0u != I22OLED_backup.enableState)
    {
        I22OLED_Enable();
        I22OLED_EnableInt();
    }
    else
    {
    #if (I22OLED_TIMEOUT_ENABLED)
        I22OLED_TimeoutEnable();
    #endif /* (I22OLED_TIMEOUT_ENABLED) */
    }
}


/* [] END OF FILE */
