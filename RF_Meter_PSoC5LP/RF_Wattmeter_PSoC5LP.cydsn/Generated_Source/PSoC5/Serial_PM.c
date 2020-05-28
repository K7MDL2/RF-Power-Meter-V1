/*******************************************************************************
* File Name: Serial_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
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


/***************************************
* Local data allocation
***************************************/

static Serial_BACKUP_STRUCT  Serial_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Serial_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the Serial_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Serial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Serial_SaveConfig(void)
{
    #if(Serial_CONTROL_REG_REMOVED == 0u)
        Serial_backup.cr = Serial_CONTROL_REG;
    #endif /* End Serial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Serial_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Serial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling Serial_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void Serial_RestoreConfig(void)
{
    #if(Serial_CONTROL_REG_REMOVED == 0u)
        Serial_CONTROL_REG = Serial_backup.cr;
    #endif /* End Serial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Serial_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The Serial_Sleep() API saves the current component state. Then it
*  calls the Serial_Stop() function and calls 
*  Serial_SaveConfig() to save the hardware configuration.
*  Call the Serial_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Serial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Serial_Sleep(void)
{
    #if(Serial_RX_ENABLED || Serial_HD_ENABLED)
        if((Serial_RXSTATUS_ACTL_REG  & Serial_INT_ENABLE) != 0u)
        {
            Serial_backup.enableState = 1u;
        }
        else
        {
            Serial_backup.enableState = 0u;
        }
    #else
        if((Serial_TXSTATUS_ACTL_REG  & Serial_INT_ENABLE) !=0u)
        {
            Serial_backup.enableState = 1u;
        }
        else
        {
            Serial_backup.enableState = 0u;
        }
    #endif /* End Serial_RX_ENABLED || Serial_HD_ENABLED*/

    Serial_Stop();
    Serial_SaveConfig();
}


/*******************************************************************************
* Function Name: Serial_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  Serial_Sleep() was called. The Serial_Wakeup() function
*  calls the Serial_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  Serial_Sleep() function was called, the Serial_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Serial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Serial_Wakeup(void)
{
    Serial_RestoreConfig();
    #if( (Serial_RX_ENABLED) || (Serial_HD_ENABLED) )
        Serial_ClearRxBuffer();
    #endif /* End (Serial_RX_ENABLED) || (Serial_HD_ENABLED) */
    #if(Serial_TX_ENABLED || Serial_HD_ENABLED)
        Serial_ClearTxBuffer();
    #endif /* End Serial_TX_ENABLED || Serial_HD_ENABLED */

    if(Serial_backup.enableState != 0u)
    {
        Serial_Enable();
    }
}


/* [] END OF FILE */
