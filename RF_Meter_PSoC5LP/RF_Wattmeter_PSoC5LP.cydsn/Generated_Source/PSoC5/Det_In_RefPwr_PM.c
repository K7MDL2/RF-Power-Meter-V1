/*******************************************************************************
* File Name: Det_In_RefPwr_PM.c
* Version 1.90
*
* Description:
*  This file provides the power management source code to the API for the 
*  OpAmp (Analog Buffer) component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "Det_In_RefPwr.h"

static Det_In_RefPwr_BACKUP_STRUCT  Det_In_RefPwr_backup;


/*******************************************************************************  
* Function Name: Det_In_RefPwr_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration registers.
* 
* Parameters:
*  void
* 
* Return:
*  void
*
*******************************************************************************/
void Det_In_RefPwr_SaveConfig(void) 
{
    /* Nothing to save as registers are System reset on retention flops */
}


/*******************************************************************************  
* Function Name: Det_In_RefPwr_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration registers.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Det_In_RefPwr_RestoreConfig(void) 
{
    /* Nothing to restore */
}


/*******************************************************************************   
* Function Name: Det_In_RefPwr_Sleep
********************************************************************************
*
* Summary:
*  Disables block's operation and saves its configuration. Should be called 
*  just prior to entering sleep.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Det_In_RefPwr_backup: The structure field 'enableState' is modified 
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void Det_In_RefPwr_Sleep(void) 
{
    /* Save OpAmp enable state */
    if((Det_In_RefPwr_PM_ACT_CFG_REG & Det_In_RefPwr_ACT_PWR_EN) != 0u)
    {
        /* Component is enabled */
        Det_In_RefPwr_backup.enableState = 1u;
         /* Stops the component */
         Det_In_RefPwr_Stop();
    }
    else
    {
        /* Component is disabled */
        Det_In_RefPwr_backup.enableState = 0u;
    }
    /* Saves the configuration */
    Det_In_RefPwr_SaveConfig();
}


/*******************************************************************************  
* Function Name: Det_In_RefPwr_Wakeup
********************************************************************************
*
* Summary:
*  Enables block's operation and restores its configuration. Should be called
*  just after awaking from sleep.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Det_In_RefPwr_backup: The structure field 'enableState' is used to 
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void Det_In_RefPwr_Wakeup(void) 
{
    /* Restore the user configuration */
    Det_In_RefPwr_RestoreConfig();

    /* Enables the component operation */
    if(Det_In_RefPwr_backup.enableState == 1u)
    {
        Det_In_RefPwr_Enable();
    } /* Do nothing if component was disable before */
}


/* [] END OF FILE */
