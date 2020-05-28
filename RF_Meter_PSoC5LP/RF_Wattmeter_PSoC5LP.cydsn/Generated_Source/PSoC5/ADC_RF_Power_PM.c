/*******************************************************************************
* File Name: ADC_RF_Power_PM.c
* Version 3.30
*
* Description:
*  This file provides the power manager source code to the API for the
*  Delta-Sigma ADC Component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "ADC_RF_Power.h"

static ADC_RF_Power_BACKUP_STRUCT ADC_RF_Power_backup =
{
    ADC_RF_Power_DISABLED,
    ADC_RF_Power_CFG1_DEC_CR
};


/*******************************************************************************
* Function Name: ADC_RF_Power_SaveConfig
********************************************************************************
*
* Summary:
*  Save the register configuration which are not retention.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC_RF_Power_backup:  This global structure variable is used to store
*  configuration registers which are non retention whenever user wants to go
*  sleep mode by calling Sleep() API.
*
*******************************************************************************/
void ADC_RF_Power_SaveConfig(void) 
{
    ADC_RF_Power_backup.deccr = ADC_RF_Power_DEC_CR_REG;
}


/*******************************************************************************
* Function Name: ADC_RF_Power_RestoreConfig
********************************************************************************
*
* Summary:
*  Restore the register configurations which are not retention.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC_RF_Power_backup:  This global structure variable is used to restore
*  configuration registers which are non retention whenever user wants to switch
*  to active power mode by calling Wakeup() API.
*
*******************************************************************************/
void ADC_RF_Power_RestoreConfig(void) 
{
    ADC_RF_Power_DEC_CR_REG = ADC_RF_Power_backup.deccr;
}


/*******************************************************************************
* Function Name: ADC_RF_Power_Sleep
********************************************************************************
*
* Summary:
*  Stops the operation of the block and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC_RF_Power_backup:  The structure field 'enableState' is modified
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void ADC_RF_Power_Sleep(void) 
{
    /* Save ADC enable state */
    if((ADC_RF_Power_ACT_PWR_DEC_EN == (ADC_RF_Power_PWRMGR_DEC_REG & ADC_RF_Power_ACT_PWR_DEC_EN)) &&
       (ADC_RF_Power_ACT_PWR_DSM_EN == (ADC_RF_Power_PWRMGR_DSM_REG & ADC_RF_Power_ACT_PWR_DSM_EN)))
    {
        /* Component is enabled */
        ADC_RF_Power_backup.enableState = ADC_RF_Power_ENABLED;
        if((ADC_RF_Power_DEC_CR_REG & ADC_RF_Power_DEC_START_CONV) != 0u)
        {   
            /* Conversion is started */
            ADC_RF_Power_backup.enableState |= ADC_RF_Power_STARTED;
        }
		
        /* Stop the configuration */
        ADC_RF_Power_Stop();
    }
    else
    {
        /* Component is disabled */
        ADC_RF_Power_backup.enableState = ADC_RF_Power_DISABLED;
    }

    /* Save the user configuration */
    ADC_RF_Power_SaveConfig();
}


/*******************************************************************************
* Function Name: ADC_RF_Power_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and enables the power to the block.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC_RF_Power_backup:  The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void ADC_RF_Power_Wakeup(void) 
{
    /* Restore the configuration */
    ADC_RF_Power_RestoreConfig();

    /* Enables the component operation */
    if(ADC_RF_Power_backup.enableState != ADC_RF_Power_DISABLED)
    {
        ADC_RF_Power_Enable();
        if((ADC_RF_Power_backup.enableState & ADC_RF_Power_STARTED) != 0u)
        {
            ADC_RF_Power_StartConvert();
        }
    } /* Do nothing if component was disable before */
}


/* [] END OF FILE */
