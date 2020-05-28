/*******************************************************************************
* File Name: ADC_RefPwr_PM.c
* Version 3.10
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

#include "ADC_RefPwr.h"


/***************************************
* Local data allocation
***************************************/

static ADC_RefPwr_BACKUP_STRUCT  ADC_RefPwr_backup =
{
    ADC_RefPwr_DISABLED
};


/*******************************************************************************
* Function Name: ADC_RefPwr_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_RefPwr_SaveConfig(void)
{
    /* All configuration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_RefPwr_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_RefPwr_RestoreConfig(void)
{
    /* All congiguration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_RefPwr_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred routine to prepare the component for sleep.
*  The ADC_RefPwr_Sleep() routine saves the current component state,
*  then it calls the ADC_Stop() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ADC_RefPwr_backup - The structure field 'enableState' is modified
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void ADC_RefPwr_Sleep(void)
{
    if((ADC_RefPwr_PWRMGR_SAR_REG  & ADC_RefPwr_ACT_PWR_SAR_EN) != 0u)
    {
        if((ADC_RefPwr_SAR_CSR0_REG & ADC_RefPwr_SAR_SOF_START_CONV) != 0u)
        {
            ADC_RefPwr_backup.enableState = ADC_RefPwr_ENABLED | ADC_RefPwr_STARTED;
        }
        else
        {
            ADC_RefPwr_backup.enableState = ADC_RefPwr_ENABLED;
        }
        ADC_RefPwr_Stop();
    }
    else
    {
        ADC_RefPwr_backup.enableState = ADC_RefPwr_DISABLED;
    }
}


/*******************************************************************************
* Function Name: ADC_RefPwr_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred routine to restore the component to the state when
*  ADC_RefPwr_Sleep() was called. If the component was enabled before the
*  ADC_RefPwr_Sleep() function was called, the
*  ADC_RefPwr_Wakeup() function also re-enables the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ADC_RefPwr_backup - The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void ADC_RefPwr_Wakeup(void)
{
    if(ADC_RefPwr_backup.enableState != ADC_RefPwr_DISABLED)
    {
        ADC_RefPwr_Enable();
        #if(ADC_RefPwr_DEFAULT_CONV_MODE != ADC_RefPwr__HARDWARE_TRIGGER)
            if((ADC_RefPwr_backup.enableState & ADC_RefPwr_STARTED) != 0u)
            {
                ADC_RefPwr_StartConvert();
            }
        #endif /* End ADC_RefPwr_DEFAULT_CONV_MODE != ADC_RefPwr__HARDWARE_TRIGGER */
    }
}


/* [] END OF FILE */
