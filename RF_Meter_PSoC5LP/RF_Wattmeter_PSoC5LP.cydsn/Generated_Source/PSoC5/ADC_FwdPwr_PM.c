/*******************************************************************************
* File Name: ADC_FwdPwr_PM.c
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

#include "ADC_FwdPwr.h"


/***************************************
* Local data allocation
***************************************/

static ADC_FwdPwr_BACKUP_STRUCT  ADC_FwdPwr_backup =
{
    ADC_FwdPwr_DISABLED
};


/*******************************************************************************
* Function Name: ADC_FwdPwr_SaveConfig
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
void ADC_FwdPwr_SaveConfig(void)
{
    /* All configuration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_FwdPwr_RestoreConfig
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
void ADC_FwdPwr_RestoreConfig(void)
{
    /* All congiguration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_FwdPwr_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred routine to prepare the component for sleep.
*  The ADC_FwdPwr_Sleep() routine saves the current component state,
*  then it calls the ADC_Stop() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ADC_FwdPwr_backup - The structure field 'enableState' is modified
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void ADC_FwdPwr_Sleep(void)
{
    if((ADC_FwdPwr_PWRMGR_SAR_REG  & ADC_FwdPwr_ACT_PWR_SAR_EN) != 0u)
    {
        if((ADC_FwdPwr_SAR_CSR0_REG & ADC_FwdPwr_SAR_SOF_START_CONV) != 0u)
        {
            ADC_FwdPwr_backup.enableState = ADC_FwdPwr_ENABLED | ADC_FwdPwr_STARTED;
        }
        else
        {
            ADC_FwdPwr_backup.enableState = ADC_FwdPwr_ENABLED;
        }
        ADC_FwdPwr_Stop();
    }
    else
    {
        ADC_FwdPwr_backup.enableState = ADC_FwdPwr_DISABLED;
    }
}


/*******************************************************************************
* Function Name: ADC_FwdPwr_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred routine to restore the component to the state when
*  ADC_FwdPwr_Sleep() was called. If the component was enabled before the
*  ADC_FwdPwr_Sleep() function was called, the
*  ADC_FwdPwr_Wakeup() function also re-enables the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ADC_FwdPwr_backup - The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void ADC_FwdPwr_Wakeup(void)
{
    if(ADC_FwdPwr_backup.enableState != ADC_FwdPwr_DISABLED)
    {
        ADC_FwdPwr_Enable();
        #if(ADC_FwdPwr_DEFAULT_CONV_MODE != ADC_FwdPwr__HARDWARE_TRIGGER)
            if((ADC_FwdPwr_backup.enableState & ADC_FwdPwr_STARTED) != 0u)
            {
                ADC_FwdPwr_StartConvert();
            }
        #endif /* End ADC_FwdPwr_DEFAULT_CONV_MODE != ADC_FwdPwr__HARDWARE_TRIGGER */
    }
}


/* [] END OF FILE */
