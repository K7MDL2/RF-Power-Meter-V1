/*******************************************************************************
* File Name: Det_In_RefPwr.c
* Version 1.90
*
* Description:
*  This file provides the source code to the API for OpAmp (Analog Buffer) 
*  Component.
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

uint8 Det_In_RefPwr_initVar = 0u;


/*******************************************************************************   
* Function Name: Det_In_RefPwr_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  Det_In_RefPwr_Start().
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Det_In_RefPwr_Init(void) 
{
    Det_In_RefPwr_SetPower(Det_In_RefPwr_DEFAULT_POWER);
}


/*******************************************************************************   
* Function Name: Det_In_RefPwr_Enable
********************************************************************************
*
* Summary:
*  Enables the OpAmp block operation
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Det_In_RefPwr_Enable(void) 
{
    /* Enable negative charge pumps in ANIF */
    Det_In_RefPwr_PUMP_CR1_REG  |= (Det_In_RefPwr_PUMP_CR1_CLKSEL | Det_In_RefPwr_PUMP_CR1_FORCE);

    /* Enable power to buffer in active mode */
    Det_In_RefPwr_PM_ACT_CFG_REG |= Det_In_RefPwr_ACT_PWR_EN;

    /* Enable power to buffer in alternative active mode */
    Det_In_RefPwr_PM_STBY_CFG_REG |= Det_In_RefPwr_STBY_PWR_EN;
}


/*******************************************************************************
* Function Name:   Det_In_RefPwr_Start
********************************************************************************
*
* Summary:
*  The start function initializes the Analog Buffer with the default values and 
*  sets the power to the given level. A power level of 0, is same as 
*  executing the stop function.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Det_In_RefPwr_initVar: Used to check the initial configuration, modified 
*  when this function is called for the first time.
*
*******************************************************************************/
void Det_In_RefPwr_Start(void) 
{
    if(Det_In_RefPwr_initVar == 0u)
    {
        Det_In_RefPwr_initVar = 1u;
        Det_In_RefPwr_Init();
    }

    Det_In_RefPwr_Enable();
}


/*******************************************************************************
* Function Name: Det_In_RefPwr_Stop
********************************************************************************
*
* Summary:
*  Powers down amplifier to lowest power state.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Det_In_RefPwr_Stop(void) 
{
    /* Disable power to buffer in active mode template */
    Det_In_RefPwr_PM_ACT_CFG_REG &= (uint8)(~Det_In_RefPwr_ACT_PWR_EN);

    /* Disable power to buffer in alternative active mode template */
    Det_In_RefPwr_PM_STBY_CFG_REG &= (uint8)(~Det_In_RefPwr_STBY_PWR_EN);
    
    /* Disable negative charge pumps for ANIF only if all ABuf is turned OFF */
    if(Det_In_RefPwr_PM_ACT_CFG_REG == 0u)
    {
        Det_In_RefPwr_PUMP_CR1_REG &= (uint8)(~(Det_In_RefPwr_PUMP_CR1_CLKSEL | Det_In_RefPwr_PUMP_CR1_FORCE));
    }
}


/*******************************************************************************
* Function Name: Det_In_RefPwr_SetPower
********************************************************************************
*
* Summary:
*  Sets power level of Analog buffer.
*
* Parameters: 
*  power: PSoC3: Sets power level between low (1) and high power (3).
*         PSoC5: Sets power level High (0)
*
* Return:
*  void
*
**********************************************************************************/
void Det_In_RefPwr_SetPower(uint8 power) 
{
    #if (CY_PSOC3 || CY_PSOC5LP)
        Det_In_RefPwr_CR_REG &= (uint8)(~Det_In_RefPwr_PWR_MASK);
        Det_In_RefPwr_CR_REG |= power & Det_In_RefPwr_PWR_MASK;      /* Set device power */
    #else
        CYASSERT(Det_In_RefPwr_HIGHPOWER == power);
    #endif /* CY_PSOC3 || CY_PSOC5LP */
}


/* [] END OF FILE */
