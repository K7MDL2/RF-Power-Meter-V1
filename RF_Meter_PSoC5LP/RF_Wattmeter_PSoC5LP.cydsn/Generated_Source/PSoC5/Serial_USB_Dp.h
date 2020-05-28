/*******************************************************************************
* File Name: Serial_USB_Dp.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Serial_USB_Dp_H) /* Pins Serial_USB_Dp_H */
#define CY_PINS_Serial_USB_Dp_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Serial_USB_Dp_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Serial_USB_Dp__PORT == 15 && ((Serial_USB_Dp__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    Serial_USB_Dp_Write(uint8 value);
void    Serial_USB_Dp_SetDriveMode(uint8 mode);
uint8   Serial_USB_Dp_ReadDataReg(void);
uint8   Serial_USB_Dp_Read(void);
void    Serial_USB_Dp_SetInterruptMode(uint16 position, uint16 mode);
uint8   Serial_USB_Dp_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the Serial_USB_Dp_SetDriveMode() function.
     *  @{
     */
        #define Serial_USB_Dp_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define Serial_USB_Dp_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define Serial_USB_Dp_DM_RES_UP          PIN_DM_RES_UP
        #define Serial_USB_Dp_DM_RES_DWN         PIN_DM_RES_DWN
        #define Serial_USB_Dp_DM_OD_LO           PIN_DM_OD_LO
        #define Serial_USB_Dp_DM_OD_HI           PIN_DM_OD_HI
        #define Serial_USB_Dp_DM_STRONG          PIN_DM_STRONG
        #define Serial_USB_Dp_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define Serial_USB_Dp_MASK               Serial_USB_Dp__MASK
#define Serial_USB_Dp_SHIFT              Serial_USB_Dp__SHIFT
#define Serial_USB_Dp_WIDTH              1u

/* Interrupt constants */
#if defined(Serial_USB_Dp__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Serial_USB_Dp_SetInterruptMode() function.
     *  @{
     */
        #define Serial_USB_Dp_INTR_NONE      (uint16)(0x0000u)
        #define Serial_USB_Dp_INTR_RISING    (uint16)(0x0001u)
        #define Serial_USB_Dp_INTR_FALLING   (uint16)(0x0002u)
        #define Serial_USB_Dp_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define Serial_USB_Dp_INTR_MASK      (0x01u) 
#endif /* (Serial_USB_Dp__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Serial_USB_Dp_PS                     (* (reg8 *) Serial_USB_Dp__PS)
/* Data Register */
#define Serial_USB_Dp_DR                     (* (reg8 *) Serial_USB_Dp__DR)
/* Port Number */
#define Serial_USB_Dp_PRT_NUM                (* (reg8 *) Serial_USB_Dp__PRT) 
/* Connect to Analog Globals */                                                  
#define Serial_USB_Dp_AG                     (* (reg8 *) Serial_USB_Dp__AG)                       
/* Analog MUX bux enable */
#define Serial_USB_Dp_AMUX                   (* (reg8 *) Serial_USB_Dp__AMUX) 
/* Bidirectional Enable */                                                        
#define Serial_USB_Dp_BIE                    (* (reg8 *) Serial_USB_Dp__BIE)
/* Bit-mask for Aliased Register Access */
#define Serial_USB_Dp_BIT_MASK               (* (reg8 *) Serial_USB_Dp__BIT_MASK)
/* Bypass Enable */
#define Serial_USB_Dp_BYP                    (* (reg8 *) Serial_USB_Dp__BYP)
/* Port wide control signals */                                                   
#define Serial_USB_Dp_CTL                    (* (reg8 *) Serial_USB_Dp__CTL)
/* Drive Modes */
#define Serial_USB_Dp_DM0                    (* (reg8 *) Serial_USB_Dp__DM0) 
#define Serial_USB_Dp_DM1                    (* (reg8 *) Serial_USB_Dp__DM1)
#define Serial_USB_Dp_DM2                    (* (reg8 *) Serial_USB_Dp__DM2) 
/* Input Buffer Disable Override */
#define Serial_USB_Dp_INP_DIS                (* (reg8 *) Serial_USB_Dp__INP_DIS)
/* LCD Common or Segment Drive */
#define Serial_USB_Dp_LCD_COM_SEG            (* (reg8 *) Serial_USB_Dp__LCD_COM_SEG)
/* Enable Segment LCD */
#define Serial_USB_Dp_LCD_EN                 (* (reg8 *) Serial_USB_Dp__LCD_EN)
/* Slew Rate Control */
#define Serial_USB_Dp_SLW                    (* (reg8 *) Serial_USB_Dp__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Serial_USB_Dp_PRTDSI__CAPS_SEL       (* (reg8 *) Serial_USB_Dp__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Serial_USB_Dp_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Serial_USB_Dp__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Serial_USB_Dp_PRTDSI__OE_SEL0        (* (reg8 *) Serial_USB_Dp__PRTDSI__OE_SEL0) 
#define Serial_USB_Dp_PRTDSI__OE_SEL1        (* (reg8 *) Serial_USB_Dp__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Serial_USB_Dp_PRTDSI__OUT_SEL0       (* (reg8 *) Serial_USB_Dp__PRTDSI__OUT_SEL0) 
#define Serial_USB_Dp_PRTDSI__OUT_SEL1       (* (reg8 *) Serial_USB_Dp__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Serial_USB_Dp_PRTDSI__SYNC_OUT       (* (reg8 *) Serial_USB_Dp__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(Serial_USB_Dp__SIO_CFG)
    #define Serial_USB_Dp_SIO_HYST_EN        (* (reg8 *) Serial_USB_Dp__SIO_HYST_EN)
    #define Serial_USB_Dp_SIO_REG_HIFREQ     (* (reg8 *) Serial_USB_Dp__SIO_REG_HIFREQ)
    #define Serial_USB_Dp_SIO_CFG            (* (reg8 *) Serial_USB_Dp__SIO_CFG)
    #define Serial_USB_Dp_SIO_DIFF           (* (reg8 *) Serial_USB_Dp__SIO_DIFF)
#endif /* (Serial_USB_Dp__SIO_CFG) */

/* Interrupt Registers */
#if defined(Serial_USB_Dp__INTSTAT)
    #define Serial_USB_Dp_INTSTAT            (* (reg8 *) Serial_USB_Dp__INTSTAT)
    #define Serial_USB_Dp_SNAP               (* (reg8 *) Serial_USB_Dp__SNAP)
    
	#define Serial_USB_Dp_0_INTTYPE_REG 		(* (reg8 *) Serial_USB_Dp__0__INTTYPE)
#endif /* (Serial_USB_Dp__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Serial_USB_Dp_H */


/* [] END OF FILE */
