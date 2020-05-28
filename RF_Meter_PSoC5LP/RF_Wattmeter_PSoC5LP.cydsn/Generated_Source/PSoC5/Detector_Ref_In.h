/*******************************************************************************
* File Name: Detector_Ref_In.h  
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

#if !defined(CY_PINS_Detector_Ref_In_H) /* Pins Detector_Ref_In_H */
#define CY_PINS_Detector_Ref_In_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Detector_Ref_In_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Detector_Ref_In__PORT == 15 && ((Detector_Ref_In__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    Detector_Ref_In_Write(uint8 value);
void    Detector_Ref_In_SetDriveMode(uint8 mode);
uint8   Detector_Ref_In_ReadDataReg(void);
uint8   Detector_Ref_In_Read(void);
void    Detector_Ref_In_SetInterruptMode(uint16 position, uint16 mode);
uint8   Detector_Ref_In_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the Detector_Ref_In_SetDriveMode() function.
     *  @{
     */
        #define Detector_Ref_In_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define Detector_Ref_In_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define Detector_Ref_In_DM_RES_UP          PIN_DM_RES_UP
        #define Detector_Ref_In_DM_RES_DWN         PIN_DM_RES_DWN
        #define Detector_Ref_In_DM_OD_LO           PIN_DM_OD_LO
        #define Detector_Ref_In_DM_OD_HI           PIN_DM_OD_HI
        #define Detector_Ref_In_DM_STRONG          PIN_DM_STRONG
        #define Detector_Ref_In_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define Detector_Ref_In_MASK               Detector_Ref_In__MASK
#define Detector_Ref_In_SHIFT              Detector_Ref_In__SHIFT
#define Detector_Ref_In_WIDTH              1u

/* Interrupt constants */
#if defined(Detector_Ref_In__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Detector_Ref_In_SetInterruptMode() function.
     *  @{
     */
        #define Detector_Ref_In_INTR_NONE      (uint16)(0x0000u)
        #define Detector_Ref_In_INTR_RISING    (uint16)(0x0001u)
        #define Detector_Ref_In_INTR_FALLING   (uint16)(0x0002u)
        #define Detector_Ref_In_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define Detector_Ref_In_INTR_MASK      (0x01u) 
#endif /* (Detector_Ref_In__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Detector_Ref_In_PS                     (* (reg8 *) Detector_Ref_In__PS)
/* Data Register */
#define Detector_Ref_In_DR                     (* (reg8 *) Detector_Ref_In__DR)
/* Port Number */
#define Detector_Ref_In_PRT_NUM                (* (reg8 *) Detector_Ref_In__PRT) 
/* Connect to Analog Globals */                                                  
#define Detector_Ref_In_AG                     (* (reg8 *) Detector_Ref_In__AG)                       
/* Analog MUX bux enable */
#define Detector_Ref_In_AMUX                   (* (reg8 *) Detector_Ref_In__AMUX) 
/* Bidirectional Enable */                                                        
#define Detector_Ref_In_BIE                    (* (reg8 *) Detector_Ref_In__BIE)
/* Bit-mask for Aliased Register Access */
#define Detector_Ref_In_BIT_MASK               (* (reg8 *) Detector_Ref_In__BIT_MASK)
/* Bypass Enable */
#define Detector_Ref_In_BYP                    (* (reg8 *) Detector_Ref_In__BYP)
/* Port wide control signals */                                                   
#define Detector_Ref_In_CTL                    (* (reg8 *) Detector_Ref_In__CTL)
/* Drive Modes */
#define Detector_Ref_In_DM0                    (* (reg8 *) Detector_Ref_In__DM0) 
#define Detector_Ref_In_DM1                    (* (reg8 *) Detector_Ref_In__DM1)
#define Detector_Ref_In_DM2                    (* (reg8 *) Detector_Ref_In__DM2) 
/* Input Buffer Disable Override */
#define Detector_Ref_In_INP_DIS                (* (reg8 *) Detector_Ref_In__INP_DIS)
/* LCD Common or Segment Drive */
#define Detector_Ref_In_LCD_COM_SEG            (* (reg8 *) Detector_Ref_In__LCD_COM_SEG)
/* Enable Segment LCD */
#define Detector_Ref_In_LCD_EN                 (* (reg8 *) Detector_Ref_In__LCD_EN)
/* Slew Rate Control */
#define Detector_Ref_In_SLW                    (* (reg8 *) Detector_Ref_In__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Detector_Ref_In_PRTDSI__CAPS_SEL       (* (reg8 *) Detector_Ref_In__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Detector_Ref_In_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Detector_Ref_In__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Detector_Ref_In_PRTDSI__OE_SEL0        (* (reg8 *) Detector_Ref_In__PRTDSI__OE_SEL0) 
#define Detector_Ref_In_PRTDSI__OE_SEL1        (* (reg8 *) Detector_Ref_In__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Detector_Ref_In_PRTDSI__OUT_SEL0       (* (reg8 *) Detector_Ref_In__PRTDSI__OUT_SEL0) 
#define Detector_Ref_In_PRTDSI__OUT_SEL1       (* (reg8 *) Detector_Ref_In__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Detector_Ref_In_PRTDSI__SYNC_OUT       (* (reg8 *) Detector_Ref_In__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(Detector_Ref_In__SIO_CFG)
    #define Detector_Ref_In_SIO_HYST_EN        (* (reg8 *) Detector_Ref_In__SIO_HYST_EN)
    #define Detector_Ref_In_SIO_REG_HIFREQ     (* (reg8 *) Detector_Ref_In__SIO_REG_HIFREQ)
    #define Detector_Ref_In_SIO_CFG            (* (reg8 *) Detector_Ref_In__SIO_CFG)
    #define Detector_Ref_In_SIO_DIFF           (* (reg8 *) Detector_Ref_In__SIO_DIFF)
#endif /* (Detector_Ref_In__SIO_CFG) */

/* Interrupt Registers */
#if defined(Detector_Ref_In__INTSTAT)
    #define Detector_Ref_In_INTSTAT            (* (reg8 *) Detector_Ref_In__INTSTAT)
    #define Detector_Ref_In_SNAP               (* (reg8 *) Detector_Ref_In__SNAP)
    
	#define Detector_Ref_In_0_INTTYPE_REG 		(* (reg8 *) Detector_Ref_In__0__INTTYPE)
#endif /* (Detector_Ref_In__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Detector_Ref_In_H */


/* [] END OF FILE */
