/*******************************************************************************
* File Name: AMux_RF_Power.h
* Version 1.80
*
*  Description:
*    This file contains the constants and function prototypes for the Analog
*    Multiplexer User Module AMux.
*
*   Note:
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_AMUX_AMux_RF_Power_H)
#define CY_AMUX_AMux_RF_Power_H

#include "cyfitter.h"
#include "cyfitter_cfg.h"

#if ((CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3) || \
         (CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC4) || \
         (CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5))    
    #include "cytypes.h"
#else
    #include "syslib/cy_syslib.h"
#endif /* ((CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3) */


/***************************************
*        Function Prototypes
***************************************/

void AMux_RF_Power_Start(void) ;
#define AMux_RF_Power_Init() AMux_RF_Power_Start()
void AMux_RF_Power_FastSelect(uint8 channel) ;
/* The Stop, Select, Connect, Disconnect and DisconnectAll functions are declared elsewhere */
/* void AMux_RF_Power_Stop(void); */
/* void AMux_RF_Power_Select(uint8 channel); */
/* void AMux_RF_Power_Connect(uint8 channel); */
/* void AMux_RF_Power_Disconnect(uint8 channel); */
/* void AMux_RF_Power_DisconnectAll(void) */


/***************************************
*         Parameter Constants
***************************************/

#define AMux_RF_Power_CHANNELS  2u
#define AMux_RF_Power_MUXTYPE   1
#define AMux_RF_Power_ATMOSTONE 1

/***************************************
*             API Constants
***************************************/

#define AMux_RF_Power_NULL_CHANNEL 0xFFu
#define AMux_RF_Power_MUX_SINGLE   1
#define AMux_RF_Power_MUX_DIFF     2


/***************************************
*        Conditional Functions
***************************************/

#if AMux_RF_Power_MUXTYPE == AMux_RF_Power_MUX_SINGLE
# if !AMux_RF_Power_ATMOSTONE
#  define AMux_RF_Power_Connect(channel) AMux_RF_Power_Set(channel)
# endif
# define AMux_RF_Power_Disconnect(channel) AMux_RF_Power_Unset(channel)
#else
# if !AMux_RF_Power_ATMOSTONE
void AMux_RF_Power_Connect(uint8 channel) ;
# endif
void AMux_RF_Power_Disconnect(uint8 channel) ;
#endif

#if AMux_RF_Power_ATMOSTONE
# define AMux_RF_Power_Stop() AMux_RF_Power_DisconnectAll()
# define AMux_RF_Power_Select(channel) AMux_RF_Power_FastSelect(channel)
void AMux_RF_Power_DisconnectAll(void) ;
#else
# define AMux_RF_Power_Stop() AMux_RF_Power_Start()
void AMux_RF_Power_Select(uint8 channel) ;
# define AMux_RF_Power_DisconnectAll() AMux_RF_Power_Start()
#endif

#endif /* CY_AMUX_AMux_RF_Power_H */


/* [] END OF FILE */
