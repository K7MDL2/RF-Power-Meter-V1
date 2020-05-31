/*******************************************************************************
* File Name: .h
* Version 3.50
*
* Description:
*  This file provides private constants and parameter values for the I2C
*  component.
*
* Note:
*
********************************************************************************
* Copyright 2012-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_PVT_I22OLED_H)
#define CY_I2C_PVT_I22OLED_H

#include "I22OLED.h"

#define I22OLED_TIMEOUT_ENABLED_INC    (0u)
#if (0u != I22OLED_TIMEOUT_ENABLED_INC)
    #include "I22OLED_TMOUT.h"
#endif /* (0u != I22OLED_TIMEOUT_ENABLED_INC) */


/**********************************
*   Variables with external linkage
**********************************/

extern I22OLED_BACKUP_STRUCT I22OLED_backup;

extern volatile uint8 I22OLED_state;   /* Current state of I2C FSM */

/* Master variables */
#if (I22OLED_MODE_MASTER_ENABLED)
    extern volatile uint8 I22OLED_mstrStatus;   /* Master Status byte  */
    extern volatile uint8 I22OLED_mstrControl;  /* Master Control byte */

    /* Transmit buffer variables */
    extern volatile uint8 * I22OLED_mstrRdBufPtr;   /* Pointer to Master Read buffer */
    extern volatile uint8   I22OLED_mstrRdBufSize;  /* Master Read buffer size       */
    extern volatile uint8   I22OLED_mstrRdBufIndex; /* Master Read buffer Index      */

    /* Receive buffer variables */
    extern volatile uint8 * I22OLED_mstrWrBufPtr;   /* Pointer to Master Write buffer */
    extern volatile uint8   I22OLED_mstrWrBufSize;  /* Master Write buffer size       */
    extern volatile uint8   I22OLED_mstrWrBufIndex; /* Master Write buffer Index      */

#endif /* (I22OLED_MODE_MASTER_ENABLED) */

/* Slave variables */
#if (I22OLED_MODE_SLAVE_ENABLED)
    extern volatile uint8 I22OLED_slStatus;         /* Slave Status  */

    /* Transmit buffer variables */
    extern volatile uint8 * I22OLED_slRdBufPtr;     /* Pointer to Transmit buffer  */
    extern volatile uint8   I22OLED_slRdBufSize;    /* Slave Transmit buffer size  */
    extern volatile uint8   I22OLED_slRdBufIndex;   /* Slave Transmit buffer Index */

    /* Receive buffer variables */
    extern volatile uint8 * I22OLED_slWrBufPtr;     /* Pointer to Receive buffer  */
    extern volatile uint8   I22OLED_slWrBufSize;    /* Slave Receive buffer size  */
    extern volatile uint8   I22OLED_slWrBufIndex;   /* Slave Receive buffer Index */

    #if (I22OLED_SW_ADRR_DECODE)
        extern volatile uint8 I22OLED_slAddress;     /* Software address variable */
    #endif   /* (I22OLED_SW_ADRR_DECODE) */

#endif /* (I22OLED_MODE_SLAVE_ENABLED) */

#if ((I22OLED_FF_IMPLEMENTED) && (I22OLED_WAKEUP_ENABLED))
    extern volatile uint8 I22OLED_wakeupSource;
#endif /* ((I22OLED_FF_IMPLEMENTED) && (I22OLED_WAKEUP_ENABLED)) */


#endif /* CY_I2C_PVT_I22OLED_H */


/* [] END OF FILE */
