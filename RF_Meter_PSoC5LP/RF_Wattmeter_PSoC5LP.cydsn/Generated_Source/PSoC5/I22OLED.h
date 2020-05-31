/*******************************************************************************
* File Name: I22OLED.h
* Version 3.50
*
* Description:
*  This file provides constants and parameter values for the I2C component.

*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_I22OLED_H)
#define CY_I2C_I22OLED_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h"

/* Check if required defines such as CY_PSOC5LP are available in cy_boot */
#if !defined (CY_PSOC5LP)
    #error Component I2C_v3_50 requires cy_boot v3.10 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define I22OLED_IMPLEMENTATION     (1u)
#define I22OLED_MODE               (2u)
#define I22OLED_ENABLE_WAKEUP      (0u)
#define I22OLED_ADDR_DECODE        (1u)
#define I22OLED_UDB_INTRN_CLOCK    (0u)


/* I2C implementation enum */
#define I22OLED_UDB    (0x00u)
#define I22OLED_FF     (0x01u)

#define I22OLED_FF_IMPLEMENTED     (I22OLED_FF  == I22OLED_IMPLEMENTATION)
#define I22OLED_UDB_IMPLEMENTED    (I22OLED_UDB == I22OLED_IMPLEMENTATION)

#define I22OLED_UDB_INTRN_CLOCK_ENABLED    (I22OLED_UDB_IMPLEMENTED && \
                                                     (0u != I22OLED_UDB_INTRN_CLOCK))
/* I2C modes enum */
#define I22OLED_MODE_SLAVE                 (0x01u)
#define I22OLED_MODE_MASTER                (0x02u)
#define I22OLED_MODE_MULTI_MASTER          (0x06u)
#define I22OLED_MODE_MULTI_MASTER_SLAVE    (0x07u)
#define I22OLED_MODE_MULTI_MASTER_MASK     (0x04u)

#define I22OLED_MODE_SLAVE_ENABLED         (0u != (I22OLED_MODE_SLAVE  & I22OLED_MODE))
#define I22OLED_MODE_MASTER_ENABLED        (0u != (I22OLED_MODE_MASTER & I22OLED_MODE))
#define I22OLED_MODE_MULTI_MASTER_ENABLED  (0u != (I22OLED_MODE_MULTI_MASTER_MASK & \
                                                            I22OLED_MODE))
#define I22OLED_MODE_MULTI_MASTER_SLAVE_ENABLED    (I22OLED_MODE_MULTI_MASTER_SLAVE == \
                                                             I22OLED_MODE)

/* Address detection enum */
#define I22OLED_SW_DECODE      (0x00u)
#define I22OLED_HW_DECODE      (0x01u)

#define I22OLED_SW_ADRR_DECODE             (I22OLED_SW_DECODE == I22OLED_ADDR_DECODE)
#define I22OLED_HW_ADRR_DECODE             (I22OLED_HW_DECODE == I22OLED_ADDR_DECODE)

/* Wakeup enabled */
#define I22OLED_WAKEUP_ENABLED             (0u != I22OLED_ENABLE_WAKEUP)

/* Adds bootloader APIs to component */
#define I22OLED_BOOTLOADER_INTERFACE_ENABLED   (I22OLED_MODE_SLAVE_ENABLED && \
                                                            ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I22OLED) || \
                                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)))

/* Timeout functionality */
#define I22OLED_TIMEOUT_ENABLE             (0u)
#define I22OLED_TIMEOUT_SCL_TMOUT_ENABLE   (0u)
#define I22OLED_TIMEOUT_SDA_TMOUT_ENABLE   (0u)
#define I22OLED_TIMEOUT_PRESCALER_ENABLE   (0u)
#define I22OLED_TIMEOUT_IMPLEMENTATION     (0u)

/* Convert to boolean */
#define I22OLED_TIMEOUT_ENABLED            (0u != I22OLED_TIMEOUT_ENABLE)
#define I22OLED_TIMEOUT_SCL_TMOUT_ENABLED  (0u != I22OLED_TIMEOUT_SCL_TMOUT_ENABLE)
#define I22OLED_TIMEOUT_SDA_TMOUT_ENABLED  (0u != I22OLED_TIMEOUT_SDA_TMOUT_ENABLE)
#define I22OLED_TIMEOUT_PRESCALER_ENABLED  (0u != I22OLED_TIMEOUT_PRESCALER_ENABLE)

/* Timeout implementation enum. */
#define I22OLED_TIMEOUT_UDB    (0x00u)
#define I22OLED_TIMEOUT_FF     (0x01u)

#define I22OLED_TIMEOUT_FF_IMPLEMENTED     (I22OLED_TIMEOUT_FF  == \
                                                        I22OLED_TIMEOUT_IMPLEMENTATION)
#define I22OLED_TIMEOUT_UDB_IMPLEMENTED    (I22OLED_TIMEOUT_UDB == \
                                                        I22OLED_TIMEOUT_IMPLEMENTATION)

#define I22OLED_TIMEOUT_FF_ENABLED         (I22OLED_TIMEOUT_ENABLED && \
                                                     I22OLED_TIMEOUT_FF_IMPLEMENTED)

#define I22OLED_TIMEOUT_UDB_ENABLED        (I22OLED_TIMEOUT_ENABLED && \
                                                     I22OLED_TIMEOUT_UDB_IMPLEMENTED)

#define I22OLED_EXTERN_I2C_INTR_HANDLER    (0u)
#define I22OLED_EXTERN_TMOUT_INTR_HANDLER  (0u)

#define I22OLED_INTERN_I2C_INTR_HANDLER    (0u == I22OLED_EXTERN_I2C_INTR_HANDLER)
#define I22OLED_INTERN_TMOUT_INTR_HANDLER  (0u == I22OLED_EXTERN_TMOUT_INTR_HANDLER)


/***************************************
*       Type defines
***************************************/

/* Structure to save registers before go to sleep */
typedef struct
{
    uint8 enableState;

#if (I22OLED_FF_IMPLEMENTED)
    uint8 xcfg;
    uint8 cfg;
    uint8 addr;
    uint8 clkDiv1;
    uint8 clkDiv2;
#else
    uint8 control;
#endif /* (I22OLED_FF_IMPLEMENTED) */

#if (I22OLED_TIMEOUT_ENABLED)
    uint16 tmoutCfg;
    uint8  tmoutIntr;
#endif /* (I22OLED_TIMEOUT_ENABLED) */

} I22OLED_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void I22OLED_Init(void)                            ;
void I22OLED_Enable(void)                          ;

void I22OLED_Start(void)                           ;
void I22OLED_Stop(void)                            ;

#define I22OLED_EnableInt()        CyIntEnable      (I22OLED_ISR_NUMBER)
#define I22OLED_DisableInt()       CyIntDisable     (I22OLED_ISR_NUMBER)
#define I22OLED_ClearPendingInt()  CyIntClearPending(I22OLED_ISR_NUMBER)
#define I22OLED_SetPendingInt()    CyIntSetPending  (I22OLED_ISR_NUMBER)

void I22OLED_SaveConfig(void)                      ;
void I22OLED_Sleep(void)                           ;
void I22OLED_RestoreConfig(void)                   ;
void I22OLED_Wakeup(void)                          ;

/* I2C Master functions prototypes */
#if (I22OLED_MODE_MASTER_ENABLED)
    /* Read and Clear status functions */
    uint8 I22OLED_MasterStatus(void)                ;
    uint8 I22OLED_MasterClearStatus(void)           ;

    /* Interrupt based operation functions */
    uint8 I22OLED_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 I22OLED_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 I22OLED_MasterGetReadBufSize(void)       ;
    uint8 I22OLED_MasterGetWriteBufSize(void)      ;
    void  I22OLED_MasterClearReadBuf(void)         ;
    void  I22OLED_MasterClearWriteBuf(void)        ;

    /* Manual operation functions */
    uint8 I22OLED_MasterSendStart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 I22OLED_MasterSendRestart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 I22OLED_MasterSendStop(void)             ;
    uint8 I22OLED_MasterWriteByte(uint8 theByte)   ;
    uint8 I22OLED_MasterReadByte(uint8 acknNak)    ;

#endif /* (I22OLED_MODE_MASTER_ENABLED) */

/* I2C Slave functions prototypes */
#if (I22OLED_MODE_SLAVE_ENABLED)
    /* Read and Clear status functions */
    uint8 I22OLED_SlaveStatus(void)                ;
    uint8 I22OLED_SlaveClearReadStatus(void)       ;
    uint8 I22OLED_SlaveClearWriteStatus(void)      ;

    void  I22OLED_SlaveSetAddress(uint8 address)   ;

    /* Interrupt based operation functions */
    void  I22OLED_SlaveInitReadBuf(uint8 * rdBuf, uint8 bufSize) \
                                                            ;
    void  I22OLED_SlaveInitWriteBuf(uint8 * wrBuf, uint8 bufSize) \
                                                            ;
    uint8 I22OLED_SlaveGetReadBufSize(void)        ;
    uint8 I22OLED_SlaveGetWriteBufSize(void)       ;
    void  I22OLED_SlaveClearReadBuf(void)          ;
    void  I22OLED_SlaveClearWriteBuf(void)         ;

    /* Communication bootloader I2C Slave APIs */
    #if defined(CYDEV_BOOTLOADER_IO_COMP) && (I22OLED_BOOTLOADER_INTERFACE_ENABLED)
        /* Physical layer functions */
        void     I22OLED_CyBtldrCommStart(void) CYSMALL \
                                                            ;
        void     I22OLED_CyBtldrCommStop(void)  CYSMALL \
                                                            ;
        void     I22OLED_CyBtldrCommReset(void) CYSMALL \
                                                            ;
        cystatus I22OLED_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) \
                                                        CYSMALL ;
        cystatus I22OLED_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)  CYSMALL \
                                                            ;

        #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I22OLED)
            #define CyBtldrCommStart    I22OLED_CyBtldrCommStart
            #define CyBtldrCommStop     I22OLED_CyBtldrCommStop
            #define CyBtldrCommReset    I22OLED_CyBtldrCommReset
            #define CyBtldrCommWrite    I22OLED_CyBtldrCommWrite
            #define CyBtldrCommRead     I22OLED_CyBtldrCommRead
        #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I22OLED) */

        /* Size of Read/Write buffers for I2C bootloader  */
        #define I22OLED_BTLDR_SIZEOF_READ_BUFFER   (0x80u)
        #define I22OLED_BTLDR_SIZEOF_WRITE_BUFFER  (0x80u)
        #define I22OLED_MIN_UNT16(a, b)            ( ((uint16)(a) < (b)) ? ((uint16) (a)) : ((uint16) (b)) )
        #define I22OLED_WAIT_1_MS                  (1u)

    #endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (I22OLED_BOOTLOADER_INTERFACE_ENABLED) */

#endif /* (I22OLED_MODE_SLAVE_ENABLED) */

/* Component interrupt handlers */
CY_ISR_PROTO(I22OLED_ISR);
#if ((I22OLED_FF_IMPLEMENTED) || (I22OLED_WAKEUP_ENABLED))
    CY_ISR_PROTO(I22OLED_WAKEUP_ISR);
#endif /* ((I22OLED_FF_IMPLEMENTED) || (I22OLED_WAKEUP_ENABLED)) */


/**********************************
*   Variable with external linkage
**********************************/

extern uint8 I22OLED_initVar;


/***************************************
*   Initial Parameter Constants
***************************************/

#define I22OLED_DATA_RATE          (100u)
#define I22OLED_DEFAULT_ADDR       (8u)
#define I22OLED_I2C_PAIR_SELECTED  (0u)

/* I2C pair enum */
#define I22OLED_I2C_PAIR_ANY   (0x01u) /* Any pins for I2C */
#define I22OLED_I2C_PAIR0      (0x01u) /* I2C0: (SCL = P12[4]) && (SCL = P12[5]) */
#define I22OLED_I2C_PAIR1      (0x02u) /* I2C1: (SCL = P12[0]) && (SDA = P12[1]) */

#define I22OLED_I2C1_SIO_PAIR  (I22OLED_I2C_PAIR1 == I22OLED_I2C_PAIR_SELECTED)
#define I22OLED_I2C0_SIO_PAIR  (I22OLED_I2C_PAIR0 == I22OLED_I2C_PAIR_SELECTED)


/***************************************
*            API Constants
***************************************/

/* Master/Slave control constants */
#define I22OLED_READ_XFER_MODE     (0x01u) /* Read */
#define I22OLED_WRITE_XFER_MODE    (0x00u) /* Write */
#define I22OLED_ACK_DATA           (0x01u) /* Send ACK */
#define I22OLED_NAK_DATA           (0x00u) /* Send NAK */
#define I22OLED_OVERFLOW_RETURN    (0xFFu) /* Send on bus in case of overflow */

#if (I22OLED_MODE_MASTER_ENABLED)
    /* "Mode" constants for MasterWriteBuf() or MasterReadBuf() function */
    #define I22OLED_MODE_COMPLETE_XFER     (0x00u) /* Full transfer with Start and Stop */
    #define I22OLED_MODE_REPEAT_START      (0x01u) /* Begin with a ReStart instead of a Start */
    #define I22OLED_MODE_NO_STOP           (0x02u) /* Complete the transfer without a Stop */

    /* Master status */
    #define I22OLED_MSTAT_CLEAR            (0x00u) /* Clear (initialize) status value */

    #define I22OLED_MSTAT_RD_CMPLT         (0x01u) /* Read complete */
    #define I22OLED_MSTAT_WR_CMPLT         (0x02u) /* Write complete */
    #define I22OLED_MSTAT_XFER_INP         (0x04u) /* Master transfer in progress */
    #define I22OLED_MSTAT_XFER_HALT        (0x08u) /* Transfer is halted */

    #define I22OLED_MSTAT_ERR_MASK         (0xF0u) /* Mask for all errors */
    #define I22OLED_MSTAT_ERR_SHORT_XFER   (0x10u) /* Master NAKed before end of packet */
    #define I22OLED_MSTAT_ERR_ADDR_NAK     (0x20u) /* Slave did not ACK */
    #define I22OLED_MSTAT_ERR_ARB_LOST     (0x40u) /* Master lost arbitration during communication */
    #define I22OLED_MSTAT_ERR_XFER         (0x80u) /* Error during transfer */

    /* Master API returns */
    #define I22OLED_MSTR_NO_ERROR          (0x00u) /* Function complete without error */
    #define I22OLED_MSTR_BUS_BUSY          (0x01u) /* Bus is busy, process not started */
    #define I22OLED_MSTR_NOT_READY         (0x02u) /* Master not Master on the bus or */
                                                            /*  Slave operation in progress */
    #define I22OLED_MSTR_ERR_LB_NAK        (0x03u) /* Last Byte Naked */
    #define I22OLED_MSTR_ERR_ARB_LOST      (0x04u) /* Master lost arbitration during communication */
    #define I22OLED_MSTR_ERR_ABORT_START_GEN  (0x05u) /* Master did not generate Start, the Slave */
                                                               /* was addressed before */

#endif /* (I22OLED_MODE_MASTER_ENABLED) */

#if (I22OLED_MODE_SLAVE_ENABLED)
    /* Slave Status Constants */
    #define I22OLED_SSTAT_RD_CMPLT     (0x01u) /* Read transfer complete */
    #define I22OLED_SSTAT_RD_BUSY      (0x02u) /* Read transfer in progress */
    #define I22OLED_SSTAT_RD_ERR_OVFL  (0x04u) /* Read overflow Error */
    #define I22OLED_SSTAT_RD_MASK      (0x0Fu) /* Read Status Mask */
    #define I22OLED_SSTAT_RD_NO_ERR    (0x00u) /* Read no Error */

    #define I22OLED_SSTAT_WR_CMPLT     (0x10u) /* Write transfer complete */
    #define I22OLED_SSTAT_WR_BUSY      (0x20u) /* Write transfer in progress */
    #define I22OLED_SSTAT_WR_ERR_OVFL  (0x40u) /* Write overflow Error */
    #define I22OLED_SSTAT_WR_MASK      (0xF0u) /* Write Status Mask  */
    #define I22OLED_SSTAT_WR_NO_ERR    (0x00u) /* Write no Error */

    #define I22OLED_SSTAT_RD_CLEAR     (0x0Du) /* Read Status clear */
    #define I22OLED_SSTAT_WR_CLEAR     (0xD0u) /* Write Status Clear */

#endif /* (I22OLED_MODE_SLAVE_ENABLED) */


/***************************************
*       I2C state machine constants
***************************************/

/* Default slave address states */
#define  I22OLED_SM_IDLE           (0x10u) /* Default state - IDLE */
#define  I22OLED_SM_EXIT_IDLE      (0x00u) /* Pass master and slave processing and go to IDLE */

/* Slave mode states */
#define  I22OLED_SM_SLAVE          (I22OLED_SM_IDLE) /* Any Slave state */
#define  I22OLED_SM_SL_WR_DATA     (0x11u) /* Master writes data to slave  */
#define  I22OLED_SM_SL_RD_DATA     (0x12u) /* Master reads data from slave */

/* Master mode states */
#define  I22OLED_SM_MASTER         (0x40u) /* Any master state */

#define  I22OLED_SM_MSTR_RD        (0x08u) /* Any master read state          */
#define  I22OLED_SM_MSTR_RD_ADDR   (0x49u) /* Master sends address with read */
#define  I22OLED_SM_MSTR_RD_DATA   (0x4Au) /* Master reads data              */

#define  I22OLED_SM_MSTR_WR        (0x04u) /* Any master read state           */
#define  I22OLED_SM_MSTR_WR_ADDR   (0x45u) /* Master sends address with write */
#define  I22OLED_SM_MSTR_WR_DATA   (0x46u) /* Master writes data              */

#define  I22OLED_SM_MSTR_HALT      (0x60u) /* Master waits for ReStart */

#define I22OLED_CHECK_SM_MASTER    (0u != (I22OLED_SM_MASTER & I22OLED_state))
#define I22OLED_CHECK_SM_SLAVE     (0u != (I22OLED_SM_SLAVE  & I22OLED_state))


/***************************************
*              Registers
***************************************/

#if (I22OLED_FF_IMPLEMENTED)
    /* Fixed Function registers */
    #define I22OLED_XCFG_REG           (*(reg8 *) I22OLED_I2C_FF__XCFG)
    #define I22OLED_XCFG_PTR           ( (reg8 *) I22OLED_I2C_FF__XCFG)

    #define I22OLED_ADDR_REG           (*(reg8 *) I22OLED_I2C_FF__ADR)
    #define I22OLED_ADDR_PTR           ( (reg8 *) I22OLED_I2C_FF__ADR)

    #define I22OLED_CFG_REG            (*(reg8 *) I22OLED_I2C_FF__CFG)
    #define I22OLED_CFG_PTR            ( (reg8 *) I22OLED_I2C_FF__CFG)

    #define I22OLED_CSR_REG            (*(reg8 *) I22OLED_I2C_FF__CSR)
    #define I22OLED_CSR_PTR            ( (reg8 *) I22OLED_I2C_FF__CSR)

    #define I22OLED_DATA_REG           (*(reg8 *) I22OLED_I2C_FF__D)
    #define I22OLED_DATA_PTR           ( (reg8 *) I22OLED_I2C_FF__D)

    #define I22OLED_MCSR_REG           (*(reg8 *) I22OLED_I2C_FF__MCSR)
    #define I22OLED_MCSR_PTR           ( (reg8 *) I22OLED_I2C_FF__MCSR)

    #define I22OLED_ACT_PWRMGR_REG     (*(reg8 *) I22OLED_I2C_FF__PM_ACT_CFG)
    #define I22OLED_ACT_PWRMGR_PTR     ( (reg8 *) I22OLED_I2C_FF__PM_ACT_CFG)
    #define I22OLED_ACT_PWR_EN         ( (uint8)  I22OLED_I2C_FF__PM_ACT_MSK)

    #define I22OLED_STBY_PWRMGR_REG    (*(reg8 *) I22OLED_I2C_FF__PM_STBY_CFG)
    #define I22OLED_STBY_PWRMGR_PTR    ( (reg8 *) I22OLED_I2C_FF__PM_STBY_CFG)
    #define I22OLED_STBY_PWR_EN        ( (uint8)  I22OLED_I2C_FF__PM_STBY_MSK)

    #define I22OLED_PWRSYS_CR1_REG     (*(reg8 *) CYREG_PWRSYS_CR1)
    #define I22OLED_PWRSYS_CR1_PTR     ( (reg8 *) CYREG_PWRSYS_CR1)

    #define I22OLED_CLKDIV1_REG    (*(reg8 *) I22OLED_I2C_FF__CLK_DIV1)
    #define I22OLED_CLKDIV1_PTR    ( (reg8 *) I22OLED_I2C_FF__CLK_DIV1)

    #define I22OLED_CLKDIV2_REG    (*(reg8 *) I22OLED_I2C_FF__CLK_DIV2)
    #define I22OLED_CLKDIV2_PTR    ( (reg8 *) I22OLED_I2C_FF__CLK_DIV2)

#else
    /* UDB implementation registers */
    #define I22OLED_CFG_REG \
            (*(reg8 *) I22OLED_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)
    #define I22OLED_CFG_PTR \
            ( (reg8 *) I22OLED_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)

    #define I22OLED_CSR_REG        (*(reg8 *) I22OLED_bI2C_UDB_StsReg__STATUS_REG)
    #define I22OLED_CSR_PTR        ( (reg8 *) I22OLED_bI2C_UDB_StsReg__STATUS_REG)

    #define I22OLED_INT_MASK_REG   (*(reg8 *) I22OLED_bI2C_UDB_StsReg__MASK_REG)
    #define I22OLED_INT_MASK_PTR   ( (reg8 *) I22OLED_bI2C_UDB_StsReg__MASK_REG)

    #define I22OLED_INT_ENABLE_REG (*(reg8 *) I22OLED_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)
    #define I22OLED_INT_ENABLE_PTR ( (reg8 *) I22OLED_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)

    #define I22OLED_DATA_REG       (*(reg8 *) I22OLED_bI2C_UDB_Shifter_u0__A0_REG)
    #define I22OLED_DATA_PTR       ( (reg8 *) I22OLED_bI2C_UDB_Shifter_u0__A0_REG)

    #define I22OLED_GO_REG         (*(reg8 *) I22OLED_bI2C_UDB_Shifter_u0__F1_REG)
    #define I22OLED_GO_PTR         ( (reg8 *) I22OLED_bI2C_UDB_Shifter_u0__F1_REG)

    #define I22OLED_GO_DONE_REG    (*(reg8 *) I22OLED_bI2C_UDB_Shifter_u0__A1_REG)
    #define I22OLED_GO_DONE_PTR    ( (reg8 *) I22OLED_bI2C_UDB_Shifter_u0__A1_REG)

    #define I22OLED_MCLK_PRD_REG   (*(reg8 *) I22OLED_bI2C_UDB_Master_ClkGen_u0__D0_REG)
    #define I22OLED_MCLK_PRD_PTR   ( (reg8 *) I22OLED_bI2C_UDB_Master_ClkGen_u0__D0_REG)

    #define I22OLED_MCLK_CMP_REG   (*(reg8 *) I22OLED_bI2C_UDB_Master_ClkGen_u0__D1_REG)
    #define I22OLED_MCLK_CMP_PTR   ( (reg8 *) I22OLED_bI2C_UDB_Master_ClkGen_u0__D1_REG)

    #if (I22OLED_MODE_SLAVE_ENABLED)
        #define I22OLED_ADDR_REG       (*(reg8 *) I22OLED_bI2C_UDB_Shifter_u0__D0_REG)
        #define I22OLED_ADDR_PTR       ( (reg8 *) I22OLED_bI2C_UDB_Shifter_u0__D0_REG)

        #define I22OLED_PERIOD_REG     (*(reg8 *) I22OLED_bI2C_UDB_Slave_BitCounter__PERIOD_REG)
        #define I22OLED_PERIOD_PTR     ( (reg8 *) I22OLED_bI2C_UDB_Slave_BitCounter__PERIOD_REG)

        #define I22OLED_COUNTER_REG    (*(reg8 *) I22OLED_bI2C_UDB_Slave_BitCounter__COUNT_REG)
        #define I22OLED_COUNTER_PTR    ( (reg8 *) I22OLED_bI2C_UDB_Slave_BitCounter__COUNT_REG)

        #define I22OLED_COUNTER_AUX_CTL_REG \
                                    (*(reg8 *) I22OLED_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)
        #define I22OLED_COUNTER_AUX_CTL_PTR \
                                    ( (reg8 *) I22OLED_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)

    #endif /* (I22OLED_MODE_SLAVE_ENABLED) */

#endif /* (I22OLED_FF_IMPLEMENTED) */


/***************************************
*        Registers Constants
***************************************/

/* I22OLED_I2C_IRQ */
#define I22OLED_ISR_NUMBER     ((uint8) I22OLED_I2C_IRQ__INTC_NUMBER)
#define I22OLED_ISR_PRIORITY   ((uint8) I22OLED_I2C_IRQ__INTC_PRIOR_NUM)

/* I2C Slave Data Register */
#define I22OLED_SLAVE_ADDR_MASK    (0x7Fu)
#define I22OLED_SLAVE_ADDR_SHIFT   (0x01u)
#define I22OLED_DATA_MASK          (0xFFu)
#define I22OLED_READ_FLAG          (0x01u)

/* Block reset constants */
#define I22OLED_CLEAR_REG          (0x00u)
#define I22OLED_BLOCK_RESET_DELAY  (2u)
#define I22OLED_FF_RESET_DELAY     (I22OLED_BLOCK_RESET_DELAY)
#define I22OLED_RESTORE_TIMEOUT    (255u)

#if (I22OLED_FF_IMPLEMENTED)
    /* XCFG I2C Extended Configuration Register */
    #define I22OLED_XCFG_CLK_EN        (0x80u) /* Enable gated clock to block */
    #define I22OLED_XCFG_I2C_ON        (0x40u) /* Enable I2C as wake up source*/
    #define I22OLED_XCFG_RDY_TO_SLEEP  (0x20u) /* I2C ready go to sleep */
    #define I22OLED_XCFG_FORCE_NACK    (0x10u) /* Force NACK all incoming transactions */
    #define I22OLED_XCFG_NO_BC_INT     (0x08u) /* No interrupt on byte complete */
    #define I22OLED_XCFG_BUF_MODE      (0x02u) /* Enable buffer mode */
    #define I22OLED_XCFG_HDWR_ADDR_EN  (0x01u) /* Enable Hardware address match */

    /* CFG I2C Configuration Register */
    #define I22OLED_CFG_SIO_SELECT     (0x80u) /* Pin Select for SCL/SDA lines */
    #define I22OLED_CFG_PSELECT        (0x40u) /* Pin Select */
    #define I22OLED_CFG_BUS_ERR_IE     (0x20u) /* Bus Error Interrupt Enable */
    #define I22OLED_CFG_STOP_IE        (0x10u) /* Enable Interrupt on STOP condition */
    #define I22OLED_CFG_CLK_RATE_MSK   (0x0Cu) /* Clock rate select */
    #define I22OLED_CFG_CLK_RATE_100   (0x00u) /* Clock rate select 100K */
    #define I22OLED_CFG_CLK_RATE_400   (0x04u) /* Clock rate select 400K */
    #define I22OLED_CFG_CLK_RATE_050   (0x08u) /* Clock rate select 50K  */
    #define I22OLED_CFG_CLK_RATE_RSVD  (0x0Cu) /* Clock rate select Invalid */
    #define I22OLED_CFG_EN_MSTR        (0x02u) /* Enable Master operation */
    #define I22OLED_CFG_EN_SLAVE       (0x01u) /* Enable Slave operation */

    #define I22OLED_CFG_CLK_RATE_LESS_EQUAL_50 (0x04u) /* Clock rate select <= 50kHz */
    #define I22OLED_CFG_CLK_RATE_GRATER_50     (0x00u) /* Clock rate select > 50kHz */

    /* CSR I2C Control and Status Register */
    #define I22OLED_CSR_BUS_ERROR      (0x80u) /* Active high when bus error has occurred */
    #define I22OLED_CSR_LOST_ARB       (0x40u) /* Set to 1 if lost arbitration in host mode */
    #define I22OLED_CSR_STOP_STATUS    (0x20u) /* Set if Stop has been detected */
    #define I22OLED_CSR_ACK            (0x10u) /* ACK response */
    #define I22OLED_CSR_NAK            (0x00u) /* NAK response */
    #define I22OLED_CSR_ADDRESS        (0x08u) /* Set in firmware 0 = status bit, 1 Address is slave */
    #define I22OLED_CSR_TRANSMIT       (0x04u) /* Set in firmware 1 = transmit, 0 = receive */
    #define I22OLED_CSR_LRB            (0x02u) /* Last received bit */
    #define I22OLED_CSR_LRB_ACK        (0x00u) /* Last received bit was an ACK */
    #define I22OLED_CSR_LRB_NAK        (0x02u) /* Last received bit was an NAK */
    #define I22OLED_CSR_BYTE_COMPLETE  (0x01u) /* Informs that last byte has been sent */
    #define I22OLED_CSR_STOP_GEN       (0x00u) /* Generate a stop condition */
    #define I22OLED_CSR_RDY_TO_RD      (0x00u) /* Set to receive mode */

    /* MCSR I2C Master Control and Status Register */
    #define I22OLED_MCSR_STOP_GEN      (0x10u) /* Firmware sets this bit to initiate a Stop condition */
    #define I22OLED_MCSR_BUS_BUSY      (0x08u) /* Status bit, Set at Start and cleared at Stop condition */
    #define I22OLED_MCSR_MSTR_MODE     (0x04u) /* Status bit, Set at Start and cleared at Stop condition */
    #define I22OLED_MCSR_RESTART_GEN   (0x02u) /* Firmware sets this bit to initiate a ReStart condition */
    #define I22OLED_MCSR_START_GEN     (0x01u) /* Firmware sets this bit to initiate a Start condition */

    /* PWRSYS_CR1 to handle Sleep */
    #define I22OLED_PWRSYS_CR1_I2C_REG_BACKUP  (0x04u) /* Enables, power to I2C regs while sleep */

#else
    /* CONTROL REG bits location */
    #define I22OLED_CTRL_START_SHIFT           (7u)
    #define I22OLED_CTRL_STOP_SHIFT            (6u)
    #define I22OLED_CTRL_RESTART_SHIFT         (5u)
    #define I22OLED_CTRL_NACK_SHIFT            (4u)
    #define I22OLED_CTRL_ANY_ADDRESS_SHIFT     (3u)
    #define I22OLED_CTRL_TRANSMIT_SHIFT        (2u)
    #define I22OLED_CTRL_ENABLE_MASTER_SHIFT   (1u)
    #define I22OLED_CTRL_ENABLE_SLAVE_SHIFT    (0u)
    #define I22OLED_CTRL_START_MASK            ((uint8) (0x01u << I22OLED_CTRL_START_SHIFT))
    #define I22OLED_CTRL_STOP_MASK             ((uint8) (0x01u << I22OLED_CTRL_STOP_SHIFT))
    #define I22OLED_CTRL_RESTART_MASK          ((uint8) (0x01u << I22OLED_CTRL_RESTART_SHIFT))
    #define I22OLED_CTRL_NACK_MASK             ((uint8) (0x01u << I22OLED_CTRL_NACK_SHIFT))
    #define I22OLED_CTRL_ANY_ADDRESS_MASK      ((uint8) (0x01u << I22OLED_CTRL_ANY_ADDRESS_SHIFT))
    #define I22OLED_CTRL_TRANSMIT_MASK         ((uint8) (0x01u << I22OLED_CTRL_TRANSMIT_SHIFT))
    #define I22OLED_CTRL_ENABLE_MASTER_MASK    ((uint8) (0x01u << I22OLED_CTRL_ENABLE_MASTER_SHIFT))
    #define I22OLED_CTRL_ENABLE_SLAVE_MASK     ((uint8) (0x01u << I22OLED_CTRL_ENABLE_SLAVE_SHIFT))

    /* STATUS REG bits location */
    #define I22OLED_STS_LOST_ARB_SHIFT         (6u)
    #define I22OLED_STS_STOP_SHIFT             (5u)
    #define I22OLED_STS_BUSY_SHIFT             (4u)
    #define I22OLED_STS_ADDR_SHIFT             (3u)
    #define I22OLED_STS_MASTER_MODE_SHIFT      (2u)
    #define I22OLED_STS_LRB_SHIFT              (1u)
    #define I22OLED_STS_BYTE_COMPLETE_SHIFT    (0u)
    #define I22OLED_STS_LOST_ARB_MASK          ((uint8) (0x01u << I22OLED_STS_LOST_ARB_SHIFT))
    #define I22OLED_STS_STOP_MASK              ((uint8) (0x01u << I22OLED_STS_STOP_SHIFT))
    #define I22OLED_STS_BUSY_MASK              ((uint8) (0x01u << I22OLED_STS_BUSY_SHIFT))
    #define I22OLED_STS_ADDR_MASK              ((uint8) (0x01u << I22OLED_STS_ADDR_SHIFT))
    #define I22OLED_STS_MASTER_MODE_MASK       ((uint8) (0x01u << I22OLED_STS_MASTER_MODE_SHIFT))
    #define I22OLED_STS_LRB_MASK               ((uint8) (0x01u << I22OLED_STS_LRB_SHIFT))
    #define I22OLED_STS_BYTE_COMPLETE_MASK     ((uint8) (0x01u << I22OLED_STS_BYTE_COMPLETE_SHIFT))

    /* AUX_CTL bits definition */
    #define I22OLED_COUNTER_ENABLE_MASK        (0x20u) /* Enable 7-bit counter */
    #define I22OLED_INT_ENABLE_MASK            (0x10u) /* Enable interrupt from status register */
    #define I22OLED_CNT7_ENABLE                (I22OLED_COUNTER_ENABLE_MASK)
    #define I22OLED_INTR_ENABLE                (I22OLED_INT_ENABLE_MASK)

#endif /* (I22OLED_FF_IMPLEMENTED) */


/***************************************
*        Marco
***************************************/

/* ACK and NACK for data and address checks */
#define I22OLED_CHECK_ADDR_ACK(csr)    ((I22OLED_CSR_LRB_ACK | I22OLED_CSR_ADDRESS) == \
                                                 ((I22OLED_CSR_LRB    | I22OLED_CSR_ADDRESS) &  \
                                                  (csr)))


#define I22OLED_CHECK_ADDR_NAK(csr)    ((I22OLED_CSR_LRB_NAK | I22OLED_CSR_ADDRESS) == \
                                                 ((I22OLED_CSR_LRB    | I22OLED_CSR_ADDRESS) &  \
                                                  (csr)))

#define I22OLED_CHECK_DATA_ACK(csr)    (0u == ((csr) & I22OLED_CSR_LRB_NAK))

/* MCSR conditions check */
#define I22OLED_CHECK_BUS_FREE(mcsr)       (0u == ((mcsr) & I22OLED_MCSR_BUS_BUSY))
#define I22OLED_CHECK_MASTER_MODE(mcsr)    (0u != ((mcsr) & I22OLED_MCSR_MSTR_MODE))

/* CSR conditions check */
#define I22OLED_WAIT_BYTE_COMPLETE(csr)    (0u == ((csr) & I22OLED_CSR_BYTE_COMPLETE))
#define I22OLED_WAIT_STOP_COMPLETE(csr)    (0u == ((csr) & (I22OLED_CSR_BYTE_COMPLETE | \
                                                                     I22OLED_CSR_STOP_STATUS)))
#define I22OLED_CHECK_BYTE_COMPLETE(csr)   (0u != ((csr) & I22OLED_CSR_BYTE_COMPLETE))
#define I22OLED_CHECK_STOP_STS(csr)        (0u != ((csr) & I22OLED_CSR_STOP_STATUS))
#define I22OLED_CHECK_LOST_ARB(csr)        (0u != ((csr) & I22OLED_CSR_LOST_ARB))
#define I22OLED_CHECK_ADDRESS_STS(csr)     (0u != ((csr) & I22OLED_CSR_ADDRESS))

/* Software start and end of transaction check */
#define I22OLED_CHECK_RESTART(mstrCtrl)    (0u != ((mstrCtrl) & I22OLED_MODE_REPEAT_START))
#define I22OLED_CHECK_NO_STOP(mstrCtrl)    (0u != ((mstrCtrl) & I22OLED_MODE_NO_STOP))

/* Send read or write completion depends on state */
#define I22OLED_GET_MSTAT_CMPLT ((0u != (I22OLED_state & I22OLED_SM_MSTR_RD)) ? \
                                                 (I22OLED_MSTAT_RD_CMPLT) : (I22OLED_MSTAT_WR_CMPLT))

/* Returns 7-bit slave address */
#define I22OLED_GET_SLAVE_ADDR(dataReg)   (((dataReg) >> I22OLED_SLAVE_ADDR_SHIFT) & \
                                                                  I22OLED_SLAVE_ADDR_MASK)

#if (I22OLED_FF_IMPLEMENTED)
    /* Check enable of module */
    #define I22OLED_I2C_ENABLE_REG     (I22OLED_ACT_PWRMGR_REG)
    #define I22OLED_IS_I2C_ENABLE(reg) (0u != ((reg) & I22OLED_ACT_PWR_EN))
    #define I22OLED_IS_ENABLED         (0u != (I22OLED_ACT_PWRMGR_REG & I22OLED_ACT_PWR_EN))

    #define I22OLED_CHECK_PWRSYS_I2C_BACKUP    (0u != (I22OLED_PWRSYS_CR1_I2C_REG_BACKUP & \
                                                                I22OLED_PWRSYS_CR1_REG))

    /* Check start condition generation */
    #define I22OLED_CHECK_START_GEN(mcsr)  ((0u != ((mcsr) & I22OLED_MCSR_START_GEN)) && \
                                                     (0u == ((mcsr) & I22OLED_MCSR_MSTR_MODE)))

    #define I22OLED_CLEAR_START_GEN        do{ \
                                                        I22OLED_MCSR_REG &=                                   \
                                                                           ((uint8) ~I22OLED_MCSR_START_GEN); \
                                                    }while(0)

    /* Stop interrupt */
    #define I22OLED_ENABLE_INT_ON_STOP     do{ \
                                                        I22OLED_CFG_REG |= I22OLED_CFG_STOP_IE; \
                                                    }while(0)

    #define I22OLED_DISABLE_INT_ON_STOP    do{ \
                                                        I22OLED_CFG_REG &=                                 \
                                                                           ((uint8) ~I22OLED_CFG_STOP_IE); \
                                                    }while(0)

    /* Transmit data */
    #define I22OLED_TRANSMIT_DATA          do{ \
                                                        I22OLED_CSR_REG = I22OLED_CSR_TRANSMIT; \
                                                    }while(0)

    #define I22OLED_ACK_AND_TRANSMIT       do{ \
                                                        I22OLED_CSR_REG = (I22OLED_CSR_ACK |      \
                                                                                    I22OLED_CSR_TRANSMIT); \
                                                    }while(0)

    #define I22OLED_NAK_AND_TRANSMIT       do{ \
                                                        I22OLED_CSR_REG = I22OLED_CSR_NAK; \
                                                    }while(0)

    /* Special case: udb needs to ack, ff needs to nak */
    #define I22OLED_ACKNAK_AND_TRANSMIT    do{ \
                                                        I22OLED_CSR_REG  = (I22OLED_CSR_NAK |      \
                                                                                     I22OLED_CSR_TRANSMIT); \
                                                    }while(0)
    /* Receive data */
    #define I22OLED_ACK_AND_RECEIVE        do{ \
                                                        I22OLED_CSR_REG = I22OLED_CSR_ACK; \
                                                    }while(0)

    #define I22OLED_NAK_AND_RECEIVE        do{ \
                                                        I22OLED_CSR_REG = I22OLED_CSR_NAK; \
                                                    }while(0)

    #define I22OLED_READY_TO_READ          do{ \
                                                        I22OLED_CSR_REG = I22OLED_CSR_RDY_TO_RD; \
                                                    }while(0)

    /* Release bus after lost arbitration */
    #define I22OLED_BUS_RELEASE    I22OLED_READY_TO_READ

    /* Master Start/ReStart/Stop conditions generation */
    #define I22OLED_GENERATE_START         do{ \
                                                        I22OLED_MCSR_REG = I22OLED_MCSR_START_GEN; \
                                                    }while(0)

    #define I22OLED_GENERATE_RESTART \
                    do{                       \
                        I22OLED_MCSR_REG = (I22OLED_MCSR_RESTART_GEN | \
                                                     I22OLED_MCSR_STOP_GEN);    \
                        I22OLED_CSR_REG  = I22OLED_CSR_TRANSMIT;       \
                    }while(0)

    #define I22OLED_GENERATE_STOP \
                    do{                    \
                        I22OLED_MCSR_REG = I22OLED_MCSR_STOP_GEN; \
                        I22OLED_CSR_REG  = I22OLED_CSR_TRANSMIT;  \
                    }while(0)

    /* Master manual APIs compatible defines */
    #define I22OLED_GENERATE_START_MANUAL      I22OLED_GENERATE_START
    #define I22OLED_GENERATE_RESTART_MANUAL    I22OLED_GENERATE_RESTART
    #define I22OLED_GENERATE_STOP_MANUAL       I22OLED_GENERATE_STOP
    #define I22OLED_TRANSMIT_DATA_MANUAL       I22OLED_TRANSMIT_DATA
    #define I22OLED_READY_TO_READ_MANUAL       I22OLED_READY_TO_READ
    #define I22OLED_ACK_AND_RECEIVE_MANUAL     I22OLED_ACK_AND_RECEIVE
    #define I22OLED_BUS_RELEASE_MANUAL         I22OLED_BUS_RELEASE

#else

    /* Masks to enable interrupts from Status register */
    #define I22OLED_STOP_IE_MASK           (I22OLED_STS_STOP_MASK)
    #define I22OLED_BYTE_COMPLETE_IE_MASK  (I22OLED_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: CSR register bit-fields */
    #define I22OLED_CSR_LOST_ARB       (I22OLED_STS_LOST_ARB_MASK)
    #define I22OLED_CSR_STOP_STATUS    (I22OLED_STS_STOP_MASK)
    #define I22OLED_CSR_BUS_ERROR      (0x00u)
    #define I22OLED_CSR_ADDRESS        (I22OLED_STS_ADDR_MASK)
    #define I22OLED_CSR_TRANSMIT       (I22OLED_CTRL_TRANSMIT_MASK)
    #define I22OLED_CSR_LRB            (I22OLED_STS_LRB_MASK)
    #define I22OLED_CSR_LRB_NAK        (I22OLED_STS_LRB_MASK)
    #define I22OLED_CSR_LRB_ACK        (0x00u)
    #define I22OLED_CSR_BYTE_COMPLETE  (I22OLED_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: MCSR registers bit-fields */
    #define I22OLED_MCSR_REG           (I22OLED_CSR_REG)  /* UDB incorporates master and slave regs */
    #define I22OLED_MCSR_BUS_BUSY      (I22OLED_STS_BUSY_MASK)      /* Is bus is busy               */
    #define I22OLED_MCSR_START_GEN     (I22OLED_CTRL_START_MASK)    /* Generate Start condition     */
    #define I22OLED_MCSR_RESTART_GEN   (I22OLED_CTRL_RESTART_MASK)  /* Generates RESTART condition  */
    #define I22OLED_MCSR_MSTR_MODE     (I22OLED_STS_MASTER_MODE_MASK)/* Define if active Master     */

    /* Data to write into TX FIFO to release FSM */
    #define I22OLED_PREPARE_TO_RELEASE (0xFFu)
    #define I22OLED_RELEASE_FSM        (0x00u)

    /* Define release command done: history of byte complete status is cleared */
    #define I22OLED_WAIT_RELEASE_CMD_DONE  (I22OLED_RELEASE_FSM != I22OLED_GO_DONE_REG)

    /* Check enable of module */
    #define I22OLED_I2C_ENABLE_REG     (I22OLED_CFG_REG)
    #define I22OLED_IS_I2C_ENABLE(reg) ((0u != ((reg) & I22OLED_ENABLE_MASTER)) || \
                                                 (0u != ((reg) & I22OLED_ENABLE_SLAVE)))

    #define I22OLED_IS_ENABLED         (0u != (I22OLED_CFG_REG & I22OLED_ENABLE_MS))

    /* Check start condition generation */
    #define I22OLED_CHECK_START_GEN(mcsr)  ((0u != (I22OLED_CFG_REG &        \
                                                             I22OLED_MCSR_START_GEN)) \
                                                    &&                                         \
                                                    (0u == ((mcsr) & I22OLED_MCSR_MSTR_MODE)))

    #define I22OLED_CLEAR_START_GEN        do{ \
                                                        I22OLED_CFG_REG &=                 \
                                                        ((uint8) ~I22OLED_MCSR_START_GEN); \
                                                    }while(0)

    /* Stop interrupt */
    #define I22OLED_ENABLE_INT_ON_STOP     do{ \
                                                       I22OLED_INT_MASK_REG |= I22OLED_STOP_IE_MASK; \
                                                    }while(0)

    #define I22OLED_DISABLE_INT_ON_STOP    do{ \
                                                        I22OLED_INT_MASK_REG &=                               \
                                                                             ((uint8) ~I22OLED_STOP_IE_MASK); \
                                                    }while(0)

    /* Transmit data */
    #define I22OLED_TRANSMIT_DATA \
                                    do{    \
                                        I22OLED_CFG_REG     = (I22OLED_CTRL_TRANSMIT_MASK | \
                                                                       I22OLED_CTRL_DEFAULT);        \
                                        I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE;   \
                                        I22OLED_GO_REG      = I22OLED_RELEASE_FSM;          \
                                    }while(0)

    #define I22OLED_ACK_AND_TRANSMIT   I22OLED_TRANSMIT_DATA

    #define I22OLED_NAK_AND_TRANSMIT \
                                        do{   \
                                            I22OLED_CFG_REG     = (I22OLED_CTRL_NACK_MASK     | \
                                                                            I22OLED_CTRL_TRANSMIT_MASK | \
                                                                            I22OLED_CTRL_DEFAULT);       \
                                            I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE;   \
                                            I22OLED_GO_REG      = I22OLED_RELEASE_FSM;          \
                                        }while(0)

    /* Receive data */
    #define I22OLED_READY_TO_READ  \
                                        do{ \
                                            I22OLED_CFG_REG     = I22OLED_CTRL_DEFAULT;       \
                                            I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE; \
                                            I22OLED_GO_REG      = I22OLED_RELEASE_FSM;       \
                                        }while(0)

    #define I22OLED_ACK_AND_RECEIVE    I22OLED_READY_TO_READ

    /* Release bus after arbitration is lost */
    #define I22OLED_BUS_RELEASE    I22OLED_READY_TO_READ

    #define I22OLED_NAK_AND_RECEIVE \
                                        do{  \
                                            I22OLED_CFG_REG     = (I22OLED_CTRL_NACK_MASK |   \
                                                                            I22OLED_CTRL_DEFAULT);     \
                                            I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE; \
                                            I22OLED_GO_REG      = I22OLED_RELEASE_FSM;       \
                                        }while(0)

    /* Master condition generation */
    #define I22OLED_GENERATE_START \
                                        do{ \
                                            I22OLED_CFG_REG     = (I22OLED_CTRL_START_MASK |  \
                                                                            I22OLED_CTRL_DEFAULT);     \
                                            I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE; \
                                            I22OLED_GO_REG      = I22OLED_RELEASE_FSM;       \
                                        }while(0)

    #define I22OLED_GENERATE_RESTART \
                                        do{   \
                                            I22OLED_CFG_REG     = (I22OLED_CTRL_RESTART_MASK | \
                                                                            I22OLED_CTRL_NACK_MASK    | \
                                                                            I22OLED_CTRL_DEFAULT);      \
                                            I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE;  \
                                            I22OLED_GO_REG  =     I22OLED_RELEASE_FSM;         \
                                        }while(0)

    #define I22OLED_GENERATE_STOP  \
                                        do{ \
                                            I22OLED_CFG_REG    = (I22OLED_CTRL_NACK_MASK |    \
                                                                           I22OLED_CTRL_STOP_MASK |    \
                                                                           I22OLED_CTRL_DEFAULT);      \
                                            I22OLED_GO_DONE_REG = I22OLED_PREPARE_TO_RELEASE; \
                                            I22OLED_GO_REG      = I22OLED_RELEASE_FSM;        \
                                        }while(0)

    /* Master manual APIs compatible macros */
    /* These macros wait until byte complete history is cleared after command issued */
    #define I22OLED_GENERATE_START_MANUAL \
                                        do{ \
                                            I22OLED_GENERATE_START;                  \
                                            /* Wait until byte complete history is cleared */ \
                                            while(I22OLED_WAIT_RELEASE_CMD_DONE)     \
                                            {                                                 \
                                            }                                                 \
                                        }while(0)
                                        
    #define I22OLED_GENERATE_RESTART_MANUAL \
                                        do{          \
                                            I22OLED_GENERATE_RESTART;                \
                                            /* Wait until byte complete history is cleared */ \
                                            while(I22OLED_WAIT_RELEASE_CMD_DONE)     \
                                            {                                                 \
                                            }                                                 \
                                        }while(0)

    #define I22OLED_GENERATE_STOP_MANUAL \
                                        do{       \
                                            I22OLED_GENERATE_STOP;                   \
                                            /* Wait until byte complete history is cleared */ \
                                            while(I22OLED_WAIT_RELEASE_CMD_DONE)     \
                                            {                                                 \
                                            }                                                 \
                                        }while(0)

    #define I22OLED_TRANSMIT_DATA_MANUAL \
                                        do{       \
                                            I22OLED_TRANSMIT_DATA;                   \
                                            /* Wait until byte complete history is cleared */ \
                                            while(I22OLED_WAIT_RELEASE_CMD_DONE)     \
                                            {                                                 \
                                            }                                                 \
                                        }while(0)

    #define I22OLED_READY_TO_READ_MANUAL \
                                        do{       \
                                            I22OLED_READY_TO_READ;                   \
                                            /* Wait until byte complete history is cleared */ \
                                            while(I22OLED_WAIT_RELEASE_CMD_DONE)     \
                                            {                                                 \
                                            }                                                 \
                                        }while(0)

    #define I22OLED_ACK_AND_RECEIVE_MANUAL \
                                        do{         \
                                            I22OLED_ACK_AND_RECEIVE;                 \
                                            /* Wait until byte complete history is cleared */ \
                                            while(I22OLED_WAIT_RELEASE_CMD_DONE)     \
                                            {                                                 \
                                            }                                                 \
                                        }while(0)

    #define I22OLED_BUS_RELEASE_MANUAL I22OLED_READY_TO_READ_MANUAL

#endif /* (I22OLED_FF_IMPLEMENTED) */


/***************************************
*     Default register init constants
***************************************/

#define I22OLED_DISABLE    (0u)
#define I22OLED_ENABLE     (1u)

#if (I22OLED_FF_IMPLEMENTED)
    /* I22OLED_XCFG_REG: bits definition */
    #define I22OLED_DEFAULT_XCFG_HW_ADDR_EN ((I22OLED_HW_ADRR_DECODE) ? \
                                                        (I22OLED_XCFG_HDWR_ADDR_EN) : (0u))

    #define I22OLED_DEFAULT_XCFG_I2C_ON    ((I22OLED_WAKEUP_ENABLED) ? \
                                                        (I22OLED_XCFG_I2C_ON) : (0u))


    #define I22OLED_DEFAULT_CFG_SIO_SELECT ((I22OLED_I2C1_SIO_PAIR) ? \
                                                        (I22OLED_CFG_SIO_SELECT) : (0u))


    /* I22OLED_CFG_REG: bits definition */
    #define I22OLED_DEFAULT_CFG_PSELECT    ((I22OLED_WAKEUP_ENABLED) ? \
                                                        (I22OLED_CFG_PSELECT) : (0u))

    #define I22OLED_DEFAULT_CLK_RATE0  ((I22OLED_DATA_RATE <= 50u) ?        \
                                                    (I22OLED_CFG_CLK_RATE_050) :     \
                                                    ((I22OLED_DATA_RATE <= 100u) ?   \
                                                        (I22OLED_CFG_CLK_RATE_100) : \
                                                        (I22OLED_CFG_CLK_RATE_400)))

    #define I22OLED_DEFAULT_CLK_RATE1  ((I22OLED_DATA_RATE <= 50u) ?           \
                                                 (I22OLED_CFG_CLK_RATE_LESS_EQUAL_50) : \
                                                 (I22OLED_CFG_CLK_RATE_GRATER_50))

    #define I22OLED_DEFAULT_CLK_RATE   (I22OLED_DEFAULT_CLK_RATE1)


    #define I22OLED_ENABLE_MASTER      ((I22OLED_MODE_MASTER_ENABLED) ? \
                                                 (I22OLED_CFG_EN_MSTR) : (0u))

    #define I22OLED_ENABLE_SLAVE       ((I22OLED_MODE_SLAVE_ENABLED) ? \
                                                 (I22OLED_CFG_EN_SLAVE) : (0u))

    #define I22OLED_ENABLE_MS      (I22OLED_ENABLE_MASTER | I22OLED_ENABLE_SLAVE)


    /* I22OLED_DEFAULT_XCFG_REG */
    #define I22OLED_DEFAULT_XCFG   (I22OLED_XCFG_CLK_EN         | \
                                             I22OLED_DEFAULT_XCFG_I2C_ON | \
                                             I22OLED_DEFAULT_XCFG_HW_ADDR_EN)

    /* I22OLED_DEFAULT_CFG_REG */
    #define I22OLED_DEFAULT_CFG    (I22OLED_DEFAULT_CFG_SIO_SELECT | \
                                             I22OLED_DEFAULT_CFG_PSELECT    | \
                                             I22OLED_DEFAULT_CLK_RATE       | \
                                             I22OLED_ENABLE_MASTER          | \
                                             I22OLED_ENABLE_SLAVE)

    /*I22OLED_DEFAULT_DIVIDE_FACTOR_REG */
    #define I22OLED_DEFAULT_DIVIDE_FACTOR  ((uint16) 15u)

#else
    /* I22OLED_CFG_REG: bits definition  */
    #define I22OLED_ENABLE_MASTER  ((I22OLED_MODE_MASTER_ENABLED) ? \
                                             (I22OLED_CTRL_ENABLE_MASTER_MASK) : (0u))

    #define I22OLED_ENABLE_SLAVE   ((I22OLED_MODE_SLAVE_ENABLED) ? \
                                             (I22OLED_CTRL_ENABLE_SLAVE_MASK) : (0u))

    #define I22OLED_ENABLE_MS      (I22OLED_ENABLE_MASTER | I22OLED_ENABLE_SLAVE)


    #define I22OLED_DEFAULT_CTRL_ANY_ADDR   ((I22OLED_HW_ADRR_DECODE) ? \
                                                      (0u) : (I22OLED_CTRL_ANY_ADDRESS_MASK))

    /* I22OLED_DEFAULT_CFG_REG */
    #define I22OLED_DEFAULT_CFG    (I22OLED_DEFAULT_CTRL_ANY_ADDR)

    /* All CTRL default bits to be used in macro */
    #define I22OLED_CTRL_DEFAULT   (I22OLED_DEFAULT_CTRL_ANY_ADDR | I22OLED_ENABLE_MS)

    /* Master clock generator: d0 and d1 */
    #define I22OLED_MCLK_PERIOD_VALUE  (0x0Fu)
    #define I22OLED_MCLK_COMPARE_VALUE (0x08u)

    /* Slave bit-counter: control period */
    #define I22OLED_PERIOD_VALUE       (0x07u)

    /* I22OLED_DEFAULT_INT_MASK */
    #define I22OLED_DEFAULT_INT_MASK   (I22OLED_BYTE_COMPLETE_IE_MASK)

    /* I22OLED_DEFAULT_MCLK_PRD_REG */
    #define I22OLED_DEFAULT_MCLK_PRD   (I22OLED_MCLK_PERIOD_VALUE)

    /* I22OLED_DEFAULT_MCLK_CMP_REG */
    #define I22OLED_DEFAULT_MCLK_CMP   (I22OLED_MCLK_COMPARE_VALUE)

    /* I22OLED_DEFAULT_PERIOD_REG */
    #define I22OLED_DEFAULT_PERIOD     (I22OLED_PERIOD_VALUE)

#endif /* (I22OLED_FF_IMPLEMENTED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

#define I22OLED_SSTAT_RD_ERR       (0x08u)
#define I22OLED_SSTAT_WR_ERR       (0x80u)
#define I22OLED_MSTR_SLAVE_BUSY    (I22OLED_MSTR_NOT_READY)
#define I22OLED_MSTAT_ERR_BUF_OVFL (0x80u)
#define I22OLED_SSTAT_RD_CMPT      (I22OLED_SSTAT_RD_CMPLT)
#define I22OLED_SSTAT_WR_CMPT      (I22OLED_SSTAT_WR_CMPLT)
#define I22OLED_MODE_MULTI_MASTER_ENABLE    (I22OLED_MODE_MULTI_MASTER_MASK)
#define I22OLED_DATA_RATE_50       (50u)
#define I22OLED_DATA_RATE_100      (100u)
#define I22OLED_DEV_MASK           (0xF0u)
#define I22OLED_SM_SL_STOP         (0x14u)
#define I22OLED_SM_MASTER_IDLE     (0x40u)
#define I22OLED_HDWR_DECODE        (0x01u)

#endif /* CY_I2C_I22OLED_H */


/* [] END OF FILE */
