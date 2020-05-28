/*******************************************************************************
* File Name: Serial.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_Serial_H)
#define CY_UART_Serial_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Serial_RX_ENABLED                     (1u)
#define Serial_TX_ENABLED                     (1u)
#define Serial_HD_ENABLED                     (0u)
#define Serial_RX_INTERRUPT_ENABLED           (0u)
#define Serial_TX_INTERRUPT_ENABLED           (0u)
#define Serial_INTERNAL_CLOCK_USED            (1u)
#define Serial_RXHW_ADDRESS_ENABLED           (0u)
#define Serial_OVER_SAMPLE_COUNT              (8u)
#define Serial_PARITY_TYPE                    (0u)
#define Serial_PARITY_TYPE_SW                 (0u)
#define Serial_BREAK_DETECT                   (0u)
#define Serial_BREAK_BITS_TX                  (13u)
#define Serial_BREAK_BITS_RX                  (13u)
#define Serial_TXCLKGEN_DP                    (1u)
#define Serial_USE23POLLING                   (1u)
#define Serial_FLOW_CONTROL                   (0u)
#define Serial_CLK_FREQ                       (0u)
#define Serial_TX_BUFFER_SIZE                 (4u)
#define Serial_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define Serial_CONTROL_REG_REMOVED            (0u)
#else
    #define Serial_CONTROL_REG_REMOVED            (1u)
#endif /* End Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Serial_backupStruct_
{
    uint8 enableState;

    #if(Serial_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Serial_CONTROL_REG_REMOVED */

} Serial_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Serial_Start(void) ;
void Serial_Stop(void) ;
uint8 Serial_ReadControlRegister(void) ;
void Serial_WriteControlRegister(uint8 control) ;

void Serial_Init(void) ;
void Serial_Enable(void) ;
void Serial_SaveConfig(void) ;
void Serial_RestoreConfig(void) ;
void Serial_Sleep(void) ;
void Serial_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Serial_RX_ENABLED) || (Serial_HD_ENABLED) )

    #if (Serial_RX_INTERRUPT_ENABLED)
        #define Serial_EnableRxInt()  CyIntEnable (Serial_RX_VECT_NUM)
        #define Serial_DisableRxInt() CyIntDisable(Serial_RX_VECT_NUM)
        CY_ISR_PROTO(Serial_RXISR);
    #endif /* Serial_RX_INTERRUPT_ENABLED */

    void Serial_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Serial_SetRxAddress1(uint8 address) ;
    void Serial_SetRxAddress2(uint8 address) ;

    void  Serial_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Serial_ReadRxData(void) ;
    uint8 Serial_ReadRxStatus(void) ;
    uint8 Serial_GetChar(void) ;
    uint16 Serial_GetByte(void) ;
    uint8 Serial_GetRxBufferSize(void)
                                                            ;
    void Serial_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Serial_GetRxInterruptSource   Serial_ReadRxStatus

#endif /* End (Serial_RX_ENABLED) || (Serial_HD_ENABLED) */

/* Only if TX is enabled */
#if(Serial_TX_ENABLED || Serial_HD_ENABLED)

    #if(Serial_TX_INTERRUPT_ENABLED)
        #define Serial_EnableTxInt()  CyIntEnable (Serial_TX_VECT_NUM)
        #define Serial_DisableTxInt() CyIntDisable(Serial_TX_VECT_NUM)
        #define Serial_SetPendingTxInt() CyIntSetPending(Serial_TX_VECT_NUM)
        #define Serial_ClearPendingTxInt() CyIntClearPending(Serial_TX_VECT_NUM)
        CY_ISR_PROTO(Serial_TXISR);
    #endif /* Serial_TX_INTERRUPT_ENABLED */

    void Serial_SetTxInterruptMode(uint8 intSrc) ;
    void Serial_WriteTxData(uint8 txDataByte) ;
    uint8 Serial_ReadTxStatus(void) ;
    void Serial_PutChar(uint8 txDataByte) ;
    void Serial_PutString(const char8 string[]) ;
    void Serial_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Serial_PutCRLF(uint8 txDataByte) ;
    void Serial_ClearTxBuffer(void) ;
    void Serial_SetTxAddressMode(uint8 addressMode) ;
    void Serial_SendBreak(uint8 retMode) ;
    uint8 Serial_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Serial_PutStringConst         Serial_PutString
    #define Serial_PutArrayConst          Serial_PutArray
    #define Serial_GetTxInterruptSource   Serial_ReadTxStatus

#endif /* End Serial_TX_ENABLED || Serial_HD_ENABLED */

#if(Serial_HD_ENABLED)
    void Serial_LoadRxConfig(void) ;
    void Serial_LoadTxConfig(void) ;
#endif /* End Serial_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Serial) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Serial_CyBtldrCommStart(void) CYSMALL ;
    void    Serial_CyBtldrCommStop(void) CYSMALL ;
    void    Serial_CyBtldrCommReset(void) CYSMALL ;
    cystatus Serial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Serial_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Serial)
        #define CyBtldrCommStart    Serial_CyBtldrCommStart
        #define CyBtldrCommStop     Serial_CyBtldrCommStop
        #define CyBtldrCommReset    Serial_CyBtldrCommReset
        #define CyBtldrCommWrite    Serial_CyBtldrCommWrite
        #define CyBtldrCommRead     Serial_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Serial) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Serial_BYTE2BYTE_TIME_OUT (25u)
    #define Serial_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define Serial_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define Serial_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Serial_SET_SPACE      (0x00u)
#define Serial_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (Serial_TX_ENABLED) || (Serial_HD_ENABLED) )
    #if(Serial_TX_INTERRUPT_ENABLED)
        #define Serial_TX_VECT_NUM            (uint8)Serial_TXInternalInterrupt__INTC_NUMBER
        #define Serial_TX_PRIOR_NUM           (uint8)Serial_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Serial_TX_INTERRUPT_ENABLED */

    #define Serial_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define Serial_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define Serial_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(Serial_TX_ENABLED)
        #define Serial_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (Serial_HD_ENABLED) */
        #define Serial_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (Serial_TX_ENABLED) */

    #define Serial_TX_STS_COMPLETE            (uint8)(0x01u << Serial_TX_STS_COMPLETE_SHIFT)
    #define Serial_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Serial_TX_STS_FIFO_EMPTY_SHIFT)
    #define Serial_TX_STS_FIFO_FULL           (uint8)(0x01u << Serial_TX_STS_FIFO_FULL_SHIFT)
    #define Serial_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Serial_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Serial_TX_ENABLED) || (Serial_HD_ENABLED)*/

#if( (Serial_RX_ENABLED) || (Serial_HD_ENABLED) )
    #if(Serial_RX_INTERRUPT_ENABLED)
        #define Serial_RX_VECT_NUM            (uint8)Serial_RXInternalInterrupt__INTC_NUMBER
        #define Serial_RX_PRIOR_NUM           (uint8)Serial_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Serial_RX_INTERRUPT_ENABLED */
    #define Serial_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Serial_RX_STS_BREAK_SHIFT             (0x01u)
    #define Serial_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Serial_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Serial_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Serial_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Serial_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Serial_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Serial_RX_STS_MRKSPC           (uint8)(0x01u << Serial_RX_STS_MRKSPC_SHIFT)
    #define Serial_RX_STS_BREAK            (uint8)(0x01u << Serial_RX_STS_BREAK_SHIFT)
    #define Serial_RX_STS_PAR_ERROR        (uint8)(0x01u << Serial_RX_STS_PAR_ERROR_SHIFT)
    #define Serial_RX_STS_STOP_ERROR       (uint8)(0x01u << Serial_RX_STS_STOP_ERROR_SHIFT)
    #define Serial_RX_STS_OVERRUN          (uint8)(0x01u << Serial_RX_STS_OVERRUN_SHIFT)
    #define Serial_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Serial_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Serial_RX_STS_ADDR_MATCH       (uint8)(0x01u << Serial_RX_STS_ADDR_MATCH_SHIFT)
    #define Serial_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Serial_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Serial_RX_HW_MASK                     (0x7Fu)
#endif /* End (Serial_RX_ENABLED) || (Serial_HD_ENABLED) */

/* Control Register definitions */
#define Serial_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Serial_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Serial_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Serial_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Serial_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Serial_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Serial_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Serial_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Serial_CTRL_HD_SEND               (uint8)(0x01u << Serial_CTRL_HD_SEND_SHIFT)
#define Serial_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Serial_CTRL_HD_SEND_BREAK_SHIFT)
#define Serial_CTRL_MARK                  (uint8)(0x01u << Serial_CTRL_MARK_SHIFT)
#define Serial_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Serial_CTRL_PARITY_TYPE0_SHIFT)
#define Serial_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Serial_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Serial_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Serial_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Serial_SEND_BREAK                         (0x00u)
#define Serial_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Serial_REINIT                             (0x02u)
#define Serial_SEND_WAIT_REINIT                   (0x03u)

#define Serial_OVER_SAMPLE_8                      (8u)
#define Serial_OVER_SAMPLE_16                     (16u)

#define Serial_BIT_CENTER                         (Serial_OVER_SAMPLE_COUNT - 2u)

#define Serial_FIFO_LENGTH                        (4u)
#define Serial_NUMBER_OF_START_BIT                (1u)
#define Serial_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define Serial_TXBITCTR_BREAKBITS8X   ((Serial_BREAK_BITS_TX * Serial_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define Serial_TXBITCTR_BREAKBITS ((Serial_BREAK_BITS_TX * Serial_OVER_SAMPLE_COUNT) - 1u)

#define Serial_HALF_BIT_COUNT   \
                            (((Serial_OVER_SAMPLE_COUNT / 2u) + (Serial_USE23POLLING * 1u)) - 2u)
#if (Serial_OVER_SAMPLE_COUNT == Serial_OVER_SAMPLE_8)
    #define Serial_HD_TXBITCTR_INIT   (((Serial_BREAK_BITS_TX + \
                            Serial_NUMBER_OF_START_BIT) * Serial_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Serial_RXBITCTR_INIT  ((((Serial_BREAK_BITS_RX + Serial_NUMBER_OF_START_BIT) \
                            * Serial_OVER_SAMPLE_COUNT) + Serial_HALF_BIT_COUNT) - 1u)

#else /* Serial_OVER_SAMPLE_COUNT == Serial_OVER_SAMPLE_16 */
    #define Serial_HD_TXBITCTR_INIT   ((8u * Serial_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define Serial_RXBITCTR_INIT      (((7u * Serial_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Serial_HALF_BIT_COUNT)
#endif /* End Serial_OVER_SAMPLE_COUNT */

#define Serial_HD_RXBITCTR_INIT                   Serial_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Serial_initVar;
#if (Serial_TX_INTERRUPT_ENABLED && Serial_TX_ENABLED)
    extern volatile uint8 Serial_txBuffer[Serial_TX_BUFFER_SIZE];
    extern volatile uint8 Serial_txBufferRead;
    extern uint8 Serial_txBufferWrite;
#endif /* (Serial_TX_INTERRUPT_ENABLED && Serial_TX_ENABLED) */
#if (Serial_RX_INTERRUPT_ENABLED && (Serial_RX_ENABLED || Serial_HD_ENABLED))
    extern uint8 Serial_errorStatus;
    extern volatile uint8 Serial_rxBuffer[Serial_RX_BUFFER_SIZE];
    extern volatile uint8 Serial_rxBufferRead;
    extern volatile uint8 Serial_rxBufferWrite;
    extern volatile uint8 Serial_rxBufferLoopDetect;
    extern volatile uint8 Serial_rxBufferOverflow;
    #if (Serial_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Serial_rxAddressMode;
        extern volatile uint8 Serial_rxAddressDetected;
    #endif /* (Serial_RXHW_ADDRESS_ENABLED) */
#endif /* (Serial_RX_INTERRUPT_ENABLED && (Serial_RX_ENABLED || Serial_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Serial__B_UART__AM_SW_BYTE_BYTE 1
#define Serial__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Serial__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Serial__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Serial__B_UART__AM_NONE 0

#define Serial__B_UART__NONE_REVB 0
#define Serial__B_UART__EVEN_REVB 1
#define Serial__B_UART__ODD_REVB 2
#define Serial__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Serial_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Serial_NUMBER_OF_STOP_BITS    (1u)

#if (Serial_RXHW_ADDRESS_ENABLED)
    #define Serial_RX_ADDRESS_MODE    (0u)
    #define Serial_RX_HW_ADDRESS1     (0u)
    #define Serial_RX_HW_ADDRESS2     (0u)
#endif /* (Serial_RXHW_ADDRESS_ENABLED) */

#define Serial_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Serial_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Serial_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Serial_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Serial_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Serial_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Serial_RX_STS_BREAK_SHIFT) \
                                        | (0 << Serial_RX_STS_OVERRUN_SHIFT))

#define Serial_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Serial_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Serial_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Serial_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Serial_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Serial_CONTROL_REG \
                            (* (reg8 *) Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Serial_CONTROL_PTR \
                            (  (reg8 *) Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Serial_TX_ENABLED)
    #define Serial_TXDATA_REG          (* (reg8 *) Serial_BUART_sTX_TxShifter_u0__F0_REG)
    #define Serial_TXDATA_PTR          (  (reg8 *) Serial_BUART_sTX_TxShifter_u0__F0_REG)
    #define Serial_TXDATA_AUX_CTL_REG  (* (reg8 *) Serial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Serial_TXDATA_AUX_CTL_PTR  (  (reg8 *) Serial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Serial_TXSTATUS_REG        (* (reg8 *) Serial_BUART_sTX_TxSts__STATUS_REG)
    #define Serial_TXSTATUS_PTR        (  (reg8 *) Serial_BUART_sTX_TxSts__STATUS_REG)
    #define Serial_TXSTATUS_MASK_REG   (* (reg8 *) Serial_BUART_sTX_TxSts__MASK_REG)
    #define Serial_TXSTATUS_MASK_PTR   (  (reg8 *) Serial_BUART_sTX_TxSts__MASK_REG)
    #define Serial_TXSTATUS_ACTL_REG   (* (reg8 *) Serial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Serial_TXSTATUS_ACTL_PTR   (  (reg8 *) Serial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Serial_TXCLKGEN_DP)
        #define Serial_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Serial_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Serial_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Serial_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Serial_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Serial_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Serial_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Serial_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Serial_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Serial_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Serial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Serial_TXCLKGEN_DP */

#endif /* End Serial_TX_ENABLED */

#if(Serial_HD_ENABLED)

    #define Serial_TXDATA_REG             (* (reg8 *) Serial_BUART_sRX_RxShifter_u0__F1_REG )
    #define Serial_TXDATA_PTR             (  (reg8 *) Serial_BUART_sRX_RxShifter_u0__F1_REG )
    #define Serial_TXDATA_AUX_CTL_REG     (* (reg8 *) Serial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Serial_TXDATA_AUX_CTL_PTR     (  (reg8 *) Serial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Serial_TXSTATUS_REG           (* (reg8 *) Serial_BUART_sRX_RxSts__STATUS_REG )
    #define Serial_TXSTATUS_PTR           (  (reg8 *) Serial_BUART_sRX_RxSts__STATUS_REG )
    #define Serial_TXSTATUS_MASK_REG      (* (reg8 *) Serial_BUART_sRX_RxSts__MASK_REG )
    #define Serial_TXSTATUS_MASK_PTR      (  (reg8 *) Serial_BUART_sRX_RxSts__MASK_REG )
    #define Serial_TXSTATUS_ACTL_REG      (* (reg8 *) Serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Serial_TXSTATUS_ACTL_PTR      (  (reg8 *) Serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Serial_HD_ENABLED */

#if( (Serial_RX_ENABLED) || (Serial_HD_ENABLED) )
    #define Serial_RXDATA_REG             (* (reg8 *) Serial_BUART_sRX_RxShifter_u0__F0_REG )
    #define Serial_RXDATA_PTR             (  (reg8 *) Serial_BUART_sRX_RxShifter_u0__F0_REG )
    #define Serial_RXADDRESS1_REG         (* (reg8 *) Serial_BUART_sRX_RxShifter_u0__D0_REG )
    #define Serial_RXADDRESS1_PTR         (  (reg8 *) Serial_BUART_sRX_RxShifter_u0__D0_REG )
    #define Serial_RXADDRESS2_REG         (* (reg8 *) Serial_BUART_sRX_RxShifter_u0__D1_REG )
    #define Serial_RXADDRESS2_PTR         (  (reg8 *) Serial_BUART_sRX_RxShifter_u0__D1_REG )
    #define Serial_RXDATA_AUX_CTL_REG     (* (reg8 *) Serial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Serial_RXBITCTR_PERIOD_REG    (* (reg8 *) Serial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Serial_RXBITCTR_PERIOD_PTR    (  (reg8 *) Serial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Serial_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Serial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Serial_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Serial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Serial_RXBITCTR_COUNTER_REG   (* (reg8 *) Serial_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Serial_RXBITCTR_COUNTER_PTR   (  (reg8 *) Serial_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Serial_RXSTATUS_REG           (* (reg8 *) Serial_BUART_sRX_RxSts__STATUS_REG )
    #define Serial_RXSTATUS_PTR           (  (reg8 *) Serial_BUART_sRX_RxSts__STATUS_REG )
    #define Serial_RXSTATUS_MASK_REG      (* (reg8 *) Serial_BUART_sRX_RxSts__MASK_REG )
    #define Serial_RXSTATUS_MASK_PTR      (  (reg8 *) Serial_BUART_sRX_RxSts__MASK_REG )
    #define Serial_RXSTATUS_ACTL_REG      (* (reg8 *) Serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Serial_RXSTATUS_ACTL_PTR      (  (reg8 *) Serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Serial_RX_ENABLED) || (Serial_HD_ENABLED) */

#if(Serial_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Serial_INTCLOCK_CLKEN_REG     (* (reg8 *) Serial_IntClock__PM_ACT_CFG)
    #define Serial_INTCLOCK_CLKEN_PTR     (  (reg8 *) Serial_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Serial_INTCLOCK_CLKEN_MASK    Serial_IntClock__PM_ACT_MSK
#endif /* End Serial_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Serial_TX_ENABLED)
    #define Serial_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Serial_TX_ENABLED */

#if(Serial_HD_ENABLED)
    #define Serial_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Serial_HD_ENABLED */

#if( (Serial_RX_ENABLED) || (Serial_HD_ENABLED) )
    #define Serial_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Serial_RX_ENABLED) || (Serial_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define Serial_WAIT_1_MS      Serial_BL_CHK_DELAY_MS   

#define Serial_TXBUFFERSIZE   Serial_TX_BUFFER_SIZE
#define Serial_RXBUFFERSIZE   Serial_RX_BUFFER_SIZE

#if (Serial_RXHW_ADDRESS_ENABLED)
    #define Serial_RXADDRESSMODE  Serial_RX_ADDRESS_MODE
    #define Serial_RXHWADDRESS1   Serial_RX_HW_ADDRESS1
    #define Serial_RXHWADDRESS2   Serial_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define Serial_RXAddressMode  Serial_RXADDRESSMODE
#endif /* (Serial_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define Serial_initvar                    Serial_initVar

#define Serial_RX_Enabled                 Serial_RX_ENABLED
#define Serial_TX_Enabled                 Serial_TX_ENABLED
#define Serial_HD_Enabled                 Serial_HD_ENABLED
#define Serial_RX_IntInterruptEnabled     Serial_RX_INTERRUPT_ENABLED
#define Serial_TX_IntInterruptEnabled     Serial_TX_INTERRUPT_ENABLED
#define Serial_InternalClockUsed          Serial_INTERNAL_CLOCK_USED
#define Serial_RXHW_Address_Enabled       Serial_RXHW_ADDRESS_ENABLED
#define Serial_OverSampleCount            Serial_OVER_SAMPLE_COUNT
#define Serial_ParityType                 Serial_PARITY_TYPE

#if( Serial_TX_ENABLED && (Serial_TXBUFFERSIZE > Serial_FIFO_LENGTH))
    #define Serial_TXBUFFER               Serial_txBuffer
    #define Serial_TXBUFFERREAD           Serial_txBufferRead
    #define Serial_TXBUFFERWRITE          Serial_txBufferWrite
#endif /* End Serial_TX_ENABLED */
#if( ( Serial_RX_ENABLED || Serial_HD_ENABLED ) && \
     (Serial_RXBUFFERSIZE > Serial_FIFO_LENGTH) )
    #define Serial_RXBUFFER               Serial_rxBuffer
    #define Serial_RXBUFFERREAD           Serial_rxBufferRead
    #define Serial_RXBUFFERWRITE          Serial_rxBufferWrite
    #define Serial_RXBUFFERLOOPDETECT     Serial_rxBufferLoopDetect
    #define Serial_RXBUFFER_OVERFLOW      Serial_rxBufferOverflow
#endif /* End Serial_RX_ENABLED */

#ifdef Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Serial_CONTROL                Serial_CONTROL_REG
#endif /* End Serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Serial_TX_ENABLED)
    #define Serial_TXDATA                 Serial_TXDATA_REG
    #define Serial_TXSTATUS               Serial_TXSTATUS_REG
    #define Serial_TXSTATUS_MASK          Serial_TXSTATUS_MASK_REG
    #define Serial_TXSTATUS_ACTL          Serial_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Serial_TXCLKGEN_DP)
        #define Serial_TXBITCLKGEN_CTR        Serial_TXBITCLKGEN_CTR_REG
        #define Serial_TXBITCLKTX_COMPLETE    Serial_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Serial_TXBITCTR_PERIOD        Serial_TXBITCTR_PERIOD_REG
        #define Serial_TXBITCTR_CONTROL       Serial_TXBITCTR_CONTROL_REG
        #define Serial_TXBITCTR_COUNTER       Serial_TXBITCTR_COUNTER_REG
    #endif /* Serial_TXCLKGEN_DP */
#endif /* End Serial_TX_ENABLED */

#if(Serial_HD_ENABLED)
    #define Serial_TXDATA                 Serial_TXDATA_REG
    #define Serial_TXSTATUS               Serial_TXSTATUS_REG
    #define Serial_TXSTATUS_MASK          Serial_TXSTATUS_MASK_REG
    #define Serial_TXSTATUS_ACTL          Serial_TXSTATUS_ACTL_REG
#endif /* End Serial_HD_ENABLED */

#if( (Serial_RX_ENABLED) || (Serial_HD_ENABLED) )
    #define Serial_RXDATA                 Serial_RXDATA_REG
    #define Serial_RXADDRESS1             Serial_RXADDRESS1_REG
    #define Serial_RXADDRESS2             Serial_RXADDRESS2_REG
    #define Serial_RXBITCTR_PERIOD        Serial_RXBITCTR_PERIOD_REG
    #define Serial_RXBITCTR_CONTROL       Serial_RXBITCTR_CONTROL_REG
    #define Serial_RXBITCTR_COUNTER       Serial_RXBITCTR_COUNTER_REG
    #define Serial_RXSTATUS               Serial_RXSTATUS_REG
    #define Serial_RXSTATUS_MASK          Serial_RXSTATUS_MASK_REG
    #define Serial_RXSTATUS_ACTL          Serial_RXSTATUS_ACTL_REG
#endif /* End  (Serial_RX_ENABLED) || (Serial_HD_ENABLED) */

#if(Serial_INTERNAL_CLOCK_USED)
    #define Serial_INTCLOCK_CLKEN         Serial_INTCLOCK_CLKEN_REG
#endif /* End Serial_INTERNAL_CLOCK_USED */

#define Serial_WAIT_FOR_COMLETE_REINIT    Serial_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Serial_H */


/* [] END OF FILE */
