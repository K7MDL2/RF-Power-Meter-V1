/***************************************************************************//**
* \file  Serial_USB.h
* \version 3.20
*
* \brief
*  This file provides function prototypes and constants for the USBFS component. 
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_USBFS_Serial_USB_H)
#define CY_USBFS_Serial_USB_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h"
#include "cyapicallbacks.h"

/*  User supplied definitions. */
/* `#START USER_DEFINITIONS` Place your declaration here */

/* `#END` */

/***************************************
* Enumerated Types and Parameters
***************************************/

/* USB IP memory management options. */
#define Serial_USB__EP_MANUAL    (0u)
#define Serial_USB__EP_DMAMANUAL (1u)
#define Serial_USB__EP_DMAAUTO   (2u)

/* USB IP memory allocation options. */
#define Serial_USB__MA_STATIC    (0u)
#define Serial_USB__MA_DYNAMIC   (1u)


/***************************************
*    Initial Parameter Constants
***************************************/

#define Serial_USB_NUM_DEVICES                 (1u)
#define Serial_USB_ENABLE_CDC_CLASS            
#define Serial_USB_ENABLE_MIDI_CLASS           (0u)
#define Serial_USB_ENABLE_MSC_CLASS            (0u)
#define Serial_USB_BOS_ENABLE                  (0u)
#define Serial_USB_ENABLE_DESCRIPTOR_STRINGS   
#define Serial_USB_ENABLE_SN_STRING            
#define Serial_USB_ENABLE_IDSN_STRING          
#define Serial_USB_ENABLE_STRINGS              
#define Serial_USB_MAX_REPORTID_NUMBER         (0u)

#define Serial_USB_MON_VBUS               (0u)
#define Serial_USB_EXTERN_VBUS            (0u)
#define Serial_USB_POWER_PAD_VBUS         (0u)
#define Serial_USB_EXTERN_VND             (0u)
#define Serial_USB_EXTERN_CLS             (0u)
#define Serial_USB_MAX_INTERFACES_NUMBER  (2u)
#define Serial_USB_EP_MM                  (0u)
#define Serial_USB_EP_MA                  (0u)
#define Serial_USB_ENABLE_BATT_CHARG_DET  (0u)
#define Serial_USB_GEN_16BITS_EP_ACCESS   (1u)

/* Enable Class APIs: MIDI, CDC, MSC. */         
#define Serial_USB_ENABLE_CDC_CLASS_API   (0u != (1u))

/* General parameters */
#define Serial_USB_EP_ALLOC_STATIC            (Serial_USB_EP_MA == Serial_USB__MA_STATIC)
#define Serial_USB_EP_ALLOC_DYNAMIC           (Serial_USB_EP_MA == Serial_USB__MA_DYNAMIC)
#define Serial_USB_EP_MANAGEMENT_MANUAL       (Serial_USB_EP_MM == Serial_USB__EP_MANUAL)
#define Serial_USB_EP_MANAGEMENT_DMA          (Serial_USB_EP_MM != Serial_USB__EP_MANUAL)
#define Serial_USB_EP_MANAGEMENT_DMA_MANUAL   (Serial_USB_EP_MM == Serial_USB__EP_DMAMANUAL)
#define Serial_USB_EP_MANAGEMENT_DMA_AUTO     (Serial_USB_EP_MM == Serial_USB__EP_DMAAUTO)
#define Serial_USB_BATT_CHARG_DET_ENABLE      (CY_PSOC4 && (0u != Serial_USB_ENABLE_BATT_CHARG_DET))
#define Serial_USB_16BITS_EP_ACCESS_ENABLE    (CY_PSOC4 && (0u != Serial_USB_GEN_16BITS_EP_ACCESS))
#define Serial_USB_VBUS_MONITORING_ENABLE     (0u != Serial_USB_MON_VBUS)
#define Serial_USB_VBUS_MONITORING_INTERNAL   (0u == Serial_USB_EXTERN_VBUS)
#define Serial_USB_VBUS_POWER_PAD_ENABLE      (0u != Serial_USB_POWER_PAD_VBUS)

/* Control endpoints availability */
#define Serial_USB_SOF_ISR_REMOVE       (0u)
#define Serial_USB_BUS_RESET_ISR_REMOVE (0u)
#define Serial_USB_EP0_ISR_REMOVE       (0u)
#define Serial_USB_ARB_ISR_REMOVE       (0u)
#define Serial_USB_DP_ISR_REMOVE        (0u)
#define Serial_USB_LPM_REMOVE           (1u)
#define Serial_USB_SOF_ISR_ACTIVE       ((0u == Serial_USB_SOF_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_BUS_RESET_ISR_ACTIVE ((0u == Serial_USB_BUS_RESET_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP0_ISR_ACTIVE       ((0u == Serial_USB_EP0_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_ARB_ISR_ACTIVE       ((0u == Serial_USB_ARB_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_DP_ISR_ACTIVE        ((0u == Serial_USB_DP_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_LPM_ACTIVE           ((CY_PSOC4 && (0u == Serial_USB_LPM_REMOVE)) ? 1u: 0u)

/* Data endpoints availability */
#define Serial_USB_EP1_ISR_REMOVE     (0u)
#define Serial_USB_EP2_ISR_REMOVE     (0u)
#define Serial_USB_EP3_ISR_REMOVE     (0u)
#define Serial_USB_EP4_ISR_REMOVE     (1u)
#define Serial_USB_EP5_ISR_REMOVE     (1u)
#define Serial_USB_EP6_ISR_REMOVE     (1u)
#define Serial_USB_EP7_ISR_REMOVE     (1u)
#define Serial_USB_EP8_ISR_REMOVE     (1u)
#define Serial_USB_EP1_ISR_ACTIVE     ((0u == Serial_USB_EP1_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP2_ISR_ACTIVE     ((0u == Serial_USB_EP2_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP3_ISR_ACTIVE     ((0u == Serial_USB_EP3_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP4_ISR_ACTIVE     ((0u == Serial_USB_EP4_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP5_ISR_ACTIVE     ((0u == Serial_USB_EP5_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP6_ISR_ACTIVE     ((0u == Serial_USB_EP6_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP7_ISR_ACTIVE     ((0u == Serial_USB_EP7_ISR_REMOVE) ? 1u: 0u)
#define Serial_USB_EP8_ISR_ACTIVE     ((0u == Serial_USB_EP8_ISR_REMOVE) ? 1u: 0u)

#define Serial_USB_EP_DMA_AUTO_OPT    ((CY_PSOC4) ? (1u) : (0u))
#define Serial_USB_DMA1_REMOVE        (1u)
#define Serial_USB_DMA2_REMOVE        (1u)
#define Serial_USB_DMA3_REMOVE        (1u)
#define Serial_USB_DMA4_REMOVE        (1u)
#define Serial_USB_DMA5_REMOVE        (1u)
#define Serial_USB_DMA6_REMOVE        (1u)
#define Serial_USB_DMA7_REMOVE        (1u)
#define Serial_USB_DMA8_REMOVE        (1u)
#define Serial_USB_DMA1_ACTIVE        ((0u == Serial_USB_DMA1_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA2_ACTIVE        ((0u == Serial_USB_DMA2_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA3_ACTIVE        ((0u == Serial_USB_DMA3_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA4_ACTIVE        ((0u == Serial_USB_DMA4_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA5_ACTIVE        ((0u == Serial_USB_DMA5_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA6_ACTIVE        ((0u == Serial_USB_DMA6_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA7_ACTIVE        ((0u == Serial_USB_DMA7_REMOVE) ? 1u: 0u)
#define Serial_USB_DMA8_ACTIVE        ((0u == Serial_USB_DMA8_REMOVE) ? 1u: 0u)


/***************************************
*    Data Structures Definition
***************************************/

typedef struct
{
    uint8  attrib;
    uint8  apiEpState;
    uint8  hwEpState;
    uint8  epToggle;
    uint8  addr;
    uint8  epMode;
    uint16 buffOffset;
    uint16 bufferSize;
    uint8  interface;
} T_Serial_USB_EP_CTL_BLOCK;

typedef struct
{
    uint8  interface;
    uint8  altSetting;
    uint8  addr;
    uint8  attributes;
    uint16 bufferSize;
    uint8  bMisc;
} T_Serial_USB_EP_SETTINGS_BLOCK;

typedef struct
{
    uint8  status;
    uint16 length;
} T_Serial_USB_XFER_STATUS_BLOCK;

typedef struct
{
    uint16  count;
    volatile uint8 *pData;
    T_Serial_USB_XFER_STATUS_BLOCK *pStatusBlock;
} T_Serial_USB_TD;

typedef struct
{
    uint8   c;
    const void *p_list;
} T_Serial_USB_LUT;

/* Resume/Suspend API Support */
typedef struct
{
    uint8 enableState;
    uint8 mode;
#if (CY_PSOC4)
    uint8 intrSeiMask;
#endif /* (CY_PSOC4) */
} Serial_USB_BACKUP_STRUCT;

/* Number of endpoint 0 data registers. */
#define Serial_USB_EP0_DR_MAPPED_REG_CNT  (8u)

/* Structure to access data registers for EP0. */
typedef struct
{
    uint8 epData[Serial_USB_EP0_DR_MAPPED_REG_CNT];
} Serial_USB_ep0_data_struct;

/* Number of SIE endpoint registers group. */
#define Serial_USB_SIE_EP_REG_SIZE   (Serial_USB_USB__SIE_EP1_CR0 - \
                                            Serial_USB_USB__SIE_EP1_CNT0)

/* Size of gap between SIE endpoint registers groups. */
#define Serial_USB_SIE_GAP_CNT        (((Serial_USB_USB__SIE_EP2_CNT0 - \
                                             (Serial_USB_USB__SIE_EP1_CNT0 + \
                                              Serial_USB_SIE_EP_REG_SIZE)) / sizeof(reg8)) - 1u)

/* Structure to access to SIE registers for endpoint. */
typedef struct
{
    uint8 epCnt0;
    uint8 epCnt1;
    uint8 epCr0;
    uint8 gap[Serial_USB_SIE_GAP_CNT];
} Serial_USB_sie_ep_struct;

/* Number of ARB endpoint registers group. */
#define Serial_USB_ARB_EP_REG_SIZE    (Serial_USB_USB__ARB_RW1_DR - \
                                             Serial_USB_USB__ARB_EP1_CFG)

/* Size of gap between ARB endpoint registers groups. */
#define Serial_USB_ARB_GAP_CNT        (((Serial_USB_USB__ARB_EP2_CFG - \
                                             (Serial_USB_USB__ARB_EP1_CFG + \
                                              Serial_USB_ARB_EP_REG_SIZE)) / sizeof(reg8)) - 1u)

/* Structure to access to ARB registers for endpoint. */
typedef struct
{
    uint8 epCfg;
    uint8 epIntEn;
    uint8 epSr;
    uint8 reserved;
    uint8 rwWa;
    uint8 rwWaMsb;
    uint8 rwRa;
    uint8 rwRaMsb;
    uint8 rwDr;
    uint8 gap[Serial_USB_ARB_GAP_CNT];
} Serial_USB_arb_ep_struct;

#if (CY_PSOC4)
    /* Number of ARB endpoint registers group (16-bits access). */
    #define Serial_USB_ARB_EP_REG16_SIZE      (Serial_USB_USB__ARB_RW1_DR16 - \
                                                     Serial_USB_USB__ARB_RW1_WA16)

    /* Size of gap between ARB endpoint registers groups (16-bits access). */
    #define Serial_USB_ARB_EP_REG16_GAP_CNT   (((Serial_USB_USB__ARB_RW2_WA16 - \
                                                     (Serial_USB_USB__ARB_RW1_WA16 + \
                                                      Serial_USB_ARB_EP_REG16_SIZE)) / sizeof(reg8)) - 1u)

    /* Structure to access to ARB registers for endpoint (16-bits access). */
    typedef struct
    {
        uint8 rwWa16;
        uint8 reserved0;
        uint8 rwRa16;
        uint8 reserved1;
        uint8 rwDr16;
        uint8 gap[Serial_USB_ARB_EP_REG16_GAP_CNT];
    } Serial_USB_arb_ep_reg16_struct;
#endif /* (CY_PSOC4) */

/* Number of endpoint (takes to account that endpoints numbers are 1-8). */
#define Serial_USB_NUMBER_EP  (9u)

/* Consoled SIE register groups for endpoints 1-8. */
typedef struct
{
    Serial_USB_sie_ep_struct sieEp[Serial_USB_NUMBER_EP];
} Serial_USB_sie_eps_struct;

/* Consolidate ARB register groups for endpoints 1-8.*/
typedef struct
{
    Serial_USB_arb_ep_struct arbEp[Serial_USB_NUMBER_EP];
} Serial_USB_arb_eps_struct;

#if (CY_PSOC4)
    /* Consolidate ARB register groups for endpoints 1-8 (16-bits access). */
    typedef struct
    {
        Serial_USB_arb_ep_reg16_struct arbEp[Serial_USB_NUMBER_EP];
    } Serial_USB_arb_eps_reg16_struct;
#endif /* (CY_PSOC4) */


/***************************************
*       Function Prototypes
***************************************/
/**
* \addtogroup group_general
* @{
*/
void   Serial_USB_InitComponent(uint8 device, uint8 mode) ;
void   Serial_USB_Start(uint8 device, uint8 mode)         ;
void   Serial_USB_Init(void)                              ;
void   Serial_USB_Stop(void)                              ;
uint8  Serial_USB_GetConfiguration(void)                  ;
uint8  Serial_USB_IsConfigurationChanged(void)            ;
uint8  Serial_USB_GetInterfaceSetting(uint8 interfaceNumber) ;
uint8  Serial_USB_GetEPState(uint8 epNumber)              ;
uint16 Serial_USB_GetEPCount(uint8 epNumber)              ;
void   Serial_USB_LoadInEP(uint8 epNumber, const uint8 pData[], uint16 length)
                                                                ;
uint16 Serial_USB_ReadOutEP(uint8 epNumber, uint8 pData[], uint16 length)
                                                                ;
void   Serial_USB_EnableOutEP(uint8 epNumber)             ;
void   Serial_USB_DisableOutEP(uint8 epNumber)            ;
void   Serial_USB_Force(uint8 bState)                     ;
uint8  Serial_USB_GetEPAckState(uint8 epNumber)           ;
void   Serial_USB_SetPowerStatus(uint8 powerStatus)       ;
void   Serial_USB_TerminateEP(uint8 epNumber)             ;

uint8 Serial_USB_GetDeviceAddress(void) ;

void Serial_USB_EnableSofInt(void)  ;
void Serial_USB_DisableSofInt(void) ;


#if defined(Serial_USB_ENABLE_FWSN_STRING)
    void   Serial_USB_SerialNumString(uint8 snString[]) ;
#endif  /* Serial_USB_ENABLE_FWSN_STRING */

#if (Serial_USB_VBUS_MONITORING_ENABLE)
    uint8  Serial_USB_VBusPresent(void) ;
#endif /*  (Serial_USB_VBUS_MONITORING_ENABLE) */

#if (Serial_USB_16BITS_EP_ACCESS_ENABLE)
    /* PSoC4 specific functions for 16-bit data register access. */
    void   Serial_USB_LoadInEP16 (uint8 epNumber, const uint8 pData[], uint16 length);
    uint16 Serial_USB_ReadOutEP16(uint8 epNumber,       uint8 pData[], uint16 length);
#endif /* (Serial_USB_16BITS_EP_ACCESS_ENABLE) */

#if (Serial_USB_BATT_CHARG_DET_ENABLE)
        uint8 Serial_USB_Bcd_DetectPortType(void);
#endif /* (Serial_USB_BATT_CHARG_DET_ENABLE) */

#if (Serial_USB_EP_MANAGEMENT_DMA)
    void Serial_USB_InitEP_DMA(uint8 epNumber, const uint8 *pData) ;   
    void Serial_USB_Stop_DMA(uint8 epNumber) ;
/** @} general */ 
#endif /* (Serial_USB_EP_MANAGEMENT_DMA) */

/**
* \addtogroup group_power
* @{
*/
uint8  Serial_USB_CheckActivity(void) ;
void   Serial_USB_Suspend(void)       ;
void   Serial_USB_Resume(void)        ;
uint8  Serial_USB_RWUEnabled(void)    ;

#if (Serial_USB_LPM_ACTIVE)
    uint32 Serial_USB_Lpm_GetBeslValue(void);
    uint32 Serial_USB_Lpm_RemoteWakeUpAllowed(void);
    void   Serial_USB_Lpm_SetResponse(uint32 response);
    uint32 Serial_USB_Lpm_GetResponse(void);
#endif /* (Serial_USB_LPM_ACTIVE) */

/** @} power */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Serial_USB) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
/**
* \addtogroup group_bootloader
* @{
*/
    void Serial_USB_CyBtldrCommStart(void)        ;
    void Serial_USB_CyBtldrCommStop(void)         ;
    void Serial_USB_CyBtldrCommReset(void)        ;
    cystatus Serial_USB_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 *count, uint8 timeOut) CYSMALL
                                                        ;
    cystatus Serial_USB_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 *count, uint8 timeOut) CYSMALL
                                                        ;
/** @} bootloader */

    #define Serial_USB_BTLDR_OUT_EP   (0x01u)
    #define Serial_USB_BTLDR_IN_EP    (0x02u)

    #define Serial_USB_BTLDR_SIZEOF_WRITE_BUFFER  (64u)   /* Endpoint 1 (OUT) buffer size. */
    #define Serial_USB_BTLDR_SIZEOF_READ_BUFFER   (64u)   /* Endpoint 2 (IN)  buffer size. */
    #define Serial_USB_BTLDR_MAX_PACKET_SIZE      Serial_USB_BTLDR_SIZEOF_WRITE_BUFFER

    #define Serial_USB_BTLDR_WAIT_1_MS            (1u)    /* Time Out quantity equal 1mS */

    /* Map-specific USB bootloader communication functions to common bootloader functions */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Serial_USB)
        #define CyBtldrCommStart        Serial_USB_CyBtldrCommStart
        #define CyBtldrCommStop         Serial_USB_CyBtldrCommStop
        #define CyBtldrCommReset        Serial_USB_CyBtldrCommReset
        #define CyBtldrCommWrite        Serial_USB_CyBtldrCommWrite
        #define CyBtldrCommRead         Serial_USB_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Serial_USB) */
#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/

#define Serial_USB_EP0                        (0u)
#define Serial_USB_EP1                        (1u)
#define Serial_USB_EP2                        (2u)
#define Serial_USB_EP3                        (3u)
#define Serial_USB_EP4                        (4u)
#define Serial_USB_EP5                        (5u)
#define Serial_USB_EP6                        (6u)
#define Serial_USB_EP7                        (7u)
#define Serial_USB_EP8                        (8u)
#define Serial_USB_MAX_EP                     (9u)

#define Serial_USB_TRUE                       (1u)
#define Serial_USB_FALSE                      (0u)

#define Serial_USB_NO_EVENT_ALLOWED           (2u)
#define Serial_USB_EVENT_PENDING              (1u)
#define Serial_USB_NO_EVENT_PENDING           (0u)

#define Serial_USB_IN_BUFFER_FULL             Serial_USB_NO_EVENT_PENDING
#define Serial_USB_IN_BUFFER_EMPTY            Serial_USB_EVENT_PENDING
#define Serial_USB_OUT_BUFFER_FULL            Serial_USB_EVENT_PENDING
#define Serial_USB_OUT_BUFFER_EMPTY           Serial_USB_NO_EVENT_PENDING

#define Serial_USB_FORCE_J                    (0xA0u)
#define Serial_USB_FORCE_K                    (0x80u)
#define Serial_USB_FORCE_SE0                  (0xC0u)
#define Serial_USB_FORCE_NONE                 (0x00u)

#define Serial_USB_IDLE_TIMER_RUNNING         (0x02u)
#define Serial_USB_IDLE_TIMER_EXPIRED         (0x01u)
#define Serial_USB_IDLE_TIMER_INDEFINITE      (0x00u)

#define Serial_USB_DEVICE_STATUS_BUS_POWERED  (0x00u)
#define Serial_USB_DEVICE_STATUS_SELF_POWERED (0x01u)

#define Serial_USB_3V_OPERATION               (0x00u)
#define Serial_USB_5V_OPERATION               (0x01u)
#define Serial_USB_DWR_POWER_OPERATION        (0x02u)

#define Serial_USB_MODE_DISABLE               (0x00u)
#define Serial_USB_MODE_NAK_IN_OUT            (0x01u)
#define Serial_USB_MODE_STATUS_OUT_ONLY       (0x02u)
#define Serial_USB_MODE_STALL_IN_OUT          (0x03u)
#define Serial_USB_MODE_RESERVED_0100         (0x04u)
#define Serial_USB_MODE_ISO_OUT               (0x05u)
#define Serial_USB_MODE_STATUS_IN_ONLY        (0x06u)
#define Serial_USB_MODE_ISO_IN                (0x07u)
#define Serial_USB_MODE_NAK_OUT               (0x08u)
#define Serial_USB_MODE_ACK_OUT               (0x09u)
#define Serial_USB_MODE_RESERVED_1010         (0x0Au)
#define Serial_USB_MODE_ACK_OUT_STATUS_IN     (0x0Bu)
#define Serial_USB_MODE_NAK_IN                (0x0Cu)
#define Serial_USB_MODE_ACK_IN                (0x0Du)
#define Serial_USB_MODE_RESERVED_1110         (0x0Eu)
#define Serial_USB_MODE_ACK_IN_STATUS_OUT     (0x0Fu)
#define Serial_USB_MODE_MASK                  (0x0Fu)
#define Serial_USB_MODE_STALL_DATA_EP         (0x80u)

#define Serial_USB_MODE_ACKD                  (0x10u)
#define Serial_USB_MODE_OUT_RCVD              (0x20u)
#define Serial_USB_MODE_IN_RCVD               (0x40u)
#define Serial_USB_MODE_SETUP_RCVD            (0x80u)

#define Serial_USB_RQST_TYPE_MASK             (0x60u)
#define Serial_USB_RQST_TYPE_STD              (0x00u)
#define Serial_USB_RQST_TYPE_CLS              (0x20u)
#define Serial_USB_RQST_TYPE_VND              (0x40u)
#define Serial_USB_RQST_DIR_MASK              (0x80u)
#define Serial_USB_RQST_DIR_D2H               (0x80u)
#define Serial_USB_RQST_DIR_H2D               (0x00u)
#define Serial_USB_RQST_RCPT_MASK             (0x03u)
#define Serial_USB_RQST_RCPT_DEV              (0x00u)
#define Serial_USB_RQST_RCPT_IFC              (0x01u)
#define Serial_USB_RQST_RCPT_EP               (0x02u)
#define Serial_USB_RQST_RCPT_OTHER            (0x03u)

#if (Serial_USB_LPM_ACTIVE)
    #define Serial_USB_LPM_REQ_ACK            (0x01u << Serial_USB_LPM_CTRL_LPM_ACK_RESP_POS)
    #define Serial_USB_LPM_REQ_NACK           (0x00u)
    #define Serial_USB_LPM_REQ_NYET           (0x01u << Serial_USB_LPM_CTRL_NYET_EN_POS)
#endif /*(Serial_USB_LPM_ACTIVE)*/

/* USB Class Codes */
#define Serial_USB_CLASS_DEVICE               (0x00u)     /* Use class code info from Interface Descriptors */
#define Serial_USB_CLASS_AUDIO                (0x01u)     /* Audio device */
#define Serial_USB_CLASS_CDC                  (0x02u)     /* Communication device class */
#define Serial_USB_CLASS_HID                  (0x03u)     /* Human Interface Device */
#define Serial_USB_CLASS_PDC                  (0x05u)     /* Physical device class */
#define Serial_USB_CLASS_IMAGE                (0x06u)     /* Still Imaging device */
#define Serial_USB_CLASS_PRINTER              (0x07u)     /* Printer device  */
#define Serial_USB_CLASS_MSD                  (0x08u)     /* Mass Storage device  */
#define Serial_USB_CLASS_HUB                  (0x09u)     /* Full/Hi speed Hub */
#define Serial_USB_CLASS_CDC_DATA             (0x0Au)     /* CDC data device */
#define Serial_USB_CLASS_SMART_CARD           (0x0Bu)     /* Smart Card device */
#define Serial_USB_CLASS_CSD                  (0x0Du)     /* Content Security device */
#define Serial_USB_CLASS_VIDEO                (0x0Eu)     /* Video device */
#define Serial_USB_CLASS_PHD                  (0x0Fu)     /* Personal Health care device */
#define Serial_USB_CLASS_WIRELESSD            (0xDCu)     /* Wireless Controller */
#define Serial_USB_CLASS_MIS                  (0xE0u)     /* Miscellaneous */
#define Serial_USB_CLASS_APP                  (0xEFu)     /* Application Specific */
#define Serial_USB_CLASS_VENDOR               (0xFFu)     /* Vendor specific */

/* Standard Request Types (Table 9-4) */
#define Serial_USB_GET_STATUS                 (0x00u)
#define Serial_USB_CLEAR_FEATURE              (0x01u)
#define Serial_USB_SET_FEATURE                (0x03u)
#define Serial_USB_SET_ADDRESS                (0x05u)
#define Serial_USB_GET_DESCRIPTOR             (0x06u)
#define Serial_USB_SET_DESCRIPTOR             (0x07u)
#define Serial_USB_GET_CONFIGURATION          (0x08u)
#define Serial_USB_SET_CONFIGURATION          (0x09u)
#define Serial_USB_GET_INTERFACE              (0x0Au)
#define Serial_USB_SET_INTERFACE              (0x0Bu)
#define Serial_USB_SYNCH_FRAME                (0x0Cu)

/* Vendor Specific Request Types */
/* Request for Microsoft OS String Descriptor */
#define Serial_USB_GET_EXTENDED_CONFIG_DESCRIPTOR (0x01u)

/* Descriptor Types (Table 9-5) */
#define Serial_USB_DESCR_DEVICE                   (1u)
#define Serial_USB_DESCR_CONFIG                   (2u)
#define Serial_USB_DESCR_STRING                   (3u)
#define Serial_USB_DESCR_INTERFACE                (4u)
#define Serial_USB_DESCR_ENDPOINT                 (5u)
#define Serial_USB_DESCR_DEVICE_QUALIFIER         (6u)
#define Serial_USB_DESCR_OTHER_SPEED              (7u)
#define Serial_USB_DESCR_INTERFACE_POWER          (8u)
#if (Serial_USB_BOS_ENABLE)
    #define Serial_USB_DESCR_BOS                  (15u)
#endif /* (Serial_USB_BOS_ENABLE) */
/* Device Descriptor Defines */
#define Serial_USB_DEVICE_DESCR_LENGTH            (18u)
#define Serial_USB_DEVICE_DESCR_SN_SHIFT          (16u)

/* Config Descriptor Shifts and Masks */
#define Serial_USB_CONFIG_DESCR_LENGTH                (0u)
#define Serial_USB_CONFIG_DESCR_TYPE                  (1u)
#define Serial_USB_CONFIG_DESCR_TOTAL_LENGTH_LOW      (2u)
#define Serial_USB_CONFIG_DESCR_TOTAL_LENGTH_HI       (3u)
#define Serial_USB_CONFIG_DESCR_NUM_INTERFACES        (4u)
#define Serial_USB_CONFIG_DESCR_CONFIG_VALUE          (5u)
#define Serial_USB_CONFIG_DESCR_CONFIGURATION         (6u)
#define Serial_USB_CONFIG_DESCR_ATTRIB                (7u)
#define Serial_USB_CONFIG_DESCR_ATTRIB_SELF_POWERED   (0x40u)
#define Serial_USB_CONFIG_DESCR_ATTRIB_RWU_EN         (0x20u)

#if (Serial_USB_BOS_ENABLE)
    /* Config Descriptor BOS */
    #define Serial_USB_BOS_DESCR_LENGTH               (0u)
    #define Serial_USB_BOS_DESCR_TYPE                 (1u)
    #define Serial_USB_BOS_DESCR_TOTAL_LENGTH_LOW     (2u)
    #define Serial_USB_BOS_DESCR_TOTAL_LENGTH_HI      (3u)
    #define Serial_USB_BOS_DESCR_NUM_DEV_CAPS         (4u)
#endif /* (Serial_USB_BOS_ENABLE) */

/* Feature Selectors (Table 9-6) */
#define Serial_USB_DEVICE_REMOTE_WAKEUP           (0x01u)
#define Serial_USB_ENDPOINT_HALT                  (0x00u)
#define Serial_USB_TEST_MODE                      (0x02u)

/* USB Device Status (Figure 9-4) */
#define Serial_USB_DEVICE_STATUS_BUS_POWERED      (0x00u)
#define Serial_USB_DEVICE_STATUS_SELF_POWERED     (0x01u)
#define Serial_USB_DEVICE_STATUS_REMOTE_WAKEUP    (0x02u)

/* USB Endpoint Status (Figure 9-4) */
#define Serial_USB_ENDPOINT_STATUS_HALT           (0x01u)

/* USB Endpoint Directions */
#define Serial_USB_DIR_IN                         (0x80u)
#define Serial_USB_DIR_OUT                        (0x00u)
#define Serial_USB_DIR_UNUSED                     (0x7Fu)

/* USB Endpoint Attributes */
#define Serial_USB_EP_TYPE_CTRL                   (0x00u)
#define Serial_USB_EP_TYPE_ISOC                   (0x01u)
#define Serial_USB_EP_TYPE_BULK                   (0x02u)
#define Serial_USB_EP_TYPE_INT                    (0x03u)
#define Serial_USB_EP_TYPE_MASK                   (0x03u)

#define Serial_USB_EP_SYNC_TYPE_NO_SYNC           (0x00u)
#define Serial_USB_EP_SYNC_TYPE_ASYNC             (0x04u)
#define Serial_USB_EP_SYNC_TYPE_ADAPTIVE          (0x08u)
#define Serial_USB_EP_SYNC_TYPE_SYNCHRONOUS       (0x0Cu)
#define Serial_USB_EP_SYNC_TYPE_MASK              (0x0Cu)

#define Serial_USB_EP_USAGE_TYPE_DATA             (0x00u)
#define Serial_USB_EP_USAGE_TYPE_FEEDBACK         (0x10u)
#define Serial_USB_EP_USAGE_TYPE_IMPLICIT         (0x20u)
#define Serial_USB_EP_USAGE_TYPE_RESERVED         (0x30u)
#define Serial_USB_EP_USAGE_TYPE_MASK             (0x30u)

/* Point Status defines */
#define Serial_USB_EP_STATUS_LENGTH               (0x02u)

/* Point Device defines */
#define Serial_USB_DEVICE_STATUS_LENGTH           (0x02u)

#define Serial_USB_STATUS_LENGTH_MAX \
                 ((Serial_USB_EP_STATUS_LENGTH > Serial_USB_DEVICE_STATUS_LENGTH) ? \
                        Serial_USB_EP_STATUS_LENGTH : Serial_USB_DEVICE_STATUS_LENGTH)

/* Transfer Completion Notification */
#define Serial_USB_XFER_IDLE                      (0x00u)
#define Serial_USB_XFER_STATUS_ACK                (0x01u)
#define Serial_USB_XFER_PREMATURE                 (0x02u)
#define Serial_USB_XFER_ERROR                     (0x03u)

/* Driver State defines */
#define Serial_USB_TRANS_STATE_IDLE               (0x00u)
#define Serial_USB_TRANS_STATE_CONTROL_READ       (0x02u)
#define Serial_USB_TRANS_STATE_CONTROL_WRITE      (0x04u)
#define Serial_USB_TRANS_STATE_NO_DATA_CONTROL    (0x06u)

/* String Descriptor defines */
#define Serial_USB_STRING_MSOS                    (0xEEu)
#define Serial_USB_MSOS_DESCRIPTOR_LENGTH         (18u)
#define Serial_USB_MSOS_CONF_DESCR_LENGTH         (40u)

/* Return values */
#define Serial_USB_BCD_PORT_SDP       (1u) /* Standard downstream port detected */
#define Serial_USB_BCD_PORT_CDP       (2u) /* Charging downstream port detected */
#define Serial_USB_BCD_PORT_DCP       (3u) /* Dedicated charging port detected */
#define Serial_USB_BCD_PORT_UNKNOWN   (0u) /* Unable to detect charging port */
#define Serial_USB_BCD_PORT_ERR       (4u) /* Error condition in detection process*/


/* Timeouts for BCD */
#define Serial_USB_BCD_TIMEOUT                (400u)  /* Copied from PBK#163 TIMEOUT (300 ms) */
#define Serial_USB_BCD_TDCD_DBNC              (10u)  /*BCD v1.2: DCD debounce time 10 ms*/
#define Serial_USB_BCD_PRIMARY_WAIT           (40u)   /* Copied from PBK#163 TIMEOUT (40 ms)  */
#define Serial_USB_BCD_SECONDARY_WAIT         (47u)   /* Copied from PBK#163 TIMEOUT (40 ms)  */
#define Serial_USB_BCD_SUSPEND_DISABLE_WAIT   (2u)    /* Copied from PBK#163 TIMEOUT (2 us)   */

/* Wait cycles required before clearing SUSPEND_DEL in POWER_CTRL: 2us */
#define Serial_USB_WAIT_SUSPEND_DEL_DISABLE   (2u)

/* Wait cycles required for USB regulator stabilization after it is enabled : 50ns */
#define Serial_USB_WAIT_VREF_STABILITY        (0u)

#if (CY_PSOC3 || CY_PSOC5LP)
/* Wait cycles required for USB reference restore: 40us */
#define Serial_USB_WAIT_VREF_RESTORE          (40u)

/* Wait cycles required for stabilization after register is written : 50ns */
#define Serial_USB_WAIT_REG_STABILITY_50NS    (0u)
#define Serial_USB_WAIT_REG_STABILITY_1US     (1u)

/* Wait cycles required after CR0 register write: 1 cycle */
#define Serial_USB_WAIT_CR0_REG_STABILITY     (1u)

/* Wait cycles required after PD_PULLUP_N bit is set in PM_USB_CR0: 2us */
#define Serial_USB_WAIT_PD_PULLUP_N_ENABLE    (2u)
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

#if (CY_PSOC4)
    #if (Serial_USB_EP_MANAGEMENT_DMA)
        #define Serial_USB_DMA_DESCR0         (0u)
        #define Serial_USB_DMA_DESCR1         (1u)
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA) */

    #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        /* BUF_SIZE-BYTES_PER_BURST examples: 0x55 - 32 bytes, 0x44 - 16 bytes, 0x33 - 8 bytes, etc. */
        #define Serial_USB_DMA_BUF_SIZE             (0x55u)
        #define Serial_USB_DMA_BYTES_PER_BURST      (32u)
        #define Serial_USB_DMA_HALFWORDS_PER_BURST  (16u)
        #define Serial_USB_DMA_BURST_BYTES_MASK     (Serial_USB_DMA_BYTES_PER_BURST - 1u)

        #define Serial_USB_DMA_DESCR0_MASK        (0x00u)
        #define Serial_USB_DMA_DESCR1_MASK        (0x80u)
        #define Serial_USB_DMA_DESCR_REVERT       (0x40u)
        #define Serial_USB_DMA_DESCR_16BITS       (0x20u)
        #define Serial_USB_DMA_DESCR_SHIFT        (7u)

        #define Serial_USB_DMA_GET_DESCR_NUM(desrc)
        #define Serial_USB_DMA_GET_BURST_CNT(dmaBurstCnt) \
                    (((dmaBurstCnt) > 2u)? ((dmaBurstCnt) - 2u) : 0u)

        #define Serial_USB_DMA_GET_MAX_ELEM_PER_BURST(dmaLastBurstEl) \
                    ((0u != ((dmaLastBurstEl) & Serial_USB_DMA_DESCR_16BITS)) ? \
                                (Serial_USB_DMA_HALFWORDS_PER_BURST - 1u) : (Serial_USB_DMA_BYTES_PER_BURST - 1u))
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
#else
    #if (Serial_USB_EP_MANAGEMENT_DMA_MANUAL)
        #define Serial_USB_DMA_BYTES_PER_BURST    (0u)
        #define Serial_USB_DMA_REQUEST_PER_BURST  (0u)
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_MANUAL) */

    #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        #define Serial_USB_DMA_BYTES_PER_BURST    (32u)
        #define Serial_USB_DMA_BYTES_REPEAT       (2u)

        /* BUF_SIZE-BYTES_PER_BURST examples: 0x55 - 32 bytes, 0x44 - 16 bytes, 0x33 - 8 bytes, etc. */
        #define Serial_USB_DMA_BUF_SIZE           (0x55u)
        #define Serial_USB_DMA_REQUEST_PER_BURST  (1u)

        #define Serial_USB_EP17_SR_MASK           (0x7Fu)
        #define Serial_USB_EP8_SR_MASK            (0x03u)
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */
#endif /* (CY_PSOC4) */

/* DIE ID string descriptor defines */
#if defined(Serial_USB_ENABLE_IDSN_STRING)
    #define Serial_USB_IDSN_DESCR_LENGTH      (0x22u)
#endif /* (Serial_USB_ENABLE_IDSN_STRING) */


/***************************************
*     Vars with External Linkage
***************************************/

/**
* \addtogroup group_globals
* @{
*/
extern uint8 Serial_USB_initVar;
extern volatile uint8 Serial_USB_device;
extern volatile uint8 Serial_USB_transferState;
extern volatile uint8 Serial_USB_configuration;
extern volatile uint8 Serial_USB_configurationChanged;
extern volatile uint8 Serial_USB_deviceStatus;
/** @} globals */

/**
* \addtogroup group_hid
* @{
*/
/* HID Variables */
#if defined(Serial_USB_ENABLE_HID_CLASS)
    extern volatile uint8 Serial_USB_hidProtocol [Serial_USB_MAX_INTERFACES_NUMBER]; 
    extern volatile uint8 Serial_USB_hidIdleRate [Serial_USB_MAX_INTERFACES_NUMBER];
    extern volatile uint8 Serial_USB_hidIdleTimer[Serial_USB_MAX_INTERFACES_NUMBER];
#endif /* (Serial_USB_ENABLE_HID_CLASS) */
/** @} hid */


/***************************************
*              Registers
***************************************/

/* Common registers for all PSoCs: 3/4/5LP */
#define Serial_USB_ARB_CFG_PTR        ( (reg8 *) Serial_USB_USB__ARB_CFG)
#define Serial_USB_ARB_CFG_REG        (*(reg8 *) Serial_USB_USB__ARB_CFG)

#define Serial_USB_ARB_EP1_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP1_CFG)
#define Serial_USB_ARB_EP1_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP1_CFG)
#define Serial_USB_ARB_EP1_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP1_INT_EN)
#define Serial_USB_ARB_EP1_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP1_INT_EN)
#define Serial_USB_ARB_EP1_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP1_SR)
#define Serial_USB_ARB_EP1_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP1_SR)
#define Serial_USB_ARB_EP1_CFG_IND    Serial_USB_USB__ARB_EP1_CFG
#define Serial_USB_ARB_EP1_INT_EN_IND Serial_USB_USB__ARB_EP1_INT_EN
#define Serial_USB_ARB_EP1_SR_IND     Serial_USB_USB__ARB_EP1_SR
#define Serial_USB_ARB_EP_BASE        (*(volatile Serial_USB_arb_eps_struct CYXDATA *) \
                                            (Serial_USB_USB__ARB_EP1_CFG - sizeof(Serial_USB_arb_ep_struct)))

#define Serial_USB_ARB_EP2_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP2_CFG)
#define Serial_USB_ARB_EP2_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP2_CFG)
#define Serial_USB_ARB_EP2_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP2_INT_EN)
#define Serial_USB_ARB_EP2_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP2_INT_EN)
#define Serial_USB_ARB_EP2_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP2_SR)
#define Serial_USB_ARB_EP2_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP2_SR)

#define Serial_USB_ARB_EP3_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP3_CFG)
#define Serial_USB_ARB_EP3_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP3_CFG)
#define Serial_USB_ARB_EP3_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP3_INT_EN)
#define Serial_USB_ARB_EP3_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP3_INT_EN)
#define Serial_USB_ARB_EP3_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP3_SR)
#define Serial_USB_ARB_EP3_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP3_SR)

#define Serial_USB_ARB_EP4_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP4_CFG)
#define Serial_USB_ARB_EP4_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP4_CFG)
#define Serial_USB_ARB_EP4_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP4_INT_EN)
#define Serial_USB_ARB_EP4_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP4_INT_EN)
#define Serial_USB_ARB_EP4_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP4_SR)
#define Serial_USB_ARB_EP4_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP4_SR)

#define Serial_USB_ARB_EP5_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP5_CFG)
#define Serial_USB_ARB_EP5_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP5_CFG)
#define Serial_USB_ARB_EP5_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP5_INT_EN)
#define Serial_USB_ARB_EP5_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP5_INT_EN)
#define Serial_USB_ARB_EP5_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP5_SR)
#define Serial_USB_ARB_EP5_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP5_SR)

#define Serial_USB_ARB_EP6_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP6_CFG)
#define Serial_USB_ARB_EP6_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP6_CFG)
#define Serial_USB_ARB_EP6_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP6_INT_EN)
#define Serial_USB_ARB_EP6_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP6_INT_EN)
#define Serial_USB_ARB_EP6_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP6_SR)
#define Serial_USB_ARB_EP6_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP6_SR)

#define Serial_USB_ARB_EP7_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP7_CFG)
#define Serial_USB_ARB_EP7_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP7_CFG)
#define Serial_USB_ARB_EP7_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP7_INT_EN)
#define Serial_USB_ARB_EP7_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP7_INT_EN)
#define Serial_USB_ARB_EP7_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP7_SR)
#define Serial_USB_ARB_EP7_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP7_SR)

#define Serial_USB_ARB_EP8_CFG_PTR    ( (reg8 *) Serial_USB_USB__ARB_EP8_CFG)
#define Serial_USB_ARB_EP8_CFG_REG    (*(reg8 *) Serial_USB_USB__ARB_EP8_CFG)
#define Serial_USB_ARB_EP8_INT_EN_PTR ( (reg8 *) Serial_USB_USB__ARB_EP8_INT_EN)
#define Serial_USB_ARB_EP8_INT_EN_REG (*(reg8 *) Serial_USB_USB__ARB_EP8_INT_EN)
#define Serial_USB_ARB_EP8_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_EP8_SR)
#define Serial_USB_ARB_EP8_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_EP8_SR)

#define Serial_USB_ARB_INT_EN_PTR     ( (reg8 *) Serial_USB_USB__ARB_INT_EN)
#define Serial_USB_ARB_INT_EN_REG     (*(reg8 *) Serial_USB_USB__ARB_INT_EN)
#define Serial_USB_ARB_INT_SR_PTR     ( (reg8 *) Serial_USB_USB__ARB_INT_SR)
#define Serial_USB_ARB_INT_SR_REG     (*(reg8 *) Serial_USB_USB__ARB_INT_SR)

#define Serial_USB_ARB_RW1_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW1_DR)
#define Serial_USB_ARB_RW1_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW1_RA)

#define Serial_USB_ARB_RW1_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW1_RA_MSB)
#define Serial_USB_ARB_RW1_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW1_WA)
#define Serial_USB_ARB_RW1_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW1_WA_MSB)
#define Serial_USB_ARB_RW1_DR_IND     Serial_USB_USB__ARB_RW1_DR
#define Serial_USB_ARB_RW1_RA_IND     Serial_USB_USB__ARB_RW1_RA
#define Serial_USB_ARB_RW1_RA_MSB_IND Serial_USB_USB__ARB_RW1_RA_MSB
#define Serial_USB_ARB_RW1_WA_IND     Serial_USB_USB__ARB_RW1_WA
#define Serial_USB_ARB_RW1_WA_MSB_IND Serial_USB_USB__ARB_RW1_WA_MSB

#define Serial_USB_ARB_RW2_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW2_DR)
#define Serial_USB_ARB_RW2_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW2_RA)
#define Serial_USB_ARB_RW2_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW2_RA_MSB)
#define Serial_USB_ARB_RW2_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW2_WA)
#define Serial_USB_ARB_RW2_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW2_WA_MSB)

#define Serial_USB_ARB_RW3_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW3_DR)
#define Serial_USB_ARB_RW3_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW3_RA)
#define Serial_USB_ARB_RW3_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW3_RA_MSB)
#define Serial_USB_ARB_RW3_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW3_WA)
#define Serial_USB_ARB_RW3_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW3_WA_MSB)

#define Serial_USB_ARB_RW4_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW4_DR)
#define Serial_USB_ARB_RW4_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW4_RA)
#define Serial_USB_ARB_RW4_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW4_RA_MSB)
#define Serial_USB_ARB_RW4_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW4_WA)
#define Serial_USB_ARB_RW4_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW4_WA_MSB)

#define Serial_USB_ARB_RW5_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW5_DR)
#define Serial_USB_ARB_RW5_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW5_RA)
#define Serial_USB_ARB_RW5_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW5_RA_MSB)
#define Serial_USB_ARB_RW5_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW5_WA)
#define Serial_USB_ARB_RW5_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW5_WA_MSB)

#define Serial_USB_ARB_RW6_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW6_DR)
#define Serial_USB_ARB_RW6_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW6_RA)
#define Serial_USB_ARB_RW6_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW6_RA_MSB)
#define Serial_USB_ARB_RW6_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW6_WA)
#define Serial_USB_ARB_RW6_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW6_WA_MSB)

#define Serial_USB_ARB_RW7_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW7_DR)
#define Serial_USB_ARB_RW7_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW7_RA)
#define Serial_USB_ARB_RW7_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW7_RA_MSB)
#define Serial_USB_ARB_RW7_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW7_WA)
#define Serial_USB_ARB_RW7_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW7_WA_MSB)

#define Serial_USB_ARB_RW8_DR_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW8_DR)
#define Serial_USB_ARB_RW8_RA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW8_RA)
#define Serial_USB_ARB_RW8_RA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW8_RA_MSB)
#define Serial_USB_ARB_RW8_WA_PTR     ( (reg8 *) Serial_USB_USB__ARB_RW8_WA)
#define Serial_USB_ARB_RW8_WA_MSB_PTR ( (reg8 *) Serial_USB_USB__ARB_RW8_WA_MSB)

#define Serial_USB_BUF_SIZE_PTR       ( (reg8 *) Serial_USB_USB__BUF_SIZE)
#define Serial_USB_BUF_SIZE_REG       (*(reg8 *) Serial_USB_USB__BUF_SIZE)
#define Serial_USB_BUS_RST_CNT_PTR    ( (reg8 *) Serial_USB_USB__BUS_RST_CNT)
#define Serial_USB_BUS_RST_CNT_REG    (*(reg8 *) Serial_USB_USB__BUS_RST_CNT)
#define Serial_USB_CWA_PTR            ( (reg8 *) Serial_USB_USB__CWA)
#define Serial_USB_CWA_REG            (*(reg8 *) Serial_USB_USB__CWA)
#define Serial_USB_CWA_MSB_PTR        ( (reg8 *) Serial_USB_USB__CWA_MSB)
#define Serial_USB_CWA_MSB_REG        (*(reg8 *) Serial_USB_USB__CWA_MSB)
#define Serial_USB_CR0_PTR            ( (reg8 *) Serial_USB_USB__CR0)
#define Serial_USB_CR0_REG            (*(reg8 *) Serial_USB_USB__CR0)
#define Serial_USB_CR1_PTR            ( (reg8 *) Serial_USB_USB__CR1)
#define Serial_USB_CR1_REG            (*(reg8 *) Serial_USB_USB__CR1)

#define Serial_USB_DMA_THRES_PTR      ( (reg8 *) Serial_USB_USB__DMA_THRES)
#define Serial_USB_DMA_THRES_REG      (*(reg8 *) Serial_USB_USB__DMA_THRES)
#define Serial_USB_DMA_THRES_MSB_PTR  ( (reg8 *) Serial_USB_USB__DMA_THRES_MSB)
#define Serial_USB_DMA_THRES_MSB_REG  (*(reg8 *) Serial_USB_USB__DMA_THRES_MSB)

#define Serial_USB_EP_ACTIVE_PTR      ( (reg8 *) Serial_USB_USB__EP_ACTIVE)
#define Serial_USB_EP_ACTIVE_REG      (*(reg8 *) Serial_USB_USB__EP_ACTIVE)
#define Serial_USB_EP_TYPE_PTR        ( (reg8 *) Serial_USB_USB__EP_TYPE)
#define Serial_USB_EP_TYPE_REG        (*(reg8 *) Serial_USB_USB__EP_TYPE)

#define Serial_USB_EP0_CNT_PTR        ( (reg8 *) Serial_USB_USB__EP0_CNT)
#define Serial_USB_EP0_CNT_REG        (*(reg8 *) Serial_USB_USB__EP0_CNT)
#define Serial_USB_EP0_CR_PTR         ( (reg8 *) Serial_USB_USB__EP0_CR)
#define Serial_USB_EP0_CR_REG         (*(reg8 *) Serial_USB_USB__EP0_CR)
#define Serial_USB_EP0_DR0_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR0)
#define Serial_USB_EP0_DR0_REG        (*(reg8 *) Serial_USB_USB__EP0_DR0)
#define Serial_USB_EP0_DR1_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR1)
#define Serial_USB_EP0_DR1_REG        (*(reg8 *) Serial_USB_USB__EP0_DR1)
#define Serial_USB_EP0_DR2_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR2)
#define Serial_USB_EP0_DR2_REG        (*(reg8 *) Serial_USB_USB__EP0_DR2)
#define Serial_USB_EP0_DR3_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR3)
#define Serial_USB_EP0_DR3_REG        (*(reg8 *) Serial_USB_USB__EP0_DR3)
#define Serial_USB_EP0_DR4_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR4)
#define Serial_USB_EP0_DR4_REG        (*(reg8 *) Serial_USB_USB__EP0_DR4)
#define Serial_USB_EP0_DR5_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR5)
#define Serial_USB_EP0_DR5_REG        (*(reg8 *) Serial_USB_USB__EP0_DR5)
#define Serial_USB_EP0_DR6_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR6)
#define Serial_USB_EP0_DR6_REG        (*(reg8 *) Serial_USB_USB__EP0_DR6)
#define Serial_USB_EP0_DR7_PTR        ( (reg8 *) Serial_USB_USB__EP0_DR7)
#define Serial_USB_EP0_DR7_REG        (*(reg8 *) Serial_USB_USB__EP0_DR7)
#define Serial_USB_EP0_DR0_IND        Serial_USB_USB__EP0_DR0
#define Serial_USB_EP0_DR_BASE        (*(volatile Serial_USB_ep0_data_struct CYXDATA *) Serial_USB_USB__EP0_DR0)

#define Serial_USB_OSCLK_DR0_PTR      ( (reg8 *) Serial_USB_USB__OSCLK_DR0)
#define Serial_USB_OSCLK_DR0_REG      (*(reg8 *) Serial_USB_USB__OSCLK_DR0)
#define Serial_USB_OSCLK_DR1_PTR      ( (reg8 *) Serial_USB_USB__OSCLK_DR1)
#define Serial_USB_OSCLK_DR1_REG      (*(reg8 *) Serial_USB_USB__OSCLK_DR1)

#define Serial_USB_SIE_EP_INT_EN_PTR  ( (reg8 *) Serial_USB_USB__SIE_EP_INT_EN)
#define Serial_USB_SIE_EP_INT_EN_REG  (*(reg8 *) Serial_USB_USB__SIE_EP_INT_EN)
#define Serial_USB_SIE_EP_INT_SR_PTR  ( (reg8 *) Serial_USB_USB__SIE_EP_INT_SR)
#define Serial_USB_SIE_EP_INT_SR_REG  (*(reg8 *) Serial_USB_USB__SIE_EP_INT_SR)

#define Serial_USB_SIE_EP1_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP1_CNT0)
#define Serial_USB_SIE_EP1_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP1_CNT0)
#define Serial_USB_SIE_EP1_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP1_CNT1)
#define Serial_USB_SIE_EP1_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP1_CNT1)
#define Serial_USB_SIE_EP1_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP1_CR0)
#define Serial_USB_SIE_EP1_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP1_CR0)
#define Serial_USB_SIE_EP1_CNT1_IND   Serial_USB_USB__SIE_EP1_CNT1
#define Serial_USB_SIE_EP1_CNT0_IND   Serial_USB_USB__SIE_EP1_CNT0
#define Serial_USB_SIE_EP1_CR0_IND    Serial_USB_USB__SIE_EP1_CR0
#define Serial_USB_SIE_EP_BASE        (*(volatile Serial_USB_sie_eps_struct CYXDATA *) \
                                            (Serial_USB_USB__SIE_EP1_CNT0 - sizeof(Serial_USB_sie_ep_struct)))

#define Serial_USB_SIE_EP2_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP2_CNT0)
#define Serial_USB_SIE_EP2_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP2_CNT0)
#define Serial_USB_SIE_EP2_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP2_CNT1)
#define Serial_USB_SIE_EP2_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP2_CNT1)
#define Serial_USB_SIE_EP2_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP2_CR0)
#define Serial_USB_SIE_EP2_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP2_CR0)

#define Serial_USB_SIE_EP3_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP3_CNT0)
#define Serial_USB_SIE_EP3_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP3_CNT0)
#define Serial_USB_SIE_EP3_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP3_CNT1)
#define Serial_USB_SIE_EP3_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP3_CNT1)
#define Serial_USB_SIE_EP3_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP3_CR0)
#define Serial_USB_SIE_EP3_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP3_CR0)

#define Serial_USB_SIE_EP4_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP4_CNT0)
#define Serial_USB_SIE_EP4_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP4_CNT0)
#define Serial_USB_SIE_EP4_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP4_CNT1)
#define Serial_USB_SIE_EP4_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP4_CNT1)
#define Serial_USB_SIE_EP4_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP4_CR0)
#define Serial_USB_SIE_EP4_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP4_CR0)

#define Serial_USB_SIE_EP5_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP5_CNT0)
#define Serial_USB_SIE_EP5_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP5_CNT0)
#define Serial_USB_SIE_EP5_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP5_CNT1)
#define Serial_USB_SIE_EP5_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP5_CNT1)
#define Serial_USB_SIE_EP5_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP5_CR0)
#define Serial_USB_SIE_EP5_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP5_CR0)

#define Serial_USB_SIE_EP6_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP6_CNT0)
#define Serial_USB_SIE_EP6_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP6_CNT0)
#define Serial_USB_SIE_EP6_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP6_CNT1)
#define Serial_USB_SIE_EP6_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP6_CNT1)
#define Serial_USB_SIE_EP6_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP6_CR0)
#define Serial_USB_SIE_EP6_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP6_CR0)

#define Serial_USB_SIE_EP7_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP7_CNT0)
#define Serial_USB_SIE_EP7_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP7_CNT0)
#define Serial_USB_SIE_EP7_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP7_CNT1)
#define Serial_USB_SIE_EP7_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP7_CNT1)
#define Serial_USB_SIE_EP7_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP7_CR0)
#define Serial_USB_SIE_EP7_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP7_CR0)

#define Serial_USB_SIE_EP8_CNT0_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP8_CNT0)
#define Serial_USB_SIE_EP8_CNT0_REG   (*(reg8 *) Serial_USB_USB__SIE_EP8_CNT0)
#define Serial_USB_SIE_EP8_CNT1_PTR   ( (reg8 *) Serial_USB_USB__SIE_EP8_CNT1)
#define Serial_USB_SIE_EP8_CNT1_REG   (*(reg8 *) Serial_USB_USB__SIE_EP8_CNT1)
#define Serial_USB_SIE_EP8_CR0_PTR    ( (reg8 *) Serial_USB_USB__SIE_EP8_CR0)
#define Serial_USB_SIE_EP8_CR0_REG    (*(reg8 *) Serial_USB_USB__SIE_EP8_CR0)

#define Serial_USB_SOF0_PTR           ( (reg8 *) Serial_USB_USB__SOF0)
#define Serial_USB_SOF0_REG           (*(reg8 *) Serial_USB_USB__SOF0)
#define Serial_USB_SOF1_PTR           ( (reg8 *) Serial_USB_USB__SOF1)
#define Serial_USB_SOF1_REG           (*(reg8 *) Serial_USB_USB__SOF1)

#define Serial_USB_USB_CLK_EN_PTR     ( (reg8 *) Serial_USB_USB__USB_CLK_EN)
#define Serial_USB_USB_CLK_EN_REG     (*(reg8 *) Serial_USB_USB__USB_CLK_EN)

#define Serial_USB_USBIO_CR0_PTR      ( (reg8 *) Serial_USB_USB__USBIO_CR0)
#define Serial_USB_USBIO_CR0_REG      (*(reg8 *) Serial_USB_USB__USBIO_CR0)
#define Serial_USB_USBIO_CR1_PTR      ( (reg8 *) Serial_USB_USB__USBIO_CR1)
#define Serial_USB_USBIO_CR1_REG      (*(reg8 *) Serial_USB_USB__USBIO_CR1)

#define Serial_USB_DYN_RECONFIG_PTR   ( (reg8 *) Serial_USB_USB__DYN_RECONFIG)
#define Serial_USB_DYN_RECONFIG_REG   (*(reg8 *) Serial_USB_USB__DYN_RECONFIG)

#if (CY_PSOC4)
    #define Serial_USB_ARB_RW1_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW1_RA16)
    #define Serial_USB_ARB_RW1_RA16_REG   (*(reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW1_RA16)
    #define Serial_USB_ARB_RW1_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW1_WA16)
    #define Serial_USB_ARB_RW1_WA16_REG   (*(reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW1_WA16)
    #define Serial_USB_ARB_RW1_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW1_DR16)
    #define Serial_USB_ARB_RW1_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW1_DR16)
    #define Serial_USB_ARB_EP16_BASE      (*(volatile Serial_USB_arb_eps_reg16_struct CYXDATA *) \
                                                (Serial_USB_USB__ARB_RW1_WA16 - sizeof(Serial_USB_arb_ep_reg16_struct)))

    #define Serial_USB_ARB_RW2_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW2_DR16)
    #define Serial_USB_ARB_RW2_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW2_RA16)
    #define Serial_USB_ARB_RW2_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW2_WA16)

    #define Serial_USB_ARB_RW3_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW3_DR16)
    #define Serial_USB_ARB_RW3_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW3_RA16)
    #define Serial_USB_ARB_RW3_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW3_WA16)

    #define Serial_USB_ARB_RW4_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW4_DR16)
    #define Serial_USB_ARB_RW4_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW4_RA16)
    #define Serial_USB_ARB_RW4_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW4_WA16)

    #define Serial_USB_ARB_RW5_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW5_DR16)
    #define Serial_USB_ARB_RW5_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW5_RA16)
    #define Serial_USB_ARB_RW5_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW5_WA16)

    #define Serial_USB_ARB_RW6_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW6_DR16)
    #define Serial_USB_ARB_RW6_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW6_RA16)
    #define Serial_USB_ARB_RW6_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW6_WA16)

    #define Serial_USB_ARB_RW7_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW7_DR16)
    #define Serial_USB_ARB_RW7_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW7_RA16)
    #define Serial_USB_ARB_RW7_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW7_WA16)

    #define Serial_USB_ARB_RW8_DR16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW8_DR16)
    #define Serial_USB_ARB_RW8_RA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW8_RA16)
    #define Serial_USB_ARB_RW8_WA16_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__ARB_RW8_WA16)

    #define Serial_USB_OSCLK_DR16_PTR     ( (reg32 *) Serial_USB_cy_m0s8_usb__OSCLK_DR16)
    #define Serial_USB_OSCLK_DR16_REG     (*(reg32 *) Serial_USB_cy_m0s8_usb__OSCLK_DR16)

    #define Serial_USB_SOF16_PTR          ( (reg32 *) Serial_USB_cy_m0s8_usb__SOF16)
    #define Serial_USB_SOF16_REG          (*(reg32 *) Serial_USB_cy_m0s8_usb__SOF16)
    
    #define Serial_USB_CWA16_PTR          ( (reg32 *) Serial_USB_cy_m0s8_usb__CWA16)
    #define Serial_USB_CWA16_REG          (*(reg32 *) Serial_USB_cy_m0s8_usb__CWA16)

    #define Serial_USB_DMA_THRES16_PTR    ( (reg32 *) Serial_USB_cy_m0s8_usb__DMA_THRES16)
    #define Serial_USB_DMA_THRES16_REG    (*(reg32 *) Serial_USB_cy_m0s8_usb__DMA_THRES16)

    #define Serial_USB_USB_CLK_EN_PTR     ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_CLK_EN)
    #define Serial_USB_USB_CLK_EN_REG     (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_CLK_EN)

    #define Serial_USB_USBIO_CR2_PTR      ( (reg32 *) Serial_USB_cy_m0s8_usb__USBIO_CR2)
    #define Serial_USB_USBIO_CR2_REG      (*(reg32 *) Serial_USB_cy_m0s8_usb__USBIO_CR2)

    #define Serial_USB_USB_MEM            ( (reg32 *) Serial_USB_cy_m0s8_usb__MEM_DATA0)

    #define Serial_USB_POWER_CTRL_REG      (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_POWER_CTRL)
    #define Serial_USB_POWER_CTRL_PTR      ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_POWER_CTRL)

    #define Serial_USB_CHGDET_CTRL_REG     (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_CHGDET_CTRL)
    #define Serial_USB_CHGDET_CTRL_PTR     ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_CHGDET_CTRL)

    #define Serial_USB_USBIO_CTRL_REG      (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_USBIO_CTRL)
    #define Serial_USB_USBIO_CTRL_PTR      ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_USBIO_CTRL)

    #define Serial_USB_FLOW_CTRL_REG       (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_FLOW_CTRL)
    #define Serial_USB_FLOW_CTRL_PTR       ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_FLOW_CTRL)

    #define Serial_USB_LPM_CTRL_REG        (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_LPM_CTRL)
    #define Serial_USB_LPM_CTRL_PTR        ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_LPM_CTRL)

    #define Serial_USB_LPM_STAT_REG        (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_LPM_STAT)
    #define Serial_USB_LPM_STAT_PTR        ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_LPM_STAT)

    #define Serial_USB_PHY_CONTROL_REG     (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_PHY_CONTROL)
    #define Serial_USB_PHY_CONTROL_PTR     ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_PHY_CONTROL)

    #define Serial_USB_INTR_SIE_REG        (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE)
    #define Serial_USB_INTR_SIE_PTR        ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE)

    #define Serial_USB_INTR_SIE_SET_REG    (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE_SET)
    #define Serial_USB_INTR_SIE_SET_PTR    ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE_SET)

    #define Serial_USB_INTR_SIE_MASK_REG   (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE_MASK)
    #define Serial_USB_INTR_SIE_MASK_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE_MASK)

    #define Serial_USB_INTR_SIE_MASKED_REG (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE_MASKED)
    #define Serial_USB_INTR_SIE_MASKED_PTR ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_SIE_MASKED)

    #define Serial_USB_INTR_LVL_SEL_REG    (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_LVL_SEL)
    #define Serial_USB_INTR_LVL_SEL_PTR    ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_LVL_SEL)

    #define Serial_USB_INTR_CAUSE_HI_REG   (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_CAUSE_HI)
    #define Serial_USB_INTR_CAUSE_HI_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_CAUSE_HI)

    #define Serial_USB_INTR_CAUSE_LO_REG   (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_CAUSE_LO)
    #define Serial_USB_INTR_CAUSE_LO_PTR   ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_CAUSE_LO)

    #define Serial_USB_INTR_CAUSE_MED_REG  (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_CAUSE_MED)
    #define Serial_USB_INTR_CAUSE_MED_PTR  ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_INTR_CAUSE_MED)

    #define Serial_USB_DFT_CTRL_REG        (*(reg32 *) Serial_USB_cy_m0s8_usb__USB_DFT_CTRL)
    #define Serial_USB_DFT_CTRL_PTR        ( (reg32 *) Serial_USB_cy_m0s8_usb__USB_DFT_CTRL)

    #if (Serial_USB_VBUS_MONITORING_ENABLE)
        #if (Serial_USB_VBUS_POWER_PAD_ENABLE)
            /* Vbus power pad pin is hard wired to P13[2] */
            #define Serial_USB_VBUS_STATUS_REG    (*(reg32 *) CYREG_GPIO_PRT13_PS)
            #define Serial_USB_VBUS_STATUS_PTR    ( (reg32 *) CYREG_GPIO_PRT13_PS)
            #define Serial_USB_VBUS_VALID         (0x04u)
        #else
            /* Vbus valid pin is hard wired to P0[0] */
            #define Serial_USB_VBUS_STATUS_REG    (*(reg32 *) CYREG_GPIO_PRT0_PS)
            #define Serial_USB_VBUS_STATUS_PTR    ( (reg32 *) CYREG_GPIO_PRT0_PS)
            #define Serial_USB_VBUS_VALID         (0x01u)
        #endif
    #endif /*(Serial_USB_VBUS_MONITORING_ENABLE) */

    #define Serial_USB_BURSTEND_0_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND0_TR_OUTPUT)
    #define Serial_USB_BURSTEND_1_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND1_TR_OUTPUT)
    #define Serial_USB_BURSTEND_2_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND2_TR_OUTPUT)
    #define Serial_USB_BURSTEND_3_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND3_TR_OUTPUT)
    #define Serial_USB_BURSTEND_4_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND4_TR_OUTPUT)
    #define Serial_USB_BURSTEND_5_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND5_TR_OUTPUT)
    #define Serial_USB_BURSTEND_6_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND6_TR_OUTPUT)
    #define Serial_USB_BURSTEND_7_TR_OUTPUT    (Serial_USB_cy_m0s8_usb__BURSTEND7_TR_OUTPUT)
    
#else /* (CY_PSOC3 || CY_PSOC5LP) */

    /* Serial_USB_PM_USB_CR0 */
    #define Serial_USB_PM_USB_CR0_PTR     ( (reg8 *) CYREG_PM_USB_CR0)
    #define Serial_USB_PM_USB_CR0_REG     (*(reg8 *) CYREG_PM_USB_CR0)

    /* Serial_USB_PM_ACT/STBY_CFG */
    #define Serial_USB_PM_ACT_CFG_PTR     ( (reg8 *) Serial_USB_USB__PM_ACT_CFG)
    #define Serial_USB_PM_ACT_CFG_REG     (*(reg8 *) Serial_USB_USB__PM_ACT_CFG)
    #define Serial_USB_PM_STBY_CFG_PTR    ( (reg8 *) Serial_USB_USB__PM_STBY_CFG)
    #define Serial_USB_PM_STBY_CFG_REG    (*(reg8 *) Serial_USB_USB__PM_STBY_CFG)

    #if (!CY_PSOC5LP)
        #define Serial_USB_USBIO_CR2_PTR  (  (reg8 *) Serial_USB_USB__USBIO_CR2)
        #define Serial_USB_USBIO_CR2_REG  (* (reg8 *) Serial_USB_USB__USBIO_CR2)
    #endif /* (!CY_PSOC5LP) */

    /* Serial_USB_USB_MEM - USB IP memory buffer */
    #define Serial_USB_USB_MEM            ((reg8 *) CYDEV_USB_MEM_BASE)

    #if (Serial_USB_VBUS_MONITORING_ENABLE)
        #if (Serial_USB_VBUS_MONITORING_INTERNAL)
            #define Serial_USB_VBUS_STATUS_REG    (*(reg8 *) Serial_USB_VBUS__PS)
            #define Serial_USB_VBUS_STATUS_PTR    ( (reg8 *) Serial_USB_VBUS__PS)
            #define Serial_USB_VBUS_VALID         (Serial_USB_VBUS__MASK)
        #else
            #define Serial_USB_VBUS_STATUS_REG    (*(reg8 *) Serial_USB_Vbus_ps_sts_sts_reg__STATUS_REG)
            #define Serial_USB_VBUS_STATUS_PTR    ( (reg8 *) Serial_USB_Vbus_ps_sts_sts_reg__STATUS_REG)
            #define Serial_USB_VBUS_VALID         (Serial_USB_Vbus_ps_sts_sts_reg__MASK)
        #endif /* (Serial_USB_VBUS_MONITORING_INTERNAL) */
    #endif /*(Serial_USB_VBUS_MONITORING_ENABLE) */
#endif /* (CY_PSOC4) */


/***************************************
*       Interrupt source constants
***************************************/

#define Serial_USB_DP_INTC_PRIORITY       Serial_USB_dp_int__INTC_PRIOR_NUM
#define Serial_USB_DP_INTC_VECT_NUM       Serial_USB_dp_int__INTC_NUMBER

#if (CY_PSOC4)
    #define Serial_USB_DMA_AUTO_INTR_PRIO (0u)
    
    #define Serial_USB_INTR_HI_PRIORITY   Serial_USB_high_int__INTC_PRIOR_NUM
    #define Serial_USB_INTR_HI_VECT_NUM   Serial_USB_high_int__INTC_NUMBER

    #define Serial_USB_INTR_MED_PRIORITY  Serial_USB_med_int__INTC_PRIOR_NUM
    #define Serial_USB_INTR_MED_VECT_NUM  Serial_USB_med_int__INTC_NUMBER

    #define Serial_USB_INTR_LO_PRIORITY   Serial_USB_lo_int__INTC_PRIOR_NUM
    #define Serial_USB_INTR_LO_VECT_NUM   Serial_USB_lo_int__INTC_NUMBER

    /* Interrupt sources in Serial_USB_isrCallbacks[] table */
    #define Serial_USB_SOF_INTR_NUM       (0u)
    #define Serial_USB_BUS_RESET_INT_NUM  (1u)
    #define Serial_USB_EP0_INTR_NUM       (2u)
    #define Serial_USB_LPM_INTR_NUM       (3u)
    #define Serial_USB_ARB_EP_INTR_NUM    (4u)
    #define Serial_USB_EP1_INTR_NUM       (5u)
    #define Serial_USB_EP2_INTR_NUM       (6u)
    #define Serial_USB_EP3_INTR_NUM       (7u)
    #define Serial_USB_EP4_INTR_NUM       (8u)
    #define Serial_USB_EP5_INTR_NUM       (9u)
    #define Serial_USB_EP6_INTR_NUM       (10u)
    #define Serial_USB_EP7_INTR_NUM       (11u)
    #define Serial_USB_EP8_INTR_NUM       (12u)

#else
    #define Serial_USB_BUS_RESET_PRIOR    Serial_USB_bus_reset__INTC_PRIOR_NUM
    #define Serial_USB_BUS_RESET_MASK     Serial_USB_bus_reset__INTC_MASK
    #define Serial_USB_BUS_RESET_VECT_NUM Serial_USB_bus_reset__INTC_NUMBER

    #define Serial_USB_SOF_PRIOR          Serial_USB_sof_int__INTC_PRIOR_NUM
    #define Serial_USB_SOF_MASK           Serial_USB_sof_int__INTC_MASK
    #define Serial_USB_SOF_VECT_NUM       Serial_USB_sof_int__INTC_NUMBER

    #define Serial_USB_EP_0_PRIOR         Serial_USB_ep_0__INTC_PRIOR_NUM
    #define Serial_USB_EP_0_MASK          Serial_USB_ep_0__INTC_MASK
    #define Serial_USB_EP_0_VECT_NUM      Serial_USB_ep_0__INTC_NUMBER

    #define Serial_USB_EP_1_PRIOR         Serial_USB_ep_1__INTC_PRIOR_NUM
    #define Serial_USB_EP_1_MASK          Serial_USB_ep_1__INTC_MASK
    #define Serial_USB_EP_1_VECT_NUM      Serial_USB_ep_1__INTC_NUMBER

    #define Serial_USB_EP_2_PRIOR         Serial_USB_ep_2__INTC_PRIOR_NUM
    #define Serial_USB_EP_2_MASK          Serial_USB_ep_2__INTC_MASK
    #define Serial_USB_EP_2_VECT_NUM      Serial_USB_ep_2__INTC_NUMBER

    #define Serial_USB_EP_3_PRIOR         Serial_USB_ep_3__INTC_PRIOR_NUM
    #define Serial_USB_EP_3_MASK          Serial_USB_ep_3__INTC_MASK
    #define Serial_USB_EP_3_VECT_NUM      Serial_USB_ep_3__INTC_NUMBER

    #define Serial_USB_EP_4_PRIOR         Serial_USB_ep_4__INTC_PRIOR_NUM
    #define Serial_USB_EP_4_MASK          Serial_USB_ep_4__INTC_MASK
    #define Serial_USB_EP_4_VECT_NUM      Serial_USB_ep_4__INTC_NUMBER

    #define Serial_USB_EP_5_PRIOR         Serial_USB_ep_5__INTC_PRIOR_NUM
    #define Serial_USB_EP_5_MASK          Serial_USB_ep_5__INTC_MASK
    #define Serial_USB_EP_5_VECT_NUM      Serial_USB_ep_5__INTC_NUMBER

    #define Serial_USB_EP_6_PRIOR         Serial_USB_ep_6__INTC_PRIOR_NUM
    #define Serial_USB_EP_6_MASK          Serial_USB_ep_6__INTC_MASK
    #define Serial_USB_EP_6_VECT_NUM      Serial_USB_ep_6__INTC_NUMBER

    #define Serial_USB_EP_7_PRIOR         Serial_USB_ep_7__INTC_PRIOR_NUM
    #define Serial_USB_EP_7_MASK          Serial_USB_ep_7__INTC_MASK
    #define Serial_USB_EP_7_VECT_NUM      Serial_USB_ep_7__INTC_NUMBER

    #define Serial_USB_EP_8_PRIOR         Serial_USB_ep_8__INTC_PRIOR_NUM
    #define Serial_USB_EP_8_MASK          Serial_USB_ep_8__INTC_MASK
    #define Serial_USB_EP_8_VECT_NUM      Serial_USB_ep_8__INTC_NUMBER

    /* Set ARB ISR priority 0 to be highest for all EPX ISRs. */
    #define Serial_USB_ARB_PRIOR          (0u)
    #define Serial_USB_ARB_MASK           Serial_USB_arb_int__INTC_MASK
    #define Serial_USB_ARB_VECT_NUM       Serial_USB_arb_int__INTC_NUMBER
#endif /* (CY_PSOC4) */


/***************************************
*       Endpoint 0 offsets (Table 9-2)
***************************************/
#define Serial_USB_bmRequestTypeReg      Serial_USB_EP0_DR_BASE.epData[0u]
#define Serial_USB_bRequestReg           Serial_USB_EP0_DR_BASE.epData[1u]
#define Serial_USB_wValueLoReg           Serial_USB_EP0_DR_BASE.epData[2u]
#define Serial_USB_wValueHiReg           Serial_USB_EP0_DR_BASE.epData[3u]
#define Serial_USB_wIndexLoReg           Serial_USB_EP0_DR_BASE.epData[4u]
#define Serial_USB_wIndexHiReg           Serial_USB_EP0_DR_BASE.epData[5u]
#define Serial_USB_wLengthLoReg          Serial_USB_EP0_DR_BASE.epData[6u]
#define Serial_USB_wLengthHiReg          Serial_USB_EP0_DR_BASE.epData[7u]

/* Compatibility defines */
#define Serial_USB_lengthLoReg           Serial_USB_EP0_DR_BASE.epData[6u]
#define Serial_USB_lengthHiReg           Serial_USB_EP0_DR_BASE.epData[7u]


/***************************************
*       Register Constants
***************************************/

#define Serial_USB_3500MV     (3500u)
#if (CY_PSOC4)
    #define Serial_USB_VDDD_MV    (CYDEV_VBUS_MV)
#else
    #define Serial_USB_VDDD_MV    (CYDEV_VDDD_MV)
#endif /* (CY_PSOC4) */


/* Serial_USB_USB_CLK */
#define Serial_USB_USB_CLK_CSR_CLK_EN_POS (0u)
#define Serial_USB_USB_CLK_CSR_CLK_EN     ((uint8) ((uint8) 0x1u << Serial_USB_USB_CLK_CSR_CLK_EN_POS))
#define Serial_USB_USB_CLK_ENABLE         (Serial_USB_USB_CLK_CSR_CLK_EN)

/* Serial_USB_CR0 */
#define Serial_USB_CR0_DEVICE_ADDRESS_POS     (0u)
#define Serial_USB_CR0_ENABLE_POS             (7u)
#define Serial_USB_CR0_DEVICE_ADDRESS_MASK    ((uint8) ((uint8) 0x7Fu << Serial_USB_CR0_DEVICE_ADDRESS_POS))
#define Serial_USB_CR0_ENABLE                 ((uint8) ((uint8) 0x01u << Serial_USB_CR0_ENABLE_POS))


/* Serial_USB_CR1 */
#define Serial_USB_CR1_REG_ENABLE_POS         (0u)
#define Serial_USB_CR1_ENABLE_LOCK_POS        (1u)
#define Serial_USB_CR1_BUS_ACTIVITY_POS       (2u)
#define Serial_USB_CR1_TRIM_OFFSET_MSB_POS    (3u)
#define Serial_USB_CR1_REG_ENABLE             ((uint8) ((uint8) 0x1u << Serial_USB_CR1_REG_ENABLE_POS))
#define Serial_USB_CR1_ENABLE_LOCK            ((uint8) ((uint8) 0x1u << Serial_USB_CR1_ENABLE_LOCK_POS))
#define Serial_USB_CR1_BUS_ACTIVITY           ((uint8) ((uint8) 0x1u << Serial_USB_CR1_BUS_ACTIVITY_POS))
#define Serial_USB_CR1_TRIM_OFFSET_MSB        ((uint8) ((uint8) 0x1u << Serial_USB_CR1_TRIM_OFFSET_MSB_POS))

/* Serial_USB_EPX_CNT */
#define Serial_USB_EP0_CNT_DATA_TOGGLE        (0x80u)
#define Serial_USB_EPX_CNT_DATA_TOGGLE        (0x80u)
#define Serial_USB_EPX_CNT0_MASK              (0x0Fu)
#define Serial_USB_EPX_CNTX_MSB_MASK          (0x07u)
#define Serial_USB_EPX_CNTX_ADDR_SHIFT        (0x04u)
#define Serial_USB_EPX_CNTX_ADDR_OFFSET       (0x10u)
#define Serial_USB_EPX_CNTX_CRC_COUNT         (0x02u)
#define Serial_USB_EPX_DATA_BUF_MAX           (512u)

/* Serial_USB_USBIO_CR0 */

#define Serial_USB_USBIO_CR0_TEN              (0x80u)
#define Serial_USB_USBIO_CR0_TSE0             (0x40u)
#define Serial_USB_USBIO_CR0_TD               (0x20u)
#define Serial_USB_USBIO_CR0_RD               (0x01u)

/* Serial_USB_USBIO_CR1 */
#define Serial_USB_USBIO_CR1_DM0_POS      (0u)
#define Serial_USB_USBIO_CR1_DP0_POS      (1u)
#define Serial_USB_USBIO_CR1_USBPUEN_POS  (2u)
#define Serial_USB_USBIO_CR1_IOMODE_POS   (5u)
#define Serial_USB_USBIO_CR1_DM0          ((uint8) ((uint8) 0x1u << Serial_USB_USBIO_CR1_DM0_POS))
#define Serial_USB_USBIO_CR1_DP0          ((uint8) ((uint8) 0x1u << Serial_USB_USBIO_CR1_DP0_POS))
#define Serial_USB_USBIO_CR1_USBPUEN      ((uint8) ((uint8) 0x1u << Serial_USB_USBIO_CR1_USBPUEN_POS))
#define Serial_USB_USBIO_CR1_IOMODE       ((uint8) ((uint8) 0x1u << Serial_USB_USBIO_CR1_IOMODE_POS))

/* Serial_USB_FASTCLK_IMO_CR */
#define Serial_USB_FASTCLK_IMO_CR_USBCLK_ON   (0x40u)
#define Serial_USB_FASTCLK_IMO_CR_XCLKEN      (0x20u)
#define Serial_USB_FASTCLK_IMO_CR_FX2ON       (0x10u)

/* Serial_USB_ARB_EPX_CFG */
#define Serial_USB_ARB_EPX_CFG_IN_DATA_RDY_POS    (0u)
#define Serial_USB_ARB_EPX_CFG_DMA_REQ_POS        (1u)
#define Serial_USB_ARB_EPX_CFG_CRC_BYPASS_POS     (2u)
#define Serial_USB_ARB_EPX_CFG_RESET_POS          (3u)
#define Serial_USB_ARB_EPX_CFG_IN_DATA_RDY        ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_CFG_IN_DATA_RDY_POS))
#define Serial_USB_ARB_EPX_CFG_DMA_REQ            ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_CFG_DMA_REQ_POS))
#define Serial_USB_ARB_EPX_CFG_CRC_BYPASS         ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_CFG_CRC_BYPASS_POS))
#define Serial_USB_ARB_EPX_CFG_RESET              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_CFG_RESET_POS))

/* Serial_USB_ARB_EPX_INT / SR */
#define Serial_USB_ARB_EPX_INT_IN_BUF_FULL_POS    (0u)
#define Serial_USB_ARB_EPX_INT_DMA_GNT_POS        (1u)
#define Serial_USB_ARB_EPX_INT_BUF_OVER_POS       (2u)
#define Serial_USB_ARB_EPX_INT_BUF_UNDER_POS      (3u)
#define Serial_USB_ARB_EPX_INT_ERR_INT_POS        (4u)
#define Serial_USB_ARB_EPX_INT_IN_BUF_FULL        ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_INT_IN_BUF_FULL_POS))
#define Serial_USB_ARB_EPX_INT_DMA_GNT            ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_INT_DMA_GNT_POS))
#define Serial_USB_ARB_EPX_INT_BUF_OVER           ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_INT_BUF_OVER_POS))
#define Serial_USB_ARB_EPX_INT_BUF_UNDER          ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_INT_BUF_UNDER_POS))
#define Serial_USB_ARB_EPX_INT_ERR_INT            ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_INT_ERR_INT_POS))

#if (CY_PSOC4)
#define Serial_USB_ARB_EPX_INT_DMA_TERMIN_POS     (5u)
#define Serial_USB_ARB_EPX_INT_DMA_TERMIN         ((uint8) ((uint8) 0x1u << Serial_USB_ARB_EPX_INT_DMA_TERMIN_POS))
#endif /* (CY_PSOC4) */

/* Common arbiter interrupt sources for all PSoC devices. */
#define Serial_USB_ARB_EPX_INT_COMMON    (Serial_USB_ARB_EPX_INT_IN_BUF_FULL | \
                                                Serial_USB_ARB_EPX_INT_DMA_GNT     | \
                                                Serial_USB_ARB_EPX_INT_BUF_OVER    | \
                                                Serial_USB_ARB_EPX_INT_BUF_UNDER   | \
                                                Serial_USB_ARB_EPX_INT_ERR_INT)

#if (CY_PSOC4)
    #define Serial_USB_ARB_EPX_INT_ALL    (Serial_USB_ARB_EPX_INT_COMMON | Serial_USB_ARB_EPX_INT_DMA_TERMIN)
#else
    #define Serial_USB_ARB_EPX_INT_ALL    (Serial_USB_ARB_EPX_INT_COMMON)
#endif /* (CY_PSOC4) */

/* Serial_USB_ARB_CFG */
#define Serial_USB_ARB_CFG_AUTO_MEM_POS   (4u)
#define Serial_USB_ARB_CFG_DMA_CFG_POS    (5u)
#define Serial_USB_ARB_CFG_CFG_CMP_POS    (7u)
#define Serial_USB_ARB_CFG_AUTO_MEM       ((uint8) ((uint8) 0x1u << Serial_USB_ARB_CFG_AUTO_MEM_POS))
#define Serial_USB_ARB_CFG_DMA_CFG_MASK   ((uint8) ((uint8) 0x3u << Serial_USB_ARB_CFG_DMA_CFG_POS))
#define Serial_USB_ARB_CFG_DMA_CFG_NONE   ((uint8) ((uint8) 0x0u << Serial_USB_ARB_CFG_DMA_CFG_POS))
#define Serial_USB_ARB_CFG_DMA_CFG_MANUAL ((uint8) ((uint8) 0x1u << Serial_USB_ARB_CFG_DMA_CFG_POS))
#define Serial_USB_ARB_CFG_DMA_CFG_AUTO   ((uint8) ((uint8) 0x2u << Serial_USB_ARB_CFG_DMA_CFG_POS))
#define Serial_USB_ARB_CFG_CFG_CMP        ((uint8) ((uint8) 0x1u << Serial_USB_ARB_CFG_CFG_CMP_POS))

/* Serial_USB_DYN_RECONFIG */
#define Serial_USB_DYN_RECONFIG_EP_SHIFT      (1u)
#define Serial_USB_DYN_RECONFIG_ENABLE_POS    (0u)
#define Serial_USB_DYN_RECONFIG_EPNO_POS      (1u)
#define Serial_USB_DYN_RECONFIG_RDY_STS_POS   (4u)
#define Serial_USB_DYN_RECONFIG_ENABLE        ((uint8) ((uint8) 0x1u << Serial_USB_DYN_RECONFIG_ENABLE_POS))
#define Serial_USB_DYN_RECONFIG_EPNO_MASK     ((uint8) ((uint8) 0x7u << Serial_USB_DYN_RECONFIG_EPNO_POS))
#define Serial_USB_DYN_RECONFIG_RDY_STS       ((uint8) ((uint8) 0x1u << Serial_USB_DYN_RECONFIG_RDY_STS_POS))

/* Serial_USB_ARB_INT */
#define Serial_USB_ARB_INT_EP1_INTR_POS          (0u) /* [0] Interrupt for USB EP1 */
#define Serial_USB_ARB_INT_EP2_INTR_POS          (1u) /* [1] Interrupt for USB EP2 */
#define Serial_USB_ARB_INT_EP3_INTR_POS          (2u) /* [2] Interrupt for USB EP3 */
#define Serial_USB_ARB_INT_EP4_INTR_POS          (3u) /* [3] Interrupt for USB EP4 */
#define Serial_USB_ARB_INT_EP5_INTR_POS          (4u) /* [4] Interrupt for USB EP5 */
#define Serial_USB_ARB_INT_EP6_INTR_POS          (5u) /* [5] Interrupt for USB EP6 */
#define Serial_USB_ARB_INT_EP7_INTR_POS          (6u) /* [6] Interrupt for USB EP7 */
#define Serial_USB_ARB_INT_EP8_INTR_POS          (7u) /* [7] Interrupt for USB EP8 */
#define Serial_USB_ARB_INT_EP1_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP1_INTR_POS))
#define Serial_USB_ARB_INT_EP2_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP2_INTR_POS))
#define Serial_USB_ARB_INT_EP3_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP3_INTR_POS))
#define Serial_USB_ARB_INT_EP4_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP4_INTR_POS))
#define Serial_USB_ARB_INT_EP5_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP5_INTR_POS))
#define Serial_USB_ARB_INT_EP6_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP6_INTR_POS))
#define Serial_USB_ARB_INT_EP7_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP7_INTR_POS))
#define Serial_USB_ARB_INT_EP8_INTR              ((uint8) ((uint8) 0x1u << Serial_USB_ARB_INT_EP8_INTR_POS))

/* Serial_USB_SIE_INT */
#define Serial_USB_SIE_INT_EP1_INTR_POS          (0u) /* [0] Interrupt for USB EP1 */
#define Serial_USB_SIE_INT_EP2_INTR_POS          (1u) /* [1] Interrupt for USB EP2 */
#define Serial_USB_SIE_INT_EP3_INTR_POS          (2u) /* [2] Interrupt for USB EP3 */
#define Serial_USB_SIE_INT_EP4_INTR_POS          (3u) /* [3] Interrupt for USB EP4 */
#define Serial_USB_SIE_INT_EP5_INTR_POS          (4u) /* [4] Interrupt for USB EP5 */
#define Serial_USB_SIE_INT_EP6_INTR_POS          (5u) /* [5] Interrupt for USB EP6 */
#define Serial_USB_SIE_INT_EP7_INTR_POS          (6u) /* [6] Interrupt for USB EP7 */
#define Serial_USB_SIE_INT_EP8_INTR_POS          (7u) /* [7] Interrupt for USB EP8 */
#define Serial_USB_SIE_INT_EP1_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP1_INTR_POS))
#define Serial_USB_SIE_INT_EP2_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP2_INTR_POS))
#define Serial_USB_SIE_INT_EP3_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP3_INTR_POS))
#define Serial_USB_SIE_INT_EP4_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP4_INTR_POS))
#define Serial_USB_SIE_INT_EP5_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP5_INTR_POS))
#define Serial_USB_SIE_INT_EP6_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP6_INTR_POS))
#define Serial_USB_SIE_INT_EP7_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP7_INTR_POS))
#define Serial_USB_SIE_INT_EP8_INTR              ((uint8) ((uint8) 0x01u << Serial_USB_SIE_INT_EP8_INTR_POS))

#if (CY_PSOC4)
    /* Serial_USB_POWER_CTRL_REG */
    #define Serial_USB_POWER_CTRL_VBUS_VALID_OVR_POS       (0u)  /* [0] */
    #define Serial_USB_POWER_CTRL_SUSPEND_POS              (2u)  /* [1] */
    #define Serial_USB_POWER_CTRL_SUSPEND_DEL_POS          (3u)  /* [3] */
    #define Serial_USB_POWER_CTRL_ISOLATE_POS              (4u)  /* [4] */
    #define Serial_USB_POWER_CTRL_CHDET_PWR_CTL_POS        (5u)  /* [5] */
    #define Serial_USB_POWER_CTRL_ENABLE_DM_PULLDOWN_POS   (25u) /* [25] */
    #define Serial_USB_POWER_CTRL_ENABLE_VBUS_PULLDOWN_POS (26u) /* [26] */
    #define Serial_USB_POWER_CTRL_ENABLE_RCVR_POS          (27u) /* [27] */
    #define Serial_USB_POWER_CTRL_ENABLE_DPO_POS           (28u) /* [28] */
    #define Serial_USB_POWER_CTRL_ENABLE_DMO_POS           (29u) /* [29] */
    #define Serial_USB_POWER_CTRL_ENABLE_CHGDET_POS        (30u) /* [30] */
    #define Serial_USB_POWER_CTRL_ENABLE_POS               (31u) /* [31] */
    #define Serial_USB_POWER_CTRL_VBUS_VALID_OVR_MASK      ((uint32) 0x03u << Serial_USB_POWER_CTRL_VBUS_VALID_OVR_POS)
    #define Serial_USB_POWER_CTRL_VBUS_VALID_OVR_0         ((uint32) 0x00u << Serial_USB_POWER_CTRL_VBUS_VALID_OVR_POS)
    #define Serial_USB_POWER_CTRL_VBUS_VALID_OVR_1         ((uint32) 0x01u << Serial_USB_POWER_CTRL_VBUS_VALID_OVR_POS)
    #define Serial_USB_POWER_CTRL_VBUS_VALID_OVR_GPIO      ((uint32) 0x02u << Serial_USB_POWER_CTRL_VBUS_VALID_OVR_POS)
    #define Serial_USB_POWER_CTRL_VBUS_VALID_OVR_PHY       ((uint32) 0x03u << Serial_USB_POWER_CTRL_VBUS_VALID_OVR_POS)
    #define Serial_USB_POWER_CTRL_SUSPEND                  ((uint32) 0x01u << Serial_USB_POWER_CTRL_SUSPEND_POS)
    #define Serial_USB_POWER_CTRL_SUSPEND_DEL              ((uint32) 0x01u << Serial_USB_POWER_CTRL_SUSPEND_DEL_POS)
    #define Serial_USB_POWER_CTRL_ISOLATE                  ((uint32) 0x01u << Serial_USB_POWER_CTRL_ISOLATE_POS)
    #define Serial_USB_POWER_CTRL_CHDET_PWR_CTL_MASK       ((uint32) 0x03u << Serial_USB_POWER_CTRL_CHDET_PWR_CTL_POS)
    #define Serial_USB_POWER_CTRL_ENABLE_DM_PULLDOWN       ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_DM_PULLDOWN_POS)
    #define Serial_USB_POWER_CTRL_ENABLE_VBUS_PULLDOWN     ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_VBUS_PULLDOWN_POS)
    #define Serial_USB_POWER_CTRL_ENABLE_RCVR              ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_RCVR_POS)
    #define Serial_USB_POWER_CTRL_ENABLE_DPO               ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_DPO_POS)
    #define Serial_USB_POWER_CTRL_ENABLE_DMO               ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_DMO_POS)
    #define Serial_USB_POWER_CTRL_ENABLE_CHGDET            ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_CHGDET_POS)
    #define Serial_USB_POWER_CTRL_ENABLE                   ((uint32) 0x01u << Serial_USB_POWER_CTRL_ENABLE_POS)

    /* Serial_USB_CHGDET_CTRL_REG */
    #define Serial_USB_CHGDET_CTRL_COMP_DP_POS        (0u)  /* [0] */
    #define Serial_USB_CHGDET_CTRL_COMP_DM_POS        (1u)  /* [1] */
    #define Serial_USB_CHGDET_CTRL_COMP_EN_POS        (2u)  /* [2] */
    #define Serial_USB_CHGDET_CTRL_REF_DP_POS         (3u)  /* [3] */
    #define Serial_USB_CHGDET_CTRL_REF_DM_POS         (4u)  /* [4] */
    #define Serial_USB_CHGDET_CTRL_REF_EN_POS         (5u)  /* [5] */
    #define Serial_USB_CHGDET_CTRL_DCD_SRC_EN_POS     (6u)  /* [6] */
    #define Serial_USB_CHGDET_CTRL_ADFT_CTRL_POS      (12u) /* [12] */
    #define Serial_USB_CHGDET_CTRL_COMP_OUT_POS       (31u) /* [31] */
    #define Serial_USB_CHGDET_CTRL_COMP_DP            ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_COMP_DP_POS)
    #define Serial_USB_CHGDET_CTRL_COMP_DM            ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_COMP_DM_POS)
    #define Serial_USB_CHGDET_CTRL_COMP_EN            ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_COMP_EN_POS)
    #define Serial_USB_CHGDET_CTRL_REF_DP             ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_REF_DP_POS)
    #define Serial_USB_CHGDET_CTRL_REF_DM             ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_REF_DM_POS)
    #define Serial_USB_CHGDET_CTRL_REF_EN             ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_REF_EN_POS)
    #define Serial_USB_CHGDET_CTRL_DCD_SRC_EN         ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_DCD_SRC_EN_POS)
    #define Serial_USB_CHGDET_CTRL_ADFT_CTRL_MASK     ((uint32) 0x03u << Serial_USB_CHGDET_CTRL_ADFT_CTRL_POS)
    #define Serial_USB_CHGDET_CTRL_ADFT_CTRL_NORMAL   ((uint32) 0x00u << Serial_USB_CHGDET_CTRL_ADFT_CTRL_POS)
    #define Serial_USB_CHGDET_CTRL_ADFT_CTRL_VBG      ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_ADFT_CTRL_POS)
    #define Serial_USB_CHGDET_CTRL_ADFT_CTRL_DONTUSE  ((uint32) 0x02u << Serial_USB_CHGDET_CTRL_ADFT_CTRL_POS)
    #define Serial_USB_CHGDET_CTRL_ADFT_CTRL_ADFTIN   ((uint32) 0x03u << Serial_USB_CHGDET_CTRL_ADFT_CTRL_POS)
    #define Serial_USB_CHGDET_CTRL_COMP_OUT           ((uint32) 0x01u << Serial_USB_CHGDET_CTRL_COMP_OUT_POS)

    /* Serial_USB_LPM_CTRL */
    #define Serial_USB_LPM_CTRL_LPM_EN_POS        (0u)
    #define Serial_USB_LPM_CTRL_LPM_ACK_RESP_POS  (1u)
    #define Serial_USB_LPM_CTRL_NYET_EN_POS       (2u)
    #define Serial_USB_LPM_CTRL_SUB_RESP_POS      (4u)
    #define Serial_USB_LPM_CTRL_LPM_EN            ((uint32) 0x01u << Serial_USB_LPM_CTRL_LPM_EN_POS)
    #define Serial_USB_LPM_CTRL_LPM_ACK_RESP      ((uint32) 0x01u << Serial_USB_LPM_CTRL_LPM_ACK_RESP_POS)
    #define Serial_USB_LPM_CTRL_NYET_EN           ((uint32) 0x01u << Serial_USB_LPM_CTRL_NYET_EN_POS)
    #define Serial_USB_LPM_CTRL_ACK_NYET_MASK     ((uint32) 0x03u << Serial_USB_LPM_CTRL_LPM_ACK_RESP_POS)
    #define Serial_USB_LPM_CTRL_SUB_RESP          ((uint32) 0x01u << Serial_USB_LPM_CTRL_SUB_RESP_POS)

    #define Serial_USB_LPM_STAT_LPM_BESL_POS          (0u)
    #define Serial_USB_LPM_STAT_LPM_REMOTE_WAKE_POS   (4u)
    #define Serial_USB_LPM_STAT_LPM_BESL_MASK         ((uint32) 0x0Fu << Serial_USB_LPM_STAT_LPM_BESL_POS)
    #define Serial_USB_LPM_STAT_LPM_REMOTE_WAKE       ((uint32) 0x01u << Serial_USB_LPM_STAT_LPM_REMOTE_WAKE_POS)

    /* Serial_USB_INTR_SIE */
    #define Serial_USB_INTR_SIE_SOF_INTR_POS          (0u) /* [0] Interrupt for USB SOF   */
    #define Serial_USB_INTR_SIE_BUS_RESET_INTR_POS    (1u) /* [1] Interrupt for BUS RESET */
    #define Serial_USB_INTR_SIE_EP0_INTR_POS          (2u) /* [2] Interrupt for EP0       */
    #define Serial_USB_INTR_SIE_LPM_INTR_POS          (3u) /* [3] Interrupt for LPM       */
    #define Serial_USB_INTR_SIE_RESUME_INTR_POS       (4u) /* [4] Interrupt for RESUME (not used by component) */
    #define Serial_USB_INTR_SIE_SOF_INTR              ((uint32) 0x01u << Serial_USB_INTR_SIE_SOF_INTR_POS)
    #define Serial_USB_INTR_SIE_BUS_RESET_INTR        ((uint32) 0x01u << Serial_USB_INTR_SIE_BUS_RESET_INTR_POS)
    #define Serial_USB_INTR_SIE_EP0_INTR              ((uint32) 0x01u << Serial_USB_INTR_SIE_EP0_INTR_POS)
    #define Serial_USB_INTR_SIE_LPM_INTR              ((uint32) 0x01u << Serial_USB_INTR_SIE_LPM_INTR_POS)
    #define Serial_USB_INTR_SIE_RESUME_INTR           ((uint32) 0x01u << Serial_USB_INTR_SIE_RESUME_INTR_POS)

    /* Serial_USB_INTR_CAUSE_LO, MED and HI */
    #define Serial_USB_INTR_CAUSE_SOF_INTR_POS       (0u)  /* [0] Interrupt status for USB SOF    */
    #define Serial_USB_INTR_CAUSE_BUS_RESET_INTR_POS (1u)  /* [1] Interrupt status for USB BUS RSET */
    #define Serial_USB_INTR_CAUSE_EP0_INTR_POS       (2u)  /* [2] Interrupt status for USB EP0    */
    #define Serial_USB_INTR_CAUSE_LPM_INTR_POS       (3u)  /* [3] Interrupt status for USB LPM    */
    #define Serial_USB_INTR_CAUSE_RESUME_INTR_POS    (4u)  /* [4] Interrupt status for USB RESUME */
    #define Serial_USB_INTR_CAUSE_ARB_INTR_POS       (7u)  /* [7] Interrupt status for USB ARB    */
    #define Serial_USB_INTR_CAUSE_EP1_INTR_POS       (8u)  /* [8] Interrupt status for USB EP1    */
    #define Serial_USB_INTR_CAUSE_EP2_INTR_POS       (9u)  /* [9] Interrupt status for USB EP2    */
    #define Serial_USB_INTR_CAUSE_EP3_INTR_POS       (10u) /* [10] Interrupt status for USB EP3   */
    #define Serial_USB_INTR_CAUSE_EP4_INTR_POS       (11u) /* [11] Interrupt status for USB EP4   */
    #define Serial_USB_INTR_CAUSE_EP5_INTR_POS       (12u) /* [12] Interrupt status for USB EP5   */
    #define Serial_USB_INTR_CAUSE_EP6_INTR_POS       (13u) /* [13] Interrupt status for USB EP6   */
    #define Serial_USB_INTR_CAUSE_EP7_INTR_POS       (14u) /* [14] Interrupt status for USB EP7   */
    #define Serial_USB_INTR_CAUSE_EP8_INTR_POS       (15u) /* [15] Interrupt status for USB EP8   */
    #define Serial_USB_INTR_CAUSE_SOF_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_SOF_INTR_POS)
    #define Serial_USB_INTR_CAUSE_BUS_RESET_INTR     ((uint32) 0x01u << Serial_USB_INTR_CAUSE_BUS_RESET_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP0_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP0_INTR_POS)
    #define Serial_USB_INTR_CAUSE_LPM_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_LPM_INTR_POS)
    #define Serial_USB_INTR_CAUSE_RESUME_INTR        ((uint32) 0x01u << Serial_USB_INTR_CAUSE_RESUME_INTR_POS)
    #define Serial_USB_INTR_CAUSE_ARB_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_ARB_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP1_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP1_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP2_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP2_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP3_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP3_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP4_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP4_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP5_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP5_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP6_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP6_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP7_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP7_INTR_POS)
    #define Serial_USB_INTR_CAUSE_EP8_INTR           ((uint32) 0x01u << Serial_USB_INTR_CAUSE_EP8_INTR_POS)

    #define Serial_USB_INTR_CAUSE_CTRL_INTR_MASK     (Serial_USB_INTR_CAUSE_SOF_INTR       | \
                                                            Serial_USB_INTR_CAUSE_BUS_RESET_INTR | \
                                                            Serial_USB_INTR_CAUSE_EP0_INTR       | \
                                                            Serial_USB_INTR_CAUSE_LPM_INTR)

    #define Serial_USB_INTR_CAUSE_EP1_8_INTR_MASK    (Serial_USB_INTR_CAUSE_EP1_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP2_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP3_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP4_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP5_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP6_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP7_INTR       | \
                                                            Serial_USB_INTR_CAUSE_EP8_INTR)

    #define Serial_USB_INTR_CAUSE_EP_INTR_SHIFT      (Serial_USB_INTR_CAUSE_ARB_INTR_POS - \
                                                           (Serial_USB_INTR_CAUSE_LPM_INTR_POS + 1u))
    #define Serial_USB_INTR_CAUSE_SRC_COUNT          (13u)

    #define Serial_USB_CHGDET_CTRL_PRIMARY    (Serial_USB_CHGDET_CTRL_COMP_EN | \
                                                     Serial_USB_CHGDET_CTRL_COMP_DM | \
                                                     Serial_USB_CHGDET_CTRL_REF_EN  | \
                                                     Serial_USB_CHGDET_CTRL_REF_DP)

    #define Serial_USB_CHGDET_CTRL_SECONDARY  (Serial_USB_CHGDET_CTRL_COMP_EN | \
                                                     Serial_USB_CHGDET_CTRL_COMP_DP | \
                                                     Serial_USB_CHGDET_CTRL_REF_EN  | \
                                                     Serial_USB_CHGDET_CTRL_REF_DM)

    #define Serial_USB_CHGDET_CTRL_DEFAULT    (0x00000900u)


#else /* (CY_PSOC3 || CY_PSOC5LP) */
    #define Serial_USB_PM_ACT_EN_FSUSB            Serial_USB_USB__PM_ACT_MSK
    #define Serial_USB_PM_STBY_EN_FSUSB           Serial_USB_USB__PM_STBY_MSK
    #define Serial_USB_PM_AVAIL_EN_FSUSBIO        (0x10u)

    #define Serial_USB_PM_USB_CR0_REF_EN          (0x01u)
    #define Serial_USB_PM_USB_CR0_PD_N            (0x02u)
    #define Serial_USB_PM_USB_CR0_PD_PULLUP_N     (0x04u)
#endif /* (CY_PSOC4) */


/***************************************
*       Macros Definitions
***************************************/

#if (CY_PSOC4)
    #define Serial_USB_ClearSieInterruptSource(intMask) \
                do{ \
                    Serial_USB_INTR_SIE_REG = (uint32) (intMask); \
                }while(0)
#else
    #define Serial_USB_ClearSieInterruptSource(intMask) \
                do{ /* Does nothing. */ }while(0)
#endif /* (CY_PSOC4) */

#define Serial_USB_ClearSieEpInterruptSource(intMask) \
            do{ \
                Serial_USB_SIE_EP_INT_SR_REG = (uint8) (intMask); \
            }while(0)

#define Serial_USB_GET_ACTIVE_IN_EP_CR0_MODE(epType)  (((epType) == Serial_USB_EP_TYPE_ISOC) ? \
                                                                (Serial_USB_MODE_ISO_IN) : (Serial_USB_MODE_ACK_IN))

#define Serial_USB_GET_ACTIVE_OUT_EP_CR0_MODE(epType) (((epType) == Serial_USB_EP_TYPE_ISOC) ? \
                                                                (Serial_USB_MODE_ISO_OUT) : (Serial_USB_MODE_ACK_OUT))

#define Serial_USB_GET_EP_TYPE(epNumber)  (Serial_USB_EP[epNumber].attrib & Serial_USB_EP_TYPE_MASK)

#define Serial_USB_GET_UINT16(hi, low)    (((uint16) ((uint16) (hi) << 8u)) | ((uint16) (low) & 0xFFu))


/***************************************
*    Initialization Register Settings
***************************************/

/* Clear device address and enable USB IP respond to USB traffic. */
#define Serial_USB_DEFUALT_CR0    (Serial_USB_CR0_ENABLE)

/* Arbiter configuration depends on memory management mode. */
#define Serial_USB_DEFAULT_ARB_CFG    ((Serial_USB_EP_MANAGEMENT_MANUAL) ? (Serial_USB_ARB_CFG_DMA_CFG_NONE) : \
                                                ((Serial_USB_EP_MANAGEMENT_DMA_MANUAL) ? \
                                                    (Serial_USB_ARB_CFG_DMA_CFG_MANUAL) : \
                                                        (Serial_USB_ARB_CFG_AUTO_MEM | Serial_USB_ARB_CFG_DMA_CFG_AUTO)))

/* Enable arbiter interrupt for active endpoints only */
#define Serial_USB_DEFAULT_ARB_INT_EN \
        ((uint8) ((uint8) Serial_USB_DMA1_ACTIVE << Serial_USB_ARB_INT_EP1_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA2_ACTIVE << Serial_USB_ARB_INT_EP2_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA3_ACTIVE << Serial_USB_ARB_INT_EP3_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA4_ACTIVE << Serial_USB_ARB_INT_EP4_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA5_ACTIVE << Serial_USB_ARB_INT_EP5_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA6_ACTIVE << Serial_USB_ARB_INT_EP6_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA7_ACTIVE << Serial_USB_ARB_INT_EP7_INTR_POS) | \
         (uint8) ((uint8) Serial_USB_DMA8_ACTIVE << Serial_USB_ARB_INT_EP8_INTR_POS))

/* Enable all SIE endpoints interrupts */
#define Serial_USB_DEFAULT_SIE_EP_INT_EN  (Serial_USB_SIE_INT_EP1_INTR | \
                                                 Serial_USB_SIE_INT_EP2_INTR | \
                                                 Serial_USB_SIE_INT_EP3_INTR | \
                                                 Serial_USB_SIE_INT_EP4_INTR | \
                                                 Serial_USB_SIE_INT_EP5_INTR | \
                                                 Serial_USB_SIE_INT_EP6_INTR | \
                                                 Serial_USB_SIE_INT_EP7_INTR | \
                                                 Serial_USB_SIE_INT_EP8_INTR)

#define Serial_USB_ARB_EPX_CFG_DEFAULT    (Serial_USB_ARB_EPX_CFG_RESET | \
                                                 Serial_USB_ARB_EPX_CFG_CRC_BYPASS)

/* Default EP arbiter interrupt source register */
#define Serial_USB_ARB_EPX_INT_COMMON_MASK   (Serial_USB_ARB_EPX_INT_IN_BUF_FULL | \
                                                    Serial_USB_ARB_EPX_INT_BUF_OVER    | \
                                                    Serial_USB_ARB_EPX_INT_BUF_UNDER   | \
                                                    Serial_USB_ARB_EPX_INT_ERR_INT     | \
                                                    (Serial_USB_EP_MANAGEMENT_DMA_MANUAL ? Serial_USB_ARB_EPX_INT_DMA_GNT : 0u))

#define Serial_USB_CLEAR_REG      (0u)

#if (CY_PSOC4)
    /* Set USB lock option when IMO is locked to USB traffic. */
    #define Serial_USB_DEFUALT_CR1    ((0u != CySysClkImoGetUsbLock()) ? (Serial_USB_CR1_ENABLE_LOCK) : (0u))

    /* Recommended value is increased from 3 to 10 due to suppress glitch on  
     * RSE0 with USB2.0 hubs (LF CLK = 32kHz equal to 350us). */
    #define Serial_USB_DEFUALT_BUS_RST_CNT  (10u)

    /* Select VBUS sources as: valid, PHY of GPIO, and clears isolate bit. */
    /* Application level must ensure that VBUS is valid valid to use. */
    #define Serial_USB_DEFAULT_POWER_CTRL_VBUS    (Serial_USB_POWER_CTRL_ENABLE_VBUS_PULLDOWN | \
                                                         ((!Serial_USB_VBUS_MONITORING_ENABLE) ? \
                                                            (Serial_USB_POWER_CTRL_VBUS_VALID_OVR_1) : \
                                                                (Serial_USB_VBUS_POWER_PAD_ENABLE ? \
                                                                    (Serial_USB_POWER_CTRL_VBUS_VALID_OVR_PHY) : \
                                                                    (Serial_USB_POWER_CTRL_VBUS_VALID_OVR_GPIO))))
    /* Enable USB IP. */
    #define Serial_USB_DEFAULT_POWER_CTRL_PHY (Serial_USB_POWER_CTRL_SUSPEND     | \
                                                     Serial_USB_POWER_CTRL_SUSPEND_DEL | \
                                                     Serial_USB_POWER_CTRL_ENABLE_RCVR | \
                                                     Serial_USB_POWER_CTRL_ENABLE_DPO  | \
                                                     Serial_USB_POWER_CTRL_ENABLE_DMO  | \
                                                     Serial_USB_POWER_CTRL_ENABLE)

    /* Assign interrupt between levels lo, med, hi. */
    #define Serial_USB_DEFAULT_INTR_LVL_SEL   ((uint32) (Serial_USB_INTR_LVL_SEL))

    /* Enable interrupt source in the INTR_SIE. The SOF is always disabled and EP0 is enabled. */
    #define Serial_USB_DEFAULT_INTR_SIE_MASK \
                ((uint32) ((uint32) Serial_USB_BUS_RESET_ISR_ACTIVE << Serial_USB_INTR_SIE_BUS_RESET_INTR_POS) | \
                 (uint32) ((uint32) Serial_USB_SOF_ISR_ACTIVE       << Serial_USB_INTR_SIE_SOF_INTR_POS)       | \
                 (uint32) ((uint32) Serial_USB_LPM_ACTIVE           << Serial_USB_INTR_SIE_LPM_INTR_POS)       | \
                 (uint32) ((uint32) Serial_USB_INTR_SIE_EP0_INTR))

    /* Arbiter interrupt sources */
    #define Serial_USB_ARB_EPX_INT_MASK   (Serial_USB_ARB_EPX_INT_COMMON_MASK | \
                                                (Serial_USB_EP_MANAGEMENT_DMA_AUTO ? Serial_USB_ARB_EPX_INT_DMA_TERMIN : 0u))

    /* Common DMA configuration */
    #define Serial_USB_DMA_COMMON_CFG     (CYDMA_PULSE | CYDMA_ENTIRE_DESCRIPTOR | \
                                                 CYDMA_NON_PREEMPTABLE)


#else
    #define Serial_USB_ARB_EPX_INT_MASK   (Serial_USB_ARB_EPX_INT_COMMON_MASK)

    #define Serial_USB_DEFUALT_CR1        (Serial_USB_CR1_ENABLE_LOCK)

    /* Recommended value is 3 for LF CLK = 100kHz equal to 100us. */
    #define Serial_USB_DEFUALT_BUS_RST_CNT    (10u)
#endif /* (CY_PSOC4) */

/*
* \addtogroup group_deprecated
* @{
*/

/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Renamed type definitions */
#define Serial_USB_CODE CYCODE
#define Serial_USB_FAR CYFAR
#if defined(__C51__) || defined(__CX51__)
    #define Serial_USB_DATA data
    #define Serial_USB_XDATA xdata
#else
    #define Serial_USB_DATA
    #define Serial_USB_XDATA
#endif /*  __C51__ */
#define Serial_USB_NULL       NULL
/** @} deprecated */
/* Renamed structure fields */
#define wBuffOffset         buffOffset
#define wBufferSize         bufferSize
#define bStatus             status
#define wLength             length
#define wCount              count

/* Renamed global variable */
#define CurrentTD           Serial_USB_currentTD
#define Serial_USB_interfaceSetting_last       Serial_USB_interfaceSettingLast

/* Renamed global constants */
#define Serial_USB_DWR_VDDD_OPERATION         (Serial_USB_DWR_POWER_OPERATION)

/* Renamed functions */
#define Serial_USB_bCheckActivity             Serial_USB_CheckActivity
#define Serial_USB_bGetConfiguration          Serial_USB_GetConfiguration
#define Serial_USB_bGetInterfaceSetting       Serial_USB_GetInterfaceSetting
#define Serial_USB_bGetEPState                Serial_USB_GetEPState
#define Serial_USB_wGetEPCount                Serial_USB_GetEPCount
#define Serial_USB_bGetEPAckState             Serial_USB_GetEPAckState
#define Serial_USB_bRWUEnabled                Serial_USB_RWUEnabled
#define Serial_USB_bVBusPresent               Serial_USB_VBusPresent

#define Serial_USB_bConfiguration             Serial_USB_configuration
#define Serial_USB_bInterfaceSetting          Serial_USB_interfaceSetting
#define Serial_USB_bDeviceAddress             Serial_USB_deviceAddress
#define Serial_USB_bDeviceStatus              Serial_USB_deviceStatus
#define Serial_USB_bDevice                    Serial_USB_device
#define Serial_USB_bTransferState             Serial_USB_transferState
#define Serial_USB_bLastPacketSize            Serial_USB_lastPacketSize

#define Serial_USB_LoadEP                     Serial_USB_LoadInEP
#define Serial_USB_LoadInISOCEP               Serial_USB_LoadInEP
#define Serial_USB_EnableOutISOCEP            Serial_USB_EnableOutEP

#define Serial_USB_SetVector                  CyIntSetVector
#define Serial_USB_SetPriority                CyIntSetPriority
#define Serial_USB_EnableInt                  CyIntEnable

/* Replace with register access. */
#define Serial_USB_bmRequestType      Serial_USB_EP0_DR0_PTR
#define Serial_USB_bRequest           Serial_USB_EP0_DR1_PTR
#define Serial_USB_wValue             Serial_USB_EP0_DR2_PTR
#define Serial_USB_wValueHi           Serial_USB_EP0_DR3_PTR
#define Serial_USB_wValueLo           Serial_USB_EP0_DR2_PTR
#define Serial_USB_wIndex             Serial_USB_EP0_DR4_PTR
#define Serial_USB_wIndexHi           Serial_USB_EP0_DR5_PTR
#define Serial_USB_wIndexLo           Serial_USB_EP0_DR4_PTR
#define Serial_USB_length             Serial_USB_EP0_DR6_PTR
#define Serial_USB_lengthHi           Serial_USB_EP0_DR7_PTR
#define Serial_USB_lengthLo           Serial_USB_EP0_DR6_PTR

/* Rename VBUS monitoring registers. */
#if (CY_PSOC3 || CY_PSOC5LP)
    #if (Serial_USB_VBUS_MONITORING_ENABLE)
        #if (Serial_USB_VBUS_MONITORING_INTERNAL)
            #define Serial_USB_VBUS_DR_PTR    ( (reg8 *) Serial_USB_VBUS__DR)
            #define Serial_USB_VBUS_DR_REG    (*(reg8 *) Serial_USB_VBUS__DR)
            #define Serial_USB_VBUS_PS_PTR    ( (reg8 *) Serial_USB_VBUS__PS)
            #define Serial_USB_VBUS_PS_REG    (*(reg8 *) Serial_USB_VBUS__PS)
            #define Serial_USB_VBUS_MASK          Serial_USB_VBUS__MASK
        #else
            #define Serial_USB_VBUS_PS_PTR    ( (reg8 *) Serial_USB_Vbus_ps_sts_sts_reg__STATUS_REG)
            #define Serial_USB_VBUS_MASK      (0x01u)
        #endif /* (Serial_USB_VBUS_MONITORING_INTERNAL) */
    #endif /*(Serial_USB_VBUS_MONITORING_ENABLE) */
        
    /* Pointer DIE structure in flash (8 bytes): Y and X location, wafer, lot msb, lot lsb, 
    *  work week, fab/year, minor. */
    #define Serial_USB_DIE_ID             CYDEV_FLSHID_CUST_TABLES_BASE

     #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
        #if (Serial_USB_DMA1_ACTIVE)
            #define Serial_USB_ep1_TD_TERMOUT_EN  (Serial_USB_ep1__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep1_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA1_ACTIVE) */

        #if (Serial_USB_DMA2_ACTIVE)
            #define Serial_USB_ep2_TD_TERMOUT_EN  (Serial_USB_ep2__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep2_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA2_ACTIVE) */

        #if (Serial_USB_DMA3_ACTIVE)
            #define Serial_USB_ep3_TD_TERMOUT_EN  (Serial_USB_ep3__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep3_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA3_ACTIVE) */

        #if (Serial_USB_DMA4_ACTIVE)
            #define Serial_USB_ep4_TD_TERMOUT_EN  (Serial_USB_ep4__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep4_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA4_ACTIVE) */

        #if (Serial_USB_DMA5_ACTIVE)
            #define Serial_USB_ep5_TD_TERMOUT_EN  (Serial_USB_ep5__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep5_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA5_ACTIVE) */

        #if (Serial_USB_DMA6_ACTIVE)
            #define Serial_USB_ep6_TD_TERMOUT_EN  (Serial_USB_ep6__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep6_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA6_ACTIVE) */

        #if (Serial_USB_DMA7_ACTIVE)
            #define Serial_USB_ep7_TD_TERMOUT_EN  (Serial_USB_ep7__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep7_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA7_ACTIVE) */

        #if (Serial_USB_DMA8_ACTIVE)
            #define Serial_USB_ep8_TD_TERMOUT_EN  (Serial_USB_ep8__TD_TERMOUT_EN)
        #else
            #define Serial_USB_ep8_TD_TERMOUT_EN  (0u)
        #endif /* (Serial_USB_DMA8_ACTIVE) */
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */   
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

/* Rename USB IP registers. */
#define Serial_USB_ARB_CFG        Serial_USB_ARB_CFG_PTR

#define Serial_USB_ARB_EP1_CFG    Serial_USB_ARB_EP1_CFG_PTR
#define Serial_USB_ARB_EP1_INT_EN Serial_USB_ARB_EP1_INT_EN_PTR
#define Serial_USB_ARB_EP1_SR     Serial_USB_ARB_EP1_SR_PTR

#define Serial_USB_ARB_EP2_CFG    Serial_USB_ARB_EP2_CFG_PTR
#define Serial_USB_ARB_EP2_INT_EN Serial_USB_ARB_EP2_INT_EN_PTR
#define Serial_USB_ARB_EP2_SR     Serial_USB_ARB_EP2_SR_PTR

#define Serial_USB_ARB_EP3_CFG    Serial_USB_ARB_EP3_CFG_PTR
#define Serial_USB_ARB_EP3_INT_EN Serial_USB_ARB_EP3_INT_EN_PTR
#define Serial_USB_ARB_EP3_SR     Serial_USB_ARB_EP3_SR_PTR

#define Serial_USB_ARB_EP4_CFG    Serial_USB_ARB_EP4_CFG_PTR
#define Serial_USB_ARB_EP4_INT_EN Serial_USB_ARB_EP4_INT_EN_PTR
#define Serial_USB_ARB_EP4_SR     Serial_USB_ARB_EP4_SR_PTR

#define Serial_USB_ARB_EP5_CFG    Serial_USB_ARB_EP5_CFG_PTR
#define Serial_USB_ARB_EP5_INT_EN Serial_USB_ARB_EP5_INT_EN_PTR
#define Serial_USB_ARB_EP5_SR     Serial_USB_ARB_EP5_SR_PTR

#define Serial_USB_ARB_EP6_CFG    Serial_USB_ARB_EP6_CFG_PTR
#define Serial_USB_ARB_EP6_INT_EN Serial_USB_ARB_EP6_INT_EN_PTR
#define Serial_USB_ARB_EP6_SR     Serial_USB_ARB_EP6_SR_PTR

#define Serial_USB_ARB_EP7_CFG    Serial_USB_ARB_EP7_CFG_PTR
#define Serial_USB_ARB_EP7_INT_EN Serial_USB_ARB_EP7_INT_EN_PTR
#define Serial_USB_ARB_EP7_SR     Serial_USB_ARB_EP7_SR_PTR

#define Serial_USB_ARB_EP8_CFG    Serial_USB_ARB_EP8_CFG_PTR
#define Serial_USB_ARB_EP8_INT_EN Serial_USB_ARB_EP8_INT_EN_PTR
#define Serial_USB_ARB_EP8_SR     Serial_USB_ARB_EP8_SR_PTR

#define Serial_USB_ARB_INT_EN     Serial_USB_ARB_INT_EN_PTR
#define Serial_USB_ARB_INT_SR     Serial_USB_ARB_INT_SR_PTR

#define Serial_USB_ARB_RW1_DR     Serial_USB_ARB_RW1_DR_PTR
#define Serial_USB_ARB_RW1_RA     Serial_USB_ARB_RW1_RA_PTR
#define Serial_USB_ARB_RW1_RA_MSB Serial_USB_ARB_RW1_RA_MSB_PTR
#define Serial_USB_ARB_RW1_WA     Serial_USB_ARB_RW1_WA_PTR
#define Serial_USB_ARB_RW1_WA_MSB Serial_USB_ARB_RW1_WA_MSB_PTR

#define Serial_USB_ARB_RW2_DR     Serial_USB_ARB_RW2_DR_PTR
#define Serial_USB_ARB_RW2_RA     Serial_USB_ARB_RW2_RA_PTR
#define Serial_USB_ARB_RW2_RA_MSB Serial_USB_ARB_RW2_RA_MSB_PTR
#define Serial_USB_ARB_RW2_WA     Serial_USB_ARB_RW2_WA_PTR
#define Serial_USB_ARB_RW2_WA_MSB Serial_USB_ARB_RW2_WA_MSB_PTR

#define Serial_USB_ARB_RW3_DR     Serial_USB_ARB_RW3_DR_PTR
#define Serial_USB_ARB_RW3_RA     Serial_USB_ARB_RW3_RA_PTR
#define Serial_USB_ARB_RW3_RA_MSB Serial_USB_ARB_RW3_RA_MSB_PTR
#define Serial_USB_ARB_RW3_WA     Serial_USB_ARB_RW3_WA_PTR
#define Serial_USB_ARB_RW3_WA_MSB Serial_USB_ARB_RW3_WA_MSB_PTR

#define Serial_USB_ARB_RW4_DR     Serial_USB_ARB_RW4_DR_PTR
#define Serial_USB_ARB_RW4_RA     Serial_USB_ARB_RW4_RA_PTR
#define Serial_USB_ARB_RW4_RA_MSB Serial_USB_ARB_RW4_RA_MSB_PTR
#define Serial_USB_ARB_RW4_WA     Serial_USB_ARB_RW4_WA_PTR
#define Serial_USB_ARB_RW4_WA_MSB Serial_USB_ARB_RW4_WA_MSB_PTR

#define Serial_USB_ARB_RW5_DR     Serial_USB_ARB_RW5_DR_PTR
#define Serial_USB_ARB_RW5_RA     Serial_USB_ARB_RW5_RA_PTR
#define Serial_USB_ARB_RW5_RA_MSB Serial_USB_ARB_RW5_RA_MSB_PTR
#define Serial_USB_ARB_RW5_WA     Serial_USB_ARB_RW5_WA_PTR
#define Serial_USB_ARB_RW5_WA_MSB Serial_USB_ARB_RW5_WA_MSB_PTR

#define Serial_USB_ARB_RW6_DR     Serial_USB_ARB_RW6_DR_PTR
#define Serial_USB_ARB_RW6_RA     Serial_USB_ARB_RW6_RA_PTR
#define Serial_USB_ARB_RW6_RA_MSB Serial_USB_ARB_RW6_RA_MSB_PTR
#define Serial_USB_ARB_RW6_WA     Serial_USB_ARB_RW6_WA_PTR
#define Serial_USB_ARB_RW6_WA_MSB Serial_USB_ARB_RW6_WA_MSB_PTR

#define Serial_USB_ARB_RW7_DR     Serial_USB_ARB_RW7_DR_PTR
#define Serial_USB_ARB_RW7_RA     Serial_USB_ARB_RW7_RA_PTR
#define Serial_USB_ARB_RW7_RA_MSB Serial_USB_ARB_RW7_RA_MSB_PTR
#define Serial_USB_ARB_RW7_WA     Serial_USB_ARB_RW7_WA_PTR
#define Serial_USB_ARB_RW7_WA_MSB Serial_USB_ARB_RW7_WA_MSB_PTR

#define Serial_USB_ARB_RW8_DR     Serial_USB_ARB_RW8_DR_PTR
#define Serial_USB_ARB_RW8_RA     Serial_USB_ARB_RW8_RA_PTR
#define Serial_USB_ARB_RW8_RA_MSB Serial_USB_ARB_RW8_RA_MSB_PTR
#define Serial_USB_ARB_RW8_WA     Serial_USB_ARB_RW8_WA_PTR
#define Serial_USB_ARB_RW8_WA_MSB Serial_USB_ARB_RW8_WA_MSB_PTR

#define Serial_USB_BUF_SIZE       Serial_USB_BUF_SIZE_PTR
#define Serial_USB_BUS_RST_CNT    Serial_USB_BUS_RST_CNT_PTR
#define Serial_USB_CR0            Serial_USB_CR0_PTR
#define Serial_USB_CR1            Serial_USB_CR1_PTR
#define Serial_USB_CWA            Serial_USB_CWA_PTR
#define Serial_USB_CWA_MSB        Serial_USB_CWA_MSB_PTR

#define Serial_USB_DMA_THRES      Serial_USB_DMA_THRES_PTR
#define Serial_USB_DMA_THRES_MSB  Serial_USB_DMA_THRES_MSB_PTR

#define Serial_USB_EP_ACTIVE      Serial_USB_EP_ACTIVE_PTR
#define Serial_USB_EP_TYPE        Serial_USB_EP_TYPE_PTR

#define Serial_USB_EP0_CNT        Serial_USB_EP0_CNT_PTR
#define Serial_USB_EP0_CR         Serial_USB_EP0_CR_PTR
#define Serial_USB_EP0_DR0        Serial_USB_EP0_DR0_PTR
#define Serial_USB_EP0_DR1        Serial_USB_EP0_DR1_PTR
#define Serial_USB_EP0_DR2        Serial_USB_EP0_DR2_PTR
#define Serial_USB_EP0_DR3        Serial_USB_EP0_DR3_PTR
#define Serial_USB_EP0_DR4        Serial_USB_EP0_DR4_PTR
#define Serial_USB_EP0_DR5        Serial_USB_EP0_DR5_PTR
#define Serial_USB_EP0_DR6        Serial_USB_EP0_DR6_PTR
#define Serial_USB_EP0_DR7        Serial_USB_EP0_DR7_PTR

#define Serial_USB_OSCLK_DR0      Serial_USB_OSCLK_DR0_PTR
#define Serial_USB_OSCLK_DR1      Serial_USB_OSCLK_DR1_PTR

#define Serial_USB_PM_ACT_CFG     Serial_USB_PM_ACT_CFG_PTR
#define Serial_USB_PM_STBY_CFG    Serial_USB_PM_STBY_CFG_PTR

#define Serial_USB_SIE_EP_INT_EN  Serial_USB_SIE_EP_INT_EN_PTR
#define Serial_USB_SIE_EP_INT_SR  Serial_USB_SIE_EP_INT_SR_PTR

#define Serial_USB_SIE_EP1_CNT0   Serial_USB_SIE_EP1_CNT0_PTR
#define Serial_USB_SIE_EP1_CNT1   Serial_USB_SIE_EP1_CNT1_PTR
#define Serial_USB_SIE_EP1_CR0    Serial_USB_SIE_EP1_CR0_PTR

#define Serial_USB_SIE_EP2_CNT0   Serial_USB_SIE_EP2_CNT0_PTR
#define Serial_USB_SIE_EP2_CNT1   Serial_USB_SIE_EP2_CNT1_PTR
#define Serial_USB_SIE_EP2_CR0    Serial_USB_SIE_EP2_CR0_PTR

#define Serial_USB_SIE_EP3_CNT0   Serial_USB_SIE_EP3_CNT0_PTR
#define Serial_USB_SIE_EP3_CNT1   Serial_USB_SIE_EP3_CNT1_PTR
#define Serial_USB_SIE_EP3_CR0    Serial_USB_SIE_EP3_CR0_PTR

#define Serial_USB_SIE_EP4_CNT0   Serial_USB_SIE_EP4_CNT0_PTR
#define Serial_USB_SIE_EP4_CNT1   Serial_USB_SIE_EP4_CNT1_PTR
#define Serial_USB_SIE_EP4_CR0    Serial_USB_SIE_EP4_CR0_PTR

#define Serial_USB_SIE_EP5_CNT0   Serial_USB_SIE_EP5_CNT0_PTR
#define Serial_USB_SIE_EP5_CNT1   Serial_USB_SIE_EP5_CNT1_PTR
#define Serial_USB_SIE_EP5_CR0    Serial_USB_SIE_EP5_CR0_PTR

#define Serial_USB_SIE_EP6_CNT0   Serial_USB_SIE_EP6_CNT0_PTR
#define Serial_USB_SIE_EP6_CNT1   Serial_USB_SIE_EP6_CNT1_PTR
#define Serial_USB_SIE_EP6_CR0    Serial_USB_SIE_EP6_CR0_PTR

#define Serial_USB_SIE_EP7_CNT0   Serial_USB_SIE_EP7_CNT0_PTR
#define Serial_USB_SIE_EP7_CNT1   Serial_USB_SIE_EP7_CNT1_PTR
#define Serial_USB_SIE_EP7_CR0    Serial_USB_SIE_EP7_CR0_PTR

#define Serial_USB_SIE_EP8_CNT0   Serial_USB_SIE_EP8_CNT0_PTR
#define Serial_USB_SIE_EP8_CNT1   Serial_USB_SIE_EP8_CNT1_PTR
#define Serial_USB_SIE_EP8_CR0    Serial_USB_SIE_EP8_CR0_PTR

#define Serial_USB_SOF0           Serial_USB_SOF0_PTR
#define Serial_USB_SOF1           Serial_USB_SOF1_PTR

#define Serial_USB_USB_CLK_EN     Serial_USB_USB_CLK_EN_PTR

#define Serial_USB_USBIO_CR0      Serial_USB_USBIO_CR0_PTR
#define Serial_USB_USBIO_CR1      Serial_USB_USBIO_CR1_PTR
#define Serial_USB_USBIO_CR2      Serial_USB_USBIO_CR2_PTR

#define Serial_USB_DM_INP_DIS_PTR     ( (reg8 *) Serial_USB_Dm__INP_DIS)
#define Serial_USB_DM_INP_DIS_REG     (*(reg8 *) Serial_USB_Dm__INP_DIS)
#define Serial_USB_DP_INP_DIS_PTR     ( (reg8 *) Serial_USB_Dp__INP_DIS)
#define Serial_USB_DP_INP_DIS_REG     (*(reg8 *) Serial_USB_Dp__INP_DIS)
#define Serial_USB_DP_INTSTAT_PTR     ( (reg8 *) Serial_USB_Dp__INTSTAT)
#define Serial_USB_DP_INTSTAT_REG     (*(reg8 *) Serial_USB_Dp__INTSTAT)
#define Serial_USB_DM_MASK            Serial_USB_Dm__0__MASK
#define Serial_USB_DP_MASK            Serial_USB_Dp__0__MASK

#define USBFS_SIE_EP_INT_EP1_MASK        (0x01u)
#define USBFS_SIE_EP_INT_EP2_MASK        (0x02u)
#define USBFS_SIE_EP_INT_EP3_MASK        (0x04u)
#define USBFS_SIE_EP_INT_EP4_MASK        (0x08u)
#define USBFS_SIE_EP_INT_EP5_MASK        (0x10u)
#define USBFS_SIE_EP_INT_EP6_MASK        (0x20u)
#define USBFS_SIE_EP_INT_EP7_MASK        (0x40u)
#define USBFS_SIE_EP_INT_EP8_MASK        (0x80u)

#define Serial_USB_ARB_EPX_SR_IN_BUF_FULL     (0x01u)
#define Serial_USB_ARB_EPX_SR_DMA_GNT         (0x02u)
#define Serial_USB_ARB_EPX_SR_BUF_OVER        (0x04u)
#define Serial_USB_ARB_EPX_SR_BUF_UNDER       (0x08u)

#define Serial_USB_ARB_EPX_INT_EN_ALL Serial_USB_ARB_EPX_INT_ALL

#define Serial_USB_CR1_BUS_ACTIVITY_SHIFT     (0x02u)

#define Serial_USB_BUS_RST_COUNT      Serial_USB_DEFUALT_BUS_RST_CNT

#define Serial_USB_ARB_INT_MASK       Serial_USB_DEFAULT_ARB_INT_EN

#if (CYDEV_CHIP_DIE_EXPECT == CYDEV_CHIP_DIE_LEOPARD)
    /* CY_PSOC3 interrupt registers */
    #define Serial_USB_USB_ISR_PRIOR  ((reg8 *) CYDEV_INTC_PRIOR0)
    #define Serial_USB_USB_ISR_SET_EN ((reg8 *) CYDEV_INTC_SET_EN0)
    #define Serial_USB_USB_ISR_CLR_EN ((reg8 *) CYDEV_INTC_CLR_EN0)
    #define Serial_USB_USB_ISR_VECT   ((cyisraddress *) CYDEV_INTC_VECT_MBASE)
#elif (CYDEV_CHIP_DIE_EXPECT == CYDEV_CHIP_DIE_PANTHER)
    /* CY_PSOC5LP interrupt registers */
    #define Serial_USB_USB_ISR_PRIOR  ((reg8 *) CYDEV_NVIC_PRI_0)
    #define Serial_USB_USB_ISR_SET_EN ((reg8 *) CYDEV_NVIC_SETENA0)
    #define Serial_USB_USB_ISR_CLR_EN ((reg8 *) CYDEV_NVIC_CLRENA0)
    #define Serial_USB_USB_ISR_VECT   ((cyisraddress *) CYDEV_NVIC_VECT_OFFSET)
#endif /*  CYDEV_CHIP_DIE_EXPECT */


#endif /* (CY_USBFS_Serial_USB_H) */


/* [] END OF FILE */
