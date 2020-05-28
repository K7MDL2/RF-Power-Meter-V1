/***************************************************************************//**
* \file .h
* \version 3.20
*
* \brief
*  This file provides private function prototypes and constants for the 
*  USBFS component. It is not intended to be used in the user project.
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_USBFS_Serial_USB_pvt_H)
#define CY_USBFS_Serial_USB_pvt_H

#include "Serial_USB.h"
   
#ifdef Serial_USB_ENABLE_AUDIO_CLASS
    #include "Serial_USB_audio.h"
#endif /* Serial_USB_ENABLE_AUDIO_CLASS */

#ifdef Serial_USB_ENABLE_CDC_CLASS
    #include "Serial_USB_cdc.h"
#endif /* Serial_USB_ENABLE_CDC_CLASS */

#if (Serial_USB_ENABLE_MIDI_CLASS)
    #include "Serial_USB_midi.h"
#endif /* (Serial_USB_ENABLE_MIDI_CLASS) */

#if (Serial_USB_ENABLE_MSC_CLASS)
    #include "Serial_USB_msc.h"
#endif /* (Serial_USB_ENABLE_MSC_CLASS) */

#if (Serial_USB_EP_MANAGEMENT_DMA)
    #if (CY_PSOC4)
        #include <CyDMA.h>
    #else
        #include <CyDmac.h>
        #if ((Serial_USB_EP_MANAGEMENT_DMA_AUTO) && (Serial_USB_EP_DMA_AUTO_OPT == 0u))
            #include "Serial_USB_EP_DMA_Done_isr.h"
            #include "Serial_USB_EP8_DMA_Done_SR.h"
            #include "Serial_USB_EP17_DMA_Done_SR.h"
        #endif /* ((Serial_USB_EP_MANAGEMENT_DMA_AUTO) && (Serial_USB_EP_DMA_AUTO_OPT == 0u)) */
    #endif /* (CY_PSOC4) */
#endif /* (Serial_USB_EP_MANAGEMENT_DMA) */

#if (Serial_USB_DMA1_ACTIVE)
    #include "Serial_USB_ep1_dma.h"
    #define Serial_USB_EP1_DMA_CH     (Serial_USB_ep1_dma_CHANNEL)
#endif /* (Serial_USB_DMA1_ACTIVE) */

#if (Serial_USB_DMA2_ACTIVE)
    #include "Serial_USB_ep2_dma.h"
    #define Serial_USB_EP2_DMA_CH     (Serial_USB_ep2_dma_CHANNEL)
#endif /* (Serial_USB_DMA2_ACTIVE) */

#if (Serial_USB_DMA3_ACTIVE)
    #include "Serial_USB_ep3_dma.h"
    #define Serial_USB_EP3_DMA_CH     (Serial_USB_ep3_dma_CHANNEL)
#endif /* (Serial_USB_DMA3_ACTIVE) */

#if (Serial_USB_DMA4_ACTIVE)
    #include "Serial_USB_ep4_dma.h"
    #define Serial_USB_EP4_DMA_CH     (Serial_USB_ep4_dma_CHANNEL)
#endif /* (Serial_USB_DMA4_ACTIVE) */

#if (Serial_USB_DMA5_ACTIVE)
    #include "Serial_USB_ep5_dma.h"
    #define Serial_USB_EP5_DMA_CH     (Serial_USB_ep5_dma_CHANNEL)
#endif /* (Serial_USB_DMA5_ACTIVE) */

#if (Serial_USB_DMA6_ACTIVE)
    #include "Serial_USB_ep6_dma.h"
    #define Serial_USB_EP6_DMA_CH     (Serial_USB_ep6_dma_CHANNEL)
#endif /* (Serial_USB_DMA6_ACTIVE) */

#if (Serial_USB_DMA7_ACTIVE)
    #include "Serial_USB_ep7_dma.h"
    #define Serial_USB_EP7_DMA_CH     (Serial_USB_ep7_dma_CHANNEL)
#endif /* (Serial_USB_DMA7_ACTIVE) */

#if (Serial_USB_DMA8_ACTIVE)
    #include "Serial_USB_ep8_dma.h"
    #define Serial_USB_EP8_DMA_CH     (Serial_USB_ep8_dma_CHANNEL)
#endif /* (Serial_USB_DMA8_ACTIVE) */


/***************************************
*     Private Variables
***************************************/

/* Generated external references for descriptors. */
extern const uint8 CYCODE Serial_USB_DEVICE0_DESCR[18u];
extern const uint8 CYCODE Serial_USB_DEVICE0_CONFIGURATION0_DESCR[67u];
extern const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE Serial_USB_DEVICE0_CONFIGURATION0_EP_SETTINGS_TABLE[3u];
extern const uint8 CYCODE Serial_USB_DEVICE0_CONFIGURATION0_INTERFACE_CLASS[2u];
extern const T_Serial_USB_LUT CYCODE Serial_USB_DEVICE0_CONFIGURATION0_TABLE[5u];
extern const T_Serial_USB_LUT CYCODE Serial_USB_DEVICE0_TABLE[3u];
extern const T_Serial_USB_LUT CYCODE Serial_USB_TABLE[1u];
extern const uint8 CYCODE Serial_USB_SN_STRING_DESCRIPTOR[2];
extern const uint8 CYCODE Serial_USB_STRING_DESCRIPTORS[177u];


extern const uint8 CYCODE Serial_USB_MSOS_DESCRIPTOR[Serial_USB_MSOS_DESCRIPTOR_LENGTH];
extern const uint8 CYCODE Serial_USB_MSOS_CONFIGURATION_DESCR[Serial_USB_MSOS_CONF_DESCR_LENGTH];
#if defined(Serial_USB_ENABLE_IDSN_STRING)
    extern uint8 Serial_USB_idSerialNumberStringDescriptor[Serial_USB_IDSN_DESCR_LENGTH];
#endif /* (Serial_USB_ENABLE_IDSN_STRING) */

extern volatile uint8 Serial_USB_interfaceNumber;
extern volatile uint8 Serial_USB_interfaceSetting[Serial_USB_MAX_INTERFACES_NUMBER];
extern volatile uint8 Serial_USB_interfaceSettingLast[Serial_USB_MAX_INTERFACES_NUMBER];
extern volatile uint8 Serial_USB_deviceAddress;
extern volatile uint8 Serial_USB_interfaceStatus[Serial_USB_MAX_INTERFACES_NUMBER];
extern const uint8 CYCODE *Serial_USB_interfaceClass;

extern volatile T_Serial_USB_EP_CTL_BLOCK Serial_USB_EP[Serial_USB_MAX_EP];
extern volatile T_Serial_USB_TD Serial_USB_currentTD;

#if (Serial_USB_EP_MANAGEMENT_DMA)
    #if (CY_PSOC4)
        extern const uint8 Serial_USB_DmaChan[Serial_USB_MAX_EP];
    #else
        extern uint8 Serial_USB_DmaChan[Serial_USB_MAX_EP];
        extern uint8 Serial_USB_DmaTd  [Serial_USB_MAX_EP];
    #endif /* (CY_PSOC4) */
#endif /* (Serial_USB_EP_MANAGEMENT_DMA) */

#if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
#if (CY_PSOC4)
    extern uint8  Serial_USB_DmaEpBurstCnt   [Serial_USB_MAX_EP];
    extern uint8  Serial_USB_DmaEpLastBurstEl[Serial_USB_MAX_EP];

    extern uint8  Serial_USB_DmaEpBurstCntBackup  [Serial_USB_MAX_EP];
    extern uint32 Serial_USB_DmaEpBufferAddrBackup[Serial_USB_MAX_EP];
    
    extern const uint8 Serial_USB_DmaReqOut     [Serial_USB_MAX_EP];    
    extern const uint8 Serial_USB_DmaBurstEndOut[Serial_USB_MAX_EP];
#else
    #if (Serial_USB_EP_DMA_AUTO_OPT == 0u)
        extern uint8 Serial_USB_DmaNextTd[Serial_USB_MAX_EP];
        extern volatile uint16 Serial_USB_inLength [Serial_USB_MAX_EP];
        extern volatile uint8  Serial_USB_inBufFull[Serial_USB_MAX_EP];
        extern const uint8 Serial_USB_epX_TD_TERMOUT_EN[Serial_USB_MAX_EP];
        extern const uint8 *Serial_USB_inDataPointer[Serial_USB_MAX_EP];
    #endif /* (Serial_USB_EP_DMA_AUTO_OPT == 0u) */
#endif /* CY_PSOC4 */
#endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */

extern volatile uint8 Serial_USB_ep0Toggle;
extern volatile uint8 Serial_USB_lastPacketSize;
extern volatile uint8 Serial_USB_ep0Mode;
extern volatile uint8 Serial_USB_ep0Count;
extern volatile uint16 Serial_USB_transferByteCount;


/***************************************
*     Private Function Prototypes
***************************************/
void  Serial_USB_ReInitComponent(void)            ;
void  Serial_USB_HandleSetup(void)                ;
void  Serial_USB_HandleIN(void)                   ;
void  Serial_USB_HandleOUT(void)                  ;
void  Serial_USB_LoadEP0(void)                    ;
uint8 Serial_USB_InitControlRead(void)            ;
uint8 Serial_USB_InitControlWrite(void)           ;
void  Serial_USB_ControlReadDataStage(void)       ;
void  Serial_USB_ControlReadStatusStage(void)     ;
void  Serial_USB_ControlReadPrematureStatus(void) ;
uint8 Serial_USB_InitControlWrite(void)           ;
uint8 Serial_USB_InitZeroLengthControlTransfer(void) ;
void  Serial_USB_ControlWriteDataStage(void)      ;
void  Serial_USB_ControlWriteStatusStage(void)    ;
void  Serial_USB_ControlWritePrematureStatus(void);
uint8 Serial_USB_InitNoDataControlTransfer(void)  ;
void  Serial_USB_NoDataControlStatusStage(void)   ;
void  Serial_USB_InitializeStatusBlock(void)      ;
void  Serial_USB_UpdateStatusBlock(uint8 completionCode) ;
uint8 Serial_USB_DispatchClassRqst(void)          ;

void Serial_USB_Config(uint8 clearAltSetting) ;
void Serial_USB_ConfigAltChanged(void)        ;
void Serial_USB_ConfigReg(void)               ;
void Serial_USB_EpStateInit(void)             ;


const T_Serial_USB_LUT CYCODE *Serial_USB_GetConfigTablePtr(uint8 confIndex);
const T_Serial_USB_LUT CYCODE *Serial_USB_GetDeviceTablePtr(void)           ;
#if (Serial_USB_BOS_ENABLE)
    const T_Serial_USB_LUT CYCODE *Serial_USB_GetBOSPtr(void)               ;
#endif /* (Serial_USB_BOS_ENABLE) */
const uint8 CYCODE *Serial_USB_GetInterfaceClassTablePtr(void)                    ;
uint8 Serial_USB_ClearEndpointHalt(void)                                          ;
uint8 Serial_USB_SetEndpointHalt(void)                                            ;
uint8 Serial_USB_ValidateAlternateSetting(void)                                   ;

void Serial_USB_SaveConfig(void)      ;
void Serial_USB_RestoreConfig(void)   ;

#if (CY_PSOC3 || CY_PSOC5LP)
    #if (Serial_USB_EP_MANAGEMENT_DMA_AUTO && (Serial_USB_EP_DMA_AUTO_OPT == 0u))
        void Serial_USB_LoadNextInEP(uint8 epNumber, uint8 mode)  ;
    #endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO && (Serial_USB_EP_DMA_AUTO_OPT == 0u)) */
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

#if defined(Serial_USB_ENABLE_IDSN_STRING)
    void Serial_USB_ReadDieID(uint8 descr[])  ;
#endif /* Serial_USB_ENABLE_IDSN_STRING */

#if defined(Serial_USB_ENABLE_HID_CLASS)
    uint8 Serial_USB_DispatchHIDClassRqst(void) ;
#endif /* (Serial_USB_ENABLE_HID_CLASS) */

#if defined(Serial_USB_ENABLE_AUDIO_CLASS)
    uint8 Serial_USB_DispatchAUDIOClassRqst(void) ;
#endif /* (Serial_USB_ENABLE_AUDIO_CLASS) */

#if defined(Serial_USB_ENABLE_CDC_CLASS)
    uint8 Serial_USB_DispatchCDCClassRqst(void) ;
#endif /* (Serial_USB_ENABLE_CDC_CLASS) */

#if (Serial_USB_ENABLE_MSC_CLASS)
    #if (Serial_USB_HANDLE_MSC_REQUESTS)
        uint8 Serial_USB_DispatchMSCClassRqst(void) ;
    #endif /* (Serial_USB_HANDLE_MSC_REQUESTS) */
#endif /* (Serial_USB_ENABLE_MSC_CLASS */

CY_ISR_PROTO(Serial_USB_EP_0_ISR);
CY_ISR_PROTO(Serial_USB_BUS_RESET_ISR);

#if (Serial_USB_SOF_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_SOF_ISR);
#endif /* (Serial_USB_SOF_ISR_ACTIVE) */

#if (Serial_USB_EP1_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_1_ISR);
#endif /* (Serial_USB_EP1_ISR_ACTIVE) */

#if (Serial_USB_EP2_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_2_ISR);
#endif /* (Serial_USB_EP2_ISR_ACTIVE) */

#if (Serial_USB_EP3_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_3_ISR);
#endif /* (Serial_USB_EP3_ISR_ACTIVE) */

#if (Serial_USB_EP4_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_4_ISR);
#endif /* (Serial_USB_EP4_ISR_ACTIVE) */

#if (Serial_USB_EP5_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_5_ISR);
#endif /* (Serial_USB_EP5_ISR_ACTIVE) */

#if (Serial_USB_EP6_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_6_ISR);
#endif /* (Serial_USB_EP6_ISR_ACTIVE) */

#if (Serial_USB_EP7_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_7_ISR);
#endif /* (Serial_USB_EP7_ISR_ACTIVE) */

#if (Serial_USB_EP8_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_EP_8_ISR);
#endif /* (Serial_USB_EP8_ISR_ACTIVE) */

#if (Serial_USB_EP_MANAGEMENT_DMA)
    CY_ISR_PROTO(Serial_USB_ARB_ISR);
#endif /* (Serial_USB_EP_MANAGEMENT_DMA) */

#if (Serial_USB_DP_ISR_ACTIVE)
    CY_ISR_PROTO(Serial_USB_DP_ISR);
#endif /* (Serial_USB_DP_ISR_ACTIVE) */

#if (CY_PSOC4)
    CY_ISR_PROTO(Serial_USB_INTR_HI_ISR);
    CY_ISR_PROTO(Serial_USB_INTR_MED_ISR);
    CY_ISR_PROTO(Serial_USB_INTR_LO_ISR);
    #if (Serial_USB_LPM_ACTIVE)
        CY_ISR_PROTO(Serial_USB_LPM_ISR);
    #endif /* (Serial_USB_LPM_ACTIVE) */
#endif /* (CY_PSOC4) */

#if (Serial_USB_EP_MANAGEMENT_DMA_AUTO)
#if (CY_PSOC4)
    #if (Serial_USB_DMA1_ACTIVE)
        void Serial_USB_EP1_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA1_ACTIVE) */

    #if (Serial_USB_DMA2_ACTIVE)
        void Serial_USB_EP2_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA2_ACTIVE) */

    #if (Serial_USB_DMA3_ACTIVE)
        void Serial_USB_EP3_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA3_ACTIVE) */

    #if (Serial_USB_DMA4_ACTIVE)
        void Serial_USB_EP4_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA4_ACTIVE) */

    #if (Serial_USB_DMA5_ACTIVE)
        void Serial_USB_EP5_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA5_ACTIVE) */

    #if (Serial_USB_DMA6_ACTIVE)
        void Serial_USB_EP6_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA6_ACTIVE) */

    #if (Serial_USB_DMA7_ACTIVE)
        void Serial_USB_EP7_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA7_ACTIVE) */

    #if (Serial_USB_DMA8_ACTIVE)
        void Serial_USB_EP8_DMA_DONE_ISR(void);
    #endif /* (Serial_USB_DMA8_ACTIVE) */

#else
    #if (Serial_USB_EP_DMA_AUTO_OPT == 0u)
        CY_ISR_PROTO(Serial_USB_EP_DMA_DONE_ISR);
    #endif /* (Serial_USB_EP_DMA_AUTO_OPT == 0u) */
#endif /* (CY_PSOC4) */
#endif /* (Serial_USB_EP_MANAGEMENT_DMA_AUTO) */


/***************************************
*         Request Handlers
***************************************/

uint8 Serial_USB_HandleStandardRqst(void) ;
uint8 Serial_USB_DispatchClassRqst(void)  ;
uint8 Serial_USB_HandleVendorRqst(void)   ;


/***************************************
*    HID Internal references
***************************************/

#if defined(Serial_USB_ENABLE_HID_CLASS)
    void Serial_USB_FindReport(void)            ;
    void Serial_USB_FindReportDescriptor(void)  ;
    void Serial_USB_FindHidClassDecriptor(void) ;
#endif /* Serial_USB_ENABLE_HID_CLASS */


/***************************************
*    MIDI Internal references
***************************************/

#if defined(Serial_USB_ENABLE_MIDI_STREAMING)
    void Serial_USB_MIDI_IN_EP_Service(void)  ;
#endif /* (Serial_USB_ENABLE_MIDI_STREAMING) */


/***************************************
*    CDC Internal references
***************************************/

#if defined(Serial_USB_ENABLE_CDC_CLASS)

    typedef struct
    {
        uint8  bRequestType;
        uint8  bNotification;
        uint8  wValue;
        uint8  wValueMSB;
        uint8  wIndex;
        uint8  wIndexMSB;
        uint8  wLength;
        uint8  wLengthMSB;
        uint8  wSerialState;
        uint8  wSerialStateMSB;
    } t_Serial_USB_cdc_notification;

    uint8 Serial_USB_GetInterfaceComPort(uint8 interface) ;
    uint8 Serial_USB_Cdc_EpInit( const T_Serial_USB_EP_SETTINGS_BLOCK CYCODE *pEP, uint8 epNum, uint8 cdcComNums) ;

    extern volatile uint8  Serial_USB_cdc_dataInEpList[Serial_USB_MAX_MULTI_COM_NUM];
    extern volatile uint8  Serial_USB_cdc_dataOutEpList[Serial_USB_MAX_MULTI_COM_NUM];
    extern volatile uint8  Serial_USB_cdc_commInEpList[Serial_USB_MAX_MULTI_COM_NUM];
#endif /* (Serial_USB_ENABLE_CDC_CLASS) */


#endif /* CY_USBFS_Serial_USB_pvt_H */


/* [] END OF FILE */
