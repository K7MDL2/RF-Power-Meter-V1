/***************************************************************************//**
* \file Serial_USB_audio.h
* \version 3.20
*
* \brief
*  This file provides function prototypes and constants for the USBFS component 
*  Audio class.
*
* Related Document:
*  Universal Serial Bus Device Class Definition for Audio Devices Release 1.0
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_USBFS_Serial_USB_audio_H)
#define CY_USBFS_Serial_USB_audio_H

#include "Serial_USB.h"


/***************************************
* Custom Declarations
***************************************/

/* `#START CUSTOM_CONSTANTS` Place your declaration here */

/* `#END` */


/***************************************
*  Constants for Serial_USB_audio API.
***************************************/

/* Audio Class-Specific Request Codes (AUDIO Table A-9) */
#define Serial_USB_REQUEST_CODE_UNDEFINED     (0x00u)
#define Serial_USB_SET_CUR                    (0x01u)
#define Serial_USB_GET_CUR                    (0x81u)
#define Serial_USB_SET_MIN                    (0x02u)
#define Serial_USB_GET_MIN                    (0x82u)
#define Serial_USB_SET_MAX                    (0x03u)
#define Serial_USB_GET_MAX                    (0x83u)
#define Serial_USB_SET_RES                    (0x04u)
#define Serial_USB_GET_RES                    (0x84u)
#define Serial_USB_SET_MEM                    (0x05u)
#define Serial_USB_GET_MEM                    (0x85u)
#define Serial_USB_GET_STAT                   (0xFFu)

/* point Control Selectors (AUDIO Table A-19) */
#define Serial_USB_EP_CONTROL_UNDEFINED       (0x00u)
#define Serial_USB_SAMPLING_FREQ_CONTROL      (0x01u)
#define Serial_USB_PITCH_CONTROL              (0x02u)

/* Feature Unit Control Selectors (AUDIO Table A-11) */
#define Serial_USB_FU_CONTROL_UNDEFINED       (0x00u)
#define Serial_USB_MUTE_CONTROL               (0x01u)
#define Serial_USB_VOLUME_CONTROL             (0x02u)
#define Serial_USB_BASS_CONTROL               (0x03u)
#define Serial_USB_MID_CONTROL                (0x04u)
#define Serial_USB_TREBLE_CONTROL             (0x05u)
#define Serial_USB_GRAPHIC_EQUALIZER_CONTROL  (0x06u)
#define Serial_USB_AUTOMATIC_GAIN_CONTROL     (0x07u)
#define Serial_USB_DELAY_CONTROL              (0x08u)
#define Serial_USB_BASS_BOOST_CONTROL         (0x09u)
#define Serial_USB_LOUDNESS_CONTROL           (0x0Au)

#define Serial_USB_SAMPLE_FREQ_LEN            (3u)
#define Serial_USB_VOLUME_LEN                 (2u)

#if !defined(USER_SUPPLIED_DEFAULT_VOLUME_VALUE)
    #define Serial_USB_VOL_MIN_MSB            (0x80u)
    #define Serial_USB_VOL_MIN_LSB            (0x01u)
    #define Serial_USB_VOL_MAX_MSB            (0x7Fu)
    #define Serial_USB_VOL_MAX_LSB            (0xFFu)
    #define Serial_USB_VOL_RES_MSB            (0x00u)
    #define Serial_USB_VOL_RES_LSB            (0x01u)
#endif /* USER_SUPPLIED_DEFAULT_VOLUME_VALUE */


/***************************************
* External data references
***************************************/
/**
* \addtogroup group_audio
* @{
*/
extern volatile uint8 Serial_USB_currentSampleFrequency[Serial_USB_MAX_EP][Serial_USB_SAMPLE_FREQ_LEN];
extern volatile uint8 Serial_USB_frequencyChanged;
extern volatile uint8 Serial_USB_currentMute;
extern volatile uint8 Serial_USB_currentVolume[Serial_USB_VOLUME_LEN];
/** @} audio */

extern volatile uint8 Serial_USB_minimumVolume[Serial_USB_VOLUME_LEN];
extern volatile uint8 Serial_USB_maximumVolume[Serial_USB_VOLUME_LEN];
extern volatile uint8 Serial_USB_resolutionVolume[Serial_USB_VOLUME_LEN];

#endif /*  CY_USBFS_Serial_USB_audio_H */


/* [] END OF FILE */
