#ifndef __NEXBUTTON_H__
#define __NEXBUTTON_H__

#include "NexTouch.h"
#include "NexHardware.h"
//#include "..\util\my_types.h"

uint16_t NexButton_getText(struct NexObject *button, char *buffer, uint16_t len);

/**
     * Set text attribute of component.
     *
     * @param buffer - text buffer terminated with '\0'. 
     * @return true if success, false for failure. 
     */
uint8_t NexButton_setText(struct NexObject *button, const char *buffer);

/**
     * Get bco attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_background_color_bco(struct NexObject *button, uint32_t *number);

/**
     * Set bco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_background_color_bco(struct NexObject *button, uint32_t number);

/**
     * Get bco2 attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_press_background_color_bco2(struct NexObject *button, uint32_t *number);

/**
     * Set bco2 attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_press_background_color_bco2(struct NexObject *button, uint32_t number);

/**
     * Get pco attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_font_color_pco(struct NexObject *button, uint32_t *number);

/**
     * Set pco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_font_color_pco(struct NexObject *button, uint32_t number);

/**
     * Get pco2 attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_press_font_color_pco2(struct NexObject *button, uint32_t *number);

/**
     * Set pco2 attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_press_font_color_pco2(struct NexObject *button, uint32_t number);

/**
     * Get xcen attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_place_xcen(struct NexObject *button, uint32_t *number);

/**
     * Set xcen attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_place_xcen(struct NexObject *button, uint32_t number);

/**
     * Get ycen attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_place_ycen(struct NexObject *button, uint32_t *number);

/**
     * Set ycen attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_place_ycen(struct NexObject *button, uint32_t number);

/**
     * Get font attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_getFont(struct NexObject *button, uint32_t *number);

/**
     * Set font attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_setFont(struct NexObject *button, uint32_t number);

/**
     * Get picc attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_background_cropi_picc(struct NexObject *button, uint32_t *number);

/**
     * Set picc attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_background_crop_picc(struct NexObject *button, uint32_t number);

/**
     * Get picc2 attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_press_background_crop_picc2(struct NexObject *button, uint32_t *number);

/**
     * Set picc2 attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_press_background_crop_picc2(struct NexObject *button, uint32_t number);

/**
     * Get pic attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_background_image_pic(struct NexObject *button, uint32_t *number);

/**
     * Set pic attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_background_image_pic(struct NexObject *button, uint32_t number);

/**
     * Get pic2 attribute of component
     *
     * @param number - buffer storing data return
     * @return the length of the data 
     */
uint32_t NexButton_Get_press_background_image_pic2(struct NexObject *button, uint32_t *number);

/**
     * Set pic2 attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
uint8_t NexButton_Set_press_background_image_pic2(struct NexObject *button, uint32_t number);

#endif /* #ifndef __NEXBUTTON_H__ */
