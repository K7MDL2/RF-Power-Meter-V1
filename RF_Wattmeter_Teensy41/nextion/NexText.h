#ifndef __NEXTEXT_H__
#define __NEXTEXT_H__
#include "NexTouch.h"
#include "NexHardware.h"
#include "..\util\Utilities.h"
/**
     * Get text attribute of component.
     *
     * @param buffer - buffer storing text returned. 
     * @param len - length of buffer. 
     * @return The real length of text returned. 
     */
unsigned int NexText_getText(struct NexObject *text, char *buffer, unsigned int len);

/**
     * Set text attribute of component.
     *
     * @param buffer - text buffer terminated with '\0'. 
     * @return true if success, false for failure. 
     */
unsigned char NexText_setText(struct NexObject *text);    //, char *buffer);  // *buffer results in endless string copy loop and crash

/**
     * Get bco attribute of component
     *
     * @param *text is object struct
     * sing global cmd to get string
     * @return the length of the data 
     */
unsigned long NexText_Get_background_color_bco(struct NexObject *text, unsigned long *number);

/**
     * Set bco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_Set_background_color_bco(struct NexObject *text, unsigned long number);

/**
     * Get pco attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexText_Get_font_color_pco(struct NexObject *text, unsigned long *number);

/**
     * Set pco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_Set_font_color_pco(struct NexObject *text, unsigned long number);

/**
     * Get xcen attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexText_Get_place_xcen(struct NexObject *text, unsigned long *number);

/**
     * Set xcen attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_Set_place_xcen(struct NexObject *text, unsigned long number);

/**
     * Get ycen attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexText_Get_place_ycen(struct NexObject *text, unsigned long *number);

/**
     * Set ycen attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_Set_place_ycen(struct NexObject *text, unsigned long number);

/**
     * Get font attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexText_getFont(struct NexObject *text, unsigned long *number);

/**
     * Set font attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_setFont(struct NexObject *text, unsigned long number);

/**
     * Get picc attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexText_Get_background_crop_picc(struct NexObject *text, unsigned long *number);

/**
     * Set picc attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_Set_background_crop_picc(struct NexObject *text, unsigned long number);

/**
     * Get pic attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexText_Get_background_image_pic(struct NexObject *text, unsigned long *number);

/**
     * Set pic attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexText_Set_background_image_pic(struct NexObject *text, unsigned long number);

#endif /* #ifndef __NEXTEXT_H__ */