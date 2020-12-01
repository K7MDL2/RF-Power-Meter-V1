#ifndef __NEXSLIDER_H__
#define __NEXSLIDER_H__
#include "..\util\Utilities.h"
#include "NexTouch.h"
#include "NexHardware.h"

/**
     * Get the value of slider. 
     * 
     * @param number - an output parameter to save the value of slider.  
     * 
     * @retval true - success. 
     * @retval false - failed. 
     */
unsigned char NexSlider_getValue(struct NexObject *slider, unsigned long *number);

/**
     * Set the value of slider.
     *
     * @param number - the value of slider.  
     *
     * @retval true - success. 
     * @retval false - failed. 
     */
unsigned char NexSlider_setValue(struct NexObject *slider, unsigned long number);

/**
     * Get bco attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexSlider_Get_background_color_bco(struct NexObject *slider, unsigned long *number);

/**
     * Set bco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexSlider_Set_background_color_bco(struct NexObject *slider, unsigned long number);

/**
     * Get pco attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexSlider_Get_font_color_pco(struct NexObject *slider, unsigned long *number);

/**
     * Set pco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexSlider_Set_font_color_pco(struct NexObject *slider, unsigned long number);

/**
     * Get wid attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexSlider_Get_pointer_thickness_wid(struct NexObject *slider, unsigned long *number);

/**
     * Set wid attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexSlider_Set_pointer_thickness_wid(struct NexObject *slider, unsigned long number);

/**
     * Get hig attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexSlider_Get_cursor_height_hig(struct NexObject *slider, unsigned long *number);

/**
     * Set hig attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexSlider_Set_cursor_height_hig(struct NexObject *slider, unsigned long number);

/**
     * Get maxval attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexSlider_getMaxval(struct NexObject *slider, unsigned long *number);

/**
     * Set maxval attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexSlider_setMaxval(struct NexObject *slider, unsigned long number);

/**
     * Get minval attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexSlider_getMinval(struct NexObject *slider, unsigned long *number);

/**
     * Set minval attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexSlider_setMinval(struct NexObject *slider, unsigned long number);

#endif /* #ifndef __NEXSLIDER_H__ */