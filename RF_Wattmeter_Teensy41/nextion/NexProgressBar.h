#ifndef __NEXPROGRESSBAR_H__
#define __NEXPROGRESSBAR_H__
#include "NexTouch.h"
#include "NexHardware.h"
#include "..\util\Utilities.h"

/**
     * Get the value of progress bar. 
     * 
     * @param number - an output parameter to save the value of progress bar.  
     * 
     * @retval true - success. 
     * @retval false - failed. 
     */
unsigned char NexProgressBar_getValue(struct NexObject *bar, unsigned long *number);

/**
     * Set the value of progress bar.
     *
     * @param number - the value of progress bar.  
     *
     * @retval true - success. 
     * @retval false - failed. 
     */
unsigned char NexProgressBar_setValue(struct NexObject *bar, unsigned long number);

/**
     * Get bco attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexProgressBar_Get_background_color_bco(struct NexObject *bar, unsigned long *number);

/**
     * Set bco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexProgressBar_Set_background_color_bco(struct NexObject *bar, unsigned long number);

/**
     * Get pco attribute of component
     *
     * @param number - buffer storing data retur
     * @return the length of the data 
     */
unsigned long NexProgressBar_Get_font_color_pco(struct NexObject *bar, unsigned long *number);

/**
     * Set pco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
unsigned char NexProgressBar_Set_font_color_pco(struct NexObject *bar, unsigned long number);

#endif /* #ifndef __NEXPROGRESSBAR_H__ */