/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_ct_lcd.
 * 
 *  \file   hal_ct_lcd.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_CT_LCD_H
#define _HAL_CT_LCD_H

/* Standard includes */
#include <stdint.h>


/* -- Type definitions
 * ------------------------------------------------------------------------- */


/**
 *  \enum   hal_ct_lcd_color_t
 *  \brief  Defines the DIPSW banks on the CT-Board.
 */
typedef enum {
    HAL_LCD_RED,
    HAL_LCD_GREEN,
    HAL_LCD_BLUE
} hal_ct_lcd_color_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Write string of text on LCD.
 *  \param  position : Starting Position to write text.
 *  \param  text : Text to write.
 */
void hal_ct_lcd_write(uint8_t position, char text[]);

/**
 *  \brief  Set the backround color of the LCD.
 *  \param  color : Color to set.
 *  \param  value : PWM value.
 */
void hal_ct_lcd_color(hal_ct_lcd_color_t color, uint16_t value);

/**
 *  \brief  Cleares the LCD display.
 */
void hal_ct_lcd_clear(void);



#endif
