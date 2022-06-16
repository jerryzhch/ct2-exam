/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_ct_seg7.
 * 
 *  \file   hal_ct_seg7.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_CT_SEG7_H
#define _HAL_CT_SEG7_H


/* Standard includes */
#include <stdint.h>


/* -- Type definitions
 * ------------------------------------------------------------------------- */
 
/**
 *  \enum   hal_ct_seg7_t
 *  \brief  Defines the 7 segment displays on the CT-Board.
 */
typedef enum {
    HAL_CT_SEG7_DS0 =  0u,
    HAL_CT_SEG7_DS1 =  8u,
    HAL_CT_SEG7_DS2 = 16u,
    HAL_CT_SEG7_DS3 = 24u,
} hal_ct_seg7_t;


/**
 *  \enum   hal_ct_seg7_segment_t
 *  \brief  Defines the segments on a 7 segment display.
 */
typedef enum {
    HAL_CT_SEG7_SEGA = 0x01,
    HAL_CT_SEG7_SEGB = 0x02,
    HAL_CT_SEG7_SEGC = 0x04,
    HAL_CT_SEG7_SEGD = 0x08,
    HAL_CT_SEG7_SEGE = 0x10,
    HAL_CT_SEG7_SEGF = 0x20,
    HAL_CT_SEG7_SEGG = 0x40,
    HAL_CT_SEG7_SEGDP = 0x80,
} hal_ct_seg7_segment_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Writes segment data to specified 7 segment display.
 *  \param  display : The display on which shall be written.
 *  \param  segment : The segments, can be combined => SEGB | SEGC => 1.
 */
void hal_ct_seg7_raw_write(hal_ct_seg7_t display, 
                           uint8_t segment);

/**
 *  \brief  Outputs a 16 bit value in hexadecimal represenation on the
 *          seven-segment display.
 *  \param  value : 16 bit binary value to display.
 */
void hal_ct_seg7_bin_write(uint16_t value);
    
/**
 *  \brief  Clear all segments on the specified display.
 *  \param  display : Display to clear.
 */
void hal_ct_seg7_clear(hal_ct_seg7_t display);

/**
 *  \brief  Clear all segments on all displays.
 */
void hal_ct_seg7_clear_all(void);

/**
 *  \brief  Set dot on specified displays.
 *  \param  display : Display to set the dot.
 */
void hal_ct_seg7_dot_set(hal_ct_seg7_t display);

/**
 *  \brief  Reset dot on specified displays.
 *  \param  display : Display to reset the dot.
 */
void hal_ct_seg7_dot_clear(hal_ct_seg7_t display);

/**
 *  \brief  Converts binary value to bcd.
 *  \param  value : Binary value to convert.
 *  \return Converted bcd value. The max. bcd value that can be displayed
 *          with 8bit is 99u.
 */
uint8_t hal_ct_seg7_get_bcd(uint8_t value);

/**
 *  \brief  Converts binary value to 7 segment code.
 *  \param  value : Binary value to convert (4 bit).
 *  \return Converted 7 segment code of 0xf if array index is out of bounds. 
 *          Else converted 7 segment code of value.
 */
uint8_t hal_ct_seg7_get_seg7(uint8_t value);


#endif
