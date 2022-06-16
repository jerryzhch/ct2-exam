/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_systick.
 * 
 *  \file   hal_systick.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_SYSTICK_H
#define _HAL_SYSTICK_H


/* Standard includes */
#include <stdint.h>


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_systick_clk_t
 *  \brief  Available clocks for the systick timer.
 */
typedef enum {
    HAL_RCC_CLK_HCLK8 = 0x0,    /** < HCLK / 8. */
    HAL_RCC_CLK_HCLK = 0x1      /** < HCLK. */
} hal_systick_clk_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets all involved registers.
 */
void hal_systick_reset(void);

/**
 *  \brief  Initializes the SysTick timer.
 *  \param  clock : Define used clock.
 *  \param  reload : 24 bit reload value.
 */
void hal_systick_init(hal_systick_clk_t clock, uint32_t reload);

/**
 *  \brief  Pauses the SysTick timer.
 */
void hal_systick_pause(void);

/**
 *  \brief  Resumes the SysTick timer.
 */
void hal_systick_resume(void);


#endif
