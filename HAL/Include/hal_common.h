/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */ 
/**
 *  \brief  Common #defines and typedefs.
 * 
 *  \file   hal_common.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_COMMON_H
#define _HAL_COMMON_H

/* Standard includes */
#include <stdint.h>


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_bool_t
 *  \brief  Often used TRUE / FALSE type.
 */
typedef enum {
    FALSE = 0u,             /**< FALSE. */
    TRUE = !FALSE,          /**< TRUE, aka. not FALSE. */
    /* Derived */
    DISABLE = FALSE,        /**< Typically used as command -> DISABLE. */
    ENABLE = TRUE,          /**< Typically used as command -> ENABLE. */
    DISABLED = FALSE,       /**< Typically used as status -> DISABLED. */
    ENABLED = TRUE          /**< Typically used as status -> ENABLED. */
} hal_bool_t;


/**
 *  \enum   hal_data_width_t
 *  \brief  Defines standard data width.
 */
typedef enum {
    BYTE = 8u,          /**< Byte. */
    HWORD = 16u,        /**< Halfword. */
    WORD = 32u,         /**< Word. */
    DWORD = 64u         /**< Doubleword. */
} hal_data_width_t;


/**
 *  \enum   hal_peripheral_t
 *  \brief  Defines a peripheral.
 */
typedef enum {
    PER_ADC1,       // APB2
    PER_ADC2,
    PER_ADC3,
    
    PER_DAC,        // APB1
    
    PER_DMA1,       // AHB1
    PER_DMA2,
    
    PER_FMC,        // AHB3
    
    PER_GPIOA,      // AHB1
    PER_GPIOB,
    PER_GPIOC,
    PER_GPIOD,
    PER_GPIOE,
    PER_GPIOF,
    PER_GPIOG,
    PER_GPIOH,
    PER_GPIOI,
    PER_GPIOJ,
    PER_GPIOK,
    
    PER_PWR,        // APB1
    
    PER_TIM2,       // APB1
    PER_TIM3,
    PER_TIM4,
    PER_TIM5
} hal_peripheral_t;


#endif
