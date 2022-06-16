/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_rcc.
 *
 *  The hardware abstraction layer for the reset and clock control unit.
 *
 *  \file   hal_rcc.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_RCC_H
#define _HAL_RCC_H


/* User includes */
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_rcc_osc_t
 *  \brief  Defines the different oscillators of the SoC.
 */
typedef enum {
    HAL_RCC_OSC_HSI = 0u,
    HAL_RCC_OSC_HSE = 16u,
    HAL_RCC_OSC_PLL = 24u,
    HAL_RCC_OSC_PLLI2S = 26u,
    HAL_RCC_OSC_PLLSAI = 28u
} hal_rcc_osc_t;


/**
 *  \enum   hal_rcc_hpre_t
 *  \brief  Defines available divider for the advanced high-performance bus.
 */
typedef enum {
    HAL_RCC_HPRE_1 = 0u,
    HAL_RCC_HPRE_2 = 8u,
    HAL_RCC_HPRE_4 = 9u,
    HAL_RCC_HPRE_8 = 10u,
    HAL_RCC_HPRE_16 = 11u,
    HAL_RCC_HPRE_64 = 12u,
    HAL_RCC_HPRE_128 = 13u,
    HAL_RCC_HPRE_256 = 14u,
    HAL_RCC_HPRE_512 = 15u
} hal_rcc_hpre_t;


/**
 *  \enum   hal_rcc_ppre_t
 *  \brief  Defines available divider for the advanced peripheral buses.
 */
typedef enum {
    HAL_RCC_PPRE_2 = 4u,
    HAL_RCC_PPRE_4 = 5u,
    HAL_RCC_PPRE_8 = 6u,
    HAL_RCC_PPRE_16 = 7u
} hal_rcc_ppre_t;


/**
 *  \struct hal_rcc_pll_init_t
 *  \brief  Initialization structure for pll configuration.
 */
typedef struct {
    hal_rcc_osc_t source;
    uint16_t n_factor;
    uint8_t p_divider;
    uint8_t q_divider;
    uint8_t r_divider;
    /* Only for main pll */
    uint8_t m_divider;
} hal_rcc_pll_init_t;


/**
 *  \struct hal_rcc_clk_init_t
 *  \brief  Initialization structure for system clock configuration.
 */
typedef struct {
    hal_rcc_osc_t osc;
    hal_rcc_hpre_t hpre;
    hal_rcc_ppre_t ppre1;
    hal_rcc_ppre_t ppre2;
} hal_rcc_clk_init_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets all involved registers.
 */
void hal_rcc_reset(void);

/**
 *  \brief  Enables/Disables the defined periphery. 
 *  \param  peripheral : Defines the peripheral to ENABLE/DISABLE.
 *  \param  status : ENABLE/DISABLE the peripheral clock.
 */
void hal_rcc_set_peripheral(hal_peripheral_t peripheral, hal_bool_t status)
__attribute__((deprecated("Please use XXX_ENABLE() or XXX_DISABLE().")));

/**
 *  \brief  Enables the defined clock source. 
 *  If a pll is choosen, make sure the configured source clock
 *  is up and running.
 *  \param  osc : Defines the oscillator to ENABLE/DISABLE.
 *  \param  status : ENABLE/DISABLE the oscillator.
 *  \return If successful ENABLED, else DISABLED.
 */
hal_bool_t hal_rcc_set_osc(hal_rcc_osc_t osc, hal_bool_t status);

/**
 *  \brief  Configures the different pll settings.
 *  Don't forget to enable the pll after configuration.
 *  \param  pll : Defines the pll to setup.
 *  \param  init : Initialisation structure for pll.
 */
void hal_rcc_setup_pll(hal_rcc_osc_t pll, hal_rcc_pll_init_t init);

/**
 *  \brief  Configure the system clocks.
 *  \param  init : Initialisation structure for the system clock.
 */
void hal_rcc_setup_clock(hal_rcc_clk_init_t init);


#endif
