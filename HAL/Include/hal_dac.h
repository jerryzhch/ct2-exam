/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_dac.
 *
 *  The hardware abstraction layer for the digital analog converter.
 * 
 *  \file   hal_dac.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_DAC_H
#define _HAL_DAC_H


/* User includes */
#include "reg_stm32f4xx.h"
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_dac_channel_t
 *  \brief  Defines the channel of a DAC.
 */
typedef enum {
    HAL_DAC_CH1 =  0u,              /**< Channel 1. */
    HAL_DAC_CH2 = 16u,              /**< Channel 2. */
} hal_dac_channel_t;


/**
 *  \enum   hal_dac_trg_t
 *  \brief  Defines the available trigger inputs.
 */
typedef enum {
    HAL_DAC_TRG_TIM6 = 0u,          /**< Timer 6 TRGO. */
    HAL_DAC_TRG_TIM8 = 1u,          /**< Timer 8 TRGO. */
    HAL_DAC_TRG_TIM7 = 2u,          /**< Timer 7 TRGO. */
    HAL_DAC_TRG_TIM5 = 3u,          /**< Timer 5 TRGO. */
    HAL_DAC_TRG_TIM2 = 4u,          /**< Timer 2 TRGO. */
    HAL_DAC_TRG_TIM4 = 5u,          /**< Timer 4 TRGO. */
    HAL_DAC_TRG_EXTI9 = 6u,         /**< EXTI line 9. */
    HAL_DAC_TRG_SW = 7u             /**< Software trigger. */
} hal_dac_trg_t;

/**
 *  \struct hal_dac_init_t
 *  \brief  Initialization structure for digital analog converter.
 */
typedef struct {
    hal_bool_t trigger_enable;      /**< ENABLE/DISABLE trigger. */
    hal_dac_trg_t trigger;          /**< Select trigger, if enabled. */
    hal_bool_t buffer_enable;       /**< ENABLE/DISABLE output buffer. */
} hal_dac_init_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets DAC to default values.
 *  \param  dac : Defines DAC to reset.
 */
void hal_dac_reset(reg_dac_t *dac)
__attribute__((deprecated("Please use DACx_RESET().")));

/**
 *  \brief  Initializes the specified digital analog converter.
 *  \param  dac : Defines DAC to interact with.
 *  \param  channel : Defines DAC channel to initialize.
 *  \param  init : Structure with DAC configuration.
 */
void hal_dac_init(reg_dac_t *dac, 
                  hal_dac_channel_t channel,
                  hal_dac_init_t init);

/**
 *  \brief  Enables DMA on specified DAC.
 *  \param  dac : Defines DAC to interact with.
 *  \param  channel : Defines DAC channel to interact with.
 *  \param  status : ENABLE/DISABLE DMA on specified DAC.
 */
void hal_dac_set_dma(reg_dac_t *dac,
                     hal_dac_channel_t channel,
                     hal_bool_t status);
 
#endif
