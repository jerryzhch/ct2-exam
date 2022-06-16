/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_adc.
 *
 *  The hardware abstraction layer for the analog to digital converter.
 * 
 *  \file   hal_adc.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_ADC_H
#define _HAL_ADC_H


/* User includes */
#include "reg_stm32f4xx.h"
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_adc_channel_t
 *  \brief  Defines the channel of a ADC.
 */
typedef enum {
    HAL_ADC_CH0 = 0u,               /**< Channel 0. */
    HAL_ADC_CH1 = 1u,               /**< Channel 1. */
    HAL_ADC_CH2 = 2u,               /**< Channel 2. */
    HAL_ADC_CH3 = 3u,               /**< Channel 3. */
    HAL_ADC_CH4 = 4u,               /**< Channel 4. */
    HAL_ADC_CH5 = 5u,               /**< Channel 5. */
    HAL_ADC_CH6 = 6u,               /**< Channel 6. */
    HAL_ADC_CH7 = 7u,               /**< Channel 7. */
    HAL_ADC_CH8 = 8u,               /**< Channel 8. */
    HAL_ADC_CH9 = 9u,               /**< Channel 9. */
    HAL_ADC_CH10 = 10u,             /**< Channel 10. */
    HAL_ADC_CH11 = 11u,             /**< Channel 11. */
    HAL_ADC_CH12 = 12u,             /**< Channel 12. */
    HAL_ADC_CH13 = 13u,             /**< Channel 13. */
    HAL_ADC_CH14 = 14u,             /**< Channel 14. */
    HAL_ADC_CH15 = 15u,             /**< Channel 15. */
    HAL_ADC_TEMP = 18u,             /**< Temperature sensor (only ADC1). */
    HAL_ADC_VREF = 17u,             /**< Internal ref. voltage (only ADC1). */
    HAL_ADC_VBAT = 18u,             /**< Battery voltage (only ADC1) */
} hal_adc_channel_t;

/**
 *  \enum   hal_adc_resolution_t
 *  \brief  Resolution available for the ADC.
 */
typedef enum {
    HAL_ADC_RES_6B = 0x03,          /**<  6 bit, takes 9 ADC cycles. */
    HAL_ADC_RES_8B = 0x02,          /**<  8 bit, takes 11 ADC cycles. */
    HAL_ADC_RES_10B = 0x01,         /**< 10 bit, takes 13 ADC cycles. */
    HAL_ADC_RES_12B = 0x00          /**< 12 bit, takes 15 ADC cycles. */
} hal_adc_resolution_t;

/**
 *  \enum   hal_adc_polarity_t
 *  \brief  Polarity of external trigger event.
 */
typedef enum {
    HAL_ADC_POLARITY_NONE = 0x0,    /**< Don't trigger. */
    HAL_ADC_POLARITY_RISING = 0x1,  /**< Trigger on rising edge. */
    HAL_ADC_POLARITY_FALLING = 0x2, /**< Trigger on falling edge. */
    HAL_ADC_POLARITY_BOOTH = 0x3    /**< Trigger on both edges. */
} hal_adc_polarity_t;

typedef enum {
    /* TIM1 */
    HAL_ADC_REGTRG_T1CC1 = 0u,      /**< Timer 1, capture compare channel 1. */
    HAL_ADC_REGTRG_T1CC2 = 1u,      /**< Timer 1, capture compare channel 2. */
    HAL_ADC_REGTRG_T1CC3 = 2u,      /**< Timer 1, capture compare channel 3. */
    /* TIM2 */
    HAL_ADC_REGTRG_T2CC2 = 3u,      /**< Timer 2, capture compare channel 2. */
    HAL_ADC_REGTRG_T2CC3 = 4u,      /**< Timer 2, capture compare channel 3. */
    HAL_ADC_REGTRG_T2CC4 = 5u,      /**< Timer 2, capture compare channel 4. */
    HAL_ADC_REGTRG_T2TRGO = 6u,     /**< Timer 2, trigger output. */
    /* TIM3 */
    HAL_ADC_REGTRG_T3CC1 = 7u,      /**< Timer 3, capture compare channel 1. */
    HAL_ADC_REGTRG_T3TRGO = 8u,     /**< Timer 3, trigger output. */
    /* TIM4 */
    HAL_ADC_REGTRG_T4CC4 = 9u,      /**< Timer 4, capture compare channel 4. */
    /* TIM5 */
    HAL_ADC_REGTRG_T5CC1 = 10u,     /**< Timer 5, capture compare channel 1. */
    HAL_ADC_REGTRG_T5CC2 = 11u,     /**< Timer 5, capture compare channel 2. */
    HAL_ADC_REGTRG_T5CC3 = 12u,     /**< Timer 5, capture compare channel 3. */
    /* TIM8 */
    HAL_ADC_REGTRG_T8CC1 = 13u,     /**< Timer 8, capture compare channel 1. */
    HAL_ADC_REGTRG_T8TRGO = 14u,    /**< Timer 8, trigger output. */
    /* others */
    HAL_ADC_REGTRG_EXTI11 = 15u     /**< External interrupt line 11. */
} hal_adc_regular_trg_t;

typedef enum {
    /* TIM1 */
    HAL_ADC_INJTRG_T1CC4 = 0u,      /**< Timer 1, capture compare channel 4. */
    HAL_ADC_INJTRG_T1TRGO = 1u,     /**< Timer 1, trigger output. */
    /* TIM2 */
    HAL_ADC_INJTRG_T2CC1 = 2u,      /**< Timer 2, capture compare channel 1. */
    HAL_ADC_INJTRG_T2TRGO = 3u,     /**< Timer 2, trigger output. */
    /* TIM3 */
    HAL_ADC_INJTRG_T3CC2 = 4u,      /**< Timer 3, capture compare channel 2. */
    HAL_ADC_INJTRG_T3CC4 = 5u,      /**< Timer 3, capture compare channel 4. */
    /* TIM4 */
    HAL_ADC_INJTRG_T4CC1 = 6u,      /**< Timer 4, capture compare channel 1. */
    HAL_ADC_INJTRG_T4CC2 = 7u,      /**< Timer 4, capture compare channel 2. */
    HAL_ADC_INJTRG_T4CC3 = 8u,      /**< Timer 4, capture compare channel 3. */
    HAL_ADC_INJTRG_T4TRGO = 9u,     /**< Timer 4, trigger output. */
    /* TIM5 */
    HAL_ADC_INJTRG_T5CC4 = 10u,     /**< Timer 5, capture compare channel 4. */
    HAL_ADC_INJTRG_T5TRGO = 11u,    /**< Timer 5, trigger output. */
    /* TIM8 */
    HAL_ADC_INJTRG_T8CC2 = 12u,     /**< Timer 8, capture compare channel 2. */
    HAL_ADC_INJTRG_T8CC3 = 13u,     /**< Timer 8, capture compare channel 3. */
    HAL_ADC_INJTRG_T8CC4 = 14u,     /**< Timer 8, capture compare channel 4. */
    /* others */
    HAL_ADC_INJTRG_EXTI15 = 15u     /**< External interrupt line 15. */
} hal_adc_injected_trg_t;

/**
 *  \enum   hal_adc_align_t
 *  \brief  Alignment of converted data in data register.
 */
typedef enum {
    HAL_ADC_ALIGN_RIGHT = 0x0,      /**< Right alignment. */
    HAL_ADC_ALIGN_LEFT = 0x1        /**< Left alignment. */
} hal_adc_align_t;

/**
 *  \enum   hal_adc_sample_time_t
 *  \brief  Sampling time of channel.
 */
typedef enum {
    HAL_ADC_SAMPLING_3C = 0u,       /**<   3 cyles. */
    HAL_ADC_SAMPLING_15C = 1u,      /**<  15 cyles. */
    HAL_ADC_SAMPLING_28C = 2u,      /**<  28 cyles. */
    HAL_ADC_SAMPLING_56C = 3u,      /**<  56 cyles. */
    HAL_ADC_SAMPLING_84C = 4u,      /**<  84 cyles. */
    HAL_ADC_SAMPLING_112C = 5u,     /**< 112 cyles. */
    HAL_ADC_SAMPLING_144C = 6u,     /**< 144 cyles. */
    HAL_ADC_SAMPLING_480C = 7u      /**< 480 cyles. */
} hal_adc_sample_time_t;

/**
 *  \struct hal_adc_init_t
 *  \brief  Initialization structure for analog digital converter.
 */
typedef struct {
    hal_adc_resolution_t resolution;    /**< Choose scan resolution. */
    hal_bool_t scan_mode;               /**< ENABLE/DISABLE scan mode. */
    hal_bool_t continuous_mode;         /**< ENABLE/DISABLE continous mode. */
    hal_adc_polarity_t polarity;        /**< Choose trigger polarity. */
    hal_adc_regular_trg_t trigger;      /**< Choose trigger source. */
    hal_adc_align_t alignment;          /**< Choose data alignment. */
    uint8_t nr_conversions;             /**< Total nr. of channels to conv. */
} hal_adc_init_t;

/**
 *  \struct hal_adc_ch_init_t
 *  \brief  Initialization structure for adc channel.
 */
typedef struct {
    uint8_t rank;                       /**< Rank in sequence (1..16). */
    hal_adc_sample_time_t cycles;       /**< Sample time of channel. */
} hal_adc_ch_init_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets adc to default values.
 *  \param  adc : Defines adc to reset.
 */
void hal_adc_reset(reg_adc_t *adc)
__attribute__((deprecated("Please use ADCx_RESET().")));

/**
 *  \brief  Initializes the specified analog digital converter.
 *  \param  adc : Defines adc to initialize.
 *  \param  init : Structure with adc configuration.
 */
void hal_adc_init_base(reg_adc_t *adc, hal_adc_init_t init);

/**
 *  \brief  Initializes the specified adc channel.
 *  \param  adc : Defines adc.
 *  \param  channel : Defines channel.
 *  \param  init : Structure with channel configuration.
 */
void hal_adc_init_channel(reg_adc_t *adc, 
                          hal_adc_channel_t channel,
                          hal_adc_ch_init_t init);

/**
 *  \brief  Start specified ADC.
 *  \param  adc : Defines ADC to start.
 */
void hal_adc_start(reg_adc_t *adc);

/**
 *  \brief  Stops specified ADC.
 *  \param  adc : Defines ADC to stop.
 */
void hal_adc_stop(reg_adc_t *adc);

/**
 *  \brief  Enables DMA on specified ADC.
 *  \param  adc : Defines adc to interact with.
 *  \param  status : ENABLE/DISABLE DMA on specified ADC.
 */
void hal_adc_set_dma(reg_adc_t *adc,
                     hal_bool_t status);


#endif
