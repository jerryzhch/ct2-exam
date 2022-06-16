/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_timer.
 *
 *  Example of programming a basic timer:
 *  - hal_timer_reset(TIM1);
 *  - enable timer peripheral clock -> TIM1_ENABLE();
 *  - fill basic init struct -> hal_timer_base_init_t timer_base = ...;
 *  - hal_timer_init_base(TIM1, timer_base);
 *  - OPTIONAL INTERRUPT:
 *      - hal_timer_irq_set(TIM1, HAL_TIMER_IRQ_CC1, ENABLE);
 *  - OPTIONAL DMA:
 *      - enable peripheral clock ->  DMA2_ENABLE();
 *      - hal_timer_dma_set(TIM1, HAL_TIMER_DMA_CC1, ENABLE);
 *      - config dma channel see hal_dma.h
 *  - hal_timer_start(TIM1);
 *
 *      NOTE: For all Additional functions the basic timer above has to
 *            be config first.
 *
 *  Additional pwm channel:
 *  - enable gpio peripheral clock -> GPIOx_ENABLE();
 *  - fill gpio output struct to define outputpin
 *          -> hal_gpio_output_t gpio = ...;
 *  - init gpio as alternative functions to specific timer
 *          -> hal_gpio_init_alternate();
 *  - fill init struct -> hal_timer_output_init_t pwm_init = ...;
 *  - hal_timer_init_output(TIM1, HAL_TIMER_CHx, pwm_init);
 *
 *  Additional output compare (oc) mode:
 *  - enable gpio peripheral clock -> GPIOx_ENABLE();
 *  - fill gpio output struct to define outputpin
 *          -> hal_gpio_output_t gpio = ...;
 *  - init gpio as alternative functions to specific timer
 *          -> hal_gpio_init_alternate();
 *  - fill output compare init struct
 *          -> hal_timer_output_init_t timer_output = ...;
 *  - hal_timer_init_output(TIM1, HAL_TIMER_CHx, hal_timer_output_init_t);
 *
 *  Additional Input capture (ic) mode:
 *      #   Only for external capture trigger
 *      - enable gpio peripheral clock -> GPIOx_ENABLE();
 *      - fill gpio output struct to defnine inputpin -> hal_gpio_output_t;
 *      - init gpio as alternative functions to specific timer
 *          -> hal_gpio_init_alternate();
 *      ##  For all Input capture trigger sources
 *      - fill input capture init struct 
 *          -> hal_timer_input_init_t timer_input = ...;
 *      - hal_timer_init_input(TIM1, HAL_TIMER_CHx, timer_input);
 *      - fill timer clock source init stuct
 *          -> hal_timer_clock_init_t timer_clock = ...;
 *      - hal_timer_init_clock(TIM1, timer_clock); 
 *      - Activate interupt if desired -> hal_timer_irq_set();
 *      - Activate dma if desired -> hal_timer_dma_set();
 *      ### Start /stop counter
 *      - start/stop timer           -> hal_timer_start(TIM1);
 *                                   -> hal_timer_stop(TIM1);
 *
 *  \file   hal_timer.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_TIMER_H
#define _HAL_TIMER_H


/* User includes */
#include "reg_stm32f4xx.h"
#include "hal_common.h"


/* -- Type definitions 
 * ------------------------------------------------------------------------- */

/* **** Basic **** */

/**
 *  \enum   hal_timer_channel_t
 *  \brief  Defines the capture / compare channel of a timer.
 */
typedef enum {
    HAL_TIMER_CH1 = 0u,             /**< Capture/compare channel 1. */
    HAL_TIMER_CH2 = 1u,             /**< Capture/compare channel 2. */
    HAL_TIMER_CH3 = 2u,             /**< Capture/compare channel 3. */
    HAL_TIMER_CH4 = 3u,             /**< Capture/compare channel 4. */
    /* Only as output in adv. timer 1/8 */
    HAL_TIMER_CH1N = 16u,           /**< Capture/compare channel 1N. */
    HAL_TIMER_CH2N = 17u,           /**< Capture/compare channel 2N. */
    HAL_TIMER_CH3N = 18u,           /**< Capture/compare channel 3N. */
} hal_timer_channel_t;


/**
 *  \enum   hal_timer_mode_t
 *  \brief  Defines counting mode of timer.
 */
typedef enum {
    HAL_TIMER_MODE_UP = 0x0,        /**< CR1: CMS[6..5] = 00, DIR[4] = 0. */
    HAL_TIMER_MODE_DOWN = 0x1,      /**< CR1: CMS[6..5] = 00, DIR[4] = 1. */
    HAL_TIMER_MODE_CENTER1 = 0x2,   /**< CR1: CMS[6..5] = 01. */
    HAL_TIMER_MODE_CENTER2 = 0x4,   /**< CR1: CMS[6..5] = 10. */
    HAL_TIMER_MODE_CENTER3 = 0x6    /**< CR1: CMS[6..5] = 11. */
} hal_timer_mode_t;


/**
 *  \enum   hal_timer_run_t
 *  \brief  Defines run mode of timer.
 */
typedef enum {
    HAL_TIMER_RUN_CONTINOUS = 0x0,  /**< Restart timer after update event. */
    HAL_TIMER_RUN_ONCE = 0x1        /**< Stop timer after update event. */
} hal_timer_run_t;


/**
 *  \enum   hal_timer_irq_t
 *  \brief  Defines the different interrupts used by the timer.
 */
typedef enum {
    HAL_TIMER_IRQ_UE = 0x01,            /**< Timer update event. */
    HAL_TIMER_IRQ_CC1 = 0x02,           /**< Capture/compare channel 1. */
    HAL_TIMER_IRQ_CC2 = 0x04,           /**< Capture/compare channel 2. */
    HAL_TIMER_IRQ_CC3 = 0x08,           /**< Capture/compare channel 3. */
    HAL_TIMER_IRQ_CC4 = 0x10,           /**< Capture/compare channel 4. */
    HAL_TIMER_IRQ_TRIGGER = 0x40        /**< Trigger event. */  
} hal_timer_irq_t;


/**
 *  \enum   hal_timer_dma_source_t
 *  \brief  Defines the different dma used by the timer.
 */
typedef enum {
    HAL_TIMER_DMA_UDE = 0x0100,         /**< Update request. */
    HAL_TIMER_DMA_CC1 = 0x0200,         /**< Capture/compare channel 1. */
    HAL_TIMER_DMA_CC2 = 0x0400,         /**< Capture/compare channel 2. */
    HAL_TIMER_DMA_CC3 = 0x0800,         /**< Capture/compare channel 3. */
    HAL_TIMER_DMA_CC4 = 0x1000,         /**< Capture/compare channel 4. */
    HAL_TIMER_DMA_TRG = 0x4000          /**< Trigger request. */
} hal_timer_dmasrc_t;


/**
 *  \enum   hal_timer_slave_t
 *  \brief  Defines the slave mode for the clock source.
 */
typedef enum {
    HAL_TIMER_MASTER_RESET = 0u,        /**< UG bit (in EGR) triggers TRGO. */
    HAL_TIMER_MASTER_ENABLE = 1u,       /**< EN bit (in CR1) triggers TRGO. */
    HAL_TIMER_MASTER_UPDATE = 2u,       /**< Update event triggers TRGO. */
    HAL_TIMER_MASTER_COMPARE = 3u,      /**< Pulse on TRGO when CC1F is set. */
    HAL_TIMER_MASTER_COMPOC1 = 4u,      /**< OC1REF used as TRGO. */
    HAL_TIMER_MASTER_COMPOC2 = 5u,      /**< OC2REF used as TRGO. */
    HAL_TIMER_MASTER_COMPOC3 = 6u,      /**< OC3REF used as TRGO. */
    HAL_TIMER_MASTER_COMPOC4 = 7u       /**< OC4REF used as TRGO. */
} hal_timer_mastermode_t;


/* **** Filter **** */

/**
 *  \enum   hal_timer_polarity_t
 *  \brief  Defines polarity.
 */
typedef enum {
    HAL_TIMER_POLARITY_HIGH = 0x0,      /**< Positive polarity, active-high. */
    HAL_TIMER_POLARITY_LOW = 0x1,       /**< Negative polarity, active-low. */
    /* Only for input */
    HAL_TIMER_POLARITY_BOTH = 0x5       /**< Trigger on both edges. */
} hal_timer_polarity_t;


/**
 *  \enum   hal_timer_psc_t
 *  \brief  Defines the filter prescaler.
 */
typedef enum {
    HAL_TIMER_PSC_OFF = 0x0,            /**< Prescaler off. */
    HAL_TIMER_PSC_DIV2 = 0x1,           /**< Divided by 2. */
    HAL_TIMER_PSC_DIV4 = 0x2,           /**< Divided by 4. */
    HAL_TIMER_PSC_DIV8 = 0x3            /**< Divided by 8. */
} hal_timer_psc_t;


/**
 *  \enum   hal_timer_ic_filter_t
 *  \brief  Defines the filter/sampling, ref. manual p623.
 */
typedef enum {
    HAL_TIMER_FILTER_OFF = 0u,
    HAL_TIMER_FILTER_INT_N2 = 1u,
    HAL_TIMER_FILTER_INT_N4 = 2u,
    HAL_TIMER_FILTER_INT_N8 = 3u,
    HAL_TIMER_FILTER_DTS2_N6 = 4u,
    HAL_TIMER_FILTER_DTS2_N8 = 5u,
    HAL_TIMER_FILTER_DTS4_N6 = 6u,
    HAL_TIMER_FILTER_DTS4_N8 = 7u,
    HAL_TIMER_FILTER_DTS8_N6 = 8u,
    HAL_TIMER_FILTER_DTS8_N8 = 9u,
    HAL_TIMER_FILTER_DTS16_N5 = 10u,
    HAL_TIMER_FILTER_DTS16_N6 = 11u,
    HAL_TIMER_FILTER_DTS16_N8 = 12u,
    HAL_TIMER_FILTER_DTS32_N5 = 13u,
    HAL_TIMER_FILTER_DTS32_N6 = 14u,
    HAL_TIMER_FILTER_DTS32_N8 = 15u
} hal_timer_filter_t;


/* **** Output Compare **** */

/**
 *  \enum   hal_timer_oc_mode_t
 *  \brief  Defines mode of the output capture unit.
 */
typedef enum {
    HAL_TIMER_OCMODE_NONE = 0x0,        /**< Not configured. */
    HAL_TIMER_OCMODE_ACTIVE   = 0x1,    /**< Active if CNT = CCRx. */
    HAL_TIMER_OCMODE_INACTIVE = 0x2,    /**< Inctive if CNT = CCRx. */
    HAL_TIMER_OCMODE_TOGGLE   = 0x3,    /**< Toggle if CNT = CCRx. */
    HAL_TIMER_OCMODE_PWM1 = 0x6,        /**< PWM 1, active if CNT < CCRx. */
    HAL_TIMER_OCMODE_PWM2 = 0x7         /**< PWM 2, active if CNT > CCRx. */
} hal_timer_oc_mode_t;


/* **** Input Capture **** */

/**
 *  \enum   hal_timer_ic_sel_t
 *  \brief  Defines the route for trigger input
 */
typedef enum {
    HAL_TIMER_IC_SEL_DIRECT = 0x1,      /**< Straight: 1->IC1 */
    HAL_TIMER_IC_SEL_INDIRECT = 0x2,    /**< Crossed: 1->IC2 */
    HAL_TIMER_IC_SEL_TRC = 0x3          /**< All pins to TRC. */
} hal_timer_ic_sel_t;


/* **** Clock Source **** */

/**
 *  \enum   hal_timer_source_t
 *  \brief  Defines the source for the counter source.
 */
typedef enum {
    HAL_TIMER_CLKSRC_INT,               /**< Internal clock source (TCLK). */
    HAL_TIMER_CLKSRC_ETRF,              /**< External clock source (ETRF). */
    HAL_TIMER_CLKSRC_TRGI               /**< Trigger clock source (TRGI). */
} hal_timer_source_t;


/**
 *  \enum   hal_timer_clk_dts_t
 *  \brief  Divider used for generating sampling clock for digital filters.
 */
typedef enum {
    HAL_TIMER_CLK_OFF = 0x00,           /**< No divider. */
    HAL_TIMER_CLK_DIV2 = 0x01,          /**< CK_INT / 2. */
    HAL_TIMER_CLK_DIV4 = 0x02           /**< CK_INT / 4. */
} hal_timer_clk_dts_t;


/**
 *  \enum   hal_timer_trgsrc_t
 *  \brief  Defines the source for TRG.
 */
typedef enum {
    HAL_TIMER_TRGSRC_ITR0 = 0u,         /**< Internal trigger 0. */
    HAL_TIMER_TRGSRC_ITR1 = 1u,         /**< Internal trigger 1. */
    HAL_TIMER_TRGSRC_ITR2 = 2u,         /**< Internal trigger 2. */
    HAL_TIMER_TRGSRC_ITR3 = 3u,         /**< Internal trigger 3. */
    HAL_TIMER_TRGSRC_TI1F_ED = 4u,      /**< TI1 edge detector. */
    HAL_TIMER_TRGSRC_TI1FP = 5u,        /**< Filtered timer input 1. */
    HAL_TIMER_TRGSRC_TI2FP = 6u,        /**< Filtered timer input 2. */
    HAL_TIMER_TRGSRC_ETRF = 7u          /**< External trigger input. */
} hal_timer_trgsrc_t;


/**
 *  \enum   hal_timer_slave_t
 *  \brief  Defines the slave mode for the clock source.
 */
typedef enum {
    HAL_TIMER_SLAVE_OFF = 0u,           /**< Slave mode disabled. */
    HAL_TIMER_SLAVE_ENC1 = 1u,          /**< Counts on edge of TI2FP1. */
    HAL_TIMER_SLAVE_ENC2 = 2u,          /**< Counts on edge of TI1FP2. */
    HAL_TIMER_SLAVE_ENC3 = 3u,          /**< Counts on edge of TIxFPx. */
    HAL_TIMER_SLAVE_RESET = 4u,         /**< Clock reset if TRGI high. */
    HAL_TIMER_SLAVE_GATED = 5u,         /**< Clock enabled, if TRGI high. */
    HAL_TIMER_SLAVE_TRIGGER = 6u,       /**< Start clock if TRGI high. */
    HAL_TIMER_SLAVE_EXT = 7u            /**< Clock enabled, if TRGI high. */
} hal_timer_slavemode_t;


/* **** Init Structs **** */

/**
 *  \struct hal_timer_base_init_t
 *  \brief  Initialization structure for basic timer functionality.
 */
typedef struct {
    uint16_t prescaler;                 /**< Choose Prescaler, HCLK / x. */
    hal_timer_mode_t mode;              /**< Choose timer mode. */
    hal_timer_run_t run_mode;           /**< Choose run mode. */
    uint32_t count;                     /**< Set timer value. */
    hal_timer_mastermode_t master_mode; /**< Select master mode. */
} hal_timer_base_init_t;


/**
 *  \struct hal_timer_filter_init_t
 *  \brief  Initialization structure for input filter.
 */
typedef struct {
    hal_timer_filter_t mode;            /**< Define the filter mode. */
    hal_timer_polarity_t polarity;      /**< Select filter polarity. */
    hal_timer_psc_t prescaler;          /**< Select filter prescaler. */
} hal_timer_filter_init_t;


/**
 *  \struct hal_timer_clock_init_t
 *  \brief  Initialization structure for basic timer functionality.
 */
typedef struct {
    hal_timer_source_t source;          /**< Select clock source. */
    hal_timer_slavemode_t slave_mode;   /**< Select slave mode. */
    /* Only if ETRF is used as source */
    hal_timer_filter_init_t filter;     /**< Filter config. */
    hal_timer_clk_dts_t filter_div;     /**< Filter clock divider. */
    /* Only if TRGI is used as source */
    hal_timer_trgsrc_t trg_source;      /**< Select source for TRG */
} hal_timer_clock_init_t;


/**
 *  \struct hal_timer_ic_init_t
 *  \brief  Initialization structure for input capture functionality.
 */
typedef struct {
    hal_timer_ic_sel_t input;           /**< Select trigger route. */
    hal_timer_filter_init_t filter;     /**< Filter config. */
    hal_bool_t input_state;             /**< ENABLE/DISABLE input channel. */
} hal_timer_input_init_t;


/**
 *  \struct hal_timer_output_init_t
 *  \brief  Initialization structure for output functionality.
 */
typedef struct {
    hal_timer_oc_mode_t mode;           /**< Select output mode. */
    hal_timer_polarity_t polarity;      /**< Select output polarity. */
    hal_bool_t output_state;            /**< ENABLE/DISABLE output channel. */
    /* Only for PWM */
    uint32_t pulse;                     /**< Set pulsewidth of pwm. */
} hal_timer_output_init_t;


/* -- Public function declarations 
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets the specified timer to its default values.
 *  \param  timer : Defines timer to reset.
 */
void hal_timer_reset(reg_tim_t *timer)
__attribute__((deprecated("Please use ATIMx_RESET().")));

/**
 *  \brief  Initializes the basic timer according to the specified parameters.
 *  \param  timer : Defines timer to interact with.
 *  \param  init : Basic initialisation structure for timer.
 */
void hal_timer_init_base(reg_tim_t *timer,
                         hal_timer_base_init_t init);

/**
 *  \brief  Initializes the clock source to be used by timer.
 *  \param  timer : Defines timer to interact with.
 *  \param  init : Initialisation structure for clock source.
 */
void hal_timer_init_clock(reg_tim_t *timer, hal_timer_clock_init_t init);

/**
 *  \brief  Configures the specified channel and timer input capture mode. 
 *  \param  timer :  Defines timer to interact with.
 *  \param  channel : Defines capture / compare channel to initialize.
 *  \param  init : Initialisation structure for ic channel.
 */
void hal_timer_init_input(reg_tim_t *timer,
                          hal_timer_channel_t channel, 
                          hal_timer_input_init_t init);

/**
 *  \brief  Configures the specified channel and timer output compare mode. 
 *  \param  timer : Defines timer to interact with.
 *  \param  channel : Defines capture / compare channel to initialize.
 *  \param  init : Initialisation structure for oc channel.
 */
void hal_timer_init_output(reg_tim_t *timer,
                           hal_timer_channel_t channel,
                           hal_timer_output_init_t init);

/**
 *  \brief  Start specified timer.
 *  \param  timer : Defines timer to start.
 */
void hal_timer_start(reg_tim_t *timer);

/**
 *  \brief  Stops specified timer.
 *  \param  timer : Defines timer to stop.
 */
void hal_timer_stop(reg_tim_t *timer);

/**
 *  \brief  Enables channel on specified timer.
 *  \param  timer : Defines timer to interact with.
 *  \param  channel : Specifies channel.
 *  \param  status : ENABLE/DISABLE specified channel.
 */
void hal_timer_channel_set(reg_tim_t *timer,
                           hal_timer_channel_t channel,
                           hal_bool_t status);

/**
 *  \brief  Enables interrupt on specified timer.
 *  \param  timer : Defines timer to interact with.
 *  \param  irq : Specifies interrupt.
 *  \param  status : ENABLE/DISABLE specified interrupt.
 */
void hal_timer_irq_set(reg_tim_t *timer,
                       hal_timer_irq_t irq,
                       hal_bool_t status);

/**
 *  \brief  Return status of specified timers interrupt.
 *  \param  timer : Defines timer to interact with.
 *  \param  irq : Interrupt to read.
 *  \return Actual status of specified interrupt.
 */
hal_bool_t hal_timer_irq_status(reg_tim_t *timer, hal_timer_irq_t irq);

/**
 *  \brief  Clear specified timers interrupt.
 *  \param  timer : Defines timer to interact with.
 *  \param  irq : Interrupt to clear.
 */
void hal_timer_irq_clear(reg_tim_t *timer, hal_timer_irq_t irq);

/**
 *  \brief  Enable DMA for specific timer
 *  \param  timer : Defines timer to interact with.
 *  \param  dma : Trigger source for DMA.
 *  \param  status : ENABLE/DISABLE specified DMA source.
 */
void hal_timer_dma_set(reg_tim_t *timer, 
                       hal_timer_dmasrc_t dma,
                       hal_bool_t status);

/**
 *  \brief  Return value of specified timers counter register.
 *  \param  timer : Defines timer to interact with.
 *  \return Actual value of the CNT register.
 */
uint32_t hal_timer_counter_read(reg_tim_t *timer);

/**
 *  \brief  Set reload value of specified timer.
 *  \param  timer : Defines timer to interact with.
 *  \param  value : New reload value for the timer.
 */
void hal_timer_reload_write(reg_tim_t *timer, uint32_t value);

/**
 *  \brief  Set prescaler value of specified timer.
 *  \param  timer : Defines timer to interact with.
 *  \param  value : New prescaler value for the timer.
 */
void hal_timer_prescaler_write(reg_tim_t *timer, uint32_t value);

/**
 *  \brief  Return compare register value of specified timer and channel.
 *  \param  timer : Defines timer to interact with.
 *  \param  channel : Defines capture / compare channel to read.
 *  \return Actual value of the capture / compare channel.
 */
uint32_t hal_timer_compare_read(reg_tim_t *timer,
                                hal_timer_channel_t channel);

/**
 *  \brief  Set value of compare register of specified timer and channel.
 *  \param  timer : Defines timer to interact with.
 *  \param  channel : Defines capture / compare channel to write.
 *  \param  value : New value of the capture / compare channel.
 *  \return (-1) if array index out of bounds, else 0 
 */
int8_t hal_timer_compare_write(reg_tim_t *timer,
                                hal_timer_channel_t channel,
                                uint32_t value);
#endif
