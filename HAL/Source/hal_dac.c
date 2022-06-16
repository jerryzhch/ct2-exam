/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Implementation of module hal_dac.
 *
 *  The hardware abstraction layer for the digital analog converter.
 *
 *  $Id$
 * ------------------------------------------------------------------------- */

/* User includes */
#include "hal_dac.h"


/* -- Public function definitions
 * ------------------------------------------------------------------------- */

/*
 * See header file
 */
void hal_dac_reset(reg_dac_t *dac)
{
    dac->CR = 0u;
    dac->SWTRIGR = 0u;
}


/*
 * See header file
 */
void hal_dac_init(reg_dac_t *dac, 
                  hal_dac_channel_t channel,
                  hal_dac_init_t init)
{
    /* Clear old configuration */
    dac->CR &= ~(0xffff << channel);
    
    /* Buffer */
    if (init.buffer_enable) {
        dac->CR |= ((0x1 << 1u) << channel);
    }
    
    /* Trigger */
    if (init.trigger_enable) {
        dac->CR |= ((0x1 << 2u) << channel);    // Enable trigger
        dac->CR |= ((init.trigger << 3u) << channel);
    }
    
    /* Enable DAC */
    dac->CR |= (0x1 << channel);
}


/*
 * See header file
 */
void hal_dac_set_dma(reg_dac_t *dac,
                     hal_dac_channel_t channel,
                     hal_bool_t status)
{
    if (status == ENABLE) {
        dac->CR |= ((0x1 << 12u) << channel);
    } else {
        dac->CR &= ~((0x1 << 12u) << channel);
    }
}

