/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_dma.
 *
 *  The hardware abstraction layer for direct memory access.
 *  
 *  \file   hal_dma.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_DMA_H
#define _HAL_DMA_H


/* User includes */
#include "reg_stm32f4xx.h"
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_dma_stream_t
 *  \brief  Defines the available streams of the dma, refer to manual p.304ff.
 */
typedef enum {
    HAL_DMA_STREAM_0 = 0u,
    HAL_DMA_STREAM_1 = 1u,
    HAL_DMA_STREAM_2 = 2u,
    HAL_DMA_STREAM_3 = 3u,
    HAL_DMA_STREAM_4 = 4u,
    HAL_DMA_STREAM_5 = 5u,
    HAL_DMA_STREAM_6 = 6u,
    HAL_DMA_STREAM_7 = 7u
} hal_dma_stream_t;


/**
 *  \enum   hal_dma_channel_t
 *  \brief  Defines the available channels of each stream,
 *          refer to manual p.304ff.
 */
typedef enum {
    HAL_DMA_CHANNEL_0 = 0u,
    HAL_DMA_CHANNEL_1 = 1u,
    HAL_DMA_CHANNEL_2 = 2u,
    HAL_DMA_CHANNEL_3 = 3u,
    HAL_DMA_CHANNEL_4 = 4u,
    HAL_DMA_CHANNEL_5 = 5u,
    HAL_DMA_CHANNEL_6 = 6u,
    HAL_DMA_CHANNEL_7 = 7u
} hal_dma_channel_t;


/**
 *  \enum   hal_dma_dir_t
 *  \brief  Defines the possible directions of the transfer,
 *          refer to manual p.306ff.
 */
typedef enum {
    HAL_DMA_PER_TO_MEM = 0x0,
    HAL_DMA_MEM_TO_PER = 0x1,
    HAL_DMA_MEM_TO_MEM = 0x2
} hal_dma_dir_t;


/**
 *  \enum   hal_dma_size_t
 *  \brief  Specifies the size of the register / data.
 */
typedef enum {
    HAL_DMA_SIZE_8B = 0x0,
    HAL_DMA_SIZE_16B = 0x1,
    HAL_DMA_SIZE_32B = 0x2
} hal_dma_size_t;


/**
 *  \struct hal_dma_init_t
 *  \brief  Initialization structure for direct memory access.
 */
typedef struct {
    hal_dma_channel_t channel;  /**< Choose channel, refer to man. p.304ff. */
    hal_dma_dir_t direction;    /**< Specify direction of transfer,
                                     refer to manual p.306ff. */
    uint32_t source;            /**< Address of source (registry),
                                     refer to manual p.306ff. */
    uint32_t destination;       /**< Address of destination (memory),
                                     refer to manual p.306ff. */
    hal_dma_size_t size;        /**< Size of data / register to transfer. */
    uint8_t nr_transactions;    /**< How many transaction, min. 1. */
    hal_bool_t continous;       /**< See circular mode in maual p.310ff. */
} hal_dma_init_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets the specified dma stream to its default values.
 *  \param  dma : Defines dma controller to interact with.
 *  \param  stream : Defines stream to reset.
 */
void hal_dma_reset(reg_dma_t *dma, hal_dma_stream_t stream)
__attribute__((deprecated("Please use DMAx_RESET().")));

/**
 *  \brief  Initializes a basic dma transfer.
 *  \param  dma : Defines the dma controller to initialize.
 *  \param  stream : Defines the dma stream to initialize.
 *  \param  init : Initialisation structure for dma transfer.
 */
void hal_dma_init_base(reg_dma_t *dma, 
                       hal_dma_stream_t stream, 
                       hal_dma_init_t init);

/**
 *  \brief  Starts the dma transfer.
 *  \param  dma : Defines dma controller to interact with.
 *  \param  stream : Defines stream to start.
 */
void hal_dma_start(reg_dma_t *dma, hal_dma_stream_t stream);

/**
 *  \brief  Starts the dma transfer.
 *  \param  dma : Defines dma controller to interact with.
 *  \param  stream : Defines stream to stop.
 */
void hal_dma_stop(reg_dma_t *dma, hal_dma_stream_t stream);


#endif
