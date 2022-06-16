/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_fmc.
 *
 *  The hardware abstraction layer for the memory controller.
 *
 *  \file   hal_fmc.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_FMC_H
#define _HAL_FMC_H


/* User includes */
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_fmc_bank_t
 *  \brief  Defines the different memory banks.
 */
typedef enum {
    HAL_FMC_SRAM_BANK1 = 0u,    /**< SRAM bank 1 of 4. */
    HAL_FMC_SRAM_BANK2 = 1u,    /**< SRAM bank 2 of 4. */
    HAL_FMC_SRAM_BANK3 = 2u,    /**< SRAM bank 3 of 4. */
    HAL_FMC_SRAM_BANK4 = 3u     /**< SRAM bank 4 of 4. */
} hal_fmc_bank_t;


/**
 *  \enum   hal_fmc_memory_type_t
 *  \brief  Defines the different memory types.
 */
typedef enum {
    HAL_FMC_TYPE_SRAM = 0u,     /**< Asynchronous SRAM. */
    HAL_FMC_TYPE_PSRAM = 1u     /**< Synchronous SRAM. */
} hal_fmc_memory_type_t;


/**
 *  \enum   hal_fmc_memory_width_t
 *  \brief  Defines the available data bus widths.
 */
typedef enum {
    HAL_FMC_WIDTH_8B = 0u,      /**< Data bus width: 8 bit. */
    HAL_FMC_WIDTH_16B = 1u,     /**< Data bus width: 16 bit. */
    HAL_FMC_WIDTH_32B = 2u      /**< Data bus width: 32 bit. */
} hal_fmc_memory_width_t;


/**
 *  \enum   hal_fmc_access_mode_t
 *  \brief  Defines the different available access modes.
 */
typedef enum {
    HAL_FMC_ACCESS_MODE_A = 0u,     /**< Access mode A. */
    HAL_FMC_ACCESS_MODE_B = 1u,     /**< Access mode B. */
    HAL_FMC_ACCESS_MODE_C = 2u,     /**< Access mode C. */
    HAL_FMC_ACCESS_MODE_D = 3u,     /**< Access mode D. */
} hal_fmc_access_mode_t;


/**
 *  \struct hal_fmc_sram_init_t
 *  \brief  Initialization structure for sram bank.
 */
typedef struct {
    hal_bool_t address_mux;         /**< Mux address and data bus. */
    hal_fmc_memory_type_t type;     /**< Choose memory type to configure. */
    hal_fmc_memory_width_t width;   /**< Choose width of the data bus. */
    hal_bool_t read_burst;          /**< ENABLE/DISABLE read burst mode. */
    hal_bool_t write_enable;        /**< ENABLE/DISABLE write access. */
    hal_bool_t write_burst;         /**< ENABLE/DISABLE write burst mode. */
    hal_bool_t continous_clock;     /**< Should the clock always be active. */
} hal_fmc_sram_init_t;


/**
 *  \struct hal_fmc_sram_timing_t
 *  \brief  Initialization structure for timing information of sram bank.
 */
typedef struct {
    uint8_t bus_turnaround;         /**< Bus turnaround time in HCLK cycles. */
    /* Only for async. SRAM */
    uint8_t address_setup;          /**< Address setup time in HCLK cycles. */
    uint8_t address_hold;           /**< Address hold time in HCLK cycles. */
    uint8_t data_setup;             /**< Data setup time in HCLK cycles. */
    /* Only for sync. PSRAM */
    uint8_t clk_divider;            /**< Divider of HCLK for FMC_CLK. */
    uint8_t data_latency;           /**< Data latency in FMC_CLK cycles. */
    hal_fmc_access_mode_t mode;     /**< Choose access mode. */
} hal_fmc_sram_timing_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/** 
 *  \brief  Resets fmc bank to default values.
 *  \param  bank : Defines bank to reset.
 */
void hal_fmc_reset(hal_fmc_bank_t bank)
__attribute__((deprecated("Please use FMC_RESET().")));

/**
 *  \brief  Initializes a fmc bank as SRAM.
 *  \param  bank : Defines bank to initialize.
 *  \param  init : Structure with sram configuration.
 *  \param  timing : Structure with sram timing information.
 */
void hal_fmc_init_sram(hal_fmc_bank_t bank, 
                       hal_fmc_sram_init_t init, 
                       hal_fmc_sram_timing_t timing);


#endif
