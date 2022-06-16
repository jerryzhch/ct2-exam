/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_spi.
 *
 *  The hardware abstraction layer for the serial peripheral interface.
 * 
 *  \file   hal_spi.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_SPI_H
#define _HAL_SPI_H

/* Removing weak tag for gcc usage*/
#ifdef TESTING
    #define __weak
#endif


/* User includes */
#include "reg_stm32f4xx.h"
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_spi_mode_t
 *  \brief  Controller mode.
 */
 typedef enum {
     HAL_SPI_MODE_SLAVE = 0x0,      /** < Slave mode. */
     HAL_SPI_MODE_MASTER = 0x1      /** < Master mode. */
 } hal_spi_mode_t;

 
/**
 *  \enum   hal_spi_dir_t
 *  \brief  Transfer direction.
 */
 typedef enum {
     HAL_SPI_DIR_BOTH = 0x0,        /** < Read and write. */
     HAL_SPI_DIR_READ = 0x2,        /** < Read only. */
     HAL_SPI_DIR_WRITE = 0x3        /** < Write only. */
 } hal_spi_dir_t;

 
/**
 *  \enum   hal_spi_size_t
 *  \brief  Size of the data frame.
 */
 typedef enum {
     HAL_SPI_SIZE_8B = 0u,          /** < Data size 8 bit. */
     HAL_SPI_SIZE_16B = 1u          /** < Data size 16 bit. */
 } hal_spi_size_t;
 
 
/**
 *  \enum   hal_spi_nss_t
 *  \brief  Control of slave select pin.
 */
 typedef enum {
     HAL_SPI_NSS_HW = 0x0,          /** < Pin controled by hardware. */
     HAL_SPI_NSS_SW = 0x3,          /** < Pin controled by software. */
 } hal_spi_nss_t;

 
/**
 *  \enum   hal_spi_prescaler_t
 *  \brief  Baude rate control.
 */
 typedef enum {
     HAL_SPI_PRE_2 = 0u,            /** < APB clock (42 MHz) / 2. */
     HAL_SPI_PRE_4 = 1u,            /** < APB clock (42 MHz) / 4. */
     HAL_SPI_PRE_8 = 2u,            /** < APB clock (42 MHz) / 8. */
     HAL_SPI_PRE_16 = 3u,           /** < APB clock (42 MHz) / 16. */
     HAL_SPI_PRE_32 = 4u,           /** < APB clock (42 MHz) / 32. */
     HAL_SPI_PRE_64 = 5u,           /** < APB clock (42 MHz) / 64. */
     HAL_SPI_PRE_128 = 6u,          /** < APB clock (42 MHz) / 128. */
     HAL_SPI_PRE_256 = 7u           /** < APB clock (42 MHz) / 256. */
 } hal_spi_prescaler_t;

 
/**
 *  \enum   hal_spi_frame_t
 *  \brief  Defines SPI frame format.
 */
 typedef enum {
     HAL_SPI_FRAME_MOTOROLLA = 0x0,     /** < Motorolla type format. */
     HAL_SPI_FRAME_TI = 0x1             /** < Texas Instruments type format. */
 } hal_spi_frame_t;

 
/**
 *  \struct hal_spi_init_t
 *  \brief  Initialization structure for the serial peripheral interface.
 */
typedef struct {
    hal_spi_mode_t mode;                /**< Master or slave mode. */
    hal_spi_dir_t direction;            /**< Transmit direction. */
    hal_spi_size_t data_size;           /**< Size of packet. */
    hal_spi_nss_t nss_mode;             /**< NSS set by sw or hw. */
    hal_spi_prescaler_t prescaler;      /**< Baudrate prescaler. */
    hal_spi_frame_t frame_type;         /**< Motorolla or TI. */
    hal_bool_t hw_crc;                  /**< ENABLE/DISABLE CRC calculation. */
} hal_spi_init_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Resets spi to default values.
 *  \param  spi : Defines spi to reset.
 */
void hal_spi_reset(reg_spi_t *spi)
__attribute__((deprecated("Please use SPIx_RESET().")));

/**
 *  \brief  Initializes the specified serial peripheral interface.
 *  \param  spi : Defines SPI to initialize.
 *  \param  init : Structure with SPI configuration.
 */
void hal_spi_init(reg_spi_t *spi, hal_spi_init_t init);

/**
 *  \brief  Start specified SPI.
 *  \param  spi : Defines SPI to start.
 *  \param  write_byte: Data to be sent via SPI.
 *  \return Data received from SPI.
 */
uint8_t hal_spi_read_write(reg_spi_t *spi, uint8_t write_byte);

 
#endif
