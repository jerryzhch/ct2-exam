/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Implementation of module hal_spi.
 *
 *  The hardware abstraction layer for the serial peripheral interface.
 *
 *  $Id$
 * ------------------------------------------------------------------------- */

/* User includes */
#include "hal_spi.h"
//#include "hal_gpio.h"         // Uncomment if needed.


/* -- Macros
 * ------------------------------------------------------------------------- */

#define BIT_TXE     (uint32_t) 0x00000002
#define BIT_RXNE    (uint32_t) 0x00000001
#define BIT_BSY     (uint32_t) 0x00000080


/* -- Local function declarations
 * ------------------------------------------------------------------------- */

__weak void spi_gpio_init(void);
__weak void set_ss_pin_low(void);
__weak void set_ss_pin_high(void);
__weak void wait_10_us(void);


/* -- Public function definitions
 * ------------------------------------------------------------------------- */

/*
 * See header file
 */
void hal_spi_reset(reg_spi_t *spi)
{
    spi->CR1 = 0u;
    spi->CR2 = 0u;
    spi->CRCPR = 7u;
}

/*
 * See header file
 */
void hal_spi_init(reg_spi_t *spi, hal_spi_init_t init)
{
    uint32_t reg = 0;
    uint32_t clear_reg = 0;
    
    /* initialize gpio if nss in sw mode */
    if (init.nss_mode == HAL_SPI_NSS_SW) {
        spi_gpio_init();
    }
    
    /* process mode */
    clear_reg &= ~(0x1 << 2u);
    reg |= (init.mode << 2u);
    
    /* process direction */
    clear_reg &= ~(0x3 << 14u);
    reg |= (init.direction << 14u);
    
    /* process size */
    clear_reg &= ~(0x1 << 11u);
    reg |= (init.data_size << 11u);
    
    /* process nss mode */
    clear_reg &= ~(0x3 << 8u);
    reg |= (init.nss_mode << 8u);
    
    /* process prescaler */
    clear_reg &= ~(0x7 << 3u);
    reg |= (init.prescaler << 3u);
    
    /* process crc mode */
    clear_reg &= ~(0x1 << 13u);
    reg |= (init.hw_crc << 13u);
    
    spi->CR1 &= clear_reg;
    spi->CR1 |= reg;
    
    /* process frame type */
    clear_reg = ~(0x1 << 4u);
    reg = (init.frame_type << 4u);
    
    spi->CR2 &= clear_reg;
    spi->CR2 |= reg;
    
    
    /* enable spi */
    spi->CR1 |= (0x1 << 6u);
}


/*
 * See header file
 */
uint8_t hal_spi_read_write(reg_spi_t *spi, uint8_t write_byte)
{
    set_ss_pin_low();
    
    SPI1->DR = write_byte;              // Copy data to output buffer.
#ifndef TESTING
    while(!(SPI1->SR & BIT_TXE));       // Wait for transmission to complete.
    while(!(SPI1->SR & BIT_RXNE));      // Wait for received data to complete.
    while((SPI1->SR & BIT_BSY));        // Wait for transfer to complete.
#endif

    wait_10_us();                       // Because display is very slow.
    set_ss_pin_high();

    return (uint8_t) SPI1->DR;
}


/* -- Local function definitions
 * ------------------------------------------------------------------------- */

/**
 *  \brief  Setup GPIO for enabling SPI device.
 */
__weak void spi_gpio_init(void)
{
    /* NOTE : Implement this function as needed. */
}


/**
 *  \brief  Set slave-select pin low.
 */
__weak void set_ss_pin_low(void)
{
    /* NOTE : Implement this function as needed. */
}


/**
 *  \brief  Set slave-select pin high.
 */
__weak void set_ss_pin_high(void)
{
    /* NOTE : Implement this function as needed. */
}


/**
 *  \brief  Wait for approximately 10us.
 */
__weak void wait_10_us(void)
{
    /* NOTE : Implement this function as needed. */
}

