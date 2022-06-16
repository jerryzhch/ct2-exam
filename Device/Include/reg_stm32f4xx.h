/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Registries for STM32F4xx.
 * 
 *  \file   reg_stm32f4xx.h
 *  $Id$
 * ------------------------------------------------------------------------- */
#ifndef _REG_STM32F4xx_H
#define _REG_STM32F4xx_H

#include <stdint.h>


/*  Utilities
 * ------------------------------------------------------------------------- */
 
/* Set and clear bit in register. */
#define REG_SET(REG, BIT)       ( (REG) |= (BIT) )
#define REG_CLR(REG, BIT)       ( (REG) &= ~(BIT) )

/* Toggle (set, then clear) bit in register */
#define REG_TGL(REG, BIT)       do {                        \
                                    REG_SET((REG), (BIT));  \
                                    REG_CLR((REG), (BIT));  \
                                } while(0)


/*  RCC - register clock control
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_rcc_t
 *  \brief  Representation of RCC register.
 *
 *  Described in reference manual p.147ff.
 */
typedef struct {
    volatile uint32_t CR;           /**< Clock control register. */
    volatile uint32_t PLLCFGR;      /**< PLL configuration register. */
    volatile uint32_t CFGR;         /**< Clock configuration register. */
    volatile uint32_t CIR;          /**< Clock interrupt register. */
    volatile uint32_t AHB1RSTR;     /**< AHB1 peripheral reset register. */
    volatile uint32_t AHB2RSTR;     /**< AHB2 peripheral reset register. */
    volatile uint32_t AHB3RSTR;     /**< AHB3 peripheral reset register. */
    uint32_t RESERVED;
    volatile uint32_t APB1RSTR;     /**< APB1 peripheral reset register. */
    volatile uint32_t APB2RSTR;     /**< APB2 peripheral reset register. */
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;      /**< AHB1 peripheral clock enable register. */
    volatile uint32_t AHB2ENR;      /**< AHB2 peripheral clock enable register. */
    volatile uint32_t AHB3ENR;      /**< AHB3 peripheral clock enable register. */
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;      /**< APB1 peripheral clock enable register. */
    volatile uint32_t APB2ENR;      /**< APB2 peripheral clock enable register. */
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;    /**< AHB1 peripheral clock enable in lp register. */
    volatile uint32_t AHB2LPENR;    /**< AHB2 peripheral clock enable in lp register. */
    volatile uint32_t AHB3LPENR;    /**< AHB3 peripheral clock enable in lp register. */
    uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;    /**< APB1 peripheral clock enable in lp register. */
    volatile uint32_t APB2LPENR;    /**< APB2 peripheral clock enable in lp register. */
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;         /**< Backup domain control register. */
    volatile uint32_t CSR;          /**< Clock controll and status register. */
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;        /**< Spreadspectrum clock gen. register. */
    volatile uint32_t PLLI2SCFGR;   /**< PLLI2S configuration register. */
    volatile uint32_t PLLSAICFGR;   /**< PLLSAI configuration register. */
    volatile uint32_t DCKCFGR;      /**< Dedicated clock conf. register. */
} reg_rcc_t;


#ifdef TESTING
    extern reg_rcc_t rcc_testing;
    #define RCC                     ( (reg_rcc_t *) &rcc_testing )
#else
    #define RCC                     ( (reg_rcc_t *) 0x40023800 )
#endif


/*  FLASH - flash interface register
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_flash_t
 *  \brief  Representation of FLASH register.
 *
 *  Described in reference manual p.73ff.
 */
typedef struct {
    volatile uint32_t ACR;      /**< Access control register. */
    volatile uint32_t KEYR;     /**< Key register. */
    volatile uint32_t OPTKEYR;  /**< Option key register. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t OPTCR;    /**< Option control register. */
    volatile uint32_t OPTCR1;   /**< Option1 control register. */
} reg_flash_t;


#ifdef TESTING
    extern reg_flash_t flash_testing;
    #define FLASH                   ( (reg_flash_t *) &flash_testing )
#else
    #define FLASH                   ( (reg_flash_t *) 0x40023c00 )
#endif

/*  CRC - checksum calculation unit
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_crc_t
 *  \brief  Representation of CRC register.
 *
 *  Described in reference manual p.112ff.
 */
typedef struct {
    volatile uint32_t DR;       /**< Data register. */
    volatile uint32_t IDR;      /**< Independent data register. */
    volatile uint32_t CR;       /**< Control register. */
} reg_crc_t;


/* Register macro */

#ifdef TESTING
    extern reg_crc_t crc_testing;
    #define CRC                   ( (reg_crc_t *) &crc_testing )
#else
    #define CRC                   ( (reg_crc_t *) 0x40023000 )
#endif


/* Helper macros */

#define CRC_RCC_PATTERN         ( 0x1 << 12u )
#define CRC_RESET()             REG_TGL(RCC->AHB1RSTR,  CRC_RCC_PATTERN)
#define CRC_ENABLE()            REG_SET(RCC->AHB1ENR,   CRC_RCC_PATTERN)
#define CRC_DISABLE()           REG_CLR(RCC->AHB1ENR,   CRC_RCC_PATTERN)
#define CRC_LP_ENABLE()         REG_SET(RCC->AHB1LPENR, CRC_RCC_PATTERN)
#define CRC_LP_DISABLE()        REG_CLR(RCC->AHB1LPENR, CRC_RCC_PATTERN)


/*  PWR - power controller
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_pwr_t
 *  \brief  Representation of PWR register.
 *
 *  Described in reference manual p.115ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t CSR;      /**< Control/status register. */
} reg_pwr_t;


/* Register macro */

#ifdef TESTING
    extern reg_pwr_t pwr_testing;
    #define PWR                     ( (reg_pwr_t *) &pwr_testing )
#else
    #define PWR                     ( (reg_pwr_t *) 0x40007000 )
#endif

/* Helper macros */

#define PWR_RCC_PATTERN         ( 0x1 << 28u )
#define PWR_RESET()             REG_TGL(RCC->APB1RSTR,  PWR_RCC_PATTERN)
#define PWR_ENABLE()            REG_SET(RCC->APB1ENR,   PWR_RCC_PATTERN)
#define PWR_DISABLE()           REG_CLR(RCC->APB1ENR,   PWR_RCC_PATTERN)
#define PWR_LP_ENABLE()         REG_SET(RCC->APB1LPENR, PWR_RCC_PATTERN)
#define PWR_LP_DISABLE()        REG_CLR(RCC->APB1LPENR, PWR_RCC_PATTERN)


/*  GPIO - general purpose input / output
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_gpio_t
 *  \brief  Representation of GPIO register.
 *
 *  Described in reference manual p.265ff.
 */
typedef struct {
    volatile uint32_t MODER;    /**< Port mode register. */
    volatile uint32_t OTYPER;   /**< Output type register. */
    volatile uint32_t OSPEEDR;  /**< Output speed register. */
    volatile uint32_t PUPDR;    /**< Port pull-up/pull-down register. */
    volatile uint32_t IDR;      /**< Input data register. */
    volatile uint32_t ODR;      /**< output data register. */
    volatile uint32_t BSRR;     /**< Bit set/reset register */
    volatile uint32_t LCKR;     /**< Port lock register. */
    volatile uint32_t AFRL;     /**< AF low register pin 0..7. */
    volatile uint32_t AFRH;     /**< AF high register pin 8..15. */                                
} reg_gpio_t;


/* Register macros */


#ifdef TESTING
    extern reg_gpio_t gpioa_testing;
    extern reg_gpio_t gpiob_testing;
    extern reg_gpio_t gpioc_testing;
    extern reg_gpio_t gpiod_testing;
    extern reg_gpio_t gpioe_testing;
    extern reg_gpio_t gpiof_testing;
    extern reg_gpio_t gpiog_testing;
    extern reg_gpio_t gpioh_testing;
    extern reg_gpio_t gpioi_testing;
    extern reg_gpio_t gpioj_testing;
    extern reg_gpio_t gpiok_testing;
    #define GPIOA                   ( (reg_gpio_t *) &gpioa_testing )
    #define GPIOB                   ( (reg_gpio_t *) &gpiob_testing )
    #define GPIOC                   ( (reg_gpio_t *) &gpioc_testing )
    #define GPIOD                   ( (reg_gpio_t *) &gpiod_testing )
    #define GPIOE                   ( (reg_gpio_t *) &gpioe_testing )
    #define GPIOF                   ( (reg_gpio_t *) &gpiof_testing )
    #define GPIOG                   ( (reg_gpio_t *) &gpiog_testing )
    #define GPIOH                   ( (reg_gpio_t *) &gpioh_testing )
    #define GPIOI                   ( (reg_gpio_t *) &gpioi_testing )
    #define GPIOJ                   ( (reg_gpio_t *) &gpioj_testing )
    #define GPIOK                   ( (reg_gpio_t *) &gpiok_testing )
#else
    #define GPIOA                   ( (reg_gpio_t *) 0x40020000 )
    #define GPIOB                   ( (reg_gpio_t *) 0x40020400 )
    #define GPIOC                   ( (reg_gpio_t *) 0x40020800 )
    #define GPIOD                   ( (reg_gpio_t *) 0x40020c00 )
    #define GPIOE                   ( (reg_gpio_t *) 0x40021000 )
    #define GPIOF                   ( (reg_gpio_t *) 0x40021400 )
    #define GPIOG                   ( (reg_gpio_t *) 0x40021800 )
    #define GPIOH                   ( (reg_gpio_t *) 0x40021c00 )
    #define GPIOI                   ( (reg_gpio_t *) 0x40022000 )
    #define GPIOJ                   ( (reg_gpio_t *) 0x40022400 )
    #define GPIOK                   ( (reg_gpio_t *) 0x40022800 )
#endif


/* Helper macros */

#define GPIOA_RCC_PATTERN       ( 0x1 << 0u )
#define GPIOA_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOA_RCC_PATTERN)
#define GPIOA_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOA_RCC_PATTERN)
#define GPIOA_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOA_RCC_PATTERN)
#define GPIOA_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOA_RCC_PATTERN)
#define GPIOA_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOA_RCC_PATTERN)

#define GPIOB_RCC_PATTERN       ( 0x1 << 1u )
#define GPIOB_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOB_RCC_PATTERN)
#define GPIOB_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOB_RCC_PATTERN)
#define GPIOB_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOB_RCC_PATTERN)
#define GPIOB_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOB_RCC_PATTERN)
#define GPIOB_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOB_RCC_PATTERN)

#define GPIOC_RCC_PATTERN       ( 0x1 << 2u )
#define GPIOC_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOC_RCC_PATTERN)
#define GPIOC_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOC_RCC_PATTERN)
#define GPIOC_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOC_RCC_PATTERN)
#define GPIOC_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOC_RCC_PATTERN)
#define GPIOC_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOC_RCC_PATTERN)

#define GPIOD_RCC_PATTERN       ( 0x1 << 3u )
#define GPIOD_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOD_RCC_PATTERN)
#define GPIOD_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOD_RCC_PATTERN)
#define GPIOD_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOD_RCC_PATTERN)
#define GPIOD_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOD_RCC_PATTERN)
#define GPIOD_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOD_RCC_PATTERN)

#define GPIOE_RCC_PATTERN       ( 0x1 << 4u )
#define GPIOE_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOE_RCC_PATTERN)
#define GPIOE_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOE_RCC_PATTERN)
#define GPIOE_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOE_RCC_PATTERN)
#define GPIOE_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOE_RCC_PATTERN)
#define GPIOE_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOE_RCC_PATTERN)

#define GPIOF_RCC_PATTERN       ( 0x1 << 5u )
#define GPIOF_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOF_RCC_PATTERN)
#define GPIOF_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOF_RCC_PATTERN)
#define GPIOF_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOF_RCC_PATTERN)
#define GPIOF_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOF_RCC_PATTERN)
#define GPIOF_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOF_RCC_PATTERN)

#define GPIOG_RCC_PATTERN       ( 0x1 << 6u )
#define GPIOG_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOG_RCC_PATTERN)
#define GPIOG_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOG_RCC_PATTERN)
#define GPIOG_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOG_RCC_PATTERN)
#define GPIOG_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOG_RCC_PATTERN)
#define GPIOG_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOG_RCC_PATTERN)

#define GPIOH_RCC_PATTERN       ( 0x1 << 7u )
#define GPIOH_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOH_RCC_PATTERN)
#define GPIOH_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOH_RCC_PATTERN)
#define GPIOH_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOH_RCC_PATTERN)
#define GPIOH_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOH_RCC_PATTERN)
#define GPIOH_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOH_RCC_PATTERN)

#define GPIOI_RCC_PATTERN       ( 0x1 << 8u )
#define GPIOI_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOI_RCC_PATTERN)
#define GPIOI_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOI_RCC_PATTERN)
#define GPIOI_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOI_RCC_PATTERN)
#define GPIOI_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOI_RCC_PATTERN)
#define GPIOI_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOI_RCC_PATTERN)

#define GPIOJ_RCC_PATTERN       ( 0x1 << 9u )
#define GPIOJ_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOJ_RCC_PATTERN)
#define GPIOJ_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOJ_RCC_PATTERN)
#define GPIOJ_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOJ_RCC_PATTERN)
#define GPIOJ_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOJ_RCC_PATTERN)
#define GPIOJ_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOJ_RCC_PATTERN)

#define GPIOK_RCC_PATTERN       ( 0x1 << 10u )
#define GPIOK_RESET()           REG_TGL(RCC->AHB1RSTR,  GPIOK_RCC_PATTERN)
#define GPIOK_ENABLE()          REG_SET(RCC->AHB1ENR,   GPIOK_RCC_PATTERN)
#define GPIOK_DISABLE()         REG_CLR(RCC->AHB1ENR,   GPIOK_RCC_PATTERN)
#define GPIOK_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, GPIOK_RCC_PATTERN)
#define GPIOK_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, GPIOK_RCC_PATTERN)


/*  SYSCFG - system configuration controller
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_syscfg_t
 *  \brief  Representation of SYSCFG register.
 *
 *  Described in reference manual p.286ff.
 */
typedef struct {
    volatile uint32_t MEMRMP;       /**< Memory remap register. */
    volatile uint32_t PMC;          /**< Peripheral mode register. */
    volatile uint32_t EXTICR1;      /**< External interrupt conf. register 1. */
    volatile uint32_t EXTICR2;      /**< External interrupt conf. register 2. */
    volatile uint32_t EXTICR3;      /**< External interrupt conf. register 3. */
    volatile uint32_t EXTICR4;      /**< External interrupt conf. register 4. */
    uint32_t RESERVED[2];
    volatile uint32_t CMPCR;        /**< Compensation cell control register. */
} reg_syscfg_t;


/* Register macro */

#ifdef TESTING
    extern reg_syscfg_t syscfg_testing;
    #define SYSCFG                  ( (reg_syscfg_t *) &syscfg_testing )
#else
    #define SYSCFG                  ( (reg_syscfg_t *) 0x40013800 )
#endif


/* Helper macros */

#define SYSCFG_RCC_PATTERN      ( 0x1 << 14u )
#define SYSCFG_RESET()          REG_TGL(RCC->APB2RSTR,  SYSCFG_RCC_PATTERN)
#define SYSCFG_ENABLE()         REG_SET(RCC->APB2ENR,   SYSCFG_RCC_PATTERN)
#define SYSCFG_DISABLE()        REG_CLR(RCC->APB2ENR,   SYSCFG_RCC_PATTERN)
#define SYSCFG_LP_ENABLE()      REG_SET(RCC->APB2LPENR, SYSCFG_RCC_PATTERN)
#define SYSCFG_LP_DISABLE()     REG_CLR(RCC->APB2LPENR, SYSCFG_RCC_PATTERN)


/*  DMA - direct memory access
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_dma_stream_t
 *  \brief  Representation of DMA stream register.
 */
typedef struct {
    volatile uint32_t CR;       /**< Stream configuration register. */
    volatile uint32_t NDTR;     /**< Number of data register. */
    volatile uint32_t PAR;      /**< Peripheral address register. */
    volatile uint32_t M0AR;     /**< Memory 0 address register. */
    volatile uint32_t M1AR;     /**< Memory 1 address register. */
    volatile uint32_t FCR;      /**< FIFO control register. */
} reg_dma_stream_t;
	

/**
 *  \struct reg_dma_t
 *  \brief  Representation of DMA register.
 *
 *  Described in reference manual p.299ff.
 */
typedef struct {
    volatile uint32_t LISR;         /**< Low interrupt status register. */
    volatile uint32_t HISR;         /**< High interrupt status register. */
    volatile uint32_t LIFCR;        /**< Low interrupt flag clear register. */
    volatile uint32_t HIFCR;        /**< High interrupt flag clear register. */
    reg_dma_stream_t  STREAM[8];    /**< Struct of stream registers. */
} reg_dma_t;


/* Register macros */

#ifdef TESTING
    extern reg_dma_t dma1_testing;
    extern reg_dma_t dma2_testing;
    #define DMA1                    ( (reg_dma_t *) &dma1_testing )
    #define DMA2                    ( (reg_dma_t *) &dma2_testing )
#else
    #define DMA1                    ( (reg_dma_t *) 0x40026000 )
    #define DMA2                    ( (reg_dma_t *) 0x40026400 )
#endif


/* Helper macros */

#define DMA1_RCC_PATTERN        ( 0x1 << 21u )
#define DMA1_RESET()            REG_TGL(RCC->AHB1RSTR,  DMA1_RCC_PATTERN)
#define DMA1_ENABLE()           REG_SET(RCC->AHB1ENR,   DMA1_RCC_PATTERN)
#define DMA1_DISABLE()          REG_CLR(RCC->AHB1ENR,   DMA1_RCC_PATTERN)
#define DMA1_LP_ENABLE()        REG_SET(RCC->AHB1LPENR, DMA1_RCC_PATTERN)
#define DMA1_LP_DISABLE()       REG_CLR(RCC->AHB1LPENR, DMA1_RCC_PATTERN)

#define DMA2_RCC_PATTERN        ( 0x1 << 22u )
#define DMA2_RESET()            REG_TGL(RCC->AHB1RSTR,  DMA2_RCC_PATTERN)
#define DMA2_ENABLE()           REG_SET(RCC->AHB1ENR,   DMA2_RCC_PATTERN)
#define DMA2_DISABLE()          REG_CLR(RCC->AHB1ENR,   DMA2_RCC_PATTERN)
#define DMA2_LP_ENABLE()        REG_SET(RCC->AHB1LPENR, DMA2_RCC_PATTERN)
#define DMA2_LP_DISABLE()       REG_CLR(RCC->AHB1LPENR, DMA2_RCC_PATTERN)


/*  DMA2D - direct memory access
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_dma2d_t
 *  \brief  Representation of DMA2D register.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t ISR;      /**< Interrupt status register. */
    volatile uint32_t IFCR;     /**< Interrupt flag clear register. */
    volatile uint32_t FGMAR;    /**< Foreground memory address register. */
    volatile uint32_t FGOR;     /**< Foreground offset register. */
    volatile uint32_t BGMAR;    /**< Background memory address register. */
    volatile uint32_t BGOR;     /**< Background offset register. */
    volatile uint32_t FGPFCCR;  /**< Foreground PFC control register. */
    volatile uint32_t FGCOLR;   /**< Foreground color register. */
    volatile uint32_t BGPFCCR;  /**< Background PFC control register. */
    volatile uint32_t BGCOLR;   /**< Background color register. */
    volatile uint32_t FGCMAR;   /**< Foreground CLUT memory address reg. */
    volatile uint32_t BGCMAR;   /**< Background CLUT memory address reg. */
    volatile uint32_t OPFCCR;   /**< Output PFC control register. */
    volatile uint32_t OCOLR;    /**< Output color register. */
    volatile uint32_t OMAR;     /**< Output memory address register. */
    volatile uint32_t OOR;      /**< Output offset register. */
    volatile uint32_t NLR;      /**< Number of line register. */
    volatile uint32_t LWR;      /**< Line watermark register. */
    volatile uint32_t AMTCR;    /**< AHB master timer configuration reg. */
} reg_dma2d_t;


/* Register macros */

#ifdef TESTING
    extern reg_dma2d_t dma2d_testing;
    #define DMA2D                   ( (reg_dma2d_t *) &dma2d_testing )
#else
    #define DMA2D                   ( (reg_dma2d_t *) 0x4002b000 )
#endif


/* Helper macros */

#define DMA2D_RCC_PATTERN       ( 0x1 << 23u )
#define DMA2D_RESET()           REG_TGL(RCC->AHB1RSTR,  DMA1_RCC_PATTERN)
#define DMA2D_ENABLE()          REG_SET(RCC->AHB1ENR,   DMA1_RCC_PATTERN)
#define DMA2D_DISABLE()         REG_CLR(RCC->AHB1ENR,   DMA1_RCC_PATTERN)
#define DMA2D_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, DMA1_RCC_PATTERN)
#define DMA2D_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, DMA1_RCC_PATTERN)


/*  NVIC - nested vector interrupt controller
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_nvic_t
 *  \brief  Representation of NVIC register.
 *
 *  Described in programming manual p.193ff.
 */
typedef struct {
    volatile uint32_t ISER0;    /**< Interrupt set-enable register 1. */
    volatile uint32_t ISER1;    /**< Interrupt set-enable register 2. */
    volatile uint32_t ISER2;    /**< Interrupt set-enable register 3. */
    uint32_t RESERVED1[29];
    volatile uint32_t ICER0;    /**< Interrupt clear-enable register 1. */
    volatile uint32_t ICER1;    /**< Interrupt clear-enable register 2. */
    volatile uint32_t ICER2;    /**< Interrupt clear-enable register 3. */
    uint32_t RESERVED2[29];
    volatile uint32_t ISPR0;    /**< Interrupt set-pending register 1. */
    volatile uint32_t ISPR1;    /**< Interrupt set-pending register 2. */
    volatile uint32_t ISPR2;    /**< Interrupt set-pending register 3. */
    uint32_t RESERVED3[29];
    volatile uint32_t ICPR0;    /**< Interrupt clear-pending register 1. */
    volatile uint32_t ICPR1;    /**< Interrupt clear-pending register 2. */
    volatile uint32_t ICPR2;    /**< Interrupt clear-pending register 3. */
    uint32_t RESERVED4[29];
    volatile uint32_t IABR0;    /**< Interrupt active bit register 1. */
    volatile uint32_t IABR1;    /**< Interrupt active bit register 2. */
    volatile uint32_t IABR2;    /**< Interrupt active bit register 3. */
    uint32_t RESERVED5[61];
    volatile uint8_t  IP[81];   /**< Interrupt priority. */
    uint8_t  RESERVED6[3];
    uint32_t RESERVED7[684];
    volatile uint32_t STIR;     /**< Software trigger interrupt register. */
} reg_nvic_t;


/* Register macro */

#ifdef TESTING
    extern reg_nvic_t nvic_testing;
    #define NVIC                    ( (reg_nvic_t *) &nvic_testing )
#else
    #define NVIC                    ( (reg_nvic_t *) 0xe000e100 )
#endif

/*  EXTI - external interrupt
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_exti_t
 *  \brief  Representation of EXTI register.
 *
 *  Described in reference manual p.368ff.
 */
typedef struct {
    volatile uint32_t IMR;      /**< Interrupt mask register. */
    volatile uint32_t EMR;      /**< Event mask register. */
    volatile uint32_t RTSR;     /**< Rising trigger selection register. */
    volatile uint32_t FTSR;     /**< Falling trigger selection register. */
    volatile uint32_t SWIER;    /**< Software interrupt event register. */
    volatile uint32_t PR;       /**< Pending register. */
} reg_exti_t;


/* Register macro */

#ifdef TESTING
    extern reg_exti_t exti_testing;
    #define EXTI                    ( (reg_exti_t *) &exti_testing )
#else
    #define EXTI                    ( (reg_exti_t *) 0x40013c00 )
#endif


/*  ADC - analog digital converter
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_adc_t
 *  \brief  Representation of ADC register.
 *
 *  Described in reference manual p.385ff.
 */
typedef struct {
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t CR1;      /**< Control register 1. */
    volatile uint32_t CR2;      /**< Control register 2. */
    volatile uint32_t SMPR1;    /**< Sample time register 1. */
    volatile uint32_t SMPR2;    /**< Sample time register 2. */
    volatile uint32_t JOFR1;    /**< Injected channel data offset register 1. */
    volatile uint32_t JOFR2;    /**< Injected channel data offset register 2. */
    volatile uint32_t JOFR3;    /**< Injected channel data offset register 3. */
    volatile uint32_t JOFR4;    /**< Injected channel data offset register 4. */
    volatile uint32_t HTR;      /**< Higher treshold register. */
    volatile uint32_t LTR;      /**< Lower treshold register. */
    volatile uint32_t SQR1;     /**< Regular sequence register 1. */
    volatile uint32_t SQR2;     /**< Regular sequence register 2. */
    volatile uint32_t SQR3;     /**< Regular sequence register 3. */
    volatile uint32_t JSQR;     /**< Injected sequence register. */
    volatile uint32_t JDR1;     /**< Injected data register 1. */
    volatile uint32_t JDR2;     /**< Injected data register 2. */
    volatile uint32_t JDR3;     /**< Injected data register 3. */
    volatile uint32_t JDR4;     /**< Injected data register 4. */
    volatile uint32_t DR;       /**< Regular data register. */
} reg_adc_t;


/* Register macros */

#ifdef TESTING
    extern reg_adc_t adc1_testing;
    extern reg_adc_t adc2_testing;
    extern reg_adc_t adc3_testing;
    #define ADC1                    ( (reg_adc_t *) &adc1_testing )
    #define ADC2                    ( (reg_adc_t *) &adc2_testing )
    #define ADC3                    ( (reg_adc_t *) &adc3_testing )
#else
    #define ADC1                    ( (reg_adc_t *) 0x40012000 )
    #define ADC2                    ( (reg_adc_t *) 0x40012100 )
    #define ADC3                    ( (reg_adc_t *) 0x40012200 )
#endif


/* Helper macros */

#define ADC1_RCC_PATTERN        ( 0x1 << 8u )
#define ADC1_ENABLE()           REG_SET(RCC->APB2ENR,   ADC1_RCC_PATTERN)
#define ADC1_DISABLE()          REG_CLR(RCC->APB2ENR,   ADC1_RCC_PATTERN)
#define ADC1_LP_ENABLE()        REG_SET(RCC->APB2LPENR, ADC1_RCC_PATTERN)
#define ADC1_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, ADC1_RCC_PATTERN)

#define ADC2_RCC_PATTERN        ( 0x1 << 9u )
#define ADC2_ENABLE()           REG_SET(RCC->APB2ENR,   ADC2_RCC_PATTERN)
#define ADC2_DISABLE()          REG_CLR(RCC->APB2ENR,   ADC2_RCC_PATTERN)
#define ADC2_LP_ENABLE()        REG_SET(RCC->APB2LPENR, ADC2_RCC_PATTERN)
#define ADC2_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, ADC2_RCC_PATTERN)

#define ADC3_RCC_PATTERN        ( 0x1 << 10u )                  
#define ADC3_ENABLE()           REG_SET(RCC->APB2ENR,   ADC3_RCC_PATTERN)
#define ADC3_DISABLE()          REG_CLR(RCC->APB2ENR,   ADC3_RCC_PATTERN)
#define ADC3_LP_ENABLE()        REG_SET(RCC->APB2LPENR, ADC3_RCC_PATTERN)
#define ADC3_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, ADC3_RCC_PATTERN)

#define ADCx_RESET()            REG_TGL(RCC->APB2RSTR,  ADC1_RCC_PATTERN)


/**
 *  \struct reg_adccom_t
 *  \brief  Representation of ADC common register.
 *
 *  Described in reference manual p.385ff.
 */
typedef struct {
    volatile uint32_t CSR;      /**< Common status register. */
    volatile uint32_t CCR;      /**< Common control register. */
    volatile uint32_t CDR;      /**< Common regular data register. */
} reg_adccom_t;


/* Register macro */

#ifdef TESTING
    extern reg_adccom_t adccom_testing;
    #define ADCCOM                  ( (reg_adccom_t *) &adccom_testing )
#else
    #define ADCCOM                  ( (reg_adccom_t *) 0x40012300 )
#endif


/*  DAC - digital analog converter
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_dac_t
 *  \brief  Representation of DAC register.
 *
 *  Described in reference manual p.430ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t SWTRIGR;  /**< Software trigger register. */
    volatile uint32_t DHR12R1;  /**< Ch1 12-bit right-aligned data register. */
    volatile uint32_t DHR12L1;  /**< Ch1 12-bit left-aligned data register. */
    volatile uint32_t DHR8R1;   /**< Ch1 8-bit right-aligned data register. */
    volatile uint32_t DHR12R2;  /**< Ch2 12-bit right-aligned data register. */
    volatile uint32_t DHR12L2;  /**< Ch2 12-bit left-aligned data register. */
    volatile uint32_t DHR8R2;   /**< Ch2 8-bit right-aligned data register. */
    volatile uint32_t DHR12RD;  /**< Dual 12-bit right-align. data register. */
    volatile uint32_t DHR12LD;  /**< Dual 12-bit left-aligned data register. */
    volatile uint32_t DHR8RD;   /**< Dual 8-bit right-aligned data register. */
    volatile uint32_t DOR1;     /**< Ch1 data output register. */
    volatile uint32_t DOR2;     /**< Ch2 data output register. */
    volatile uint32_t SR;       /**< Status register. */
} reg_dac_t;


/* Register macro */

#ifdef TESTING
    extern reg_dac_t dac_testing;
    #define DAC                     ( (reg_dac_t *) &dac_testing )
#else
    #define DAC                     ( (reg_dac_t *) 0x40007400 )
#endif


/* Helper macros */

#define DAC_RCC_PATTERN         ( 0x1 << 29u )
#define DAC_RESET()             REG_TGL(RCC->APB1RSTR,  DAC_RCC_PATTERN)
#define DAC_ENABLE()            REG_SET(RCC->APB1ENR,   DAC_RCC_PATTERN)
#define DAC_DISABLE()           REG_CLR(RCC->APB1ENR,   DAC_RCC_PATTERN)
#define DAC_LP_ENABLE()         REG_SET(RCC->APB1LPENR, DAC_RCC_PATTERN)
#define DAC_LP_DISABLE()        REG_CLR(RCC->APB1LPENR, DAC_RCC_PATTERN)


/*  DCMI - digital camery mxxx interface
 * ------------------------------------------------------------------------- */
	
/**
 *  \struct reg_dcmi_t
 *  \brief  Representation of DCMI register.
 *
 *  Described in reference manual p.452ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t RIS;      /**< Raw interrupt status register. */
    volatile uint32_t IER;      /**< Interrupt enable register. */
    volatile uint32_t MIS;      /**< Masked interrupt status register. */
    volatile uint32_t ICR;      /**< Interrupt clear register. */
    volatile uint32_t ESCR;     /**< Embedded sync. code register. */
    volatile uint32_t ESUR;     /**< Embedded sync. unmask register. */
    volatile uint32_t CWSTRT;   /**< Crop window start. */
    volatile uint32_t CWSIZE;   /**< Crop window size. */
    volatile uint32_t DR;       /**< Data register. */
} reg_dcmi_t;


/* Register macro */

#ifdef TESTING
    extern reg_dcmi_t dcmi_testing;
    #define DCMI                    ( (reg_dcmi_t *) &dcmi_testing )
#else
    #define DCMI                    ( (reg_dcmi_t *) 0x50050000 )
#endif


/* Helper macros */

#define DCMI_RCC_PATTERN        ( 0x1 << 0u )
#define DCMI_RESET()            REG_TGL(RCC->AHB2RSTR,  DCMI_RCC_PATTERN)
#define DCMI_ENABLE()           REG_SET(RCC->AHB2ENR,   DCMI_RCC_PATTERN)
#define DCMI_DISABLE()          REG_CLR(RCC->AHB2ENR,   DCMI_RCC_PATTERN)
#define DCMI_LP_ENABLE()        REG_SET(RCC->AHB2LPENR, DCMI_RCC_PATTERN)
#define DCMI_LP_DISABLE()       REG_CLR(RCC->AHB2LPENR, DCMI_RCC_PATTERN)


/*  LTDC - liquid txxx display controller
 * ------------------------------------------------------------------------- */
	
/**
 *  \struct reg_ltdc_lc_t
 *  \brief  Representation of LTDC layer control register.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t WHPCR;    /**< Window horizontal pos. conf. register. */
    volatile uint32_t WVPCR;    /**< Window vertical pos. conf. register. */
    volatile uint32_t CKCR;     /**< Color keying configuration register. */
    volatile uint32_t PFCR;     /**< Pixel format configuration register. */
    volatile uint32_t CACR;     /**< Constant alpha configuration register. */
    volatile uint32_t DCCR;     /**< Default color configuration register. */
    volatile uint32_t BFCR;     /**< Blending factors conf. register. */
    volatile uint32_t CFBAR;    /**< Color frame buffer address register. */
    volatile uint32_t CFBLR;    /**< Color frame buffer length register. */
    volatile uint32_t CFBLNR;   /**< Color frame buffer line nr. register. */
    volatile uint32_t CLUTWR;   /**< CLUT write register. */
} reg_ltdc_lc_t;

	
/**
 *  \struct reg_ltdc_t
 *  \brief  Representation of LTDC register.
 *
 *  Described in reference manual p.475ff.
 */
typedef struct {
    uint32_t RESERVED1[2];
    volatile uint32_t SSCR;     /**< Sync. size configuration register. */
    volatile uint32_t BPCR;     /**< Back porch :D configuration register. */
    volatile uint32_t AWCR;     /**< Active width configuration register. */
    volatile uint32_t TWCR;     /**< Total width configuration register. */
    volatile uint32_t GCR;      /**< Global configuration register. */
    volatile uint32_t SRCR;     /**< Shadow reload configuration register. */
    volatile uint32_t BCCR;     /**< Background color conf. register. */
    volatile uint32_t IER;      /**< Interrupt enable register. */
    volatile uint32_t ISR;      /**< Interrupt status register. */
    volatile uint32_t ICR;      /**< Interrupt clear register. */
    volatile uint32_t LIPCR;    /**< Line interrupt position conf. register. */
    volatile uint32_t CPSR;     /**< Current position status register. */
    volatile uint32_t CDSR;     /**< Current display status register. */
    uint32_t RESERVED2[14];
    reg_ltdc_lc_t LAYER1;       /**< Layer 1 registers. */
    uint32_t RESERVED3[15];
    reg_ltdc_lc_t LAYER2;       /**< Layer 2 registers. */
} reg_ltdc_t;


/* Register macro */

#ifdef TESTING
    extern reg_ltdc_t ltdc_testing;
    #define LTDC                    ( (reg_ltdc_t *) &ltdc_testing )
#else
    #define LTDC                    ( (reg_ltdc_t *) 0x40016800 )
#endif


/* Helper macros */

#define LTDC_RCC_PATTERN        ( 0x1 << 26u )
#define LTDC_RESET()              REG_TGL(RCC->APB2RSTR,  LTDC_RCC_PATTERN)
#define LTDC_ENABLE()             REG_SET(RCC->APB2ENR,   LTDC_RCC_PATTERN)
#define LTDC_DISABLE()            REG_CLR(RCC->APB2ENR,   LTDC_RCC_PATTERN)
#define LTDC_LP_ENABLE()          REG_SET(RCC->APB2LPENR, LTDC_RCC_PATTERN)
#define LTDC_LP_DISABLE()         REG_CLR(RCC->APB2LPENR, LTDC_RCC_PATTERN)


/*  TIM - timer
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_tim_t
 *  \brief  Representation of Timer register.
 *
 *  Described in reference manual p.507ff.
 */
typedef struct {
    volatile uint32_t CR1;      /**< Configuration register 1. */
    volatile uint32_t CR2;      /**< Configuration register 2. */
    volatile uint32_t SMCR;     /**< Slave mode control register. */
    volatile uint32_t DIER;     /**< DMA/interrupt enable register. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t EGR;      /**< Event generation register. */
    volatile uint32_t CCMR1;    /**< Capture/compare mode register 1. */
    volatile uint32_t CCMR2;    /**< Capture/compare mode register 2. */
    volatile uint32_t CCER;     /**< Capture/compare enable register. */
    volatile uint32_t CNT;      /**< Count register. */
    volatile uint32_t PSC;      /**< Prescaler register. */
    volatile uint32_t ARR;      /**< Auto reload register. */
    volatile uint32_t RCR;      /**< Repetition counter register. */
    volatile uint32_t CCR1;     /**< Capture/compare register 1. */
    volatile uint32_t CCR2;     /**< Capture/compare register 2. */
    volatile uint32_t CCR3;     /**< Capture/compare register 3. */
    volatile uint32_t CCR4;     /**< Capture/compare register 4. */
    volatile uint32_t BDTR;     /**< Break and dead-time register. */
    volatile uint32_t DCR;      /**< DMA control register. */
    volatile uint32_t DMAR;     /**< DMA address for full transfer. */
    volatile uint32_t OR;       /**< Option register. */
} reg_tim_t;


/* Register macros */


#ifdef TESTING
    extern reg_tim_t tim1_testing;
    extern reg_tim_t tim2_testing;
    extern reg_tim_t tim3_testing;
    extern reg_tim_t tim4_testing;
    extern reg_tim_t tim5_testing;
    extern reg_tim_t tim6_testing;
    extern reg_tim_t tim7_testing;
    extern reg_tim_t tim8_testing;
    extern reg_tim_t tim9_testing;
    extern reg_tim_t tim10_testing;
    extern reg_tim_t tim11_testing;
    extern reg_tim_t tim12_testing;
    extern reg_tim_t tim13_testing;
    extern reg_tim_t tim14_testing;
    #define TIM1                    ( (reg_tim_t *) &tim1_testing )
    #define TIM2                    ( (reg_tim_t *) &tim2_testing )
    #define TIM3                    ( (reg_tim_t *) &tim3_testing )
    #define TIM4                    ( (reg_tim_t *) &tim4_testing )
    #define TIM5                    ( (reg_tim_t *) &tim5_testing )
    #define TIM6                    ( (reg_tim_t *) &tim6_testing )
    #define TIM7                    ( (reg_tim_t *) &tim7_testing )
    #define TIM8                    ( (reg_tim_t *) &tim8_testing )
    #define TIM9                    ( (reg_tim_t *) &tim9_testing )
    #define TIM10                   ( (reg_tim_t *) &tim10_testing )
    #define TIM11                   ( (reg_tim_t *) &tim11_testing )
    #define TIM12                   ( (reg_tim_t *) &tim12_testing )
    #define TIM13                   ( (reg_tim_t *) &tim13_testing )
    #define TIM14                   ( (reg_tim_t *) &tim14_testing )
#else
    #define TIM1                    ( (reg_tim_t *) 0x40010000 )
    #define TIM2                    ( (reg_tim_t *) 0x40000000 )
    #define TIM3                    ( (reg_tim_t *) 0x40000400 )
    #define TIM4                    ( (reg_tim_t *) 0x40000800 )
    #define TIM5                    ( (reg_tim_t *) 0x40000c00 )
    #define TIM6                    ( (reg_tim_t *) 0x40001000 )
    #define TIM7                    ( (reg_tim_t *) 0x40001400 )
    #define TIM8                    ( (reg_tim_t *) 0x40010400 )
    #define TIM9                    ( (reg_tim_t *) 0x40014000 )
    #define TIM10                   ( (reg_tim_t *) 0x40014400 )
    #define TIM11                   ( (reg_tim_t *) 0x40014800 )
    #define TIM12                   ( (reg_tim_t *) 0x40001800 )
    #define TIM13                   ( (reg_tim_t *) 0x40001c00 )
    #define TIM14                   ( (reg_tim_t *) 0x40002000 )
#endif


/* Helper macros */

#define TIM1_RCC_PATTERN        ( 0x1 << 0u )
#define TIM1_RESET()            REG_TGL(RCC->APB2RSTR,  TIM1_RCC_PATTERN)
#define TIM1_ENABLE()           REG_SET(RCC->APB2ENR,   TIM1_RCC_PATTERN)
#define TIM1_DISABLE()          REG_CLR(RCC->APB2ENR,   TIM1_RCC_PATTERN)
#define TIM1_LP_ENABLE()        REG_SET(RCC->APB2LPENR, TIM1_RCC_PATTERN)
#define TIM1_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, TIM1_RCC_PATTERN)

#define TIM2_RCC_PATTERN        ( 0x1 << 0u )
#define TIM2_RESET()            REG_TGL(RCC->APB1RSTR,  TIM2_RCC_PATTERN)
#define TIM2_ENABLE()           REG_SET(RCC->APB1ENR,   TIM2_RCC_PATTERN)
#define TIM2_DISABLE()          REG_CLR(RCC->APB1ENR,   TIM2_RCC_PATTERN)
#define TIM2_LP_ENABLE()        REG_SET(RCC->APB1LPENR, TIM2_RCC_PATTERN)
#define TIM2_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, TIM2_RCC_PATTERN)

#define TIM3_RCC_PATTERN        ( 0x1 << 1u )
#define TIM3_RESET()            REG_TGL(RCC->APB1RSTR,  TIM3_RCC_PATTERN)
#define TIM3_ENABLE()           REG_SET(RCC->APB1ENR,   TIM3_RCC_PATTERN)
#define TIM3_DISABLE()          REG_CLR(RCC->APB1ENR,   TIM3_RCC_PATTERN)
#define TIM3_LP_ENABLE()        REG_SET(RCC->APB1LPENR, TIM3_RCC_PATTERN)
#define TIM3_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, TIM3_RCC_PATTERN)
           
#define TIM4_RCC_PATTERN        ( 0x1 << 2u )
#define TIM4_RESET()            REG_TGL(RCC->APB1RSTR,  TIM4_RCC_PATTERN)
#define TIM4_ENABLE()           REG_SET(RCC->APB1ENR,   TIM4_RCC_PATTERN)
#define TIM4_DISABLE()          REG_CLR(RCC->APB1ENR,   TIM4_RCC_PATTERN)
#define TIM4_LP_ENABLE()        REG_SET(RCC->APB1LPENR, TIM4_RCC_PATTERN)
#define TIM4_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, TIM4_RCC_PATTERN)
           
#define TIM5_RCC_PATTERN        ( 0x1 << 3u )
#define TIM5_RESET()            REG_TGL(RCC->APB1RSTR,  TIM5_RCC_PATTERN)
#define TIM5_ENABLE()           REG_SET(RCC->APB1ENR,   TIM5_RCC_PATTERN)
#define TIM5_DISABLE()          REG_CLR(RCC->APB1ENR,   TIM5_RCC_PATTERN)
#define TIM5_LP_ENABLE()        REG_SET(RCC->APB1LPENR, TIM5_RCC_PATTERN)
#define TIM5_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, TIM5_RCC_PATTERN)
           
#define TIM6_RCC_PATTERN        ( 0x1 << 4u )
#define TIM6_RESET()            REG_TGL(RCC->APB1RSTR,  TIM6_RCC_PATTERN)
#define TIM6_ENABLE()           REG_SET(RCC->APB1ENR,   TIM6_RCC_PATTERN)
#define TIM6_DISABLE()          REG_CLR(RCC->APB1ENR,   TIM6_RCC_PATTERN)
#define TIM6_LP_ENABLE()        REG_SET(RCC->APB1LPENR, TIM6_RCC_PATTERN)
#define TIM6_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, TIM6_RCC_PATTERN)

#define TIM7_RCC_PATTERN        ( 0x1 << 5u )
#define TIM7_RESET()            REG_TGL(RCC->APB1RSTR,  TIM7_RCC_PATTERN)
#define TIM7_ENABLE()           REG_SET(RCC->APB1ENR,   TIM7_RCC_PATTERN)
#define TIM7_DISABLE()          REG_CLR(RCC->APB1ENR,   TIM7_RCC_PATTERN)
#define TIM7_LP_ENABLE()        REG_SET(RCC->APB1LPENR, TIM7_RCC_PATTERN)
#define TIM7_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, TIM7_RCC_PATTERN)

#define TIM8_RCC_PATTERN        ( 0x1 << 1u )
#define TIM8_RESET()            REG_TGL(RCC->APB2RSTR,  TIM8_RCC_PATTERN)
#define TIM8_ENABLE()           REG_SET(RCC->APB2ENR,   TIM8_RCC_PATTERN)
#define TIM8_DISABLE()          REG_CLR(RCC->APB2ENR,   TIM8_RCC_PATTERN)
#define TIM8_LP_ENABLE()        REG_SET(RCC->APB2LPENR, TIM8_RCC_PATTERN)
#define TIM8_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, TIM8_RCC_PATTERN)

#define TIM9_RCC_PATTERN        ( 0x1 << 16u )
#define TIM9_RESET()            REG_TGL(RCC->APB2RSTR,  TIM9_RCC_PATTERN)
#define TIM9_ENABLE()           REG_SET(RCC->APB2ENR,   TIM9_RCC_PATTERN)
#define TIM9_DISABLE()          REG_CLR(RCC->APB2ENR,   TIM9_RCC_PATTERN)
#define TIM9_LP_ENABLE()        REG_SET(RCC->APB2LPENR, TIM9_RCC_PATTERN)
#define TIM9_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, TIM9_RCC_PATTERN)

#define TIM10_RCC_PATTERN       ( 0x1 << 17u )
#define TIM10_RESET()           REG_TGL(RCC->APB2RSTR,  TIM10_RCC_PATTERN)
#define TIM10_ENABLE()          REG_SET(RCC->APB2ENR,   TIM10_RCC_PATTERN)
#define TIM10_DISABLE()         REG_CLR(RCC->APB2ENR,   TIM10_RCC_PATTERN)
#define TIM10_LP_ENABLE()       REG_SET(RCC->APB2LPENR, TIM10_RCC_PATTERN)
#define TIM10_LP_DISABLE()      REG_CLR(RCC->APB2LPENR, TIM10_RCC_PATTERN)

#define TIM11_RCC_PATTERN       ( 0x1 << 18u )
#define TIM11_RESET()           REG_TGL(RCC->APB2RSTR,  TIM11_RCC_PATTERN)
#define TIM11_ENABLE()          REG_SET(RCC->APB2ENR,   TIM11_RCC_PATTERN)
#define TIM11_DISABLE()         REG_CLR(RCC->APB2ENR,   TIM11_RCC_PATTERN)
#define TIM11_LP_ENABLE()       REG_SET(RCC->APB2LPENR, TIM11_RCC_PATTERN)
#define TIM11_LP_DISABLE()      REG_CLR(RCC->APB2LPENR, TIM11_RCC_PATTERN)

#define TIM12_RCC_PATTERN       ( 0x1 << 6u )
#define TIM12_RESET()           REG_TGL(RCC->APB1RSTR,  TIM12_RCC_PATTERN)
#define TIM12_ENABLE()          REG_SET(RCC->APB1ENR,   TIM12_RCC_PATTERN)
#define TIM12_DISABLE()         REG_CLR(RCC->APB1ENR,   TIM12_RCC_PATTERN)
#define TIM12_LP_ENABLE()       REG_SET(RCC->APB1LPENR, TIM12_RCC_PATTERN)
#define TIM12_LP_DISABLE()      REG_CLR(RCC->APB1LPENR, TIM12_RCC_PATTERN)
                            
#define TIM13_RCC_PATTERN       ( 0x1 << 7u )
#define TIM13_RESET()           REG_TGL(RCC->APB1RSTR,  TIM13_RCC_PATTERN)
#define TIM13_ENABLE()          REG_SET(RCC->APB1ENR,   TIM13_RCC_PATTERN)
#define TIM13_DISABLE()         REG_CLR(RCC->APB1ENR,   TIM13_RCC_PATTERN)
#define TIM13_LP_ENABLE()       REG_SET(RCC->APB1LPENR, TIM13_RCC_PATTERN)
#define TIM13_LP_DISABLE()      REG_CLR(RCC->APB1LPENR, TIM13_RCC_PATTERN)
                            
#define TIM14_RCC_PATTERN       ( 0x1 << 8u )
#define TIM14_RESET()           REG_TGL(RCC->APB1RSTR,  TIM14_RCC_PATTERN)
#define TIM14_ENABLE()          REG_SET(RCC->APB1ENR,   TIM14_RCC_PATTERN)
#define TIM14_DISABLE()         REG_CLR(RCC->APB1ENR,   TIM14_RCC_PATTERN)
#define TIM14_LP_ENABLE()       REG_SET(RCC->APB1LPENR, TIM14_RCC_PATTERN)
#define TIM14_LP_DISABLE()      REG_CLR(RCC->APB1LPENR, TIM14_RCC_PATTERN)


/*  IWDG - independent watchdog
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_iwdg_t
 *  \brief  Representation of IWDG register.
 *
 *  Described in reference manual p.689ff.
 */
typedef struct {
    volatile uint32_t KR;       /**< Key register. */
    volatile uint32_t PR;       /**< Prescaler register. */
    volatile uint32_t RLR;      /**< Reload register. */
    volatile uint32_t SR;       /**< Status register. */
} reg_iwdg_t;


/* Register macro */

#ifdef TESTING
    extern reg_iwdg_t iwdg_testing;
    #define IWDG                    ( (reg_iwdg_t *) &iwdg_testing )
#else
    #define IWDG                    ( (reg_iwdg_t *) 0x40003000 )
#endif


/*  WWDG - window watch dog
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_iwdg_t
 *  \brief  Representation of IWDG register.
 *
 *  Described in reference manual p.689ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t CFR;      /**< Configuration register. */
    volatile uint32_t SR;       /**< Status register. */
} reg_wwdg_t;


/* Register macro */

#ifdef TESTING
    extern reg_wwdg_t wwgd_testing;
    #define WWDG                    ( (reg_wwdg_t *) &wwgd_testing )
#else
    #define WWDG                    ( (reg_wwdg_t *) 0x40002c00 )
#endif


/* Helper macros */

#define WWDG_RCC_PATTERN        ( 0x1 << 11u )
#define WWDG_RESET()            REG_TGL(RCC->APB1RSTR,  WWDG_RCC_PATTERN)
#define WWDG_ENABLE()           REG_SET(RCC->APB1ENR,   WWDG_RCC_PATTERN)
#define WWDG_DISABLE()          REG_CLR(RCC->APB1ENR,   WWDG_RCC_PATTERN)
#define WWDG_LP_ENABLE()        REG_SET(RCC->APB1LPENR, WWDG_RCC_PATTERN)
#define WWDG_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, WWDG_RCC_PATTERN)


/*  CRYP - cryptographic processor
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_cryp_t
 *  \brief  Representation of CRYP register.
 *
 *  Described in reference manual p.704ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t DIN;      /**< Data input register. */
    volatile uint32_t DOUT;     /**< Data output register. */
    volatile uint32_t DMACR;    /**< DMA control register. */
    volatile uint32_t IMSCR;    /**< Interrupt mask set/clear register. */
    volatile uint32_t RISR;     /**< Raw interrupt status register. */
    volatile uint32_t MISR;     /**< Masked interrupt status register. */
    volatile uint32_t K0LR;     /**< Key (left) register 1. */
    volatile uint32_t K0RR;     /**< Key (right) register 1. */
    volatile uint32_t K1LR;     /**< Key (left) register 2. */
    volatile uint32_t K1RR;     /**< Key (right) register 2. */
    volatile uint32_t K2LR;     /**< Key (left) register 3. */
    volatile uint32_t K2RR;     /**< Key (right) register 3. */
    volatile uint32_t K3LR;     /**< Key (left) register 4. */
    volatile uint32_t K3RR;     /**< Key (right) register 4. */
    volatile uint32_t IV0LR;    /**< Initialisation vector (left) reg. 1. */
    volatile uint32_t IV0RR;    /**< Initialisation vector (right) reg. 1. */
    volatile uint32_t IV1LR;    /**< Initialisation vector (left) reg. 2. */
    volatile uint32_t IV1RR;    /**< Initialisation vector (right) reg. 2. */
    volatile uint32_t CSGCMCCM0R;       /**< Context swap register 1. */
    volatile uint32_t CSGCMCCM1R;       /**< Context swap register 2. */
    volatile uint32_t CSGCMCCM2R;       /**< Context swap register 3. */
    volatile uint32_t CSGCMCCM3R;       /**< Context swap register 4. */
    volatile uint32_t CSGCMCCM4R;       /**< Context swap register 5. */
    volatile uint32_t CSGCMCCM5R;       /**< Context swap register 6. */
    volatile uint32_t CSGCMCCM6R;       /**< Context swap register 7. */
    volatile uint32_t CSGCMCCM7R;       /**< Context swap register 8. */
    volatile uint32_t CSGCM0R;          /**< Context swap register 1. */
    volatile uint32_t CSGCM1R;          /**< Context swap register 2. */
    volatile uint32_t CSGCM2R;          /**< Context swap register 3. */
    volatile uint32_t CSGCM3R;          /**< Context swap register 4. */
    volatile uint32_t CSGCM4R;          /**< Context swap register 5. */
    volatile uint32_t CSGCM5R;          /**< Context swap register 6. */
    volatile uint32_t CSGCM6R;          /**< Context swap register 7. */
    volatile uint32_t CSGCM7R;          /**< Context swap register 8. */
} reg_cryp_t;	


/* Register macro */

#ifdef TESTING
    extern reg_cryp_t cryp_testing;
    #define CRYP                    ( (reg_cryp_t *) &cryp_testing )
#else
    #define CRYP                    ( (reg_cryp_t *) 0x50060000 )
#endif


/* Helper macros */

#define CRYP_RCC_PATTERN        ( 0x1 << 4u )
#define CRYP_RESET()            REG_TGL(RCC->AHB2RSTR,  CRYP_RCC_PATTERN)
#define CRYP_ENABLE()           REG_SET(RCC->AHB2ENR,   CRYP_RCC_PATTERN)
#define CRYP_DISABLE()          REG_CLR(RCC->AHB2ENR,   CRYP_RCC_PATTERN)
#define CRYP_LP_ENABLE()        REG_SET(RCC->AHB2LPENR, CRYP_RCC_PATTERN)
#define CRYP_LP_DISABLE()       REG_CLR(RCC->AHB2LPENR, CRYP_RCC_PATTERN)


/*  RNG - random number generator
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_rng_t
 *  \brief  Representation of RNG register.
 *
 *  Described in reference manual p.750ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t DR;       /**< Data register. */
} reg_rng_t;	


/* Register macro */

#ifdef TESTING
    extern reg_rng_t rng_testing;
    #define RNG                     ( (reg_rng_t *) &rng_testing )
#else
    #define RNG                     ( (reg_rng_t *) 0x50060800 )
#endif


/* Helper macros */

#define RNG_RCC_PATTERN         ( 0x1 << 6u )
#define RNG_RESET()             REG_TGL(RCC->AHB2RSTR,  RNG_RCC_PATTERN)
#define RNG_ENABLE()            REG_SET(RCC->AHB2ENR,   RNG_RCC_PATTERN)
#define RNG_DISABLE()           REG_CLR(RCC->AHB2ENR,   RNG_RCC_PATTERN)
#define RNG_LP_ENABLE()         REG_SET(RCC->AHB2LPENR, RNG_RCC_PATTERN)
#define RNG_LP_DISABLE()        REG_CLR(RCC->AHB2LPENR, RNG_RCC_PATTERN)


/*  HASH - hashxxx
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_hash_t
 *  \brief  Representation of HASH register.
 *
 *  Described in reference manual p.755ff.
 */
typedef struct {
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t DIN;      /**< Data input register. */
    volatile uint32_t STR;      /**< Start register. */
//    volatile uint32_t HR[5];
    uint32_t RESERVED1[5];
    volatile uint32_t IMR;      /**< Interrupt enable register. */
    volatile uint32_t SR;       /**< Status register. */
    uint32_t RESERVED2[48];
    volatile uint32_t CSR[54];  /**< Context swap registers. */
    uint32_t RESERVED3[80];
    volatile uint32_t HR0;      /**< Hash digest register 1. */
    volatile uint32_t HR1;      /**< Hash digest register 2. */
    volatile uint32_t HR2;      /**< Hash digest register 3. */
    volatile uint32_t HR3;      /**< Hash digest register 4. */
    volatile uint32_t HR4;      /**< Hash digest register 5. */
    volatile uint32_t HR5;      /**< Hash digest register 6. */
    volatile uint32_t HR6;      /**< Hash digest register 7. */
    volatile uint32_t HR7;      /**< Hash digest register 8. */
} reg_hash_t;	


/* Register macro */

#ifdef TESTING
    extern reg_hash_t hash_testing;
    #define HASH                     ( (reg_hash_t *) &hash_testing )
#else
    #define HASH                    ( (reg_hash_t *) 0x50060400 )
#endif


/* Helper macros */

#define HASH_RCC_PATTERN        ( 0x1 << 5u )
#define HASH_RESET()            REG_TGL(RCC->AHB2RSTR,  HASH_RCC_PATTERN)
#define HASH_ENABLE()           REG_SET(RCC->AHB2ENR,   HASH_RCC_PATTERN)
#define HASH_DISABLE()          REG_CLR(RCC->AHB2ENR,   HASH_RCC_PATTERN)
#define HASH_LP_ENABLE()        REG_SET(RCC->AHB2LPENR, HASH_RCC_PATTERN)
#define HASH_LP_DISABLE()       REG_CLR(RCC->AHB2LPENR, HASH_RCC_PATTERN)


/*  RTC - real time clock
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_rtc_t
 *  \brief  Representation of RTC register.
 *
 *  Described in reference manual p.778ff.
 */
typedef struct {
    volatile uint32_t TR;       /**< Time register. */
    volatile uint32_t DR;       /**< Date register. */
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t ISR;      /**< Initialization/status register. */
    volatile uint32_t PRER;     /**< Prescaler register. */
    volatile uint32_t WUTR;     /**< Wake-up timer register. */
    volatile uint32_t CALIBR;   /**< Calibration register. */
    volatile uint32_t ALRMAR;   /**< Alarm A register. */
    volatile uint32_t ALRMBR;   /**< Alarm B register. */
    volatile uint32_t WPR;      /**< Write protection register. */
    volatile uint32_t SSR;      /**< Sub-second register. */
    volatile uint32_t SHIFTR;   /**< Shift control register. */
    volatile uint32_t TSTR;     /**< Timestamp time register. */
    volatile uint32_t TSDR;     /**< Timestamp date register. */
    volatile uint32_t TSSSR;    /**< Timestamp sub-second register. */
    volatile uint32_t CALR;     /**< Calibration register. */
    volatile uint32_t TAFCR;    /**< Tamper and AF configuration register. */
    volatile uint32_t ALRMASSR; /**< Alarm A sub-second register. */
    volatile uint32_t ALRMBSSR; /**< Alarm B sub-second register. */
    uint32_t RESERVED;
    volatile uint32_t BKP0R;    /**< Backup register 1. */
    volatile uint32_t BKP1R;    /**< Backup register 2. */
    volatile uint32_t BKP2R;    /**< Backup register 3. */
    volatile uint32_t BKP3R;    /**< Backup register 4. */
    volatile uint32_t BKP4R;    /**< Backup register 5. */
    volatile uint32_t BKP5R;    /**< Backup register 6. */
    volatile uint32_t BKP6R;    /**< Backup register 7. */
    volatile uint32_t BKP7R;    /**< Backup register 8. */
    volatile uint32_t BKP8R;    /**< Backup register 9. */
    volatile uint32_t BKP9R;    /**< Backup register 10. */
    volatile uint32_t BKP10R;   /**< Backup register 11. */
    volatile uint32_t BKP11R;   /**< Backup register 12. */
    volatile uint32_t BKP12R;   /**< Backup register 13. */
    volatile uint32_t BKP13R;   /**< Backup register 14. */
    volatile uint32_t BKP14R;   /**< Backup register 15. */
    volatile uint32_t BKP15R;   /**< Backup register 16. */
    volatile uint32_t BKP16R;   /**< Backup register 17. */
    volatile uint32_t BKP17R;   /**< Backup register 18. */
    volatile uint32_t BKP18R;   /**< Backup register 19. */
    volatile uint32_t BKP19R;   /**< Backup register 20. */
} reg_rtc_t;	


/* Register macro */

#ifdef TESTING
    extern reg_rtc_t rtc_testing;
    #define RTC                     ( (reg_rtc_t *) &hash_testing )
#else
    #define RTC                     ( (reg_rtc_t *) 0x40002800 )
#endif


/*  I2C - inter-integrated circuit interface
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_i2c_t
 *  \brief  Representation of I2C register.
 *
 *  Described in reference manual p.817ff.
 */
typedef struct {
    volatile uint32_t CR1;      /**< Control register 1. */
    volatile uint32_t CR2;      /**< Control register 2. */
    volatile uint32_t OAR1;     /**< Own address register 1. */
    volatile uint32_t OAR2;     /**< Own address register 2. */
    volatile uint32_t DR;       /**< Data register. */
    volatile uint32_t SR1;      /**< Status register 1. */
    volatile uint32_t SR2;      /**< Status register 2. */
    volatile uint32_t CCR;      /**< Clock control register. */
    volatile uint32_t TRISE;    /**< TRISE register. */
    volatile uint32_t FLTR;     /**< FLTR register. */
} reg_i2c_t;


/* Register macros */

#ifdef TESTING
    extern reg_i2c_t i2c1_testing;
    extern reg_i2c_t i2c2_testing;
    extern reg_i2c_t i2c3_testing;
    #define I2C1                    ( (reg_i2c_t *) &i2c1_testing )
    #define I2C2                    ( (reg_i2c_t *) &i2c2_testing )
    #define I2C3                    ( (reg_i2c_t *) &i2c3_testing )
#else
    #define I2C1                    ( (reg_i2c_t *) 0x40005400 )
    #define I2C2                    ( (reg_i2c_t *) 0x40005800 )
    #define I2C3                    ( (reg_i2c_t *) 0x40005c00 )
#endif


/* Helper macros */

#define I2C1_RCC_PATTERN        ( 0x1 << 21u )
#define I2C1_RESET()            REG_TGL(RCC->APB1RSTR,  I2C1_RCC_PATTERN)
#define I2C1_ENABLE()           REG_SET(RCC->APB1ENR,   I2C1_RCC_PATTERN)
#define I2C1_DISABLE()          REG_CLR(RCC->APB1ENR,   I2C1_RCC_PATTERN)
#define I2C1_LP_ENABLE()        REG_SET(RCC->APB1LPENR, I2C1_RCC_PATTERN)
#define I2C1_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, I2C1_RCC_PATTERN)

#define I2C2_RCC_PATTERN        ( 0x1 << 22u )
#define I2C2_RESET()            REG_TGL(RCC->APB1RSTR,  I2C2_RCC_PATTERN)
#define I2C2_ENABLE()           REG_SET(RCC->APB1ENR,   I2C2_RCC_PATTERN)
#define I2C2_DISABLE()          REG_CLR(RCC->APB1ENR,   I2C2_RCC_PATTERN)
#define I2C2_LP_ENABLE()        REG_SET(RCC->APB1LPENR, I2C2_RCC_PATTERN)
#define I2C2_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, I2C2_RCC_PATTERN)

#define I2C3_RCC_PATTERN        ( 0x1 << 23u )                 
#define I2C3_RESET()            REG_TGL(RCC->APB1RSTR,  I2C3_RCC_PATTERN)
#define I2C3_ENABLE()           REG_SET(RCC->APB1ENR,   I2C3_RCC_PATTERN)
#define I2C3_DISABLE()          REG_CLR(RCC->APB1ENR,   I2C3_RCC_PATTERN)
#define I2C3_LP_ENABLE()        REG_SET(RCC->APB1LPENR, I2C3_RCC_PATTERN)
#define I2C3_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, I2C3_RCC_PATTERN)


/*  SPI - serial peripheral interface
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_spi_t
 *  \brief  Representation of SPI register.
 *
 *  Described in reference manual p.852ff.
 */
typedef struct {
    volatile uint32_t CR1;      /**< Control register 1. */
    volatile uint32_t CR2;      /**< Control register 2. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t DR;       /**< Data register. */
    volatile uint32_t CRCPR;    /**< CRC polynomial register. */
    volatile uint32_t RXCRCR;   /**< RX CRC register. */
    volatile uint32_t TXCRCR;   /**< TX CRC register. */
    volatile uint32_t I2SCFGR;  /**< Configuration register. */
    volatile uint32_t I2SPR;    /**< Prescaler register. */
} reg_spi_t;


/* Register macros */

#ifdef TESTING
    extern reg_spi_t spi1_testing;
    extern reg_spi_t spi2_testing;
    extern reg_spi_t spi3_testing;
    extern reg_spi_t spi4_testing;
    extern reg_spi_t spi5_testing;
    extern reg_spi_t spi6_testing;
    #define SPI1                    ( (reg_spi_t *) &spi1_testing )
    #define SPI2                    ( (reg_spi_t *) &spi2_testing )
    #define SPI3                    ( (reg_spi_t *) &spi3_testing )
    #define SPI4                    ( (reg_spi_t *) &spi4_testing )
    #define SPI5                    ( (reg_spi_t *) &spi5_testing )
    #define SPI6                    ( (reg_spi_t *) &spi6_testing )
#else
    #define SPI1                    ( (reg_spi_t *) 0x40013000 )
    #define SPI2                    ( (reg_spi_t *) 0x40003800 )
    #define SPI3                    ( (reg_spi_t *) 0x40003c00 )
    #define SPI4                    ( (reg_spi_t *) 0x40013400 )
    #define SPI5                    ( (reg_spi_t *) 0x40015000 )
    #define SPI6                    ( (reg_spi_t *) 0x40015400 )
#endif


/* Helper macros */

#define SPI1_RCC_PATTERN        ( 0x1 << 12u )
#define SPI1_RESET()            REG_TGL(RCC->APB2RSTR,  SPI1_RCC_PATTERN)
#define SPI1_ENABLE()           REG_SET(RCC->APB2ENR,   SPI1_RCC_PATTERN)
#define SPI1_DISABLE()          REG_CLR(RCC->APB2ENR,   SPI1_RCC_PATTERN)
#define SPI1_LP_ENABLE()        REG_SET(RCC->APB2LPENR, SPI1_RCC_PATTERN)
#define SPI1_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, SPI1_RCC_PATTERN)

#define SPI2_RCC_PATTERN        ( 0x1 << 14u )
#define SPI2_RESET()            REG_TGL(RCC->APB1RSTR,  SPI2_RCC_PATTERN)
#define SPI2_ENABLE()           REG_SET(RCC->APB1ENR,   SPI2_RCC_PATTERN)
#define SPI2_DISABLE()          REG_CLR(RCC->APB1ENR,   SPI2_RCC_PATTERN)
#define SPI2_LP_ENABLE()        REG_SET(RCC->APB1LPENR, SPI2_RCC_PATTERN)
#define SPI2_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, SPI2_RCC_PATTERN)

#define SPI3_RCC_PATTERN        ( 0x1 << 15u )
#define SPI3_RESET()            REG_TGL(RCC->APB1RSTR,  SPI3_RCC_PATTERN)
#define SPI3_ENABLE()           REG_SET(RCC->APB1ENR,   SPI3_RCC_PATTERN)
#define SPI3_DISABLE()          REG_CLR(RCC->APB1ENR,   SPI3_RCC_PATTERN)
#define SPI3_LP_ENABLE()        REG_SET(RCC->APB1LPENR, SPI3_RCC_PATTERN)
#define SPI3_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, SPI3_RCC_PATTERN)

#define SPI4_RCC_PATTERN        ( 0x1 << 13u )
#define SPI4_RESET()            REG_TGL(RCC->APB2RSTR,  SPI4_RCC_PATTERN)
#define SPI4_ENABLE()           REG_SET(RCC->APB2ENR,   SPI4_RCC_PATTERN)
#define SPI4_DISABLE()          REG_CLR(RCC->APB2ENR,   SPI4_RCC_PATTERN)
#define SPI4_LP_ENABLE()        REG_SET(RCC->APB2LPENR, SPI4_RCC_PATTERN)
#define SPI4_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, SPI4_RCC_PATTERN)

#define SPI5_RCC_PATTERN        ( 0x1 << 20u )
#define SPI5_RESET()            REG_TGL(RCC->APB2RSTR,  SPI5_RCC_PATTERN)
#define SPI5_ENABLE()           REG_SET(RCC->APB2ENR,   SPI5_RCC_PATTERN)
#define SPI5_DISABLE()          REG_CLR(RCC->APB2ENR,   SPI5_RCC_PATTERN)
#define SPI5_LP_ENABLE()        REG_SET(RCC->APB2LPENR, SPI5_RCC_PATTERN)
#define SPI5_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, SPI5_RCC_PATTERN)

#define SPI6_RCC_PATTERN        ( 0x1 << 21u )
#define SPI6_RESET()            REG_TGL(RCC->APB2RSTR,  SPI6_RCC_PATTERN)
#define SPI6_ENABLE()           REG_SET(RCC->APB2ENR,   SPI6_RCC_PATTERN)
#define SPI6_DISABLE()          REG_CLR(RCC->APB2ENR,   SPI6_RCC_PATTERN)
#define SPI6_LP_ENABLE()        REG_SET(RCC->APB2LPENR, SPI6_RCC_PATTERN)
#define SPI6_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, SPI6_RCC_PATTERN)


/*  SAI - sxxx audio interface
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_sai_block_t
 *  \brief  Serial audio interface subblock register.
 *
 *  Described in reference manual p.912ff.
 */
typedef struct {
    volatile uint32_t CR1;      /**< Configuration register 1. */
    volatile uint32_t CR2;      /**< Configuration register 2. */
    volatile uint32_t FRCR;     /**< Frame configuration register. */
    volatile uint32_t SLOTR;    /**< Slot register. */
    volatile uint32_t IM;       /**< Interrupt mask register. */
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t CLRFR;    /**< Clear flag register. */
    volatile uint32_t DR;       /**< Data register. */
} reg_sai_block_t;


/**
 *  \struct reg_sai_t
 *  \brief  Serial audio interface register.
 *
 *  Described in reference manual p.912ff.
 */
typedef struct {
    volatile reg_sai_block_t BLOCK1;    /**< Audio subblock 1. */
    volatile reg_sai_block_t BLOCK2;    /**< Audio subblock 2. */
} reg_sai_t;


/* Register macros */

#ifdef TESTING
    extern reg_sai_t sat1_testing;
    #define SAI1                    ( (reg_sai_t *) &sat1_testing )
#else
    #define SAI1                    ( (reg_sai_t *) 0x40015800 )
#endif

/* Helper macros */

#define SAI1_RCC_PATTERN        ( 0x1 << 22u )
#define SAI1_RESET()            REG_TGL(RCC->APB2RSTR,  SAI1_RCC_PATTERN)
#define SAI1_ENABLE()           REG_SET(RCC->APB2ENR,   SAI1_RCC_PATTERN)
#define SAI1_DISABLE()          REG_CLR(RCC->APB2ENR,   SAI1_RCC_PATTERN)
#define SAI1_LP_ENABLE()        REG_SET(RCC->APB2LPENR, SAI1_RCC_PATTERN)
#define SAI1_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, SAI1_RCC_PATTERN)


/*  USART - universal synchronous/asynchronous receiver transmitter
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_usart_t
 *  \brief  Universal sync./async. receiver transmiter register.
 *
 *  Described in reference manual p.951ff.
 */
typedef struct {
    volatile uint32_t SR;       /**< Status register. */
    volatile uint32_t DR;       /**< Data register. */
    volatile uint32_t BRR;      /**< Baud rate register. */
    volatile uint32_t CR1;      /**< Configuration register 1. */
    volatile uint32_t CR2;      /**< Configuration register 2. */
    volatile uint32_t CR3;      /**< Configuration register 3. */
    volatile uint32_t GPTR;     /**< Guard time and prescaler. */
} reg_usart_t;


/* Register macros */

#ifdef TESTING
    extern reg_usart_t usart1_testing;
    extern reg_usart_t usart2_testing;
    extern reg_usart_t usart3_testing;
    extern reg_usart_t uart4_testing;
    extern reg_usart_t uart5_testing;
    extern reg_usart_t usart6_testing;
    extern reg_usart_t usart7_testing;
    extern reg_usart_t usart8_testing;
    #define USART1                  ( (reg_usart_t *) &usart1_testing )
    #define USART2                  ( (reg_usart_t *) &usart2_testing )
    #define USART3                  ( (reg_usart_t *) &usart3_testing )
    #define UART4                   ( (reg_usart_t *) &uart4_testing )
    #define UART5                   ( (reg_usart_t *) &uart5_testing )
    #define USART6                  ( (reg_usart_t *) &usart6_testing )
    #define USART7                  ( (reg_usart_t *) &usart7_testing )
    #define USART8                  ( (reg_usart_t *) &usart8_testing )
#else
    #define USART1                  ( (reg_usart_t *) 0x40011000 )
    #define USART2                  ( (reg_usart_t *) 0x40004400 )
    #define USART3                  ( (reg_usart_t *) 0x40004800 )
    #define UART4                   ( (reg_usart_t *) 0x40004c00 )
    #define UART5                   ( (reg_usart_t *) 0x40005000 )
    #define USART6                  ( (reg_usart_t *) 0x40011400 )
    #define USART7                  ( (reg_usart_t *) 0x40007800 )
    #define USART8                  ( (reg_usart_t *) 0x40007c00 )
#endif


/* Helper macros */

#define USART1_RCC_PATTERN      ( 0x1 << 4u )
#define USART1_RESET()          REG_TGL(RCC->APB2RSTR,  USART1_RCC_PATTERN)
#define USART1_ENABLE()         REG_SET(RCC->APB2ENR,   USART1_RCC_PATTERN)
#define USART1_DISABLE()        REG_CLR(RCC->APB2ENR,   USART1_RCC_PATTERN)
#define USART1_LP_ENABLE()      REG_SET(RCC->APB2LPENR, USART1_RCC_PATTERN)
#define USART1_LP_DISABLE()     REG_CLR(RCC->APB2LPENR, USART1_RCC_PATTERN)

#define USART2_RCC_PATTERN      ( 0x1 << 17u )
#define USART2_RESET()          REG_TGL(RCC->APB1RSTR,  USART2_RCC_PATTERN)
#define USART2_ENABLE()         REG_SET(RCC->APB1ENR,   USART2_RCC_PATTERN)
#define USART2_DISABLE()        REG_CLR(RCC->APB1ENR,   USART2_RCC_PATTERN)
#define USART2_LP_ENABLE()      REG_SET(RCC->APB1LPENR, USART2_RCC_PATTERN)
#define USART2_LP_DISABLE()     REG_CLR(RCC->APB1LPENR, USART2_RCC_PATTERN)

#define USART3_RCC_PATTERN      ( 0x1 << 18u )
#define USART3_RESET()          REG_TGL(RCC->APB1RSTR,  USART3_RCC_PATTERN)
#define USART3_ENABLE()         REG_SET(RCC->APB1ENR,   USART3_RCC_PATTERN)
#define USART3_DISABLE()        REG_CLR(RCC->APB1ENR,   USART3_RCC_PATTERN)
#define USART3_LP_ENABLE()      REG_SET(RCC->APB1LPENR, USART3_RCC_PATTERN)
#define USART3_LP_DISABLE()     REG_CLR(RCC->APB1LPENR, USART3_RCC_PATTERN)

#define UART4_RCC_PATTERN       ( 0x1 << 19u )
#define UART4_RESET()           REG_TGL(RCC->APB1RSTR,  UART4_RCC_PATTERN)
#define UART4_ENABLE()          REG_SET(RCC->APB1ENR,   UART4_RCC_PATTERN)
#define UART4_DISABLE()         REG_CLR(RCC->APB1ENR,   UART4_RCC_PATTERN)
#define UART4_LP_ENABLE()       REG_SET(RCC->APB1LPENR, UART4_RCC_PATTERN)
#define UART4_LP_DISABLE()      REG_CLR(RCC->APB1LPENR, UART4_RCC_PATTERN)

#define UART5_RCC_PATTERN       ( 0x1 << 20u )
#define UART5_RESET()           REG_TGL(RCC->APB1RSTR,  UART5_RCC_PATTERN)
#define UART5_ENABLE()          REG_SET(RCC->APB1ENR,   UART5_RCC_PATTERN)
#define UART5_DISABLE()         REG_CLR(RCC->APB1ENR,   UART5_RCC_PATTERN)
#define UART5_LP_ENABLE()       REG_SET(RCC->APB1LPENR, UART5_RCC_PATTERN)
#define UART5_LP_DISABLE()      REG_CLR(RCC->APB1LPENR, UART5_RCC_PATTERN)

#define USART6_RCC_PATTERN      ( 0x1 << 5u )
#define USART6_RESET()          REG_TGL(RCC->APB2RSTR,  USART6_RCC_PATTERN)
#define USART6_ENABLE()         REG_SET(RCC->APB2ENR,   USART6_RCC_PATTERN)
#define USART6_DISABLE()        REG_CLR(RCC->APB2ENR,   USART6_RCC_PATTERN)
#define USART6_LP_ENABLE()      REG_SET(RCC->APB2LPENR, USART6_RCC_PATTERN)
#define USART6_LP_DISABLE()     REG_CLR(RCC->APB2LPENR, USART6_RCC_PATTERN)

#define USART7_RCC_PATTERN      ( 0x1 << 30u )
#define USART7_RESET()          REG_TGL(RCC->APB1RSTR,  USART7_RCC_PATTERN)
#define USART7_ENABLE()         REG_SET(RCC->APB1ENR,   USART7_RCC_PATTERN)
#define USART7_DISABLE()        REG_CLR(RCC->APB1ENR,   USART7_RCC_PATTERN)
#define USART7_LP_ENABLE()      REG_SET(RCC->APB1LPENR, USART7_RCC_PATTERN)
#define USART7_LP_DISABLE()     REG_CLR(RCC->APB1LPENR, USART7_RCC_PATTERN)

#define USART8_RCC_PATTERN      ( 0x1 << 31u )
#define USART8_RESET()          REG_TGL(RCC->APB1RSTR,  USART8_RCC_PATTERN)
#define USART8_ENABLE()         REG_SET(RCC->APB1ENR,   USART8_RCC_PATTERN)
#define USART8_DISABLE()        REG_CLR(RCC->APB1ENR,   USART8_RCC_PATTERN)
#define USART8_LP_ENABLE()      REG_SET(RCC->APB1LPENR, USART8_RCC_PATTERN)
#define USART8_LP_DISABLE()     REG_CLR(RCC->APB1LPENR, USART8_RCC_PATTERN)


/*  SDIO - secure digital input output
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_sdio_t
 *  \brief  Representation of SDIO register.
 *
 *  Described in reference manual p.1003ff.
 */
typedef struct {
    volatile uint32_t POWER;    /**< Power control register. */
    volatile uint32_t CLKCR;    /**< Clock control register. */
    volatile uint32_t ARG;      /**< Argument register. */
    volatile uint32_t CMD;      /**< Command register. */
    volatile uint32_t RESPCMD;  /**< Command response register. */
    volatile uint32_t RESP1;    /**< Response register 1. */
    volatile uint32_t RESP2;    /**< Response register 2. */
    volatile uint32_t RESP3;    /**< Response register 3. */
    volatile uint32_t RESP4;    /**< Response register 4. */
    volatile uint32_t DTIMER;   /**< Data timer register. */
    volatile uint32_t DLEN;     /**< Data length register. */
    volatile uint32_t DCTRL;    /**< Data control register. */
    volatile uint32_t DCOUNT;   /**< Data counter register. */
    volatile uint32_t STA;      /**< Status register. */
    volatile uint32_t ICR;      /**< Interrupt clear register. */
    volatile uint32_t MASK;     /**< Mask register. */
    uint32_t RESERVED[2];
    volatile uint32_t FIFOCNT;  /**< FIFO count register. */
    volatile uint32_t FIFO;     /**< Data FIFO register. */
} reg_sdio_t;


/* Register macros */

#ifdef TESTING
    extern reg_sdio_t sdio_testing;
    #define SDIO                    ( (reg_sdio_t *) &sdio_testing )
#else
    #define SDIO                    ( (reg_sdio_t *) 0x40006400 )
#endif

/* Helper macros */

#define SDIO_RCC_PATTERN        ( 0x1 << 11u )
#define SDIO_RESET()            REG_TGL(RCC->APB2RSTR,  SDIO_RCC_PATTERN)
#define SDIO_ENABLE()           REG_SET(RCC->APB2ENR,   SDIO_RCC_PATTERN)
#define SDIO_DISABLE()          REG_CLR(RCC->APB2ENR,   SDIO_RCC_PATTERN)
#define SDIO_LP_ENABLE()        REG_SET(RCC->APB2LPENR, SDIO_RCC_PATTERN)
#define SDIO_LP_DISABLE()       REG_CLR(RCC->APB2LPENR, SDIO_RCC_PATTERN)


/*  CAN - cxxx
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_can_t
 *  \brief  Representation of CAN register.
 *
 *  Described in reference manual p.1059ff.
 */
typedef struct {
    volatile uint32_t MCR;          /** Master control register. */
    volatile uint32_t MSR;          /** Master status regiser. */
    volatile uint32_t TSR;          /** Transmit status register. */
    volatile uint32_t RF0R;         /** Receive FIFO0 register. */
    volatile uint32_t RF1R;         /** Receive FIFO1 register. */
    volatile uint32_t IER;          /** Interrupt enable register. */
    volatile uint32_t ESR;          /** Error status register. */
    volatile uint32_t BTR;          /** Bit timing register. */
    uint32_t RESERVED1[88];         
    volatile uint32_t TI0R;         /** TX mailbox identifier register 0. */
    volatile uint32_t TDT0R;        /** TX mailbox data length and
                                        time stamp register 0. */
    volatile uint32_t TDL0R;        /** TX mailbox data low register 0. */
    volatile uint32_t TDH0R;        /** TX mailbox data high register 0. */
    volatile uint32_t TI1R;         /** TX mailbox identifier register 1. */
    volatile uint32_t TDT1R;        /** TX mailbox data length and
                                        time stamp register 1. */
    volatile uint32_t TDL1R;        /** TX mailbox data low register 1. */
    volatile uint32_t TDH1R;        /** TX mailbox data high register 1. */
    volatile uint32_t TI2R;         /** TX mailbox identifier register 2. */
    volatile uint32_t TDT2R;        /** TX mailbox data length and
                                        time stamp register 2. */
    volatile uint32_t TDL2R;        /** TX mailbox data low register 2. */
    volatile uint32_t TDH2R;        /** TX mailbox data high register 2. */
    volatile uint32_t RI0R;         /** RX mailbox identifier register 0. */
    volatile uint32_t RDT0R;        /** RX mailbox data length and 
                                        time stamp register 0. */
    volatile uint32_t RDL0R;        /** RX mailbox data low register 0. */
    volatile uint32_t RDH0R;        /** RX mailbox data high register 0. */
    volatile uint32_t RI1R;         /** RX mailbox identifier register 1. */
    volatile uint32_t RDT1R;        /** RX mailbox data length and 
                                        time stamp register 1. */
    volatile uint32_t RDL1R;        /** RX mailbox data low register 1. */
    volatile uint32_t RDH1R;        /** RX mailbox data high register 1. */
    uint32_t RESERVED2[12];
    volatile uint32_t FMR;          /** Filter master register. */
    volatile uint32_t FM1R;         /** Filter mode register. */
    uint32_t RESERVED3;
    volatile uint32_t FS1R;         /** Filter scale register. */
    uint32_t RESERVED4;
    volatile uint32_t FFA1R;        /** Filter FIFO assignment register. */
    uint32_t RESERVED5;
    volatile uint32_t FA1R;         /** Filter activation register. */
    uint32_t RESERVED6[8];
    volatile uint32_t FR[28][2];    /** FR[x][1] Filter bank 1 register 1.
                                        FR[x][2] Filter bank 1 register 2. */
} reg_can_t;


/* Register macros */

#ifdef TESTING
    extern reg_can_t can1_testing;
    extern reg_can_t can2_testing;
    #define CAN1                    ( (reg_can_t *) &can1_testing )
    #define CAN2                    ( (reg_can_t *) &can2_testing )
#else
    #define CAN1                    ( (reg_can_t *) 0x40006400 )
    #define CAN2                    ( (reg_can_t *) 0x40006800 )
#endif


/* Helper macros */

#define CAN1_RCC_PATTERN        ( 0x1 << 25u )
#define CAN1_RESET()            REG_TGL(RCC->APB1RSTR,  CAN1_RCC_PATTERN)
#define CAN1_ENABLE()           REG_SET(RCC->APB1ENR,   CAN1_RCC_PATTERN)
#define CAN1_DISABLE()          REG_CLR(RCC->APB1ENR,   CAN1_RCC_PATTERN)
#define CAN1_LP_ENABLE()        REG_SET(RCC->APB1LPENR, CAN1_RCC_PATTERN)
#define CAN1_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, CAN1_RCC_PATTERN)

#define CAN2_RCC_PATTERN        ( 0x1 << 26u )
#define CAN2_RESET()            REG_TGL(RCC->APB1RSTR,  CAN2_RCC_PATTERN)
#define CAN2_ENABLE()           REG_SET(RCC->APB1ENR,   CAN2_RCC_PATTERN)
#define CAN2_DISABLE()          REG_CLR(RCC->APB1ENR,   CAN2_RCC_PATTERN)
#define CAN2_LP_ENABLE()        REG_SET(RCC->APB1LPENR, CAN2_RCC_PATTERN)
#define CAN2_LP_DISABLE()       REG_CLR(RCC->APB1LPENR, CAN2_RCC_PATTERN)


/*  FMC - flexible memory controller
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_fmc_sram_t
 *  \brief  Representation of SRAM register (Bank 1).
 */
typedef struct {
    volatile uint32_t BCR1;     /**< (Sub)Bank 1 control register. */
    volatile uint32_t BTR1;     /**< (Sub)Bank 1 timing registers. */
    volatile uint32_t BCR2;     /**< (Sub)Bank 2 control register. */
    volatile uint32_t BTR2;     /**< (Sub)Bank 2 timing registers. */
    volatile uint32_t BCR3;     /**< (Sub)Bank 3 control register. */
    volatile uint32_t BTR3;     /**< (Sub)Bank 3 timing registers. */
    volatile uint32_t BCR4;     /**< (Sub)Bank 4 control register. */
    volatile uint32_t BTR4;     /**< (Sub)Bank 4 timing registers. */
    uint32_t RESERVED1[57];
    volatile uint32_t BWTR1;    /**< (Sub)Bank 1 write timing registers. */
    uint32_t RESERVED2;
    volatile uint32_t BWTR2;    /**< (Sub)Bank 2 write timing registers. */
    uint32_t RESERVED3;
    volatile uint32_t BWTR3;    /**< (Sub)Bank 3 write timing registers. */
    uint32_t RESERVED4;
    volatile uint32_t BWTR4;    /**< (Sub)Bank 4 write timing registers. */
} reg_fmc_sram_t;
	

/**
 *  \struct reg_fmc_t
 *  \brief  Representation of FMC register.
 *
 *  Described in reference manual p.1580ff.
 */
typedef struct {
    reg_fmc_sram_t SRAM;        /**< SRAM registers (Bank 1). */
} reg_fmc_t;


/* Register macro */

#ifdef TESTING
    extern reg_fmc_t fmc_testing;
    #define FMC                     ( (reg_fmc_t *) &fmc_testing )
#else
    #define FMC                     ( (reg_fmc_t *) 0xa0000000 )
#endif


/* Helper macros */

#define FMC_RCC_PATTERN         ( 0x1 << 0u )
#define FMC_RESET()             REG_TGL(RCC->AHB3RSTR,  FMC_RCC_PATTERN)
#define FMC_ENABLE()            REG_SET(RCC->AHB3ENR,   FMC_RCC_PATTERN)
#define FMC_DISABLE()           REG_CLR(RCC->AHB3ENR,   FMC_RCC_PATTERN)
#define FMC_LP_ENABLE()         REG_SET(RCC->AHB3LPENR, FMC_RCC_PATTERN)
#define FMC_LP_DISABLE()        REG_CLR(RCC->AHB3LPENR, FMC_RCC_PATTERN)


/*  MPU - memory protection unit
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_mpu_t
 *  \brief  Representation of MPU register.
 *
 *  Described in programming manual p.178ff.
 */
typedef struct {
    volatile uint32_t TYPER;    /**< Type register. */
    volatile uint32_t CTRL;     /**< Control register. */
    volatile uint32_t RNR;      /**< Region number register. */
    volatile uint32_t RBAR;     /**< Region base address register. */
    volatile uint32_t RASR;     /**< Region sttribute and size register. */
} reg_mpu_t;


/* Register macro */

#ifdef TESTING
    extern reg_mpu_t mpu_testing;
    #define MPU                     ( (reg_mpu_t *) &mpu_testing )
#else
    #define MPU                     ( (reg_mpu_t *) 0xe000ed90 )
#endif


/*  SCB - system control block
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_scb_t
 *  \brief  Representation of SCB register.
 *
 *  Described in programming manual p.206ff.
 */
typedef struct {
    volatile uint32_t CPUID;    /**< CPU ID register. */
    volatile uint32_t ICSR;     /**< Interrupt control/state register. */
    volatile uint32_t VTOR;     /**< Vector table offset register. */
    volatile uint32_t AIRCR;    /**< Application irq/reset control register. */
    volatile uint32_t SCR;      /**< System control register. */
    volatile uint32_t CCR;      /**< Configuration control register. */
    volatile uint32_t SHPR1;    /**< System handler priority register 1. */
    volatile uint32_t SHPR2;    /**< System handler priority register 2. */
    volatile uint32_t SHPR3;    /**< System handler priority register 3. */
    volatile uint32_t SHCSR;    /**< System handler control/status register. */
    volatile uint32_t CFSR;     /**< Configurable fault status register.
                                     UFSR: Usage fault status register. 
                                     BFSR: Bus fault status register. 
                                     MMFSR: Memory managm. fault status reg. */
    volatile uint32_t HFSR;     /**< Hard fault status register. */
    uint32_t RESERVED;
    volatile uint32_t MMAR;     /**< Memory management fault addr. register. */
    volatile uint32_t BFAR;     /**< Bus fault address register. */
    volatile uint32_t AFSR;     /**< Auxiliary fault status register. */
} reg_scb_t;


/* Register macro */

#ifdef TESTING
    extern reg_scb_t scb_testing;
    #define SCB                     ( (reg_scb_t *) &scb_testing )
#else
    #define SCB                     ( (reg_scb_t *) 0xe000ed00 )
#endif


/*  STK - system tick timer
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_stk_t
 *  \brief  Representation of STK register.
 *
 *  Described in programming manual p.230ff.
 */
typedef struct {
    volatile uint32_t CTRL;     /**< Control and status register. */
    volatile uint32_t LOAD;     /**< Reload value register. */
    volatile uint32_t VAL;      /**< Current value register. */
    volatile uint32_t CALIB;    /**< Calibration value register. */
} reg_stk_t;


/* Register macro */

#ifdef TESTING
    extern reg_stk_t stk_testing;
    #define STK                     ( (reg_stk_t *) &stk_testing )
#else
    #define STK                     ( (reg_stk_t *) 0xe000e010 )
#endif


/*  FPU - floating point unit
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_fpu_t
 *  \brief  Representation of FPU register.
 *
 *  Described in programming manual p.236ff.
 */
typedef struct {
    volatile uint32_t CPACR;    /**< Coprocessor access control register. */
    uint32_t RESERVED[106];
    volatile uint32_t FPCCR;    /**< Floating point context control reg. */
    volatile uint32_t FPCAR;    /**< Floating point context address reg. */
    volatile uint32_t FPDSCR;   /**< Floating point status control register. */
} reg_fpu_t;


/* Register macro */

#ifdef TESTING
    extern reg_fpu_t fpu_testing;
    #define FPU                     ( (reg_fpu_t *) &fpu_testing )
#else
    #define FPU                     ( (reg_fpu_t *) 0xe000ed88 )
#endif


/*  DBG - debug support
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_dbg_t
 *  \brief  Representation of DBG register.
 *
 *  Described in reference manual p.1655ff.
 */
typedef struct {
    volatile uint32_t IDCODE;   /**< Device ID. */
    volatile uint32_t CR;       /**< Control register. */
    volatile uint32_t APB1_FZ;  /**< APB1 freeze register. */
    volatile uint32_t APB2_FZ;  /**< APB2 freeze register. */
} reg_dbg_t;


/* Register macro */

#ifdef TESTING
    extern reg_dbg_t dbg_testing;
    #define DBG                     ( (reg_dbg_t *) &dbg_testing )
#else
    #define DBG                     ( (reg_dbg_t *) 0xe0042000 )
#endif


/*  SRAM - static RAM
 * ------------------------------------------------------------------------- */

/* Helper macros */

#define SRAM1_RCC_PATTERN       ( 0x1 << 16u )
#define SRAM1_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, SRAM1_RCC_PATTERN)
#define SRAM1_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, SRAM1_RCC_PATTERN)

#define SRAM2_RCC_PATTERN       ( 0x1 << 17u )
#define SRAM2_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, SRAM2_RCC_PATTERN)
#define SRAM2_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, SRAM2_RCC_PATTERN)

#define SRAM3_RCC_PATTERN       ( 0x1 << 19u )
#define SRAM3_LP_ENABLE()       REG_SET(RCC->AHB1LPENR, SRAM3_RCC_PATTERN)
#define SRAM3_LP_DISABLE()      REG_CLR(RCC->AHB1LPENR, SRAM3_RCC_PATTERN)


/*  BKPSRAM - backup static RAM
 * ------------------------------------------------------------------------- */

/* Helper macros */

#define BKPSRAM_RCC_PATTERN     ( 0x1 << 18u )
#define BKPSRAM_ENABLE()        REG_SET(RCC->AHB3ENR,   BKPSRAM_RCC_PATTERN)
#define BKPSRAM_DISABLE()       REG_CLR(RCC->AHB3ENR,   BKPSRAM_RCC_PATTERN)
#define BKPSRAM_LP_ENABLE()     REG_SET(RCC->AHB1LPENR, BKPSRAM_RCC_PATTERN)
#define BKPSRAM_LP_DISABLE()    REG_CLR(RCC->AHB1LPENR, BKPSRAM_RCC_PATTERN)


/*  CCM - core coupled memory
 * ------------------------------------------------------------------------- */

/* Helper macros */

#define CCM_RCC_PATTERN         ( 0x1 << 20u )
#define CCM_ENABLE()            REG_SET(RCC->AHB1ENR,   CCM_RCC_PATTERN)
#define CCM_DISABLE()           REG_CLR(RCC->AHB1ENR,   CCM_RCC_PATTERN)


#endif
