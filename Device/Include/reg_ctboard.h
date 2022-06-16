/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Registries for CT Board.
 * 
 *  \file   reg_ctboard.h
 *  $Id$
 * ------------------------------------------------------------------------- */
#ifndef _REG_CTBOARD_H
#define _REG_CTBOARD_H

#include <stdint.h>


/*  LED register
 * ------------------------------------------------------------------------- */

/**
 *  \union  reg_ct_led_t
 *  \brief  Representation of LED interface.
 */
typedef union {
    struct {
        volatile uint8_t LED7_0;    /**< LED 7..0. */
        volatile uint8_t LED15_8;   /**< LED 15..8. */
        volatile uint8_t LED23_16;  /**< LED 23..16. */
        volatile uint8_t LED31_24;  /**< LED 31..24. */
    } BYTE;
    struct {
        volatile uint16_t LED15_0;  /**< LED 15..0. */
        volatile uint16_t LED31_16; /**< LED 31..16. */
    } HWORD;
    volatile uint32_t WORD;         /**< LED 31..0. */
} reg_ct_led_t;

#ifdef TESTING
    extern reg_ct_led_t led_testing;
    #define CT_LED                  ( (reg_ct_led_t *) &led_testing )
#else
    #define CT_LED                  ( (reg_ct_led_t *) 0x60000100 )
#endif


/*  7 segment display register
 * ------------------------------------------------------------------------- */

/**
 *  \union  reg_ct_seg7_raw_t
 *  \brief  Representation of 7 segment display RAW interface.
 */
typedef union {
    struct {
        volatile uint8_t DS0;       /**< Display 0. */
        volatile uint8_t DS1;       /**< Display 1. */
        volatile uint8_t DS2;       /**< Display 2. */
        volatile uint8_t DS3;       /**< Display 3. */
    } BYTE;
    struct {
        volatile uint16_t DS1_0;    /**< Display 0 & 1. */
        volatile uint16_t DS3_2;    /**< Display 2 & 3. */
    } HWORD;
    volatile uint32_t WORD;         /**< Display 0 to 3. */
} reg_ct_seg7_raw_t;


/**
 *  \union  reg_ct_seg7_bin_t
 *  \brief  Representation of 7 segment display BINary interface.
 */
typedef union {
    struct {
        volatile uint8_t DS1_0;     /**< Byte 0. */
        volatile uint8_t DS3_2;     /**< Byte 1. */
    } BYTE;
    volatile uint16_t HWORD;        /**< Byte 1 & 0. */
} reg_ct_seg7_bin_t;
	

/**
 *  \struct reg_ct_seg7_t
 *  \brief  Representation of 7 segment display register.
 *
 *  Described in ct board manual p.?ff.
 */
typedef struct {
    reg_ct_seg7_raw_t RAW;          /**< RAW interface. */
    reg_ct_seg7_bin_t BIN;          /**< BINary interface. */
} reg_ct_seg7_t;

#ifdef TESTING
    extern reg_ct_seg7_t seg7_testing;
    #define CT_SEG7                 ( (reg_ct_seg7_t *) &seg7_testing )
#else
    #define CT_SEG7                 ( (reg_ct_seg7_t *) 0x60000110 )
#endif

/*  DIP switch register
 * ------------------------------------------------------------------------- */

/**
 *  \union  reg_ct_dipsw_t
 *  \brief  Representation of DIP switch interface.
 */
typedef union {
    struct {
        volatile uint8_t S7_0;      /**< Switch 7..0. */
        volatile uint8_t S15_8;     /**< Switch 15..8. */
        volatile uint8_t S23_16;    /**< Switch 23..16. */
        volatile uint8_t S31_24;    /**< Switch 31..0. */
    } BYTE;
    struct {
        volatile uint16_t S15_0;    /**< Switch 15..0. */
        volatile uint16_t S31_16;   /**< Switch 31..16. */
    } HWORD;
    volatile uint32_t WORD;         /**< Switch 31..0. */
} reg_ct_dipsw_t;

#ifdef TESTING
    extern reg_ct_dipsw_t dipsw_testing;
    #define CT_DIPSW                ( (reg_ct_dipsw_t *) &dipsw_testing )
#else
    #define CT_DIPSW                ( (reg_ct_dipsw_t *) 0x60000200 )
#endif

/*  BUTTON register
 * ------------------------------------------------------------------------- */

#ifdef TESTING
    extern uint8_t button_testing;
    #define CT_BUTTON                (* (uint8_t *) &button_testing )
#else
    #define CT_BUTTON                (* (uint8_t *) 0x60000210 )
#endif


/*  HEX switch register
 * ------------------------------------------------------------------------- */

#ifdef TESTING
    extern uint8_t hexsw_testing;
    #define CT_HEXSW                 (* (uint8_t *) &hexsw_testing )
#else
    #define CT_HEXSW                 (* (uint8_t *) 0x60000211 )
#endif

/*  LCD register
 * ------------------------------------------------------------------------- */

/**
 *  \struct reg_ct_lcd_bg_t
 *  \brief  Representation of LCD background interface.
 */
typedef struct {
    volatile uint16_t RED;          /** Red PWM duty cycle register. */
    volatile uint16_t GREEN;        /** Green PWM duty cycle register. */
    volatile uint16_t BLUE;         /** Blue PWM duty cycle register. */
} reg_ct_lcd_bg_t;


/**
 *  \union  reg_ct_lcd_bin_t
 *  \brief  Representation of LCD BINary interface.
 */
typedef union {
    struct {
        volatile uint8_t B7_0;      /**< Bit 7..0. */
        volatile uint8_t B15_8;     /**< Bit 15..8. */
        volatile uint8_t B23_16;    /**< Bit 23..16. */
        volatile uint8_t B31_24;    /**< Bit 31..24. */
        volatile uint8_t B39_32;    /**< Bit 39..32. */
        volatile uint8_t B47_40;    /**< Bit 47..40. */
        volatile uint8_t B55_48;    /**< Bit 55..48. */
        volatile uint8_t B63_56;    /**< Bit 63..56. */
        volatile uint8_t B71_64;    /**< Bit 71..64. */
        volatile uint8_t B79_72;    /**< Bit 79..72. */
        volatile uint8_t B87_80;    /**< Bit 87..80. */
        volatile uint8_t B95_88;    /**< Bit 95..88. */
        volatile uint8_t B103_96;   /**< Bit 103..96. */
        volatile uint8_t B111_104;  /**< Bit 111..104. */
        volatile uint8_t B119_112;  /**< Bit 119..112. */
        volatile uint8_t B127_120;  /**< Bit 127..120. */
    } BYTE;
    struct {
        volatile uint16_t B15_0;    /**< Bit 15..0. */
        volatile uint16_t B31_16;   /**< Bit 31..16. */
        volatile uint16_t B47_32;   /**< Bit 47..32. */
        volatile uint16_t B63_48;   /**< Bit 63..48. */
        volatile uint16_t B79_64;   /**< Bit 79..64. */
        volatile uint16_t B95_80;   /**< Bit 95..80. */
        volatile uint16_t B111_96;  /**< Bit 111..96. */
        volatile uint16_t B127_112; /**< Bit 127..112. */
    } HWORD;
    struct {
        volatile uint32_t B31_0;    /**< Bit 31..0. */
        volatile uint32_t B63_32;   /**< Bit 63..32. */
        volatile uint32_t B95_64;   /**< Bit 95..64. */
        volatile uint32_t B127_96;  /**< Bit 127..96. */
    } WORD;
    struct {
        volatile uint64_t B63_0;    /**< Bit 63..0. */
        volatile uint64_t B127_64;  /**< Bit 127..64. */
    } DWORD;
} reg_ct_lcd_bin_t;


/**
 *  \struct reg_ct_lcd_t
 *  \brief  Representation of GPIO register.
 *
 *  Described in ct board manual p.?ff.
 */
typedef struct {
    volatile uint8_t ASCII[40];     /**< ASCII interface. */
    uint8_t RESERVED[8];
    reg_ct_lcd_bin_t BIN;           /**< BINary interface. */
    reg_ct_lcd_bg_t BG;             /**< Background interface. */
} reg_ct_lcd_t;


#ifdef TESTING
    extern reg_ct_lcd_t lcd_testing;
    #define CT_LCD                ( (reg_ct_lcd_t *) &lcd_testing )
#else
    #define CT_LCD                ( (reg_ct_lcd_t *) 0x60000300 )
#endif


/*  GPIO register
 * ------------------------------------------------------------------------- */

/**
 *  \union  reg_ct_gpio_port_t
 *  \brief  Representation of GPIO port.
 */
typedef union {
    struct {
        volatile uint8_t P1;        /**< GPIO Port 1. */
        volatile uint8_t P2;        /**< GPIO Port 2. */
        volatile uint8_t P3;        /**< GPIO Port 3. */
        volatile uint8_t P4;        /**< GPIO Port 4. */
    } BYTE;
    struct {
        volatile uint16_t P2_1;     /**< GPIO Port 1 & 2. */
        volatile uint16_t P4_3;     /**< GPIO Port 3 & 4. */
    } HWORD;
    volatile uint32_t WORD;         /**< GPIO Port 1 to 4. */
} reg_ct_gpio_port_t;
	

/**
 *  \struct reg_ct_gpio_t
 *  \brief  Representation of GPIO register.
 *
 *  Described in ct board manual p.?ff.
 */
typedef struct {
    reg_ct_gpio_port_t OUT;         /**< OUTput. */
    uint8_t RESERVED[12];
    reg_ct_gpio_port_t IN;          /**< INput. */
} reg_ct_gpio_t;


#ifdef TESTING
    extern reg_ct_gpio_t gpio_testing;
    #define CT_GPIO               ( (reg_ct_gpio_t *) &gpio_testing )
#else
    #define CT_GPIO               ( (reg_ct_gpio_t *) 0x60000400 )
#endif
 
#endif
