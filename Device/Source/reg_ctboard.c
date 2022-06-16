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
 *  \file   reg_stm32f4xx.c
 *  $Id$
 * ------------------------------------------------------------------------- */

#include "reg_ctboard.h"

#ifdef TESTING
reg_ct_led_t led_testing;
reg_ct_seg7_t seg7_testing;
reg_ct_dipsw_t dipsw_testing;
uint8_t button_testing;
uint8_t hexsw_testing;
reg_ct_lcd_t lcd_testing;
reg_ct_gpio_t gpio_testing;
#endif
