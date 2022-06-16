/* ----------------------------------------------------------------------------
 * --  _____       ______  _____                                              -
 * -- |_   _|     |  ____|/ ____|                                             -
 * --   | |  _ __ | |__  | (___    Institute of Embedded Systems              -
 * --   | | | '_ \|  __|  \___ \   Zurich University of                       -
 * --  _| |_| | | | |____ ____) |  Applied Sciences                           -
 * -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland               -
 * ------------------------------------------------------------------------- */
/**
 *  \brief  Interface of module hal_pwr.
 *
 *  The hardware abstraction layer for the power control unit.
 *
 *  \file   hal_pwr.h
 *  $Id$
 * ------------------------------------------------------------------------- */

/* Re-definition guard */
#ifndef _HAL_PWR_H
#define _HAL_PWR_H


/* User includes */
#include "hal_common.h"


/* -- Type definitions
 * ------------------------------------------------------------------------- */

/**
 *  \enum   hal_pwr_regulator_t
 *  \brief  Defines the available voltage regulators.
 */
typedef enum {
    HAL_PWR_REGULATOR_MAIN = 0u,
    HAL_PWR_REGULATOR_LOWPOWER = 1u
} hal_pwr_regulator_t;

/**
 *  \enum   hal_pwr_lp_entry_t
 *  \brief  Defines the method used to enter/leave the low power modes.
 */
typedef enum {
    HAL_PWR_LP_ENTRY_WFI = 0u,
    HAL_PWR_LP_ENTRY_WFE = 1u
} hal_pwr_lp_entry_t;


/* -- Public function declarations
 * ------------------------------------------------------------------------- */

/** 
 *  \brief  Resets the periphery to default values.
 */
void hal_pwr_reset(void)
__attribute__((deprecated("Please use PWR_RESET().")));

/**
 *  \brief  Enables/disables the backup domain (RTC, backup sram/registers).
 *  \param  status : Either ENABLE or DISABLE.
 *  \return If successful ENABLED, else DISABLED.
 */
hal_bool_t hal_pwr_set_backup_domain(hal_bool_t status);

/**
 *  \brief  Enables/Disables write access to backup domain.
 *  \param  status : Either ENABLE or DISABLE.
 */
void hal_pwr_set_backup_access(hal_bool_t status);

/**
 *  \brief  Enables/Disables the wakeup pin.
 *  \param  status : Either ENABLE or DISABLE.
 */
void hal_pwr_set_wakeup_pin(hal_bool_t status);

/**
 *  \brief  Enables/Disables the flash powerdown mode.
 *  \param  status : Either ENABLE or DISABLE.
 */
void hal_pwr_set_flash_powerdown(hal_bool_t status);

/**
 *  \brief  Enables/Disables the voltage regulator overdrive mode.
 *  \param  status : Either ENABLE or DISABLE.
 *  \return If successful ENABLED, else DISABLED.
 */
hal_bool_t hal_pwr_set_overdrive(hal_bool_t status);

/**
 *  \brief  Enables/Disables the voltage regulator underdrive mode.
 *  \param  status : Either ENABLE or DISABLE.
 *  \return If successful ENABLED, else DISABLED.
 */
hal_bool_t hal_pwr_set_underdrive(hal_bool_t status);



/* Power voltage detection -> irq if Vdd high/low, progr. treshhold
 * Maybe for later

void        HAL_PWR_PVDConfig(PWR_PVDTypeDef *sConfigPVD);

void        HAL_PWR_EnablePVD(void);
void        HAL_PWR_DisablePVD(void);

void        HAL_PWR_PVD_IRQHandler(void);
void        HAL_PWR_PVDCallback(void);
*/

#endif
