/*
 * ----------------------------------------------------------------------
 *
 * File: pwr.c
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philipp Schilk (schilkp@ethz.ch), ETH Zurich
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pwr.h"

#include "bsp/pmic_bsp.h"
#include "bsp/pwr_bsp.h"
#include "config/pmic_default.h"
#include "pwr/pwr_common.h"
#include "pwr/thread_pwr.h"

#include "max77654.h"

#include "zephyr/sys/reboot.h"

#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// ======== Defines/Variables ======================================================================

LOG_MODULE_REGISTER(pwr);

#define GPIO_NODE_led_pwr DT_NODELABEL(gpio_led_pwr)
#define GPIO_NODE_gap9_pwr DT_NODELABEL(gpio_gap9_pwr)

static const struct gpio_dt_spec gpio_gap9_pwr_sel = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_pwr, gpios);
static const struct gpio_dt_spec gpio_led_pwr = GPIO_DT_SPEC_GET(GPIO_NODE_led_pwr, gpios);

static const struct device *pwr_i2c = DEVICE_DT_GET(DT_ALIAS(i2ca));

/* Powering the LED rail in pwr_start() is optional so applications can keep
 * every non-essential power domain off (e.g. BioGAP does not use the LED). */
#ifndef CONFIG_PWR_START_LED_POWER
#define CONFIG_PWR_START_LED_POWER 1
#endif

// PWR mutex:
// Has to be held to operate on the power subsys. For the public pwr interface this is managed automatically.
K_MUTEX_DEFINE(pwr_mutex);

// Driver handle:
struct max77654_h pmic_h = {
    .read_regs = pmic_bsp_read_regs,
    .write_regs = pmic_bsp_write_regs,
    .adc_read = pmic_bsp_adc_read,
    .log = pmic_bsp_log,
    .conf = PMIC_CONFIG_DEFAULT,
};

atomic_t pwr_is_initialised = ATOMIC_INIT(0);
atomic_t pwr_is_running = ATOMIC_INIT(0);

// ======== Functions ==============================================================================

int pwr_init() {

  if (!device_is_ready(pwr_i2c)) {
    LOG_ERR("I2C device %s is not ready\n", pwr_i2c->name);
    return -1;
  }

  if (gpio_pin_configure_dt(&gpio_gap9_pwr_sel, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("GAP9 EN GPIO init error");
    return -1;
  }

  if (gpio_pin_configure_dt(&gpio_led_pwr, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("LED EN GPIO init error");
    return -1;
  }

  int err = 0;

  if (k_mutex_lock(&pwr_mutex, PWR_MUTEX_TIMEOUT) < 0) {
    LOG_ERR("PWR Deadlock!");
    return -1;
  }

  if (pwr_bsp_init()) {
    LOG_ERR("PMIC BSP Init failed!");
    err = -1;
    goto cleanup;
  }

  if (max77654_reset(&pmic_h) != E_MAX77654_SUCCESS) {
    LOG_ERR("RESET failed!");
    err = -1;
    goto cleanup;
  }

  k_sleep(K_MSEC(300));

  if (max77654_init(&pmic_h) != E_MAX77654_SUCCESS) {
    LOG_ERR("Driver init failed.");
    err = -1;
    goto cleanup;
  }

  LOG_INF("driver init OK.");

  // Init pwr thread:
  if (thread_pwr_init() < 0) {
    LOG_ERR("pwr thread init failed.");
    err = -1;
    goto cleanup;
  }

  // Enabeling the i2cb switch seems to cause a glitch on the i2c bus which
  // has the PMIC fail it's next i2c transaction. This hotfix resets the
  // bus after to ensure the PMIC is happy.
  // pmic_i2c_reset_hotfix();

  atomic_set(&pwr_is_initialised, true);

  k_sleep(K_MSEC(100));

cleanup:
  k_mutex_unlock(&pwr_mutex);
  return err;
}

int pwr_start() {
  __ASSERT_NO_MSG(atomic_get(&pwr_is_initialised) == true);
  __ASSERT_NO_MSG(atomic_get(&pwr_is_running) == false);

  // Power up LED
  if (CONFIG_PWR_START_LED_POWER && gpio_pin_set_dt(&gpio_led_pwr, 1) < 0) {
    LOG_ERR("LED EN GPIO configuration error");
    return -1;
  }

  //if (pwr_bsp_start()) {
  //  LOG_ERR("PMIC BSP Start failed!");
  //  return -1;
  //}

  thread_pwr_start();

  atomic_set(&pwr_is_running, true);

  return 0;
}

extern struct k_sem pmic_update_sem;

void pwr_kill() {
  bool did_lock = false;
  // FIXME: This whole thing is very hacky and a workaround until the PMIC's FSM is working.

  LOG_INF("Stopping power subsystem...");
  for (uint32_t i = 0; i < 3; i++) {
    if (k_mutex_lock(&pwr_mutex, K_MSEC(10)) < 0) {
      LOG_ERR("PWR MUTEX Timeout!");
    } else {
      did_lock = true;
      break;
    }
  }

  if (!did_lock) {
    LOG_ERR("Ignoring pwr mutex. Forcibly shutting down!");
  }

  // hotfix PMIC i2c:
  // pmic_i2c_reset_hotfix();

  // Power down LED
  if (gpio_pin_set_dt(&gpio_led_pwr, 0) < 0) {
    LOG_ERR("Failed to power down LED");
  }

  int err = 0;
  do {
    pmic_h.conf.sbb_conf[0].en = MAX77654_REG_OFF;
    pmic_h.conf.sbb_conf[1].en = MAX77654_REG_OFF;
    pmic_h.conf.sbb_conf[2].en = MAX77654_REG_OFF;
    pmic_h.conf.ldo_conf[0].en = MAX77654_REG_OFF;
    pmic_h.conf.ldo_conf[1].en = MAX77654_REG_OFF;
    pmic_h.conf.main_bias_force_enable = false;
    err = max77654_config(&pmic_h);
    if (err) {
      LOG_ERR("Shutdown error!... retrying...");
      k_sleep(K_MSEC(10));
    }
  } while (err != 0);

  LOG_INF("Rebooting...");

  k_sleep(K_MSEC(500));
  sys_reboot(SYS_REBOOT_COLD);

  LOG_ERR("REBOOT FAIL!");

  if (did_lock) {
    k_mutex_unlock(&pwr_mutex);
  }
}

uint32_t pwr_bat_perc() {
  __ASSERT_NO_MSG(atomic_get(&pwr_is_initialised) == true);
  return atomic_get(&battery_perc);
}

uint32_t pwr_bat_mV() {
  __ASSERT_NO_MSG(atomic_get(&pwr_is_initialised) == true);
  return atomic_get(&battery_mV);
}

// Optional application veto for PMIC measurement activity:
static bool (*pwr_measurement_gate)(void);

void pwr_set_measurement_gate(bool (*allowed)(void)) { pwr_measurement_gate = allowed; }

bool pwr_measurements_allowed(void) {
  bool (*gate)(void) = pwr_measurement_gate;
  return (gate == NULL) || gate();
}

static bool (*pwr_adc_gate)(void);

void pwr_set_adc_gate(bool (*allowed)(void)) { pwr_adc_gate = allowed; }

bool pwr_adc_allowed(void) {
  bool (*gate)(void) = pwr_adc_gate;
  return (gate == NULL) || gate();
}

void pwr_get_status(struct pwr_status *out) {
  __ASSERT_NO_MSG(atomic_get(&pwr_is_initialised) == true);

  uint32_t flags = atomic_get(&pwr_status_flags);

  out->batt_mV = (uint16_t)atomic_get(&battery_mV);
  out->batt_perc = (uint8_t)atomic_get(&battery_perc);
  out->chgin_present = flags & PWR_STATUS_FLAG_CHGIN_PRESENT;
  out->charging = flags & PWR_STATUS_FLAG_CHARGING;
  out->chg_fault = flags & PWR_STATUS_FLAG_CHG_FAULT;
  out->thermal_alarm = flags & PWR_STATUS_FLAG_THERMAL_ALARM;
  out->chg_details = (uint8_t)atomic_get(&pwr_chg_details);
  out->vsys_mV = (uint16_t)atomic_get(&pwr_vsys_mV);
  out->chgin_mV = (uint16_t)atomic_get(&pwr_chgin_mV);
  out->chgin_dmA = (uint16_t)atomic_get(&pwr_chgin_dmA);
  out->batt_dmA = (uint16_t)atomic_get(&pwr_batt_dmA);
  out->chgin_power_mW = (uint16_t)atomic_get(&pwr_chgin_power_mW);
  out->batt_power_mW = (uint16_t)atomic_get(&pwr_batt_power_mW);
}

void gap9_pwr(bool on) {
  __ASSERT_NO_MSG(atomic_get(&pwr_is_initialised) == true);
  if (on) {
    atomic_set(&gap9_pwr_state, 1);
    gpio_pin_set_dt(&gpio_gap9_pwr_sel, 1);
  } else {
    atomic_set(&gap9_pwr_state, 0);
    gpio_pin_set_dt(&gpio_gap9_pwr_sel, 0);
  }
}