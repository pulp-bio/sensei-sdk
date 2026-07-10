/*
 * ----------------------------------------------------------------------
 *
 * File: thread_pwr.c
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

#include "pwr/thread_pwr.h"
#include "pwr/pwr.h"
#include "pwr/pwr_common.h"

#include "config/tasks.h"

#include "max77654.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdio.h>

// ======== Definitions/Variables ==================================================================

LOG_MODULE_REGISTER(thread_pwr);

/* The devicetree node identifier */
#define GPIO_NODE_pmic_irq DT_NODELABEL(gpio_pmic_irq)
#define GPIO_NODE_soft_nrst DT_NODELABEL(gpio_soft_nrst)
#define GPIO_NODE_gap9_pwr DT_NODELABEL(gpio_gap9_pwr)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec gpio_pmic_irq = GPIO_DT_SPEC_GET(GPIO_NODE_pmic_irq, gpios);
static const struct gpio_dt_spec gpio_soft_nrst = GPIO_DT_SPEC_GET(GPIO_NODE_soft_nrst, gpios);
static const struct gpio_dt_spec gpio_gap9_pwr_sel = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_pwr, gpios);

// Semaphore to indicate that an update should be performed:
K_SEM_DEFINE(pmic_update_sem, 0, 2);

// Current battery percentage (0-100):
atomic_t battery_perc = ATOMIC_INIT(0);

// Current battery voltage (mV):
atomic_t battery_mV = ATOMIC_INIT(0);

// Extended PMIC telemetry, refreshed by check_telemetry() every thread cycle.
// All values are cached in atomics so readers never touch the PMIC/I2C.
atomic_t pwr_status_flags = ATOMIC_INIT(0);  // Bitmask of PWR_STATUS_FLAG_*
atomic_t pwr_chg_details = ATOMIC_INIT(0);   // Raw max77654_chg_dtls_t
atomic_t pwr_vsys_mV = ATOMIC_INIT(0);       // System voltage (mV)
atomic_t pwr_chgin_mV = ATOMIC_INIT(0);      // Charger input voltage (mV), 0 if unplugged
atomic_t pwr_chgin_dmA = ATOMIC_INIT(0);     // Charger input current (0.1 mA), 0 if unplugged
atomic_t pwr_batt_dmA = ATOMIC_INIT(0);      // Battery current (0.1 mA): charge current while
                                             // charging, discharge current otherwise
atomic_t pwr_chgin_power_mW = ATOMIC_INIT(0); // Charger input power (mW), 0 if unplugged
atomic_t pwr_batt_power_mW = ATOMIC_INIT(0);  // Battery-side power (mW): into the battery while
                                              // charging, out of the battery otherwise

atomic_t gap9_pwr_state = ATOMIC_INIT(0);

/* Set on PMIC nIRQ: the cached charger status (pmic.status) may be stale.
 * Cleared when a full cycle re-reads the status registers. */
static atomic_t pmic_status_dirty = ATOMIC_INIT(0);

// PMIC interrupt callback:
static struct gpio_callback pmic_int_cb;
static void pmic_int_cb_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

// Soft Reset interrupt callback:
static struct gpio_callback softRst_int_cb;
static void softRst_int_cb_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

static int64_t last_keydown_ms;
static int64_t last_keyup_ms;

// Check the current PMIC/charger status:
static int check_status();

// Measure the current battery percentage:
static int check_battery();

// Measure VSYS/CHGIN/battery currents and update the telemetry atomics:
static int check_telemetry();

// Handle interrupts from the PMIC:
static int handle_interrupts();

// Re-apply charger configuration to the PMIC:
static int reapply_config();

// Convert battery mV to percentage:
static uint32_t battery_perc_conversion(uint32_t mV);

struct max77654_device {
  struct max77654_h *pmic_h;
  struct max77654_int flags;
  struct max77654_stat status;
};

struct max77654_device pmic;

// ======== Thread =================================================================================

int thread_pwr_init() {
  pmic.pmic_h = &pmic_h;

  // Setup PMIC int pin interrupt:
  if (gpio_pin_interrupt_configure_dt(&gpio_pmic_irq, GPIO_INT_EDGE_FALLING) < 0) {
    LOG_ERR("GPIO Int. config error!");
    return -1;
  }

  // Setup soft reset pin:
  if (gpio_pin_interrupt_configure_dt(&gpio_soft_nrst, GPIO_INT_EDGE_RISING) < 0) {
    LOG_ERR("GAP9 EN GPIO configuration error");
    return -1;
  }

  // Create a callback:
  gpio_init_callback(&pmic_int_cb, pmic_int_cb_handler, BIT(gpio_pmic_irq.pin));
  gpio_init_callback(&softRst_int_cb, softRst_int_cb_handler, BIT(gpio_soft_nrst.pin));

  // Attach callback to pin interrupt:
  if (gpio_add_callback(gpio_pmic_irq.port, &pmic_int_cb) < 0) {
    LOG_ERR("GPIO callback add error!");
    return -1;
  }

  if (gpio_add_callback(gpio_soft_nrst.port, &softRst_int_cb) < 0) {
    LOG_ERR("GPIO callback add error!");
    return -1;
  }

  // Populate all caches once so readers get valid data even before the
  // thread's first cycle (or if the thread is never started):
  check_status();
  check_battery();
  check_telemetry();

  return 0;
}

static void thread_pwr(void *_a, void *_b, void *_c) {
  LOG_INF("Running.");

  // Keep track of the last time the charger was re-configured (millisec):
  int64_t last_reconfig_ms = k_uptime_get();

  // Last measurement time, for the slower quiet-cycle cadence (millisec):
  int64_t last_meas_ms = k_uptime_get();

  last_keydown_ms = 0;
  last_keyup_ms = 0;

  while (1) {

    int err = 0;

    // Grab power mutex to be allowed to operate on PMIC:
    if (k_mutex_lock(&pwr_mutex, PWR_MUTEX_TIMEOUT) < 0) {
      LOG_ERR("Power deadlock!");
      err = -1;
      goto skip_update;
    }

    /* Applications can veto part of the PMIC activity in phases where it
     * would disturb them. Measured on BioGAP (ExG streaming, noise-
     * injection sweep): I2C READ transactions from the PMIC inject noise
     * into the ExG signals - during a read the PMIC sinks the SDA pull-up
     * current through its own die ground, and that ground bounce couples
     * into the regulators feeding the AFE. I2C WRITES and AMUX/SAADC
     * measurements are clean. So while vetoed, run a reduced "quiet"
     * cycle: keep the write-only measurements (voltage, currents, power)
     * live, and skip everything that reads from the PMIC (status
     * registers, charger pause/reconfig). The cached status flags are
     * reused; if a PMIC interrupt signals that they may be stale (USB
     * plugged/unplugged), hold everything until the next full cycle. */
    if (pwr_measurements_allowed()) {

      atomic_clear(&pmic_status_dirty);
      last_meas_ms = k_uptime_get();

      // Grab current PMIC status, issuing charger on / charger off events:
      if (check_status() < 0) {
        err = -1;
      }

      // Measure VSYS/CHGIN/battery currents and derived power (uses the
      // previous cycle's battery voltage for the power values, where the
      // staleness is negligible):
      if (check_telemetry() < 0) {
        err = -1;
      }

      // Measure battery level, issuing battery ok / critical / low events:
      if (check_battery() < 0) {
        err = -1;
      }

      // Handle interrupts from PMIC
      if (handle_interrupts() < 0) {
        err = -1;
      }

      // Re-apply charger configuration periodically. Recommended by PMIC Datasheet.
      if ((k_uptime_get() - last_reconfig_ms) > THREAD_PWR_CHARGER_RECONIFG_PERIOD_MS) {
        last_reconfig_ms = k_uptime_get();
        LOG_INF("Re-applying PMIC configuration");
        if (reapply_config() < 0) {
          err = -1;
        };
      }
    } else if (!atomic_get(&pmic_status_dirty) && !pmic.status.charging_active &&
               (k_uptime_get() - last_meas_ms) >= THREAD_PWR_QUIET_UPDATE_PERIOD_MS) {
      /* Quiet cycle: write-only measurements against the cached (frozen)
       * status flags, at the slower THREAD_PWR_QUIET_UPDATE_PERIOD_MS
       * cadence. Not while charging - the rest-voltage sampling in
       * check_battery would toggle the charger (a read-modify-write, and a
       * charge-current step); hold the values instead until the veto ends. */
      last_meas_ms = k_uptime_get();
      if (check_telemetry() < 0) {
        err = -1;
      }
      if (check_battery() < 0) {
        err = -1;
      }
    }

    // Handle button press that are longer than 2 seconds
    if (CONFIG_PWR_LONG_PRESS_KILL && (last_keyup_ms - last_keydown_ms > THREAD_PWR_SOFT_RESET_LATENCY_MS)) {
      LOG_INF("Button pressed for more than 2 seconds");
      last_keydown_ms = 0;
      last_keyup_ms = 0;
      pwr_kill();
    }

  skip_update:
    if (err < 0) {
      LOG_ERR("Error in power thread!");
    }

    k_mutex_unlock(&pwr_mutex);

    // Update on pmic interrupt but at least every PWR_THREAD_UPDATE_PERIOD_MS
    k_sem_take(&pmic_update_sem, K_MSEC(THREAD_PWR_UPDATE_PERIOD_MS));
  }
}

// ======== Functions ==============================================================================

static void pmic_int_cb_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
  LOG_DBG("PMIC interrupt!");
  last_keydown_ms = k_uptime_get();
  atomic_set(&pmic_status_dirty, 1);
  k_sem_give(&pmic_update_sem);
}

static void softRst_int_cb_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
  LOG_DBG("Soft RST interrupt!");
  last_keyup_ms = k_uptime_get();
  k_sem_give(&pmic_update_sem);
}

static int check_status() {
  // Grab pmic flags to clear interrupt:
  if (max77654_get_int_flags(&pmic_h, &pmic.flags) != E_MAX77654_SUCCESS) {
    LOG_ERR("pmic get_int_flags failed!");
    return -1;
  }

  // Get current status:
  if (max77654_get_stat(&pmic_h, &pmic.status) != E_MAX77654_SUCCESS) {
    LOG_ERR("pmic get_stat failed!");
    return -1;
  }

  // Notify state machine of current charger status:
  // bool chg_active = status.chgin_status != MAX77654_CHGIN_DTLS_UVLO;
  // LOG_INF("Charger %s\r\n", chg_active ? "active" : "inactive");

  return 0;
}

static int handle_interrupts() {
  // Check PMIC interrupt flags if button was pressed:
  if (pmic.flags.nen_f) {
    if (atomic_get(&gap9_pwr_state) == 1) {
      // Reset GAP9
      LOG_WRN("Resetting GAP9");
      if (gpio_pin_set_dt(&gpio_gap9_pwr_sel, 0) < 0) {
        LOG_ERR("GAP9 EN GPIO configuration error");
        return -1;
      }
      k_msleep(100);
      if (gpio_pin_set_dt(&gpio_gap9_pwr_sel, 1) < 0) {
        LOG_ERR("GAP9 EN GPIO configuration error");
        return -1;
      }
    } else {
      LOG_INF("GAP9 is off. Ignoring reset.");
    }
  }

  return 0;
}

/* While charging, sample the cell's rest voltage every N thread cycles
 * (~60 s at the default 20 s update period) by briefly pausing the charger.
 * With external power attached the system runs from CHGIN, so the paused
 * battery is unloaded and its terminal voltage relaxes towards the rest
 * voltage that the percentage lookup is calibrated for. */
#define CHG_REST_SAMPLE_PERIOD_CYCLES 3

/* Relaxation time between pausing the charger and measuring. Keep the total
 * pause well below 500 ms: that is the pwr_mutex timeout after which
 * app-side rail helpers proceed unlocked. */
#define CHG_REST_SETTLE_MS 350

/* With no battery attached, the BATT pin is driven by the charger towards
 * the VSYS regulation point and reads ~4.5 V - far above any real LiPo
 * terminal voltage (<= ~4.25 V at full charge). Readings above this
 * threshold therefore mean "no battery": report 0 mV / 0 % instead of
 * pretending a full battery is present. */
#define BATT_ABSENT_THRESHOLD_MV 4300

// Measure the battery voltage 3 times and average (ignoring flukes):
static int measure_batt_mV_avg(uint32_t *out_mV) {
  uint32_t battery_mV_avg = 0;
  uint32_t measurements = 0;

  for (uint32_t i = 0; i < 3; i++) {
    uint32_t result;
    if (max77654_measure(&pmic_h, MAX77654_BATT_V, &result) != E_MAX77654_SUCCESS) {
      LOG_ERR("pmic measure_bat failed!");
      return -1;
    }

    // Ignore any measurements below 100mV:
    // Usually the first ever ADC reading is 0-10mV, this prevents
    // this reading (or other fluke readings) from triggering a low/critical battery
    // warning.
    if (result <= 100) {
      LOG_ERR("Ignoring measurement %i mV", result);
    } else {
      measurements++;
      battery_mV_avg += result;
    }

    k_sleep(K_MSEC(1));
  }

  // If however, all measurements were invalid, fail:
  if (measurements == 0) {
    LOG_ERR("All measurements invalid!");
    return -1;
  }

  // Average the performed measurements:
  *out_mV = battery_mV_avg / measurements;
  return 0;
}

static int check_battery() {
  const struct max77654_stat *st = &pmic.status; // Filled by check_status()
  static uint32_t rest_sample_countdown;

  bool chg_done = (st->charger_status == MAX77654_CHG_DTLS_DONE ||
                   st->charger_status == MAX77654_CHG_DTLS_DONE_JEITA);

  if (st->charging_active && !chg_done) {
    /* While the charger runs, the terminal voltage tracks the charger, not
     * the cell, so a live reading is useless for the percentage. Pause the
     * charger, let the cell relax, measure its near-rest voltage, resume.
     * Between samples, hold the last values. The pause is invisible to
     * hosts: all BLE-visible state is served from the cache, which was
     * filled before the pause. */
    if (rest_sample_countdown > 0) {
      rest_sample_countdown--;
      return 0;
    }
    rest_sample_countdown = CHG_REST_SAMPLE_PERIOD_CYCLES - 1;

    if (max77654_set_charger_enabled(&pmic_h, false) != E_MAX77654_SUCCESS) {
      LOG_ERR("charger pause failed!");
      return -1;
    }

    k_sleep(K_MSEC(CHG_REST_SETTLE_MS));

    uint32_t rest_mV = 0;
    int err = measure_batt_mV_avg(&rest_mV);

    // Always resume charging, even if the measurement failed:
    if (max77654_set_charger_enabled(&pmic_h, true) != E_MAX77654_SUCCESS) {
      LOG_ERR("charger resume failed!");
      err = -1;
    }

    if (err < 0) {
      return -1;
    }

    if (rest_mV > BATT_ABSENT_THRESHOLD_MV) {
      // No battery attached:
      atomic_set(&battery_mV, 0);
      atomic_set(&battery_perc, 0);
      return 0;
    }

    atomic_set(&battery_mV, rest_mV);

    // Leave 100% to the charger's DONE signal:
    uint32_t perc = battery_perc_conversion(rest_mV);
    if (perc > 99) {
      perc = 99;
    }
    atomic_set(&battery_perc, perc);
    return 0;
  }

  // Not actively charging: measure directly. Reset the countdown so the
  // first rest sample happens right after charging starts:
  rest_sample_countdown = 0;

  uint32_t mV = 0;
  if (measure_batt_mV_avg(&mV) < 0) {
    return -1;
  }

  if (mV > BATT_ABSENT_THRESHOLD_MV) {
    // No battery attached (the pin is driven by the charger/VSYS):
    atomic_set(&battery_mV, 0);
    atomic_set(&battery_perc, 0);
    return 0;
  }

  atomic_set(&battery_mV, mV);
  atomic_set(&battery_perc, chg_done ? 100 : battery_perc_conversion(mV));

  return 0;
}

/* Convert a fast-charge constant-current setting to 0.1 mA units.
 * The CHG_CC code is (I / 7.5mA) - 1, so I = (code + 1) * 7.5 mA. */
static uint16_t chg_cc_to_dmA(max77654_chg_cc_t cc) { return ((uint16_t)cc + 1) * 75; }

/* Measure the battery discharge current with a simple auto-range: start on
 * the 103.4 mA scale, re-measure on 300 mA if railed or on 8.2 mA if tiny.
 * The PMIC reports percent-of-fullscale, so the absolute resolution is
 * ~1% of the selected scale. Result in 0.1 mA units. */
static int measure_discharge_dmA(uint32_t *out_dmA) {
  uint32_t pct;
  if (max77654_measure(&pmic_h, MAX77654_BATT_I_103MA4, &pct) != E_MAX77654_SUCCESS) {
    return -1;
  }
  if (pct >= 95) {
    if (max77654_measure(&pmic_h, MAX77654_BATT_I_300MA, &pct) != E_MAX77654_SUCCESS) {
      return -1;
    }
    *out_dmA = (pct * 3000) / 100;
  } else if (pct < 8) {
    if (max77654_measure(&pmic_h, MAX77654_BATT_I_8MA2, &pct) != E_MAX77654_SUCCESS) {
      return -1;
    }
    *out_dmA = (pct * 82) / 100;
  } else {
    *out_dmA = (pct * 1034) / 100;
  }
  return 0;
}

static int check_telemetry() {
  const struct max77654_stat *st = &pmic.status; // Filled by check_status()

  uint32_t flags = 0;
  bool chgin_present = (st->chgin_status == MAX77654_CHGIN_DTLS_OK);
  bool charging = st->charging_active;

  if (chgin_present) {
    flags |= PWR_STATUS_FLAG_CHGIN_PRESENT;
  }
  if (charging) {
    flags |= PWR_STATUS_FLAG_CHARGING;
  }
  if (st->charger_status == MAX77654_CHG_DTLS_PREQUAL_TIMER_FAULT ||
      st->charger_status == MAX77654_CHG_DTLS_FASTCHARGE_TIMER_FAULT ||
      st->charger_status == MAX77654_CHG_DTLS_BAT_TEMPERATURE_FAULT) {
    flags |= PWR_STATUS_FLAG_CHG_FAULT;
  }
  if (st->thermal_alarm_1 || st->thermal_alarm_2) {
    flags |= PWR_STATUS_FLAG_THERMAL_ALARM;
  }

  uint32_t vsys = 0;
  if (max77654_measure(&pmic_h, MAX77654_VSYS, &vsys) != E_MAX77654_SUCCESS) {
    LOG_ERR("pmic measure vsys failed!");
    return -1;
  }

  uint32_t chgin_v = 0, chgin_i_dmA = 0, batt_i_dmA = 0;

  if (chgin_present) {
    uint32_t chgin_i_mA = 0;
    if (max77654_measure(&pmic_h, MAX77654_CHGIN_V, &chgin_v) != E_MAX77654_SUCCESS ||
        max77654_measure(&pmic_h, MAX77654_CHGIN_I, &chgin_i_mA) != E_MAX77654_SUCCESS) {
      LOG_ERR("pmic measure chgin failed!");
      return -1;
    }
    chgin_i_dmA = chgin_i_mA * 10;

    if (charging) {
      // Charge current is reported as percent of the fast-charge setting:
      uint32_t pct = 0;
      if (max77654_measure(&pmic_h, MAX77654_BATT_I_CHG, &pct) != E_MAX77654_SUCCESS) {
        LOG_ERR("pmic measure charge current failed!");
        return -1;
      }
      batt_i_dmA = (pct * chg_cc_to_dmA(pmic_h.conf.fast_chg_cc)) / 100;
    }
  } else {
    // The discharge monitor is only meaningful without external power:
    if (measure_discharge_dmA(&batt_i_dmA) < 0) {
      LOG_ERR("pmic measure discharge current failed!");
      return -1;
    }
  }

  atomic_set(&pwr_vsys_mV, vsys);
  atomic_set(&pwr_chgin_mV, chgin_v);
  atomic_set(&pwr_chgin_dmA, chgin_i_dmA);
  atomic_set(&pwr_batt_dmA, batt_i_dmA);
  // mV * 0.1mA = 0.1 µW; /10000 -> mW
  atomic_set(&pwr_chgin_power_mW, (chgin_v * chgin_i_dmA) / 10000);
  atomic_set(&pwr_batt_power_mW, ((uint32_t)atomic_get(&battery_mV) * batt_i_dmA) / 10000);
  atomic_set(&pwr_chg_details, st->charger_status);
  atomic_set(&pwr_status_flags, flags);

  return 0;
}

int measure_currents() {
  int current;

  // Array with name, unit and index
  const struct {
    const char *name;
    const char *unit;
    int index;
  } current_names[] = {
      {"CHGIN Current             ", "mA", MAX77654_CHGIN_I},
      {"Battery Current           ", "%", MAX77654_BATT_I_CHG},
      {"Battery Discharge Current (   8.2 mA)", "mA", MAX77654_BATT_I_8MA2},
      {"Battery Discharge Current ( 40.5 mA)", "mA", MAX77654_BATT_I_40MA5},
      {"Battery Discharge Current ( 72.3 mA)", "mA", MAX77654_BATT_I_72MA3},
      {"Battery Discharge Current (103.4 mA)", "mA", MAX77654_BATT_I_103MA4},
      {"Battery Discharge Current (134.1 mA)", "mA", MAX77654_BATT_I_134MA1},
      {"Battery Discharge Current (164.1 mA)", "mA", MAX77654_BATT_I_164MA1},
      {"Battery Discharge Current (193.7 mA)", "mA", MAX77654_BATT_I_193MA7},
      {"Battery Discharge Current (222.7 mA)", "mA", MAX77654_BATT_I_222MA7},
      {"Battery Discharge Current (251.2 mA)", "mA", MAX77654_BATT_I_251MA2},
      {"Battery Discharge Current (279.3 mA)", "mA", MAX77654_BATT_I_279MA3},
      {"Battery Discharge Current (300.0 mA)", "mA", MAX77654_BATT_I_300MA},
  };

  // Iterate over all max77654_measure_t types:
  LOG_INF("Measuring PMIC Currents:");
  for (uint32_t i = 0; i < sizeof(current_names) / sizeof(current_names[0]); i++) {
    if (max77654_measure(&pmic_h, current_names[i].index, &current) != E_MAX77654_SUCCESS) {
      LOG_ERR("pmic measure failed!");
      return -1;
    }
    LOG_INF("  %s: %i %s", current_names[i].name, current, current_names[i].unit);
  }

  return 0;
}

int measure_voltages() {
  int voltage;

  // Array with name, unit and index
  const struct {
    const char *name;
    const char *unit;
    int index;
  } voltage_names[] = {
      {"AGND Voltage              ", "mV", MAX77654_AGND},
      {"VSYS Voltage              ", "mV", MAX77654_VSYS},
      {"CHGIN Voltage             ", "mV", MAX77654_CHGIN_V},
      {"CHGIN Current             ", "mA", MAX77654_CHGIN_I},
      {"Battery Voltage           ", "mV", MAX77654_BATT_V},
      {"Battery Current           ", "%", MAX77654_BATT_I_CHG},
      {"Battery Discharge Current ", "mA", MAX77654_BATT_I_8MA2},
      {"Thermistor Voltage        ", "mV", MAX77654_THM},
      {"Thermistor Bias           ", "mV", MAX77654_TBIAS},
  };

  // Iterate over all max77654_measure_t types:
  LOG_INF("Measuring PMIC Voltages:");
  for (uint32_t i = 0; i < sizeof(voltage_names) / sizeof(voltage_names[0]); i++) {
    if (max77654_measure(&pmic_h, voltage_names[i].index, &voltage) != E_MAX77654_SUCCESS) {
      LOG_ERR("pmic measure failed!");
      return -1;
    }
    LOG_INF("  %s: %i %s", voltage_names[i].name, voltage, voltage_names[i].unit);
  }

  return 0;
}

static int reapply_config() {

  // Apply charger/global resource config to PMIC:
  if (max77654_config_charger(&pmic_h) != E_MAX77654_SUCCESS) {
    LOG_ERR("PMIC config_charger failed!");
    return -1;
  }

  if (max77654_config_global(&pmic_h) != E_MAX77654_SUCCESS) {
    LOG_ERR("PMIC config_charger failed!");
    return -1;
  }
  return 0;
}

// A battery mV / charge percentage reference point.
struct reference_point {
  uint32_t mV;
  uint32_t perc;
};

static const struct reference_point perc_ref_points[] = {
    {.mV = 4200, .perc = 100}, {.mV = 4000, .perc = 85}, {.mV = 3900, .perc = 60}, {.mV = 3700, .perc = 40},
    {.mV = 3600, .perc = 20},  {.mV = 3500, .perc = 10}, {.mV = 3400, .perc = 5},  {.mV = 3300, .perc = 0},
};

static const uint32_t perc_ref_points_count = sizeof(perc_ref_points) / sizeof(struct reference_point);

static uint32_t battery_perc_conversion(uint32_t mV) {

  // Edge cases:
  if (mV > perc_ref_points[0].mV) {
    return 100;
  }
  if (mV <= perc_ref_points[perc_ref_points_count - 1].mV) {
    return 0;
  }

  // Find two reference points between which the reading lies:
  const struct reference_point *ref_high = 0;
  const struct reference_point *ref_low = 0;

  for (uint32_t i = 1; i < perc_ref_points_count; i++) {
    if (mV > perc_ref_points[i].mV) {
      ref_low = &perc_ref_points[i];
      ref_high = &perc_ref_points[i - 1];
      break;
    }
  }
  __ASSERT_NO_MSG(ref_high != 0);
  __ASSERT_NO_MSG(ref_low != 0);
  __ASSERT_NO_MSG(ref_high->mV >= mV);
  __ASSERT_NO_MSG(ref_low->mV < mV);

  // Linearly interpolate:
  return ref_low->perc + ((mV - ref_low->mV) * (ref_high->perc - ref_low->perc)) / (ref_high->mV - ref_low->mV);
}

// ======== Thread Setup ===========================================================================

static K_THREAD_STACK_DEFINE(thread_pwr_stack, 1024);
static struct k_thread thread_pwr_data;
k_tid_t pwr_t;

void thread_pwr_start() {

  pwr_t = k_thread_create(&thread_pwr_data,                                          // Thread data
                          thread_pwr_stack, K_THREAD_STACK_SIZEOF(thread_pwr_stack), // Thread Stack
                          thread_pwr,                                                // Entry
                          NULL, NULL, NULL,                                          // Args...
                          CONFIG_PWR_THREAD_PRIORITY,                                // Priority
                          0,                                                         // Options
                          K_NO_WAIT                                                  // Delay
  );
  k_thread_name_set(pwr_t, "pwr");
}
