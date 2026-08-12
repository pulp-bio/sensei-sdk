/*
 * ----------------------------------------------------------------------
 *
 * File: pwr.h
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

#ifndef PWR_H_
#define PWR_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#define PWR_BAT_CRITICAL_MV (2900)
#define PWR_BAT_LOW_MV (3100)
#define PWR_BAT_OK_MV (3300)

/**
 * @brief Initialise the power sub-system
 * @return negative on error, 0 otherwise.
 */
int pwr_init();

/**
 * @brief Start the power sub-system
 * @note May only be called once.
 * @return negative on error, 0 otherwise.
 */
int pwr_start();

// note: mutex lock dies
void pwr_kill();

// Note must be active or init?
uint32_t pwr_bat_perc();

// Note must be active or init?
uint32_t pwr_bat_mV();

/**
 * @brief Snapshot of the cached battery/charger telemetry.
 *
 * Filled from atomics that the pwr thread refreshes periodically (and on
 * PMIC interrupt) - reading a snapshot never touches the PMIC or the I2C
 * bus, so it is safe from any context, including during data streaming.
 */
struct pwr_status {
  uint16_t batt_mV;         //! Battery voltage (mV)
  uint8_t batt_perc;        //! Battery state of charge (0-100 %), voltage-estimated
  bool chgin_present;       //! External power present on CHGIN (USB)
  bool charging;            //! Charger actively charging the battery
  bool chg_fault;           //! Charger timer/temperature fault
  bool thermal_alarm;       //! PMIC junction thermal alarm
  uint8_t chg_details;      //! Raw charger state machine (max77654_chg_dtls_t)
  uint16_t vsys_mV;         //! System voltage (mV)
  uint16_t chgin_mV;        //! Charger input voltage (mV), 0 if unplugged
  uint16_t chgin_dmA;       //! Charger input current (0.1 mA), 0 if unplugged
  uint16_t batt_dmA;        //! Battery current (0.1 mA): charge current while charging,
                            //! discharge current otherwise
  uint16_t chgin_power_mW;  //! Charger input power (mW), 0 if unplugged
  uint16_t batt_power_mW;   //! Battery-side power (mW): into the battery while charging,
                            //! out of the battery otherwise
};

/**
 * @brief Copy the current cached telemetry into @p out.
 * @note Values are 0 until thread_pwr_init() has run (part of pwr_init()).
 */
void pwr_get_status(struct pwr_status *out);

/**
 * @brief Register a gate that can veto the noisy part of PMIC activity.
 *
 * While the gate returns false, the periodic power thread runs a reduced
 * "quiet" cycle: write-only AMUX/SAADC measurements (battery voltage,
 * VSYS, currents, power) stay live, but everything that READS from the
 * PMIC is skipped - status registers, charger pause and charger reconfig.
 * The cached status flags are reused; while charging (per the cached
 * flags) or after a PMIC interrupt marks them stale, all values are held.
 * Everything catches up on the next allowed cycle.
 *
 * Rationale (measured on BioGAP with an ExG noise-injection sweep): when
 * the PMIC drives SDA during an I2C read, it sinks the pull-up current
 * through its die ground and the bounce couples into the regulators that
 * feed the AFE. I2C writes and AMUX/SAADC measurements are clean.
 *
 * @param allowed Returns true when full cycles may run. Must be fast and
 *                callable from the power thread. Pass NULL to remove.
 */
void pwr_set_measurement_gate(bool (*allowed)(void));

/** @brief True if no measurement gate is registered or it currently allows. */
bool pwr_measurements_allowed(void);

/**
 * @brief Register a veto on using the battery-monitor ADC at all.
 *
 * Distinct from the measurement gate above, which only suppresses *full*
 * cycles: while it vetoes, the power thread still runs reduced "quiet" cycles
 * because I2C writes and AMUX/SAADC measurements are electrically clean. That
 * reasoning holds only while the ADC pin belongs to the ADC.
 *
 * On boards where the battery-monitor input is multiplexed with an output the
 * application drives (BioGAP: P0.07 / AIN3 is also the mmWave shield's
 * power-enable), a measurement is not merely inaccurate — Zephyr's SAADC driver
 * re-applies the channel input on every read, which switches the pad to analog
 * mode and drops the application's drive. Whatever that pin was holding up
 * turns off. Register this gate so no measurement path touches the ADC while
 * the pin is in use.
 *
 * @param allowed Returns true when the ADC may be used. Must be fast and
 *                callable from the power thread. Pass NULL to remove.
 */
void pwr_set_adc_gate(bool (*allowed)(void));

/** @brief True if no ADC gate is registered or it currently allows. */
bool pwr_adc_allowed(void);

// Configure GAP9 power supply
void gap9_pwr(bool);

#endif /* PWR_H_ */
