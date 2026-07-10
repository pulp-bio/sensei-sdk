/*
 * ----------------------------------------------------------------------
 *
 * File: thread_pwr.h
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

#ifndef THREAD_PWR_H_
#define THREAD_PWR_H_

#include "zephyr/kernel.h"

/**
 * @brief Initialise the power thread
 * @note pwr mutex must be held when this function is called.
 * @return negative on error, 0 otherwise.
 */
int thread_pwr_init();

/**
 * @brief Start the pwr thread.
 * @note May only be called once.
 */
void thread_pwr_start();

/**
 * @brief Measure currents
 * @note pwr mutex must be held when this function is called.
 * @return negative on error, 0 otherwise.
 */
int measure_currents();

/**
 * @brief Measure battery voltage and percentage.
 * @note pwr mutex must be held when this function is called.
 * @return negative on error, 0 otherwise.
 */
int measure_voltages();

/**
 * @brief Current battery charge state, in percent.
 * @note 0 before first read, which is performed by thread_pwr_init.
 */
extern atomic_t battery_perc;

/**
 * @brief Current battery voltage, in millivolts
 * @note 0 before first read, which is performed by thread_pwr_init.
 */
extern atomic_t battery_mV;

/** Bits of @ref pwr_status_flags: */
#define PWR_STATUS_FLAG_CHGIN_PRESENT (1U << 0) //! External power on CHGIN
#define PWR_STATUS_FLAG_CHARGING (1U << 1)      //! Charger actively charging
#define PWR_STATUS_FLAG_CHG_FAULT (1U << 2)     //! Charger timer/temperature fault
#define PWR_STATUS_FLAG_THERMAL_ALARM (1U << 3) //! PMIC junction thermal alarm

/**
 * @brief Extended PMIC telemetry, refreshed every thread cycle.
 * @note 0 before first read, which is performed by thread_pwr_init.
 * Prefer the snapshot getter pwr_get_status() over reading these directly.
 */
extern atomic_t pwr_status_flags;   //! Bitmask of PWR_STATUS_FLAG_*
extern atomic_t pwr_chg_details;    //! Raw charger state (max77654_chg_dtls_t)
extern atomic_t pwr_vsys_mV;        //! System voltage (mV)
extern atomic_t pwr_chgin_mV;       //! Charger input voltage (mV), 0 if unplugged
extern atomic_t pwr_chgin_dmA;      //! Charger input current (0.1 mA), 0 if unplugged
extern atomic_t pwr_batt_dmA;       //! Battery current (0.1 mA); charge or discharge, see flags
extern atomic_t pwr_chgin_power_mW; //! Charger input power (mW), 0 if unplugged
extern atomic_t pwr_batt_power_mW;  //! Battery-side power (mW); direction per flags

/**
 * @brief Current GAP9 power state.
 * @note 0 before first read, which is performed by thread_pwr_init.
 */
extern atomic_t gap9_pwr_state;

/** @brief Power thread ID */
extern k_tid_t pwr_t;

#endif /* THREAD_PWR_H_ */
