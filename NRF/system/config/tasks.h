/*
 * ----------------------------------------------------------------------
 *
 * File: tasks.h
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
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

#ifndef TASKS_H_
#define TASKS_H_

/*******************************************************************************/
/**                          Power Management                                **/
/*******************************************************************************/

/** Power management thread priority */
#ifndef CONFIG_PWR_THREAD_PRIORITY
#define CONFIG_PWR_THREAD_PRIORITY 10
#endif

/** Minimum update period in ms for battery percentage / pmic status */
#ifndef THREAD_PWR_UPDATE_PERIOD_MS
#define THREAD_PWR_UPDATE_PERIOD_MS (1000 * 20)
#endif

/** Minimum period in ms between quiet-cycle measurements while the
 * application gate vetoes full cycles (e.g. during ExG streaming). The
 * thread still wakes every THREAD_PWR_UPDATE_PERIOD_MS so it resumes full
 * cycles promptly once the veto ends. */
#ifndef THREAD_PWR_QUIET_UPDATE_PERIOD_MS
#define THREAD_PWR_QUIET_UPDATE_PERIOD_MS (1000 * 120)
#endif

/** // Period in msat which the charger configuration is re-applied to the PMIC */
#ifndef THREAD_PWR_CHARGER_RECONIFG_PERIOD_MS
#define THREAD_PWR_CHARGER_RECONIFG_PERIOD_MS (1000 * 60 * 25)
#endif

#ifndef THREAD_PWR_SOFT_RESET_LATENCY_MS
#define THREAD_PWR_SOFT_RESET_LATENCY_MS 2000
#endif

/** Long-press (nEN down -> soft-reset release) shutdown via pwr_kill().
 * Applications that implement their own button/shutdown handling (e.g.
 * BioGAP: any press -> factory ship mode in main.c) must set this to 0,
 * otherwise both shutdown paths race on a button press. */
#ifndef CONFIG_PWR_LONG_PRESS_KILL
#define CONFIG_PWR_LONG_PRESS_KILL 1
#endif
#endif // TASKS_H_