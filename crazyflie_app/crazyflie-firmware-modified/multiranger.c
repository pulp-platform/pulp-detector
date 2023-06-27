/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */

/*  Modified by:
                Lorenzo Lamberti    <lorenzo.lamberti@unibo.it>
    Date:       01.04.2023
*/

/* multiranger.c: Multiranger deck driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "MR"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l1x.h"
#include "range.h"
#include "static_mem.h"
#include "config.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;

#define MR_PIN_UP     PCA95X4_P0
#define MR_PIN_FRONT  PCA95X4_P4
#define MR_PIN_BACK   PCA95X4_P1
#define MR_PIN_LEFT   PCA95X4_P6
#define MR_PIN_RIGHT  PCA95X4_P2

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devFront;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devBack;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devUp;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devLeft;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devRight;

/* --- Distance mode --- */
// #define RANGE_MODE VL53L1_DISTANCEMODE_SHORT     // Maximum distance: up to 1.3m     -    Benefit: better ambient immunity
// #define RANGE_MODE VL53L1_DISTANCEMODE_MEDIUM    // Maximum distance: up to 3m
#define RANGE_MODE VL53L1_DISTANCEMODE_LONG         // Maximum distance: up to 4m       -    Benefit: Maximum distance

/* --- Preset mode --- */
#define PRESET_MODE VL53L1_PRESETMODE_LITE_RANGING

/** --- Timing budget ---
 * Timing budget is the time required by the sensor to perform one range measurement.
 * The minimum and maximum timing budgets are [20 ms, 1000 ms]
 */
#define TIMING_BUDGET_US 50000 // [us]      (example: 50000=50ms)
/** --- Sensor status ---
 * The driver uses two parameters to qualify the ranging measurement: signal and sigma.
 * If signal or sigma are outside the limits, the ranging is flagged as invalid (note that RangeStatus is different than zero).
 *      1. Sigma: VL53L1X_CHECKENABLE_SIGMA_FINAL_RANGE
 *         Sigma is expressed in mm and is the estimation of the standard deviation of the measurement.
 *      2. Signal: VL53L1X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE
 *         The signal rate measurement, expressed in MCPS, represents the amplitude of the signal reflected from the target and detected by the device.
 */


struct{
    int16_t front;
    int16_t back;
    int16_t up;
    int16_t right;
    int16_t left;
}range_value;

struct{
    uint8_t front;
    uint8_t back;
    uint8_t up;
    uint8_t right;
    uint8_t left;
}range_states;

static bool mrInitSensor(VL53L1_Dev_t *pdev, uint32_t pca95pin, char *name)
{
    bool status;

    // Bring up VL53 by releasing XSHUT
    pca95x4SetOutput(pca95pin);
    // Let VL53 boot
    vTaskDelay(M2T(2));
    // Init VL53
    if (vl53l1xInit(pdev, I2C1_DEV)){
        DEBUG_PRINT("[%s] init sensor [OK]\n", name);
        status = true;
    }
    else{
        DEBUG_PRINT("[%s] init sensor [FAIL]\n", name);
        status = false;
    }

    if (status == true){
        /* --- Print Preset mode --- */
        VL53L1_PresetModes presetMode;
        VL53L1_GetPresetMode(&devFront, &presetMode);
        switch(presetMode){
            case VL53L1_PRESETMODE_AUTONOMOUS:
                DEBUG_PRINT("[%s] Preset mode: Autonomous\n", name);
                break;
            case VL53L1_PRESETMODE_LITE_RANGING:
                DEBUG_PRINT("[%s] Preset mode: Lite ranging\n", name);
                break;
            case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
                DEBUG_PRINT("[%s] Preset mode: Low power autonomous\n", name);
                break;
        }
        /* --- Print Distance mode --- */
        VL53L1_DistanceModes distanceMode;
        VL53L1_GetDistanceMode(&pdev, &distanceMode);
        switch(presetMode){
            case VL53L1_DISTANCEMODE_SHORT:
                DEBUG_PRINT("[%s] Distance mode: Short\n", name);
                break;
            case VL53L1_DISTANCEMODE_MEDIUM:
                DEBUG_PRINT("[%s] Distance mode: Medium\n", name);
                break;
            case VL53L1_DISTANCEMODE_LONG:
                DEBUG_PRINT("[%s] Distance mode: Long\n", name);
                break;
        }
        /* --- Print Timing Budget  --- */
        uint32_t measurementTimingBudgetMicroSeconds;
        VL53L1_GetMeasurementTimingBudgetMicroSeconds(&pdev, &measurementTimingBudgetMicroSeconds);
        DEBUG_PRINT("[%s] Measurement timing budget: %u us\n",  name, measurementTimingBudgetMicroSeconds);
        /* --- Print Inter Measurement Period  --- */
        uint32_t interMeasurementPeriodMilliSeconds;
        VL53L1_GetInterMeasurementPeriodMilliSeconds(&pdev, &interMeasurementPeriodMilliSeconds);
        DEBUG_PRINT("[%s] Inter measurement period: %u ms\n", name, interMeasurementPeriodMilliSeconds);
        /* --- Print Sigma & Signal  --- */
        FixPoint1616_t limitCheckValue;
        VL53L1_GetLimitCheckValue(&pdev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, &limitCheckValue);
        DEBUG_PRINT("[%s] Sigma final range: %f\n", name, limitCheckValue / ((float) (1<<16)));
        VL53L1_GetLimitCheckValue(&pdev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &limitCheckValue);
        DEBUG_PRINT("[%s] Signal rate final range: %f\n", name, limitCheckValue/ ((float) (1<<16)));
  }
  return status;
}

static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev, uint8_t *range_status)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;

    status = VL53L1_WaitMeasurementDataReady(dev);
    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    status = VL53L1_ClearInterruptAndStartMeasurement(dev);
    status = status;

    *range_status = rangingData.RangeStatus;
    return (uint16_t) rangingData.RangeMilliMeter;
}

static void mrTask(void *param)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    // Change the distance mode and set the maximum allowed time for the sensor measurements
    status = VL53L1_SetPresetMode(&devFront, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devFront, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devFront, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devBack, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devBack, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devBack, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devUp, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devUp, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devUp, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devLeft, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devLeft, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devLeft, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devRight, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devRight, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devRight, TIMING_BUDGET_US);


    // Restart all sensors
    status = VL53L1_StopMeasurement(&devFront);
    status = VL53L1_StartMeasurement(&devFront);
    status = VL53L1_StopMeasurement(&devBack);
    status = VL53L1_StartMeasurement(&devBack);
    status = VL53L1_StopMeasurement(&devUp);
    status = VL53L1_StartMeasurement(&devUp);
    status = VL53L1_StopMeasurement(&devLeft);
    status = VL53L1_StartMeasurement(&devLeft);
    status = VL53L1_StopMeasurement(&devRight);
    status = VL53L1_StartMeasurement(&devRight);
    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(100));

        range_value.front = mrGetMeasurementAndRestart(&devFront, &range_states.front);
        rangeSet(rangeFront, range_value.front/1000.0f);

        range_value.back = mrGetMeasurementAndRestart(&devBack, &range_states.back);
        rangeSet(rangeBack, range_value.back/1000.0f);

        range_value.up = mrGetMeasurementAndRestart(&devUp, &range_states.up);
        rangeSet(rangeUp, range_value.up/1000.0f);

        range_value.left = mrGetMeasurementAndRestart(&devLeft, &range_states.left);
        rangeSet(rangeLeft, range_value.left/1000.0f);

        range_value.right = mrGetMeasurementAndRestart(&devRight, &range_states.right);
        rangeSet(rangeRight, range_value.right/1000.0f);
    }
}

static void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    pca95x4ConfigOutput(~(MR_PIN_UP |
                          MR_PIN_RIGHT |
                          MR_PIN_LEFT |
                          MR_PIN_FRONT |
                          MR_PIN_BACK));

    pca95x4ClearOutput(MR_PIN_UP |
                       MR_PIN_RIGHT |
                       MR_PIN_LEFT |
                       MR_PIN_FRONT |
                       MR_PIN_BACK);

    isInit = true;

    xTaskCreate(mrTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL,
        MULTIRANGER_TASK_PRI, NULL);
}

static bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    isPassed &= mrInitSensor(&devFront, MR_PIN_FRONT, "front");
    isPassed &= mrInitSensor(&devBack, MR_PIN_BACK, "back");
    isPassed &= mrInitSensor(&devUp, MR_PIN_UP, "up");
    isPassed &= mrInitSensor(&devLeft, MR_PIN_LEFT, "left");
    isPassed &= mrInitSensor(&devRight, MR_PIN_RIGHT, "right");

    isTested = true;

    return isPassed;
}

static const DeckDriver multiranger_deck = {
    .vid = 0xBC,
    .pid = 0x0C,
    .name = "bcMultiranger",

    .usedGpio = 0, // FIXME: set the used pins
    .usedPeriph = DECK_USING_I2C,

    .init = mrInit,
    .test = mrTest,
};

DECK_DRIVER(multiranger_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)

PARAM_GROUP_STOP(deck)

LOG_GROUP_START(mRange)
LOG_ADD(LOG_INT16, ValF, &range_value.front)
LOG_ADD(LOG_INT16, ValB, &range_value.back)
LOG_ADD(LOG_INT16, ValU, &range_value.up)
LOG_ADD(LOG_INT16, ValL, &range_value.left)
LOG_ADD(LOG_INT16, ValR, &range_value.right)
LOG_ADD(LOG_UINT8, StatF, &range_states.front)
LOG_ADD(LOG_UINT8, StatB, &range_states.back)
LOG_ADD(LOG_UINT8, StatU, &range_states.up)
LOG_ADD(LOG_UINT8, StatL, &range_states.left)
LOG_ADD(LOG_UINT8, StatR, &range_states.right)
LOG_GROUP_STOP(mRange)
