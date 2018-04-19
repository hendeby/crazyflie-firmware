/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

#include "motors.h"
#include "pm.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

const float LOW_BATTERY = 3.3f;

static void stabilizerTask(void* param);

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  stateControllerInit();
  powerDistributionInit();
  if (estimator == kalmanEstimator)
  {
    sitAwInit();
  }

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

#define YAW_CONTROL_MODE
#ifdef YAW_CONTROL_MODE
static int yawCtrlMode = 0;  /* 0 is inactive, higher numbers for different modes of control */
static float yawCtrlOffset = 0.0;  /* When the control is active, we let all motors get this PWM duty cycle signal as a base. This value should be 0-1 */
static float yawCtrlKP = 0;  /* P gain in the PID */
static float yawCtrlKI = 0;  /* I gain in the PID */
static float yawCtrlKD = 0;  /* D gain in the PID */
static float yawCtrlRef = 10;  /* The reference/target yaw angle in degrees */
static float yawU = 0;  /* Control signal for yaw control. */
static float yawError = 0;  /* Yaw control error. */

static void yawPowerDistribution(const float u[4]);
static float limitDutyCycle(float u);

void yawPowerDistribution(const float u[4]) {
  // Limit the control signals to ensure that they are
  // i)  valid and
  // ii) does not make the drone fly away
  // Then map the control signals from numbers 0-1 to 0-65535, and send to motors
  for (int i=0; i<4; ++i)
    motorsSetRatio(i, (int16_t) (65535*limitDutyCycle(u[i])));
}

float limitDutyCycle(float u)
{
  // Props cannot turn the other way
  if (u < 0) u = 0;

  // Make sure they do not spin so fast that the drone lifts
  float maxDuty = 0.6;
  if (u > maxDuty) u = maxDuty;

  return u;
}
#endif

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;
  
  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    getExtPosition(&state);
    stateEstimator(&state, &sensorData, &control, tick);

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &setpoint, &sensorData, &state, tick);

#ifdef YAW_CONTROL_MODE
    // Switch sign of reference point every N ticks.
    // This allows us to test the controller without having to
    // touch the drone to change its angle or enter different reference
    // values
    if (tick % 3000 == 0) {
      yawCtrlRef = -yawCtrlRef;
    }

    // Calculate the control error and make sure that the angle error stays
    // within the interval (-180,180]. Otherwise, you might get error 350 degs
    // when the reference signal is 175 and the current yaw is -175, when
    // in fact the error is -10.
    yawError = yawCtrlRef - state.attitude.yaw;
    while (yawError > 180)
       yawError -= 360;
    while (yawError < -180)
       yawError += 360;

    // Here we can define a number of different controllers (modes).
    // mode=0 is assumed to refer to the case where you do not do anything

    // The putput of our controller will be put into the array u.
    // It is assumed that we work with the duty cycle, i.e. a number between
    // 0 and 1 as the control signal. This will then map that to the
    // PWM values 0-65535 at the end.
    //
    // We set the control signal to 0 to start with
    float u[4] = { 0., 0., 0., 0. };
    if (yawCtrlMode == 1) {
      // The following 4 lines will make all props spin with the same
      // speed, defined by the parameter yawCtrlOffset which you can set in
      // the Parameter tab.
      //
      // You can use this controller by etting yawCtrlMode to 1 in the
      // Parameter tab in the Crazyflie Client
      u[0] = yawCtrlOffset;
      u[1] = yawCtrlOffset;
      u[2] = yawCtrlOffset;
      u[3] = yawCtrlOffset;
    } else if (yawCtrlMode == 2) {
      //
      // YOUR JOB IS TO CREATE A FEEDBACK CONTROLLER here, i.e. an
      // algorithm that makes the yawError defined above go to zero by
      // an appropriate choice of control signals to motors M1-M4. In other
      // words calculate u[0], ..., u[3] based on yawError and remember that the
      // values should be 0-1, where 0 means motors completely off and 1 means
      // max thrust.
      //
      // The controller should work for different values of yawCtrlRef
      //
      // You would activet this controller by setting yawCtrlMode to 2 in
      // the Parameter tab in the Crazyflie Client
    }
#endif

    checkEmergencyStopTimeout();

    // Perform emergency stop is battery level is too low!
    // Or the battery might be damaged.
    emergencyStop |= pmGetBatteryVoltageMin() < LOW_BATTERY;

    if (emergencyStop)
      powerStop();
#ifdef YAW_CONTROL_MODE
    else if (yawCtrlMode) // If active, execute our control law
      yawPowerDistribution(u);
#endif
    else  // Execute default control law
      powerDistribution(&control);

    tick++;
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

#ifdef YAW_CONTROL_MODE
PARAM_GROUP_START(yawCtrlPar)
PARAM_ADD(PARAM_UINT8, mYawCtrlMode, &yawCtrlMode)
PARAM_ADD(PARAM_FLOAT, myawCtrlOffset, &yawCtrlOffset)
PARAM_ADD(PARAM_FLOAT, mYawCtrlKP, &yawCtrlKP)
PARAM_ADD(PARAM_FLOAT, mYawCtrlKI, &yawCtrlKI)
PARAM_ADD(PARAM_FLOAT, mYawCtrlKD, &yawCtrlKD)
PARAM_ADD(PARAM_FLOAT, mYawCtrlRef, &yawCtrlRef)
PARAM_GROUP_STOP(yawCtrlPar)

LOG_GROUP_START(yawCtrlLog)
LOG_ADD(LOG_FLOAT, lU, &yawU)
LOG_ADD(LOG_FLOAT, lError, &yawError)
LOG_ADD(LOG_UINT8, lYawCtrlMode, &yawCtrlMode)
LOG_ADD(LOG_FLOAT, lYaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, lYawRef, &yawCtrlRef)
LOG_ADD(LOG_FLOAT, lYaw, &state.attitude.yaw)
LOG_GROUP_STOP(yawCtrlLog)
#endif

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_GROUP_STOP(stateEstimate)
