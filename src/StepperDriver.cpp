/*
  The algorithm of the realtime stepper-motor acceleration was taken from the following article:
  https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

  Code by evermake, 2020
*/
#include "Arduino.h"
#include "StepperDriver.h"

#ifndef STEP_PULSE_WIDTH
#define STEP_PULSE_WIDTH 4
#endif

void StepperDriver::m_makeStep() {
  if ((!reverseDirection && m_direction == CLOCKWISE) || (reverseDirection && m_direction == COUNTER_CLOCKWISE))
    digitalWrite(m_dirPin, LOW);
  else
    digitalWrite(m_dirPin, HIGH);
  digitalWrite(m_stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_WIDTH);
  digitalWrite(m_stepPin, LOW);
}

boolean StepperDriver::m_makeStepIfRequired() {
  if (m_stepInterval == 0)
    return false;
  
  uint32_t time = micros();

  if (time - m_lastStepTime >= m_stepInterval) {
    m_makeStep();

    if (m_direction == CLOCKWISE)
      m_currentPosition++;
    else
      m_currentPosition--;

    // Without taking into account the time spent on the step
    m_lastStepTime = time;

    return true;
  }
  return false;
}

void StepperDriver::setUnit(float stepsPerUnit) {
  m_stepsPerUnit = fabs(stepsPerUnit);
}

void StepperDriver::setPosition(int32_t steps) {
  m_currentPosition = m_targetPosition = steps;
  m_stepNum = m_stepInterval = 0;
  m_speed = 0.0;
}

void StepperDriver::setPositionDegrees(float degrees) {
  setPosition(degrees * m_stepsPerDegree);
}

void StepperDriver::setPositionUnits(float units) {
  setPosition(units * m_stepsPerUnit);
}

int32_t StepperDriver::getTargetDistance() {
  return m_targetPosition - m_currentPosition;
}

inline uint32_t StepperDriver::m_getStepsToStop() {
  // Acceleration should always be greater than zero!
  return (m_speed * m_speed) / (2.0 * m_acceleration);
}

void StepperDriver::m_recalculateSpeed() {
  int32_t targetDistance = getTargetDistance();
  int32_t stepsToStop = m_getStepsToStop();

  // We are already at the target position
  if (targetDistance == 0.0 && stepsToStop <= 1) {
    m_stepInterval = 0;
    m_speed = 0.0;
    m_stepNum = 0;
    return;
  }

  // If should rotate clockwise to the target position
  if (targetDistance > 0) {
    // Start to decelerate, if now is accelerating and if the target distance is lower than
    // the stopping distance or if we are in the wrong direction
    if (m_stepNum > 0 && (targetDistance <= stepsToStop || m_direction == COUNTER_CLOCKWISE))
      m_stepNum = -stepsToStop;
    
    // Start to accelerate, if now is decelerating and if we are in the right direction and
    // the target distance is greater than the stopping distance 
    else if (m_stepNum < 0 && (m_direction == CLOCKWISE && targetDistance > stepsToStop))
      m_stepNum *= -1;
  }
  // Else if should rotate counter-clockwise to the target position
  else if (targetDistance < 0) {
    // Start to decelerate, if now is accelerating and if the target distance is lower than
    // the stopping distance or if we are in the wrong direction
    if (m_stepNum > 0 && (-targetDistance <= stepsToStop || m_direction == CLOCKWISE))
      m_stepNum = -stepsToStop;

    // Start to accelerate, if now is decelerating and if we are in the right direction and
    // the target distance is greater than the stopping distance 
    else if (m_stepNum < 0 && (m_direction == COUNTER_CLOCKWISE && -targetDistance > stepsToStop))
      m_stepNum *= -1;
  }

  if (m_stepNum == 0) {
    m_lastStepSize = m_initialStepSize;
    m_direction = (targetDistance > 0) ? CLOCKWISE : COUNTER_CLOCKWISE;
  }
  else {
    // Calculate subsequent step (Equation #13)
    m_lastStepSize = m_lastStepSize - ((2.0 * m_lastStepSize) / (4.0 * m_stepNum + 1));

    if (m_lastStepSize < m_minStepSize)
      m_lastStepSize = m_minStepSize;
  }

  m_stepInterval = m_lastStepSize;
  m_speed = 1000000.0 / m_lastStepSize;
  if (m_direction == COUNTER_CLOCKWISE)
    m_speed *= -1;
  m_stepNum++;
}

void StepperDriver::setMaxSpeed(float maxSpeed) {
  // Max speed must be positive
  if (maxSpeed < 0.0)
    maxSpeed *= -1;

  m_maxSpeed = maxSpeed;

  // Update minimum step size
  m_minStepSize = 1000000.0f / m_maxSpeed;
  
  // Recalculate m_stepNum if moving
  if (m_stepNum != 0) {
    m_stepNum = (m_speed * m_speed) / (2.0f * m_acceleration);
    m_recalculateSpeed();
  }
}

void StepperDriver::setAcceleration(float acceleration) {
  // Acceleration must be non-zero
  if (acceleration == 0.0)
    return;

  // Acceleration must be positive
  if (acceleration < 0.0)
    acceleration *= -1;
  
  if (m_acceleration != acceleration) {
    // Recalculate step num (Equation #17)
    m_stepNum = m_stepNum * (m_acceleration / acceleration);

    m_acceleration = acceleration;

    // Recalculate initial step size (Equation #15)
    m_initialStepSize = 0.676 * sqrt(2.0f / m_acceleration) * 1000000.0f;

    m_recalculateSpeed();
  }
}

void StepperDriver::setSpeed(float speed) {
  m_speed = speed;

  m_speed = constrain(m_speed, -m_maxSpeed, m_maxSpeed);

  if (m_speed == 0.0f) {
    m_stepInterval = 0;
  }
  else {
    m_stepInterval = fabs(1000000.0f / m_speed);
    m_direction = (m_speed > 0) ? CLOCKWISE : COUNTER_CLOCKWISE;
  }
}

inline float StepperDriver::m_clearAngle(float angle) {
  int32_t tmp = angle * 100;
  tmp %= 36000;
  if (tmp < 0) tmp += 36000;
  return tmp / 100.0f;
}

boolean StepperDriver::run() {
  // Make one step if it's time. If the step occurred, recalculate speed.
  if (m_makeStepIfRequired())
    m_recalculateSpeed();

  return m_speed != 0.0 || getTargetDistance() != 0;
}

void StepperDriver::stop(boolean smoothly) {
  if (smoothly) {
    m_targetPosition = m_currentPosition + m_getStepsToStop() * (m_direction == CLOCKWISE ? 1 : -1);
    m_recalculateSpeed();
  }
  else {
    m_targetPosition = m_currentPosition;
    m_speed = 0.0f;
    m_stepInterval = m_stepNum = 0;
  }
}

StepperDriver::StepperDriver(uint32_t stepsPerRev, uint8_t stepPin, uint8_t dirPin) {
  m_stepsPerDegree = stepsPerRev / 360.0f;
  m_stepPin = stepPin;
  m_dirPin = dirPin;

  // Defaults
  reverseDirection = false;
  positioning = RELATIVE_POS;
  m_currentPosition = m_targetPosition = 0;
  m_direction = CLOCKWISE;
  m_maxSpeed = 200.0f;
  m_speed = 0.0f;
  m_stepNum = 0;
  m_initialStepSize = 0.0f;
  m_lastStepSize = 0.0f;
  m_minStepSize = 1.0f;
  setAcceleration(1.0f);

  pinMode(m_stepPin, OUTPUT);
  pinMode(m_dirPin, OUTPUT);
  digitalWrite(m_stepPin, LOW);
  digitalWrite(m_dirPin, LOW);
}
