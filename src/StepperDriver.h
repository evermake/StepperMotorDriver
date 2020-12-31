/*
  By evermake, 2020
*/
#ifndef StepperDriver_h
#define StepperDriver_h

#include "Arduino.h"


enum PositioningMode {
  RELATIVE_POS, ABSOLUTE_POS
};


class StepperDriver {
  public:
    // StepperDriver class constructor
    // @param[in] stepsPerRev Number of motor steps per one revolution
    // @param[in] stepPin Step pin of the stepper-motor driver
    // @param[in] dirPin Direction pin of the stepper-motor driver
    StepperDriver(uint32_t stepsPerRev, uint8_t stepPin, uint8_t dirPin);

    // Sets the number of steps per unit of measurement
    void setUnit(float stepsPerUnit);

    // Direction pin inversion
    boolean reverseDirection = false;

    // Default positioning mode for the functions
    PositioningMode positioning;

    // This is main function that drives the motor. 
    // Call it as often as possible, but at least once per step.
    // @returns true, if the motor is moving to the target position
    boolean run();

    // Sets the maximum speed
    // @param[in] New maximum speed (in fs per second)
    void setMaxSpeed(float maxSpeed);

    // Sets acceleration
    // @param[in] New acceleration (in steps per second)
    void setAcceleration(float acceleration);

    // Sets new speed
    // @param[in] New speed (in steps per second)
    void setSpeed(float speed);

    // Sets the current position
    // @param[in] steps The position in steps which should be set
    void setPosition(int32_t steps);

    // Sets the current position
    // @param[in] degrees The position in degrees which should be set
    void setPositionDegrees(float degrees);

    // Sets the current position
    // @param[in] units The position in units which should be set
    void setPositionUnits(float units);

    // @returns Distance to the target position regarding the current (in steps)
    int32_t getTargetDistance();

    // Rotate the motor shaft on the specified units in absolute/relative positioning
    // @param[in] degrees The position to which the shaft should be moved 
    // (in units which were set by the setUnits function)
    void move(float units);

    // Rotate the motor shaft on the specified degrees in absolute/relative positioning
    // @param[in] degrees The position to which the shaft should be moved (in degrees)
    void rotate(float degrees);

    // Turns the motor shaft at the specified angle. No more than a half turn will be made.
    // @param[in] angle The angle to which the shaft should be turned (in degrees)
    void rotateAtDegree(float angle);

    // Stops the motor
    // @param smoothly if true, the stop will be smooth, otherwise instant
    void stop(boolean smoothly);

  private:
    enum MovementDirection {
      CLOCKWISE, COUNTER_CLOCKWISE
    };

    // Current movements direction
    MovementDirection m_direction;

    // Number of steps per degree
    float m_stepsPerDegree;

    // Number of steps per specified unit
    float m_stepsPerUnit;

    // Step pin of the stepper motor driver
    uint8_t m_stepPin;

    // Dir pin of the stepper motor driver
    uint8_t m_dirPin;

    // Current speed (in steps per second)
    float m_speed;

    // Maximum speed (in steps per second)
    float m_maxSpeed;

    // Current acceleration/deceleration (in steps per second)
    float m_acceleration;

    // Current interval between steps (in microseconds)
    uint32_t m_stepInterval;

    // Time of the last step
    uint32_t m_lastStepTime;

    // Step counter for acceleration calculation
    int32_t m_stepNum;

    // Initial step size (in microseconds)
    float m_initialStepSize;

    // Size of the last step (in microseconds)
    float m_lastStepSize;

    // Minimum step size (in microseconds) based on max speed
    float m_minStepSize;

    // Current absolute position (in steps)
    int32_t m_currentPosition;

    // Target position (in steps)
    int32_t m_targetPosition;

    // Make one step
    void m_makeStep();

    // Check is it required to make step and do it if is
    // @returns true, if a step occurred
    boolean m_makeStepIfRequired();

    // Update current speed, step interval, etc.
    void m_recalculateSpeed();

    // @returns Number of steps required for the motor to stop smoothly
    // @note Equation #16
    inline uint32_t m_getStepsToStop();

    // Convert angle (in degrees) to range 0 ~ 360
    inline float m_clearAngle(float angle);
};


#endif
