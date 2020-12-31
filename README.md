# StepperMotorDriver

## Quick start
Firstly, include the library
```cpp
#include "StepperDriver.h"
```

To create the motor object use the following constructor:
```cpp
StepperDriver(uint32_t stepsPerRev, uint8_t stepPin, uint8_t dirPin);
```

E.g. if you want to use motor with 1000 steps per revolution, with step pin connected to pin 3 and with dir pin connected to port 4, create the motor object like so:
```cpp
StepperDriver motor = StepperDriver(1000, 3, 4);
```

To make the motor turning, you need to call `run()` method of the object.
**Without calling this method the motor will not move.**
```cpp
// This is the main function that drives the motor. 
// Call it as often as possible, but at least once per step.
// @returns true, if the motor is moving to the target position
bool run();
```

There are three basic functions to move the motor:
```cpp
// Rotate the motor shaft on the specified units in absolute/relative positioning
void move(float units);

// Rotate the motor shaft on the specified degrees in absolute/relative positioning
void rotate(float degrees);

// Turns the motor shaft at the specified angle. No more than a half turn will be made.
void rotateAtDegree(float angle);
```

First two functions can work in relative or absolute positioning. You can specify the positioning mode like this:
```cpp
// Use absolute positioning
motor.positioning = ABSOLUTE_POS;

// Use relative positioning
motor.positioning = RELATIVE_POS;
```
Example: if you call `rotate(360)` twice in relative positioning mode, motor will make 2 revolutions, but in absolute positioning mode only one revolution will be done.

To set the current position use the `setPosition()` method:
```cpp
// Set current position to the zero
  motor.setPosition(0);
```

The third function (`rotateAtDegree`) will rotate the motor shaft at the specified angle (0 - 360). This function will make no more than a half turn.

To set motor speed and acceleration you can use the following functions:
```cpp
// Sets the maximum speed
// @param[in] New maximum speed (in fs per second)
void setMaxSpeed(float maxSpeed);

// Sets acceleration
// @param[in] New acceleration (in steps per second)
void setAcceleration(float acceleration);

// Sets new speed
// @param[in] New speed (in steps per second)
void setSpeed(float speed);
```

### Complete example
```cpp
#include "StepperDriver.h"

// Create motor object (steps per revolution = 200, step pin = 3, dir pin = 4)
StepperDriver motor = StepperDriver(200, 3, 4);

void setup() {
  // Use absolute positioning
  motor.positioning = ABSOLUTE_POS;
  // Set current position to the zero
  motor.setPosition(0);

  // Set speeds
  motor.setSpeed(0);
  motor.setMaxSpeed(200);
  motor.setAcceleration(50);
}

void loop() {
  // Rotate motor to position 720 degrees
  motor.rotate(720.0f);
  while (motor.run());

  // Wait 1 second
  delay(1000);

  // Rotate motor to position -720 degrees
  // (the motor will make 4 turns in the opposite direction)
  motor.rotate(-720.0f);
  while (motor.run());

  // Wait 1 second
  delay(1000);
}
```