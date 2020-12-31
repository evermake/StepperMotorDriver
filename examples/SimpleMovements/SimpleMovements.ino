#include "StepperDriver.h"

// Create motor object (steps per revolution = 200, step pin = 3, dir pin = 4)
StepperDriver motor = StepperDriver(200, 3, 4);

void setup() {
  // Use absolute positioning
  motor.positioning = ABSOLUTE_POS;
  // Set current position to the zero
  motor.setPosition(0);
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
