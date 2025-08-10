// Test when sing the fried board, had to connect to a buck converter athen to the 5V and G pinss on the driverboard
#include <Stepper.h>  // Include built-in Arduino Stepper library

// Number of steps per full revolution of the 28BYJ-48 stepper motor
int stepsPerRevolution = 2048;  

// Motor speed in RPM (rotations per minute)
int rpm = 20;  

// Initialalize stepper library using Arduino pins 8, 10, 9, and 11
// pin order: IN1, IN3, IN2, IN4
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  // Set motor speed
  myStepper.setSpeed(rpm);
}

void loop() {
  // Spin one full rotation clockwise (2048 steps)
  myStepper.step(stepsPerRevolution);
  delay(0);  // Pause for 0.5 seconds

  // Spin one full rotation counter-clockwise
  myStepper.step(-stepsPerRevolution);
  delay(0);  // Pause again
}
