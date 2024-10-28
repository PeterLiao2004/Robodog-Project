#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define minimum and maximum pulse lengths for the servos
#define SERVOMIN  150 // Minimum pulse length count (calibrate this value)
#define SERVOMAX  600 // Maximum pulse length count (calibrate this value)

// Knob
#define KnobPin A0

// Specific servo indices to be used
const int servoIndices[12] = {0, 1, 2, 4, 5, 6, 9, 10, 11, 13, 14, 15}; // List of servo channels to use

// Array for servo base positions, directions, and other configurations
  // Control servo on channel Elbow Rear Left (0)
  // Control servo on channel Sholder Rear Left (1)
  // Control servo on channel Joint Rear Left (2)

  // Control servo on channel Elbow Front Left (4)
  // Control servo on channel Sholder Front Left (5)
  // Control servo on channel Joint Front Left (6)

  // Control servo on channel Elbow Front Right (9)
  // Control servo on channel Sholder Front Right (10)
  // Control servo on channel Joint Front Right (11)

  // Control servo on channel Elbow Rear Right (13)
  // Control servo on channel Sholder Rear Right (14)
  // Control servo on channel Joint Rear Right (15)


const int directions[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // Reflection correction for opposite sides
const int base[12] = {72, 129, 116, 40, 131, 90, 115, 48, 90, 117, 32, 63}; // Base positions for each servo

int currentPosition[12];  // Tracks the current position of each servo

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit PWM Servo Driver test!");

  // Initialize the PWM driver with the default address (0x40)
  pwm.begin();

  // Set the PWM frequency to 60 Hz, which is typical for servos
  pwm.setPWMFreq(60);

  // Small delay for the settings to take effect
  delay(10);

  //Set servos to their base positions
  for (int i = 0; i < 12; i++) {
    setServoAngle(i, base[i]);
  }
}


void loop() {
  moveServoSlowly(0, 72, 20);
  moveServoSlowly(9, 90, 20);
  delay(1000);
  moveServoSlowly(0, 100, 20);
  moveServoSlowly(9, 130, 20);
  delay(1000);

}

// Function to set servo angle using Adafruit PWM servo driver
void setServoAngle(int index, int angle) {
  // Correct direction based on the direction array
  if (directions[index] == -1) {
    angle = 180 - angle;
  }
  
  // Map angle (0-180) to pulse length
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Set the servo channel (use servoIndices array for channel number)
  pwm.setPWM(servoIndices[index], 0, pulse);
}

  
// Function to move servo slowly from current angle to target angle
void moveServoSlowly(int index, int targetAngle, int delayTime) {
  int currentAngle = currentPosition[index];  // Start from the servo's current position
  int step = (targetAngle > currentAngle) ? 1 : -1;  // Determine the direction of movement

  // Move the servo gradually to the target angle
  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    setServoAngle(index, angle);  // Set the servo to the current angle
    delay(delayTime);  // Small delay to make the movement slow and smooth
  }
  
  // Update the current position once the servo reaches the target angle
  currentPosition[index] = targetAngle;
}