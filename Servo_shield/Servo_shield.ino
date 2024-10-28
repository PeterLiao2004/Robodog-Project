#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int potPin = A0; // Pin where the potentiometer is connected
int potValue = 0;      // Variable to store potentiometer value
int servoMin = 150;    // Minimum PWM value for servos
int servoMax = 600;    // Maximum PWM value for servos

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  pwm.begin();         // Initialize the PWM driver
  pwm.setPWMFreq(60);  // Set frequency to 60 Hz for servos
}

void loop() {
  // Read the potentiometer value (0 to 1023)
  potValue = analogRead(potPin);

  // Map potentiometer value to servo range (150 to 600)
  int pwmValue = map(potValue, 0, 1023, servoMin, servoMax);

  //Control three servos on channels
  pwm.setPWM(0, 0, 329);  // Control servo on channel Elbow Rear Left
  pwm.setPWM(1, 0, 460);  // Control servo on channel Sholder Rear Left
  pwm.setPWM(2, 0, 440);  // Control servo on channel Joint Rear Left

  pwm.setPWM(4, 0, 260);  // Control servo on channel Elbow Front Left
  pwm.setPWM(5, 0, 479);  // Control servo on channel Sholder Front Left
  pwm.setPWM(6, 0, 375);  // Control servo on channel Joint Front Left

  pwm.setPWM(9, 0, 438);  // Control servo on channel Elbow Front Right
  pwm.setPWM(10, 0, 270);  // Control servo on channel Sholder Front Right
  pwm.setPWM(11, 0, 375);  // Control servo on channel Joint Front Right

  pwm.setPWM(13, 0, 442);  // Control servo on channel Elbow Rear Right
  pwm.setPWM(14, 0, 231);  // Control servo on channel Sholder Rear Right
  pwm.setPWM(15, 0, 309);  // Control servo on channel Joint Rear Right


    

  // Print the mapped value for debugging
  Serial.print("Potentiometer: ");
  Serial.print(potValue);
  Serial.print(" -> PWM: ");
  Serial.println(pwmValue);

  delay(50);  // Small delay to smooth out the potentiometer reading
}
