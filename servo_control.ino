#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50); // Set the PWM frequency to 50 Hz for servo motors

  // Attach servos to PCA9685 channels
/*  pwm.setPin(9, 0);   // servo1 attached to channel 0
  pwm.setPin(11, 1);  // servo3 attached to channel 1
  pwm.setPin(12, 2);  // servo4 attached to channel 2
  pwm.setPin(13, 3);  // servo5 attached to channel 3*/

  // Set initial positions
    pwm.writeMicroseconds(0, angleToMicroseconds(110));  // servo1 initial position (110 degrees)
    pwm.writeMicroseconds(1, angleToMicroseconds(8));  // servo3 initial position (8 degrees)
    pwm.writeMicroseconds(2, angleToMicroseconds(10));  // servo4 initial position (10 degrees)
    pwm.writeMicroseconds(3, angleToMicroseconds(90));  // servo5 initial position (90 degrees)

  pinMode(8, OUTPUT);
  Serial.begin(9600);
  delay(500);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\r');
    int s1, s3, s4, s5;
    char colour[20];
    char shape[20];
    sscanf(data.c_str(), "%d,%d,%d,%d,%[^,],%s", &s1, &s3, &s4, &s5, colour, shape);
    digitalWrite(8, LOW);

    moveServo(0, s1);   // Move servo1 to the desired angle
    moveServo(1, s3);  // Move servo3 to the desired angle
    moveServo(2, s4);  // Move servo4 to the desired angle
    moveServo(3, s5);  // Move servo5 to the desired angle

    /*Serial.print(s1);
    Serial.print(",");
    Serial.print(s3);
    Serial.print(",");
    Serial.print(s4);
    Serial.print(",");
    Serial.print(s5);
    Serial.print(",");
    Serial.print(colour);
    Serial.print(",");
    Serial.println(shape);*/

    delay(5000);
    // Reset servo positions
    moveServo(0, 110);  // servo1 reset position
    delay(2000);
    moveServo(1, 10);  // servo3 reset position
    delay(2000);
    moveServo(2, 10);  // servo4 reset position
    delay(2000); 

    if (strcmp(colour, "Red") == 0 && strcmp(shape, "Circle") == 0) {
      moveServo(3, 180);  // servo5 position for red circle
      delay(1500);
      digitalWrite(8, HIGH);
      delay(1500);
    }
    else if (strcmp(colour, "Yellow") == 0 && strcmp(shape, "Circle") == 0) {
      moveServo(3, 145);  // servo5 position for yellow circle
      delay(1500);
      digitalWrite(8, HIGH);
      delay(1500);
    }
    else if (strcmp(colour, "Green") == 0 && strcmp(shape, "Circle") == 0) {
      moveServo(3, 110);  // servo5 position for green circle
      delay(1500);
      digitalWrite(8, HIGH);
      delay(1500);
    }
    else if (strcmp(colour, "Green") == 0 && strcmp(shape, "Square") == 0) {
      moveServo(3, 65);  // servo5 position for green square
      delay(1500);
      digitalWrite(8, HIGH);
      delay(1500);
    }
    else if (strcmp(colour, "Yellow") == 0 && strcmp(shape, "Square") == 0) {
      moveServo(3, 35);  // servo5 position for yellow square
      delay(1500);
      digitalWrite(8, HIGH);
      delay(1500);
    }
    else if (strcmp(colour, "Red") == 0 && strcmp(shape, "Square") == 0) {
      moveServo(3, 0);  // servo5 position for red square
      delay(1500);
      digitalWrite(8, HIGH);
      delay(1500);
    }

    moveServo(3, 100);  // Reset servo5 position
    delay(2000);
    data = "";
  }
}

void moveServo(int channel, int angle) {
  int pulse = angleToMicroseconds(angle);
  pwm.writeMicroseconds(channel, pulse);
}

int angleToMicroseconds(int angle) {
  return map(angle, 0, 180, 500, 2500);  // Map angle to pulse length range0 (50 to 2500 microseconds)
}
