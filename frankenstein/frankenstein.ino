#include "Servo.h"
#include <LedControl.h>
#include <cmath>

// *********** For arms ***********
#define RIGHT_BUTTON_PIN 2
#define LEFT_BUTTON_PIN 3
#define UP_BUTTON_PIN 4
#define DOWN_BUTTON_PIN 5

#define UP_SERVO_PIN 9
#define DOWN_SERVO_PIN 8

#define INVALID_POSITION -10

float a1 = 3.5;
float a2 = 2.5;
float originalX = 1;
float originalY = 5.5;

int prevLeftButtonState = LOW;
int prevRightButtonState = LOW;
int prevDownButtonState = LOW;
int prevUpButtonState = LOW;

float leftX = 0;
float rightX = 0;
float downY = 0;
float upY = 0;

float prevLeftX = leftX;
float prevRightX = rightX;
float prevDownY = downY;
float prevUpY = upY;

float maxMoveSpeed = 0.004;
float minMoveSpeed = 0.002;
float moveSpeed = (minMoveSpeed + maxMoveSpeed) / 2;
long prevMoveTime = 0;

// *********** For LED Matrix ***********
#define DIN_PIN 12
#define CS_PIN 10
#define CLK_PIN 11

Servo upMotor;
Servo downMotor;

LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);

int prevFaceIndex = 0;

byte expressions[8][8] = {
  { B00000000,  // Smile Face!
    B01100110,
    B01100110,
    B00000000,
    B10000001,
    B01000010,
    B00111100,
    B00000000 },
  { B00000000,  // Normal Face!
    B01100110,
    B01100110,
    B00000000,
    B00000000,
    B01111110,
    B00000000,
    B00000000 },
  { B00000000,  // Sad Face!
    B01100110,
    B01100110,
    B00000000,
    B00111100,
    B01000010,
    B10000001,
    B00000000 }
};

void setup() {
  Serial.begin(9600);

  upMotor.attach(UP_SERVO_PIN);
  downMotor.attach(DOWN_SERVO_PIN);

  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(LEFT_BUTTON_PIN, INPUT);
  pinMode(UP_BUTTON_PIN, INPUT);
  pinMode(DOWN_BUTTON_PIN, INPUT);

  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  displayFace(expressions[prevFaceIndex]);
}

void loop() {
  float analogValue = analogRead(A0);
  moveSpeed = mapFloat(analogValue, 0.0, 1100.0, minMoveSpeed, maxMoveSpeed);
  moveSpeed = constrain(moveSpeed, minMoveSpeed, maxMoveSpeed);

  updateValueOnButtonPress(LEFT_BUTTON_PIN, prevLeftButtonState, leftX, moveSpeed);
  updateValueOnButtonPress(RIGHT_BUTTON_PIN, prevRightButtonState, rightX, moveSpeed);
  updateValueOnButtonPress(UP_BUTTON_PIN, prevUpButtonState, upY, moveSpeed);
  updateValueOnButtonPress(DOWN_BUTTON_PIN, prevDownButtonState, downY, moveSpeed);
  float currentX = originalX + rightX - leftX;
  float currentY = originalY + upY - downY;

  float *angles = inverseKinematics(currentX, currentY, a1, a2);
  float t1 = angles[0];
  float t2 = angles[1];
  float angle1 = radiansToDegrees(t1);
  float angle2 = radiansToDegrees(t2) + angle1;
  float servoAngle1 = 180 - angle1;
  float servoAngle2 = 180 - angle2;
  int faceIndex = 0;

  // Display sad face if out of range, and restore the right positions.
  if (t1 == INVALID_POSITION || !isValidServoAngle(servoAngle1) || !isValidServoAngle(servoAngle2)) {
    faceIndex = 1;
    rightX = prevRightX;
    leftX = prevLeftX;
    upY = prevUpY;
    downY = prevDownY;
  } else {
    faceIndex = 0;
    // Display smile face if in range, and move the arms.
    if (millis() - prevMoveTime > 20) {
      downMotor.write(servoAngle1);
      upMotor.write(servoAngle2);
      prevMoveTime = millis();
    }
  }

  if (prevFaceIndex != faceIndex) displayFace(expressions[faceIndex]);
  prevFaceIndex = faceIndex;

  // Store current positions for restoring.
  prevRightX = rightX;
  prevLeftX = leftX;
  prevUpY = upY;
  prevDownY = downY;

  delete[] angles;
}

void updateValueOnButtonPress(int pin, int &prevButtonState, float &value, float moveSpeed) {
  int buttonState = digitalRead(pin);
  if (buttonState == prevButtonState && buttonState == LOW) value += moveSpeed;
  prevButtonState = buttonState;
}

void displayFace(byte *face) {
  lc.clearDisplay(0);
  for (int i = 0; i < 8; i++) lc.setColumn(0, 7 - i, face[i]);
}

bool isValidServoAngle(float angle) {
  return angle >= 0 && angle <= 180;
}

float radiansToDegrees(float radians) {
  return radians * (180.0 / M_PI);
}

float mapFloat(float value, float domainMin, float domainMax, float rangeMin, float rangeMax) {
  if (domainMin == domainMax) return rangeMin;
  return rangeMin + (value - domainMin) * (rangeMax - rangeMin) / (domainMax - domainMin);
}

/**
 * Compute the angles (in radians) for given position and lengths.
 *
 * px: the x of the expected position
 * py: the y of the expected position
 * a1: the length of the bottom arm
 * a2: the length of the up arm
 *
 * angles[0]: the angle for the bottom arm relative to ground
 * angles[1]: the angle for the up arm relative to the bottom arm
 */
float *inverseKinematics(float px, float py, float a1, float a2) {
  float *angles = new float[2];

  float c2 = (px * px + py * py - a2 * a2 - a1 * a1) / (2 * a1 * a2);

  // Out of range.
  if (c2 > 1 || c2 < -1) {
    angles[0] = INVALID_POSITION;
    angles[0] = INVALID_POSITION;
    return angles;
  }

  float s2 = -1 * sqrt(1 - c2 * c2);
  float t2 = atan2(s2, c2);
  float t1 = atan2(py, px) - atan2(a2 * s2, a1 + a2 * c2);

  angles[0] = t1;
  angles[1] = t2;
  return angles;
}
