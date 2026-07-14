#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"


byte lineValue[5];

int lastLineError    = 0;
int previousLineError = 0;
int lastKnownLineSide = 0;

unsigned long lastLinePrint = 0;

inline void lineFollowerInit() {
  pinMode(IR_FAR_LEFT, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FAR_RIGHT, INPUT);
}

inline void readLineSensors() {
  lineValue[0] = digitalRead(IR_FAR_LEFT);
  lineValue[1] = digitalRead(IR_LEFT);
  lineValue[2] = digitalRead(IR_CENTER);
  lineValue[3] = digitalRead(IR_RIGHT);
  lineValue[4] = digitalRead(IR_FAR_RIGHT);
}

inline bool isBlack(byte index) {
  return LINE_IS_BLACK(lineValue[index]);
}

inline byte blackCount() {
  byte count = 0;

  for (byte i = 0; i < 5; i++) {
    if (isBlack(i)) {
      count++;
    }
  }

  return count;
}

inline bool isStopMarker() {
  return isBlack(0) && isBlack(1) && isBlack(2) && isBlack(3) && isBlack(4);
}

inline bool isBranchJunction() {
  if (isStopMarker()) {
    return false;
  }

  return (isBlack(0) || isBlack(4)) && isBlack(2) && (blackCount() >= 3);
}

inline int lineError() {
  long sum = 0;
  byte count = 0;

  if (isBlack(0)) { sum -= 4; count++; }
  if (isBlack(1)) { sum -= 2; count++; }
  if (isBlack(2)) { count++; }
  if (isBlack(3)) { sum += 2; count++; }
  if (isBlack(4)) { sum += 4; count++; }

  if (count == 0) {
    return 99;
  }

  int error = sum / count;

  if (error < 0) {
    lastKnownLineSide = -1;
  } else if (error > 0) {
    lastKnownLineSide = 1;
  }

  lastLineError = error;

  return error;
}

inline void printLineDebug(const char *stateLabel) {
  if (millis() - lastLinePrint < LINE_PRINT_MS) {
    return;
  }

  lastLinePrint = millis();
  Serial.print("Sensors: ");

for (int i = 0; i < 5; i++) {
    Serial.print(isBlack(i) ? "B " : "W ");
}

Serial.print(" | Error=");
Serial.print(lastLineError);

Serial.print(" | Count=");
Serial.println(blackCount());
}

inline bool lineFollowPIDStep() {
  byte count = blackCount();

  if (count == 0) {
    return false;
  }

  if (millis() - lastLinePrint == 0) {
  }

  if (isBlack(0) && !isBlack(4)) {
    lastKnownLineSide = -1;
    lastLineError = -4;
    setMotors(-LINE_PIVOT_REVERSE, LINE_PIVOT_FAST);
    return true;
  }

  if (isBlack(4) && !isBlack(0)) {
    lastKnownLineSide = 1;
    lastLineError = 4;
    setMotors(LINE_PIVOT_FAST, -LINE_PIVOT_REVERSE);
    return true;
  }

  int error = lineError();
  int derivative = error - previousLineError;
  previousLineError = error;

  int correction = (LINE_KP * error) + (LINE_KD * derivative);

  int leftSpeed  = LINE_BASE_SPEED + correction;
  int rightSpeed = LINE_BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed, LINE_MIN_SPEED, LINE_MAX_SPEED);
  rightSpeed = constrain(rightSpeed, LINE_MIN_SPEED, LINE_MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
  return true;
}

inline bool isLineCentered() {
    return
        (isBlack(1) && isBlack(2) && isBlack(3)) ||
        (isBlack(0) && isBlack(1) && isBlack(2)) ||
        (isBlack(2) && isBlack(3) && isBlack(4));
}

#endif
