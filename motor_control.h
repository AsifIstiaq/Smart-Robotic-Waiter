#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

inline void motorControlInit() {
  pinMode(IN1_LEFT_DIR, OUTPUT);
  pinMode(IN2_LEFT_DIR, OUTPUT);
  pinMode(IN3_RIGHT_DIR, OUTPUT);
  pinMode(IN4_RIGHT_DIR, OUTPUT);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(ENA_LEFT_PWM, LEFT_PWM_CHANNEL);

  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(ENB_RIGHT_PWM, RIGHT_PWM_CHANNEL);

  digitalWrite(IN1_LEFT_DIR, LOW);
  digitalWrite(IN2_LEFT_DIR, LOW);
  digitalWrite(IN3_RIGHT_DIR, LOW);
  digitalWrite(IN4_RIGHT_DIR, LOW);

  ledcWrite(LEFT_PWM_CHANNEL, 0);
  ledcWrite(RIGHT_PWM_CHANNEL, 0);
}

inline void setLeftMotor(int speedValue) {
  int pwm = constrain(abs(speedValue), 0, 255);

  if (speedValue > 0) {
    digitalWrite(IN1_LEFT_DIR, HIGH);
    digitalWrite(IN2_LEFT_DIR, LOW);
  } else if (speedValue < 0) {
    digitalWrite(IN1_LEFT_DIR, LOW);
    digitalWrite(IN2_LEFT_DIR, HIGH);
  } else {
    digitalWrite(IN1_LEFT_DIR, LOW);
    digitalWrite(IN2_LEFT_DIR, LOW);
  }

  ledcWrite(LEFT_PWM_CHANNEL, pwm);
}

inline void setRightMotor(int speedValue) {
  int pwm = constrain(abs(speedValue), 0, 255);

  if (speedValue > 0) {
    digitalWrite(IN3_RIGHT_DIR, HIGH);
    digitalWrite(IN4_RIGHT_DIR, LOW);
  } else if (speedValue < 0) {
    digitalWrite(IN3_RIGHT_DIR, LOW);
    digitalWrite(IN4_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(IN3_RIGHT_DIR, LOW);
    digitalWrite(IN4_RIGHT_DIR, LOW);
  }

  ledcWrite(RIGHT_PWM_CHANNEL, pwm);
}

extern volatile bool emergencyStop;

inline void setMotors(int leftSpeed, int rightSpeed) {
    if (emergencyStop) {
    setLeftMotor(0);
    setRightMotor(0);
    return;
  }
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

inline void stopMotors() {
  setMotors(0, 0);
}

inline void buzzerBeep(unsigned long ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

inline void brakeTurn(char direction) {
  if (direction == 'R') {
    setMotors(-TURN_BRAKE_SPEED, TURN_BRAKE_SPEED);
  } else {
    setMotors(TURN_BRAKE_SPEED, -TURN_BRAKE_SPEED);
  }

  delay(TURN_BRAKE_MS);
  stopMotors();
  delay(200);
}

#endif
