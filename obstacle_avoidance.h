#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"

inline void obstacleAvoidanceInit() {

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  digitalWrite(ULTRASONIC_TRIG, LOW);

  Serial.println("Obstacle Avoidance Initialized");
}

inline long readDistanceCM() {

  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(ULTRASONIC_TRIG, LOW);

  unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000UL);

  if (duration == 0) {
    return -1;
  }

  return duration / 58;
}

inline void sonarDebug() {

  long distance = readDistanceCM();

  Serial.print("Distance: ");

  if (distance < 0)
    Serial.println("No Echo");
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
}

inline bool obstacleDetected() {

  long distance = readDistanceCM();

  if (distance < 0)
    return false;

  return (distance <= OBSTACLE_STOP_CM);
}

inline void avoidObstacle() {

  Serial.println("Obstacle Detected!");

  stopMotors();

  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  buzzerBeep(150);

  while (true) {

    long distance = readDistanceCM();

    Serial.print("Waiting... Distance = ");

    if (distance < 0)
      Serial.println("No Echo");
    else {
      Serial.print(distance);
      Serial.println(" cm");
    }

    if (distance < 0 || distance > OBSTACLE_STOP_CM) {
      break;
    }

    delay(200);
  }

  Serial.println("Obstacle Cleared");

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
}

#endif