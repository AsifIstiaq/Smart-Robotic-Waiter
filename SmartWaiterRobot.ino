#include "config.h"
#include "motor_control.h"
#include "mpu6050_helper.h"
#include "line_follower.h"
#include "obstacle_avoidance.h"
#include "navigation.h"
#include "web_interface.h"

unsigned long lastStateChangeTime = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Smart Waiter Robot booting..."));

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  motorControlInit();
  stopMotors();

  lineFollowerInit();
  obstacleAvoidanceInit();

  mpu6050Init();

  webInterfaceInit();

  robotState = STATE_IDLE;
  currentLoc = DEST_HOME;

  buzzerBeep(120);
  Serial.println(F("Ready. Robot parked at Home."));
}

void loop() {
  webInterfaceLoop();
  navigationStep();
}
