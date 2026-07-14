#ifndef MPU6050_HELPER_H
#define MPU6050_HELPER_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"

bool mpuReady = false;

int16_t AcX, AcY, AcZ;
int16_t Tmp;
int16_t GyX, GyY, GyZ;

float gyroOffsetZ = 0;

float yawDeg = 0;

unsigned long lastGyroMicros = 0;

inline bool readMPU()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0)
        return false;

    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

    if (Wire.available() != 14)
        return false;

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    Tmp = Wire.read() << 8 | Wire.read();

    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    return true;
}

inline bool wakeMPU6050()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    return Wire.endTransmission() == 0;
}

inline void calibrateGyroZ()
{
    Serial.println(F("Keep robot still..."));
    Serial.println(F("Calibrating MPU6050..."));

    const int samples = 1000;
    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
        if (readMPU())
            sum += GyZ;

        delay(2);
    }

    gyroOffsetZ = (float)sum / samples;

    Serial.print(F("Gyro Z Offset = "));
    Serial.println(gyroOffsetZ);
}

inline void mpu6050Init()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    mpuReady = wakeMPU6050();

    if (!mpuReady)
    {
        Serial.println(F("MPU6050 NOT FOUND"));
        return;
    }

    Serial.println(F("MPU6050 Connected"));

    calibrateGyroZ();

    lastGyroMicros = micros();
    yawDeg = 0;
}

inline void resetYaw()
{
    yawDeg = 0;
    lastGyroMicros = micros();
}

inline void updateYaw()
{
    if (!mpuReady)
        return;

    if (!readMPU())
        return;

    unsigned long now = micros();

    float dt = (now - lastGyroMicros) / 1000000.0f;

    lastGyroMicros = now;

    float gz = (GyZ - gyroOffsetZ) / 131.0f;

    if (fabs(gz) < GYRO_DEADBAND_DPS)
        gz = 0;

    yawDeg += gz * dt * TURN_CORRECTION;
}

inline float getYaw()
{
    return yawDeg;
}

inline float getWrappedYaw()
{
    float wrapped = yawDeg;
    while (wrapped > 180.0f)  wrapped -= 360.0f;
    while (wrapped < -180.0f) wrapped += 360.0f;
    return wrapped;
}

inline float getStopAngle(char direction, float requestedAngle)
{
    if (requestedAngle >= 170 && requestedAngle <= 190)
    {
        return direction == 'R'
                   ? RIGHT_180_STOP_ANGLE
                   : LEFT_180_STOP_ANGLE;
    }

    if (requestedAngle >= 85 && requestedAngle <= 95)
    {
        return direction == 'R'
                   ? RIGHT_90_STOP_ANGLE
                   : LEFT_90_STOP_ANGLE;
    }

    return requestedAngle * 0.90f;
}

#endif