#include <agv/range_sensor.h>
#include <wiringPi.h>
#include <chrono>
#include <string>

#define SENSOR_TIMEOUT 250000 // 0.5 sec
#define TRIG 4
#define ECHO 5
#define SMOOTH_FACTOR 2

RangeSensor::RangeSensor()
        : smoothDistance_(0)
        {
                if (wiringPiSetup() == -1)
                {
                        exit(EXIT_FAILURE);
                }
                pinMode(TRIG, OUTPUT);
                pinMode(ECHO, INPUT);
        }

float RangeSensor::getDistance()
        {
                long startTime = 0;
                long stopTime = 0;

                // Ensure trigger is low.
                digitalWrite(TRIG, LOW);
                delay(50);

                // Trigger the ping.
                digitalWrite(TRIG, HIGH);
                delayMicroseconds(10);
                digitalWrite(TRIG, LOW);

                // Wait for ping response, or timeout.
                startTime = micros();
                while (digitalRead(ECHO) == LOW && micros() - startTime < SENSOR_TIMEOUT);

                // Cancel on timeout.
                if (micros() - startTime > SENSOR_TIMEOUT)
                {
                        printf("No response from sensor\n");
                        return -1;
                }

                startTime = micros();
                stopTime = micros();
                // Wait for pong response, or timeout.
                while (digitalRead(ECHO) == HIGH && micros() - stopTime < SENSOR_TIMEOUT);

                // Cancel on timeout.
                if (micros() - stopTime > SENSOR_TIMEOUT)
                {
                        printf("Out of range\n");
                        return -1;
                }

                stopTime = micros();

                // Convert ping duration to distance.
                float distance = (float)(stopTime - startTime) * 0.017150;
                smoothDistance_ = smoothe(distance, smoothDistance_);
                return smoothDistance_;
        }

float RangeSensor::smoothe(const float input, const float data)
        {
                return (data == 0) ? input : ((data * (SMOOTH_FACTOR - 1) + input) / SMOOTH_FACTOR);
        }