#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <esp_now.h>

#include <L298NX2.h>
#include <TinyGPS++.h>

#include "structures.h"
#include "config.h"

// External references so the .cpp can see variables in the .ino (or vice versa)
extern L298NX2 motors;
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;
extern TelemetryMessage telemetry;
extern SemaphoreHandle_t telemetryMutex;
extern uint8_t broadcastAddress[];

extern volatile long pulseCountA;
extern volatile long pulseCountB;

// Task Function Prototypes
void UpdateGPSPositionTask(void *parameter);
void SendTelemetryTask(void *parameter);

#endif