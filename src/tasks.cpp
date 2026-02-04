#include <tasks.h>

void UpdateGPSPositionTask(void *parameter) {
  for (;;) {
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (gps.location.isUpdated()) {
          if (xSemaphoreTake(telemetryMutex, MUTEX_TIMEOUT)){
            telemetry.lat = gps.location.lat();
            telemetry.lon = gps.location.lng();
            telemetry.gpsSpeed = gps.speed.mps();

            Serial.printf("%d, %d\n", telemetry.lat, telemetry.lon);
            xSemaphoreGive(telemetryMutex);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to CPU for 10ms
  }
}

void SendTelemetryTask(void *parameter) {
  unsigned long lastTime = millis();
  TelemetryMessage localCopy; // Buffer for sending

  for (;;) { // Infinite loop
    if (xSemaphoreTake(telemetryMutex, MUTEX_TIMEOUT)){
      telemetry.leftMotorDirection = motors.getDirectionA() == L298N::FORWARD;
      telemetry.leftMotorSpeed = map(motors.getSpeedA(), 0, 255, 0, 100);
  
      telemetry.rightMotorDirection = motors.getDirectionB() == L298N::FORWARD;
      telemetry.rightMotorSpeed = map(motors.getSpeedB(), 0, 255, 0, 100);
  
      unsigned long currentTime = millis();
      unsigned long deltaTime = currentTime - lastTime;
  
      if (deltaTime > 0) {
        // 1. Atomically grab the pulse count and reset it
        noInterrupts(); // Disable interrupts briefly to read
        long currentPulsesA = pulseCountA;
        long currentPulsesB = pulseCountB;
        pulseCountA = 0;
        pulseCountB = 0;
        interrupts();
  
        // 2. Calculate Revolutions per Second (RPS)
        float rpsA = ((float)currentPulsesA / PPR) / (deltaTime / 1000.0);
        float rpsB = ((float)currentPulsesB / PPR) / (deltaTime / 1000.0);
  
        // 3. Calculate Linear Speed (cm/s)
        float speedCmSA = rpsA * (PI * WHEEL_DIAMETER);
        float speedCmSB = rpsB * (PI * WHEEL_DIAMETER);
  
        // 4. Update your telemetry struct
        telemetry.trueLeftSpeed = speedCmSA;
        telemetry.trueRightSpeed = speedCmSB;
      }
      
      localCopy = telemetry;
      lastTime = currentTime;
      xSemaphoreGive(telemetryMutex);
    }


    esp_err_t result =
        esp_now_send(broadcastAddress, (uint8_t *)&localCopy, sizeof(localCopy));

    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
