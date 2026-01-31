#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>

#include <L298NX2.h>
#include <structures.h>

uint8_t carAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// Create a struct_messages
CommandMessage command;
TelemetryMessage telemetry;

const uint8_t IN1_A = 12;
const uint8_t IN2_A = 14;

const uint8_t IN1_B = 27;
const uint8_t IN2_B = 26;

// Initialize both motors
L298NX2 motors(IN1_A, IN2_A, IN1_B, IN2_B);

TaskHandle_t SendTelemetryTaskHandle = NULL;

// Motor A encoder
volatile long pulseCountA = 0;
const int ENCODER_PIN_A = 32; // Change to your actual pin

// Motor A encoder
volatile long pulseCountB = 0;
const int ENCODER_PIN_B = 33; // Change to your actual pin

const int PPR = 20;               // Pulses per revolution
const float WHEEL_DIAMETER = 6.5; // in cm

// Interrupt routine (must be in RAM for speed)
void IRAM_ATTR countPulseA() { pulseCountA++; }

// Interrupt routine (must be in RAM for speed)
void IRAM_ATTR countPulseB() { pulseCountB++; }

void SendTelemetryTask(void *parameter) {
  unsigned long lastTime = millis();
  for (;;) { // Infinite loop
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

    lastTime = currentTime;

    esp_err_t result =
        esp_now_send(carAddress, (uint8_t *)&telemetry, sizeof(telemetry));

    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&command, incomingData, sizeof(command));

  uint16_t leftMotorSpeed = map(command.leftMotorSpeed, 0, 100, 0, 255);
  uint16_t rightMotorSpeed = map(command.rightMotorSpeed, 0, 100, 0, 255);

  if (leftMotorSpeed == 0) {
    motors.stopA();
  } else {
    motors.setSpeedA(leftMotorSpeed);
    motors.runA(command.leftMotorDirection ? L298N::FORWARD : L298N::BACKWARD);
  }

  if (rightMotorSpeed == 0) {
    motors.stopB();
  } else {
    motors.setSpeedB(rightMotorSpeed);
    motors.runB(command.rightMotorDirection ? L298N::FORWARD : L298N::BACKWARD);
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), countPulseB, RISING);

  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());


  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer
  memcpy(peerInfo.peer_addr, carAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  xTaskCreatePinnedToCore(SendTelemetryTask,        // Task function
                          "SendTelemetryTask",      // Task name
                          4096,                     // Stack size (bytes)
                          NULL,                     // Parameters
                          1,                        // Priority
                          &SendTelemetryTaskHandle, // Task handle
                          1                         // Core 1
  );
}

void loop() {}