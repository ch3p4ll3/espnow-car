#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>

#include <L298NX2.h>
#include <TinyGPS++.h>

#include <tasks.h>

// Motor A encoder
volatile long pulseCountA = 0;
// Motor B encoder
volatile long pulseCountB = 0;

uint64_t last_command = 0;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// Create a struct_messages
CommandMessage command;
TelemetryMessage telemetry;

// Initialize both motors
L298NX2 motors(ENA_A, IN1_A, IN2_A, ENA_B, IN1_B, IN2_B);

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

TaskHandle_t SendTelemetryTaskHandle = NULL;
TaskHandle_t UpdateGPSPositionTaskHandle = NULL;

SemaphoreHandle_t telemetryMutex = NULL;

// Interrupt routine (must be in RAM for speed)
void IRAM_ATTR countPulseA() { pulseCountA++; }

// Interrupt routine (must be in RAM for speed)
void IRAM_ATTR countPulseB() { pulseCountB++; }

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

  last_command = millis();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), countPulseB, RISING);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("Serial 2 started at 9600 baud rate");
  
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
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  telemetryMutex = xSemaphoreCreateMutex();
  if (telemetryMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);
  }

  xTaskCreatePinnedToCore(SendTelemetryTask,        // Task function
                          "SendTelemetryTask",      // Task name
                          4096,                     // Stack size (bytes)
                          NULL,                     // Parameters
                          1,                        // Priority
                          &SendTelemetryTaskHandle, // Task handle
                          1                         // Core 1
  );

  xTaskCreatePinnedToCore(UpdateGPSPositionTask,        // Task function
                          "UpdateGPSPositionTask",      // Task name
                          4096,                     // Stack size (bytes)
                          NULL,                     // Parameters
                          1,                        // Priority
                          &UpdateGPSPositionTaskHandle, // Task handle
                          1                         // Core 1
  );
}

void loop() {
  if (millis() - last_command >= MAX_TIMEOUT){
    motors.stop();
  }
}