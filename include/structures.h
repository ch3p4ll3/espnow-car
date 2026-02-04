#include <stdint.h>

// Structure for received data
typedef struct CommandMessage {
    uint8_t leftMotorDirection;    // true = forward, false = backwards
    uint16_t leftMotorSpeed;    // 0 - 100%, 0 = stop

    uint8_t rightMotorDirection;   // true = forward, false = backwards
    uint16_t rightMotorSpeed;   // 0 - 100%, 0 = stop
} CommandMessage;

typedef struct TelemetryMessage {
    uint8_t leftMotorDirection;    // true = forward, false = backwards
    uint16_t leftMotorSpeed;    // 0 - 100%, 0 = stop
    float trueLeftSpeed;        // cm/s

    uint8_t rightMotorDirection;   // true = forward, false = backwards
    uint16_t rightMotorSpeed;   // 0 - 100%, 0 = stop
    float trueRightSpeed;       // cm/s

    double lat;
    double lon;
    double gpsSpeed;
} TelemetryMessage;
