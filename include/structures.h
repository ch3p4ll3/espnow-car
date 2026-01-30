#include <stdint.h>

// Structure for received data
typedef struct CommandMessage {
    bool leftMotorDirection;  // true = forward, false = backwards
    uint16_t leftMotorSpeed;  // 0 -100%, 0 = stop

    bool rightMotorDirection; // true = forward, false = backwards
    uint16_t rightMotorSpeed; // 0 -100%, 0 = stop
} CommandMessage;

typedef struct TelemetryMessage {
    bool leftMotorDirection;  // true = forward, false = backwards
    uint16_t leftMotorSpeed;  // 0 -100%, 0 = stop
    float trueLeftSpeed;

    bool rightMotorDirection; // true = forward, false = backwards
    uint16_t rightMotorSpeed; // 0 -100%, 0 = stop
    float trueRightSpeed;
} TelemetryMessage;
