#define REMOTEXY_MODE__ESP32CORE_BLE
#define REMOTEXY_BLUETOOTH_NAME "PM-25MV Z LIFT"

#include <Arduino.h>
#include <RemoteXY.h>

#define MICROSTEPS 32
#define STEPPER_STEPS_PER_ROTATION 200
#define STEPS_PER_ROTATION (STEPPER_STEPS_PER_ROTATION * MICROSTEPS)
#define MAX_RPM 12
#define MAX_US_PER_STEP (1000000 / STEPS_PER_ROTATION) / MAX_RPM

#define STEP_PIN 22
#define ENABLE_PIN 26
#define DIR_PIN 23
#define CW_PIN 14
#define CCW_PIN 12
#define FEED_RATE_PIN 36
#define ROTARY_PIN_A 18
#define ROTARY_PIN_B 19

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = { 255,8,0,1,0,78,0,16,8,1,66,1,27,43,9,37,2,26,129,0,
  2,13,42,4,17,77,83,116,101,112,115,32,32,32,32,32,32,32,77,97,
  120,32,70,32,32,32,32,32,32,32,32,32,70,105,110,101,0,7,52,42,
  19,18,5,2,26,2,7,44,22,19,17,5,2,26,2,1,7,52,2,19,
  18,5,2,26,2 };
struct
{
	int16_t microsteps;
	float max_rpm;
	int16_t fine_step;
	int8_t percent_max_rpm;
	uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)