#define MICROSTEPS 1
#define STEPPER_STEPS_PER_ROTATION 200
#define STEPS_PER_ROTATION (STEPPER_STEPS_PER_ROTATION * MICROSTEPS)
#define MAX_RPM 8
#define MAX_US_PER_STEP (1000000 / STEPS_PER_ROTATION) / MAX_RPM

#define STEP_PIN 4
#define STEP_GROUND 16
#define DIR_PIN 17
#define DIR_GROUND 5

#define CW_PIN 12
#define CW_GROUND 14 

#define CCW_PIN 26
#define CCW_GROUND 27

#define FEED_GROUND 0
#define FEED_RATE_PIN 2
#define FEED_POWER 15

#define ENCODER_PIN_A 34
#define ENCODER_PIN_B 35
#define ENCODER_CLICK_PIN 32
#define ENCODER_POWER_PIN 33
#define ENCODER_GROUND_PIN 25
#define ENCODER_PULSES_PER_ROTATION 28
