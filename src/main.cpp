#include "Constants.h"
#include <Arduino.h>
#include <AceButton.h>
#include <RotaryEncoder.h>
#include <FastAccelStepper.h>
#include "LowPassFilter.h"
#include "nvs_flash.h"
#include "esp_pthread.h"

using namespace ace_button;

RotaryEncoder fineStepEncoder(ROTARY_PIN_A, ROTARY_PIN_B, RotaryEncoder::LatchMode::TWO03);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;
ButtonConfig cw_button_config;
AceButton cw_button(&cw_button_config);
ButtonConfig ccw_button_config;
AceButton ccw_button(&ccw_button_config);
ButtonConfig toggle_encoder_button_config;
AceButton toggle_encoder_button(&toggle_encoder_button_config);

auto encoderEnabled = true;
auto lastEncoderPosition = 0;

void processPotentiometer()
{
	static auto potentiometerFilter = LowPassFilter(0.12);
	static auto lastSteps = 0;

	auto potentiometerReading = analogRead(FEED_RATE_PIN);
	auto readPercent = (int)map(potentiometerReading, 0, 4095, 0, 100);
	float filteredPercent = (int)potentiometerFilter(readPercent);
	auto currentSteps = MAX_US_PER_STEP / (filteredPercent / 100);

	if (currentSteps != lastSteps)
	{
		lastSteps = currentSteps;
		stepper->setSpeedInUs(currentSteps);
		stepper->applySpeedAcceleration();
	}
}

void processEncoder()
{
	if (!encoderEnabled) return;
	auto currentPosition = fineStepEncoder.getPosition();
	float delta = currentPosition - lastEncoderPosition;
	if (delta == 0) return;
	stepper->move(delta * 2);
	lastEncoderPosition = currentPosition;
}

void* inputProcessingTask(void *pvParameter)
{
	while (1)
	{
		cw_button.check();
		ccw_button.check();
		toggle_encoder_button.check();
		processPotentiometer();
		processEncoder();
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

extern "C" void app_main()
{
	esp_log_level_set("*", ESP_LOG_ERROR);
	ESP_ERROR_CHECK(nvs_flash_init());
	Serial.begin(115200);

	attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), []() { fineStepEncoder.tick(); }, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), []() { fineStepEncoder.tick(); }, CHANGE);

	pinMode(CW_PIN, INPUT_PULLUP);
	cw_button.init(CW_PIN);
	cw_button.setEventHandler([](AceButton *b, uint8_t eventType, uint8_t state)
	{
		if (eventType == AceButton::kEventPressed) stepper->runForward();
		else if (eventType == AceButton::kEventReleased) stepper->forceStop();
	});

	pinMode(CCW_PIN, INPUT_PULLUP);
	ccw_button.init(CCW_PIN);
	ccw_button.setEventHandler([](AceButton *b, uint8_t eventType, uint8_t state)
	{
		if (eventType == AceButton::kEventPressed) stepper->runBackward();
		else if (eventType == AceButton::kEventReleased) stepper->forceStop();
	});

	pinMode(TOGGLE_ENCODER_PIN, INPUT_PULLUP);
	toggle_encoder_button.init(TOGGLE_ENCODER_PIN);
	toggle_encoder_button.setEventHandler([](AceButton *b, uint8_t eventType, uint8_t state)
	{
		if (eventType != AceButton::kEventPressed) return;
		encoderEnabled = !encoderEnabled;
		lastEncoderPosition = fineStepEncoder.getPosition();
	});

	engine.init();
	stepper = engine.stepperConnectToPin(STEP_PIN);
	stepper->setEnablePin(ENABLE_PIN);
	stepper->setDirectionPin(DIR_PIN);
	stepper->enableOutputs();
	stepper->setAcceleration(1000000 / 5);
	stepper->setAutoEnable(true);

	pthread_t thread;
	pthread_create(&thread, NULL, inputProcessingTask, NULL);
}
