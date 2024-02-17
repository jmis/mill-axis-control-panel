#include "Constants.h"
#include <AccelStepper.h>
#include <AceButton.h>
#include <nvs_flash.h>
#include <EEPROM.h>
#include <RotaryEncoder.h>
#include <FastAccelStepper.h>
#include "LowPassFilter.h"

using namespace ace_button;

RotaryEncoder fineStepEncoder(ROTARY_PIN_A, ROTARY_PIN_B, RotaryEncoder::LatchMode::TWO03);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper;
ButtonConfig cw_button_config;
AceButton cw_button(&cw_button_config);
ButtonConfig ccw_button_config;
AceButton ccw_button(&ccw_button_config);

void processPotentiometer()
{
	static auto potentiometerFilter = LowPassFilter(0.12);
	static auto lastSteps = 0;

	auto potentiometerReading = analogRead(FEED_RATE_PIN);
	auto readPercent = (int) map(potentiometerReading, 0, 4095, 0, 100);
	float filteredPercent = (int)potentiometerFilter(readPercent);
	auto currentSteps = MAX_US_PER_STEP / (filteredPercent / 100);

	if (currentSteps != lastSteps)
	{
		lastSteps = currentSteps;
		stepper->setSpeedInUs(currentSteps);
		stepper->applySpeedAcceleration();
		RemoteXY.percent_max_rpm = filteredPercent;
	}
}

void processEncoder()
{
	static auto lastPosition = 0;
	auto currentPosition = fineStepEncoder.getPosition();
	float delta = currentPosition - lastPosition;
	if (delta == 0) return;
	stepper->move((long) (delta / 48 * 6400L));
	lastPosition = currentPosition;
}

void inputProcessingTask(void *pvParameter)
{
	while (1)
	{
		RemoteXY_Handler();
		cw_button.check();
		ccw_button.check();
		processPotentiometer();
		processEncoder();
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

extern "C" void app_main()
{
	Serial.begin(115200);
	nvs_flash_init();

	RemoteXY_Init();
	RemoteXY.connect_flag = 1;
	RemoteXY.max_rpm = 7;

	attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A),[]() { fineStepEncoder.tick(); }, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B),[]() { fineStepEncoder.tick(); }, CHANGE);

	pinMode(CW_PIN, INPUT_PULLUP);
	cw_button.init(CW_PIN);
	cw_button.setEventHandler([](AceButton* b, uint8_t eventType, uint8_t state)
	{
		if (eventType == AceButton::kEventPressed) stepper->runForward();
		else if (eventType == AceButton::kEventReleased) stepper->forceStop();
	});

	pinMode(CCW_PIN, INPUT_PULLUP);
	ccw_button.init(CCW_PIN);
	ccw_button.setEventHandler([](AceButton* b, uint8_t eventType, uint8_t state)
	{
		if (eventType == AceButton::kEventPressed) stepper->runBackward();
		else if (eventType == AceButton::kEventReleased) stepper->forceStop();
	});

	engine.init();
	stepper = engine.stepperConnectToPin(STEP_PIN);
	stepper->setEnablePin(ENABLE_PIN);
	stepper->setDirectionPin(DIR_PIN);
	stepper->enableOutputs();
	stepper->setAcceleration(1000000 / 5);
	stepper->setAutoEnable(true);

	xTaskCreate(&inputProcessingTask, "inputProcessingTask", 2048 * 2, NULL, 20, NULL);
}

// esp_log_level_set("*", ESP_LOG_NONE);
// EEPROM.begin(1024);
// EEPROM.writeUInt(6400);
// EEPROM.commit();