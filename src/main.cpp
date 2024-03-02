#include "Constants.h"
#include "Utilities.h"
#include "nvs_flash.h"

using namespace std;

extern "C" void app_main()
{
	esp_log_level_set("*", ESP_LOG_ERROR);
	ESP_ERROR_CHECK(nvs_flash_init());
	Serial.begin(115200);

	auto stepper = configureStepper(
		STEP_PIN,
		STEP_GROUND,
		DIR_PIN,
		DIR_GROUND);

	configurePotentiometer(
		FEED_RATE_PIN,
		FEED_POWER,
		FEED_GROUND,
		[=](int percent)
		{
			auto currentSteps = percent == 0 ? 0 : MAX_US_PER_STEP / (percent / (float) 100);
			stepper->setSpeedInUs(currentSteps);
			stepper->applySpeedAcceleration();
		});

	configureButton(
		CW_PIN,
		CW_GROUND,
		[=]() { stepper->runForward(); },
		[=]() { stepper->forceStop(); });
		
	configureButton(
		CCW_PIN,
		CCW_GROUND,
		[=]() { stepper->runBackward(); },
		[=]() { stepper->forceStop(); });

	configureEncoder(
		ENCODER_PIN_A,
		ENCODER_PIN_B,
		ENCODER_CLICK_PIN,
		ENCODER_POWER_PIN,
		ENCODER_GROUND_PIN,
		[=](int amount) { stepper->move(amount * 2); });
}
