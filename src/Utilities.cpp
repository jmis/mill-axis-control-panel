#include "Utilities.h"
#include "LowPassFilter.h"
#include <Arduino.h>
#include <RotaryEncoder.h>
#include <FastAccelStepper.h>

using namespace ace_button;
using namespace std;

class SimpleButtonConfig : public ButtonConfig
{
	public:
		SimpleButtonConfig();
		function<void()> onPress;
		function<void()> onRelease;
};

SimpleButtonConfig::SimpleButtonConfig() : ButtonConfig()
{
	setEventHandler([](AceButton *b, uint8_t eventType, uint8_t state)
	{
		auto config = (SimpleButtonConfig*) b->getButtonConfig();
		if (eventType == AceButton::kEventPressed) config->onPress();
		else if (eventType == AceButton::kEventReleased) config->onRelease();
	});
}

void TaskFunctionWrapper(void* pvParameters)
{
	auto* lambda = static_cast<function<void()>*>(pvParameters);
	(*lambda)();
	delete lambda;
}

void configureButton(int activatePin, int groundPin, const function<void()>& onPress, const function<void()>& onRelease)
{
	auto* aceButtonConfig = new SimpleButtonConfig();
	aceButtonConfig->onPress = onPress;
	aceButtonConfig->onRelease = onRelease;
	auto* aceButton = new AceButton(aceButtonConfig, activatePin);
	auto task = [aceButton]() { while (1) { aceButton->check(); vTaskDelay(5 / portTICK_PERIOD_MS); } };

	pinMode(activatePin, INPUT_PULLUP);
	pinMode(groundPin, OUTPUT);
	digitalWrite(groundPin, LOW);
	xTaskCreate(TaskFunctionWrapper, "Button", 2048, new function<void()>(task), tskIDLE_PRIORITY, NULL);
}

void configureEncoder(int pinA, int pinB, int pinClick, int powerPin, int groundPin, const function<void(int)>& onMove)
{
	static RotaryEncoder fineStepEncoder(pinA, pinB, RotaryEncoder::LatchMode::TWO03);
	static auto encoderEnabled = true;
	static auto lastEncoderPosition = 0;

	static auto task = [onMove]()
	{
		while (1)
		{
			vTaskDelay(5 / portTICK_PERIOD_MS);
			if (!encoderEnabled) continue;
			auto currentPosition = fineStepEncoder.getPosition();
			float delta = currentPosition - lastEncoderPosition;
			if (delta == 0) continue;
			onMove(delta);
			lastEncoderPosition = currentPosition;
		}
	};

	static auto onPress = []()
	{
		encoderEnabled = !encoderEnabled;
		lastEncoderPosition = fineStepEncoder.getPosition();
	};

	configureButton(pinClick, groundPin, onPress, [](){});
	pinMode(powerPin, OUTPUT);
	digitalWrite(powerPin, HIGH);
	pinMode(groundPin, OUTPUT);
	digitalWrite(groundPin, LOW);
	attachInterrupt(pinA, []() { fineStepEncoder.tick(); }, CHANGE);
	attachInterrupt(pinB, []() { fineStepEncoder.tick(); }, CHANGE);
	xTaskCreate(TaskFunctionWrapper, "Encoder", 2048, new function<void()>(task), tskIDLE_PRIORITY, NULL);
}

void configurePotentiometer(int feedRatePin, int powerPin, int groundPin, const function<void(int)>& percentChanged)
{
	static auto potentiometerFilter = LowPassFilter(0.12);
	static int lastPercent = 0;

	static auto task = [feedRatePin, percentChanged]()
	{
		while (1)
		{
			auto potentiometerReading = analogRead(feedRatePin);
			auto readPercent = (int)map(potentiometerReading, 0, 4095, 0, 100);
			float currentPercent = (int)potentiometerFilter(readPercent);

			if (currentPercent != lastPercent)
			{
				lastPercent = currentPercent;
				percentChanged(currentPercent);
			}

			vTaskDelay(5 / portTICK_PERIOD_MS);
		}
	};

	pinMode(powerPin, OUTPUT);
	digitalWrite(powerPin, HIGH);
	pinMode(groundPin, OUTPUT);
	digitalWrite(groundPin, LOW);
	pinMode(feedRatePin, INPUT);
	xTaskCreate(TaskFunctionWrapper, "Potentiometer", 2048 * 10, new function<void()>(task), tskIDLE_PRIORITY, NULL);
}

FastAccelStepper* configureStepper(int stepPin, int stepGround, int dirPin, int dirGround)
{
	pinMode(stepGround, OUTPUT);
	digitalWrite(stepGround, LOW);
	pinMode(dirGround, OUTPUT);
	digitalWrite(dirGround, LOW);

	FastAccelStepperEngine engine = FastAccelStepperEngine();
	FastAccelStepper *stepper;

	engine.init();
	stepper = engine.stepperConnectToPin(stepPin);
	stepper->setDirectionPin(dirPin);
	stepper->enableOutputs();
	stepper->setAcceleration(1000000 / 5);
	stepper->setAutoEnable(true);

	return stepper;
}