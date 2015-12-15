#include <EngineBoard.h>

using namespace std;

EngineBoard::EngineBoard(const int PIN1, const int PIN2, const int PIN3, const int PIN4) {
	auto gpio = GpioController::GetDefault();
	if (!gpio)
		return;

	pin1 = gpio->OpenPin(PIN1);
	if (!pin1)
		return;

	pin1->Write(pinValue);
	pin1->SetDriveMode(GpioPinDriveMode::Output);

	pin2 = gpio->OpenPin(PIN2);
	if (!pin2)
		return;

	pin2->Write(pinValue);
	pin2->SetDriveMode(GpioPinDriveMode::Output);

	pin3 = gpio->OpenPin(PIN3);
	if (!pin3)
		return;

	pin3->Write(pinValue);
	pin3->SetDriveMode(GpioPinDriveMode::Output);
	
	pin4 = gpio->OpenPin(PIN4);
	if (!pin4)
		return;

	pin4->Write(pinValue);
	pin4->SetDriveMode(GpioPinDriveMode::Output);
}

void EngineBoard::Engine1OnTick() {
	if (pinValue == GpioPinValue::High)
		pinValue = GpioPinValue::Low;
	else
		pinValue = GpioPinValue::High;
	pin1->Write(pinValue);
}

void EngineBoard::Engine2OnTick() {
	if (pinValue == GpioPinValue::High)
		pinValue = GpioPinValue::Low;
	else
		pinValue = GpioPinValue::High;
	pin2->Write(pinValue);
}

void EngineBoard::Engine3OnTick() {
	if (pinValue == GpioPinValue::High)
		pinValue = GpioPinValue::Low;
	else
		pinValue = GpioPinValue::High;
	pin3->Write(pinValue);
}

void EngineBoard::Engine4OnTick() {
	if (pinValue == GpioPinValue::High)
		pinValue = GpioPinValue::Low;
	else
		pinValue = GpioPinValue::High;
	pin4->Write(pinValue);
}