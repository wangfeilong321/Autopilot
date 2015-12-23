#include <EngineBoard.h>
#include <Base.h>
#include <Timer.h>

#include <chrono>

using namespace std;
using namespace chrono;

EngineBoard::EngineBoard() : ifConnected(false) {}

void EngineBoard::Connect() {
	auto gpio = GpioController::GetDefault();
	if (!gpio)
		return;

	pin1 = gpio->OpenPin(ENGINE_PIN_1);
	if (!pin1)
		return;

	pin1->Write(pinValue);
	pin1->SetDriveMode(GpioPinDriveMode::Output);

	pin2 = gpio->OpenPin(ENGINE_PIN_2);
	if (!pin2)
		return;

	pin2->Write(pinValue);
	pin2->SetDriveMode(GpioPinDriveMode::Output);

	pin3 = gpio->OpenPin(ENGINE_PIN_3);
	if (!pin3)
		return;

	pin3->Write(pinValue);
	pin3->SetDriveMode(GpioPinDriveMode::Output);

	pin4 = gpio->OpenPin(ENGINE_PIN_4);
	if (!pin4)
		return;

	pin4->Write(pinValue);
	pin4->SetDriveMode(GpioPinDriveMode::Output);

	ifConnected = true;
}

bool EngineBoard::Connected() {
	return ifConnected;
}

bool EngineBoard::Run() {
	static auto start1 = high_resolution_clock::now();
	static auto start2 = high_resolution_clock::now();
	static auto start3 = high_resolution_clock::now();
	static auto start4 = high_resolution_clock::now();

	auto duration1 = duration_cast<microseconds>(high_resolution_clock::now() - start1);
	if (duration1.count() >= deltaTmcs) {
		Engine1OnTick();
		start1 = high_resolution_clock::now();
	}
	auto duration2 = duration_cast<microseconds>(high_resolution_clock::now() - start2);
	if (duration2.count() >= deltaTmcs) {
		Engine2OnTick();
		start2 = high_resolution_clock::now();
	}
	auto duration3 = duration_cast<microseconds>(high_resolution_clock::now() - start3);
	if (duration3.count() >= deltaTmcs) {
		Engine3OnTick();
		start3 = high_resolution_clock::now();
	}
	auto duration4 = duration_cast<microseconds>(high_resolution_clock::now() - start4);
	if (duration4.count() >= deltaTmcs) {
		Engine4OnTick();
		start4 = high_resolution_clock::now();
	}
	return true;
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