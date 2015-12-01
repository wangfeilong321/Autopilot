#include <Engine.h>
#include <Timer.h>

using namespace std;

atomic<int>Engine::counter = 0;
bool Engine::canContinue = false;

Engine::Engine(const int PIN) {
	auto gpio = GpioController::GetDefault();
	if (!gpio)
		return;

	pin = gpio->OpenPin(PIN);
	if (!pin)
		return;

	pin->Write(pinValue);
	pin->SetDriveMode(GpioPinDriveMode::Output);
}

void Engine::OnTick() {
	if (pinValue == GpioPinValue::High)
		pinValue = GpioPinValue::Low;
	else
		pinValue = GpioPinValue::High;
	pin->Write(pinValue);
}

void Engine::StartEngine() {
	engineThread = unique_ptr<thread>( new thread(&Engine::ProcessPin, this));
	engineThread->detach();
}

void Engine::SetContinue(bool ifContinue) {
	canContinue = ifContinue;
}

bool Engine::GetReadyAll() {
	return counter == 4;
}

void Engine::OneIsReady() {
	counter++;
}

void Engine::ProcessPin() {
	{
		OneIsReady();
		unique_lock<mutex> lk(lockEngine);
		threadReadiness.wait(lk, []() {return canContinue;});
	}
	{
		const int calibrationDeltaTmcs = 2000;
		const int calibrationTime = 3;
		Timer timer(true);
		auto start = chrono::high_resolution_clock::now();
		while (timer.Elapsed().count() < calibrationTime) {
			auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
			if (duration.count() >= calibrationDeltaTmcs) {
				OnTick();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
	{
		const int calibrationDeltaTmcs = 1000;
		const int calibrationTime = 3;
		Timer timer(true);
		auto start = chrono::high_resolution_clock::now();
		while (timer.Elapsed().count() < calibrationTime) {
			auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
			if (duration.count() >= calibrationDeltaTmcs) {
				OnTick();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
	{
		auto start = chrono::high_resolution_clock::now();
		while (true) {
			auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
			if (duration.count() >= deltaTmcs) {
				OnTick();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
}

void Engine::SetThrottle(double cmd) { 
	deltaTmcs = static_cast<int>(1000 + cmd * 1000);
}