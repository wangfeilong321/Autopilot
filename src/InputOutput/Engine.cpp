#include <Engine.h>
#include <Timer.h>

using namespace std;

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

void Engine::imReady() {
	lock_guard<mutex> lk(mut);
	static auto count = 0;
	count++;
	if (count == 4) {
		data_cond.notify_all();
	}
}

void Engine::ProcessPin() {
	//waiting for all threads to be launched
	{
		imReady();
		unique_lock<mutex> lk(mut);
		data_cond.wait(lk);
	}
	//waiting for 5 sec 
	{

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