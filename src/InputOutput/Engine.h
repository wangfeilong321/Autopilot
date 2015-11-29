#ifndef ENGINE_H
#define ENGINE_H

#include <Interface.h>
#include <atomic>
#include <mutex>
#include <condition_variable>

using namespace Windows::Devices::Gpio;

class Engine {
public:
	Engine(const int PIN);

	void SetThrottle(double cmd);

private:
	void OnTick();
	void ProcessPin();
	void imReady();

	std::atomic<int> deltaTmcs;
	std::thread engine;
	std::mutex mut;
	std::condition_variable data_cond;
		
	GpioPinValue pinValue = GpioPinValue::Low;
	GpioPin ^pin;
};

#endif
