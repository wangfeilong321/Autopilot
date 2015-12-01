#ifndef ENGINE_H
#define ENGINE_H

#include <Interface.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <thread>

using namespace Windows::Devices::Gpio;

class Engine {
public:
	explicit Engine(const int PIN);

	void SetThrottle(double cmd);
	void StartEngine();
	static void SetContinue(bool ifContinue);
	static bool GetReadyAll();

private:
	void OnTick();
	void ProcessPin();
	void OneIsReady();

	std::atomic<int> deltaTmcs;
	std::unique_ptr<std::thread> engineThread;
	std::mutex lockEngine;
	std::condition_variable threadReadiness;
	
	static bool canContinue;
	static std::atomic<int> counter;

	GpioPinValue pinValue = GpioPinValue::Low;
	GpioPin ^pin;
};

#endif
