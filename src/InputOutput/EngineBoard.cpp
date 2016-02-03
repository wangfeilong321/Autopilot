#include <EngineBoard.h>
#include <Base.h>
#include <Timer.h>

#include <chrono>

using namespace std;
using namespace chrono;

EngineBoard::EngineBoard(const std::shared_ptr<StateSpace> ISS) : IState(ISS), ifConnected(false), deltaTmcs(MIN_THROTTLE) {}

void EngineBoard::Connect() {
	auto gpio = GpioController::GetDefault();
	if (!gpio)
		return;

	#ifdef OPEN_1
		pin1 = gpio->OpenPin(ENGINE_PIN_1);
		if (!pin1)
			return;
		pin1->Write(pinValue);
		pin1->SetDriveMode(GpioPinDriveMode::Output);
	#endif
	
	#ifdef OPEN_2
		pin2 = gpio->OpenPin(ENGINE_PIN_2);
		if (!pin2)
			return;
		pin2->Write(pinValue);
		pin2->SetDriveMode(GpioPinDriveMode::Output);
	#endif

	#ifdef OPEN_3
		pin3 = gpio->OpenPin(ENGINE_PIN_3);
		if (!pin3)
			return;
		pin3->Write(pinValue);
		pin3->SetDriveMode(GpioPinDriveMode::Output);
	#endif

	#ifdef OPEN_4
		pin4 = gpio->OpenPin(ENGINE_PIN_4);
		if (!pin4)
			return;
		pin4->Write(pinValue);
		pin4->SetDriveMode(GpioPinDriveMode::Output);
	#endif

	IState->Wait();
	
	#ifdef CALIBRATE
		const int calibrationTimeSec = 2;
		deltaTmcs = static_cast<int>(1000 + 1000 * IState->getThrottle());
		auto start = high_resolution_clock::now();
		Timer timer(true);
		while (timer.Elapsed().count() <= calibrationTimeSec) {
			auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
			if (duration.count() >= deltaTmcs) {
				OnTick();
				start = high_resolution_clock::now();
			}
		}
	#endif
	
	ifConnected = true;
}

bool EngineBoard::Connected() {
	return ifConnected;
}

bool EngineBoard::Run() {

	deltaTmcs = static_cast<int>(1000+1000*IState->getThrottle());

	IState->setEnginesPRM(deltaTmcs, deltaTmcs, deltaTmcs, deltaTmcs);

	static auto start = high_resolution_clock::now();

	auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
	if (duration.count() >= deltaTmcs) {
		OnTick();
		start = high_resolution_clock::now();
	}
	return true;
}

void EngineBoard::OnTick() {
	if (pinValue == GpioPinValue::High)
		pinValue = GpioPinValue::Low;
	else
		pinValue = GpioPinValue::High;
	pin1->Write(pinValue);
	pin2->Write(pinValue);
	pin3->Write(pinValue);
	pin4->Write(pinValue);
}