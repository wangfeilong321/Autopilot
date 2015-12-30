#ifndef ENGINE_BOARD_H
#define ENGINE_BOARD_H

#include <Interface.h>
#include <StateSpace.h>

#include <atomic>
#include <memory>

using namespace Windows::Devices::Gpio;

class EngineBoard : public Interface {
public:
	EngineBoard(const std::shared_ptr<StateSpace> ISS);
	virtual ~EngineBoard() = default;

	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();
	
protected:
	void Engine1OnTick();
	void Engine2OnTick();
	void Engine3OnTick();
	void Engine4OnTick();

private:
	std::atomic<int> deltaTmcs = 2000;
	std::shared_ptr<StateSpace> IState;
	bool ifConnected;
	GpioPinValue pinValue = GpioPinValue::Low;
	GpioPin ^pin1;
	GpioPin ^pin2;
	GpioPin ^pin3;
	GpioPin ^pin4;
};

#endif
