#pragma once

#include <Interface.h>
#include <StateSpace.h>

#include <memory>

using namespace Windows::Devices::Gpio;

class EngineBoard : public Interface {
public:
  EngineBoard(const std::shared_ptr<StateSpace>& ISS);
	virtual ~EngineBoard() = default;
	
	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();
	
protected:
	void OnTick();

private:
	int deltaTmcs;
	std::shared_ptr<StateSpace> IState;
	bool ifConnected;
	GpioPinValue pinValue = GpioPinValue::Low;
	GpioPin ^pin1;
	GpioPin ^pin2;
	GpioPin ^pin3;
	GpioPin ^pin4;
};