#ifndef ENGINE_BOARD_H
#define ENGINE_BOARD_H

using namespace Windows::Devices::Gpio;

class EngineBoard {
public:
	EngineBoard(const int PIN1, const int PIN2, const int PIN3, const int PIN4);

	void OnTick();

private:
	GpioPinValue pinValue = GpioPinValue::Low;
	GpioPin ^pin1;
	GpioPin ^pin2;
	GpioPin ^pin3;
	GpioPin ^pin4;
};

#endif
