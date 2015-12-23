#include <Controller.h>
#include <PID.h>

using namespace std;

Controller::Controller() : ISensor(new SensorBoard), IEngine(new EngineBoard()), ISocket(new SocketBoard()) {}

Controller::~Controller() {}

void Controller::Connect() {
	ISensor->Connect();
	if (!ISensor->Connected()) {
		std::wostringstream msg;
		msg << L"Sensor device is not connected. Please ensure that you connected Sensor Board to computer.";
		throw wexception(msg.str());
	}

	IEngine->Connect();
	if (!IEngine->Connected()) {
		std::wostringstream msg;
		msg << L"Engine device is not connected. Please ensure that you connected Engine Board to computer.";
		throw wexception(msg.str());
	}
	
	ISocket->Connect();
	if (!ISocket->Connected()) {
		std::wostringstream msg;
		msg << L"Socket device is not connected. Please ensure that you connected Socket Board to computer.";
		throw wexception(msg.str());
	}
}

bool Controller::Connected() {
	return ISensor->Connected() && IEngine->Connected() && ISocket->Connected();
}

bool Controller::Run() {
	return (ISensor->Run() && IEngine->Run() && ISocket->Run()) ? true : false;
}