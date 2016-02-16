#include <Controller.h>

using namespace std;

Controller::Controller() : IState(new StateSpace()), ISensor(new SensorBoard(IState)), IEngine(new EngineBoard(IState)), ISocket(new SocketBoard(IState)) {}

void Controller::Connect() {
	ISensor->Connect();
	if (!ISensor->Connected()) {
		std::ostringstream msg;
		msg << "Sensor device is not connected. Please ensure that you connected Sensor Board to computer.";
		throw exception(msg.str());
	}

	ISocket->Connect();
	if (!ISocket->Connected()) {
		std::ostringstream msg;
		msg << "Socket device is not connected. Please ensure that you connected Socket Board to computer.";
		throw exception(msg.str());
	}

	IEngine->Connect();
	if (!IEngine->Connected()) {
		std::ostringstream msg;
		msg << "Engine device is not connected. Please ensure that you connected Engine Board to computer.";
		throw exception(msg.str());
	}
}

bool Controller::Connected() {
	return ISensor->Connected() && IEngine->Connected() && ISocket->Connected();
}

bool Controller::Run() {
	return (
		ISensor->Run() && // get data from sensor 
		ISocket->Run() && // get data from socket
		IState->Run() && // calculate aircraft state
		IEngine->Run() // send commands to engines
		) ? true : false;
}