#include <memory>
#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include "Interface.h"
#include "InputSocketTCP.h"
#include "InputSocketUDP.h"
#include "InputDevice.h"
#include "Orientation.h"
#include "PID.h"

using namespace std;

int main(Platform::Array<Platform::String^>^ args) {
	ofstream logFile;
	logFile.open("log.txt");
	if (!logFile.is_open())
		return EXIT_FAILURE;

	unique_ptr<InputDevice> IDevice = unique_ptr<InputDevice>(new InputDevice());

	while (!IDevice->Connected()) {
		IDevice->Connect();
		Sleep(500);
	}
	
	unique_ptr<InputSocketUDP> ISocket = unique_ptr<InputSocketUDP>(new InputSocketUDP(5555));
	
	while (!ISocket->Connected()) {
		ISocket->Connect();
		Sleep(500);
	}

	Orientation Attitude;
	vector<double> sensorInput;
	vector<double> controlInput;
	vector<double> currentAngles;

	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	
	while ( IDevice->Run() && ISocket->Run() ) {
		sensorInput = IDevice->GetSensorInput();
		currentAngles = Attitude.GetAngles(sensorInput[0], sensorInput[1], sensorInput[2], sensorInput[3], sensorInput[4], sensorInput[5], sensorInput[6], sensorInput[7], sensorInput[8]);
		controlInput = ISocket->GetControlInput();

		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
		if (duration.count() > 1000) {
			logFile << "sensor roll: " << currentAngles[0] << endl;
			logFile << "desired roll: " << controlInput[1] << endl;
			logFile << "sensor pitch: " << currentAngles[1] << endl;
			logFile << "desired pitch: " << controlInput[2] << endl;
			logFile << "sensor yaw: " << currentAngles[2] << endl;
			logFile << "desired yaw: " << controlInput[3] << endl;
			logFile << endl;
			start = std::chrono::high_resolution_clock::now();
		}
	}

	return EXIT_SUCCESS;
}


/*
thread t(rotationThread);

t.join();
*/

/*
class Timer {
typedef std::chrono::high_resolution_clock high_resolution_clock;
typedef std::chrono::seconds seconds;
public:
explicit Timer(bool run = false) {
if (run)
Reset();
}
void Reset() {
_start = high_resolution_clock::now();
}
seconds Elapsed() const {
return std::chrono::duration_cast<seconds>(high_resolution_clock::now() - _start);
}
private:
high_resolution_clock::time_point _start;
};


int deltaTmcs = 2000;


void rotationThread() {
shared_ptr<InputDevice> Device = shared_ptr<InputDevice>(new InputDevice());
auto start = std::chrono::high_resolution_clock::now();
while (true) {
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
if (duration.count() >= deltaTmcs) {
Device->OnTick();
start = std::chrono::high_resolution_clock::now();
}
}
}
*/


/*
Timer timer(true);

while (true) {
if (timer.Elapsed().count() <= 3) {
deltaTmcs = 2000;
}
if (timer.Elapsed().count() > 3 && timer.Elapsed().count() <= 8) {
deltaTmcs = 1000;
}
if (timer.Elapsed().count() > 8 && timer.Elapsed().count() <= 18) {
deltaTmcs = 1150;
}
if (timer.Elapsed().count() > 28 && timer.Elapsed().count() <= 38) {
deltaTmcs = 1250;
}
if (timer.Elapsed().count() > 38 && timer.Elapsed().count() <= 48) {
deltaTmcs = 1350;
}
if (timer.Elapsed().count() > 48 && timer.Elapsed().count() <= 58) {
deltaTmcs = 1450;
}
if (timer.Elapsed().count() > 58 && timer.Elapsed().count() <= 68) {
deltaTmcs = 1550;
}
if (timer.Elapsed().count() > 68) {
deltaTmcs = 2000;
}
}
*/