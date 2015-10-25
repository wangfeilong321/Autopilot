#include <memory>
#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include "Interface.h"
#include "InputSocketUDP.h"
#include "OutputSocketUDP.h"
#include "InputDevice.h"
#include "Orientation.h"
#include "PID.h"
#include "Timer.h"

using namespace std;

int deltaTmcs = 1000;

void Calibration(const unique_ptr<InputDevice>& IDevice, const unique_ptr<OutputSocket>& OSocket) {
	{
		const int calibrationDeltaTmcs = 2000;
		const int calibrationTime = 3;
		Timer timer(true);
		auto start = chrono::high_resolution_clock::now();
		while (timer.Elapsed().count() < calibrationTime) {
			auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
			if (duration.count() >= calibrationDeltaTmcs) {
				IDevice->OnTick();
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
				IDevice->OnTick();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
	vector<double> calibrationOutput;
	calibrationOutput.push_back(0.0); //time;
	calibrationOutput.push_back(1.0); //altitudeASL
	calibrationOutput.push_back(0.0); //vNorth
	calibrationOutput.push_back(0.0); //vEast
	calibrationOutput.push_back(0.0); //vDown
	calibrationOutput.push_back(0.0); //U
	calibrationOutput.push_back(0.0); //V
	calibrationOutput.push_back(0.0); //W
	calibrationOutput.push_back(0.0); //Roll
	calibrationOutput.push_back(0.0); //Pitch
	calibrationOutput.push_back(0.0); //Yaw
	calibrationOutput.push_back(0.0); //P
	calibrationOutput.push_back(0.0); //Q
	calibrationOutput.push_back(0.0); //R
	calibrationOutput.push_back(0.0); //velDotX
	calibrationOutput.push_back(0.0); //velDotY
	calibrationOutput.push_back(0.0); //velDotZ
	calibrationOutput.push_back(0.0); //vcas
	OSocket->SetControlOutput(calibrationOutput);
	OSocket->Run();
}

void PWDThread(const unique_ptr<InputDevice>& IDevice) {
	auto start = chrono::high_resolution_clock::now();
	while (true) {
		auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
		if (duration.count() >= deltaTmcs) {
			IDevice->OnTick();
			start = chrono::high_resolution_clock::now();
		}
	}
}

int main(Platform::Array<Platform::String^>^ args) {
	
	cout << "Connecting to socket for input" << endl;
	unique_ptr<InputSocket> ISocket = unique_ptr<InputSocket>(new InputSocketUDP("192.168.88.150", 5502));
	while (!ISocket->Connected()) {
		ISocket->Connect();
		Sleep(500);
	}
	cout << "Input socket has been connected successfully" << endl;

	cout << "Connecting to socket for output" << endl;
	unique_ptr<OutputSocket> OSocket = unique_ptr<OutputSocket>(new OutputSocketUDP("192.168.88.101", 5503));
	while (!OSocket->Connected()) {
		OSocket->Connect();
		Sleep(500);
	}
	cout << "Output socket has been connected successfully" << endl;
	
	cout << "Connecting to sensor" << endl;
	unique_ptr<InputDevice> IDevice = unique_ptr<InputDevice>(new InputDevice());
	while (!IDevice->Connected()) {
		IDevice->Connect();
		Sleep(500);
	}
	cout << "Sensor has been connected successfully" << endl;

	Orientation Attitude;
	vector<double> sensorInput;
	vector<double> controlInput;
	vector<double> sensorOutput;
	vector<double> controlOutput;
	controlOutput.reserve(18);

	Calibration(IDevice, OSocket);
	
	cout << "Entering main loop" << endl;
	
	chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();

	while ( true ) {
		
		if ( IDevice->Run() ) {
			sensorInput = IDevice->GetSensorInput();
			sensorOutput = Attitude.GetAngles(sensorInput[0], sensorInput[1], sensorInput[2], sensorInput[3], sensorInput[4], sensorInput[5], sensorInput[6], sensorInput[7], sensorInput[8]);
		}

		auto duration = chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - start);
		if ( duration.count() > 33 ) {
			
			if (ISocket->Run()) {
				controlInput = ISocket->GetControlInput();
				deltaTmcs = 1000 + (controlInput[4] * 1000);
			}
			
			controlOutput.push_back(0.0); //time;
			controlOutput.push_back(1.0); //altitudeASL
			controlOutput.push_back(0.0); //vNorth
			controlOutput.push_back(0.0); //vEast
			controlOutput.push_back(0.0); //vDown
			controlOutput.push_back(0.0); //U
			controlOutput.push_back(0.0); //V
			controlOutput.push_back(0.0); //W
			controlOutput.push_back(sensorOutput[0]*degtorad); //Roll
			controlOutput.push_back(sensorOutput[1]*degtorad); //Pitch
			controlOutput.push_back(sensorOutput[2]*degtorad); //Yaw
			controlOutput.push_back(0.0); //P
			controlOutput.push_back(0.0); //Q
			controlOutput.push_back(0.0); //R
			controlOutput.push_back(0.0); //velDotX
			controlOutput.push_back(0.0); //velDotY
			controlOutput.push_back(0.0); //velDotZ
			controlOutput.push_back(0.0); //vcas

			OSocket->SetControlOutput(controlOutput);
			if ( OSocket->Run() )
				controlOutput.clear();
			start = chrono::high_resolution_clock::now();
		}
	}
	
	cout << "Exit main loop" << endl;
	
	return EXIT_SUCCESS;
}