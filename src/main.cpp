#include <memory>
#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include "Interface.h"
#include "TCPSocket.h"
#include "InputDevice.h"
#include "PID.h"
#include "Timer.h"

using namespace std;

int main(Platform::Array<Platform::String^>^ args) {
	
	unique_ptr<TCPSocket> Socket = unique_ptr<TCPSocket>(new TCPSocket("192.168.0.10", 3001));
	while (!Socket->Connected()) {
		Socket->Connect();
		Sleep(500);
	}
	
	unique_ptr<InputDevice> IDevice = unique_ptr<InputDevice>(new InputDevice());
	while (!IDevice->Connected()) {
		IDevice->Connect();
		Sleep(500);
	}

	vector<float> sensorInput;
	vector<float> controlInput;
	vector<float> controlOutput;
	
	vector<float> calibrationOutput(18, 0.0);
	Socket->SetControlOutput(calibrationOutput);

	auto start = chrono::high_resolution_clock::now();

	while ( true ) {
		
		if ( IDevice->Run() ) {
			sensorInput = IDevice->GetAngles();
		}
		/*
		auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start);
		if (duration.count() >= 20) {
			controlInput = Socket->GetControlInput();

			if (!controlInput.empty())
				IDevice->SetPWM(static_cast<int>(1000 + controlInput[3]*1000));
				*/
		auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start);
		if (duration.count() >= 10) {
			//controlInput = Socket->GetControlInput();

			controlOutput.push_back(0.0); //time;
			controlOutput.push_back(128.0f / 0.3028f); //altitudeASL
			controlOutput.push_back(0.0); //vNorth
			controlOutput.push_back(0.0); //vEast
			controlOutput.push_back(0.0); //vDown
			controlOutput.push_back(0.0); //U
			controlOutput.push_back(0.0); //V
			controlOutput.push_back(0.0); //W
			controlOutput.push_back(sensorInput[0] * degtorad); //Roll
			controlOutput.push_back(sensorInput[1] * degtorad); //Pitch
			controlOutput.push_back(sensorInput[2] * degtorad); //Yaw
			controlOutput.push_back(0.0); //P
			controlOutput.push_back(0.0); //Q
			controlOutput.push_back(0.0); //R
			controlOutput.push_back(0.0); //velDotX
			controlOutput.push_back(0.0); //velDotY
			controlOutput.push_back(0.0); //velDotZ
			controlOutput.push_back(0.0); //vcas

			Socket->SetControlOutput(controlOutput);

			controlOutput.clear();
			start = chrono::high_resolution_clock::now();
		}
	}
	
	//t.join();
	
	return EXIT_SUCCESS;
}