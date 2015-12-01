#include <thread>
#include <memory>
#include <string>
#include <Interface.h>
#include <TCPSocket.h>
#include <SensorBoard.h>
#include <EngineBoard.h>
#include <PID.h>
#include <Timer.h>

using namespace std;

void deviceRun(const unique_ptr<SensorBoard>& ISensor) {
	while (ISensor->Run());
}

int main(Platform::Array<Platform::String^>^ args) {
	
	unique_ptr<TCPSocket> Socket = unique_ptr<TCPSocket>(new TCPSocket("192.168.0.10", 3001));
	while (!Socket->Connected()) {
		Socket->Connect();
		Sleep(500);
	}
	
	unique_ptr<SensorBoard> ISensor = unique_ptr<SensorBoard>(new SensorBoard());
	while (!ISensor->Connected()) {
		ISensor->Connect();
		Sleep(500);
	}

	unique_ptr<EngineBoard> IEngine = unique_ptr<EngineBoard>(new EngineBoard);
	IEngine->StartEngines();

	thread sensorRun(deviceRun, ref(ISensor));
	
	vector<float> sensorInput(3, 0.0);
	vector<float> controlInput(4, 0.0);
	vector<float> controlOutput(18, 0.0);
		
	Socket->SetControlOutput(controlOutput);
	
	while ( true ) {
		
		sensorInput = ISensor->GetAngles();
			
		controlInput = Socket->GetControlInput();

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
	}

	return EXIT_SUCCESS;
}