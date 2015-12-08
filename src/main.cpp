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

std::atomic<int> deltaTmcs;

void SensorRun(const unique_ptr<SensorBoard>& ISensor) {
	while (ISensor->Run());
}

void EngineRun(const unique_ptr<EngineBoard>& IEngine) {
	auto start = chrono::high_resolution_clock::now();
	while (true) {
		auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
		if (duration.count() >= deltaTmcs) {
			IEngine->OnTick();
			start = chrono::high_resolution_clock::now();
		}
	}
}

void Calibrate(const unique_ptr<TCPSocket>& ISocket, const unique_ptr<EngineBoard>& IEngine) {
	{
		//wait here until timeout for battery connection is expired.
		int timer = 15;

		vector<float> controlOutput(18, 0.0);

		auto start = chrono::high_resolution_clock::now();
		while (timer >= 0) {

			auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start);
			if (duration.count() >= 1) {
				
				controlOutput[0] = timer*1.0f; //time;
				controlOutput[1] = 128.0f / 0.3028f; //altitudeASL
				controlOutput[2] = 0.0; //vNorth
				controlOutput[3] = 0.0; //vEast
				controlOutput[4] = 0.0; //vDown
				controlOutput[5] = 0.0; //U
				controlOutput[6] = 0.0; //V
				controlOutput[7] = 0.0; //W
				controlOutput[8] = 0.0; //Roll
				controlOutput[9] = 0.0; //Pitch
				controlOutput[10] = 0.0; //Yaw
				controlOutput[11] = 0.0; //P
				controlOutput[12] = 0.0; //Q
				controlOutput[13] = 0.0; //R
				controlOutput[14] = 0.0; //velDotX
				controlOutput[15] = 0.0; //velDotY
				controlOutput[16] = 0.0; //velDotZ
				controlOutput[17] = 0.0; //vcas
				ISocket->SetControlOutput(controlOutput);

				timer--; //minus 1 second each pass
								
				ISocket->GetControlInput();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
	{
		const int calibrationDeltaTmcs = 2000;
		const int calibrationTime = 5;
		Timer timer(true);
		auto start = chrono::high_resolution_clock::now();
		while (timer.Elapsed().count() < calibrationTime) {
			auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
			if (duration.count() >= calibrationDeltaTmcs) {
				IEngine->OnTick();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
	{
		const int calibrationDeltaTmcs = 1000;
		const int calibrationTime = 5;
		Timer timer(true);
		auto start = chrono::high_resolution_clock::now();
		while (timer.Elapsed().count() < calibrationTime) {
			auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
			if (duration.count() >= calibrationDeltaTmcs) {
				IEngine->OnTick();
				start = chrono::high_resolution_clock::now();
			}
		}
	}
}

int main(Platform::Array<Platform::String^>^ args) {
	
	unique_ptr<TCPSocket> ISocket = unique_ptr<TCPSocket>(new TCPSocket("192.168.0.10", 3001));
	while (!ISocket->Connected()) {
		ISocket->Connect();
		Sleep(500);
	}
	
	unique_ptr<SensorBoard> ISensor = unique_ptr<SensorBoard>(new SensorBoard());
	while (!ISensor->Connected()) {
		ISensor->Connect();
		Sleep(500);
	}
	
	thread sensorRun(SensorRun, ref(ISensor));

	unique_ptr<EngineBoard> IEngine = unique_ptr<EngineBoard>(new EngineBoard(ENGINE_PIN_1, ENGINE_PIN_2, ENGINE_PIN_3, ENGINE_PIN_4));
	Calibrate(ISocket, IEngine);																							
																																						
	thread engineRun(EngineRun, ref(IEngine));

	vector<float> sensorInput(3, 0.0);
	vector<float> controlInput(4, 0.0);
	vector<float> controlOutput(18, 0.0);
	
	while ( true ) {
		
		sensorInput = ISensor->GetAngles();
			
		controlOutput[0] = 0.0; //time;
		controlOutput[1] = 128.0f / 0.3028f; //altitudeASL
		controlOutput[2] = 0.0; //vNorth
		controlOutput[3] = 0.0; //vEast
		controlOutput[4] = 0.0; //vDown
		controlOutput[5] = 0.0; //U
		controlOutput[6] = 0.0; //V
		controlOutput[7] = 0.0; //W
		controlOutput[8] = sensorInput[0] * degtorad; //Roll
		controlOutput[9] = sensorInput[1] * degtorad; //Pitch
		controlOutput[10] = sensorInput[2] * degtorad; //Yaw
		controlOutput[11] = 0.0; //P
		controlOutput[12] = 0.0; //Q
		controlOutput[13] = 0.0; //R
		controlOutput[14] = 0.0; //velDotX
		controlOutput[15] = 0.0; //velDotY
		controlOutput[16] = 0.0; //velDotZ
		controlOutput[17] = 0.0; //vcas
		ISocket->SetControlOutput(controlOutput);

		controlInput = ISocket->GetControlInput();
		if (controlInput.size() == 4)
			deltaTmcs = static_cast<int>(1000 + controlInput[3] * 1000);
	}

	return EXIT_SUCCESS;
}