#include <asio/deadline_timer.hpp>
#include <asio/basic_deadline_timer.hpp>
#include <asio/error_code.hpp>
#include <asio/system_error.hpp>
#include <asio/io_service.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/read_until.hpp>
#include <asio/streambuf.hpp>
#include <asio/write.hpp>
#include <asio.hpp>

#include <SensorBoard.h>
#include <EngineBoard.h>
#include <PID.h>
#include <Timer.h>
#include <StringUtilities.h>

#include <memory>
#include <string>
#include <array>
#include <thread>

using namespace std;
using namespace chrono;

atomic<int> deltaTmcs = 2000;

class client {
public:
	
	client(asio::io_service& io_service, const shared_ptr<SensorBoard>& Sensor) : socket(io_service), TimerClock(10), ISensor(Sensor) {}
	
	void start(asio::ip::tcp::endpoint endpoint) {
		socket.async_connect(endpoint, [this](asio::error_code ec) {
			if (!ec) {
				if (socket.is_open()) {
					timer();
					do_write();
					do_read();
				}
			}
			else {
				stop();
			}
		});
	}

	void stop() {
		socket.close();
	}
	
private:
	
	void do_read() {
		asio::async_read_until(socket, input_buffer, string("\r\n"), [this](const asio::error_code& ec, size_t length) {
			if (!ec) {
				string data;
				istream is(&input_buffer);
				getline(is, data);
				
				auto start = data.find_first_not_of(string("\r\n"), 0);
				if (start == string::npos)
					return;
				string line = data.substr(start, length - start);
				if (line.size() == 0)
					return;

				line = trim(line);

				vector <string> tokens = split(line, ',');

				if ((!is_number(tokens[0])) ||
					(!is_number(tokens[1])) ||
					(!is_number(tokens[2])) ||
					(!is_number(tokens[3]))) {
					return;
				}
				else {
					controlInput[0] = stof(trim(tokens[0]));
					controlInput[1] = stof(trim(tokens[1]));
					controlInput[2] = stof(trim(tokens[2]));
					controlInput[3] = stof(trim(tokens[3]));
					deltaTmcs = static_cast<int>(1000 + controlInput[3] * 1000);
				}
				do_write();
			}
			else {
				stop();
			}
		});
	}
	
	void do_write() {
		
		sensorInput = ISensor->GetAngles();

		controlOutput[0] = TimerClock*1.0f; //timer;
		controlOutput[1] = 38.0477f / 0.3028f; //altitudeASL ft
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

		string commandToSend;
		for (auto i = 0u; i < controlOutput.size(); ++i) {
			commandToSend.append(to_string(controlOutput[i]));
			if (i < controlOutput.size() - 1)
				commandToSend.append(",");
		}
		commandToSend.append("\r\n");
		size_t length = commandToSend.length();
		asio::async_write(socket, asio::buffer(commandToSend.c_str(), length), [this](const asio::error_code& ec, size_t /*length*/) {
			if (!ec) {
				do_read();
			}
			else {
				stop();
			}
		});
	}

	void timer() {
		auto start = high_resolution_clock::now();
		while (TimerClock > 0) {
			auto duration = duration_cast<seconds>(high_resolution_clock::now() - start);
			if (duration.count() >= 1) {
				TimerClock--;
				start = high_resolution_clock::now();
			}
		}
	}

private:
	asio::ip::tcp::socket socket;
	asio::streambuf input_buffer;
	array<float, 4> controlInput;
	array<float, 18> controlOutput;
	array<float, 3> sensorInput;
	shared_ptr<SensorBoard> ISensor;
	int TimerClock;
};

int main(Platform::Array<Platform::String^>^ args) {
	try {
		shared_ptr<SensorBoard> ISensor = shared_ptr<SensorBoard>(new SensorBoard);
		ISensor->Connect();
		if(!ISensor->Connected())
			return EXIT_FAILURE;

		unique_ptr<EngineBoard> IEngine = unique_ptr<EngineBoard>(new EngineBoard(ENGINE_PIN_1, ENGINE_PIN_2, ENGINE_PIN_3, ENGINE_PIN_4));

		asio::io_service service;
		asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string("192.168.0.10"), 3001);
		client c(service, ISensor);
		c.start(endpoint);

		thread t([&service]() { service.run(); });
		
		auto start1 = high_resolution_clock::now();
		auto start2 = high_resolution_clock::now();
		auto start3 = high_resolution_clock::now();
		auto start4 = high_resolution_clock::now();

		while (ISensor->Run()) {

			auto duration1 = duration_cast<microseconds>(high_resolution_clock::now() - start1);
			if (duration1.count() >= deltaTmcs) {
				IEngine->Engine1OnTick();
				start1 = high_resolution_clock::now();
			}
			auto duration2 = duration_cast<microseconds>(high_resolution_clock::now() - start2);
			if (duration2.count() >= deltaTmcs) {
				IEngine->Engine2OnTick();
				start2 = high_resolution_clock::now();
			}
			auto duration3 = duration_cast<microseconds>(high_resolution_clock::now() - start3);
			if (duration3.count() >= deltaTmcs) {
				IEngine->Engine3OnTick();
				start3 = high_resolution_clock::now();
			}
			auto duration4 = duration_cast<microseconds>(high_resolution_clock::now() - start4);
			if (duration4.count() >= deltaTmcs) {
				IEngine->Engine4OnTick();
				start4 = high_resolution_clock::now();
			}
		}

		c.stop();
		t.join();

	}
	catch (std::exception& e) {
		string what = e.what();
	}
	
	return EXIT_SUCCESS;
}