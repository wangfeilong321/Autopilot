#include <EngineBoard.h>
#include <Base.h>

using namespace std;

EngineBoard::EngineBoard() {
	engine1 = unique_ptr<Engine>(new Engine(ENGINE_PIN_1));
	engine2 = unique_ptr<Engine>(new Engine(ENGINE_PIN_2));
	engine3 = unique_ptr<Engine>(new Engine(ENGINE_PIN_3));
	engine4 = unique_ptr<Engine>(new Engine(ENGINE_PIN_4));
}

void EngineBoard::StartEngines() {
	engine1->StartEngine();
	engine2->StartEngine();
	engine3->StartEngine();
	engine4->StartEngine();
	//wait here for all threads to be ready.
	unique_lock<mutex> lk(lock);
	threadReadiness.wait(lk, [](){return Engine::GetReadyAll();});
	StartTimeout();
}

void EngineBoard::StartTimeout() {
	const int timeout = 5;
	Timer timer(true);
	auto start = chrono::high_resolution_clock::now();
	while (timer.Elapsed().count() < timeout) {
		start = chrono::high_resolution_clock::now();
	}
	Engine::SetContinue(true);
}