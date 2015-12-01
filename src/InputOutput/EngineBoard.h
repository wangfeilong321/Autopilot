#ifndef ENGINE_BOARD_H
#define ENGINE_BOARD_H

#include <mutex>
#include <condition_variable>
#include <memory>
#include <thread>
#include <Engine.h>
#include <Timer.h>

class EngineBoard {
public:
	EngineBoard();

	void StartEngines();

private:
	void StartTimeout();

	std::unique_ptr<Engine> engine1;
	std::unique_ptr<Engine> engine2;
	std::unique_ptr<Engine> engine3;
	std::unique_ptr<Engine> engine4;
	std::mutex lock;
	std::condition_variable threadReadiness;
};

#endif
