#ifndef TIMER_H
#define TIMER_H

#include <chrono>

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

#endif