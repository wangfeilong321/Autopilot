#pragma once

#include <Interface.h>
#include <memory>
#include <StateSpace.h>
#include <PID.h>

class Channel : public Interface {
public:
	Channel(const std::shared_ptr<StateSpace>& ISS);
	virtual ~Channel() = default;

	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();

private:
	PID PidRoll, PidPitch, PidYaw;
	std::shared_ptr<StateSpace> IState;
};