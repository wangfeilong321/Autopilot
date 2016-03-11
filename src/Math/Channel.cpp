#include <Channel.h>

Channel::Channel(const std::shared_ptr<StateSpace>& ISS) : IState(ISS) {}

void Channel::Connect() {}

bool Channel::Connected() {
	return PidPitch.Connected() && PidRoll.Connected() && PidYaw.Connected();
}

bool Channel::Run() {
	auto DesiredPitch = IState->getElevator();
	auto PitchError = DesiredPitch - IState->getPitch();
	PidPitch.SetInput(PitchError);
	PidPitch.Run();

	auto DesiredRoll = IState->getAileron();
	auto RollError = DesiredRoll - IState->getRoll();
	PidRoll.SetInput(RollError);
	PidRoll.Run();

	auto DesiredYaw = IState->getRudder();
	auto YawError = DesiredYaw - IState->getYaw();
	PidYaw.SetInput(YawError);
	PidYaw.Run();
	return true;
}