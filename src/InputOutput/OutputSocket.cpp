#include "OutputSocket.h"

OutputSocket::OutputSocket(u_short /*port*/) {}

OutputSocket::~OutputSocket() {}

bool OutputSocket::Run() {
	return false;
}

bool OutputSocket::Connected() {
	return false;
}

void OutputSocket::Connect() {}

void OutputSocket::SetControlOutput(const std::vector<double>& /*controlOutput*/) {}

bool OutputSocket::Send(const std::vector<double>& /*data*/) {
	return false;
}