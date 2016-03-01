#pragma once

#include <Interface.h>
#include <StateSpace.h>

#include <ppltasks.h>

using namespace Windows::Networking;
using namespace Windows::Networking::Sockets;
using namespace Concurrency;
using namespace Windows::Storage::Streams;
using namespace Windows::System;
using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::System::Threading;

class SocketBoard : public Interface {
public:
	SocketBoard(const std::shared_ptr<StateSpace>& ISS);
	virtual ~SocketBoard();
	
	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();

protected:
	void doRead();
	void doWrite();

private:
	bool ifConnected;
	StreamSocket^ socket;
	DataWriter^ writer;
	DataReader^ reader;
	std::shared_ptr<StateSpace> IState;
};