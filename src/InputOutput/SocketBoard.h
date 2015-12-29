#ifndef SOCKETBOARD_H
#define SOCKETBOARD_H

#include <Interface.h>
#include <ppltasks.h>

using namespace Windows::Networking;
using namespace Windows::Networking::Sockets;
using namespace Concurrency;
using namespace Windows::Storage::Streams;
using namespace Windows::System;
using namespace Platform;

class SocketBoard : public Interface {
public:
	SocketBoard();
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
};

#endif SOCKET_H
