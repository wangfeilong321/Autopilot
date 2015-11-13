#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include "Interface.h"
#include <winsock2.h>
#include <vector>
#include <string>

class TCPSocket : public Interface {
public:
	TCPSocket(const std::string& address, u_short port);
	~TCPSocket();

	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual bool SetControlOutput(const std::vector<double>& output);
	virtual std::vector<double> GetControlInput();

protected:
	virtual bool Send();
	virtual bool Recv();

private:
	std::vector<double> controlOutput;
	std::vector<double> controlInput;
	SOCKET sckt;
	struct sockaddr_in scktName;
	bool connected;
	bool canRead;
	static const size_t BUFFER_SIZE = 128;
	std::string data;
};

#endif
