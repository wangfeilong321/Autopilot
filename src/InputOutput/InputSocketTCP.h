#ifndef INPUTSOCKETTCP_H
#define INPUTSOCKETTCP_H

#include "InputSocket.h"
#include <winsock2.h>

class InputSocketTCP : public InputSocket {
public:
	InputSocketTCP(const std::string& address, u_short port);
	~InputSocketTCP();
	
	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual std::vector<double> GetControlInput();

private:
	virtual std::string Receive(void);

	std::string data;
  SOCKET sckt, sckt_in;
	struct sockaddr_in scktName;
};

#endif