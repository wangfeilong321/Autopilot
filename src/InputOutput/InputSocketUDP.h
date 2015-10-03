#ifndef INPUTSOCKETUDP_H
#define INPUTSOCKETUDP_H

#include "InputSocket.h"
#include <winsock2.h>

class InputSocketUDP : public InputSocket {
public:
	InputSocketUDP(u_short port);
	~InputSocketUDP();
	
	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual bool IfGetData();
	virtual std::vector<double> GetControlInput();

private:
	virtual std::string Receive(void);
	virtual void Debug(int from);

	std::string data;
  SOCKET sckt;
	struct sockaddr_in scktName;
	bool connected;
};

#endif