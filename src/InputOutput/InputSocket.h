#ifndef INPUTSOCKET_H
#define INPUTSOCKET_H

#include <string>
#include <vector>
#include <winsock2.h>
#include "Interface.h"

class InputSocket : public Interface {
public:
	InputSocket(u_short port);
	virtual ~InputSocket();
	
	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual std::vector<double> GetControlInput();

protected:
	virtual std::string Receive(void);

private:
	std::string data;
	std::vector<double> controlInput;
};

#endif
