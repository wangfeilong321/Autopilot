#ifndef INPUTSOCKET_H
#define INPUTSOCKET_H

#include <string>
#include <vector>
#include "Interface.h"
#include <winsock2.h>

class InputSocket : public Interface {
public:
	InputSocket(u_short port);
	virtual ~InputSocket();
	
	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual bool IfGetData();
	virtual std::vector<double> GetControlInput();

protected:
	virtual std::string Receive(void);
	virtual void Debug(int from);

private:
	std::string data;
	std::vector<double> controlInput;
};

#endif
