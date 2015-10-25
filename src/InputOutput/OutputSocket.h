#ifndef OUTPUTSOCKET_H
#define OUTPUTSOCKET_H

#include <string>
#include <vector>
#include "Interface.h"
#include <winsock2.h>

class OutputSocket : public Interface {
public:
	OutputSocket(u_short port);
	virtual ~OutputSocket();

	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual void SetControlOutput(const std::vector<double>& controlOutput);

protected:
	virtual bool Send(const std::vector<double>& data);
};

#endif
