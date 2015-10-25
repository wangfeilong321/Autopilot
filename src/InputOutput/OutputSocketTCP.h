#ifndef OUTPUTSOCKETTCP_H
#define OUTPUTSOCKETTCP_H

#include "OutputSocket.h"
#include <winsock2.h>

class OutputSocketTCP : public OutputSocket {
public:
	OutputSocketTCP(const std::string& address, u_short port);
	~OutputSocketTCP();

	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual void SetControlOutput(const std::vector<double>& controlOutput);

protected:
	virtual bool Send(const std::vector<double>& controlOutput);

private:
	std::vector<double> toSend;
	SOCKET sckt;
	struct sockaddr_in scktName;
	bool connected;
};

#endif
