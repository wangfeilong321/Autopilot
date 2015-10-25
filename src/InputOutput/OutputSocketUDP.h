#ifndef OUTPUTSOCKETUDP_H
#define OUTPUTSOCKETUDP_H

#include "OutputSocket.h"
#include <winsock2.h>

class OutputSocketUDP : public OutputSocket {
public:
	OutputSocketUDP(const std::string& address, u_short port);
	~OutputSocketUDP();

	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();
	virtual void SetControlOutput(const std::vector<double>& controlOutput);

protected:
	virtual bool Send(const std::vector<double>& data);

private:
	std::vector<double> toSend;
	SOCKET sckt;
	bool connected;
	struct sockaddr_in siOther;
	int slen;
};

#endif
