#include "OutputSocketUDP.h"
#include <iostream>

using namespace std;

OutputSocketUDP::OutputSocketUDP(const std::string& address, u_short port) : OutputSocket(port), sckt(0) {
	WSADATA wsaData;
	if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
		cout << "Winsock DLL loaded" << endl;
	else
		cout << "Winsock DLL not initialized" << endl;

	sckt = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	cout << "Creating UDP socket on port " << port << endl;

	if (sckt >= 0) {   //successfully created socket
		memset(&siOther, 0, sizeof(struct sockaddr_in));
		siOther.sin_family = AF_INET;
		siOther.sin_port = htons(port);
		siOther.sin_addr.s_addr = inet_addr(address.c_str());
		slen = sizeof(siOther);
		cout << "Successfully created socket for output on port " << port << endl;
	}
	else   // unsuccessful creating of the socket
		cout << "Could not create socket for FDM output. Error:" << WSAGetLastError() << endl;
}

OutputSocketUDP::~OutputSocketUDP() {
	if (sckt) closesocket(sckt);
}

bool OutputSocketUDP::Run() {
	return Send(toSend);
}

bool OutputSocketUDP::Connected() {
	return connected;
}

void OutputSocketUDP::Connect() {
	connected = true;
}

void OutputSocketUDP::SetControlOutput(const std::vector<double>& controlOutput) {
	toSend.assign(controlOutput.begin(), controlOutput.end());
}

bool OutputSocketUDP::Send(const std::vector<double>& data) {
	string buf;
	for (size_t i = 0; i < data.size(); ++i) {
		buf.append(to_string(data[i]));
		if(i < data.size() - 1)
			buf.append(",");
	}
	buf.append("\r\n");
	if (sendto(sckt, buf.c_str(), buf.length(), 0, (struct sockaddr*)&siOther, slen) == SOCKET_ERROR) {
		int error = WSAGetLastError();
		cout << "sendto() failed with error code: " << error << endl;
		return false;
	}
	return true;
}