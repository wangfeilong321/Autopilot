#include "OutputSocketTCP.h"
#include <iostream>

using namespace std;

OutputSocketTCP::OutputSocketTCP(const std::string& address, u_short port) : OutputSocket(port) {
	sckt = 0;
	WSADATA wsaData;
	if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
		cout << "Winsock DLL loaded" << endl;
	else
		cout << "Winsock DLL not initialized" << endl;

	sckt = socket(AF_INET, SOCK_STREAM, 0);
	cout << "Creating TCP socket on port " << port << endl;

	if (sckt >= 0) {   //successfully created socket
		memset(&scktName, 0, sizeof(struct sockaddr_in));
		scktName.sin_family = AF_INET;
		scktName.sin_port = htons(port);
		scktName.sin_addr.s_addr = inet_addr(address.c_str());
		cout << "Successfully created socket for output on port: " << port << endl;
	}
	else   // unsuccessful creating of the socket
		cout << "Could not create socket for FDM output. Error: " << WSAGetLastError() << endl;
}

OutputSocketTCP::~OutputSocketTCP() {
	if (sckt) closesocket(sckt);
}

bool OutputSocketTCP::Connected() {
	return connected;
}

void OutputSocketTCP::Connect() {
	if (sckt > 0) {
		int len = sizeof(struct sockaddr_in);
		if (connect(sckt, (struct sockaddr*)&scktName, len) == 0) {   // successful
			cout << "Successfully connected to socket for output" << endl;
			connected = true;
		}
		else                // unsuccessful
			cout << "Could not connect to socket for output. Error: " << WSAGetLastError() << endl;
	}
}

bool OutputSocketTCP::Run() {
	return Send(toSend);
}

void OutputSocketTCP::SetControlOutput(const std::vector<double>& controlOutput) {
	toSend.assign(controlOutput.begin(), controlOutput.end());
}

bool OutputSocketTCP::Send(const std::vector<double>& data) {
	string buf;
	for (size_t i = 0; i < data.size(); ++i) {
		buf.append(to_string(data[i]));
		if (i < data.size() - 1)
			buf.append(",");
	}
	buf.append("\r\n");
	if (send(sckt, buf.c_str(), buf.length(), 0) == SOCKET_ERROR) {
		int error = WSAGetLastError();
		cout << "sendto() failed with error code: " << error << endl;
		return false;
	}
	return true;
}

