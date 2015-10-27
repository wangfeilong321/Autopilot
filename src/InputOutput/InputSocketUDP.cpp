#include "InputSocketUDP.h"
#include <iostream>

using namespace std;

InputSocketUDP::InputSocketUDP(const std::string& address, u_short port) : InputSocket (port), connected(false), sckt(0) {
	WSADATA wsaData;
	if ( !WSAStartup(MAKEWORD(1,1), &wsaData) ) 
	  cout << "Winsock DLL loaded" << endl;
	else 
		cout << "Winsock DLL not initialized" << endl;
	
	sckt = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	cout << "Creating UDP socket on port " << port << endl;
	
	if ( sckt >= 0 ) {   //successfully created socket
	  memset(&scktName, 0, sizeof(struct sockaddr_in));
	  scktName.sin_family = AF_INET;
	  scktName.sin_port = htons(port);
		scktName.sin_addr.s_addr = inet_addr(address.c_str());
		cout << "Successfully created socket for input on port: " << port << endl;
	} else   // unsuccessful creating of the socket
	  cout << "Could not create socket for FDM input. Error: " << WSAGetLastError() << endl;
}

InputSocketUDP::~InputSocketUDP() {
	if (sckt) closesocket(sckt);
}

bool InputSocketUDP::Connected() {
	return connected;
}

void InputSocketUDP::Connect() {
	int length = sizeof(struct sockaddr_in);
	if( bind(sckt, (struct sockaddr*)&scktName, length) == 0 ) {   //successfull bind
		cout << "Successfully bound to socket for input" << endl;
		connected = true;
	} else {   // unsuccessful bind 
		cout << "Could not bind to socket for input. Error: " << WSAGetLastError() << endl;
		connected = false;
	}
}

bool InputSocketUDP::Run() {
	return InputSocket::Run();
}

std::vector<double> InputSocketUDP::GetControlInput() {
	return InputSocket::GetControlInput();
}

string InputSocketUDP::Receive(void) {
	char buf[BUFFER_SIZE];
	memset(buf, 0, BUFFER_SIZE);

	struct sockaddr_in inc_addr;
	int inc_len = sizeof(inc_addr);
	int recv_chars = recvfrom(sckt, buf, BUFFER_SIZE, 0, (struct sockaddr *)&inc_addr, &inc_len);
	if (recv_chars > 0)
		data.assign(buf, recv_chars);
	else {
		int error = WSAGetLastError();
		cout << "Could not read socket. Error: " << error << endl;
	}
	return data;
}