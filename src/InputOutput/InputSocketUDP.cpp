#include "InputSocketUDP.h"
#include <iostream>

using namespace std;

InputSocketUDP::InputSocketUDP(u_short port) : InputSocket (port), connected(false) {
	sckt = 0;
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
		scktName.sin_addr.s_addr = htonl(INADDR_ANY);
		cout << "Successfully created socket for input on port " << port << endl;
	} else   // unsuccessful creating of the socket
	  cout << "Could not create socket for FDM input. Error:" << WSAGetLastError() << endl;
}

InputSocketUDP::~InputSocketUDP() {
	if (sckt) closesocket(sckt);
}

bool InputSocketUDP::Connected() {
	return connected;
}

bool InputSocketUDP::IfGetData() {
	return data != "";
}

void InputSocketUDP::Connect() {
	unsigned long NoBlock = false;
	int length = sizeof(struct sockaddr_in);
	if( bind(sckt, (struct sockaddr*)&scktName, length) == 0 ) {   //successfull bind
		cout << "Successfully bound to socket for input" << endl;
		ioctlsocket(sckt, FIONBIO, &NoBlock);
		connected = true;
	} else {   // unsuccessful bind 
		cout << "Could not bind to socket for input. Error:" << WSAGetLastError() << endl;
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
	char buf[128];
	int num_chars = 0;
	data.clear();

	struct sockaddr_in inc_addr;
	int inc_len = sizeof(inc_addr);

	if (sckt > 0) {
		while ((num_chars = recvfrom(sckt, buf, sizeof buf, 0, (struct sockaddr *)&inc_addr, &inc_len)) > 0) {
			data.append(buf, num_chars);
		} 

		// when nothing received and the error isn't "would block"
		// then assume that the client has closed the socket.
		if (num_chars == 0) {
			DWORD err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK) {
				cout << "Socket Closed. back to listening" << endl;
				closesocket (sckt);
				sckt = 0;
			}
		}
	}
	
	return data;
}

void InputSocketUDP::Debug(int /*from*/) {}