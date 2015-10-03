#include "InputSocketTCP.h"
#include <iostream>

using namespace std;

InputSocketTCP::InputSocketTCP(u_short port) : InputSocket (port) {
	sckt = sckt_in = 0;
	WSADATA wsaData;
	if ( !WSAStartup(MAKEWORD(1,1), &wsaData) ) 
	  cout << "Winsock DLL loaded" << endl;
	else 
	  cout << "Winsock DLL not initialized" << endl;
	
	sckt = socket(AF_INET, SOCK_STREAM, 0);
	cout << "Creating TCP socket on port " << port << endl;
		
	if ( sckt >= 0 ) {   //successfully created socket
	    memset(&scktName, 0, sizeof(struct sockaddr_in));
	    scktName.sin_family = AF_INET;
	    scktName.sin_port = htons(port);
		cout << "Successfully created socket for input on port " << port << endl;
	} else   // unsuccessful creating of the socket
	    cout << "Could not create socket for FDM input. Error:" << WSAGetLastError() << endl;
}

InputSocketTCP::~InputSocketTCP() {
	if (sckt) closesocket(sckt);
	if (sckt_in) closesocket(sckt_in);
}

bool InputSocketTCP::Connected() {
	return sckt_in != 0;
}

bool InputSocketTCP::IfGetData() {
	return data != "";
}

void InputSocketTCP::Connect() {
	unsigned long NoBlock = false;
	int length = sizeof(struct sockaddr_in);
	if ( bind(sckt, (struct sockaddr*)&scktName, length) == 0 ) {   //successfull bind
		cout << "Successfully bound to socket for input" << endl;
		if (listen(sckt, 5) >= 0 ) {  // successful listen
			ioctlsocket(sckt, FIONBIO, &NoBlock);
			sckt_in = accept(sckt, (struct sockaddr*)&scktName, &length);
		} else {   // successful listen
			cout << "Could not listen. Error:" << WSAGetLastError() << endl;
		}
	} else {  // unsuccessful bind 
		cout << "Could not bind to socket for input. Error:" << WSAGetLastError() << endl;
	}
}

bool InputSocketTCP::Run() {
	return InputSocket::Run();
}

std::vector<double> InputSocketTCP::GetControlInput() {
	return InputSocket::GetControlInput();
}

string InputSocketTCP::Receive(void) {
	char buf[128];
	int len = sizeof(struct sockaddr_in);
	int num_chars = 0;
	unsigned long NoBlock = true;
	data.clear();

	if (sckt_in <= 0) {
		sckt_in = accept(sckt, (struct sockaddr*)&scktName, &len);
		if (sckt_in > 0) {
			ioctlsocket(sckt_in, FIONBIO,&NoBlock);
		} 
	}

	if (sckt_in > 0) {
		while ((num_chars = recv(sckt_in, buf, sizeof buf, 0)) > 0) {
		  data.append(buf, num_chars);
		}
			
		// when nothing received and the error isn't "would block"
		// then assume that the client has closed the socket.
		if (num_chars == 0) {
			DWORD err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK) {
				cout << "Socket Closed. back to listening" << endl;
				closesocket (sckt_in);
				sckt_in = 0;
			}
		}
	}
	
	return data;
}

void InputSocketTCP::Debug(int /*from*/) {}