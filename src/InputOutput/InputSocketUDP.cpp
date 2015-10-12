#include "InputSocketUDP.h"
#include "Base.h"

using namespace std;

InputSocketUDP::InputSocketUDP(u_short port) : InputSocket (port), connected(false) {
	sckt = 0;
	WSADATA wsaData;
	if ( !WSAStartup(MAKEWORD(1,1), &wsaData) ) 
	  logFile << "Winsock DLL loaded" << endl;
	else 
	  logFile << "Winsock DLL not initialized" << endl;
	
	sckt = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	logFile << "Creating UDP socket on port " << port << endl;
	
	if ( sckt >= 0 ) {   //successfully created socket
	  memset(&scktName, 0, sizeof(struct sockaddr_in));
	  scktName.sin_family = AF_INET;
	  scktName.sin_port = htons(port);
		scktName.sin_addr.s_addr = htonl(INADDR_ANY);
		logFile << "Successfully created socket for input on port " << port << endl;
	} else   // unsuccessful creating of the socket
	  logFile << "Could not create socket for FDM input. Error:" << WSAGetLastError() << endl;
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
	int length = sizeof(struct sockaddr_in);
	if( bind(sckt, (struct sockaddr*)&scktName, length) == 0 ) {   //successfull bind
		logFile << "Successfully bound to socket for input" << endl;
		connected = true;
	} else {   // unsuccessful bind 
		logFile << "Could not bind to socket for input. Error:" << WSAGetLastError() << endl;
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
	memset(buf, 0, 128);
	int num_chars = 0;
	data.clear();

	struct sockaddr_in inc_addr;
	int inc_len = sizeof(inc_addr);

	while (num_chars <= 128) {
		int recv_chars = recvfrom(sckt, buf, 128, 0, (struct sockaddr *)&inc_addr, &inc_len);
		if (recv_chars > 0) {
			data = data.append(buf, recv_chars);		
			num_chars += recv_chars;
			memset(buf, 0, 128);
		}
	}
	
	return data;
}

void InputSocketUDP::Debug(int /*from*/) {}