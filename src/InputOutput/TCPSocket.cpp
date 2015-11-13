#include "TCPSocket.h"
#include "StringUtilities.h"
#include <iostream>

using namespace std;

TCPSocket::TCPSocket(const std::string& address, u_short port) : sckt(0) {
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

TCPSocket::~TCPSocket() {
	if (sckt) closesocket(sckt);
}

bool TCPSocket::Connected() {
	return connected;
}

void TCPSocket::Connect() {
	if (sckt > 0) {
		int len = sizeof(struct sockaddr_in);
		if (connect(sckt, (struct sockaddr*)&scktName, len) == 0) {   // successful
			cout << "Successfully connected to socket for output" << endl;
			connected = true;
		}
		else {
			int error = WSAGetLastError();
			cout << "Could not connect to socket for output. Error: " << error << endl;
		}
	}
}

bool TCPSocket::Run() {
	return true;
}

bool TCPSocket::SetControlOutput(const vector<double>& output) {
	controlOutput.assign(output.begin(), output.end());
	return Send();
}

std::vector<double> TCPSocket::GetControlInput() {
	return Recv() ? controlInput : vector<double>();
}

bool TCPSocket::Send() {
	string buf;
	for (size_t i = 0; i < controlOutput.size(); ++i) {
		buf.append(to_string(controlOutput[i]));
		if (i < controlOutput.size() - 1)
			buf.append(",");
	}
	buf.append("\r\n");
	if (send(sckt, buf.c_str(), buf.length(), 0) == SOCKET_ERROR) {
		int error = WSAGetLastError();
		cout << "send: " << error << endl;
		return false;
	}
	return true;
}

bool TCPSocket::Recv() {
	char buf[BUFFER_SIZE];
	memset(buf, 0, BUFFER_SIZE);

	auto recv_chars = recv(sckt, buf, BUFFER_SIZE, 0);
	if (recv_chars == SOCKET_ERROR) {
		int error = WSAGetLastError();
		cout << "recv: " << error << endl;
		return false;
	}
	else if( recv_chars > 0 && recv_chars <= BUFFER_SIZE ) {
		data.assign(buf, recv_chars);
	}

	double time, aileron_cmd, elevator_cmd, rudder_cmd, throttle_cmd, rolltrim_cmd, pitchtrim_cmd, elevation;
	
	string line, token;
	size_t start = 0, string_start = 0, string_end = 0;

	string_start = data.find_first_not_of("\r\n", start);
	if (string_start == string::npos) 
		return false;
	string_end = data.find_first_of("\r\n", string_start);
	if (string_end == string::npos) 
		return false;
	line = data.substr(string_start, string_end - string_start);
	if (line.size() == 0) 
		return false;

	line = trim(line);

	vector <string> tokens = split(line, ',');

	if ((!is_number(tokens[0])) ||
			(!is_number(tokens[1])) ||
			(!is_number(tokens[2])) ||
			(!is_number(tokens[3])) ||
			(!is_number(tokens[4])) ||
			(!is_number(tokens[5])) ||
			(!is_number(tokens[6])) ||
			(!is_number(tokens[7]))) {
			return false;
	}
	else {
		controlInput.clear();

		time = stod(trim(tokens[0]));
		aileron_cmd = stod(trim(tokens[1]));
		elevator_cmd = stod(trim(tokens[2]));
		rudder_cmd = stod(trim(tokens[3]));
		throttle_cmd = stod(trim(tokens[4]));
		rolltrim_cmd = stod(trim(tokens[5]));
		pitchtrim_cmd = stod(trim(tokens[6]));
		elevation = stod(trim(tokens[7]));

		controlInput.push_back(time);
		controlInput.push_back(aileron_cmd);
		controlInput.push_back(elevator_cmd);
		controlInput.push_back(rudder_cmd);
		controlInput.push_back(throttle_cmd);
		controlInput.push_back(rolltrim_cmd);
		controlInput.push_back(pitchtrim_cmd);
		controlInput.push_back(elevation);
	}
	return true;
}
