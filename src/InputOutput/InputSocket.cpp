#include "InputSocket.h"
#include "StringUtilities.h"

using namespace std;

InputSocket::InputSocket(u_short /*port*/) : data( string() ){
	controlInput.reserve(5);
}

InputSocket::~InputSocket() {}

bool InputSocket::Connected() {
	return false;
}

bool InputSocket::IfGetData() {
	return false;
}

void InputSocket::Connect() {}

bool InputSocket::Run() {
	string line, token;
	size_t start=0, string_start=0, string_end=0;

	data.append(Receive());  // get socket transmission if present

	if (data.size() > 0) {
		double time, aileron_cmd, elevator_cmd, rudder_cmd, throttle_cmd;
		
		string_start = data.find_first_not_of("\r\n", start);
		if (string_start == string::npos) return false;
		string_end = data.find_first_of("\r\n", string_start);
		if (string_end == string::npos) return false;
		line = data.substr(string_start, string_end-string_start);
		if (line.size() == 0) return false;

		line = trim(line);

		vector <string> tokens = split(line, ',');
		
		if ( (!is_number(tokens[0])) || 
			   (!is_number(tokens[1])) || 
			   (!is_number(tokens[2])) || 
			   (!is_number(tokens[3])) || 
			   (!is_number(tokens[4])) ) {
			data.clear();
			return false;
		} else {
			time = stod(trim(tokens[0]));
			aileron_cmd = stod(trim(tokens[1]));
			elevator_cmd = stod(trim(tokens[2]));
			rudder_cmd = stod(trim(tokens[3]));
			throttle_cmd = stod(trim(tokens[4]));
			controlInput.clear();
			controlInput.push_back(time);
			controlInput.push_back(aileron_cmd);
			controlInput.push_back(elevator_cmd);
			controlInput.push_back(rudder_cmd);
			controlInput.push_back(throttle_cmd);
			data = data.erase(string_start, string_end + 1);
			return true;
		}
	}
	return false;
}

std::vector<double> InputSocket::GetControlInput() {
	return controlInput;
}

string InputSocket::Receive(void) {
	return string();
}

void InputSocket::Debug(int /*from*/ ) {}