#include <StringUtilities.h>

using namespace std;

string& trim_left(string& str) {
	while (str.size() && isspace((unsigned char)str[0])) {
	  str = str.erase(0,1);
	}
	return str;
}

string& trim_right(string& str) {
	while (str.size() && isspace((unsigned char)str[str.size()-1])) {
	  str = str.erase(str.size()-1,1);
	}
	return str;
}

string& trim(string& str) {
	if (str.size() == 0) return str;
	string temp_str = trim_right(str);
	return str = trim_left(temp_str);
}

bool is_number(const string& str) {
	if (str.size())
	  return (str.find_first_not_of("+-.0123456789Ee") == string::npos);
	else
	  return false;
}

vector <string> split(string str, char d) {
	vector <string> str_array;
	size_t index=0;
	string temp = "";
	
	trim(str);
	index = str.find(d);
	while (index != string::npos) {
	  temp = str.substr(0,index);
	  trim(temp);
	  if (temp.size() > 0) str_array.push_back(temp);
	  str = str.erase(0,index+1);
	  index = str.find(d);
	}
	if (str.size() > 0) {
	  temp = trim(str);
	  if (temp.size() > 0) str_array.push_back(temp);
	}
	return str_array;
}