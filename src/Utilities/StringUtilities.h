#ifndef STRINGUTILS_H
#define STRINGUTILS_H

#include <string>
#include <vector>

extern std::string& trim_left(std::string& str);
extern std::string& trim_right(std::string& str);
extern std::string& trim(std::string& str);
extern bool is_number(const std::string& str);
extern std::vector <std::string> split(std::string str, char d);

#endif

