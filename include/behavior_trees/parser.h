#ifndef PARSER_H_
#define PARSER_H_

#include <iostream>
#include <vector>

extern std::vector<std::string> global_varname;
extern std::vector<double>      global_varvalue;

int process_substring(std::string sub);

int parse_file(std::string name);

#endif
