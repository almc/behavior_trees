#include <behavior_trees/robot_config.h>
#include <iostream>

po::options_description desc("Allowed options");

void setupCmdLineReader()
{
	addCmdLineOption("agent");
}

void addCmdLineOption(std::string argumentName)
{
	desc.add_options()
		(argumentName.c_str(), po::value<std::string>(), ("Custom" + argumentName).c_str());
}

std::string readCmdLineOption(int argc, char** argv, std::string argument_name)
{
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);
	std::string return_string = "";
	if (vm.count(argument_name))
	{
		return_string = vm[argument_name].as<std::string>();
//		std::cout << argument_name << " was set to " << return_string << std::endl;
	}
	else
	{
//		std::cout << argument_name << " was not set" << std::endl;;
	}
	return return_string;
}

// std::string readRobotIPFromCmdLine(int argc, char** argv)
// {
// 	std::string robot_ip = readCmdLineOption(argc, argv, "robot_ip");
// 	if (robot_ip.length() < 1)
// 	{
// 		robot_ip = "localhost";
// 	}
// 	return robot_ip;
// }

// std::string readColorFromCmdLine(int argc, char** argv)
// {
// 	std::string color_string = readCmdLineOption(argc, argv, "color");
// 	if (color_string.length() < 1)
// 	{
// 		color_string = "red";
// 	}
// 	return color_string;
// }

std::string readAgentFromCmdLine(int argc, char** argv)
{
	std::string agent_string = readCmdLineOption(argc, argv, "agent");
	if (agent_string.length() < 1)
	{
		agent_string = "default";
	}
	return agent_string;
}
