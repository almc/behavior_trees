#include "behavior_trees/parser.h"
#include "behavior_trees/node.h"

#include <sstream>
#include <fstream>

extern Node *node;
bool action_detected = false;

int process_substring(std::string sub)
{

	bool star_detected = false;
	if (sub.length() == 2)
	{
		if (sub.at(1) == '*')
		{
			star_detected = true;
			sub = sub.at(0);
		}
	}

	switch ( *(sub.c_str()) )
	{
	case '{': std::cout << "Open  Brace" << std::endl;
		if (!star_detected) node = new NodeSelector(node);
		else node = new NodeSelectorStar(node);
		break;
	case '}': std::cout << "Close Brace" << std::endl;
		node = node->get_parent(); break;
	case '[': std::cout << "Open  Bracket" << std::endl;
		if (!star_detected) node = new NodeSequence(node);
		else node = new NodeSequenceStar(node);
		break;
	case ']': std::cout << "Close Bracket" << std::endl;
		node = node->get_parent(); break;
	case '/': std::cout << "Open  Slash" << std::endl;
		node = new NodeParallel(node); break;
	case '|': std::cout << "Close Slash" << std::endl;
		node = node->get_parent(); break;
	case 'A': std::cout << "ROS Action Detected" << std::endl;
		action_detected = true; break;
	default: break;
	}
	return 0;
}

int parse_file(std::string name)
{
	std::string line;
	std::ifstream file (name);
	if (!file.is_open())
	{
		std::cout << "Couldn't Open File" << std::endl;
		return 1;
	}
	std::cout << "File Called " << name << " Opened Correctly" << std::endl;

	while ( file.good() )
	{
		getline (file,line);
		std::cout << "Reading line: " << line << std::endl;
		std::istringstream iss (line);
		int i = 0;
		do
		{
			std::string sub;
			iss >> sub;
			std::cout << "Substring: " << " i: " << i
			          << " n: " << sub << std::endl;
			process_substring(sub);
			if (action_detected)
			{
				iss >> sub;
				std::cout << "ROS Action Detected: " << sub << std::endl;
				node = new NodeROS(node, sub);
				node = node->get_parent();
				action_detected = false;
			}
			i++;
		} while (iss);
	}
	file.close();
	return 0;
}
