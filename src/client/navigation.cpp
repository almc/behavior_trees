#include "behavior_trees/navigation.h"

extern Node *node_cursor;

bool navigate_left()
{
	if (node_cursor->get_prev_brother() != NULL)
	{
		node_cursor->set_highlighted(false);
		node_cursor = node_cursor->get_prev_brother();
		node_cursor->set_highlighted(true);
		return true;
	}
	else
	{
		std::cout << '\a' << std::endl;
		return false;
	}
}

bool navigate_right()
{
	if (node_cursor->get_next_brother() != NULL)
	{
		node_cursor->set_highlighted(false);
		node_cursor = node_cursor->get_next_brother();
		node_cursor->set_highlighted(true);
		return true;
	}
	else
	{
		std::cout << '\a' << std::endl;
		return false;
	}
}

bool navigate_up()
{
	if (node_cursor->get_parent() != NULL)
	{
		node_cursor->set_highlighted(false);
		node_cursor = node_cursor->get_parent();
		node_cursor->set_highlighted(true);
		return true;
	}
	else
	{
		std::cout << '\a' << std::endl;
		return false;
	}
}

bool navigate_down()
{
	if (node_cursor->get_first_child() != NULL)
	{
		node_cursor->set_highlighted(false);
		node_cursor = node_cursor->get_first_child();
		node_cursor->set_highlighted(true);
		return true;
	}
	else
	{
		std::cout << '\a' << std::endl;
		return false;
	}
}

// this function needs to be checked
void set_node_state(STATE state)
{
	NODE_TYPE node_type = node_cursor->get_node_type();
	if (node_type == ACTION)
	{
		if (state != NODE_ERROR)
			node_cursor->set_overwrite(state, true);
		else
		{
			std::cout << '\a' << std::endl;
			std::cout << "Action Result Selected Error" << std::endl;
		}
	}
	else if (node_type == CONDITION)
	{
		if (state != NODE_ERROR && state != RUNNING)
			node_cursor->set_overwrite(state, true);
		else
		{
			std::cout << '\a' << std::endl;
			std::cout << "Condition Result Selected Error" << std::endl;
		}
	}
	else
	{
		std::cout << '\a' << std::endl;
		std::cout << "Node Is Not Action, Cannot be Modified" << std::endl;
	}
}

void reset_overwritten()
{
	node_cursor->set_overwrite(NODE_ERROR, false);
}

void reset_node_state()
{
	node_cursor->set_overwrite(NODE_ERROR, false);
}

void print_node_info()
{
	node_cursor->print_info();
}
