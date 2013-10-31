#include <iostream>

#include "behavior_trees/node.h"
#include "behavior_trees/display.h"

// constructor for the root node
Node::Node()
{
	depth_ = 0;
	number_children_ = 0;
	children_number_ = 0;
	highlighted_ = false;
	overwritten_ = false;
	overwritten_result_ = NODE_ERROR;
	node_status_ = NODE_ERROR;
	child_status_ = NODE_ERROR;
	first_child_ = NULL;
	curr_child_ = NULL;
	exec_child_ = NULL;
	next_brother_ = NULL;
	prev_brother_ = NULL;
	parent_ = NULL;
}

// constructor for any node but root
Node::Node(Node* parent)
{
	depth_ = parent->get_depth() + 1;
	number_children_ = 0;
	children_number_ = 0;
	highlighted_ = false;
	overwritten_ = false;
	overwritten_result_ = NODE_ERROR;
	node_status_ = NODE_ERROR;
	child_status_ = NODE_ERROR;
	first_child_ = NULL;
	curr_child_ = NULL;
	exec_child_ = NULL;
	next_brother_ = NULL;
	prev_brother_ = NULL;
	parent_ = parent;
	parent_->add_child(this);
}

// set node status to default and its children too (recursive)
void Node::execute_reset_status()
{
	node_status_ = NODE_ERROR;

	exec_child_ = first_child_;
	for (int i = 0; i < number_children_; i++)
	{
		child_status_ = NODE_ERROR;
		exec_child_->execute_reset_status();
		exec_child_ = exec_child_->next_brother_;
	}
}

// add child distinguishing whether it is the first one or not
Node* Node::add_child(Node* my_child)
{
	if (number_children_ == 0)
	{
		std::cout << "***adding first child***" << std::endl;
		first_child_ = curr_child_ = my_child;
	}
	else
	{
		std::cout << "***adding brother***" << std::endl;
		curr_child_ = curr_child_->add_brother(my_child, number_children_);
	}

	number_children_++;
	return curr_child_;
}

// if it is not the first child, it will be treated as a brother instead
Node* Node::add_brother(Node* my_brother, int children_number)
{
	next_brother_ = my_brother;
	next_brother_->set_prev_brother(this);

	next_brother_->set_children_number(this->get_children_number() + 1);
	return next_brother_;
}

// prints the depth, status, and number of children of a certain node
void Node::print_info()
{
	std::cout << "Depth: " << depth_
	          << "  Status: " << node_status_
	          << "  N. Children: " << number_children_
	          << std::endl;
}

// prints the information of the whole tree recursively calling itself
void Node::print_subtree()
{
	print_info();
	if (number_children_ > 0)
	{
		first_child_->print_subtree();
	}
	if (next_brother_ != NULL)
	{
		next_brother_->print_subtree();
	}
}

// draws the tree in OpenGL trying to spread the nodes over the screen
void Node::draw_subtree(GLfloat parent_x, GLfloat parent_y,
                        int number_children, GLfloat parent_space)
{
	GLfloat separation = parent_space / (number_children);

	// std::cout << "depth:   " << depth_ << "    separation:   " << separation
	//           << "    parent space:   " << parent_space
	//           << "    number of children:   " << number_children << std::endl;

	GLfloat x = (GLfloat) (
		parent_x + (
			((GLfloat)children_number_) - ((GLfloat)number_children - 1) / 2
			) * separation);

	GLfloat y = (GLfloat) (parent_y - SPACE_HEIGHT);

	draw_node(x, y, this->get_node_type());
	draw_connector(parent_x, parent_y, x, y);
	draw_status(x, y, node_status_);
	draw_string(x - CURSOR_WIDTH, y + 2.1 * CURSOR_WIDTH,
	            get_node_name().c_str());

	if (this->get_node_type() == ACTION)
		draw_string(x - CURSOR_WIDTH, y + 2.2 * NODE_WIDTH,
		            get_ros_node_name().c_str());

	if (highlighted_)
		draw_cursor(x, y);

	if (number_children_ > 0)
	{
		first_child_->draw_subtree(x, y, number_children_, separation);
	}
	if (next_brother_ != NULL)
	{
		next_brother_->draw_subtree(parent_x, parent_y,
		                            number_children, parent_space);
	}
}

/* -------------------------------------------------------------------------- */
/* ----------------------------Basic Nodes----------------------------------- */
/* -------------------------------------------------------------------------- */

NodeSelector::NodeSelector(Node* node)
	: Node(node) {}

STATE NodeSelector::execute()
{
	std::cout << "Executing Selector" << std::endl;
	exec_child_ = first_child_;
	for (int i = 0; i < number_children_; i++)
	{
		child_status_ = exec_child_->execute();
		if (child_status_ == NODE_ERROR)
			return node_status_ = NODE_ERROR;
		else if (child_status_ == RUNNING)
			return node_status_ = RUNNING;
		else if (child_status_ == SUCCESS)
			return node_status_ = SUCCESS;
		exec_child_ = exec_child_->get_next_brother();
	}
	return node_status_ = FAILURE;
}

NodeSequence::NodeSequence(Node* node)
	: Node(node) {}


STATE NodeSequence::execute()
{
	std::cout << "Executing Sequence" << std::endl;
	exec_child_ = first_child_;
	for (int i = 0; i < number_children_; i++)
	{
		child_status_ = exec_child_->execute();
		if (child_status_ == NODE_ERROR)
			return node_status_ = NODE_ERROR;
		else if (child_status_ == RUNNING)
			return node_status_ =  RUNNING;
		else if (child_status_ == FAILURE)
			return node_status_ = FAILURE;
		exec_child_ = exec_child_->get_next_brother();
	}
	return node_status_ = SUCCESS;
}

NodeParallel::NodeParallel(Node* node)
	: Node(node) {}

STATE NodeParallel::execute()
{
	int number_failure = 0;
	int number_success = 0;
	int number_error = 0;
	std::cout << "Executing Parallel" << std::endl;
	exec_child_ = first_child_;
	for (int i = 0; i < number_children_; i++)
	{
		child_status_ = exec_child_->execute();
		if (child_status_ == NODE_ERROR)
			number_error++;
		else if (child_status_ == FAILURE)
			number_failure++;
		else if (child_status_ == SUCCESS)
			number_success++;
		exec_child_ = exec_child_->get_next_brother();
	}
	if (number_error > 0)
		return node_status_ = NODE_ERROR;
	else if (number_success > number_children_/2)
		return node_status_ = SUCCESS;
	else if (number_failure  > number_children_/2)
		return node_status_ = FAILURE;
	else
		return node_status_ = RUNNING;
}

STATE NodeRoot::execute()
{
	// there is no need to reset status because the nodes should
	// have a timeout to become NODE_ERROR after a while, if they
	// don't receive a feedback from the server.
	// execute_reset_status();
	std::cout << "---------- Executing Root ----------" << std::endl;
	return child_status_ = first_child_->execute();
}

/* -------------------------------------------------------------------------- */
/* ----------------------------Star Nodes------------------------------------ */
/* -------------------------------------------------------------------------- */

NodeSelectorStar::NodeSelectorStar(Node* node)
	: 	Node(node),
		current_running_child_(first_child_) {}

STATE NodeSelectorStar::execute()
{
	std::cout << "Executing Selector Star" << std::endl;

	if (current_running_child_ == NULL) {
		current_running_child_ = first_child_;
	}

	exec_child_ = current_running_child_;

	do
	{
		child_status_ = exec_child_->execute();
		if (child_status_ == NODE_ERROR)
		{
			return node_status_ = NODE_ERROR;
		}
		else if (child_status_ == RUNNING)
		{
			current_running_child_ = exec_child_;
			return node_status_ = RUNNING;
		}
		else if (child_status_ == SUCCESS)
		{
			current_running_child_ = NULL;
			return node_status_ = SUCCESS;
		}
		exec_child_=exec_child_->get_next_brother();
	} while(exec_child_ != NULL);

	current_running_child_ = NULL;
	return node_status_ = FAILURE;
}

NodeSequenceStar::NodeSequenceStar(Node* node)
	: 	Node(node),
		current_running_child_(first_child_) {}

STATE NodeSequenceStar::execute()
{
	std::cout << "Executing Sequence Star" << std::endl;

	if (current_running_child_ == NULL) {
		current_running_child_ = first_child_;
	}

	exec_child_ = current_running_child_;


	do
	{
		child_status_ = exec_child_->execute();
		if (child_status_ == NODE_ERROR)
		{
			return node_status_ = NODE_ERROR;
		}
		else if (child_status_ == RUNNING)
		{
			current_running_child_ = exec_child_;
			return node_status_ =  RUNNING;
		}
		else if (child_status_ == FAILURE)
		{
			current_running_child_ = NULL;
			return node_status_ = FAILURE;
		}
		exec_child_=exec_child_->get_next_brother();
	} while (exec_child_ != NULL);

	current_running_child_ = NULL;
	return node_status_ = SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* ------------------------------ROS Nodes----------------------------------- */
/* -------------------------------------------------------------------------- */

NodeROS::NodeROS(Node* node, std::string name)
	: Node(node),
	  ros_node_name_(name),
	  ac_(name, true)
{
	std::cout << "ROS Client: " << ros_node_name_ << std::endl;
	ROS_INFO("Waiting for the corresponding actuator server to start.");
	ac_.waitForServer();
}

void NodeROS::doneCb(const actionlib::SimpleClientGoalState& state,
                     const behavior_trees::ROSResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: %i", result->RESULT_);
	{
		boost::lock_guard<boost::mutex> lock(mutex_finished_);
		finished_ = true;
	}
	std::cout << "JeSUS christ, it is the END" << std::endl;
}

void NodeROS::activeCb()
{
	ROS_INFO("Goal just went active");
}

void NodeROS::feedbackCb(const behavior_trees::ROSFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback status: %i", feedback->FEEDBACK_);
	// std::cout << "%%% cb var: " << node_status_ << std::endl;
	boost::lock_guard<boost::mutex> lock(mutex_node_status_);
	node_status_ = (STATE) feedback->FEEDBACK_;
}

STATE NodeROS::execute()
{
	if (overwritten_)
	{
		return node_status_ = FAILURE; //overwritten_result_;
	}
	else
	{
		bool finished;
		{
			boost::lock_guard<boost::mutex> lock(mutex_finished_);
			finished = finished_;
		}
		if (!finished)
		{
			std::cout << "Sending Goal Client: "
			          << ros_node_name_ << std::endl;
			behavior_trees::ROSGoal goal;
			goal.GOAL_ = 1; // possitive tick

			ac_.sendGoal(goal,
			             boost::bind(&NodeROS::doneCb, this, _1, _2),
			             boost::bind(&NodeROS::activeCb, this),
			             boost::bind(&NodeROS::feedbackCb, this, _1));
		}
		{
			boost::lock_guard<boost::mutex> lock(mutex_node_status_);
			std::cout << "STATUS: " << node_status_ << std::endl;
			return node_status_;
		}
	}
}
