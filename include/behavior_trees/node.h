#ifndef NODE_H_
#define NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_trees/ROSAction.h>

#include <GL/freeglut.h>
#include <GL/glu.h>


enum NODE_TYPE
{
	SELECTOR,
	SELECTOR_STAR,
	SEQUENCE,
	SEQUENCE_STAR,
	PARALLEL,
	DECORATOR,
	ACTION,
	CONDITION,
	ROOT
};

enum STATE
{
	FAILURE = 0,
	SUCCESS = 1,
	RUNNING = 2,
	NODE_ERROR = 3
};

class Node
{
protected:
	int depth_;
	int number_children_; // number of children that this node has e.g. (this node has N children)
	int children_number_; // number of this children among its brothers e.g. (index from 0 to N-1 children)
	bool highlighted_;
	bool overwritten_;
	STATE overwritten_result_;
	STATE node_status_;
	STATE child_status_;
	Node *first_child_;
	Node *curr_child_;
	Node *exec_child_;
	Node *next_brother_;
	Node *prev_brother_;
	Node *parent_;

public:

// constructor for the root node
	Node();

// constructor for any node but root
	Node(Node* parent);

// set node status to default and its children too (recursive)
	void execute_reset_status();

// virtual functions
	virtual STATE execute() =0;

	virtual NODE_TYPE get_node_type() =0;

	virtual std::string get_node_name() =0;

	virtual void set_ros_node_name(std::string name) {}

	virtual std::string get_ros_node_name() { return "error"; }

// add child distinguishing whether it is the first one or not
	Node* add_child(Node* my_child);

// if it is not the first child, it will be treated as a brother instead
	Node* add_brother(Node* my_brother, int children_number);

// prints the depth, status, and number of children of a certain node
	void print_info();

// prints the information of the whole tree recursively calling itself
	void print_subtree();

// draws the tree in OpenGL trying to spread the nodes over the screen
	void draw_subtree(GLfloat parent_x, GLfloat parent_y, int number_children,
	                  GLfloat parent_space);

// inlined functions
	inline int get_depth() { return depth_; }

	inline int get_number_children() { return number_children_; }

	inline int get_children_number() { return children_number_; }

	inline Node* get_parent() { return parent_; }

	inline Node* get_first_child() { return first_child_; }

	inline Node* get_next_brother() { return next_brother_; }

	inline Node* get_prev_brother() { return prev_brother_; }

	inline void set_children_number(int number) { children_number_ = number; }

	inline void set_next_brother(Node* pointer) { next_brother_ = pointer; }

	inline void set_prev_brother(Node* pointer) { prev_brother_ = pointer; }

	inline void set_highlighted(bool boolean) { highlighted_ = boolean; }

	inline void set_overwrite(STATE state, bool overwritten)
		{ overwritten_result_ = state; overwritten_ = overwritten; }
};

/* -------------------------------------------------------------------------- */
/* ----------------------------Basic Nodes----------------------------------- */
/* -------------------------------------------------------------------------- */

class NodeSelector : public Node
{
public:
	NodeSelector(Node* node);
private:
	STATE execute();
	inline NODE_TYPE get_node_type() { return SELECTOR; }
	inline std::string get_node_name() { return "Selector"; }
};

class NodeSequence : public Node
{
public:
	NodeSequence(Node* node);
private:
	STATE execute();
	inline NODE_TYPE get_node_type() { return SEQUENCE; }
	inline std::string get_node_name() { return "Sequence"; }
};

class NodeParallel : public Node
{
public:
	NodeParallel(Node* node);
private:
	STATE execute();
	inline NODE_TYPE get_node_type() { return PARALLEL; }
	inline std::string get_node_name() { return "Parallel"; }
};

class NodeRoot : public Node
{
public:
	STATE execute();
private:
	inline NODE_TYPE get_node_type() { return ROOT; }
	inline std::string get_node_name() { return "Root"; }
};

/* -------------------------------------------------------------------------- */
/* ----------------------------Star Nodes------------------------------------ */
/* -------------------------------------------------------------------------- */

class NodeSelectorStar : public Node
{
public:
	NodeSelectorStar(Node* node);
private:
	STATE execute();
	inline NODE_TYPE get_node_type() { return SELECTOR_STAR; }
	inline std::string get_node_name() { return "Selector Star"; }
	Node *current_running_child_;
};

class NodeSequenceStar : public Node
{
public:
	NodeSequenceStar(Node* node);
private:
	STATE execute();
	inline NODE_TYPE get_node_type() { return SEQUENCE_STAR; }
	inline std::string get_node_name() { return "Sequence Star"; }
	Node *current_running_child_;
};

/* -------------------------------------------------------------------------- */
/* ------------------------------ROS Nodes----------------------------------- */
/* -------------------------------------------------------------------------- */
// typedef actionlib::SimpleActionClient<behavior_trees::ROSAction> Client;

class NodeROS : public Node
{
public:
	NodeROS(Node* node, std::string name);

// called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
	            const behavior_trees::ROSResultConstPtr& result);
// called once when the goal becomes active
	void activeCb();
// called every time feedback is received for the goal
	void feedbackCb(const behavior_trees::ROSFeedbackConstPtr& feedback);

private:
	inline NODE_TYPE get_node_type() { return ACTION; }
	inline std::string get_node_name() { return "Action"; }
	inline void set_ros_node_name(std::string name) { ros_node_name_ = name; }
	inline std::string get_ros_node_name() { return ros_node_name_; }
	STATE execute();

	std::string ros_node_name_;
	bool finished_;
	bool received_;
	actionlib::SimpleActionClient<behavior_trees::ROSAction> ac_;
	boost::mutex mutex_node_status_;
	boost::mutex mutex_finished_;
};

class NodeCondition : public Node
{
public:
	NodeCondition(Node* node, std::string varlabel,
	              std::string relation, std::string constant);

private:
	inline NODE_TYPE get_node_type() { return CONDITION; }
	inline std::string get_node_name() { return "Condition"; }
	inline void set_condition_name(std::string name) { condition_name_ = name; }
	inline std::string get_condition_name() { return condition_name_; }
	STATE execute();

	std::string condition_name_;
	std::string varlabel_;
	std::string relation_;
	std::string constant_;
};

class NodeDecorator : public Node
{
public:
	NodeDecorator(Node* node, std::string next_state, std::string curr_state,
	              std::string prev_status);

private:
	inline NODE_TYPE get_node_type() { return DECORATOR; }
	inline std::string get_node_name() { return "Decorator"; }
	inline void set_decorator_name(std::string name) { decorator_name_ = name; }
	inline std::string get_decorator_name() { return decorator_name_; }
	STATE execute();

	std::string decorator_name_;
	std::string next_state_;
	std::string curr_state_;
	std::string prev_status_;
};

#endif

// actionlib::SimpleClientGoalState state = ac_.getState();
// ROS_INFO("Action finished: %s",state.toString().c_str());

// goal.GOAL_ = 1; // possitive tick
// goal.GOAL_ = 0; // neutral tick
// goal.GOAL_ =-1; // negative tick

// std::cout << "node executing: " << this << std::endl;
// std::cout << "*** ex var: " << node_status_ << std::endl;
