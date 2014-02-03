#ifndef ROSACTION_H_
#define ROSACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "behavior_trees/node.h"
#include "behavior_trees/parameters.h"

extern bool busy;

class ROSAction
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_trees::ROSAction> as_;
	std::string action_name_;

	bool started_;
	bool active_;

	int goal_;
	behavior_trees::ROSFeedback feedback_;
	behavior_trees::ROSResult result_;

	ros::Time start_time_;
	ros::Duration elapsed_time_;

	boost::thread execution_thread_;
	boost::mutex mutex_started_;
	boost::mutex mutex_active_;
	boost::mutex mutex_start_time_;
	boost::mutex mutex_elapsed_time_;
	boost::mutex mutex_feedback_;
	boost::mutex mutex_result_;

public:

	// construct action with: A name
	ROSAction(std::string name);

	// destroy action, i.e. default thread
	~ROSAction(void);

	// thread that calls executeCB
	void executionThread();

	// called each time a goal is received
	void goalCB();

	// called each time a goal is preempted
	void preemptCB();

	// start thread
	void start();

	// stop thread
	void stop();

	// activate executeCB
	void activate();

	// deactivate executeCB
	void deactivate();

	// get status of started
	bool is_started();

	// get status of active
	bool is_active();

	// check if no tick was received is TIMEOUT_THRESHOLD
	bool timeout_check();

	// reset because tick was received possibly at TICK_FREQUENCY
	void reset_timeout();

	// send partial feedback
	void send_feedback();

	// send final result
	void send_result();

	// sets the feedback to a certain value before sent in callback
	void set_feedback(STATE state);

	// sets the result to a certain value before sent in callback
	void set_result(STATE state);

	// function called periodically at EXECUTION_FREQUENCY
	virtual int executeCB(ros::Duration dt) { return 0; }

	// called when the thread is started only
	virtual void resetCB() {}
};

#endif
