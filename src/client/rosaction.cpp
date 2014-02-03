#include "behavior_trees/rosaction.h"		// contains ROSAction class to implement servers
#include <behavior_trees/ROSAction.h>		// automatically generated actionlib header
#include <boost/thread.hpp>

bool busy = false;
// construct action with: A name
ROSAction::ROSAction(std::string name) :
	as_(nh_, name, false),
	action_name_(name),
	started_(false),
	active_(false),
	start_time_(ros::Time::now()),
	elapsed_time_((ros::Duration) 0)
{
	feedback_.FEEDBACK_ = NODE_ERROR;
	result_.RESULT_     = NODE_ERROR;
	as_.registerGoalCallback(boost::bind(&ROSAction::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&ROSAction::preemptCB, this));
	as_.start();
}

// destroy action, i.e. default thread
ROSAction::~ROSAction(void)
{}

// thread that calls executeCB
void ROSAction::executionThread()
{
	ros::Rate r(EXECUTION_FREQUENCY);
	ros::Time t0 = start_time_ = ros::Time::now();

	// while (as_.isPreemptRequested() || ros::ok())
	while ( is_active() && ros::ok())
	{
		std::cout << "execution_thread_.get_id()" << execution_thread_.get_id() << std::endl;

		bool active = timeout_check(); // check if tick was received
		std::cout << "im active" << active << std::endl;
		{
			boost::lock_guard<boost::mutex> lock(mutex_active_);
			active_ = active = active ? active_ : false;
		}
		if (active)
		{
			ros::Duration dt = ros::Time::now() - t0;
			t0 = ros::Time::now();
			std::cout << "executing cb" << dt << std::endl;
			if (executeCB(dt))	// execute user personal code
				break;			// if finished exit fast
		}
		else
		{
			std::cout << "executionThread not active" << std::endl;
		}
		r.sleep();		     // wait to match frequency
	}
	std::cout << "About to Destroy Thread" << std::endl;
	// set_feedback(NODE_ERROR);
	// send_feedback();
	std::cout << "----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
	stop();
	// execution_thread_.join();
}

// called each time a goal is received
void ROSAction::goalCB()
{
	std::cout << "****************************%%%%%%%%%% goalCB %%%%%%%%%%" << std::endl;
	// could add conditions to accept goal or not
	goal_ = as_.acceptNewGoal()->GOAL_;
	std::cout << "Received Goal: " << goal_ << std::endl;

	send_feedback();

	// if (!busy)
	// {
	// 	feedback_.FEEDBACK_ = NODE_ERROR;
	// 	result_.RESULT_     = NODE_ERROR;
	// }

	if (feedback_.FEEDBACK_ != SUCCESS &&
	    feedback_.FEEDBACK_ != FAILURE)
	{
		bool started;		// is thread running?
		{
			boost::lock_guard<boost::mutex> lock(mutex_started_);
			started = started_;
		}
		std::cout << "started: " << started << std::endl;
		if (started)
		{
			if (goal_ > 0)	    // possitive tick
				reset_timeout();
			else if (goal_ < 0) // negative tick
				stop();
			else			    // neutral tick
			{}
		}
		else
		{
			if (goal_ > 0)	    // possitive tick
				start();
			else if (goal_ < 0) // negative tick
			{}
			else			    // neutral tick
			{}
		}
	}
	if (feedback_.FEEDBACK_ == SUCCESS ||
	    feedback_.FEEDBACK_ == FAILURE)
	{
		feedback_.FEEDBACK_ = NODE_ERROR;
		result_.RESULT_     = NODE_ERROR;
	}
}

// called each time a goal is preempted
void ROSAction::preemptCB()
{
	std::cout << "%%%%%%%%%% preemptCB %%%%%%%%%%" << std::endl;
	reset_timeout();
	// as_.setPreempted();
}

// start thread
void ROSAction::start()
{
	std::cout << "Executing Reset Callback Now" << std::endl;
	resetCB();
	{
		boost::lock_guard<boost::mutex> lock(mutex_started_);
		started_ = true;
	}
	{
		boost::lock_guard<boost::mutex> lock(mutex_active_);
		active_ = true;
	}
	std::cout << "Starting Thread Now" << std::endl;
	std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

	execution_thread_ = boost::thread(
		boost::bind(&ROSAction::executionThread, this) );
}

// stop thread
void ROSAction::stop()
{
	std::cout << "Stopping Thread Now" << std::endl;
	{
		boost::lock_guard<boost::mutex> lock(mutex_started_);
		started_ = false;
	}
	{
		boost::lock_guard<boost::mutex> lock(mutex_active_);
		active_ = false;
	}
	// execution_thread_.join();
}

// activate executeCB
void ROSAction::activate()
{
	std::cout << "Activating Thread Now" << std::endl;
	boost::lock_guard<boost::mutex> lock(mutex_active_);
	active_ = true;
}

// deactivate executeCB
void ROSAction::deactivate()
{
	std::cout << "Deactivating Thread Now" << std::endl;
	boost::lock_guard<boost::mutex> lock(mutex_active_);
	active_ = false;
}

// get status of started
bool ROSAction::is_started()
{
	boost::lock_guard<boost::mutex> lock(mutex_started_);
	return started_;
}

// get status of active
bool ROSAction::is_active()
{
	boost::lock_guard<boost::mutex> lock(mutex_active_);
	return active_;
}

// check if no tick was received is TIMEOUT_THRESHOLD
bool ROSAction::timeout_check()		// called from executionThread
{
	ros::Duration dt = (ros::Duration) 0;
	{
		boost::lock_guard<boost::mutex> lock_a(mutex_start_time_);
		boost::lock_guard<boost::mutex> lock_b(mutex_elapsed_time_);
		dt = elapsed_time_ = ros::Time::now() - start_time_;
	}
	std::cout << "elapsed_time since tick: " << dt.toSec() << std::endl;
	return dt.toSec() > TIMEOUT_THRESHOLD ? false : true;
}

// reset because tick was received possibly at TICK_FREQUENCY
void ROSAction::reset_timeout()		// called from goalCB
{
	boost::lock_guard<boost::mutex> lock(mutex_start_time_);
	start_time_ = ros::Time::now();
}

// send partial feedback
void ROSAction::send_feedback()
{
	std::cout << "Sending Feedback now" << std::endl;
	boost::lock_guard<boost::mutex> lock(mutex_feedback_);
	// feedback_.FEEDBACK_ = state;
	as_.publishFeedback(feedback_);
}

// send final result
void ROSAction::send_result()
{
	std::cout << "Sending Result now" << std::endl;
	boost::lock_guard<boost::mutex> lock(mutex_result_);
	// result_.RESULT_ = state;
	as_.setSucceeded(result_);
}

// sets the feedback to a certain value before sent in callback
void ROSAction::set_feedback(STATE state)
{
	boost::lock_guard<boost::mutex> lock(mutex_feedback_);
	feedback_.FEEDBACK_ = state;
}

// sets the result to a certain value before sent in callback
void ROSAction::set_result(STATE state)
{
	boost::lock_guard<boost::mutex> lock(mutex_result_);
	result_.RESULT_ = state;
}
