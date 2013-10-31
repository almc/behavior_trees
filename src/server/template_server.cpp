#include "behavior_trees/rosaction.h"

// simple template to implement actions and conditions for the
// platform being used, the execute callback is going to be
// executed with frenquency: EXECUTION_FREQUENCY

// class definition overriding the functions executeCB and resetCB
class ActionName : ROSAction
{
public:
	ActionName(std::string name) :
		ROSAction(name),
		time_to_complete_((ros::Duration) 0) // used for this example
		{}

	// the action succeeds automatically after 5 seconds
	void executeCB(ros::Duration dt)
		{
			std::cout << "**ActionName -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;

			time_to_complete_ += dt;

			if (time_to_complete_.toSec() < 5)
			{
				feedback_.FEEDBACK_ = RUNNING;
			}
			else if (time_to_complete_.toSec() >= 5)
			{
				feedback_.FEEDBACK_ = FAILURE;
				result_.RESULT_ = FAILURE;
				as_.setSucceeded(result_);
				stop();
			}

			std::cout << "**ActionName -%- elapsed_time: "
			          << time_to_complete_.toSec() << std::endl;

			as_.publishFeedback(feedback_);
		}

	void resetCB()
		{
			time_to_complete_ = (ros::Duration) 0;
		}

	// member variables
	ros::Duration time_to_complete_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ActionName"); // name used for bt.txt
	ActionName server(ros::this_node::getName());
	ros::spin();
	return 0;
}
