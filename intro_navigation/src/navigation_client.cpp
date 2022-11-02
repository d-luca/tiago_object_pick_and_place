#include <ros/ros.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <intro_navigation/NavigationAction.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_client");
    if(argc != 5) {
        ROS_INFO("usage: navigation_client xp yp wz");
        return 1;
    }
    else {
        // create the action client
        // true causes the client to spin its own thread
        actionlib::SimpleActionClient<intro_navigation::NavigationAction> ac("our_server", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        intro_navigation::NavigationGoal goal;

        goal.xp = std::stof(argv[1]);
        goal.yp = std::stof(argv[2]);
        goal.wz = std::stof(argv[3]);
        goal.ww = std::stof(argv[4]);

        // send a goal to the action
        ac.sendGoal(goal);

        boost::shared_ptr<intro_navigation::NavigationActionFeedback const> shared_nav_feedback_msg;
        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ac.getState() != actionlib::SimpleClientGoalState::ABORTED) {
            shared_nav_feedback_msg = ros::topic::waitForMessage<intro_navigation::NavigationActionFeedback>("/our_server/feedback", ros::Duration (1));
            if(shared_nav_feedback_msg != NULL) {
                ROS_INFO("[F]: %s", shared_nav_feedback_msg->feedback.f.c_str());
            }
        }

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached");
            intro_navigation::NavigationResultConstPtr my_result = ac.getResult();

            ROS_INFO("---Detected obstacles coordinates---");
            for (int i = 0; i < my_result->radius.size(); ++i) {
                ROS_INFO("[%d] x: %f    y: %f", i+1, my_result->points[i].x, my_result->points[i].y);
            }
            ROS_INFO("------------------------------------");
        }
        else{
            ROS_INFO("Failed to reach the goal for some reason");
            return 1;
        }
    }
    //exit
    return 0;
}

