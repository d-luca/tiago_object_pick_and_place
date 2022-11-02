#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <intro_navigation/NavigationAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <intro_navigation/ObstaclePositionScan.h>

class NavigationAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<intro_navigation::NavigationAction> as_;
    ros::Subscriber scan_sub;
    std::string action_name_;
    intro_navigation::NavigationFeedback feedback_;
    intro_navigation::NavigationResult result_;
public:
    NavigationAction(std::string name) : as_(nh_, name, boost::bind(&NavigationAction::executeCB, this, _1), false), action_name_(name) {
        as_.start();
    }

    ~NavigationAction(void) {}

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    bool callMoveBaseServer(const intro_navigation::NavigationGoalConstPtr &goal) {

        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Call to move_base server: Waiting for the move_base action server to come up");
            feedback_.f = "Call to move_base server: Waiting for the move_base action server to come up";
            as_.publishFeedback(feedback_);
        }

        move_base_msgs::MoveBaseGoal g;

        g.target_pose.header.frame_id = "map";
        g.target_pose.header.stamp = ros::Time::now();

        g.target_pose.pose.position.x = goal->xp;
        g.target_pose.pose.position.y = goal->yp;
        g.target_pose.pose.orientation.z = goal->wz;
        g.target_pose.pose.orientation.w = goal->ww;

        ROS_INFO("Call to move_base server: Sending goal to the move_base server");
        feedback_.f = "Call to move_base server: Sending goal to the move_base server";
        as_.publishFeedback(feedback_);
        ac.sendGoal(g);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Call to move_base server: Goal reached");
            feedback_.f = "Call to move_base server: Goal reached";
            as_.publishFeedback(feedback_);
            return true;
        }
        else{
            //this else can be deleted because the if up here will return
            ROS_INFO("Call to move_base server: Failed to reach the goal for some reason");
            feedback_.f = "Call to move_base server: Failed to reach the goal for some reason";
            as_.publishFeedback(feedback_);
            return false;
        }
    }//callMoveBaseServer(const intro_navigation::NavigationGoalConstPtr &goal)

    //our_server callback, will contains the call to the move_base action server
    // and the scan part
    void executeCB(const intro_navigation::NavigationGoalConstPtr &goal) {
        ros::Rate r(1);
        bool success = true;

        feedback_.f = "Call to move_base server: Callback started";
        as_.publishFeedback(feedback_);

        //call to the move_base server as client
        success = callMoveBaseServer(goal);

        if (success) {
            //to use at the end when we will merge the scan part
            ROS_INFO("Call to move_base server: Callback ended well, starting retrieving data from scan topic");
            feedback_.f = "Call to move_base server: Callback ended well, starting retrieving data from scan topic";
            as_.publishFeedback(feedback_);

            //use a shared pointer to retrieve data from the topic and save those data in a local one (not shared)
            boost::shared_ptr<intro_navigation::ObstaclePositionScan const> shared_scan_msg;
            intro_navigation::ObstaclePositionScan scan_msg;
            //work on new vectors so that we will return only the point of this callback
            //even if the server has done more than one
            std::vector<geometry_msgs::Point32> points_v;
            std::vector<float> radius_v;

            shared_scan_msg = ros::topic::waitForMessage<intro_navigation::ObstaclePositionScan>("/intro_navigation/obstacles_centers");
            if(shared_scan_msg != NULL) {
                scan_msg = *shared_scan_msg;
                ROS_INFO("Retrieving data from scan topic: data retrieved successfully");
                feedback_.f = "Retrieving data from scan topic: data retrieved successfully";
                as_.publishFeedback(feedback_);

                for (int i = 0; i < scan_msg.x.size(); ++i) {
                    geometry_msgs::Point32 temp;
                    temp.x = scan_msg.x[i];
                    temp.y = scan_msg.y[i];
                    points_v.push_back(temp);
                    radius_v.push_back(scan_msg.r[i]);
                }

                result_.points = points_v;
                result_.radius = radius_v;

                ROS_INFO("Callback finished");
                as_.setSucceeded(result_);
            }
            else {
                ROS_INFO("Retrieving data from scan topic: error while retrieving data");
                feedback_.f = "Retrieving data from scan topic: error while retrieving data";
                as_.publishFeedback(feedback_);
                result_.points = points_v;
                result_.radius = radius_v;
                as_.setAborted(result_);
            }
        }
    } //void executeCB(const intro_navigation::NavigationGoalConstPtr &goal)
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "our_server");
    NavigationAction navigation("our_server");
    ros::spin();
    return 0;
}

