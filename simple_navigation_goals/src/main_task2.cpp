 /**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

  
	// /////////////////////////////////////// Start of waypoint setting ///////////////////////////////////
    // /////////////////////////////////////////// TASK 1 - RUNDKURS /////////////////////////////////////
	// Set waypoints for the fist task.
/*
    geometry_msgs::Pose waypoint1;              //first goal straight on the hallway
    waypoint1.position.x = 10.23;
    waypoint1.position.y = 10.22;
    waypoint1.position.z = 0;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.7;
    waypoint1.orientation.w = 0.71;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;              //after the first corner
    waypoint2.position.x = 11.29;
    waypoint2.position.y = 6.42;
    waypoint2.position.z = 0;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = 0.0;
    waypoint2.orientation.w = 0.99;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;              //corner before small door
    waypoint3.position.x = 21.7;
    waypoint3.position.y = 5.6;
    waypoint3.position.z = 0;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = 0.23;
    waypoint3.orientation.w = 0.96;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint5;              //in the small door
    waypoint5.position.x = 23.15;
    waypoint5.position.y = 7.3;
    waypoint5.position.z = 0.0;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = 0.71;
    waypoint5.orientation.w = 0.69;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;              //before the third corner
    waypoint6.position.x = 23.89;
    waypoint6.position.y = 17.9;
    waypoint6.position.z = 0.0;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = 0.79;
    waypoint6.orientation.w = 0.61;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;              //after the third corner
    waypoint7.position.x = 22.0;
    waypoint7.position.y = 19.3;
    waypoint7.position.z = 0.0;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = 0.99;
    waypoint7.orientation.w = 0.00;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;              //middle of the hallway
    waypoint8.position.x = 17.6;
    waypoint8.position.y = 19.3;
    waypoint8.position.z = 0.0;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = 0.99;
    waypoint8.orientation.w = 0.00;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;              //before fourth corner
    waypoint9.position.x = 13.47;
    waypoint9.position.y = 19.87;
    waypoint9.position.z = 0.0;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = 0.99;
    waypoint9.orientation.w = -0.07;
    waypoints.push_back(waypoint9);


    geometry_msgs::Pose waypoint10;              //end position after the fourth corner
    waypoint10.position.x = 10.8;
    waypoint10.position.y = 17.7;
    waypoint10.position.z = 0.0;
    waypoint10.orientation.x = 0.000;
    waypoint10.orientation.y = 0.000;
    waypoint10.orientation.z = -0.7;
    waypoint10.orientation.w = 0.7;
    waypoints.push_back(waypoint10);
	*/
	
	// ///////////////////////////////////////TASK 2 - SLALOM/////////////////////////////////////
	// Setting 14 waypoints for the second task.
	
    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 23.90;
    waypoint1.position.y = 17.8;
    waypoint1.position.z = 0;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.54;
    waypoint1.orientation.w = 0.83;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 24.16;
    waypoint2.position.y = 17.09;
    waypoint2.position.z = 0;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = -0.67;
    waypoint2.orientation.w = 0.73;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 24.08;
    waypoint3.position.y = 16.36;
    waypoint3.position.z = 0;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = 0.91;
    waypoint3.orientation.w = -0.43;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 23.66;
    waypoint4.position.y = 16.00;
    waypoint4.position.z = 0;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = 0.97;
    waypoint4.orientation.w = -0.23;
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 23.13;
    waypoint5.position.y = 15.79;
    waypoint5.position.z = 0;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = -0.81;
    waypoint5.orientation.w = 0.57;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 22.93;
    waypoint6.position.y = 15.4;
    waypoint6.position.z = 0;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = -0.66;
    waypoint6.orientation.w = 0.74;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = 23.04;
    waypoint7.position.y = 14.85;
    waypoint7.position.z = 0;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = -0.53;
    waypoint7.orientation.w = 0.84;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = 23.24;
    waypoint8.position.y = 14.50;
    waypoint8.position.z = 0;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = -0.39;
    waypoint8.orientation.w = 0.91;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = 23.73;
    waypoint9.position.y = 14.07;
    waypoint9.position.z = 0;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = -0.58;
    waypoint9.orientation.w = 0.80;
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint10;
    waypoint10.position.x = 23.86;
    waypoint10.position.y = 13.73;
    waypoint10.position.z = 0;
    waypoint10.orientation.x = 0.000;
    waypoint10.orientation.y = 0.000;
    waypoint10.orientation.z = -0.74;
    waypoint10.orientation.w = 0.67;
    waypoints.push_back(waypoint10);

    geometry_msgs::Pose waypoint11;
    waypoint11.position.x = 23.81;
    waypoint11.position.y = 13.22;
    waypoint11.position.z = 0;
    waypoint11.orientation.x = 0.000;
    waypoint11.orientation.y = 0.000;
    waypoint11.orientation.z = 0.90;
    waypoint11.orientation.w = -0.43;
    waypoints.push_back(waypoint11);

    geometry_msgs::Pose waypoint12;
    waypoint12.position.x = 23.47;
    waypoint12.position.y = 12.86;
    waypoint12.position.z = 0;
    waypoint12.orientation.x = 0.000;
    waypoint12.orientation.y = 0.000;
    waypoint12.orientation.z = 0.95;
    waypoint12.orientation.w = -0.28;
    waypoints.push_back(waypoint12);

    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = 22.89;
    waypoint13.position.y = 12.52;
    waypoint13.position.z = 0;
    waypoint13.orientation.x = 0.000;
    waypoint13.orientation.y = 0.000;
    waypoint13.orientation.z = -0.84;
    waypoint13.orientation.w = 0.53;
    waypoints.push_back(waypoint13);

    geometry_msgs::Pose waypoint14;
    waypoint14.position.x = 22.45;
    waypoint14.position.y = 10.78;
    waypoint14.position.z = 0;
    waypoint14.orientation.x = 0.000;
    waypoint14.orientation.y = 0.000;
    waypoint14.orientation.z = -0.74;
    waypoint14.orientation.w = 0.67;
    waypoints.push_back(waypoint14);
	
	
	// ///////////////////////////////////////End of waypoint setting/////////////////////////////////////
	

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
