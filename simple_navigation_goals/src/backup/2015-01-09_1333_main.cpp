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

  /*  geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 22.0;
    waypoint1.position.y = 10.75;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = 0;
    waypoint1.orientation.w = 1;
    waypoints.push_back(waypoint1);
  */
    // /////////////////////////////////////////////Parallel Parking////////////////////////////
    /*
    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 9.1;
    waypoint1.position.y = 18.1;
    waypoint1.position.z = 0.18;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.78;
    waypoint1.orientation.w = 0.6;
    waypoints.push_back(waypoint1);


    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 9.1;
    waypoint2.position.y = 17.9;
    waypoint2.position.z = 0.18;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = -0.78;
    waypoint2.orientation.w = 0.6;
    waypoints.push_back(waypoint2);


    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 9.1;
    waypoint3.position.y = 17.7;
    waypoint3.position.z = 0.18;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = -0.78;
    waypoint3.orientation.w = 0.6;
    waypoints.push_back(waypoint3);


    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 9.1;
    waypoint4.position.y = 17.5;
    waypoint4.position.z = 0.18;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = -0.78;
    waypoint4.orientation.w = 0.6;
    waypoints.push_back(waypoint4);


    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 9.1;
    waypoint5.position.y = 17.3;
    waypoint5.position.z = 0.18;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = -0.78;
    waypoint5.orientation.w = 0.6;
    waypoints.push_back(waypoint5);


    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 9.1;
    waypoint6.position.y = 17.1;
    waypoint6.position.z = 0.18;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = -0.78;
    waypoint6.orientation.w = 0.6;
    waypoints.push_back(waypoint6);


    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = 9.1;
    waypoint7.position.y = 16.9;
    waypoint7.position.z = 0.18;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = -0.78;
    waypoint7.orientation.w = 0.6;
    waypoints.push_back(waypoint7);


    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = 9.1;
    waypoint8.position.y = 16.7;
    waypoint8.position.z = 0.18;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = -0.78;
    waypoint8.orientation.w = 0.6;
    waypoints.push_back(waypoint8);


    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = 9.1;
    waypoint9.position.y = 16.5;
    waypoint9.position.z = 0.18;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = -0.78;
    waypoint9.orientation.w = 0.6;
    waypoints.push_back(waypoint9);


    geometry_msgs::Pose waypoint10;
    waypoint10.position.x = 8.7;
    waypoint10.position.y = 16.3;
    waypoint10.position.z = 0.18;
    waypoint10.orientation.x = 0.000;
    waypoint10.orientation.y = 0.000;
    waypoint10.orientation.z = -0.78;
    waypoint10.orientation.w = 0.6;
    waypoints.push_back(waypoint10);


    geometry_msgs::Pose waypoint11;
    waypoint11.position.x = 9.1;
    waypoint11.position.y = 16.1;
    waypoint11.position.z = 0.18;
    waypoint11.orientation.x = 0.000;
    waypoint11.orientation.y = 0.000;
    waypoint11.orientation.z = -0.78;
    waypoint11.orientation.w = 0.6;
    waypoints.push_back(waypoint11);


    geometry_msgs::Pose waypoint12;
    waypoint12.position.x = 9.1;
    waypoint12.position.y = 15.9;
    waypoint12.position.z = 0.18;
    waypoint12.orientation.x = 0.000;
    waypoint12.orientation.y = 0.000;
    waypoint12.orientation.z = -0.78;
    waypoint12.orientation.w = 0.6;
    waypoints.push_back(waypoint12);


    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = 9.1;
    waypoint13.position.y = 15.8;
    waypoint13.position.z = 0.18;
    waypoint13.orientation.x = 0.000;
    waypoint13.orientation.y = 0.000;
    waypoint13.orientation.z = -0.78;
    waypoint13.orientation.w = 0.6;
    waypoints.push_back(waypoint13);


    geometry_msgs::Pose waypoint14;
    waypoint14.position.x = 9.1;
    waypoint14.position.y = 17.8;
    waypoint14.position.z = 0.18;
    waypoint14.orientation.x = 0.000;
    waypoint14.orientation.y = 0.000;
    waypoint14.orientation.z = 0.78;
    waypoint14.orientation.w = 0.6;
    waypoints.push_back(waypoint14);

    MoveBaseClient ac("move_base", true); // action client to spin a thread by default
    */
    // ///////////////////////////////////Parallel Parking///////////////////////////////
    
    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 21.12;
    waypoint1.position.y = 18.39;
    waypoint1.position.z = 0.18;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.78;
    waypoint1.orientation.w = 0.6;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 21.33;
    waypoint2.position.y = 17.58;
    waypoint2.position.z = 0.18;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = -0.78;
    waypoint2.orientation.w = 0.6;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 21.55;
    waypoint3.position.y = 17.28;
    waypoint3.position.z = 0.18;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = -0.78;
    waypoint3.orientation.w = 0.6;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 21.67;
    waypoint4.position.y = 16.62;
    waypoint4.position.z = 0.18;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = -0.78;
    waypoint4.orientation.w = 0.6;
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 21.55;
    waypoint5.position.y = 16.16;
    waypoint5.position.z = 0.18;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = -0.78;
    waypoint5.orientation.w = 0.6;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 21.25;
    waypoint6.position.y = 15.88;
    waypoint6.position.z = 0.18;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = -0.78;
    waypoint6.orientation.w = 0.6;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = 20.75;
    waypoint7.position.y = 15.65;
    waypoint7.position.z = 0.18;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = 0.96;
    waypoint7.orientation.w = -0.27;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = 20.44;                       // 20.44
    waypoint8.position.y = 15.23;
    waypoint8.position.z = 0.18;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = 0.91;
    waypoint8.orientation.w = -0.39;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = 20.38;                       // 20.38
    waypoint9.position.y = 14.80;
    waypoint9.position.z = 0.18;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = -0.82;
    waypoint9.orientation.w = 0.56;
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint10;
    waypoint10.position.x = 20.55;                      // 20.55
    waypoint10.position.y = 14.39;
    waypoint10.position.z = 0.18;
    waypoint10.orientation.x = 0.000;
    waypoint10.orientation.y = 0.000;
    waypoint10.orientation.z = -0.66;
    waypoint10.orientation.w = 0.74;
    waypoints.push_back(waypoint10);

    geometry_msgs::Pose waypoint11;
    waypoint11.position.x = 20.89;
    waypoint11.position.y = 14.05;
    waypoint11.position.z = 0.18;
    waypoint11.orientation.x = 0.000;
    waypoint11.orientation.y = 0.000;
    waypoint11.orientation.z = -0.49;
    waypoint11.orientation.w = 0.87;
    waypoints.push_back(waypoint11);

    geometry_msgs::Pose waypoint12;
    waypoint12.position.x = 21.29;
    waypoint12.position.y = 13.75;
    waypoint12.position.z = 0.18;
    waypoint12.orientation.x = 0.000;
    waypoint12.orientation.y = 0.000;
    waypoint12.orientation.z = -0.40;
    waypoint12.orientation.w = 0.91;
    waypoints.push_back(waypoint12);

    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = 21.60;
    waypoint13.position.y = 13.17;
    waypoint13.position.z = 0.18;
    waypoint13.orientation.x = 0.000;
    waypoint13.orientation.y = 0.000;
    waypoint13.orientation.z = -0.55;
    waypoint13.orientation.w = 0.83;
    waypoints.push_back(waypoint13);

    geometry_msgs::Pose waypoint14;
    waypoint14.position.x = 21.41;
    waypoint14.position.y = 12.65;
    waypoint14.position.z = 0.18;
    waypoint14.orientation.x = 0.000;
    waypoint14.orientation.y = 0.000;
    waypoint14.orientation.z = -0.80;
    waypoint14.orientation.w = 0.59;
    waypoints.push_back(waypoint14);

    geometry_msgs::Pose waypoint15;
    waypoint15.position.x = 20.93;
    waypoint15.position.y = 12.45;
    waypoint15.position.z = 0.18;
    waypoint15.orientation.x = 0.000;
    waypoint15.orientation.y = 0.000;
    waypoint15.orientation.z = 0.88;
    waypoint15.orientation.w = -0.46;
    waypoints.push_back(waypoint15);

    geometry_msgs::Pose waypoint16;
    waypoint16.position.x = 20.41;
    waypoint16.position.y = 12.17;
    waypoint16.position.z = 0.18;
    waypoint16.orientation.x = 0.000;
    waypoint16.orientation.y = 0.000;
    waypoint16.orientation.z = 0.95;
    waypoint16.orientation.w = -0.28;
    waypoints.push_back(waypoint16);

    geometry_msgs::Pose waypoint17;
    waypoint17.position.x = 20.10;
    waypoint17.position.y = 11.71;
    waypoint17.position.z = 0.18;
    waypoint17.orientation.x = 0.000;
    waypoint17.orientation.y = 0.000;
    waypoint17.orientation.z = 0.89;
    waypoint17.orientation.w = -0.46;
    waypoints.push_back(waypoint17);

    geometry_msgs::Pose waypoint18;
    waypoint18.position.x = 20.10;
    waypoint18.position.y = 11.18;
    waypoint18.position.z = 0.18;
    waypoint18.orientation.x = 0.000;
    waypoint18.orientation.y = 0.000;
    waypoint18.orientation.z = -0.76;
    waypoint18.orientation.w = 0.64;
    waypoints.push_back(waypoint18);

    geometry_msgs::Pose waypoint19;
    waypoint19.position.x = 20.20;
    waypoint19.position.y = 10.60;
    waypoint19.position.z = 0.18;
    waypoint19.orientation.x = 0.000;
    waypoint19.orientation.y = 0.000;
    waypoint19.orientation.z = -0.72;
    waypoint19.orientation.w = 0.68;
    waypoints.push_back(waypoint19);



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
