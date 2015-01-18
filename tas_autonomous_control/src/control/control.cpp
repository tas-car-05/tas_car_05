#include "control.h"
#include <fstream>

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

	// Subscribe to the slam_out_pose topic with the master. ROS will call the positionCallback() function whenever a new message arrives. 
	// The 2nd argument is the queue size, in case we are not able to process messages fast enough
    position_sub = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 1000, &control::positionCallback, this);
}

// We can subscribe to the odom here and get some feedback signals so later we can build our controllers

/**************************************************************************************************************
*	function positionCallback
*
*		programmer:
*			- Christoph Allig		christoph.allig@tum.de
*			- Marcin Kasperek		marcin.kasperek@tum.de
*	
*		functionality:
*			- subscribes to the topic slam_out_pose (robots pose without covariance)
*			- definition of areas for each curve
*			  => detect whether the car is driving in a curve or on a straight track 
*		
*		last modified: 16.01.2015
**************************************************************************************************************/
void control::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// save the relevant data from the subscribed topic
    est_position_x = msg->pose.position.x;
    est_position_y = msg->pose.position.y;

	// for Debugging
    // ROS_INFO("est_position_x: [%f]", est_position_x);
	
            // curve on the left upper side
            if(9.5 < est_position_x && est_position_x < 14 && -1.5 < est_position_y && 1.5 < est_position_y)
            { 
				// ROS_INFO for debugging
				// ROS_INFO("curve on the left upper side");
				// ROS_INFO("est_position_x: [%f]", est_position_x);
				// ROS_INFO("est_position_y: [%f]", est_position_y);
				Kurve = 1;
            }

            // curve on the right upper side
            else if(10.5 < est_position_x && est_position_x < 14 && -14 < est_position_y && est_position_y< -11)
            {
                // ROS_INFO for debugging
                // ROS_INFO("curve on the right upper side");
                // ROS_INFO("est_position_x: [%f]", est_position_x);
                // ROS_INFO("est_position_y: [%f]", est_position_y);
                Kurve = 1;
            }

            // curve on the right lower side
            else if(-2 < est_position_x && est_position_x< 1.5 && -14 < est_position_y && est_position_y < -10.5)
            {
                // ROS_INFO for debugging
                // ROS_INFO("curve on the right lower side");
                // ROS_INFO("est_position_x: [%f]", est_position_x);
                // ROS_INFO("est_position_y: [%f]", est_position_y);
                Kurve = 1;
            }

            // curve on the left upper side
            else if((6 < est_position_x < 10) && (4.5 < est_position_y < 8.5))
            {
                //ROS_INFO for debugging
				//ROS_INFO("curve on the left upper side");
				//ROS_INFO("est_position_x: [%f]", est_position_x);
				//ROS_INFO("est_position_y: [%f]", est_position_y);
                Kurve = 1;
            }

            // straight
            else
            {
				// ROS_INFO for debugging
                // ROS_INFO("est_position_x_Gerade: [%f]");
                // ROS_INFO("est_position_x: [%f]", est_position_x);
                // ROS_INFO("est_position_y: [%f]", est_position_y);
                Kurve = 0;
            }
}

void control::odomCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    odom_linearVelocity = msg->linear.x;
    odom_angularVelocity = msg->angular.z;

    odom_steeringAngle = 180/PI*atan(odom_angularVelocity/odom_linearVelocity*CAR_LENGTH);

    odom_steeringAngle = 1500 + 500/30*odom_steeringAngle;

    if(odom_steeringAngle > 2000)
    {
        odom_steeringAngle = 2000;
    }
    else if(odom_steeringAngle < 1000)
    {
        odom_steeringAngle = 1000;
    }
}

//Subscribe to the local planner and map the steering angle (and the velocity-but we dont do that here-) to pulse width modulation values.
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_linearVelocity = msg->linear.x;
    cmd_angularVelocity = msg->angular.z;

    cmd_steeringAngle = 180/PI*atan(cmd_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle;

    if(cmd_steeringAngle > 2000)
    {
        cmd_steeringAngle = 2000;
    }
    else if(cmd_steeringAngle < 1000)
    {
        cmd_steeringAngle = 1000;
    }
}
// a flag method that tells us if we are controlling the car manually or automatically
void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}

//geometry_msgs::Vector3 control::P_Controller()
//{
//    current_ServoMsg.x = previous_ServoMsg.x + Fp*(cmd_linearVelocity - odom_linearVelocity);

//    current_ServoMsg.y = cmd_steeringAngle;


//    if(current_ServoMsg.x > 1580)
//    {
//        current_ServoMsg.x = 1580;
//    }
//    else if(current_ServoMsg.x < 1300)
//    {
//        current_ServoMsg.x = 1300;
//    }

//    if(current_ServoMsg.y > 2000)
//    {
//        current_ServoMsg.y = 2000;
//    }
//    else if(current_ServoMsg.y < 1000)
//    {
//        current_ServoMsg.y = 1000;
//    }

//    previous_ServoMsg = current_ServoMsg;

//    return current_ServoMsg;
//}
