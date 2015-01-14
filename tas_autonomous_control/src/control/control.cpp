#include "control.h"
#include <fstream>

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

    position_sub = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 1000, &control::positionCallback, this);

//    Fp = 10;// need to test! defult:125

//    current_ServoMsg.x = 1500;
//    current_ServoMsg.y = 1500;

//    previous_ServoMsg.x = 1500;
//    previous_ServoMsg.y = 1500;

}
// We can subscribe to the odom here and get some feedback signals so later we can build our controllers



void control::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    est_position_x = msg->pose.position.x;
    est_position_y = msg->pose.position.y;

    //ROS_INFO("est_position_x: [%f]", est_position_x);
            //Kurve links oben
            if(9.5 < est_position_x && est_position_x < 14 && -1.5 < est_position_y && 1.5 < est_position_y)
            {

                {  //ROS_INFO("Kurve links oben");
                   //ROS_INFO("est_position_x: [%f]", est_position_x);
                   //ROS_INFO("est_position_y: [%f]", est_position_y);

                   Kurve = 1;
                }
            }

            //Kurve rechts oben
            else if(10.5 < est_position_x && est_position_x < 14 && -14 < est_position_y && est_position_y< -11)
            {
                //ROS_INFO("Kurve rechts oben");
                //ROS_INFO("est_position_x: [%f]", est_position_x);
                //ROS_INFO("est_position_y: [%f]", est_position_y);
                Kurve = 1;
            }

            //Kurve rechts unten
            else if(-2 < est_position_x && est_position_x< 1.5 && -14 < est_position_y && est_position_y < -10.5)
            {
                //ROS_INFO("Kurve rechts unten");
                //ROS_INFO("est_position_x: [%f]", est_position_x);
                //ROS_INFO("est_position_y: [%f]", est_position_y);
                Kurve = 1;
            }
/*
            //Kurve links unten
            else if((6 < est_position_x < 10) && (4.5 < est_position_y < 8.5))
            {
                Kurve = 1;
            }
*/
            //Gerade
            else
            {
                 //ROS_INFO("est_position_x_Gerade: [%f]");
                 //ROS_INFO("est_position_x: [%f]", est_position_x);
                 //ROS_INFO("est_position_y: [%f]", est_position_y);
                 Kurve = 0;
            }

/*
            ofstream datei1 ("papa.dat");

            if (!datei1)
            {
                cerr << "Dateien werden nicht geoeffnet.";

            }

            datei1 << '\n' << est_position_x << '\t' << est_position_y;
*/




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
