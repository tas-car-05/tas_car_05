/**************************************************************************************************************
*	node tas_autonomous_control_node:
*
*		programmer:
*			- Christoph Allig		christoph.allig@tum.de
*			- Marcin Kasperek		marcin.kasperek@tum.de
*	
*		functionality:
*			- Control the velocity of the car dependent on the delivered velocity from the local planer 
*
*		New implementation compared to the origin node:
*			- if the car is located on the straight track, the velocity is higher than in a curve
*			- The acceleration is limited to provide a soft start (commented, because the influence is
*			  not tested so far)
*	
*		last modified: 17.01.2015
**************************************************************************************************************/

#include "control/control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;
	
	// 50Hz
    ros::Rate loop_rate(50);            

    while(ros::ok())
    {
		// Manually Control
        if(false)//autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }
		// Automatic Control
        else
        {
			// car brakes due to pressing the wii-button
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
			// control the velocity of the car dependent on the delivered velocity of the car
            else
            {
                ROS_INFO("Automatic Control!");
				// driving forward
                if(autonomous_control.cmd_linearVelocity>0)
                {
					// driving in a curve
                    if (autonomous_control.Kurve == 1)
                    {
						autonomous_control.control_servo.x = 1550;
						
						// for Debugging
						// ROS_INFO("Curve");
                        // ROS_INFO("est_position_x_1550");

						/*****************************+********************************************************
						*	// part of the non-tested acceleration limit
						*	// Counter reset for acceleration limit
						*	autonomous_control.Counter = 0;
                        **************************************************************************************/

						/**************************************************************************************
						*	// first idea, but did not worked due to friction 
						* 	// the car is only driving with values bigger than 1540
						*	autonomous_control.control_servo.x = 1500 + 36*autonomous_control.cmd_linearVelocity;
						**************************************************************************************/
                    }
					// driving on a straight track
                    else
                    {
						autonomous_control.control_servo.x = 1555;
						
						// for Debugging
						// ROS_INFO("straight");
                        // ROS_INFO("est_position_x_1555");
						
						/************************************************************************************** 
						*	// part of the non-tested acceleration limit
						*	// changing speed after 0.5 s  
						*	// Increment the speed with 1
						*	// Start speed: 1550
						*	// Maximum speed:1565
                        *	if(autonomous_control.Counter>24 & autonomous_control.control_servo.x < 1565)
                        *	{
						*		autonomous_control.control_servo.x = autonomous_control.control_servo.x +1;
						*		autonomous_control.Counter = 0;
						*	}
                        *	// set the start speed to 1550. Increment the counter.
						*	else
						*	{
						*		// set the speed on the very first time to 1550
						*		if(autonomous_control.control_servo.x <1550)
						*		{
						*			autonomous_control.control_servo.x = 1550;
						*		}
						*		// increment the counter
						*		else
						*		{
						*			autonomous_control.Counter = autonomous_control.Counter + 1;
						*		}
						*
						*	}
						**************************************************************************************/

						/**************************************************************************************
						*	// first idea, but did not worked due to friction 
						* 	// the car is only driving with values bigger than 1540
						*	autonomous_control.control_servo.x = 1510 + 36*autonomous_control.cmd_linearVelocity;
						**************************************************************************************/					   
                    }
				}
				// driving backward
                else if(autonomous_control.cmd_linearVelocity<0)
                {
					autonomous_control.control_servo.x = 1420;
					
					// for Debugging
                    // ROS_INFO("est_position_x_1420");
                }
				// car stops
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }
				// steering
                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }
			// publish topic servo
            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
