#include "control/control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);            //50Hz

    while(ros::ok())
    {
        if(false) //autonomous_control.control_Mode.data==0)
        {
            //ROS_INFO("Manually Control!");
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                ROS_INFO("Automatic Control!");

                if(autonomous_control.cmd_linearVelocity>0)
                {
                    if (autonomous_control.Kurve == 1)
                    {
                        ROS_INFO("est_position_x_1550:");


                        //sanftes Anfahren
                        /*
                        // Counter ruecksetzen fuer sanfte Anfahrt
                        autonomous_control.Counter = 0;
                        */


                        autonomous_control.control_servo.x = 1550;
                        //autonomous_control.control_servo.x = 1500 + 36*autonomous_control.cmd_linearVelocity;
                    }
                    else
                    {
                        ROS_INFO("est_position_x_1650:");
                        autonomous_control.control_servo.x = 1555;
                       //autonomous_control.control_servo.x = 1500 + 36*autonomous_control.cmd_linearVelocity;
                    }
                                    }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    ROS_INFO("est_position_x_1400:");
                    autonomous_control.control_servo.x = 1420;
                }
                else
                {
                    ROS_INFO("est_position_x_1500: ");
                    // sanfte Anfahrt
                   /*
                    if(autonomous_control.Counter == 0 )
                    {
                        autonomous_control.control_servo.x = 1551;
                        autonomous_control.Counter = 1;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 1 )
                    {
                        autonomous_control.control_servo.x = 1552;
                        autonomous_control.Counter = 2;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 2 )
                    {
                        autonomous_control.control_servo.x = 1553;
                        autonomous_control.Counter = 3;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 3 )
                    {
                        autonomous_control.control_servo.x = 1554;
                        autonomous_control.Counter = 4;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 4 )
                    {
                        autonomous_control.control_servo.x = 1555;
                        autonomous_control.Counter = 5;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 5 )
                    {
                        autonomous_control.control_servo.x = 1556;
                        autonomous_control.Counter = 6;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 6 )
                    {
                        autonomous_control.control_servo.x = 1557;
                        autonomous_control.Counter = 7;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 7 )
                    {
                        autonomous_control.control_servo.x = 1558;
                        autonomous_control.Counter = 8;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 8 )
                    {
                        autonomous_control.control_servo.x = 1559;
                        autonomous_control.Counter = 9;
                        sleep(0.5);
                    }
                    else if(autonomous_control.Counter == 9 )
                    {
                        autonomous_control.control_servo.x = 1560;
                        autonomous_control.Counter = 10;
                        sleep(0.5);
                    }
*/
                    autonomous_control.control_servo.x = 1500;
                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
