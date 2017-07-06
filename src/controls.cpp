#include "ros/ros.h"
#include <ros/console.h>
#include "mainframe/RawControl.h"

#include "gamepad.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls");
  ros::NodeHandle n;
  ros::Publisher control_data_pub = n.advertise<mainframe::RawControl>("control_data", 1000);
  ros::Rate loop_rate(10); // 10 Hz

  GamepadInit(); // Initialise the Xbox gamepad

  while (ros::ok())
  {
    GamepadUpdate(); // Updates the state of the gamepad
    //float l_stick_x = 0;
    //float l_stick_y = 0;
    int l_stick_x = 0;
    int l_stick_y = 0;

    // Broadcasting left stick values at 10 Hz
    
    //GamepadStickNormXY(GAMEPAD_0, STICK_LEFT, &l_stick_x, &l_stick_y);
    GamepadStickXY(GAMEPAD_0, STICK_LEFT, &l_stick_x, &l_stick_y);

    mainframe::RawControl stick_msg; // Message to use for stick values

    // GAMEPAD_DEADZONE_LEFT_STICK is handy
    float stick_x = ((float) l_stick_x)/32767;
    float stick_y = ((float) l_stick_y)/32767;

    //stick_msg.type = 1;
    //stick_msg.id   = 0;
    stick_msg.axis_x = stick_x; 
    stick_msg.axis_y = stick_y;      

    control_data_pub.publish(stick_msg);

    //stick_msg.id   = 1;
    //stick_msg.length = l_stick_y;      
    //control_data_pub.publish(stick_msg);

    // Testing
    //int x;
    //int y;
    //GamepadStickXY(GAMEPAD_0, STICK_LEFT, &x, &y);
    //ROS_INFO("%d", x);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

