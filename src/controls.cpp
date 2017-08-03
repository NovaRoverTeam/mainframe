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

    int l_stick_x = 0;
    int l_stick_y = 0;

    // Broadcasting left stick values at 10 Hz
    GamepadStickXY(GAMEPAD_0, STICK_LEFT, &l_stick_x, &l_stick_y);

    bool r_trigger = GamepadTriggerTriggered(GAMEPAD_0, TRIGGER_RIGHT);
    bool l_trigger = GamepadTriggerTriggered(GAMEPAD_0, TRIGGER_LEFT);

    bool r_trigger_rel = GamepadTriggerReleased(GAMEPAD_0, TRIGGER_RIGHT);
    bool l_trigger_rel = GamepadTriggerReleased(GAMEPAD_0, TRIGGER_LEFT);
  
    bool a_but = GamepadButtonTriggered(GAMEPAD_0, BUTTON_A);

    mainframe::RawControl stick_msg; // Msg to use for stick vals

    // GAMEPAD_DEADZONE_LEFT_STICK is handy
    float stick_x = ((float) l_stick_x)/32767;
    float stick_y = ((float) l_stick_y)/32767;

    stick_msg.axis_x = stick_x; 
    stick_msg.axis_y = stick_y;      

    if (r_trigger == true)
    {
      stick_msg.depress = true;
      stick_msg.trigger_left = false;
    }
    else if (r_trigger_rel == true)
    {
      stick_msg.depress = false;
      stick_msg.trigger_left = false;
    }
    else if (l_trigger == true)
    {
      stick_msg.depress = true;
      stick_msg.trigger_left = true;
    }
    else if (l_trigger_rel == true)
    {
      stick_msg.depress = false;
      stick_msg.trigger_left = true;
    }

    stick_msg.button = a_but;

    control_data_pub.publish(stick_msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

