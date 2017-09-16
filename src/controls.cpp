/*
  mainframe - controls.cpp

This is a ROS node to handle the input from an Xbox controller. It
reads values from the sticks, triggers and buttons and distributes
them into the ROS network as "RawControl" messages.

Author: Benjamin Steer
Organisation: Nova Rover Team
*/

#include "ros/ros.h"
#include <ros/console.h>

#include <gamepad/gamepad.h> // Xbox controller library

#include <mainframe/RawControl.h> // Custom ROS msg

// Sign function
int sgn(int val) {
    return (0 < val) - (val < 0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls");
  ros::NodeHandle n;
  ros::Rate loop_rate(10); // 10 Hz

  // Create a ROS "Publisher" object to distribute information from the
  // Xbox controller to the rest of the ROS network
  ros::Publisher control_data_pub = n.advertise<mainframe::RawControl>("control_data", 1000);
 
  GamepadInit(); // Initialise the Xbox gamepad

  // Deadzone compensation - see more info in main ROS loop below
  const int dead_l = GAMEPAD_DEADZONE_LEFT_STICK;
  const int dead_r = GAMEPAD_DEADZONE_RIGHT_STICK;
  const int stick_max_l = 32767 - dead_l;
  const int stick_max_r = 32767 - dead_r;

  // Initialising variables to be refreshed during loop
  int stick_lx = 0;
  int stick_ly = 0;
  int stick_rx = 0;
  int stick_ry = 0;
  bool but_a = false;

  while (ros::ok())
  {
    GamepadUpdate(); // Updates the state of the gamepad

    // Broadcasting stick values
    GamepadStickXY(GAMEPAD_0, STICK_LEFT, &stick_lx, &stick_ly);
    GamepadStickXY(GAMEPAD_0, STICK_RIGHT, &stick_rx, &stick_ry);

    //bool r_trigger = GamepadTriggerDown(GAMEPAD_0, TRIGGER_RIGHT);
    //bool l_trigger = GamepadTriggerDown(GAMEPAD_0, TRIGGER_LEFT);
  
    bool but_a = GamepadButtonTriggered(GAMEPAD_0, BUTTON_A);

    mainframe::RawControl msg; // Msg to use for stick vals

    // The gamepad sticks have a deadzone - which means for a small amount of
    // movement of the stick, the reading remains at zero. This means as soon
    // as you move the stick out of the deadzone, the reading will jump from 
    // zero to some higher value. The calculations here account for this and 
    // rescale the values to remove this jump.
    float stick_lx = sgn(stick_lx)*((float) (abs(stick_lx) - dead_l))/stick_max_l;
    float stick_ly = sgn(stick_ly)*((float) (abs(stick_ly) - dead_l))/stick_max_l;

    float stick_rx = sgn(stick_rx)*((float) (stick_rx - dead_l))/stick_max_r;
    float stick_ry = sgn(stick_ry)*((float) (stick_ry - dead_l))/stick_max_r;

    // Set the values in the ROS msg
    msg.axis_lx = stick_lx; 
    msg.axis_ly = stick_ly;   

    msg.axis_rx = stick_rx; 
    msg.axis_ry = stick_ry;     

    msg.but_a = but_a;

    // Publish the ROS msg
    control_data_pub.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

