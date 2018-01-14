#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <std_msgs/Empty.h>
#include <mainframe/RawControl.h> // Custom ROS msgs
#include <rover/DriveCmd.h>
#include <rover/ArmCmd.h>

#define LOOP_HZ 10
#define AXIS_THRES 0.5

float drive_percent = 0; // Speed control command, from -100% to 100%.
float steer_angle = 0; // Steering angle command, from -45 to 45 degrees

int base     = 0; // Arm commands
int shoulder = 0;
int forearm  = 0;
int wrist_x  = 0;
int wrist_y  = 0;
int twist    = 0;
int grip     = 0;

int sensitivity = 1; // Default arm sensitivity - slowest mode (1-5)

bool arm_mode = false; // driving only enabled if not in arm mode

// Clamp value within range - convenience function
int clamp(int value, int max, int min)
{
  if (value > max) return max;
  else if (value < min) return min;
  else return value;
}

void ctrl_data_cb(const mainframe::RawControl::ConstPtr& msg)  
{      
  if (msg->but_y) // Toggle control mode if Y button pressed (drive/arm mode)
  {
    arm_mode = !arm_mode;
    drive_percent = 0;
    steer_angle   = 0;

    base          = 0; 
    shoulder      = 0;
    forearm       = 0;
    wrist_x       = 0;
    wrist_y       = 0;
    grip          = 0; 
    twist         = 0; 
    // sensitivity is remembered
  }

  if (!arm_mode) // If driving
  {
    drive_percent = 100*(msg->axis_ly); // Drive with left stick vertical
    steer_angle = 45*(msg->axis_rx); // Steer with right stick horizontal

    //ROS_INFO_STREAM("Drive: " << drive_percent << "%"); 
    //ROS_INFO_STREAM("Steer: " << steer_angle << "%"); 
  }
  else // If controlling arm
  {
    base     = (msg->axis_rx > AXIS_THRES) - (msg->axis_rx < -AXIS_THRES); // -1, 0 or 1
    shoulder = (msg->axis_ry > AXIS_THRES) - (msg->axis_ry < -AXIS_THRES); // -1, 0 or 1
    forearm  = (msg->axis_ly > AXIS_THRES) - (msg->axis_ly < -AXIS_THRES); // -1, 0 or 1
    wrist_x  = msg->axis_dx;
    wrist_y  = msg->axis_dy;
    twist    = msg->bump_r - msg->bump_l; // -1, 0 or 1
    grip     = msg->trig_r - msg->trig_l; // -1, 0 or 1

    sensitivity = clamp(sensitivity + msg->but_b - msg->but_a, 5, 1); // + on B, - on A    
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "overlord");
  ros::NodeHandle n; 
  ros::Rate loop_rate(LOOP_HZ); // Loop rate in Hz

  ros::Subscriber control_sub = n.subscribe("ctrl_data", 10, ctrl_data_cb);

  ros::Publisher drivecmd_pub = n.advertise<rover::DriveCmd>("cmd_data", 10);
  ros::Publisher armcmd_pub = n.advertise<rover::ArmCmd>("arm_cmd_data", 10);

  ros::Publisher hbeat_pub = n.advertise<std_msgs::Empty>("hbeat", 1);

  int hbeat_loop_cnt = 0;

  while (ros::ok())
  {    

    if (!arm_mode) // If driving
    {
      // Create ROS msg for drive command
      rover::DriveCmd msg;

      // Store current values in ROS msg
      msg.acc = drive_percent; // Named because of plans for acceleration control
      msg.steer = steer_angle;

      // Publish the ROS msg
      drivecmd_pub.publish(msg);
    }
    else // If controlling arm
    {
      // Create ROS msg for arm command
      rover::ArmCmd msg;

      // Store current arm parameters
      msg.base     = base;
      msg.shoulder = shoulder;
      msg.forearm  = forearm;
      msg.wrist_x  = wrist_x;
      msg.wrist_y  = wrist_y;
      msg.grip     = grip;
      msg.twist    = twist;

      msg.sensitivity = sensitivity;

      armcmd_pub.publish(msg);
    }

    // Send a heartbeat once per second
    if (hbeat_loop_cnt > LOOP_HZ)
    {
      std_msgs::Empty msg;
      hbeat_pub.publish(msg);
      hbeat_loop_cnt = 0;
    }

    hbeat_loop_cnt++;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

