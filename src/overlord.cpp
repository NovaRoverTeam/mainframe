#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <std_msgs/Empty.h>
#include <mainframe/RawControl.h> // Custom ROS msgs
#include <rover/DriveCmd.h>

#define LOOP_HZ 10

float drive_percent = 0; // Speed control command, from -100% to 100%.
float steer_angle = 0; // Steering angle command, from -45 to 45 degrees

void ctrl_data_cb(const mainframe::RawControl::ConstPtr& msg)  
{      
  drive_percent = 100*(msg->axis_ly); // Drive with left stick vertical
  steer_angle = 45*(msg->axis_rx); // Steer with right stick horizontal

  ROS_INFO_STREAM("Drive: " << drive_percent << "%"); 
  ROS_INFO_STREAM("Steer: " << steer_angle << "%"); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "overlord");
  ros::NodeHandle n; 
  ros::Rate loop_rate(LOOP_HZ); // Loop rate in Hz

  ros::Subscriber control_sub = n.subscribe("ctrl_data", 10, ctrl_data_cb);

  ros::Publisher drivecmd_pub = n.advertise<rover::DriveCmd>("cmd_data", 10);

  ros::Publisher hbeat_pub = n.advertise<std_msgs::Empty>("hbeat", 1);

  int hbeat_loop_cnt = 0;

  while (ros::ok())
  {    
    // Create ROS msg for drive command
    rover::DriveCmd msg;

    // Store current values in ROS msg
    msg.acc = drive_percent; // Named because of plans for acceleration control
    msg.steer = steer_angle;

    // Publish the ROS msg
    drivecmd_pub.publish(msg);

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

