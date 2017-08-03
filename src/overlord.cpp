#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include "mainframe/RawControl.h"
#include "mainframe/Command.h"

#include <rover/DriveCommand.h>
#include <rover/SteerCommand.h>

#define MAX_PWM 4096

float stick_threshold = 0;

class PubScrub
{
public:
  PubScrub()
  {
    pub_ = n_.advertise<mainframe::Command>("command_data", 1000);
    
    scrub_ = n_.subscribe("control_data", 1000, &PubScrub::control_dataCallback, this);
    
    drive_client = n_.serviceClient<rover::DriveCommand>("DriveCommand");

    steer_client = n_.serviceClient<rover::SteerCommand>("SteerCommand");
  }
  
  void control_dataCallback(const mainframe::RawControl::ConstPtr& msg)  
  {    
    float x_axis = msg->axis_x;
    float y_axis = msg->axis_y;

    ROS_INFO_STREAM(y_axis); 

    if (fabs(y_axis) > stick_threshold) 
    {
      rover::DriveCommand srv;

      ROS_INFO_STREAM(y_axis*((float)MAX_PWM)); 
      ROS_INFO_STREAM(int(y_axis*((float)MAX_PWM))); 

      srv.request.f_wheel_l = int(y_axis*((float)MAX_PWM));
      srv.request.f_wheel_r = int(y_axis*((float)MAX_PWM));
      srv.request.b_wheel_l = int(y_axis*((float)MAX_PWM));
      srv.request.b_wheel_r = int(y_axis*((float)MAX_PWM));

      drive_client.call(srv);
    }    

    rover::SteerCommand srv;

    srv.request.single = msg->button;
    srv.request.start = msg->depress;
    srv.request.steer_left = msg->trigger_left;

    ROS_INFO_STREAM(srv.request.steer_left); 

    steer_client.call(srv);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber scrub_;
  ros::ServiceClient drive_client;
  ros::ServiceClient steer_client;

}; // END OF CLASS - PubScrub

int main(int argc, char **argv)
{
  ros::init(argc, argv, "overlord");

  PubScrub PS; // Create PubScrub object

  ros::spin();

  return 0;
}

