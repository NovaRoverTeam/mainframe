#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include "mainframe/RawControl.h"
#include "mainframe/Command.h"
#include <rover/DriveCommand.h>

#define MAX_PWM 4096

float stick_threshold = 0.1;

class PubScrub
{
public:
  PubScrub()
  {
    pub_ = n_.advertise<mainframe::Command>("command_data", 1000);
    
    scrub_ = n_.subscribe("control_data", 1000, &PubScrub::control_dataCallback, this);
    
    client_ = n_.serviceClient<rover::DriveCommand>("DriveCommand");
  }
  
  void control_dataCallback(const mainframe::RawControl::ConstPtr& msg)  
  {    
    int x_axis = msg->axis_x;
    int y_axis = msg->axis_y;

    ROS_INFO_STREAM(y_axis); 

    if (fabs(y_axis) > stick_threshold) 
    {
      rover::DriveCommand srv;

      srv.request.f_wheel_l = (int) y_axis*MAX_PWM;
      srv.request.f_wheel_r = (int) y_axis*MAX_PWM;
      srv.request.b_wheel_l = (int) y_axis*MAX_PWM;
      srv.request.b_wheel_r = (int) y_axis*MAX_PWM;

      client_.call(srv);
    }    
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber scrub_;
  ros::ServiceClient client_;

}; // END OF CLASS - PubScrub

int main(int argc, char **argv)
{
  ros::init(argc, argv, "overlord");

  PubScrub PS; // Create PubScrub object

  ros::spin();

  return 0;
}

