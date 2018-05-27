#include "ros/ros.h"
#include <ros/console.h>
#include <stdlib.h>
#include <math.h>

#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <mainframe/RawControl.h> // Custom ROS msgs
#include <rover/DriveCmd.h>
#include <rover/ArmCmd.h>
#include <rover/RedCmd.h>

using namespace std;

#define LOOP_HZ 10

#define AXIS_THRES 0.2
#define MAX_DELTA_DRIVE 10.0 // % a drive command can change in an iteration

float drive_percent = 0; // Speed control command, from -100% to 100%.
float steer_angle = 0; // Steering angle command, from -45 to 45 degrees

bool start    = 0;
bool back     = 0;
int base      = 0; // Arm commands
int shoulder  = 0;
int forearm   = 0;
int wrist_x   = 0;
int wrist_y   = 0;
int twist     = 0;
int end_angle = 0;
int end_pos   = 0;
int sensitivity = 1; // Default arm sensitivity - slowest mode (1-5)

int drill_spd = 0;
int sensor_spd = 0;
int stepper_pos = 0;
int actuator_spd = 0;

ros::NodeHandle* n; 
ros::ServiceClient toggle_mode_clnt; // Service client for mode toggling

// Clamp value within range - convenience function
int clamp(int value, int max, int min)
{
  if (value > max) return max;
  else if (value < min) return min;
  else return value;
}

int sign(float num)
{
  return (num > 0) - (num < 0);
}

void ctrl_data_cb(const mainframe::RawControl::ConstPtr& msg)  
{      
  string STATE; (*n).getParam("STATE", STATE); // Grab the current state

  if (msg->but_y) // Toggle control mode if Y button pressed (drive/arm mode)
  {
    std_srvs::Trigger srv; // Call service to toggle state/mode
    toggle_mode_clnt.call(srv);
    ROS_INFO_STREAM(srv.response.message); // Print response

    drive_percent = 0;
    steer_angle   = 0;

    start         = 0;
    back          = 0;
    base          = 0; 
    shoulder      = 0;
    forearm       = 0;
    wrist_x       = 0;
    wrist_y       = 0;
    end_angle     = 0; 
    end_pos       = 0; 
    // sensitivity is remembered
  }
  else if (STATE == "DRIVE") // If driving
  {
    // Limit the amount the drive_percent can change by in this single iteration
    float delta_drive_pcnt = 100*(msg->axis_ly) - drive_percent;  
    delta_drive_pcnt = clamp(delta_drive_pcnt, MAX_DELTA_DRIVE, -MAX_DELTA_DRIVE);

    //drive_percent = drive_percent + delta_drive_pcnt; // Apply change
    drive_percent = 100*(msg->axis_ly);

    steer_angle = 100*(msg->axis_rx); // Steer with right stick horizontal

    //ROS_INFO_STREAM("Drive: " << drive_percent << "%"); 
    //ROS_INFO_STREAM("Steer: " << steer_angle << "%"); 
  }
  else if (STATE == "ARM") // If controlling arm
  {
    start     = msg->start;
    back      = msg->back;

    float scaler = 4095.0/(1.0 - AXIS_THRES);

    base      = (fabs(msg->axis_rx) > AXIS_THRES)
                  *(fabs(msg->axis_rx) - AXIS_THRES)*scaler/2;

    shoulder  = (fabs(msg->axis_ry) > AXIS_THRES)
                 *(fabs(msg->axis_ry) - AXIS_THRES)*scaler;

    wrist_y   = (fabs(msg->axis_ly) > AXIS_THRES)
                 *(fabs(msg->axis_ly) - AXIS_THRES)*scaler;

    wrist_x   = (fabs(msg->axis_lx) > AXIS_THRES)
                 *(fabs(msg->axis_lx) - AXIS_THRES)*scaler;

    base     = sign(msg->axis_rx) *clamp(base,     2000, 0);
    shoulder = sign(msg->axis_ry) *clamp(shoulder, 4095, 0);
    wrist_y  = sign(msg->axis_ly) *clamp(wrist_y,  4095, 0);
    wrist_x  = sign(msg->axis_lx) *clamp(wrist_x,  4095, 0);

    twist     = msg->axis_dx;
    forearm   = msg->axis_dy;

    end_pos   = msg->trig_r - msg->trig_l; // -1, 0 or 1
    end_angle = msg->bump_r - msg->bump_l; // -1, 0 or 1

    sensitivity = clamp(sensitivity + msg->but_b - msg->but_a, 5, 1); // + on B, - on A    
  }
  else if (STATE == "DRILL") // If driving
  {
    if (msg->trig_r)      drill_spd =  255*(msg->trig_r_val);
    else                  drill_spd = -255*(msg->trig_l_val);

    sensor_spd = 255*(msg->axis_ry);
    stepper_pos = msg->axis_dx;
    actuator_spd = 255*(msg->axis_ly);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "overlord");
  n = new ros::NodeHandle("/"); 

  ros::Rate loop_rate(LOOP_HZ); // Loop rate in Hz

  ros::Subscriber control_sub = (*n).subscribe("ctrl_data", 10, ctrl_data_cb);

  ros::Publisher drivecmd_pub = (*n).advertise<rover::DriveCmd>("cmd_data", 10);
  ros::Publisher armcmd_pub = (*n).advertise<rover::ArmCmd>("arm_cmd_data", 1);
  ros::Publisher redcmd_pub = (*n).advertise<rover::RedCmd>("red_cmd_data", 1);

  ros::Publisher hbeat_pub = (*n).advertise<std_msgs::Empty>("/hbeat", 1);

  toggle_mode_clnt = (*n).serviceClient<std_srvs::Trigger>("/Toggle_Mode");

  int hbeat_loop_cnt = 0;

  while (ros::ok())
  {     
    string STATE; (*n).getParam("STATE", STATE); // Grab the current state

    if (STATE == "DRIVE") // If driving
    {
      // Create ROS msg for drive command
      rover::DriveCmd msg;

      // Store current values in ROS msg
      msg.acc = drive_percent; // Named because of plans for acceleration control
      msg.steer = steer_angle;

      // Publish the ROS msg
      drivecmd_pub.publish(msg);
    }
    else if (STATE == "ARM") // If controlling arm
    {
      rover::ArmCmd msg; // Create ROS msg for arm command

      // Store current arm parameters
      msg.start     = start;
      msg.back      = back;
      msg.base      = base;
      msg.shoulder  = -shoulder;
      msg.forearm   = forearm;
      msg.wrist_x   = -wrist_x;
      msg.wrist_y   = wrist_y;
      msg.twist     = twist;
      msg.end_angle = end_angle;
      msg.end_pos   = end_pos;

      msg.sensitivity = sensitivity;

      armcmd_pub.publish(msg);
    }
    else if (STATE == "DRILL") // If controlling arm
    {
      rover::RedCmd msg; // Create ROS msg for arm command
  
      msg.drillSpd = drill_spd;
      msg.sensorSpeed = sensor_spd;
      msg.stepperPos = stepper_pos;
      msg.actuatorSpeed = actuator_spd;

      redcmd_pub.publish(msg);
    }
    else if (STATE == "STANDBY")
    {
      rover::DriveCmd msg;
      msg.acc = 0;
      msg.steer = 0;
      drivecmd_pub.publish(msg);
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

