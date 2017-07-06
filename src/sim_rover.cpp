#include "ros/ros.h"
#include <ros/console.h>
#include <string>

// Message Definitions
#include "mainframe/Command.h"
//#include "gazebo_msgs/ApplyJointEffort.h"
//#include "gazebo_msgs/JointRequest.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h" 
#include "std_msgs/Float64.h"

class PubScrub
{
public:
  PubScrub()
  {
    bufSize = 100;
    stiffness = 1000.0;
    driveTorque = 4.0;
    steerTorque = 8.0;

    nh.getParam("/robot_name", robot_name);

    // ******************* SUBSCRIBERS **********************

    scrub_cmd = nh.subscribe("command_data", bufSize, &PubScrub::command_data_cb, this);

    scrub_jnt = nh.subscribe("/"+ robot_name +"/joint_states", bufSize, &PubScrub::joint_states_cb, this);
    
    // ******************* PUBLISHERS ***********************

    pub_bleg_r = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/bleg_r_eff/cmd", bufSize);    
    pub_bleg_l = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/bleg_l_eff/cmd", bufSize);
    pub_fleg_r = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/fleg_r_eff/cmd", bufSize);
    pub_fleg_l = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/fleg_l_eff/cmd", bufSize);

    pub_bwheel_r = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/bwheel_r_eff/cmd", bufSize);    
    pub_bwheel_l = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/bwheel_l_eff/cmd", bufSize);
    pub_fwheel_r = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/fwheel_r_eff/cmd", bufSize);    
    pub_fwheel_l = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/fwheel_l_eff/cmd", bufSize);

    pub_bsteer_r = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/bsteer_r_eff/cmd", bufSize);    
    pub_bsteer_l = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/bsteer_l_eff/cmd", bufSize);
    pub_fsteer_r = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/fsteer_r_eff/cmd", bufSize);    
    pub_fsteer_l = nh.advertise<std_msgs::Float64>("/"+ robot_name +"/fsteer_l_eff/cmd", bufSize);
  }

  void command_data_cb(const mainframe::Command::ConstPtr& msg)
  { 
    std_msgs::Float64 torque_l;
    std_msgs::Float64 torque_r;
    
    if (msg->command == 1) // Driving
    {
      torque_l.data = (msg->value1)*driveTorque; 
      torque_r.data = (msg->value2)*driveTorque; 
    }
    else if (msg->command == 2) // Steering
    {
      torque_l.data = (msg->value1)*steerTorque; 
      torque_r.data = (msg->value2)*steerTorque; 
    }

    /*
    pub_wheel_fr.publish(torque_r);    
    pub_wheel_br.publish(torque_r);
    pub_wheel_fl.publish(torque_l);    
    pub_wheel_bl.publish(torque_l);
    */
  }    

  void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
  { 
    sensor_msgs::JointState jstate = *(msg);
    ros::Publisher* pub_force;

    for (int i = 0; i < 4; i++) 
    {
      if (jstate.name[4*i] == "j_bleg_r") {      
        pub_force = &pub_bleg_r;
      } 
      else if (jstate.name[4*i] == "j_bleg_l") {        
        pub_force = &pub_bleg_l;
      } 
      else if (jstate.name[4*i] == "j_fleg_r") {      
        pub_force = &pub_fleg_r;
      } 
      else if (jstate.name[4*i] == "j_fleg_l") {        
        pub_force = &pub_fleg_l;
      }  

      float position = jstate.position[i];
      std_msgs::Float64 force;       // Force to publish to joint
      force.data = -position*stiffness; // Spring force

      //pub_force->publish(force);
    }
  }   

private:
  ros::NodeHandle nh;   
  ros::Subscriber scrub_cmd;
  ros::Subscriber scrub_jnt;

  ros::Publisher pub_bleg_r;
  ros::Publisher pub_bleg_l;
  ros::Publisher pub_fleg_r;
  ros::Publisher pub_fleg_l;

  ros::Publisher pub_bwheel_r;
  ros::Publisher pub_bwheel_l;
  ros::Publisher pub_fwheel_r;
  ros::Publisher pub_fwheel_l;

  ros::Publisher pub_bsteer_r;
  ros::Publisher pub_bsteer_l;
  ros::Publisher pub_fsteer_r;
  ros::Publisher pub_fsteer_l;

  int bufSize;
  
  float stiffness; // suspension spring stiffness (N/m)  
  float driveTorque;
  float steerTorque;

  std::string robot_name;
  
  float init_bleg_r;
  float init_bleg_l;
  float init_fleg_r;
  float init_fleg_l;

}; // END OF CLASS - PubScrub

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_rover");

  PubScrub PS;

  ros::spin();
  return 0;
}











/*
class ClientScrub
{
public:
  ClientScrub()
  {
    client1_ = n_.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");

    client2_ = n_.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces");

    scrub_ = n_.subscribe("command_data", 1000, &ClientScrub::command_dataCallback, this);
  }

  void command_dataCallback(const mainframe::Command::ConstPtr& msg)
  { 
    if (msg->command == 1) 
    {
      ROS_INFO("Forward");
      //actuateJoint("test_rover::chassis_JOINT_0", 1);
    }
    else if (msg->command == 2)
    {
      ROS_INFO("Backward");
      //actuateJoint("test_rover::chassis_JOINT_0", -1);\    
    }    
    else 
    {
      gazebo_msgs::JointRequest jr;

      //jr.request.joint_name = "test_rover::chassis_JOINT_0";
      client2_.call(jr);

      //jr.request.joint_name = "test_rover::chassis_JOINT_3";
      client2_.call(jr);
    }
  }

  void actuateJoint(const char joint_name[], int dir)
  {    
    gazebo_msgs::ApplyJointEffort gz_joint;
    gz_joint.request.joint_name = joint_name;
    gz_joint.request.effort = dir*50;
    gz_joint.request.duration = ros::Duration(0.1);

    client1_.call(gz_joint);
  }

private:
  ros::NodeHandle n_; 
  ros::ServiceClient client1_;
  ros::ServiceClient client2_;
  ros::Subscriber scrub_;

}; // END OF CLASS - ClientScrub
*/
