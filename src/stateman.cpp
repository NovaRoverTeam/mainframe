#include "ros/ros.h"
#include <stdlib.h>
#include <string>

#include <gps/Gps.h>
#include <autopilot/calc_route.h>
#include <autopilot/set_dest.h>
#include <std_srvs/Trigger.h>

#include <boost/thread.hpp> // Handles multi-threading
#include <signal.h>

using namespace std;

ros::NodeHandle* n; 
ros::ServiceClient start_auto_client;
ros::ServiceClient set_dest_client;
boost::thread* spinner_t; // Thread for ROS spinning

string parseError;
string toggleError;
string cancel;
string back;

double test_lat, test_long;

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Toggle_Mode:
//    This service toggles the current state between driving, robot arm
//    and drilling.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Toggle_Mode(std_srvs::Trigger::Request  &req,
                 std_srvs::Trigger::Response &res)
{
  bool parsed = true; // Valid state stored in STATE var

  string STATE; n->getParam("/STATE", STATE);

  // *** TOGGLE THE STATE BETWEEN DRIVE, ARM, DRILL ***
  if      (STATE == "DRIVE") n->setParam("/STATE", "ARM");
  else if (STATE == "ARM"  ) n->setParam("/STATE", "DRILL");
  else if (STATE == "DRILL") n->setParam("/STATE", "DRIVE");
  else // If current state is not any of these three, give error
  {
    cout << toggleError;
    parsed = false;
    res.success = false;
    res.message = toggleError;
  }

  n->getParam("/STATE", STATE); // Retrieve the state once more
  if (parsed) // Return true and give a positive response message
  {
    res.success = true;
    res.message = "\n-- Successfully set mode to " + STATE;
    cout << "\n-- Service call has toggled mode to " << STATE << "\n";
  }

  return true;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Set_AUTO:
//    Provides a menu for inputting the appropriate data for setting
//    the rover to autonomous mode, and then starting autonomous mode.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Set_AUTO()
{
  autopilot::calc_route srv; // Create service message

  string input; 
  cout << "\tLat+long or bearing+distance? [ll/BD/t]: ";
  cin >> input;

  if (input == "ll")
  {
    gps::Gps gps_coord; // Get some GPS coordinate

    cout << "\t\tLatitude: "; cin >> input; // Grab latitude from user
    gps_coord.latitude = stof(input.c_str());

    cout << "\t\tLongitude: "; cin >> input; // Grab longitude from user
    gps_coord.longitude = stof(input.c_str());

    srv.request.latlng = true;
    srv.request.destination = gps_coord;
  }
  else if (input == "BD")
  {
    cout << "\t\tBearing (deg N+): "; cin >> input; 
    double bearing = stof(input.c_str());

    cout << "\t\tDistance (m): "; cin >> input; // Grab longitude from user
    double distance = stof(input.c_str());

    srv.request.latlng = false; // Using bearing+distance method
    srv.request.bearing = bearing;
    srv.request.distance = distance;
  }
  else if (input == "t")
  {
    gps::Gps gps_coord; // Get some GPS coordinate

    cout << "\t\tUsing latitude  " << test_lat <<  ",\n";
    cout << "\t\t      longitude " << test_long << ".\n"; 

    gps_coord.latitude = test_lat;
    gps_coord.longitude = test_long;

    srv.request.latlng = true;
    srv.request.destination = gps_coord;
  }
  else if (input == back) 
  {
    cout << cancel;
    return false;
  }
  else 
  {
    cout << parseError;
    return false;
  }

  cout << "\tEnable LIDAR? [y/N]: ";
  cin >> input;

  if (input == "y")      srv.request.disable_lidar = 0;
  else if (input == "N") srv.request.disable_lidar = 1;
  else 
  {
    cout << parseError;
    return false;
  }

  cout << "\tRelease Glen? [y/N]: ";
  cin >> input;

  if (input == "y")      srv.request.glen_enabled = 1;
  else if (input == "N") srv.request.glen_enabled = 0;
  else 
  {
    cout << parseError;
    return false;
  }

  cout << "\tCommence AUTO mode now? [y/N]: ";
  cin >> input;
  
  if (input == "y")
  {
    bool res = start_auto_client.call(srv); // Did the service call succeed?
    ROS_INFO_STREAM("-- Result of start auto service call was " << res);
    return res;
  }
  else if (input == "N" || input == back)
  {
    cout << cancel;
    return false;
  }
  else 
  {
    cout << parseError;
    return false;
  }
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Set_RETRIEVE:
//    Provides a menu for inputting the appropriate data for setting
//    a destination for the interface to display for extreme retrieval.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Set_RETRIEVE()
{
  autopilot::set_dest srv; // Create service message

  string input; 

  gps::Gps gps_coord; // Get some GPS coordinate
  
  cout << "\t\tLatitude: "; cin >> input; // Grab latitude from user
  gps_coord.latitude = stof(input.c_str());

  cout << "\t\tLongitude: "; cin >> input; // Grab longitude from user
  gps_coord.longitude = stof(input.c_str());

  srv.request.destination = gps_coord;
 
  cout << "\tCommence RETRIEVE mode now? [y/N]: ";
  cin >> input;
  
  if (input == "y")
  {
    bool res = set_dest_client.call(srv); // Did the service call succeed?
    return res;
  }
  else if (input == "N" || input == back)
  {
    cout << cancel;
    return false;
  }
  else 
  {
    cout << parseError;
    return false;
  }
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// SigintHandler:
//    Overrides the default ROS sigint handler for Ctrl+C.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void SigintHandler(int sig)
{
  spinner_t->interrupt();
  spinner_t->join(); 

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Spinner:
//    Thread to continually spin, checking for service calls.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void Spinner()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {  
    boost::this_thread::interruption_point();

    ros::spinOnce();
    loop_rate.sleep();
  }
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main:
//    Setup and main menu.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{
  ros::init(argc, argv, "stateman");
  n = new ros::NodeHandle("/");

  // Override the default ros sigint handler.
  signal(SIGINT, SigintHandler);

  start_auto_client = 
    n->serviceClient<autopilot::calc_route>("/Start_Auto");

  set_dest_client = 
    n->serviceClient<autopilot::set_dest>("/Set_Dest");

  ros::ServiceServer service = 
    n->advertiseService("Toggle_Mode", Toggle_Mode);

  // Possible states are STANDBY, DRIVE, ARM, DRILL and AUTO, set default
  n->setParam("/STATE", "STANDBY");

  n->getParam("/stateman/test_lat", test_lat);
  n->getParam("/stateman/test_long", test_long);
  test_lat = -37.9106944;
  test_long = 145.1355000;

  parseError = "-- Failed to parse command.\n";
  toggleError = "\n-- Can't toggle state in STANDBY or AUTO modes.\n";
  cancel = "-- Cancelled action.\n";
  back = "x";

  boost::thread spin_t{Spinner}; // Start Spinner thread
  spinner_t = &spin_t;

  cout << "-- Welcome to the Nova Rover State Manager!";
  while (ros::ok())
  {     
    //string STATE; n->getParam("/STATE", STATE);    
    //cout << "** Current mode is " << STATE << "**\n";   

    string input; 
    cout << "\nSet mode: "; cin >> input; // Get user input to set state

    bool success = false; // Menu entry succeeded
    bool parsed = true; // Able to parse command

    // Run menu functions
    if      (input == "STANDBY" ) success = true;
    else if (input == "DRIVE"   ) success = true;
    else if (input == "RETRIEVE") success = Set_RETRIEVE();
    else if (input == "ARM"     ) success = true;
    else if (input == "DRILL"   ) success = true;
    else if (input == "AUTO"    ) success = Set_AUTO();
    else if (input == back) 
    {
      parsed = false;
      cout << "-- Shutting down.\n";
      SigintHandler(0);
    }
    else 
    {
      parsed = false;
      cout << parseError;
    }

    if (success) // Set state
    {
      n->setParam("/STATE", input);
      cout << "-- Set mode to " << input << "\n";
    }
    else if (parsed)
    {
      cout << "-- Unable to set mode to " << input << "\n";
    }

    ros::spinOnce();
  }

  return 0;
}
