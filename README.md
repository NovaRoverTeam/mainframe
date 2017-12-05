# mainframe
Contains the software packages to run at the base station to control the rover.

## Controls (node)
Handles input from the user via physical controller devices and distributes the input into the ROS network.

### Publications
Topic:       **ctrl_data**<br />
Msg type:    mainframe/RawControl (custom)<br />
Description: Passes raw physical controller inputs to other nodes in the network.

## Overlord (node)
Integrates user input and passes commands to the rover.

### Subscriptions
Topic:       **ctrl_data**<br />
Msg type:    mainframe/RawControl (custom)<br />
Description: Passes raw physical controller inputs to other nodes in the network.

### Publications
Topic:       **hbeat**<br />
Msg type:    std_msgs/Empty<br />
Description: Sends periodic heartbeat to rover in order to determine whether or not we have lost comms.

Topic:       **cmd_data**<br />
Msg type:    rover/DriveCmd (custom)<br />
Description: Sends speed and steering commands to the rover.
