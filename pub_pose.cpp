#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <stdlib.h>   
#define REPLY_SIZE 8
#define TIMEOUT 1000
   
   // This example opens the serial port and sends a request 'R' at 1Hz and waits for a reply.
   int main(int argc, char** argv)
   {
      ros::init(argc, argv, "example_node");
       ros::NodeHandle n;
   
      serial::Serial device("/dev/ttyUSB1", 57600, serial::Timeout::simpleTimeout(TIMEOUT));

      
      if (device.isOpen())
      ROS_INFO("The serial port is opened.");
  
      ros::Rate r(50);
      while(ros::ok())
      {
          // Send 'R' over the serial port
	  //device.write("R");
	  device.write("m");
	  std::string receiver = device.read(8);
	  ROS_INFO("Receiver: ");
	  ROS_INFO_STREAM(receiver);
         ros::spinOnce();
         r.sleep();
      }
  }