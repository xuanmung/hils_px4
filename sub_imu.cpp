#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
//#include <btQuaternion.h>
#include <tf/transform_datatypes.h>
#include <iomanip>

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <stdlib.h> 
#include <sstream>
#include <iostream>

#define TIMEOUT 1000
#define ROSRATE 100
#define PI	3.14159f

#define BAUDRATE 57600


int phy, theta, psy, p, q, r;
//std::vector<uint8_t> buffer;
std::string mBuffer;
std::ostringstream mOss;
serial::Serial mSerial; 

int rad2cdeg(double rad){
 return (int) (rad * 180.0)/PI*100;
}


void imuMessageReceived(const sensor_msgs::Imu& msg){ 
	geometry_msgs::Vector3 omega = msg.angular_velocity; 
	geometry_msgs::Quaternion orient_quat =  msg.orientation; 
	//tf::Matrix3x3 m(orient_quat); 
	
	//double roll, pitch, yaw;
	//m.getRPY(roll, pitch, yaw);
	//tf::transform_datatypes mtf;
	geometry_msgs::Vector3 orient_euler;
	orient_euler.z = - tf::getYaw(orient_quat) * 57.296; //- 57.296 * atan2(2*(orient_quat.x*orient_quat.y + orient_quat.z*orient_quat.w),
				 //1 - 2*(orient_quat.y*orient_quat.y + orient_quat.z*orient_quat.z));
	orient_euler.y = 57.296 * asin(2*(orient_quat.x*orient_quat.z - orient_quat.w*orient_quat.y));
	orient_euler.x = 180 - 57.296 * atan2(2*(orient_quat.x*orient_quat.w + orient_quat.y*orient_quat.z), 
				 1 - 2*(orient_quat.z*orient_quat.z + orient_quat.w*orient_quat.w));
				 
	orient_euler.x = orient_euler.x > 180.0 ? orient_euler.x - 360.0 : (orient_euler.x < -180 ? orient_euler.x + 360.0 : orient_euler.x);
	
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
	//<< "Attitude=(" << orient_euler.x << ", " << orient_euler.y
	//<< ", " << orient_euler.z << ") "); 
	
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
	//<< "Omega = (" << omega.x << ", " << omega.y
	//<< ", " << omega.z << ") \n"); 
	
	phy	= (int) (orient_euler.x * 100);
	theta	= (int) (orient_euler.y * 100);
	psy	= (int) (orient_euler.z * 100);
	p	= (int) (omega.x * 100);
	q	= (int) (omega.y * 100);
	r	= (int) (omega.z * 100);
	
	

	//mOss << phy;
	//buffer = (std::string)("123") + (std::string)("456");
		
}

//This function is to convert an integer number, num, into a form of normlized string with the length of normSize
std::string norm_str(int num, uint normSize){
	
	std::string out_str("");
	std::string sign("");
	uint num_cpy;
	
	if (num < 0){
	 sign = "-";
	 num_cpy = -num;
	}
	else{
	 sign = "+";
	 num_cpy = num;
	}
	
	std::ostringstream oss;
	oss << num_cpy;
	std::string num_str = oss.str();
	oss.str("");
	oss.clear();
	uint num_len = num_str.length();
	
	if (num_len < normSize){ 	
	 uint gap = normSize - num_len;
	 
	 for (int i = 1; i<= gap; i++){	  
	  out_str = (std::string) (out_str + (std::string)"0");
	 }
	 out_str = (std::string) (sign + out_str + num_str);
	}
	else out_str = (std::string) (sign + num_str);
	
	return out_str;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "subscribe_to_Imu");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("mavros/imu/data", 10, 
		&imuMessageReceived);
		
	//("/dev/ttyUSB1", 57600, serial::Timeout::simpleTimeout(TIMEOUT));
	try{
		mSerial.setPort("/dev/ttyUSB1");
		mSerial.setBaudrate(BAUDRATE);
		serial::Timeout mTo = serial::Timeout::simpleTimeout(TIMEOUT);
		mSerial.setTimeout(mTo);
		mSerial.open();
	}
	catch (serial::IOException& e) {
		ROS_ERROR_STREAM("Unable to open serial port");
	}
	
	if (mSerial.isOpen())
		ROS_INFO("The serial port is opened.");
	
		
	ros::Rate rate(ROSRATE);
	
	while(ros::ok())
	{
	  // Send 'R' over the serial port
	  //device.write("R");
	  
	  mSerial.write("a");
	  mOss << phy;
	  ROS_INFO_STREAM("Phy = " << (std::string)mOss.str());
	  //mSerial.write((std::string)mOss.str());
	  mSerial.write(norm_str(phy, 5));
	  mOss.str("");
	  mOss.clear();
	  
	  mSerial.write("b");
	  mOss << theta;
	  ROS_INFO_STREAM("Theta = " << (std::string)mOss.str());
	  mSerial.write(norm_str(theta, 5));
	  mOss.str("");
	  mOss.clear();

	  	  mSerial.write("c");
	  mOss << psy;
	  ROS_INFO_STREAM("Psy = " << (std::string)mOss.str());
	  mSerial.write(norm_str(psy, 5));
	  mOss.str("");
	  mOss.clear();
	  
	  mSerial.write("p");
	  mOss << p;
	  ROS_INFO_STREAM("p = " << (std::string)mOss.str());
	  mSerial.write(norm_str(p, 5));
	  mOss.str("");
	  mOss.clear();
	  
	  mSerial.write("q");
	  mOss << q;
	  ROS_INFO_STREAM("q = " << (std::string)mOss.str());
	  mSerial.write(norm_str(q, 5));
	  mOss.str("");
	  mOss.clear();
	  
	  mSerial.write("r");
	  mOss << r;
	  ROS_INFO_STREAM("r = " << (std::string)mOss.str());
	  mSerial.write(norm_str(r, 5));
	  mOss.str("");
	  mOss.clear();
	  
	  if (mSerial.available()){
		  std::string receiver = mSerial.read(8);
		  
		  ROS_INFO_STREAM("Receiver: " << receiver);
	  }
	  
	 ros::spinOnce();
	 rate.sleep();
	}
      
	//ros::spin();
}