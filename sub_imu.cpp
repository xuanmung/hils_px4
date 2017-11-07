#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <iomanip>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <stdlib.h> 
#include <sstream>
#include <iostream>

#define TIMEOUT			1000
#define ROSRATE			400
#define PI			3.14159f
#define GRAVITY			9.81f
	
#define BAUDRATE		57600

#define k1Pos			1.39f
#define k2Pos			0.46f	
#define LEASH_LENGTH	1.5f	// [m]
#define	VEL_LIM		3.5f	// [m/s]
#define ROLL_LIM		0.25f	// [rad]
#define PITCH_LIM		0.25f	// [rad]
#define DEAD_LENGTH	0.06f

#define k1Alt					0.86f
#define k2Alt					28.1f
#define HOVER_THR			80.0f
#define HOVER_THR_MAP	0.43f	

#define MASS					1.780f

#define ALT_SP				2.5f	// [m]

#define f1x				0.34f
#define f1y				0.32f
#define ATTITUDE_ZERO_ZONE		100.0f	// [cdeg]
#define LIFT_ZERO_ZONE			1.0

#define LPF_WINDOW_SIZE			20
	
int								phy, theta, psy, p, q, r;
std::string							mBuffer;
std::ostringstream				mOss;
serial::Serial						mSerial; 
geometry_msgs::Quaternion	orient_quat;
geometry_msgs::Vector3		orient_euler, omega;

std::string							receiverBuff;
geometry_msgs::Vector3		spAtt, spPos, estPos, estVel;
double								alt_fb, vz;
int									fbRequestreceived = 0;
uint								count, posTaksMng;

double								Tx, Ty, fbFz, cmdRoll, cmdPitch;

int rad2cdeg(double rad){
 return (int) (rad * 180.0)/PI*100;
}

double cdeg2rad(double cdeg){
	return (cdeg/100.0 * PI / 180.0);
}

double rad2deg(double rad){
 return rad * 180.0 / PI;
}

double deg2rad(double deg){
 return deg * PI / 180.0;
}

double saturate(double input, double min, double max){
	double output;
	output = input > max ? max : (input < min ? min : input);
	return output;
}

geometry_msgs::Vector3 get_pos_PD(double x_c, double y_c, double x, double y, double vx, double vy){
			double dis_to_target = sqrt((x_c - x)*(x_c - x) + (y_c - y)*(y_c - y)); 
			double pos_ex, pos_ey; 
	
			if (dis_to_target >= LEASH_LENGTH){
				pos_ex = LEASH_LENGTH * (x_c - x) / dis_to_target;
				pos_ey = LEASH_LENGTH * (y_c - y) / dis_to_target; 
			}
			else{
				pos_ex = x_c - x;
				pos_ey = y_c - y;
			}
	
			if (pos_ex < DEAD_LENGTH && pos_ex > -DEAD_LENGTH) pos_ex = 0.0;
			if (pos_ey < DEAD_LENGTH && pos_ey > -DEAD_LENGTH) pos_ey = 0.0;
			
			double vel_cx	= pos_ex * k2Pos;
			double vel_cy	= pos_ey * k2Pos;
			double vel_c	= sqrt(vel_cx*vel_cx + vel_cy*vel_cy); 
		
			if (vel_c > VEL_LIM){
				vel_cx	= vel_cx * VEL_LIM / vel_c;
				vel_cy	= vel_cy * VEL_LIM / vel_c;
			}
			double vel_ex, vel_ey;
			vel_ex	= vel_cx - vx;
			vel_ey	= vel_cy - vy;
	
			geometry_msgs::Vector3 out;
			out.x = k1Pos * vel_ex;
			out.y = k1Pos * vel_ey;
		
			return out;
}

geometry_msgs::Vector3 get_commanded_att(double Tx, double Ty, double Tz, double yaw){
			double T = sqrt(Tz*Tz);//sqrt(Tx*Tx + Ty*Ty + Tz*Tz);
			geometry_msgs::Vector3 out;
			out.x = saturate((-Tx*sin(yaw) + Ty*cos(yaw)) / T, 
						-ROLL_LIM, ROLL_LIM);
				
			out.y = - saturate((Tx*cos(yaw) + Ty*sin(yaw)) / T, 
						-PITCH_LIM, PITCH_LIM);
				
			return out;
}
void localAltReceive(const geometry_msgs::PoseStamped& msg){
	alt_fb = msg.pose.position.z;
	//ROS_INFO_STREAM("\nAltitude = " << alt_fb);
}
void localVzReceive(const geometry_msgs::TwistStamped& msg){
	vz = msg.twist.linear.z;
	//ROS_INFO_STREAM("\nVel z = " << vz);
	//ROS_INFO_STREAM("\nVel x = " << msg.twist.linear.x << "\nVel y = " << msg.twist.linear.y);
}
//this is the function subscribing the IMU data
void imuMessageReceived(const sensor_msgs::Imu& msg){ 
	omega = msg.angular_velocity; 
	//geometry_msgs::Quaternion orient_quat =  msg.orientation; //uncomment this line if errors occur
	orient_quat =  msg.orientation; 
	//tf::Matrix3x3 m(orient_quat); 
	
	//double roll, pitch, yaw;
	//m.getRPY(roll, pitch, yaw);
	//tf::transform_datatypes mtf;
	
	orient_euler.z = - tf::getYaw(orient_quat) * 57.296; 
	//- 57.296 * atan2(2*(orient_quat.x*orient_quat.y + orient_quat.z*orient_quat.w),
				 //1 - 2*(orient_quat.y*orient_quat.y + orient_quat.z*orient_quat.z));
	orient_euler.z = orient_euler.z < -90.0 ? 360.0 + orient_euler.z + 90.0 : orient_euler.z + 90.0;
		 
	orient_euler.y = 57.296 * asin(2*(orient_quat.x*orient_quat.z - orient_quat.w*orient_quat.y));
	
	orient_euler.x = 180.0 - 57.296 * atan2(2*(orient_quat.x*orient_quat.w + orient_quat.y*orient_quat.z), 
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

/*
* This function is to convert an integer number, num, into a form of 
* normlized string with the length of normSize
*/
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
	ros::init(argc, argv, "hils_px4");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("mavros/imu/data", 10, 
		&imuMessageReceived);
		
	ros::Subscriber sub_alt= nh.subscribe("mavros/local_position/pose", 10, 
		&localAltReceive);
		
	ros::Subscriber sub_vz = nh.subscribe("mavros/local_position/velocity", 10, 
		&localVzReceive);
		
	ros::Publisher spAttPublisher = nh.advertise<geometry_msgs::PoseStamped>(
		"mavros/setpoint_attitude/attitude", 100);
		
		ros::Publisher spThrPublisher = nh.advertise<std_msgs::Float64>(
		"mavros/setpoint_attitude/att_throttle", 100);
		
	ros::Publisher estPosPublisher = nh.advertise<geometry_msgs::PoseStamped>(
		"mavros/vision_pose/pose", 100);
		
	ros::Publisher estVelPublisher = nh.advertise<geometry_msgs::Vector3Stamped>(
		"mavros/vision_speed/speed_vector", 100);
			
	ros::Publisher spPosPublisher = nh.advertise<geometry_msgs::PoseStamped>(
		"mavros/setpoint_position/local", 100);	
		
	//("/dev/ttyUSB1", 57600, serial::Timeout::simpleTimeout(TIMEOUT));
	int serialConnectionCount	= 1;
	int serialTimeOut		= 1000;
	int serialPortFound		= 0;
	//while (serialConnectionCount < serialTimeOut){
		//serialConnectionCount++;
		//while (serialPortFound ==0){		
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
	
		if (mSerial.isOpen()){
			ROS_INFO("The serial port is opened.");
			serialConnectionCount = serialTimeOut;
			serialPortFound = 1;
			}
		//else{
		//	if (serialConnectionCount == serialTimeOut)
		//	serialPortFound = 0;
		//	ROS_ERROR_STREAM("Serial port connection Timeout");
		//}
		
	//}
	
		
	ros::Rate rate(ROSRATE);
	
	count =			1;
	std_msgs::Float64	thrCmd;
	double			lamda = 0.46;
	
	double			ax_body, ay_body, az_body;
	double			ax_body_filtered = 0.0;
	double			ay_body_filtered = 0.0;
	double			az_body_filtered = 0.0;
	
	double			ax_body_previous = 0.0;
	double			ay_body_previous = 0.0;
	double			az_body_previous = 0.0;
	
	double			u_body = 0.0;
	double			v_body = 0.0;
	double			w_body = 0.0;
	
	double			xdot, ydot, zdot;
	
	double			xdot_previous = 0.0;
	double			ydot_previous = 0.0;
	double			zdot_previous = 0.0;
	
	double			pos_north_integral	= 0.0;
	double			pos_east_integral	= 0.0;
	double			pos_height_integral	= 0.0;
	
	double			dt; //delta-time constant;
	double			phy_rad, theta_rad, psy_rad;
	double			phy_rad_filtered	= 0.0;
	double			theta_rad_filtered	= 0.0;
	
	double			phy_rad_arr[LPF_WINDOW_SIZE], theta_rad_arr[LPF_WINDOW_SIZE], 
				ax_body_arr[LPF_WINDOW_SIZE], ay_body_arr[LPF_WINDOW_SIZE],
				az_body_arr[LPF_WINDOW_SIZE];
	for (int i = 0; i < LPF_WINDOW_SIZE; i++){
		phy_rad_arr[i]		= 0.0;
		theta_rad_arr[i]	= 0.0;
		ax_body_arr[i]		= 0.0;
		ay_body_arr[i]		= 0.0;
		az_body_arr[i]		= 0.0;
	}
	
	spPos.x		= 0.0;
	spPos.y		= 0.0;
	spPos.z		= 0.0;
	estPos.x	= 0.0;
	estPos.y	= 0.0;
	estPos.z	= 0.0;
	double t1	= 0;
	
	while(ros::ok())
	{
	  // Send 'R' over the serial port
	  //device.write("R");
	  if (ros::Time::now().toSec() - t1 > 2 
		&& serialPortFound == 0
		&& serialConnectionCount < serialTimeOut) {
		serialConnectionCount++;
		//ROS_INFO_STREAM("Time = " << ros::Time::now().toSec());
		 t1 = ros::Time::now().toSec();
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
	
		if (mSerial.isOpen()){
			ROS_INFO("The serial port is opened.");
			serialConnectionCount = serialTimeOut;
			serialPortFound = 1;
			}
		else{
			serialPortFound = 0;
			if (serialConnectionCount == serialTimeOut){
				ROS_ERROR_STREAM("Serial port connection Timeout");
			}
		}
	  }
	  
	  
	  //ROS_INFO(ros::Time::now().toSec());
		  std::string sendBuff("");
	  
		  //mSerial.write("a");
		  mOss << phy;
		  //ROS_INFO_STREAM("Phy = " << (std::string)mOss.str());
		  //mSerial.write((std::string)mOss.str());
		  //mSerial.write(norm_str(phy, 5));
		  sendBuff = (std::string)(sendBuff + "a" + norm_str(phy, 5));
		  mOss.str("");
		  mOss.clear();
	  
		  //mSerial.write("b");
		  mOss << theta;
		  //ROS_INFO_STREAM("Theta = " << (std::string)mOss.str());
		  //mSerial.write(norm_str(theta, 5));
		  sendBuff = (std::string)(sendBuff + "b" + norm_str(theta, 5));
		  mOss.str("");
		  mOss.clear();

		  //mSerial.write("c");
		  mOss << psy;
		  //ROS_INFO_STREAM("Psy = " << (std::string)mOss.str());
		  //mSerial.write(norm_str(psy, 5));
		  sendBuff = (std::string)(sendBuff + "c" + norm_str(psy, 5));
		  mOss.str("");
		  mOss.clear();
	  
		  //mSerial.write("p");
		  mOss << p;
		  //ROS_INFO_STREAM("p = " << (std::string)mOss.str());
		  //mSerial.write(norm_str(p, 5));
		  sendBuff = (std::string)(sendBuff + "p" + norm_str(p, 5));
		  mOss.str("");
		  mOss.clear();
	  
		  //mSerial.write("q");
		  mOss << q;
		  //ROS_INFO_STREAM("q = " << (std::string)mOss.str());
		  //mSerial.write(norm_str(q, 5));
		  sendBuff = (std::string)(sendBuff + "q" + norm_str(-q, 5));
		  mOss.str("");
		  mOss.clear();
	  
		  //mSerial.write("r");
		  mOss << r;
		  //ROS_INFO_STREAM("r = " << (std::string)mOss.str());
		  //mSerial.write(norm_str(r, 5));
		  sendBuff = (std::string)(sendBuff + "r" + norm_str(-r, 5));
		  mOss.str("");
		  mOss.clear();
		  //ROS_INFO(sendBuff);
	  //if (fbRequestreceived == 1){
	//	  mSerial.write(sendBuff);
	//	  fbRequestreceived = 0;
	  //}
	  
	  
	  // Tinh Pseudo Velocities va Positions
	  dt = 1.0 / ROSRATE; 
	  	  
	  //Ma tran quay R tu Body sang Earth 
	  double r11, r12, r13, r21, r22, r23, r31, r32, r33; 
	  
	  phy_rad	= deg2rad(orient_euler.x);
	  theta_rad	= deg2rad(orient_euler.y);
	  psy_rad	= deg2rad(orient_euler.z); 
	  
	  phy_rad_filtered	= phy_rad_filtered + (phy_rad - phy_rad_arr[0]) / (LPF_WINDOW_SIZE * 1.0);
	  theta_rad_filtered	= theta_rad_filtered + (theta_rad - theta_rad_arr[0]) / (LPF_WINDOW_SIZE * 1.0);
	  
	  for (int i = 0; i < LPF_WINDOW_SIZE -1; i++){
		phy_rad_arr[i]	= phy_rad_arr[i+1];
		theta_rad_arr[i]= theta_rad_arr[i+1];
	  }
	  
	  phy_rad_arr[LPF_WINDOW_SIZE -1]	= phy_rad;
	  theta_rad_arr[LPF_WINDOW_SIZE -1]	= theta_rad;
	  //ROS_INFO_STREAM("phy_rad_filtered =  " << phy_rad_filtered);
	  //ROS_INFO_STREAM("theta_rad_filtered =  " << theta_rad_filtered << "\n\n");
	  
	  ax_body = - 15.0*f1x * u_body - GRAVITY * sin(theta_rad) + omega.z * v_body - omega.y * w_body;
	  ay_body = - 15.0*f1y * v_body + GRAVITY * sin(phy_rad) * cos(theta_rad) + omega.x * w_body - omega.z * u_body;
	  //in the NED frame
	  if (pos_height_integral >= 0.0 && fbFz > -LIFT_ZERO_ZONE){ 
		az_body = 0.0;
	  }
	  else{
		az_body = (fbFz / (MASS * 1.0)) * cos(phy_rad) * cos(theta_rad) 
				+ omega.z * v_body - omega.y * w_body;
	  }
	  //az_body = (fbFz / (MASS * 1.0) + GRAVITY) * cos(phy_rad) * cos(theta_rad) + omega.z * v_body - omega.y * w_body;
	  
	  ax_body_filtered = ax_body_filtered + (ax_body - ax_body_arr[0]) / (LPF_WINDOW_SIZE * 1.0);
	  ay_body_filtered = ay_body_filtered + (ay_body - ay_body_arr[0]) / (LPF_WINDOW_SIZE * 1.0);
	  az_body_filtered = az_body_filtered + (az_body - az_body_arr[0]) / (LPF_WINDOW_SIZE * 1.0);
	  
	  for (int i = 0; i < LPF_WINDOW_SIZE -1; i++){
		ax_body_arr[i]	= ax_body_arr[i+1];
		ay_body_arr[i]	= ay_body_arr[i+1];
		az_body_arr[i]	= az_body_arr[i+1];
	  }
	  ax_body_arr[LPF_WINDOW_SIZE -1] = ax_body;
	  ay_body_arr[LPF_WINDOW_SIZE -1] = ay_body;
	  az_body_arr[LPF_WINDOW_SIZE -1] = az_body;
	  
	  //ROS_INFO_STREAM("Integral ax_body =  " << ax_body << "  ax_filtered = " << ax_body_filtered);
	  //ROS_INFO_STREAM("Integral ay_body =  " << ay_body << "  ay_filtered = " << ay_body_filtered << "\n\n");
	  ROS_INFO_STREAM("Fz =  " << fbFz);
	  ROS_INFO_STREAM("az_body =  " << az_body << "  az_filtered = " << az_body_filtered);
	  
	  u_body = u_body + (ax_body_filtered + ax_body_previous) * dt / 2.0;
	  v_body = v_body + (ay_body_filtered + ay_body_previous) * dt / 2.0;
	  w_body = w_body + (az_body_filtered + az_body_previous) * dt / 2.0;
	  //ROS_INFO_STREAM("dt =  " << dt);
	  //ROS_INFO_STREAM("Integral u_body =  " << u_body);
	  //ROS_INFO_STREAM("Integral v_body =  " << v_body << "\n\n");
	  
	  
	  ax_body_previous = ax_body_filtered;
	  ay_body_previous = ay_body_filtered;
	  az_body_previous = az_body_filtered;
	  
	  if (fabs(theta) <= ATTITUDE_ZERO_ZONE)	u_body = 0.0;
	  if (fabs(phy) <= ATTITUDE_ZERO_ZONE)		v_body = 0.0;	  
	  if (fabs(fbFz) <= LIFT_ZERO_ZONE)		w_body = 0.0;
	  r11 = cos(psy_rad) * cos(theta_rad);
	  r12 = cos(psy_rad) * sin(theta_rad) * sin(phy_rad) - sin(psy_rad) * cos(phy_rad);
	  r13 = cos(psy) * sin(theta_rad) * cos(phy_rad) + sin(psy_rad) * sin (phy_rad); 
	  
	  r21 = sin(psy_rad) * cos(theta_rad);
	  r22 = sin(psy_rad) * sin(theta_rad) * sin(phy_rad) + cos(psy_rad) * cos(phy_rad);
	  r23 = sin(psy_rad) * sin(theta_rad) * cos(phy_rad) - cos(psy_rad) * sin (phy_rad); 
	  
	  r31 = - sin(theta_rad);
	  r32 = cos(theta_rad) * sin(phy_rad); 
	  r33 = cos(theta_rad) * cos(phy_rad); 
	  
	  xdot = r11 * u_body + r12 * v_body + r13 * w_body;
	  ydot = r21 * u_body + r22 * v_body + r23 * w_body;
	  zdot = r31 * u_body + r32 * v_body + r33 * w_body;
	  
	  //ROS_INFO_STREAM("Integral Xdot =  " << xdot);
	  //ROS_INFO_STREAM("Integral Ydoy =  " << ydot << "\n\n");
	  
	  if (fabs(theta) > ATTITUDE_ZERO_ZONE){
		pos_north_integral	= pos_north_integral + (xdot + xdot_previous) * dt / 2.0;
	  }
	  if (fabs(phy) > ATTITUDE_ZERO_ZONE){
		pos_east_integral	= pos_east_integral + (ydot + ydot_previous) * dt / 2.0;
	  }
	  if (fabs(fbFz) > LIFT_ZERO_ZONE){
		pos_height_integral	= pos_height_integral + (zdot + zdot_previous) * dt / 2.0;
	  }
	  if (pos_height_integral > 0.0) pos_height_integral = 0.0;
	  
	  xdot_previous = xdot;
	  ydot_previous = ydot;
	  zdot_previous = zdot;
	  
	  ROS_INFO_STREAM("zdot =  " << zdot);
	  //ROS_INFO_STREAM("Integral X =  " << pos_north_integral);
	  //ROS_INFO_STREAM("Integral Y =  " << pos_east_integral << "\n\n");
	  ROS_INFO_STREAM("Integral z =  " << pos_height_integral << "\n\n");
	  
	  //receiving data sent from Matlab GUI (mGCS)
	  if (mSerial.available()){
		  
		  //ROS_INFO("Ting-ting!!!");
		  
		  std::stringstream ss; 
		  
		  std::string flag = mSerial.read(1); 
		  
		  if	((flag != (std::string)"a")	//roll
			&& (flag != (std::string)"b")	//pitch
			&& (flag != (std::string)"c")	//yaw
			&& (flag != (std::string)"l")	//commanded north
			&& (flag != (std::string)"i")	//commanded east
			&& (flag != (std::string)"d")	//commanded down
			&& (flag != (std::string)"x")	//estimated north
			&& (flag != (std::string)"y")	//estimated east
			&& (flag != (std::string)"h")	//estimated down
			&& (flag != (std::string)"v")	//estimated velocity
			&& (flag != (std::string)"f")	//Vertical force
			&& (flag != (std::string)"s")	//Data transfer request
			&& (flag != (std::string)"r"))	//Reset position control
			{
			//ROS_INFO_STREAM("Flag: " << flag);
		  }
		  else{
			//ROS_INFO_STREAM("Flag: " << flag);
			
			if (flag == (std::string)"a"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> spAtt.x;
				ss.str("");
				ss.clear();
				
				ROS_INFO_STREAM("Roll: " << spAtt.x);
				
				spAtt.x = cdeg2rad(spAtt.x);
			  }
	  
			  else if (flag == (std::string)"b"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> spAtt.y;
				ss.str("");
				ss.clear();
				
				ROS_INFO_STREAM("Pitch: " << spAtt.y);
				
				spAtt.y = cdeg2rad(-spAtt.y);
			  }
			  else if (flag == (std::string)"c"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> spAtt.z;
				ss.str("");
				ss.clear();
				
				ROS_INFO_STREAM("Yaw: " << spAtt.z);
				
				spAtt.z = spAtt.z > 36000.0 ? 36000.0 : 
					(spAtt.z > 18000.0 ? spAtt.z - 36000.0 : spAtt.z);
				spAtt.z = - spAtt.z + 9000.0;
				//spAtt.z = spAtt.z < -180.0 ? 
				//	-180.0 : (spAtt.z > 360.0 ? 360.0 : spAtt.z);
				//spAtt.z = spAtt.z < -90.0 ? 
				//	360.0 + spAtt.z + 90.0 : spAtt.z + 90.0; 
				spAtt.z = cdeg2rad(spAtt.z);
			  }
			  else if (flag == (std::string)"l"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> spPos.x;
				ss.str("");
				ss.clear();
				ROS_INFO_STREAM("North: " << spPos.x);
			  }
			  else if (flag == (std::string)"i"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> spPos.y;
				ss.str("");
				ss.clear();
				ROS_INFO_STREAM("East: " << spPos.y);
			  }
			  else if (flag == (std::string)"d"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> spPos.z;
				ss.str("");
				ss.clear();
				//ROS_INFO_STREAM("Down: " << spPos.z);
			  }
			  else if (flag == (std::string)"x"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> estPos.x;
				ss.str("");
				ss.clear();
				//ROS_INFO_STREAM("Pseudo X: " << estPos.x);
				//fbRequestreceived = 1;
			  }
			  else if (flag == (std::string)"y"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> estPos.y;
				ss.str("");
				ss.clear();
				//ROS_INFO_STREAM("Pseudo Y: " << estPos.y);
			  }
			  else if (flag == (std::string)"h"){
				receiverBuff = mSerial.read(6);
				ss << receiverBuff;
				ss >> estPos.z;
				ss.str("");
				ss.clear();
				//ROS_INFO_STREAM("Pseudo h: " << estPos.z);
			  }
			  else if (flag == (std::string)"v"){
				//ROS_INFO("TINGTING");
				std::string subFlag = mSerial.read(1);
				if (subFlag == (std::string)"n"){
					receiverBuff = mSerial.read(6);
					ss << receiverBuff;
					ss >> estVel.x;
					ss.str("");
					ss.clear();
					//ROS_INFO_STREAM("Vx: " << estVel.x);
				}
				else if (subFlag == (std::string)"i"){
					receiverBuff = mSerial.read(6);
					ss << receiverBuff;
					ss >> estVel.y;
					ss.str("");
					ss.clear();
					//ROS_INFO_STREAM("Vy: " << estVel.y);
				}
				else if (subFlag == (std::string)"d"){
					receiverBuff = mSerial.read(6);
					ss << receiverBuff;
					ss >> estVel.z;
					ss.str("");
					ss.clear();
					//ROS_INFO_STREAM("Vz: " << estVel.z);
				}
			  }
			  else if (flag == (std::string)"f"){
				std::string subFlag = mSerial.read(1);
				if (subFlag == (std::string)"z"){
					receiverBuff = mSerial.read(6);
					ss << receiverBuff; 
					ss >> fbFz;
					ss.str("");
					ss.clear();
					//ROS_INFO_STREAM("Fz: " << fbFz);
				}
			  }
			  else if (flag == (std::string)"s"){
				mSerial.write(sendBuff);
				//fbRequestreceived = 1;
				//ROS_INFO("TingTing! Request Received");
			  }
			  else if (flag == (std::string)"r"){
				pos_north_integral = 0.0;
				pos_east_integral = 0.0;
				xdot = 0.0;
				ydot = 0.0; 
				xdot_previous = 0.0;
				ydot_previous = 0.0; 
				u_body = 0.0;
				v_body = 0.0;
				ax_body = 0.0;
				ay_body = 0.0;
				ax_body_previous = 0.0;
				ay_body_previous = 0.0;
			  }
		  }
	  }
	  
	  tf::Quaternion		tfQuat, m1;
	  geometry_msgs::Quaternion	msgQuat;
	  
	  tfQuat.setRPY(spAtt.x, spAtt.y, spAtt.z);
	  //tfQuat.setRPY(PI/4.0, -PI/8.0, PI/2.0);
	  quaternionTFToMsg(tfQuat, msgQuat);
	  //quaternionMsgToTF(msgQuat, m1);
	  //double xx, yy, zz;
	  //tf::Matrix3x3(m1).getRPY(xx, yy, zz);
	  //ROS_INFO_STREAM("R: " << xx << " P: " << yy << " Y: " << zz);
	  
  	  //Publishing the setpoint attitude to the appropriate topic
	  geometry_msgs::PoseStamped spAttMsg;
	  spAttMsg.header.seq = count;
	  spAttMsg.header.stamp = ros::Time::now();
	  spAttMsg.header.frame_id = "1";
	  //spAttMsg.pose.position.x = spPos.x;
	  //spAttMsg.pose.position.y = spPos.y;
	  //spAttMsg.pose.position.z = spPos.z;
	  spAttMsg.pose.orientation = msgQuat;
	  //spAttMsg.pose.orientation.x = 0;
	  //spAttMsg.pose.orientation.y = 0;
	  //spAttMsg.pose.orientation.z = 0;
	  //spAttMsg.pose.orientation.w = 1;
	  
	  spAttPublisher.publish(spAttMsg);
	  
	  
	  /*
	  //ACTUAL ALTITUDE CONTROLLER:
	  double alt_err = (ALT_SP - alt_fb) > LEASH_LENGTH ? LEASH_LENGTH : 
		((ALT_SP - alt_fb) < -LEASH_LENGTH ? -LEASH_LENGTH : ALT_SP - alt_fb);
		 
	  double thrOut = ((ALT_SP - alt_fb) * k2Alt - vz) * k1Alt;
	  thrOut = thrOut < -80.0 ? -80.0 : (thrOut > 80.0 ? 80.0 : thrOut);
	  thrOut = thrOut * HOVER_THR_MAP / HOVER_THR + HOVER_THR_MAP;
	  //ROS_INFO_STREAM("\nThrottle: " << thrOut);  
	  
	  //Publishing att_throttle
	  //thrCmd.data = thrOut; //lamda;
	  thrCmd.data = lamda;
	  spThrPublisher.publish(thrCmd);
	  */
	  
	  //HIL ALTITUDE CONTROLLER
	  double hil_height	= -pos_height_integral; 
	  double hil_vz		= - zdot;
	  double alt_err = (ALT_SP - hil_height) > LEASH_LENGTH ? LEASH_LENGTH : 
		((ALT_SP - hil_height) < -LEASH_LENGTH ? -LEASH_LENGTH : ALT_SP - hil_height);
		 
	  double thrOut = ((ALT_SP - hil_height) * k2Alt - hil_vz) * k1Alt;
	  thrOut = thrOut < -80.0 ? -80.0 : (thrOut > 80.0 ? 80.0 : thrOut);
	  thrOut = thrOut * HOVER_THR_MAP / HOVER_THR + HOVER_THR_MAP;
	  //ROS_INFO_STREAM("\nThrottle: " << thrOut);  
	  
	  //Publishing att_throttle
	  thrCmd.data = thrOut; //lamda;
	  //thrCmd.data = lamda;
	  spThrPublisher.publish(thrCmd);
	  	  
	  	  
	  //////////////////////////////////
	  
	  //Publishing the estimated position to the appropriate topic
	  geometry_msgs::PoseStamped estPosMsg;
	  
	  estPosMsg.header.seq = count;
	  estPosMsg.header.stamp = ros::Time::now();
	  estPosMsg.header.frame_id = "Vision_Position";
	  estPosMsg.pose.position.x = 0;//estPos.x;
	  estPosMsg.pose.position.y = 0;//estPos.y;
	  estPosMsg.pose.position.z = estPos.z;
	  
	  geometry_msgs::Vector3 estAtt;
	  estAtt.x = orient_euler.x;
	  estAtt.y = - orient_euler.y;
	  estAtt.z = - orient_euler.z;// + PI/2.0;
	  tfQuat.setRPY(estAtt.x, estAtt.y, estAtt.z);
	  //tfQuat.setRPY(PI/4.0, -PI/8.0, PI/2.0);
	  quaternionTFToMsg(tfQuat, msgQuat);
	  
	  estPosMsg.pose.orientation = msgQuat;
	  //estPosMsg.pose.orientation.x = 0;
	  //estPosMsg.pose.orientation.y = 0;
	  //estPosMsg.pose.orientation.z = 0;
	  //estPosMsg.pose.orientation.w = 1;
	  
	  //ROS_INFO_STREAM("YAW FEEDBACK 1 = " << (- tf::getYaw(orient_quat) * 57.296));
	  //ROS_INFO_STREAM("YAW FEEDBACK 2 = " << (- 57.296 * atan2(2*(orient_quat.x*orient_quat.y + 
	//		orient_quat.z*orient_quat.w),
	//		1 - 2*(orient_quat.y*orient_quat.y + orient_quat.z*orient_quat.z))));
	  
	  //estPosPublisher.publish(estPosMsg);
	  
	  geometry_msgs::Vector3Stamped estVelMsg;
	  estVelMsg.header.seq = count;
	  estVelMsg.header.stamp = ros::Time::now();
	  estVelMsg.header.frame_id = "Vision_Speed";
	  
	  estVelMsg.vector.x = 0;//estVel.x;
	  estVelMsg.vector.y = 0;//estVel.y;
	  estVelMsg.vector.z = 0;//-estVel.z;
	  
	  //estVelPublisher.publish(estVelMsg);
	  
	  //Publishing the setpoint position to the appropriate topic
	  geometry_msgs::PoseStamped spPosMsg;
	  
	  spPosMsg.header.seq = count;
	  spPosMsg.header.stamp = ros::Time::now();
	  spPosMsg.header.frame_id = "Setpoint_Position";
	  
	  spPosMsg.pose.position.x = spPos.x;
	  spPosMsg.pose.position.y = spPos.y;
	  spPosMsg.pose.position.z = spPos.z;
	  //spPosMsg.pose.orientation = orient_quat;
	  tfQuat.setRPY(0.0, 0.0, PI/2);
	  //tfQuat.setRPY(PI/4.0, -PI/8.0, PI/2.0);
	  quaternionTFToMsg(tfQuat, msgQuat);
	  spPosMsg.pose.orientation = msgQuat;
	  
	  //spPosMsg.pose.orientation.x = 0;
	  //spPosMsg.pose.orientation.y = 0;
	  //spPosMsg.pose.orientation.z = 0;
	  //spPosMsg.pose.orientation.w = 1;
	  
	  //spPosPublisher.publish(spPosMsg);
	  
	  ///////////////////////
	  
	  posTaksMng++; 
	  	  
	  if (posTaksMng >= 3){
	  //geometry_msgs::Vector3 axy = get_pos_PD(spPos.x, spPos.y, 
	  //			estPos.x, estPos.y, estVel.x, estVel.y);
	  geometry_msgs::Vector3 axy = get_pos_PD(spPos.x, spPos.y, 
				pos_north_integral, pos_east_integral, xdot, ydot); 		
				
	  Tx = axy.x;// * MASS;
	  Ty = axy.y;// * MASS;
	  
	  //get attitude command (NED frame)
	  //geometry_msgs::Vector3 spRollPitch = get_commanded_att(Tx, Ty, fbFz, 
	 //			cdeg2rad(orient_euler.z*100)); 
	  geometry_msgs::Vector3 spRollPitch = get_commanded_att(Tx, Ty, fbFz, 
				0.0); 
				
	  //command attitude (ENU frame)			
	  spAtt.x = spRollPitch.x;
	  spAtt.y = -spRollPitch.y;
	  	  
	  posTaksMng = 0;
	  }
	 count++;
	 ros::spinOnce();
	 rate.sleep();
	}
      
	//ros::spin();
}