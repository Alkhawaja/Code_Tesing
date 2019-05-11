#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/FluidPressure.h"
#include <math.h> 
//subscribers
ros::Subscriber sub_scan;
ros::Subscriber sub_scann;


//Publishers
ros::Publisher pub_p;
ros::Publisher pub_v;
ros::Publisher pub_a;
ros::Publisher pub_P;
ros::Publisher pub_Q;
double z_acco = 0.0;
double x_acco = 0.0;
double y_acco = 0.0;
double x_acc = 0.0;
double y_acc = 0.0;
double z_acc = 0.0;
double x_acc_p = 0.0;
double y_acc_p = 0.0;
double z_acc_p = 0.0;
double x_vel = 0.0;
double y_vel = 0.0;
double z_vel = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double z_pos = 0.0;
double x_delta = 0.0;
double y_delta = 0.0;
double z_delta = 0.0;
double P0=101325;
double P;
double roll;
double pitch;
double yaw;
double Alt=0;
double Alt0=0;
double sqw;
double sqx;
double sqy;
double sqz;
double jerk0;
double jerk1;
double rotxrad;
double rotyrad;
double rotzrad;
double samplePeriod = 0.33390625;
int t=0;
int count=-1;
double remove(double value){
value=value*10;
int int_value=value;
return int_value/10.0;
}
double sec1;
double sec3;

double sec4;

double z_acc_2;
double z_acc_3;
double z_acc_4;
double z_acc_1;
double x_acc_2;
double x_acc_3;
double x_acc_4;
double x_acc_1;
double y_acc_2;
double y_acc_3;
double y_acc_4;
double y_acc_1;
void callback_pressure(const sensor_msgs::FluidPressure &pressure){
	P=pressure.fluid_pressure;
if (t==0){
Alt0=44330*(1-pow((P/P0),(1/5.255)));
t=1;}
else
Alt=44330*(1-pow((P/P0),(1/5.255)))-Alt0;
	geometry_msgs::Point alt;
	alt.z=Alt;
	pub_P.publish(alt);

}

void callback_imu(const sensor_msgs::Imu &posee){/*
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * posee.orientation.w * posee.orientation.x + posee.orientation.y * posee.orientation.z;
	double cosr_cosp = +1.0 - 2.0 * posee.orientation.x* posee.orientation.x+ posee.orientation.y* posee.orientation.y;
	roll = atan2(sinr_cosp, cosr_cosp)*180/M_PI;

	// pitch (y-axis rotation)
	double sinp = -2.0 * posee.orientation.w* posee.orientation.y- posee.orientation.z* posee.orientation.x;
	
		pitch = asin(sinp)*180/M_PI;

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * posee.orientation.w* posee.orientation.z+ posee.orientation.x* posee.orientation.y;
	double cosy_cosp = +1.0 - 2.0 * posee.orientation.y* posee.orientation.y+ posee.orientation.z* posee.orientation.z;  
	yaw = atan2(siny_cosp, cosy_cosp)*180/M_PI;*/

	sqw = posee.orientation.w * posee.orientation.w;
	sqx = posee.orientation.x * posee.orientation.x;
	sqy = posee.orientation.y * posee.orientation.y;
	sqz = posee.orientation.z * posee.orientation.z;
	
	rotxrad = (double)atan2l(2.0 * ( posee.orientation.y * posee.orientation.z + posee.orientation.x * posee.orientation.w ) , ( -sqx - sqy + sqz + sqw ));
	rotyrad = (double)asinl(-2.0 * ( posee.orientation.x * posee.orientation.z - posee.orientation.y * posee.orientation.w ));
	rotzrad = (double)atan2l(2.0 * ( posee.orientation.x * posee.orientation.y + posee.orientation.z * posee.orientation.w ) , (  sqx - sqy - sqz + sqw ));
	

//	ROS_INFO("sec1 %f",sec1);

if (count==-1){
z_acco =posee.linear_acceleration.z;
x_acco =posee.linear_acceleration.x;
y_acco =posee.linear_acceleration.y;
count=0;	}
/*if (count==0){
z_acc_1 = (posee.linear_acceleration.z-z_acco);
x_acc_1 = (posee.linear_acceleration.x-x_acco);
y_acc_1 = (posee.linear_acceleration.y-y_acco);
ROS_INFO("0");

count=1;}
else if (count==1){
z_acc_2 = (posee.linear_acceleration.z-z_acco);
x_acc_2 = (posee.linear_acceleration.x-x_acco);
y_acc_2 = (posee.linear_acceleration.y-y_acco);
ROS_INFO("1");

count=2;
}
else if (count==2){
z_acc_3 = (posee.linear_acceleration.z-z_acco);
x_acc_3 = (posee.linear_acceleration.x-x_acco);
y_acc_3 = (posee.linear_acceleration.y-y_acco);
ROS_INFO("2");

count=3;
}
else {
z_acc_4 = (posee.linear_acceleration.z-z_acco);
x_acc_4 = (posee.linear_acceleration.x-x_acco);
y_acc_4 = (posee.linear_acceleration.y-y_acco);
ROS_INFO("3");
*/if (count==0){
z_acc_1 = (posee.linear_acceleration.z-z_acco)*cos(rotyrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotxrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotxrad);
x_acc_1 = (posee.linear_acceleration.x-x_acco)*cos(rotyrad)/2+(posee.linear_acceleration.x-x_acco)*cos(rotzrad)/2+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotzrad);
y_acc_1 = (posee.linear_acceleration.y-y_acco)*cos(rotxrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotzrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotzrad)+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotxrad);
ROS_INFO("0");

count=1;}
else if (count==1){
z_acc_2 = (posee.linear_acceleration.z-z_acco)*cos(rotyrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotxrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotxrad);
x_acc_2 = (posee.linear_acceleration.x-x_acco)*cos(rotyrad)/2+(posee.linear_acceleration.x-x_acco)*cos(rotzrad)/2+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotzrad);
y_acc_2 = (posee.linear_acceleration.y-y_acco)*cos(rotxrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotzrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotzrad)+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotxrad);
ROS_INFO("1");

count=2;
}
else if (count==2){
z_acc_3 = (posee.linear_acceleration.z-z_acco)*cos(rotyrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotxrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotxrad);
x_acc_3 = (posee.linear_acceleration.x-x_acco)*cos(rotyrad)/2+(posee.linear_acceleration.x-x_acco)*cos(rotzrad)/2+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotzrad);
y_acc_3 = (posee.linear_acceleration.y-y_acco)*cos(rotxrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotzrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotzrad)+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotxrad);
ROS_INFO("2");

count=3;
}
else {
z_acc_4 = (posee.linear_acceleration.z-z_acco)*cos(rotyrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotxrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotxrad);
x_acc_4 = (posee.linear_acceleration.x-x_acco)*cos(rotyrad)/2+(posee.linear_acceleration.x-x_acco)*cos(rotzrad)/2+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotyrad)+(posee.linear_acceleration.y-y_acco)*cos(M_PI/2-rotzrad);
y_acc_4 = (posee.linear_acceleration.y-y_acco)*cos(rotxrad)/2+(posee.linear_acceleration.z-z_acco)*cos(rotzrad)/2+(posee.linear_acceleration.x-x_acco)*cos(M_PI/2-rotzrad)+(posee.linear_acceleration.z-z_acco)*cos(M_PI/2-rotxrad);
ROS_INFO("3");

count=0;
jerk0=(z_acc_2-z_acc_1)/samplePeriod;
jerk1=(z_acc_4-z_acc_3)/samplePeriod;
z_acc=(jerk1-jerk0)*samplePeriod/2;
jerk0=(x_acc_2-x_acc_1)/samplePeriod;
jerk1=(x_acc_4-x_acc_3)/samplePeriod;
x_acc=(jerk1-jerk0)*samplePeriod/2;
jerk0=(y_acc_2-y_acc_1)/samplePeriod;
jerk1=(y_acc_4-y_acc_3)/samplePeriod;
y_acc=(jerk1-jerk0)*samplePeriod/2;
geometry_msgs::Vector3Stamped msg;

//x_acc = posee.linear_acceleration.x;
//y_acc = posee.linear_acceleration.y;
//z_acc = posee.linear_acceleration.z-9.7;
x_vel = (x_acc)/2*samplePeriod+x_vel;
y_vel = (y_acc)/2*samplePeriod+y_vel;
z_vel = (z_acc)/2*samplePeriod+z_vel;
x_delta=remove(x_vel*samplePeriod);
y_delta=remove(y_vel*samplePeriod);
z_delta=remove(z_vel*samplePeriod);
x_pos = x_pos + x_delta;
y_pos = y_pos + y_delta; 
z_pos = z_pos + z_delta;
msg.header.stamp=ros::Time::now();
msg.vector.x=x_vel;
msg.vector.y=y_vel;
msg.vector.z=z_vel;
pub_v.publish(msg);

msg.vector.x=remove(x_acc);
msg.vector.y=remove(y_acc);
msg.vector.z=remove(z_acc);
pub_a.publish(msg);
msg.vector.x=(rotxrad*180/M_PI);
msg.vector.y=(rotyrad*180/M_PI);
msg.vector.z=(rotzrad*180/M_PI);
pub_Q.publish(msg);
x_acc_p = x_acc;
y_acc_p = y_acc;
z_acc_p = z_acc;
double sec2 =ros::Time::now().toSec();
//ROS_INFO("sec2 %f",sec2);
sec4 =ros::Time::now().toSec();
if (sec4-sec3>1){
	msg.vector.x=x_pos;
msg.vector.y=y_pos;
msg.vector.z=z_pos;
pub_p.publish(msg);
sec3=sec4;}
double samplePeriod=sec2-sec1;
if (samplePeriod>5)
	{	x_vel=0;
		z_vel=0;
		y_vel=0;
		sec1=sec2;	ROS_INFO("secs %f",samplePeriod);}

	}
	}




int main(int argc, char **argv){

  ros::init(argc, argv, "helloo");
  ros::NodeHandle nh;
  ROS_INFO("Running...");
//Publishers
  pub_p = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/fposition", 10);
  pub_v = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/velocity", 100);
  pub_a = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/acc", 100);
  pub_P = nh.advertise<geometry_msgs::Point>("/imu/pressure", 100);
  pub_Q = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/yaw", 100);


//Subscribers
sub_scan = nh.subscribe("/imu/data", 1, callback_imu);
sub_scann = nh.subscribe("/pressure", 1, callback_pressure);
sec1 =ros::Time::now().toSec();
sec3 =ros::Time::now().toSec();

  
//Code
  while(ros::ok()){
  ros::spin();}
}