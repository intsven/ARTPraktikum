#include <iostream>
using std::cerr;
using std::endl;
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using std::max;
using std::min;

#include "gio_path.cpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "volksbot/vels.h"
#include "volksbot/velocities.h"
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>

#include <sstream>

ofstream giofile;

double gio_u = 0.0;
double gio_omega = 0.0;
double gio_vleft = 0.0;
double gio_vright = 0.0;
int    drive_a_path = 0;

double x = 0, y = 0, theta = 0; //-1.51;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;

	theta = tf::getYaw(msg->pose.pose.orientation);
	
	//x = (x * cos(theta)) + (y * sin(theta));
  //y = (-x * sin(theta)) + (y * cos(theta));
	ROS_INFO("Pose-> x: [%f], y: [%f], theta: [%f]", x, y, theta);
}


int main(int argc, char **argv)
{
  double leftspeed, rightspeed, v_diff;
  double scale_factor;


  double u, omega;
  double u_max = 1.0;

  ros::init(argc, argv, "robot_control");
	ros::NodeHandle n;

	ros::Publisher vels_pub = n.advertise<volksbot::vels>("Vel", 100);
	ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);

  ros::Rate loop_rate(5);

  
  CGioController *gio_control = new CGioController(); // object and also init function
  if (!gio_control->getPathFromFile("path.dat"))
    cout<<"ERROR: Can not open GioPath File\n";
  else
    drive_a_path = 1;
  
  gio_control->setCurrentVelocity(u_max);
  gio_control->setAxisLength(0.485);

  // debugging
  giofile.open("pos_odom.dat");

  int count = 0;

  //loop
  while  (drive_a_path && ros::ok()) {
    //    gio_control->setPose(x * 0.001, y_from_encoder*0.001,theta_from_encoder);
    gio_control->setPose(x, y, theta);
    // get trajectory
    if (gio_control->getNextState(gio_u, gio_omega, leftspeed, rightspeed, 0)==0) {
	     cout<<"finish";
	     drive_a_path = 0;
    }

    scale_factor = 1.0;
    if (fabs(leftspeed) > u_max) scale_factor = fabs(u_max / leftspeed);
    if (fabs(rightspeed) > u_max) scale_factor = fabs(u_max / rightspeed);
    leftspeed *= scale_factor;
    rightspeed *= scale_factor;
  
    giofile << gio_u << " " << gio_omega << " "
		  << x << " " << y << " " << theta << " "
		  << leftspeed << " " << rightspeed << endl;

    cout.flush();
    giofile.flush();
		
		double motor_scale = -20;
		volksbot::vels velocity;
		velocity.left = leftspeed * motor_scale;
		velocity.right = rightspeed * motor_scale;
		vels_pub.publish(velocity);

    ros::spinOnce();

    loop_rate.sleep();
		++count;
  }
	volksbot::vels velocity;
	velocity.left = 0;
	velocity.right = 0;
	vels_pub.publish(velocity);

  giofile.close();
  giofile.clear();
  return 0;

}
