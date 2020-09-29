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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>

#include <sstream>

ofstream giofile;

double gio_u = 0.0;
double gio_omega = 0.0;
double gio_vleft = 0.0;
double gio_vright = 0.0;
int    drive_a_path = 0;

double x_mess = 0, y_mess = 0, theta_mess = 0;
double x = 0, y = 0, theta = 0; //-1.51;
double x_start = 0, y_start = 0, theta_start = 0;
int start_counter = 0;


void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

  x_mess = msg->pose.pose.position.x;
	y_mess = msg->pose.pose.position.y;

	theta_mess = tf::getYaw(msg->pose.pose.orientation);

  if(start_counter < 5) {
    x_start = x_mess;
    y_start = y_mess;
    theta_start = theta_mess;
    ++start_counter;
  }
  theta = theta_mess - theta_start;
  //if(theta < -M_PI / 2)
  //  theta += M_PI * 2;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x_mess - x_start, y_mess - y_start, 0.0));
  transform.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -theta_start));
  x = transform.getOrigin().getX();
  y = transform.getOrigin().getY();
  
  //x = cos(-theta_start) * (x_mess - x_start) + sin(-theta_start) * (y_mess - y_start);
  //y = -sin(-theta_start) * (x_mess - x_start) + cos(-theta_start) * (y_mess - y_start);

  //x = x_mess - x_start;
  //y = y_mess - y_start;

  ROS_INFO("Position-> x: [%f], y: [%f], theta: [%f]", x,y, theta);
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
	ros::Subscriber sub = n.subscribe("amcl_pose", 1000, amclCallback);

  ros::Duration(5.0).sleep();

  ros::Rate loop_rate(5);

  
  CGioController *gio_control = new CGioController(); // object and also init function
  if (!gio_control->getPathFromFile("path.dat"))
    cout<<"ERROR: Can not open GioPath File\n";
  else
    drive_a_path = 1;
  
  gio_control->setCurrentVelocity(u_max);
  gio_control->setAxisLength(0.485);

  // debugging
  giofile.open("pos_amcl3.dat");

  int count = 0;

  tf::TransformListener listener;

  //loop
  while  (drive_a_path && ros::ok()) {
    /*
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    x_mess = transform.getOrigin().x();
    y_mess = transform.getOrigin().y();

    theta_mess = tf::getYaw(transform.getRotation());

    if(count < 10) {
      x_start = x_mess;
      y_start = y_mess;

      theta_start = theta_mess;
    }

    //x = transform.getOrigin().x() - x_start;
    //y = transform.getOrigin().y() - y_start;

    theta = theta_mess - theta_start;

    x = cos(-theta_start) * (x_mess - x_start) + sin(-theta_start) * (y_mess - y_start);
    y = -sin(-theta_start) * (x_mess - x_start) + cos(-theta_start) * (y_mess - y_start);
    */
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

    if(leftspeed < 0)
      leftspeed = 0;
    if(rightspeed < 0) 
      rightspeed = 0;
  
    giofile << gio_u << " " << gio_omega << " "
		  << x << " " << y << " " << theta << " "
		  << leftspeed << " " << rightspeed << endl;

    cout.flush();
    giofile.flush();
		
		double motor_scale = -10;

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
