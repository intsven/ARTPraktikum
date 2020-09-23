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

#include "gio_pf_class.c"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#include <sstream>

ofstream giofile;

double gio_u = 0.0;
double gio_omega = 0.0;
double gio_vleft = 0.0;
double gio_vright = 0.0;
int    drive_a_path = 0;


/*----------------------------------------------------------------*/
/*   Geschwindigkeiten und Positionsaenderungen berechnen         */
/*----------------------------------------------------------------*/
int PredictRobotBehaviour(double leftspeed, double rightspeed, 
                          double betaCoorTrans, double elapsed_time, 
                          double *dbeta, double *backDX, double *backDY)
{
  double epsilon = 0.000001;
  double rad_abstand = 280.0 / 1000.0;
  double radius, alpha = 0.0, dx, dy;

  if (fabs(leftspeed - rightspeed) < epsilon) {
    if (fabs(leftspeed) < epsilon) {
	 radius = 0.0;
	 dx = 0.0;
	 dy = 0.0;
    }
    else {
	 dx = 0.0;
	 dy = elapsed_time * (leftspeed + rightspeed) * 0.5;
    }
  }
  else {
    radius = rad_abstand * 0.5 * (leftspeed + rightspeed) /
	 (rightspeed - leftspeed);

    alpha = elapsed_time * (rightspeed - leftspeed) / rad_abstand;
    dx =  radius * (cos(alpha) - 1.0);
    dy =  radius * sin(alpha);
  }

  *dbeta = alpha;
    
  /* Koordinaten transformation */
  *backDX = (dx * cos(betaCoorTrans)) + (dy * sin(betaCoorTrans));
  *backDY = (-dx * sin(betaCoorTrans)) + (dy * cos(betaCoorTrans));

  return(0);
} 


int main(int argc, char **argv)
{
  /* #####################
   *  Prediction of robot
   * #################### */
  double leftspeed, rightspeed, v_diff;
  double dx = 0.0, dy = 0.0, dbeta;
  double scale_factor;
  double elapsed_time = 0.01; // 10 ms
  double epsilon = 0.01;
  double x = 0, y = 0, theta = -1.51;

  double u, omega;
  double u_max = 1.0;

  ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time;

  ros::Rate loop_rate(10);

  
  CGioController *gio_control = new CGioController(); // object and also init function
  if (!gio_control->getPathFromFile("quadrat.dat"))
    cout<<"ERROR: Can not open GioPath File\n";
  else
    drive_a_path = 1;
  
  gio_control->setCurrentVelocity(u_max);
  gio_control->setAxisLength(280.0 / 1000.0);

  // debugging
  giofile.open("pos_quadrat.dat");

  int count = 0;

  //loop
  while  (drive_a_path && ros::ok()) {
    //    gio_control->setPose(x * 0.001, y_from_encoder*0.001,theta_from_encoder);
    gio_control->setPose(x, y, theta);
    // get trajectory
    if (gio_control->getNextState(gio_u, gio_omega, leftspeed, rightspeed, 1)==0) {
	     cout<<"finish";
	     drive_a_path = 0;
    }
    
    // v_diff = gio_omega * gio_u / M_PI;
    leftspeed = (float) (gio_u + gio_omega * (280.0 / 1000.0) * 0.5); // - fabs(v_diff) - v_diff); 
    rightspeed = (float) (gio_u - gio_omega * (280.0 / 1000.0) * 0.5); // fabs(v_diff) + v_diff); 

    scale_factor = 1.0;
    if (fabs(leftspeed) > u_max) scale_factor = fabs(u_max / leftspeed);
    if (fabs(rightspeed) > u_max) scale_factor = fabs(u_max / rightspeed);
    leftspeed *= scale_factor;
    rightspeed *= scale_factor;

		double motor_scale = 1;


    
    // SET SPEED HERE =====================================
    /*
    set_wheel_speed2(v_l_soll, v_r_soll,
				 v_l_ist, v_r_ist,
				 omega, Get_mtime_diff(9), AntiWindup);
    */
    
    /* ######################################
	*  Implementation of the robot simulator
	* ###################################### */
    PredictRobotBehaviour(leftspeed * motor_scale, rightspeed * motor_scale, theta, elapsed_time, &dbeta, &dx, &dy);  
    x += dx;
    y += dy;
    theta += dbeta;
    //gio_control->getRoboterPose(leftspeed, rightspeed, x, y, theta);
  
    giofile << gio_u << " " << gio_omega << " "
		  << x << " " << y << " " << theta << " "
		  << leftspeed << " " << rightspeed << endl;
    /*
    cout    << gio_u << " " << gio_omega << " "
		  << x << " " << y << " " << theta << " "
		  << leftspeed << " " << rightspeed << endl;
    */
    cout.flush();
    giofile.flush();

		current_time = ros::Time::now();

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-theta +1.51);

		//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

		std_msgs::String msg;
		nav_msgs::Odometry odom;



		odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0;

		
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

		chatter_pub.publish(msg);
		odom_pub.publish(odom);

    ros::spinOnce();

    loop_rate.sleep();
		++count;
  }
  giofile.close();
  giofile.clear();
  return 0;

}
