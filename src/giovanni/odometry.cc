//
// STANDALONE TEST g++ -o odometrie odometrie.cc -lm -Wall
//
 
/***************************************************************************
 * INCLUDED HEADERS
 ***************************************************************************/
#include <iostream>
#include <fstream>
using std::ofstream;
using std::cout;
using std::endl;
#include <cmath>
#include <cstdlib>


/***************************************************************************
 * CONSTANTS & MACROS
 ***************************************************************************/  
#define FALSE                  0
#define TRUE                   1
#define INVPIMUL180            57.29578
#define SQR(a)                 ((a)*(a))          

const double rad_abstand = 0.362;          // in Meter
const double epsilon = 0.000001;
const double elapsed_time = 0.01;          // 10 ms


/*----------------------------------------------------------------*/
/*   Geschwindigkeiten und Positionsaenderungen berechnen         */
/*----------------------------------------------------------------*/
int PredictRobotBehaviour(const double leftspeed, const double rightspeed, 
                          const double theta,
					 const double elapsed_time, 
                          double &dtheta, double &dx, double &dy)
{
  double v = 0.0;

  if (fabs(rightspeed) < epsilon) {
    if (fabs(leftspeed) < epsilon) {
	 dtheta = 0.0;
	 dx = 0.0;
	 dy = 0.0;
	 return(2);
    }
  }

  v = 0.5 * (leftspeed + rightspeed);
  dtheta = elapsed_time * (rightspeed - leftspeed) / rad_abstand;
  
  dx = v * cos(theta + dtheta * 0.5) * elapsed_time;
  dy = v * sin(theta + dtheta * 0.5) * elapsed_time;
  
  return(0);
} 

int main (int argc, char **argv)
{
  double x = 1, y = 1, theta = 0.0 ;
  double u, omega;
  double u_max = 0.35;

  /* #####################
   *  Prediction of robot
   * #################### */
  double leftspeed, rightspeed;
  double dx = 0.0, dy = 0.0, dtheta;
  double scale_factor;
  double epsilon = 0.01;
  int    i = 0, nr = 3000;

  ofstream out("pos.dat");
  
  do {
    //  ####################################
    //  Implementatierung des Regelgesetzes
    //  #################################### 
    //    CalcNextVelocities(u_max, x, y, theta, &u, &omega);

    u = 0.1;
    omega = 0.5;
    
    leftspeed  = (u - omega * rad_abstand * 0.5);
    rightspeed = (u + omega * rad_abstand * 0.5); 

    scale_factor = 1.0;
    if (fabs(leftspeed) > u_max) scale_factor = fabs(u_max / leftspeed);
    if (fabs(rightspeed) > u_max) scale_factor = fabs(u_max / rightspeed);
    leftspeed  *= scale_factor;
    rightspeed *= scale_factor;

    if (leftspeed  < 0) leftspeed  = 0.0;
    if (rightspeed < 0) rightspeed = 0.0;

    /* ######################################
	*  Implementation of the robot simulator
	* ###################################### */
    PredictRobotBehaviour(leftspeed, rightspeed,
					 theta,
					 elapsed_time,
					 dtheta, dx, dy);  

    x += dx;
    y += dy;
    theta += dtheta;
    
    out << i << " "
	   << x << " "
	   << y << " "
	   << theta << " "
	   << u << " "
	   << omega << " "
	   << leftspeed << " "
	   << rightspeed
	   << endl;
    
  } while (((fabs(x) > epsilon) || (fabs(y) > epsilon)) && (i++ < nr) );

  return 0;
}






