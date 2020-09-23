/*
  code fragment / example how to use this class:

  
#include "gio_pf_class.h"

  /debugging
  ofstream giofile;

  double gio_u = 0.0;
  double gio_omega = 0.0;
  double gio_vleft = 0.0;
  double gio_vright = 0.0;
int    drive_a_path = 0;


  CGioController *gio_control = new CGioController(); // object and also init function
  if (!gio_control->getPathFromFile("giopath.dat")) cout<<"ERROR: Can not open GioPath File\n";
else drive_a_path = 1;
  gio_control->setCurrentVelocity(Get_MaxSpeed());
  gio_control->setAxisLength(Get_Axis_Length()/1000.0);
  // debugging
  giofile.open("/ramdisk/giofile.dat");


//loop
if (drive_a_path) {
    gio_control->setPose(x_from_encoder*0.001,y_from_encoder*0.001,theta_from_encoder);
// get trajectory
    if (gio_control->getNextState(gio_u,gio_omega,v_l_soll,v_r_soll, 2)==0) {
	     cout<<"finish";
	     drive_a_path = 0;
    }
    giofile << gio_u<<" "<< gio_omega<<" "<<x_from_encoder<<" "<<y_from_encoder<<" "<<theta_from_encoder<<" "<<v_l_soll<<" "<<v_r_soll<<"\n";
}
else {
         v_diff = omega * u / M_PI;
	 v_l_soll = (float) (u - fabs(v_diff) - v_diff); 
	 v_r_soll = (float) (u - fabs(v_diff) + v_diff); 
}   
 
	 // SET SPEED HERE =====================================
	 set_wheel_speed2(v_l_soll, v_r_soll,
				   v_l_ist, v_r_ist,
				   omega, Get_mtime_diff(9), AntiWindup);



// after loop;

 giofile.flush();
  giofile.close();
  giofile.clear();
*/

#include "gio_pf_class.h"


void CGioController::InitDefault(){
  this->loop_exit    = 0;
  this->AXIS_LENGTH  = 0.485;
  this->t_adaptation = 1.0; // 0.795;
  this->Vm           = 1; // 3;
  this->d_y          = 0.001;
  this->d_th         = 0.0034;
  this->kr_max       = 2/AXIS_LENGTH;
  this->e_time       = 0.01;
  this->u0           = Vm/2.0;
  this->epsilon      = 0.001;
  this->a            = 1.2;

  this->path         = new CCurve();

  this->setPose(0.0,0.0,0.0);
  this->setLocalSystem(0.0);
}

void  CGioController::setLocalSystem(double ang){
  ex[0] = cos(ang); // cos(M_PI_2 - ang); //geaendert von Niko; war "cos(ang)"
  ex[1] = sin(ang); // sin(M_PI_2 - ang); //geaendert von Niko; war "sin(ang)"
  ey[0] = -sin(ang); // cos(-ang);  //geaendert von Niko; war "-sin(ang)"
  ey[1] = cos(ang); //sin(-ang);  //geaendert von Niko; war "cos(ang)"
}

double  CGioController::H_case_1(double y, double th, double u, double alpha, double *gama){
  double tmp, tmp1, tmp2, res;

  tmp1 = th*th;
  tmp2 = (1+2*alpha)*(1+2*alpha);
  tmp = tmp1*tmp2;
  tmp1 = kr_max*kr_max;
  res = tmp1/tmp;
  *gama = 2*alpha*u*sqrt(res);
  return res;
};

double  CGioController::H_case_2(double y, double th, double u, double alpha, double *gama){
  double tmp1, tmp, tmp2, res;
  tmp = -(th*th)/(y*y);

  tmp1 = tmp*tmp;

  tmp2 = kr_max*kr_max;
  tmp2*=4;
  tmp2/=y*y;
  tmp2/=(1+2*alpha)*(1+2*alpha);

  tmp2+= tmp1;

  tmp1 = sqrt(tmp2);

  res = 0.5*(tmp+tmp1);

  *gama = 2*alpha*u*sqrt(res);
  return res;
};

double  CGioController::Compute_W(double y, double th, double a, double u, int *err){
  double res, h, gama, t1, t2;

  if(y==0.0){
    if(fabs(th)>= d_th){
	 h = H_case_1(y, th, u, a, &gama);
	 *err = 0;
    }else{
	 h = H_case_1(y, d_th, u, a, &gama);
	 *err = 0;
    }
  }else
    if(fabs(y)>=d_y){
	 h = H_case_2(y, d_th, u, a, &gama);
	 *err = 0;
    }else{
	 if(fabs(th)<d_th){
	   h = H_case_2(d_y, d_th, u, a, &gama);
	   *err = 0;
	 }else{
	   h = H_case_2(d_y,th,u,a,&gama);
	   *err = 0;
	 }
    }

  if(fabs(th)>=d_th){
    t1 = sin(th)/th;
    t2 = h*u*y*t1;
    res = -t2-gama*th;
  }else{
    if(fabs(th)<=d_th && fabs(th)>=d_th_0){
	 res = -h*u*y-gama*th;
    }else{
	 t2 = h*u*y;
	 res = -t2;
    }
  }

  return res;
}

CGioController::CGioController(){
  this->InitDefault();
};

CGioController::~CGioController(){
  delete path;
}
    
void CGioController::reinitialize(){
  InitDefault();
}

 void CGioController::setAxisLength(double val){
  if(fabs(val)>0){
    this->AXIS_LENGTH = fabs(val);
    this->kr_max = 2/AXIS_LENGTH;
  }
}

 double CGioController::getAxisLength(){
  return this->AXIS_LENGTH;
}

 void CGioController::setTurningAdaptation(double val){
  if(fabs(val)>0)
    this->t_adaptation = fabs(val);
}

 double CGioController::getTurningAdaptation(){
  return this->t_adaptation;
}

 void CGioController::setVelocityConstraint(double val){
  if(fabs(val)>=0) {
    this->Vm = fabs(val);
    this->u0 = Vm/2.0;
  }
}

 double CGioController::getVelocityConstraint(){
  return this->Vm;
}

 void CGioController::setDistanceConstraint(double val){
  if(fabs(val)>=0)
    this->d_y = fabs(val);
}

 double CGioController::getDistanceConstraint(){
  return this->d_y;
}

 void CGioController::setAngleConstraint(double val, int mess){
  if(fabs(val)>=0){
    if(mess)
	 this->d_th = fabs(val);
    else
	 this->d_th = fabs(val)*GradToRadian;
  }
}

 double CGioController::getAngleConstraint(int mess){
  if(mess)
    return this->d_th;
  else
    return this->d_th*GradToRadian;
}

 double CGioController::getMaximalCurvatur(){
  return this->kr_max;
}

 void CGioController::setCurrentVelocity(double val, int abs){
  if(abs)
    this->u0 = val;
  else
    this->u0 = (this->u0>val)?(val+this->u0):(this->u0+val);
  this->Vm = u0*2.0;
}

 double CGioController::getCurrentVelocity(){
  return this->u0;
}

 void CGioController::setControlTimingValue(double val){
  if(fabs(val)>0)
    this->e_time = fabs(val);
}

 double CGioController::getControlTimingValue(){
  return this->e_time;
}

 void CGioController::setEpsilon(double val){
  if(fabs(val)>0)
    this->epsilon = fabs(val);
}

 double CGioController::getEpsilon(){
  return this->epsilon;
}

 void CGioController::setPose(double x, double y, double phi){
  this->x0 = x;
  this->y0 = y;
  this->phi0 = phi;
  NormalizeAngle(phi0);
  //setLocalSystem(phi0);
}

 void CGioController::getPose(double &x, double &y, double &ph){
  x = this->x0;
  y = this->y0;
  ph = this->phi0;
}

int CGioController::getPathFromFile(const char* fname){
  int res = path->LoadFromFile(fname);
  if(res){
    path->initTraversal();
    this->loop_exit = path->getNext();
  }
  return res;
}

int CGioController::writePathToFile(const char* fname){
  return path->WriteToFile(fname);
}

int CGioController::canDetermineRoboterPosition(int looped){
  int exit;
  int check_prev;

  exit       = 0;
  check_prev = 0;

  while(this->loop_exit != 0 && !exit) {
    if(path->pointInn(x0,y0)) {
	 if(path->getDistanceToEnd(x0, y0) > 0.1) {
	   exit = 1;
	   continue;
	 }
    }/*else{
	  check_prev = 1;
	  }
	*/
    /*	    if(check_prev){
		    path->getPrev(looped);
		    if(path->pointInn(x0,y0)) {
		    exit = 1;
		    continue;
		    }
		    check_prev = 0;
		    path->getNext(looped);
		    }
    */
    this->loop_exit = path->getNext(looped);
  }
	
  return exit;
}

int CGioController::getRoboterPose(double vleft, double vright, double &x, double &y, double &phi){
  double dx, dy, ddx, ddy, vel, w = 0.0;
  if(path->getCount() == 0){
    x = this->x0;
    y = this->y0;
    phi = this->phi0;
    return 0;
  }

  if(fabs(vright-vleft)<epsilon){
    if(fabs(vleft)<epsilon){
	 vel = 0.0;
	 dx = 0.0;
	 dy = 0.0;
    }else{
	 dx = this->e_time*(vright+vleft)/2.0;
	 dy = 0;
    }
  }else{
    if(vright<vleft)
	 vel = 0.5*(vright+vleft)*this->e_time;
    else
	 vel = 0.5*(vleft+vright)*this->e_time;
    if(vright<-vleft)
	 w = t_adaptation*(vright-vleft)/AXIS_LENGTH*e_time;
    else
	 w = t_adaptation*(-vleft+vright)/AXIS_LENGTH*e_time;
    dx = vel*sin(w);
    dy = vel*cos(w);
  }

  ddx = dx*ex[0]+dy*ey[0];
  ddy = dx*ex[1]+dy*ey[1];

  x0 += ddx;
  y0 += ddy;
  phi0 += w;
  // NormalizeAngle(phi0);

  setLocalSystem(phi0);

  x = this->x0;
  y = this->y0;
  phi = this->phi0;
  return 1;
}

int CGioController::getNextState(double &u, double &w, double &vleft, double &vright, int looped){
  double l, phic, tmpw, vdiff;
  int err;
  if(!canDetermineRoboterPosition(looped)){
    u = 0;
    w = 0;
    vleft = 0;
    vright = 0;
    return 0;
  }
	
  l = path->getDistance(x0,y0);
  if( path->Evaluate(x0,y0) > 0.00005 )
    l = -l;
  
  //	phic = phi0 - M_PI_2;
  //	NormalizeAngle(phic);
  phic = (M_PI_2 - phi0) - path->getAng();
  NormalizeAngle(phic);

  cout << "l = " << l << " phi0 = "<< phi0*RadianToGrad <<" phic = "<< phic*RadianToGrad << endl;

  u = this->u0;

  w = Compute_W(l,phic,this->a,u,&err);
  
  tmpw = this->Vm/this->AXIS_LENGTH;
  
  if(fabs(w)>tmpw)
    w = (w<0)?-tmpw:tmpw;
  //	cout<<"w = "<<w<<" rad/s"<<endl<<endl;

  vright = u + AXIS_LENGTH * w * 0.5;
  vleft  = u - AXIS_LENGTH * w * 0.5;

  return 1;
}
