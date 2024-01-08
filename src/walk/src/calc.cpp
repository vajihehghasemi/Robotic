#include "calc.h"

calc::calc()
{

}



double calc::saturate(double a, double min, double max){
  if(a<min){//ROS_INFO("subceeding!");
    qWarning()<<"subceeding! mag:"<<a<<"  min:"<<min;
    return min;}
  else if(a>max){//ROS_INFO("exceeding!");
    qWarning()<<"exceeding! mag:"<<a<<"  max:"<<max;
    return max;}
  else{return a;}
}

double calc::saturate(QString str, double a, double min, double max){
  if(a<min){//ROS_INFO("subceeding!");
    qWarning()<<str<<" subceeding! mag:"<<a<<"  min:"<<min;
    return min;}
  else if(a>max){//ROS_INFO("exceeding!");
    qWarning()<<str<<" exceeding! mag:"<<a<<"  max:"<<max;
    return max;}
  else{return a;}
}

double calc::d2r(double d){
  return d*M_PI/180;
}
void calc::matrix_view(MatrixXd M){

  for (int i = 0; i <M.rows() ; ++i) {
    QString str;
    for (int j = 0; j <M.cols() ; ++j) {
      str+=QString::number(M(i,j));
      str+="   ";
    }
    qDebug()<<str;
  }
  qDebug()<<"";
}

void calc::matrix_view(VectorXd M){
  QString str;
  for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
  qDebug()<<str;
  qDebug()<<"";
}
double calc::move_rest_back(double max,double t_local,double T_start ,double T_move,double T_rest,double T_back){
  double c3=(10*max)/pow(T_move,3);
  double c4=-(15*max)/pow(T_move,4);
  double c5=(6*max)/pow(T_move,5);
  double c3_r=(10*max)/pow(T_back,3);
  double c4_r=-(15*max)/pow(T_back,4);
  double c5_r=(6*max)/pow(T_back,5);
  double T_end=T_start+T_move+T_rest+T_back;
  double theta=0;
  if(t_local<T_start){theta=0;}
  else if (t_local<T_start+T_move){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
  else if (t_local<T_start+T_move+T_rest){theta=max;}
  else if (t_local<T_start+T_move+T_rest+T_back){theta=c3_r*pow(T_end-t_local,3)+c4_r*pow(T_end-t_local,4)+c5_r*pow(T_end-t_local,5);}
  return theta;
}

double calc::move2pose(double max,double t_local,double T_start ,double T_end){
  double T_move=T_end-T_start;
  double c3=(10*max)/pow(T_move,3);
  double c4=-(15*max)/pow(T_move,4);
  double c5=(6*max)/pow(T_move,5);
  double theta=0;
  if(t_local<T_start){theta=0;}
  else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
  else{theta=max;}
  return theta;
}

double calc::move2pose_diff(double max,double t_local,double T_start ,double T_end){
  double T_move=T_end-T_start;
  double c3=(10*max)/pow(T_move,3);
  double c4=-(15*max)/pow(T_move,4);
  double c5=(6*max)/pow(T_move,5);
  double theta=0;
  if(t_local<T_start){theta=0;}
  else if (t_local<T_end){theta=c3*pow(t_local-T_start,2)*3+c4*pow(t_local-T_start,3)*4+c5*pow(t_local-T_start,4)*5;}
  else{theta=0;}
  return theta;
}

double calc::move2zero(double theta,double t,double T_home){
  double c3=10/pow(T_home,3);
  double c4=-15/pow(T_home,4);
  double c5=6/pow(T_home,5);
  double theta0;
  if(t<=0){return theta;}
  if(t>=T_home){return 0;}
  double dt=0.005;
  if(fabs(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5))>1e-6){
    theta0=theta/(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5));
    // qDebug()<<theta0;
    return theta0*(1-c3*pow(t,3)-c4*pow(t,4)-c5*pow(t,5));
  }


}


MatrixXd calc::rot(int axis , double q ,int dim){
  if (dim==3){
    MatrixXd R(3,3);
    if (axis==1){
      R<<1,0,0,
          0,cos(q),-sin(q),
          0,sin(q),cos(q);
    }

    if (axis==2){
      R<<cos(q),0,sin(q),
          0,1,0 ,
          -sin(q),0,cos(q);
    }

    if (axis==3){
      R<<cos(q),-sin(q),0,
          sin(q),cos(q),0,
          0,0,1;
    }
    return R;
  }

  if(dim==4){
    MatrixXd R(4,4);
    if (axis==1){
      R<<1,0,0,0,
          0,cos(q),-sin(q),0,
          0,sin(q),cos(q),0,
          0,0,0,1;
    }

    if (axis==2){
      R<<cos(q),0,sin(q),0,
          0,1,0,0,
          -sin(q),0,cos(q),0,
          0,0,0,1;
    }

    if (axis==3){
      R<<cos(q),-sin(q),0,0,
          sin(q),cos(q),0,0,
          0,0,1,0,
          0,0,0,1;
    }
    return R;
  }

}

MatrixXd calc::trans(int axis, double d){
  MatrixXd H(4,4);
  H=MatrixXd::Identity(4,4);
  H(axis-1,3)=d;
  return H;
}

MatrixXd calc::trans(Vector3d d){
  MatrixXd H(4,4);
  H=MatrixXd::Identity(4,4);
  H.block(0,3,3,1)=d;
  return H;
}

double calc::distance(VectorXd V1,VectorXd V2){
  double d;
  d=sqrt(pow(V1(0)-V2(0),2)+pow(V1(1)-V2(1),2)+pow(V1(2)-V2(2),2));
  return d;
}

double calc::distance(double x1, double y1,double x2, double y2){
  double d;
  d=sqrt(pow(x1-x2,2)+pow(y1-y2,2));
  return d;
}

double calc::distance(double x1, double y1,double z1,double x2, double y2, double z2){
  double d;
  d=sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2));
  return d;
}


void calc::numplot(double num,double min,double max){
  //⬛

  QString str;
  int l=100;
  int n=int((num-min)/(max-min)*l);
  if (num<min){n=0;}
  if (num>max){n=100;}
  str+=QString::number(min);
  str+="|";
  if (n<=l/2){
    for (int i = 0; i < n; ++i) {
      str+=" ";
    }
    for (int i = 0; i < l/2-n; ++i) {
      str+="|";

    }
    str+="|";
    for (int i = 0; i < l/2; ++i) {
      str+=" ";
    }
  }
  else {
    for (int i = 0; i < l/2; ++i) {
      str+=" ";
    }
    for (int i = 0; i < n-l/2; ++i) {
      str+="|";

    }
    str+="|";
    for (int i = 0; i < l-n; ++i) {
      str+=" ";
    }

  }

  str+="|";
  str+=QString::number(max);
  str+="=>";str+=QString::number(num);
  qDebug()<<str;
  qDebug()<<"";


}


double calc::norm(VectorXd V){
  double s=0;
  for (int i = 0; i < V.rows(); ++i) {
    s+=V(i)*V(i);
  }
  return sqrt(s);
}

double calc::Velocity(double _L,double _t, double _T){

  double _V=30*_L/pow(_T,3)*pow(_t,2)-60*_L/pow(_T,4)*pow(_t,3)+30*_L/pow(_T,5)*pow(_t,4);
  return _V;
}

double calc::Position(double _L,double _t, double _T){
  double _P=10*_L/pow(_T,3)*pow(_t,3)-15*_L/pow(_T,4)*pow(_t,4)+6*_L/pow(_T,5)*pow(_t,5);
  return _P;
}




double calc::polynomial(double t,double t1,double t2,double x1,double  xd1,double xdd1,double x2,double xd2,double xdd2){
  double a[6];
  double x=0;
  if(t<=t2 && t>=t1){
  a[0]=(2*pow(t1,5)*x2 - 2*pow(t2,5)*x1 + 10*t1*pow(t2,4)*x1 - 10*pow(t1,4)*t2*x2 + 2*t1*pow(t2,5)*xd1 - 2*pow(t1,5)*t2*xd2 - 20*pow(t1,2)*pow(t2,3)*x1 + 20*pow(t1,3)*pow(t2,2)*x2 - 10*pow(t1,2)*pow(t2,4)*xd1 + 8*pow(t1,3)*pow(t2,3)*xd1 - 8*pow(t1,3)*pow(t2,3)*xd2 + 10*pow(t1,4)*pow(t2,2)*xd2 - pow(t1,2)*pow(t2,5)*xdd1 + 2*pow(t1,3)*pow(t2,4)*xdd1 - pow(t1,4)*pow(t2,3)*xdd1 + pow(t1,3)*pow(t2,4)*xdd2 - 2*pow(t1,4)*pow(t2,3)*xdd2 + pow(t1,5)*pow(t2,2)*xdd2)/(2*pow((t1 - t2),5));
  a[1]=(2*pow(t1,5)*xd2 - 2*pow(t2,5)*xd1 + 10*t1*pow(t2,4)*xd1 - 10*pow(t1,4)*t2*xd2 + 2*t1*pow(t2,5)*xdd1 - 2*pow(t1,5)*t2*xdd2 + 60*pow(t1,2)*pow(t2,2)*x1 - 60*pow(t1,2)*pow(t2,2)*x2 + 16*pow(t1,2)*pow(t2,3)*xd1 - 24*pow(t1,3)*pow(t2,2)*xd1 + 24*pow(t1,2)*pow(t2,3)*xd2 - 16*pow(t1,3)*pow(t2,2)*xd2 - pow(t1,2)*pow(t2,4)*xdd1 - 4*pow(t1,3)*pow(t2,3)*xdd1 + 3*pow(t1,4)*pow(t2,2)*xdd1 - 3*pow(t1,2)*pow(t2,4)*xdd2 + 4*pow(t1,3)*pow(t2,3)*xdd2 + pow(t1,4)*pow(t2,2)*xdd2)/(2*pow((t1 - t2),5));
  a[2]=(pow(t1,5)*xdd2 - pow(t2,5)*xdd1 - 60*t1*pow(t2,2)*x1 - 60*pow(t1,2)*t2*x1 + 60*t1*pow(t2,2)*x2 + 60*pow(t1,2)*t2*x2 - 36*t1*pow(t2,3)*xd1 + 24*pow(t1,3)*t2*xd1 - 24*t1*pow(t2,3)*xd2 + 36*pow(t1,3)*t2*xd2 - 4*t1*pow(t2,4)*xdd1 - 3*pow(t1,4)*t2*xdd1 + 3*t1*pow(t2,4)*xdd2 + 4*pow(t1,4)*t2*xdd2 + 12*pow(t1,2)*pow(t2,2)*xd1 - 12*pow(t1,2)*pow(t2,2)*xd2 + 8*pow(t1,2)*pow(t2,3)*xdd1 - 8*pow(t1,3)*pow(t2,2)*xdd2)/(2*pow((t1 - t2),5));
  a[3]=(20*pow(t1,2)*x1 - 20*pow(t1,2)*x2 + 20*pow(t2,2)*x1 - 20*pow(t2,2)*x2 - 8*pow(t1,3)*xd1 - 12*pow(t1,3)*xd2 + 12*pow(t2,3)*xd1 + 8*pow(t2,3)*xd2 + pow(t1,4)*xdd1 - 3*pow(t1,4)*xdd2 + 3*pow(t2,4)*xdd1 - pow(t2,4)*xdd2 + 28*t1*pow(t2,2)*xd1 - 32*pow(t1,2)*t2*xd1 + 32*t1*pow(t2,2)*xd2 - 28*pow(t1,2)*t2*xd2 + 4*pow(t1,3)*t2*xdd1 - 4*t1*pow(t2,3)*xdd2 - 8*pow(t1,2)*pow(t2,2)*xdd1 + 8*pow(t1,2)*pow(t2,2)*xdd2 + 80*t1*t2*x1 - 80*t1*t2*x2)/(2*pow((t1 - t2),5));
  a[4]=-(30*t1*x1 - 30*t1*x2 + 30*t2*x1 - 30*t2*x2 - 14*pow(t1,2)*xd1 - 16*pow(t1,2)*xd2 + 16*pow(t2,2)*xd1 + 14*pow(t2,2)*xd2 + 2*pow(t1,3)*xdd1 - 3*pow(t1,3)*xdd2 + 3*pow(t2,3)*xdd1 - 2*pow(t2,3)*xdd2 - 4*t1*pow(t2,2)*xdd1 - pow(t1,2)*t2*xdd1 + t1*pow(t2,2)*xdd2 + 4*pow(t1,2)*t2*xdd2 - 2*t1*t2*xd1 + 2*t1*t2*xd2)/(2*pow((t1 - t2),5));
  a[5]=(12*x1 - 12*x2 - 6*t1*xd1 - 6*t1*xd2 + 6*t2*xd1 + 6*t2*xd2 + pow(t1,2)*xdd1 - pow(t1,2)*xdd2 + pow(t2,2)*xdd1 - pow(t2,2)*xdd2 - 2*t1*t2*xdd1 + 2*t1*t2*xdd2)/(2*pow((t1 - t2),5));


  x=a[0]+a[1]*t+a[2]*pow(t,2)+a[3]*pow(t,3)+a[4]*pow(t,4)+a[5]*pow(t,5);
      }

      return x;
}


MatrixXd calc::quater2rot(double w,double x,double y, double z){
    MatrixXd R(3,3);
    R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
    return R;
}

//*****quaternion to euler params in ankle
double calc::quaternion2euler_pitch(double q0,double q1,double q2,double q3){
    double R11,R32,R33,R31,theta;
    R31=2*(q1*q3-q0*q2);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    theta=atan2(-R31,sqrt(R32*R32+R33*R33));
    return theta;
}

double calc::quaternion2euler_roll(double q0,double q1,double q2,double q3){
    double phi,R33,R32;
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    phi=atan2(R32,R33);
    return phi;
}


