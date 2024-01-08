#ifndef CALC_H
#define CALC_H
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Eigen/QuadProg.h"
#include "Eigen/testQPsolvers.hpp"
#include "Eigen/eiquadprog.hpp"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<math.h>
#include<qdebug.h>
#include<termios.h>
#include <iostream>
#include <cstdlib>
#include <cstring>


class calc
{
public:
    calc();
    double saturate(double a, double min, double max);
    double d2r(double d);
    void matrix_view(MatrixXd M);
    void matrix_view(VectorXd M);
    MatrixXd trans(int axis, double d);
    MatrixXd trans(Vector3d d);
    double distance(VectorXd V1, VectorXd V2);
    double distance(double x1, double y1,double z1,double x2, double y2, double z2);
    double distance(double x1, double y1,double x2, double y2);
    void numplot(double num, double min, double max);
    double norm(VectorXd V);
    double Velocity(double _L, double _t, double _T);
    double Position(double _L, double _t, double _T);
    MatrixXd rot(int axis, double q, int dim);
    int getch();
    double move2zero(double theta, double t, double T_home);
    double move2pose_diff(double max, double t_local, double T_start, double T_end);
    double move2pose(double max, double t_local, double T_start, double T_end);
    double move_rest_back(double max, double t_local, double T_start, double T_move, double T_rest, double T_back);
    double saturate(QString str, double a, double min, double max);
    double polynomial(double t, double t1, double t2, double x1, double xd1, double xdd1, double x2, double xd2, double xdd2);
    MatrixXd quater2rot(double w, double x, double y, double z);
    double quaternion2euler_pitch(double q0, double q1, double q2, double q3);
    double quaternion2euler_roll(double q0, double q1, double q2, double q3);
};

#endif // CALC_H
