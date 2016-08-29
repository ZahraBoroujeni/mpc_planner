#ifndef CONFIG_CAR_PLAN_H_
#define CONFIG_CAR_PLAN_H_


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <fub_mpc_planner/paramConfig.h>
#include "NLF.h"
#include "BoundConstraint.h"
#include "NonLinearInequality.h"
#include "CompoundConstraint.h"
#include "OptNIPS.h"
#include <string.h>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision


using NEWMAT::ColumnVector;
using NEWMAT::Matrix;
using NEWMAT::SymmetricMatrix;

using namespace OPTPP;
using namespace NEWMAT;


class fub_mpc_planner
{
private:
  
  dynamic_reconfigure::Server<fub_mpc_planner::paramConfig> server;
  dynamic_reconfigure::Server<fub_mpc_planner::paramConfig>::CallbackType f;
  void init_state(int ndim, ColumnVector& x);
  void system_dynamic(int mode, int ndim, const ColumnVector& x, double& fx, ColumnVector& gx, SymmetricMatrix& Hx, int& result);
  void system_constrains(int mode, int ndim, const ColumnVector& x,ColumnVector& cx, Matrix& cgx,OptppArray<SymmetricMatrix>& cHx, int& result);

  float vx,vy;

  float x,y,y_reference;
  //contrains
  float vx_min,vx_max,vy_min,vy_max;
  float ax_min,ax_max,ay_min,ay_max;
  float left_boundry, right_boundry;
  float vehicle_width;
  float delta_t;

public:
  void config_callback(fub_mpc_planner::paramConfig &config, uint32_t level);
  fub_mpc_planner(ros::NodeHandle nh);
  ~fub_mpc_planner(){}

};





#endif
