#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <fub_mpc_planner/paramConfig.h>

#include <string.h>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <Eigen/Eigen>
#include <Eigen/LU>


class fub_mpc_lqr_planner
{
private:
  
  dynamic_reconfigure::Server<fub_mpc_planner::paramConfig> server;
  dynamic_reconfigure::Server<fub_mpc_planner::paramConfig>::CallbackType f;
  int init_state();
  void system_dynamic(int ndim);


  double vx,vy;

  double y,y_ref;
  //contrains
  double vx_min,vx_max,vy_min,vy_max;
  double ax_min,ax_max,ay_min,ay_max;
  double left_boundry, right_boundry;
  double vehicle_width;
  double delta_t;
  Eigen::VectorXd x; //states
  Eigen::MatrixXd A,B,R,Q,K,P,U,W,controllability;
  void config_callback(fub_mpc_planner::paramConfig &config, uint32_t level);

public:
  
  fub_mpc_lqr_planner(ros::NodeHandle nh);
  ~fub_mpc_lqr_planner(){}
  void optimize(int ndim);
};
