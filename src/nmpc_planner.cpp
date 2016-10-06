/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            K.U. Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/** \brief Writing a multiple shooting code from scratch 

  This example demonstrates how to write a simple, yet powerful multiple shooting code from 
  scratch using CasADi. For clarity, the code below uses a simple formulation with only states 
  and controls, and no path constrants. It relies on CasADi machinery to keep track of sparsity,
  formulate ODE sensitivity equations and build up the Jacobian of the NLP constraint function.
    
  By extending the code below, it should be possible for a user to solve ever more complex 
  problems. For example, one can easily make a multi-stage formulation by simply allocating 
  another integrator instance and use the two integrators instances for different shooting nodes.
  By replacing the explicit CVodes integrator with the fully-implicit IDAS integrator, one can
  solve optimal control problems in differential-algebraic equations.
  
  \author Joel Andersson
  \date 2011-2012
*/

// CasADi core
#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <nodelet/loader.h>
#include "nav_msgs/Path.h"

using namespace casadi;
using namespace std;

int main(int argc, char **argv){

  ros::init(argc, argv, "fmpc_node");
  ros::NodeHandle nodeh;
  ros::Publisher path_car_pub,path_obstacle_pub;
  path_car_pub=nodeh.advertise<nav_msgs::Path>("/path_fmpc", 1000);
  path_obstacle_pub=nodeh.advertise<nav_msgs::Path>("/path_obstacle", 1000);
  std::vector<geometry_msgs::PoseStamped> plan,plan_obstacle;
  // Declare variables
  SX u  = SX::sym("u",4);  // input controls
  SX x  = SX::sym("x",8);   // states

  // Number of differential states
  int nx = x.size1();
  
  // Number of controls
  int nu = u.size1();

  // Bounds and initial guess for the control
  vector<double> u_min = {-0.5, -1, -0.01,-1};
  vector<double> u_max = {+0.5, +1, +0.01,+1};
  vector<double> u_init = {0,0,0,0};

  // Bounds and initial guess for the state
  vector<double> x0_min = {-1.5, 0, 0.7, 1.5,-1, 1.11, 0.1, 0.8};
  vector<double> x0_max = {-1.5, 0, 0.7, 1.5,-1, 1.11, 0.1, 0.8};
  vector<double> x_min  = {-3,-1,0,-3.14,-3, 0, 0, -3.14};
  vector<double> x_max  = { 3, 3,2,+3.14, 3, 3, 1, +3.14};
  vector<double> xf_min = {-3,-1,0,-3.14,-3, 0, 0, -3.14};
  vector<double> xf_max = { 3, 3,2,+3.14, 3, 3, 1, +3.14};
  vector<double> x_init = {-1.5, 0, 0.7, 1.5,-1, 1.11, 0.1, 0.8};

  // Final time
  double tf = 20.0;
  
  // Number of shooting nodes
  int ns = 50;

  // ODE right hand side and quadrature
  SX ode = vertcat(x[2]*cos(x[3]),x[2]*sin(x[3]),u[0],u[1]);
  ode=vertcat(ode, x[6]*cos(x[7]),x[6]*sin(x[7]),u[2]);
  ode=vertcat(ode,u[3]);
  
  SX quad = 0.1*pow(u[0],2) + 0.01*pow(u[1],2)+ 0.1*pow(u[2],2) + 0.01*pow(u[3],2)+0.001*pow(pow(x[0],2)+pow(x[1],2)-2.25,2)+10*(pow(x[0]-1.5,2) +pow(x[1]-0,2));
  SXDict dae = {{"x", x}, {"p", u}, {"ode", ode}, {"quad", quad}};

  // Create an integrator (CVodes)
  Function F = integrator("integrator", "cvodes", dae, {{"t0", 0}, {"tf", tf/ns}});
  
  // Total number of NLP variables
  int NV = nx*(ns+1) + nu*ns;
  
  // Declare variable vector for the NLP
  MX V = MX::sym("V",NV);

  // NLP variable bounds and initial guess
  vector<double> v_min,v_max,v_init;
  
  // Offset in V
  int offset=0; 

  // State at each shooting node and control for each shooting interval
  vector<MX> X, U;
  for(int k=0; k<ns; ++k){
    // Local state
    X.push_back( V[Slice(offset,offset+nx)] );
    if(k==0){
      v_min.insert(v_min.end(), x0_min.begin(), x0_min.end());
      v_max.insert(v_max.end(), x0_max.begin(), x0_max.end());
    } else {
      v_min.insert(v_min.end(), x_min.begin(), x_min.end());
      v_max.insert(v_max.end(), x_max.begin(), x_max.end());
    }
    v_init.insert(v_init.end(), x_init.begin(), x_init.end());
    offset += nx;
    
    // Local control
    U.push_back( V[Slice(offset,offset+nu)] );
    v_min.insert(v_min.end(), u_min.begin(), u_min.end());
    v_max.insert(v_max.end(), u_max.begin(), u_max.end());
    v_init.insert(v_init.end(), u_init.begin(), u_init.end());
    offset += nu;
  }
  
  // State at end
  X.push_back(V[Slice(offset,offset+nx)]);
  v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
  v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
  v_init.insert(v_init.end(), x_init.begin(), x_init.end());    
  offset += nx;
  
  // Make sure that the size of the variable vector is consistent with the number of variables that we have referenced
  casadi_assert(offset==NV);

  // Objective function
  MX J = 0;
  
  //Constraint function and bounds
  vector<MX> g;
  MX Xk_end;
  vector<double> gl,gh,g_min,g_max;
  MX theta;
  // Loop over shooting nodes
  for(int k=0; k<ns; ++k){
    // Create an evaluation node
    MXDict I_out = F(MXDict{{"x0", X[k]}, {"p", U[k]}});
    Xk_end=I_out.at("xf");
    // Save continuity constraints
    g.push_back( Xk_end - X[k+1] );
    theta=atan2(-2*Xk_end[4]/(2*(2.25-pow(Xk_end[4],2))),1);

    //street boundries and obstacles constraints

    g.push_back(pow(Xk_end[0],2)+pow(Xk_end[1],2));
    g.push_back(pow(Xk_end[4],2)+pow(Xk_end[5],2)-2.25);
    g.push_back(pow(cos(theta)*(Xk_end[0]-Xk_end[4])+sin(theta)*(Xk_end[1]-Xk_end[5]),2)/pow(0.3,2)+pow(sin(theta)*(Xk_end[0]-Xk_end[4])-cos(theta)*(Xk_end[1]-Xk_end[5]),2)/0.25);
    
    gl={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.2,-0.1,1.0};
    gh={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.0,0.1,inf};
    g_min.insert(g_min.end(), gl.begin(), gl.end());
    g_max.insert(g_max.end(), gh.begin(), gh.end());

    // Add objective function contribution
    J += I_out.at("qf");
  }
  
  // NLP 
  MXDict nlp = {{"x", V}, {"f", J}, {"g", vertcat(g)}};

  // Set options
  Dict opts;
  opts["ipopt.tol"] = 1e-5;
  opts["ipopt.max_iter"] = 100;
  //opts["ipopt.linear_solver"] = "ma27";

  // Create an NLP solver and buffers
  Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);
  std::map<std::string, DM> arg, res;

  // Bounds and initial guess
  arg["lbx"] = v_min;
  arg["ubx"] = v_max;
  arg["lbg"] = g_min;
  arg["ubg"] = g_max;
  arg["x0"] = v_init;

  // Solve the problem
  res = solver(arg);
    
  // Optimal solution of the NLP
  vector<double> V_opt(res.at("x"));
  
  // Get the optimal state trajectory
  vector<double> x_opt(ns+1), y_opt(ns+1),xo_opt(ns+1), yo_opt(ns+1);
  for(int i=0; i<=ns; ++i){
    x_opt[i] = V_opt.at(i*(nx+nu));
    y_opt[i] = V_opt.at(1+i*(nx+nu));
    xo_opt[i] = V_opt.at(4+i*(nx+nu));
    yo_opt[i] = V_opt.at(5+i*(nx+nu));
  }
  cout << "x_opt = " << endl << x_opt << endl;
  cout << "y_opt = " << endl << y_opt << endl;
  
  // Get the optimal control
  vector<double> u_opt(ns);
  for(int i=0; i<ns; ++i){
    u_opt[i] = V_opt.at(9 + i*(nx+nu));
  }
  cout << "u_opt = " << endl << u_opt << endl;
  

     // free(Xhist); free(Uhist);
  
  nav_msgs::Path gui_path_car,gui_path_obstacle;
  geometry_msgs::PoseStamped new_goal;
  int sampling_num=ns;
  double last_x=0;
  for (int i=0; i<sampling_num; i++){

     new_goal.pose.position.x = x_opt[i];
     new_goal.pose.position.y = y_opt[i];
     new_goal.pose.orientation.x = 0;
     new_goal.pose.orientation.y =0;
     new_goal.pose.orientation.z = 0;
     new_goal.pose.orientation.w =1;
     new_goal.header.stamp = ros::Time::now();
     new_goal.header.seq++;
     plan.push_back(new_goal);

 }
 gui_path_car.poses.resize(plan.size());

  if(!plan.empty()){
        gui_path_car.header.frame_id = "map";
        gui_path_car.header.stamp = plan[0].header.stamp;
  }

  for(unsigned int i=0; i < plan.size(); i++){
        gui_path_car.poses[i] = plan[i];
  }

  for (int i=0; i<sampling_num; i++){

     new_goal.pose.position.x = xo_opt[i];
     new_goal.pose.position.y = yo_opt[i];
     new_goal.pose.orientation.x = 0;
     new_goal.pose.orientation.y =0;
     new_goal.pose.orientation.z = 0;
     new_goal.pose.orientation.w =1;
     new_goal.header.stamp = ros::Time::now();
     new_goal.header.seq++;
     plan_obstacle.push_back(new_goal);

 }
 gui_path_obstacle.poses.resize(plan_obstacle.size());

  if(!plan_obstacle.empty()){
        gui_path_obstacle.header.frame_id = "map";
        gui_path_obstacle.header.stamp = plan_obstacle[0].header.stamp;
  }

  for(unsigned int i=0; i < plan_obstacle.size(); i++){
        gui_path_obstacle.poses[i] = plan_obstacle[i];
  }

   while (ros::ok()) {
      // ros::spinOnce();
      path_car_pub.publish(gui_path_car);
      path_obstacle_pub.publish(gui_path_obstacle);
   }

  return 0;
}
