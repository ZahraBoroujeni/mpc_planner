#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Path.h"

#include <autonomos_atlas_arnd/Atlas.h>
#include <autonomos_atlas_lib/Atlas.h>
#include <autonomos_atlas_lib/Boundary.h>
#include <autonomos_atlas_lib/Lane.h>
#include <autonomos_atlas_lib/Lanes.h>
#include <autonomos_atlas_lib/RoadSection.h>
#include <autonomos_atlas_lib/RoadSections.h>
#include <autonomos_atlas_lib/Spline.h>
#include <autonomos_atlas_lib/lanemarking/LaneMarking.h>
#include <autonomos_atlas_lib/lanetype/LaneType.h>
#include <iostream>

// CasADi core
#include <casadi/casadi.hpp>

using namespace std;
using namespace autonomos::atlas;
using namespace casadi;


/** Starting point for the node.
 **
 ** @param argc
 ** @param argv
 ** @return
 **
 ** @ingroup @@
 */

struct mpc_obj {
  vector<double> x0_min;
  vector<double> x0_max;
  vector<double> x_min;
  vector<double> x_max;
  vector<double> xf_min;
  vector<double> xf_max;
  vector<double> x_init;
  vector<double> u_min;
  vector<double> u_max;
  vector<double> u_init;
  ConstSplinePtr drive;
  ConstSplinePtr rightBoundary;
  ConstSplinePtr leftBoundary;

};
class MyCallback : public Callback {
 private:
   // Data members
   mpc_obj mpc_obj1;
   // Private constructor
   MyCallback(mpc_obj mpc_obj) : mpc_obj1(mpc_obj) {}
   ~MyCallback(){} 
 public:
   // Creator function, creates an owning reference
   static Function create(const std::string& name, mpc_obj mpc_obj,
                          const Dict& opts=Dict()) {
     return Callback::create(name, new MyCallback(mpc_obj), opts);
   }

   // Initialize the object
   virtual void init() {
     std::cout << "initializing object" << std::endl;
   }
   
   virtual bool has_jacobian() {return true;}
   virtual Function get_jacobian(const std::string& name, const Dict& opts){
        SX x  = SX::sym("x");
        SX y  = SX::sym("y");
        return Function("f",{x,y},{DM::zeros(1,2)});
    }

  // virtual Function get_jacobian(const std::string& name, const Dict& opts=Dict());

    
   // Number of inputs and outputs
   virtual int get_n_in() { return 2;}
   virtual int get_n_out() { return 2;}

   // Evaluate numerically
    virtual std::vector<DM> eval(std::vector<DM>& arg) {
        double x = arg.at(0).scalar();
        double y = arg.at(1).scalar();
        Position pos_car;
        pos_car[0] = (double) x * boost::units::si::meter;
        pos_car[1] = (double) y * boost::units::si::meter;
        Length par=mpc_obj1.drive->findClosestParameter(pos_car);
        pos_car=mpc_obj1.leftBoundary->interpolate(par);
        std::vector<DM>  f = {pos_car[0].value(),pos_car[1].value()};
        return f;
    }
 };
nav_msgs::Path creat_path(vector<double> x, vector<double> y)
{
    nav_msgs::Path gui_path_car;
    geometry_msgs::PoseStamped new_goal;
    std::vector<geometry_msgs::PoseStamped> plan;
    int sampling_num=x.size();
    double last_x=0;
    for (int i=0; i<sampling_num; i++){

     new_goal.pose.position.x = x[i];
     new_goal.pose.position.y = y[i];
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

    return gui_path_car;
}
// void Error(double x, double y,mpc_obj mpc_obj)
// {
//  Position pos_car;
//  pos_car[0] = 0 * boost::units::si::meter;
//  pos_car[1] = 0 * boost::units::si::meter;
//     Length par=mpc_obj.drive->findClosestParameter(pos_car);
//     pos_car=mpc_obj.leftBoundary->interpolate(par);
// }


nav_msgs::Path mpc(int Num_obstacles,mpc_obj mpc_obj)
{
    // Declare variables
    SX u  = SX::sym("u",2*(1+Num_obstacles));  // input controls
    SX x  = SX::sym("x",4*(1+Num_obstacles));   // states
    SX x_d  = SX::sym("x_d",2);   // states

    // Number of differential states
    int nx = x.size1();

    // Number of controls
    int nu = u.size1();

    // Final time
    double tf = 20.0;

    // Number of shooting nodes
    int ns = 50;

    // ODE right hand side and quadrature
    SX ode=vertcat(x[2]*cos(x[3]),x[2]*sin(x[3]),u[0],u[1]);
    SX ode_obstacles;
    for(int k=0; k<Num_obstacles; ++k)
    {
        ROS_INFO("obsatcle number: %i",k);
        ode_obstacles=vertcat(x[2+k*4]*cos(x[3+k*4]),x[2+k*4]*sin(x[3+k*4]),u[0+2*k],u[0+2*k]);
        ode=vertcat(ode,ode_obstacles);
    }

    vector<double> xd,yd;
    Position pos_desired;
    // double d = 0.2;
    //      pos_desired=mpc_obj.drive->interpolate(mpc_obj.drive->length() * d );

    for(int k=0; k<ns; k++)
    {
        double d= double(k)/ns;
        ROS_INFO("%f",d);
        pos_desired=mpc_obj.drive->interpolate(mpc_obj.drive->length() * d);
        xd.push_back(pos_desired[0].value());
        yd.push_back(pos_desired[1].value());
    }

        //refrence
        // Position pos_car;
        // pos_car[0] = double (X[k][0]) * boost::units::si::meter;
        // pos_car[1] = double (X[k][1]) * boost::units::si::meter;
        //Length par=mpc_obj.drive->findClosestParameter(pos_car);
        //pos1=mpc_obj.leftBoundary->interpolate(param1);

    //Position pos_desired=mpc_obj.drive->interpolate();
    SX drive_spline=0;

    SX quad = 0.1*pow(u[0],2) + 0.001*pow(u[1],2);//+(pow(x[0]-mpc_obj.xf_min[0],2)+pow(x[1]-mpc_obj.xf_min[1],2);

    for(int k=0; k<Num_obstacles; ++k)
    {
        quad=quad+0.1*pow(u[0+k*2],2) + 0.01*pow(u[1+k*2],2);
    }
    // make a discrete dynamic
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
          v_min.insert(v_min.end(), mpc_obj.x0_min.begin(), mpc_obj.x0_min.end());
          v_max.insert(v_max.end(), mpc_obj.x0_max.begin(), mpc_obj.x0_max.end());
        } else {
          v_min.insert(v_min.end(), mpc_obj.x_min.begin(), mpc_obj.x_min.end());
          v_max.insert(v_max.end(), mpc_obj.x_max.begin(), mpc_obj.x_max.end());
        }
        v_init.insert(v_init.end(), mpc_obj.x_init.begin(), mpc_obj.x_init.end());
        offset += nx;

        // Local control
        U.push_back( V[Slice(offset,offset+nu)] );
        v_min.insert(v_min.end(), mpc_obj.u_min.begin(), mpc_obj.u_min.end());
        v_max.insert(v_max.end(), mpc_obj.u_max.begin(), mpc_obj.u_max.end());
        v_init.insert(v_init.end(), mpc_obj.u_init.begin(), mpc_obj.u_init.end());
        offset += nu;
    }
    // State at end
    X.push_back(V[Slice(offset,offset+nx)]);
    v_min.insert(v_min.end(), mpc_obj.xf_min.begin(), mpc_obj.xf_min.end());
    v_max.insert(v_max.end(), mpc_obj.xf_max.begin(), mpc_obj.xf_max.end());
    v_init.insert(v_init.end(), mpc_obj.x_init.begin(), mpc_obj.x_init.end());    
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

    vector<MX> X_D;
    X_D.push_back( V[Slice(0,0+2)] );
    Function f = MyCallback::create("f", mpc_obj);

    // Loop over shooting nodes
    for(int k=0; k<ns; ++k){
        // Create an evaluation node

        MXDict I_out = F(MXDict{{"x0", X[k]},{"p", U[k]}});
        Xk_end=I_out.at("xf");

        // Save continuity constraints
        g.push_back( Xk_end - X[k+1] );
        gl={0.0,0.0,0.0,0.0};
        gh={0.0,0.0,0.0,0.0};
        g_min.insert(g_min.end(), gl.begin(), gl.end());
        g_max.insert(g_max.end(), gh.begin(), gh.end());

        //street boundries and obstacles constraints
        // for(int k=0; k<Num_obstacles; ++k)
        // {
            
        //  theta=atan2(-2*Xk_end[4*k]/(2*(2.25-pow(Xk_end[4*k],2))),1);
        //  //obstacle road spline!
        //  g.push_back(pow(Xk_end[4*k],2)+pow(Xk_end[1+4*k],2)-2.25);
        //  g.push_back(pow(cos(theta)*(Xk_end[0]-Xk_end[4*k])+sin(theta)*(Xk_end[1]-Xk_end[1+4*k]),2)/pow(0.3,2)+pow(sin(theta)*(Xk_end[0]-Xk_end[4*k])-cos(theta)*(Xk_end[1]-Xk_end[1+4*k]),2)/0.25);
        //  gl={0.0,0.0,0.0,0.0,-0.1,1.0};
        //  gh={0.0,0.0,0.0,0.0,0.1,inf};
        //  g_min.insert(g_min.end(), gl.begin(), gl.end());
        //  g_max.insert(g_max.end(), gh.begin(), gh.end());
        // }
        // //spline_left_boundry
        // g.push_back(pow(Xk_end[0],2)+pow(Xk_end[1],2)-2);
        // //spline_right_boundry
        // g.push_back(pow(Xk_end[0],2)+pow(Xk_end[1],2)-9);
        // gl={0,-inf};
        // gh={inf,0};
        // g_min.insert(g_min.end(), gl.begin(), gl.end());
        // g_max.insert(g_max.end(), gh.begin(), gh.end());

        // Add objective function contribution
        //refrence
        //Error(X[k][0],X[k][1]);
        //Function= external('Error',*X[k][0],*X[k][1],mpc_obj);

        vector<MX> arg={X[k][0],X[k][1]};
        std::vector<MX> res = f(arg);
        std::cout << res.at(1) << std::endl;
        //std::vector<MX> res = {pos_car[0].value(),pos_car[1].value()};
        MX driveSpline=0.01*(pow(X[k][0]-res.at(0),2)+pow(X[k][1]-res.at(1),2));
        J += I_out.at("qf")+driveSpline;
    }

    // NLP 
    MXDict nlp = {{"x", V}, {"f", J}, {"g", vertcat(g)}};

    // Set options
    Dict opts;
    opts["verbose_init"] = 0;
    opts["verbose"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.tol"] = 1e-5;
    opts["ipopt.max_iter"] = 200;
    opts["ipopt.hessian_approximation"] = "limited-memory";
    opts["ipopt.derivative_test"] = "first-order";
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
    }
    
    cout << "x_opt = " << endl << x_opt << endl;
    cout << "y_opt = " << endl << y_opt << endl;
    return creat_path(x_opt,y_opt);

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "fub_mpc_planner");
    ros::NodeHandle n;
    nav_msgs::Path path_car,path_refrence,path_lb,path_rb;
    ros::Publisher path_car_pub,path_refrence_pub,path_lb_pub,path_rb_pub;
    path_car_pub=n.advertise<nav_msgs::Path>("/path_nmpc", 1000);
    path_refrence_pub=n.advertise<nav_msgs::Path>("/path_refrence", 1000);
    path_lb_pub=n.advertise<nav_msgs::Path>("/path_left_boundry", 1000);
    path_rb_pub=n.advertise<nav_msgs::Path>("/path_right_boundry", 1000);
    mpc_obj mpc_obj;
    //read the map
    std::string packagePath = ros::package::getPath("fub_mpc_planner");
    n.setParam("/mapfile", packagePath + "/maps/tempelhof.arnd3");
    std::string arndMapFile;
    n.getParam("/mapfile", arndMapFile);
    shared_ptr<autonomos::atlas::Atlas> atlas;
    atlas.reset(new autonomos::atlas::arnd::Atlas(arndMapFile));
    //TODO: read the spline of drive lane and boundries
    //select first road section
    ConstRoadSectionsPtr roadSectionsPtr = atlas->roadSections();
    RoadSections::ConstIterator it = roadSectionsPtr->begin();
    ConstRoadSectionPtr roadSectionPtr = *it;

    //select first lane of the section  
    ConstLanesPtr lanesPtr = roadSectionPtr->lanes();
    
    //drive spline 
    Lanes::ConstIterator itl = lanesPtr->begin();
    ConstLanePtr lanePtr = *itl;
    ConstSplinePtr splineDrivePtr = lanePtr->drive();
    mpc_obj.drive=splineDrivePtr;
    //left boudries:
    ConstBoundaryPtr leftBoundaryPtr=lanePtr->leftBoundary();
    mpc_obj.leftBoundary=leftBoundaryPtr->spline();

    //select end lane of the section
    Lanes::ConstIterator itlend = lanesPtr->end();
    lanePtr = *(--itlend);
    //right boudries:
    ConstBoundaryPtr rightBoundaryPtr=lanePtr->rightBoundary();
    mpc_obj.rightBoundary=rightBoundaryPtr->spline();

    
    
    Position pos1=splineDrivePtr->supportPoint(splineDrivePtr->numSupportPoints() - 1);;
    ROS_INFO("before: %f",pos1[0].value());
    Length param1=mpc_obj.leftBoundary->findClosestParameter(pos1);
    pos1=mpc_obj.leftBoundary->interpolate(param1);
    ROS_INFO("interpolate: %f",pos1[0].value());
    //TODO:Read the obstacles type and positions:
    //assume that there is not any obstacles:
    int Num_obstacles=0;

    /*******************input contrain******************************************************
        * umin < U < umax
    ********************************************************************************/
    mpc_obj.u_min = {-5, -1};
    mpc_obj.u_max = {+5, +1};
    mpc_obj.u_init = {0, 0};
    /*******************state contrain******************************************************
        * xmin < X < xmax X=[x,y,theta,velocity]
    ********************************************************************************/
    // Bounds and initial guess for the state
    //TODO: read it from the map
    //find minumum and maximum  point of the road section point
    //the minimum velocity is 0: the car could not go backward in highway
    mpc_obj.x_min  = {-150,-150,0,-3.14};
    mpc_obj.x_max  = { 150, 150,20,+3.14};

    //TODO: Read current position and velocity of the car form gps
    //for now I just read the first vertex of the road section as the initial point
    Position pos = splineDrivePtr->supportPoint(2);
    
    mpc_obj.x0_min = {pos[0].value(), pos[1].value(), 0.0, 0.0};
    mpc_obj.x0_max = {pos[0].value(), pos[1].value(), 0.0, 0.0};
    mpc_obj.x_init = {pos[0].value(), pos[1].value(), 0.0, 0.0};
    //final point could be every point in the map!
    pos = splineDrivePtr->supportPoint(splineDrivePtr->numSupportPoints() - 3);
    mpc_obj.xf_min  = {pos[0].value(), pos[1].value(),0,-3.14};
    mpc_obj.xf_max  = {pos[0].value(), pos[1].value(),20,+3.14};
    /************************************************************************/
    //TODO: based on type of the obstacle
    //if (obstacles.type==car) and assume that the car with constant velocity will follow the road!
    vector<double> uobs_min = {-0.01,-1};
    vector<double> uobs_max = { +0.01,+1};
    //TODO: read the position and velocity of the obstacles
    vector<double> x_obs_min  = {-150,-150,0,-3.14};
    vector<double> x_obs_max  = { 150, 150,20,+3.14};
    vector<double> xf_obs_min  = {-150,-150,0,-3.14};
    vector<double> xf_obs_max  = { 150, 150,20,+3.14};

    //TODO: Read current position and velocity of the car form gps
    //for now I just read the first vertex of the road section as the initial point
    pos = splineDrivePtr->supportPoint(splineDrivePtr->numSupportPoints()/2);
    vector<double> x0_obs_min = {pos[0].value(), pos[1].value(), 0.0, 0.0};
    vector<double> x0_obs_max = {pos[0].value(), pos[1].value(), 0.0, 0.0};
    vector<double> x_obs_init = {pos[0].value(), pos[1].value(), 0.0, 0.0};

    for (int k=0;k<Num_obstacles;k++)
    {
        mpc_obj.u_min.insert(mpc_obj.u_min.end(),uobs_min.begin(), uobs_min.end());
        mpc_obj.u_max.insert(mpc_obj.u_max.end(),uobs_max.begin(), uobs_max.end());
        mpc_obj.x_min.insert(mpc_obj.x_min.end(),x_obs_min.begin(), x_obs_min.end());
        mpc_obj.x_max.insert(mpc_obj.x_max.end(),x_obs_max.begin(), x_obs_max.end());
        mpc_obj.xf_min.insert(mpc_obj.xf_min.end(),xf_obs_min.begin(), xf_obs_min.end());
        mpc_obj.xf_max.insert(mpc_obj.xf_max.end(),xf_obs_max.begin(), xf_obs_max.end());
        mpc_obj.x0_min.insert(mpc_obj.x0_min.end(),x0_obs_min.begin(), x0_obs_min.end());
        mpc_obj.x0_max.insert(mpc_obj.x0_max.end(),x0_obs_max.begin(), x0_obs_max.end());
        mpc_obj.x_init.insert(mpc_obj.x_init.end(),x_obs_init.begin(), x_obs_init.end());
    }

    path_car=mpc(Num_obstacles,mpc_obj);
    ros::Rate rate(0.5);

    //Visualize refrence 
    vector<double> xd,yd;
    Position pos_desired;
            //Visualize left boundry
    for(int k=0; k<50; k++)
    {
        double d= double(k)/50;
        pos_desired=mpc_obj.leftBoundary->interpolate(mpc_obj.leftBoundary->length() * d);
        xd.push_back(pos_desired[0].value());
        yd.push_back(pos_desired[1].value());
    }
    path_lb=creat_path(xd,yd);
    xd.clear();
    yd.clear();
        //Visualize refrence 
    for(int k=0; k<50; k++)
    {
        double d= double(k)/50;
        pos_desired=mpc_obj.drive->interpolate(mpc_obj.drive->length() * d);
        xd.push_back(pos_desired[0].value());
        yd.push_back(pos_desired[1].value());
    }
    path_refrence=creat_path(xd,yd);
    xd.clear();
    yd.clear();

    //Visualize right boundry
    for(int k=0; k<50; k++)
    {
        double d= double(k)/50;
        pos_desired=mpc_obj.rightBoundary->interpolate(mpc_obj.rightBoundary->length() * d);
        xd.push_back(pos_desired[0].value());
        yd.push_back(pos_desired[1].value());
    }
    path_rb=creat_path(xd,yd);
    xd.clear();
    yd.clear();



    while (ros::ok()) {

        // Wait until it is time for another iteration.
        path_car_pub.publish(path_car);
        path_refrence_pub.publish(path_refrence);
        path_lb_pub.publish(path_lb);
        path_rb_pub.publish(path_rb);
        rate.sleep();
    }

    ros::spin();
}
