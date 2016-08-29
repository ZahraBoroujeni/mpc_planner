#include <fub_mpc_planner/optimal_planner.h>
#include <ros/ros.h>

using namespace std;
using namespace autonomos::atlas;
using namespace fub::structured_planner;
using namespace fub::local_planner;

using namespace boost::units::si;


nav_msgs::Path optimal_planner::subsample(CubicSpline cubic,float start_x, float end_x)
{
    std::vector<geometry_msgs::PoseStamped> plan;
    nav_msgs::Path gui_path;
   geometry_msgs::PoseStamped new_goal;
   int sampling_num=100;
   for (int i=0; i<sampling_num; i++){

      new_goal.pose.position.x = i*(end_x-start_x)/sampling_num+start_x;
      new_goal.pose.position.y = cubic(i*(end_x-start_x)/sampling_num+start_x);
       new_goal.pose.orientation.x = 0;
       new_goal.pose.orientation.y =0;
       new_goal.pose.orientation.z = 0;
       new_goal.pose.orientation.w =1;
       new_goal.header.stamp = ros::Time::now();
       new_goal.header.seq++;
      plan.push_back(new_goal);

   }
   gui_path.poses.resize(plan.size());

    if(!plan.empty()){
          gui_path.header.frame_id = "map";
          gui_path.header.stamp = plan[0].header.stamp;
    }

    for(unsigned int i=0; i < plan.size(); i++){
          gui_path.poses[i] = plan[i];
    }
    return gui_path;

}
CubicSpline optimal_planner::calculateSpline(Position p_start,Position p_end, double car_direction, double road_direction_p_end) 
{
    Array<double> x_set(2);
    Array<double> y_set(2);
    x_set << p_start[0].value(),p_end[0].value();
    y_set << p_start[1].value(),p_end[1].value();
    double ydot_0 = car_direction;
    //double ydot_f= endEdge->getDriveSplinePtr()->interpolate(endEdge->getDriveSplinePtr()->length() * endPosition)->derivative;
    double ydot_f = road_direction_p_end;
    CubicSpline cubic = CubicSpline::DerivativeHeuristic(x_set, y_set, ydot_0, ydot_f);
    return cubic;
}

double optimal_planner::evaluateSpline(CubicSpline cubic,float start_x, float end_x) {

    double cost=cubic(end_x)-cubic(start_x);
    return cost;
}
double optimal_planner::scaled_value(Position pos,Position pos_min){
  return sqrt(pow(pos[0].value()-pos_min[0].value(),2)+pow(pos[1].value()-pos_min[1].value(),2));
}

vector<nav_msgs::Path> optimal_planner::createOptimalPlan(vector<structured_planner::EdgePtr> route,double obstacle_velocity) {
  std_msgs::String StateMachin;

  vector<nav_msgs::Path>  gui_path;
  vector<CubicSpline> cubic;
   
  
  geometry_msgs::PoseStamped new_goal;
 //double cost[3];
unsigned long int pervious_path_size;
for (int i = 0; i < 3; ++i )
{
  nav_msgs::Path path;
  for (int rout_id = 0; rout_id < route.size(); ++rout_id) 
  {
    if (rout_id==0)
      StateMachin.data="take_over";
    else
      StateMachin.data="follow";
    structured_planner::EdgePtr routeEdge = route.at(rout_id);
    if ((StateMachin.data=="turn_left")||(StateMachin.data=="take_over")) 
    {
      double Max_velocity = 200;
      Position laneChangeStartPos=routeEdge->getDriveSplinePtr()->interpolate(0);//routeEdge->getStart()->getPosition;
      double laneChangeLength =10+(obstacle_velocity/Max_velocity);
      Length laneChangeStart,laneChangeEnd;
      if (routeEdge->getDriveSplinePtr() != NULL) 
      {
        laneChangeLength=10+((obstacle_velocity/Max_velocity)*(1+(i-1)*2));
        laneChangeStart=scaled_value(laneChangeStartPos,routeEdge->getDriveSplinePtr()->interpolate(0)) * meter;
        laneChangeEnd=(laneChangeStart.value()+laneChangeLength) * meter;
        double splineLength=routeEdge->getDriveSplinePtr()->length().value();
        if (laneChangeEnd.value()>splineLength)
          laneChangeEnd=splineLength * meter;
        //ROS_INFO("%g",laneChangeEnd.value());
        Position p_start = routeEdge->getDriveSplinePtr()->interpolate(laneChangeStart);
        Position p_end = routeEdge->getDriveSplinePtr()->interpolate(laneChangeEnd);
        //I need the derivative of route
        double car_direction=-routeEdge->getDriveSplinePtr()->gradient(laneChangeStart)[0].value();
        double road_direction_p_end=-routeEdge->getDriveSplinePtr()->gradient(laneChangeEnd)[0].value();
        p_end[0]=(p_end[0].value() - 4) * meter;
        CubicSpline cubic1;
        if (p_end[0].value()>p_start[0].value())
          cubic1= calculateSpline(p_start,p_end,car_direction,road_direction_p_end);
        else
          cubic1= calculateSpline(p_end,p_start,road_direction_p_end,car_direction);
        cubic.push_back(cubic1);
        //cost[i]=evaluateSpline(cubic1,p_start[0].value(),p_end[0].value());
       
        path=subsample(cubic1,p_start[0].value(),p_end[0].value());
        std::vector<geometry_msgs::PoseStamped> plan;
        for (double j = laneChangeEnd.value(); j < splineLength; j += 0.5) 
        {
          Position p = routeEdge->getDriveSplinePtr()->interpolate(j * meter);
          new_goal.pose.position.x = p[0].value()-4.0;
          new_goal.pose.position.y = p[1].value();
          new_goal.pose.orientation.x = 0;
          new_goal.pose.orientation.y =0;
          new_goal.pose.orientation.z = 0;
          new_goal.pose.orientation.w =1;
          new_goal.header.stamp = ros::Time::now();
          new_goal.header.seq++;
          plan.push_back(new_goal);
        }
        
        pervious_path_size=path.poses.size();
        path.poses.resize(pervious_path_size+plan.size());
        ROS_INFO("size1 > %lu ",path.poses.size());

        if(!plan.empty()){
              path.header.frame_id = "map";
              path.header.stamp = plan[0].header.stamp;
        }
        for(unsigned int j=0; j < plan.size(); j++){
              path.poses[pervious_path_size+j] = plan[j];
        }
      }
    }
    else if (routeEdge->getDriveSplinePtr() != NULL)
    {
  
      std::vector<geometry_msgs::PoseStamped> plan;
      for (double j = 0; j < 1; j += 0.01) 
      {
        Position p = routeEdge->getDriveSplinePtr()->interpolate(routeEdge->getDriveSplinePtr()->length() * j);
        new_goal.pose.position.x = p[0].value();
        new_goal.pose.position.y = p[1].value();
        new_goal.pose.orientation.x = 0;
        new_goal.pose.orientation.y =0;
        new_goal.pose.orientation.z = 0;
        new_goal.pose.orientation.w =1;
        new_goal.header.stamp = ros::Time::now();
        new_goal.header.seq++;
        plan.push_back(new_goal);
      }
      pervious_path_size=path.poses.size();
      path.poses.resize(pervious_path_size+plan.size());
      ROS_INFO("size2 > %lu ",path.poses.size());
      if(!plan.empty()){
            path.header.frame_id = "map";
            path.header.stamp = plan[0].header.stamp;
      }
      for(unsigned int j=0; j < plan.size(); j++){
              path.poses[pervious_path_size+j] = plan[j];
      }
    }
  }
  gui_path.push_back(path);
}
  return gui_path;
}