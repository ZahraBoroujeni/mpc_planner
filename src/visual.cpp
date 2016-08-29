#include <ros/ros.h>
#include <fub_atlas_msgs/Atlas.h>
#include <fub_atlas_publisher/AtlasDummyDecoder.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fub_mpc_planner/optimal_planner.h>
#include <std_msgs/Float32.h>


using namespace std;
using namespace fub::structured_planner;
using namespace fub::local_planner;
double speed=0;

void obstaclesCallback(const std_msgs::Float32& msg)
{
  speed=msg.data;
}

int main(int argc, char **argv) {

	if (argc < 2) { // 1 arguments is given default by rosrun
		cerr << "Too few arguments, bagfile needed!" << endl;
		return -1;
	}

	ros::init(argc, argv, "fub_optimal_planner");
	ros::NodeHandle n;
	ros::Publisher path_pub1,path_pub2,path_pub3;
	ros::Subscriber obstacles;
	obstacles=n.subscribe("/obstacle_speed",1 ,&obstaclesCallback);
	path_pub1=n.advertise<nav_msgs::Path>("/path_DerivativeHeuristic1", 1000);
	path_pub2=n.advertise<nav_msgs::Path>("/path_DerivativeHeuristic2", 1000);
	path_pub3=n.advertise<nav_msgs::Path>("/path_DerivativeHeuristic3", 1000);
	// shared_ptr<ros::Publisher> path_pub2(new ros::Publisher(n.advertise<nav_msgs::Path>("/path_DerivativeHeuristic2", 1000)));
	// shared_ptr<ros::Publisher> path_pub3(new ros::Publisher(n.advertise<nav_msgs::Path>("/path_DerivativeHeuristic3", 1000)));

	// load map from bag file
	fub_atlas_msgs::Atlas::ConstPtr msgPtr(new fub_atlas_msgs::Atlas());
	rosbag::Bag bag;
	bag.open(argv[1], rosbag::bagmode::Read);

	rosbag::View view(bag, rosbag::TopicQuery("fub_atlas_msgs/Atlas"));

	BOOST_FOREACH(rosbag::MessageInstance const m, view){
	fub_atlas_msgs::Atlas::ConstPtr atlasConstPtr = m.instantiate<fub_atlas_msgs::Atlas>();

	if (atlasConstPtr != NULL) {
		msgPtr = atlasConstPtr;
	}
}

	bag.close();
	shared_ptr<fub::atlas::AtlasDummyDecoder> decoder(new fub::atlas::AtlasDummyDecoder());
	shared_ptr<autonomos::atlas::Atlas> atlas = decoder->decode(*msgPtr);

	shared_ptr<TrajectoryPlanner> planner(new TrajectoryPlanner());
	planner->setAtlas(atlas);
	planner->initLaneGraph();


	GraphPtr graph = planner->getLaneGraph();

	vector<EdgePtr> edges = graph->getEdges();

	vector<EdgePtr> route;
	// route.push_back(edges.at(1));
	// route.push_back(edges.at(1)); // lane change
	route.push_back(edges.at(2));
	route.push_back(edges.at(4));
	// route.push_back(edges.at(7)); // lane change
	// route.push_back(edges.at(4));
	 route.push_back(edges.at(8));
	 route.push_back(edges.at(12));
	 route.push_back(edges.at(16));
	 route.push_back(edges.at(20));
	// route.push_back(edges.at(13)); // lane change
	// route.push_back(edges.at(14));
	ROS_INFO("create route");
	fub::local_planner::optimal_planner optim_planner;
	vector<nav_msgs::Path>  gui_path;

	

	ros::Rate rate(1);

	while (ros::ok()) {
		ros::spinOnce();
		gui_path=optim_planner.createOptimalPlan(route,speed);
    	path_pub1.publish(gui_path.at(0));
    	path_pub2.publish(gui_path.at(1));
    	path_pub3.publish(gui_path.at(2));
		// Wait until it is time for another iteration.
		rate.sleep();
	}

	ros::spin();
}
