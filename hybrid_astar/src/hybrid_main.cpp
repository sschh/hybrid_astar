#include <iostream>
#include <ctime>
#include <cmath>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "cJSON/cJSON.h"
#include <math.h>

#include "Input.h"
#include "collisiondetection.h"
#include "algorithm.h"
#include "node2d.h"
#include "node3d.h"
#include "path.h"


using namespace std;
using namespace HybridAStar;
//using json=nlohmann::json;

class Planner {
 public:
  /// The default constructor
  Planner();

  void setMap(const nav_msgs::OccupancyGrid::Ptr map);
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
  
  void tracePath(const Node3D* node, int = 0, std::vector<Node3D> path = std::vector<Node3D>());
  
  void plan();

 private:
  /// The node handle
  ros::NodeHandle nh;
  /// A publisher publishing the start position for RViz
  ros::Publisher pubStart;
  /// A subscriber for receiving map updates
  ros::Subscriber subMap;
  /// A subscriber for receiving start updates
  //ros::Subscriber subStart;
  /// A subscriber for receiving goal updates
  //ros::Subscriber subGoal;

  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The path produced by the hybrid A* algorithm
  Path path;
  
  std::vector<Node3D> resultpath;
  
  /// A pointer to the grid the planner runs on
  //nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  //geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  //geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
};

Planner::Planner() {
   // TOPICS TO PUBLISH
  pubStart = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  
  // TOPICS TO SUBSCRIBE
  //subMap = nh.subscribe("/map", 1, &Planner::setMap, this);
  //subStart = nh.subscribe("/initialpose", 1, &Planner::setStart, this);
  //subGoal = nh.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this); 
};

//###################################################
//                                                MAP
//###################################################
/*
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
}
*/

//###################################################
//                                   INITIALIZE START
//###################################################
/*
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position =initial->pose.pose.position;
  startN.pose.orientation =initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;
    // publish start for RViz
    pubStart.publish(startN);
    
    plan();
  } 
}
*/

//###################################################
//                                    INITIALIZE GOAL
//###################################################
/*
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;
    
    plan();
  } 
}
*/

void Planner::tracePath(const Node3D* node, int i, std::vector<Node3D> path) {
  if (node == nullptr) {
    this->resultpath = path;
    reverse(this->resultpath.begin(),this->resultpath.end());
    float path_length=0;
    for(int j=0;j<i;j++){
      if (j>0) path_length+=hypot(this->resultpath[j].getX()-this->resultpath[j-1].getX(),this->resultpath[j].getY()-this->resultpath[j-1].getY());
      std::cout <<j<<":"<<this->resultpath[j].getX() <<" "<<this->resultpath[j].getY() <<" "<< this->resultpath[j].getT()<<std::endl;
    }
    std::cout <<"path_length: "<<path_length<<std::endl;
    return;
  }

  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}


//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  //if (validStart && validGoal) {

    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = Input::getInstance()->grid_width;//grid->info.width;
    int height =Input::getInstance()->grid_height;//grid->info.height;
    int depth = Constants::headings;
    //std::cout << "width,height,depth: " << width << height << depth << std::endl;

    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[width * height * depth]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    //float x = goal.pose.position.x / Constants::cellSize;
    //float y = goal.pose.position.y / Constants::cellSize;
    //float t = tf::getYaw(goal.pose.orientation);
    float x = 10 / Constants::cellSize;
    float y = 10/ Constants::cellSize;
    float t = 3;
    // set theta to a value (0,2PI]
    //t = Constants::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
  
    // ________________________
    // retrieving start position
    //x = start.pose.pose.position.x / Constants::cellSize;
    //y = start.pose.pose.position.y / Constants::cellSize;
    //t = tf::getYaw(start.pose.pose.orientation);
    x = 48/ Constants::cellSize;
    y = 14 / Constants::cellSize;
    t = 1;
    // set theta to a value (0,2PI]
    //t = Constants::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();
    // CLEAR THE PATH
    path.clear();
    
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace);
    // TRACE THE PATH
    tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(resultpath);

    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;
    //ROS_INFO_STREAM("TIME in ms: "+std::to_string(d.toSec()*1000));
    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();

    delete [] nodes3D;
    delete [] nodes2D;

  //}
}

int main(int argc, char** argv) {

  std::cout<<"grid_width: "<<Input::getInstance()->grid_width<<endl;
  std::cout<<"grid_height: "<<Input::getInstance()->grid_height<<endl;
  std::cout<<"grid_data: \n"<<Input::getInstance()->grid_data[2]<<endl;
  //ros::init(argc, argv, "a_star");
  //Planner hy;
  //hy.plan(); 
  delete []Input::getInstance()->grid_data;
  //ros::spin();
  return 0;
}
