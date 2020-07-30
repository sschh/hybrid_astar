#ifndef  PLANNER_H
#define  PLANNER_H

#include <math.h>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <stdio.h>

#include "Input.h"
#include "collisiondetection.h"
#include "algorithm.h"
#include "path.h"

namespace HybridAStar {

class Planner {
 public:
  /// The default constructor
  Planner();
 
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
  
  void tracePath(const Node3D* node, int = 0, std::vector<Node3D> path = std::vector<Node3D>());

  float curFind(Node3D node1,Node3D node2,Node3D node3);

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
  float path_length=0;
  int node_nums;
  float *path_speed=new float[node_nums];
  float *path_timestamp=new float[node_nums];
  float *path_curvature=new float[node_nums];
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

}
#endif