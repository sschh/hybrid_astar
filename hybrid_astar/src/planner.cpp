#include  "planner.h"

using namespace HybridAStar;

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
    this->node_nums=i;
    this->path_length=0;
    float prenode_dis=0;//dis between two nodes(from front)
    float postnode_dis=0;//dis between two nodes(from behind)
    reverse(this->resultpath.begin(),this->resultpath.end());
    //delete redundant node; sum path
    for(int j=0;j<i;j++){
      if (j>0){
        if (this->resultpath[j].getX()==this->resultpath[j-1].getX() and this->resultpath[j].getY()==this->resultpath[j-1].getY() and this->resultpath[j].getT()==this->resultpath[j-1].getT()){
        this->resultpath.erase(std::begin(this->resultpath)+j);
        j-=1;
        i-=1;
        continue;
        }
      this->path_length+=hypot(this->resultpath[j].getX()-this->resultpath[j-1].getX(),this->resultpath[j].getY()-this->resultpath[j-1].getY());
      }
    }
    //calculate every node speed\curvature\timestamp
    for(int j=0;j<i;j++){
      if (j>0) {
        if (j>i-j){break;}
        if (j<=i-1-j){
              prenode_dis=hypot(this->resultpath[j].getX()-this->resultpath[j-1].getX(),this->resultpath[j].getY()-this->resultpath[j-1].getY());
              postnode_dis=hypot(this->resultpath[i-1-j].getX()-this->resultpath[i-j].getX(),this->resultpath[i-1-j].getY()-this->resultpath[i-j].getY());
              this->path_speed[j]=std::min(Constants::maxSpeed,sqrt( this->path_speed[j-1]* this->path_speed[j-1]+2*Constants::acceleration*prenode_dis));
              this->path_speed[i-1-j]=std::min(Constants::maxSpeed,sqrt( this->path_speed[i-j]* this->path_speed[i-j]+2*Constants::acceleration*postnode_dis));
              this->path_curvature[j]=curFind(this->resultpath[j],this->resultpath[j+1],this->resultpath[j-1]);
              this->path_curvature[i-1-j]=curFind(this->resultpath[i-1-j],this->resultpath[i-j],this->resultpath[i-j-2]);
        }
        //timestamp
        if (this->path_speed[j]!=this->path_speed[j-1]){
          this->path_timestamp[j]=(this->path_speed[j]-this->path_speed[j-1])/Constants::acceleration;
        }
        else{
          this->path_timestamp[j]=prenode_dis/this->path_speed[j];
        }
        if (this->path_speed[i-j]!=this->path_speed[i-j-1]){
          this->path_timestamp[i-j]=(this->path_speed[i-j-1]-this->path_speed[i-j])/Constants::acceleration;
        }
        else{
          this->path_timestamp[i-j]=postnode_dis/this->path_speed[i-j];
        }
      }
      else
      {
       this->path_speed[j]=0;
       this->path_speed[i-1]=0;
       this->path_curvature[j]=0;
       this->path_curvature[i-1]=0;
       this->path_timestamp[j]=0;
      }
    }
    for(int j=1;j<i;j++){
        this->path_timestamp[j]+=this->path_timestamp[j-1];
    }
    //cout every node information
    std::cout <<"TIMESTAMP"<<"    "<<"X"<<"      "<<"Y"<<"      "<< "ANGLE"<<"      "<<"SPEED"<<"     "<<"CURVATURE"<<std::endl; 
    for(int j=0;j<i;j++){
          std::cout <<j<<":"<< this->path_timestamp[j]<<"  "<<this->resultpath[j].getX() <<"  "<<this->resultpath[j].getY() <<
          "  "<< this->resultpath[j].getT()<<"   "<<this->path_speed[j]<<"   "<<this->path_curvature[j]<<std::endl; 
    }
    std::cout <<"path_length: "<<this->path_length<<std::endl;
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
    float x = Input::getInstance()->goal_x / Constants::cellSize;
    float y = Input::getInstance()->goal_y/ Constants::cellSize;
    float t = Input::getInstance()->goal_heading;
    // set theta to a value (0,2PI]
    //t = Constants::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
  
    // ________________________
    // retrieving start position
    //x = start.pose.pose.position.x / Constants::cellSize;
    //y = start.pose.pose.position.y / Constants::cellSize;
    //t = tf::getYaw(start.pose.pose.orientation);
    x = Input::getInstance()->start_x/ Constants::cellSize;
    y = Input::getInstance()->start_y / Constants::cellSize;
    t = Input::getInstance()->start_heading;
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

//###########################
//GIven three points on a circle,find the radius
//###############
float Planner::curFind(Node3D node1,Node3D node2,Node3D node3){
  if (node2.getY()==node1.getY() and node2.getY() - node3.getY()) {return 0;} 
  if (node2.getX()==node1.getX() and node2.getX() - node3.getX()) {return 0;} 
  float radius=0;
	float mid1_x = (node1.getX() + node2.getX()) / 2;
	float mid1_y = (node1.getY() + node2.getY()) / 2;
	//求出点3和点1的中点
	float mid2_x = (node3.getX() + node2.getX()) / 2;
	float mid2_y = (node3.getY() + node2.getY()) / 2;
	//求出分别与直线pt1pt2，pt1pt3垂直的直线的斜率
	float k1 = -(node2.getX() - node1.getX()) / (node2.getY() - node1.getY());
	float k2 = -(node3.getX()- node2.getX()) / (node3.getY()-node2.getY());
	//连立两条中垂线方程求解交点得到：
	float center_x = (mid2_y - mid1_y - k2* mid2_x + k1*mid1_x) / (k1 - k2);
	float center_y = mid1_y + k1*(mid2_y - mid1_y - k2*mid2_x + k2*mid1_x) / (k1 - k2);
	//用圆心和其中一个点求距离得到半径：
	radius = sqrt((center_x - node1.getX())*(center_x - node1.getX()) + (center_y - node1.getY() )*(center_y - node1.getY()));
  return 1/radius;
}