
#include <fstream>

#include "cJSON/cJSON.h"

#include  "planner.h"

using namespace std;
using namespace HybridAStar;
//using json=nlohmann::json;


int main(int argc, char** argv) {

  std::cout<<"grid_width: "<<Input::getInstance()->grid_width<<std::endl;
  std::cout<<"grid_height: "<<Input::getInstance()->grid_height<<std::endl;
  std::cout<<"grid_data:"<<sizeof(Input::getInstance()->grid_data)/sizeof(Input::getInstance()->grid_data[0])<<std::endl;
  /*
  for (int i=0;i<Input::getInstance()->grid_width*Input::getInstance()->grid_height;i++){
    std::cout<<Input::getInstance()->grid_data[2]<<' ';
    if ((i+1)%Input::getInstance()->grid_width==0){std::cout<<"\t";}
  }
  */
  ros::init(argc, argv, "a_star");
  Planner hy;
  hy.plan(); 
 // delete [] Input::getInstance()->grid_data;
  //ros::spin();
  return 0;
}
