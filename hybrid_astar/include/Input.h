#ifndef INPUT_H
#define INPUT_H
#include "cJSON/cJSON.h"
namespace HybridAStar {

/*!
   \brief 
*/
struct Input {
 public:
  static Input *getInstance();
  void readData();
  void doit(char *text);
  int grid_width; 
  int grid_height; 
  int *grid_data=new int [grid_width*grid_height]();

private:
  static Input *instance;
  Input();

};
}
#endif