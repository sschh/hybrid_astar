
#include "Input.h"
#include "cJSON/cJSON.h"

using namespace HybridAStar;

Input *Input::instance=new Input();
Input *Input::getInstance(){
	return instance;
}
Input::Input(){
  /*
	char *file="/home/ssc/leinuo/src/hybrid_astar/maps/map.json";
  //ifstream ifs; 
  FILE *f;long len;char *data;
	
	f=fopen(file,"rb");
  if (!f){
    std::cout<<"path error!"<<std::endl;
  }
  fseek(f,0,SEEK_END);
  len=ftell(f);
  fseek(f,0,SEEK_SET);
  data=(char*)malloc(len+1);
  fread(data,1,len,f);
  fclose(f);
  doit(data);
  free(data);
  */
  doit("abc");
}

void Input::doit(char *text)
{
	char *out;
  //cJSON *json;
  //cJSON *cjsonGridData;
  int tmp;
	
  //json=cJSON_Parse(text);
	//if (!json) {std::cout<<"Error "<<std::endl;}
	//else
	//{
    /*
    grid_width=cJSON_GetObjectItem(json,"width")->valueint;
    grid_height=cJSON_GetObjectItem(json,"height")->valueint;
    goal_x=cJSON_GetObjectItem(json,"goal_x")->valueint;
    goal_y=cJSON_GetObjectItem(json,"goal_y")->valueint;
    goal_heading=cJSON_GetObjectItem(json,"goal_heading")->valueint;
    start_x=cJSON_GetObjectItem(json,"start_x")->valueint;
    start_y=cJSON_GetObjectItem(json,"start_y")->valueint;
    start_heading=cJSON_GetObjectItem(json,"start_heading")->valueint;
    */
    grid_width=80;
    grid_height=80;
    goal_x=10;
    goal_y=10;
    goal_heading=1;
    start_x=25;
    start_y=30;
    start_heading=3;

    
    //cjsonGridData=cJSON_GetObjectItem(json,"data");
    /*
    for (int i=0;i<grid_width*grid_height;i++){
      //pixel valid
      tmp=cJSON_GetArrayItem(cjsonGridData,i)->valueint;
      if (tmp!=0){grid_data[i]=tmp;}
    }
    */
  //}
}