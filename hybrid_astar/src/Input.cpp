
#include "Input.h"
#include "cJSON/cJSON.h"
#include <iostream>
using namespace HybridAStar;

Input *Input::instance=new Input();
Input *Input::getInstance(){
	return instance;
}
Input::Input(){
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
}

void Input::doit(char *text)
{
	char *out;
  cJSON *json;
  cJSON *cjsonGridData;
  int tmp;
	
	json=cJSON_Parse(text);
	if (!json) {std::cout<<"Error "<<std::endl;}
	else
	{
    grid_width=cJSON_GetObjectItem(json,"width")->valueint;
    grid_height=cJSON_GetObjectItem(json,"height")->valueint;
    cjsonGridData=cJSON_GetObjectItem(json,"data");
    for (int i=0;i<grid_width*grid_height;i++){
      //pixel valid
      tmp=cJSON_GetArrayItem(cjsonGridData,i)->valueint;
      if (tmp!=0){grid_data[i]=tmp;}
    }
  }
}