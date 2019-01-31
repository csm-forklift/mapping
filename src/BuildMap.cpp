
#include <ros/ros.h>
#include <string>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include "mapping/ElevationMapping.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

#include <geometry_msgs/PoseStamped.h>


class Map_Builder {


private:
ros::NodeHandle nh_;
ros::Subscriber submap;
ros::Publisher pubmap;
float ele_min_data = 0.0;
float ele_max_data = 1.0;
float ele_clearance =0.00;
double base_height;// = 0.13;
double robot_width;
double resolution;
nav_msgs::OccupancyGrid ele_occ; 
nav_msgs::MapMetaData datz;
grid_map::GridMap map;

public:
  Map_Builder(){

    pubmap = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1,true);
    submap = nh_.subscribe("/mapping/ele_map",1,&Map_Builder::GridMapCallback, this);
    nh_.param("base_height", base_height, 0.14);
    nh_.param("robot_width", robot_width,0.67);
    nh_.param("/mapping/resolution", resolution, 0.67);
  }

void GridMapCallback(const grid_map_msgs::GridMap& message){

    grid_map::GridMapRosConverter::fromMessage(message, map);
    const std::string& layer = "elevation";
    float temp_value;
   // nh_.getParam("base_height",base_height);
    
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      if(map.at(layer, *iterator) < 0.0){
        temp_value = abs(map.at(layer,*iterator));
      }
      else{
        temp_value = map.at(layer,*iterator);
      }
      map.at(layer, *iterator) = temp_value;
    }

    grid_map::GridMapRosConverter::toOccupancyGrid(map,layer,ele_min_data,ele_max_data, ele_occ);
    datz = ele_occ.info;

//Following loops select data from grid maps and places -1, 0, or 100 into 2D array for dilation.

   int input_occ_map[datz.height][datz.width];
    for (int rows = 0; rows < datz.height; rows++){

      for(int cols = 0; cols < datz.width; cols++){
            
        int i = cols*datz.height;
        if(ele_occ.data[i+rows] == -1){
          ele_occ.data[i+rows] = -1;
          input_occ_map[rows][cols] = -1;
          //continue;
        }
        else if(ele_occ.data[i+rows] <= 100*(base_height - ele_clearance - ele_min_data)/(ele_max_data-ele_min_data)){
          ele_occ.data[i+rows] = 0;
          input_occ_map[rows][cols] = 0;
        }
        else{
          ele_occ.data[i+rows] = 100;
          input_occ_map[rows][cols] = 100;
        } 
      }
    }

  //Dilation(input_occ_map);
  // The next section is the dilation loops this should be its own function
   // for(int k = 0; k < 0.5*robot_width/resolution; k++){
/*    for (int i=0; i<datz.height; i++){
        for (int j=0; j<datz.width; j++){
            if (input_occ_map[i][j] == 100){
                if (i>0) input_occ_map[i-1][j] = 2;
                if (j>0) input_occ_map[i][j-1] = 2;
                if (i+1<datz.height) input_occ_map[i+1][j] = 2;
                if (j+1<datz.width) input_occ_map[i][j+1] = 2;
                if (i>0 && j> 0) input_occ_map[i-1][j-1] = 2;
                if (i>0 && j+1<datz.width) input_occ_map[i-1][j+1] = 2;
                if (i+1<datz.height && j>0) input_occ_map[i+1][j-1] = 2;
                if (i+1<datz.height && j+1<datz.width) input_occ_map[i+1][j+1] = 2;
            }
        }
    }
    for (int i=0; i<datz.height; i++){
        for (int j=0; j<datz.width; j++){
            if (input_occ_map[i][j] == 2){
                input_occ_map[i][j] = 100; 
                if(input_occ_map[i+1][j+1]==0)  input_occ_map[i+1][j+1] =90;
                if(input_occ_map[i][j+1]==0)  input_occ_map[i][j+1] =90;
                if(input_occ_map[i-1][j-1]==0)  input_occ_map[i-1][j-1] =90;
                if(input_occ_map[i][j-1]==0)  input_occ_map[i][j-1] =90;
                if(input_occ_map[i+1][j-1]==0)  input_occ_map[i+1][j+1] =90;
                if(input_occ_map[i+1][j]==0)  input_occ_map[i+1][j] =90;
                }

            }
        }
        for (int i=0; i<datz.height; i++){
           for (int j=0; j<datz.width; j++){
               if (input_occ_map[i][j] == 90){
                 if(input_occ_map[i+1][j+1]==0)  input_occ_map[i+1][j+1] =50;
                 if(input_occ_map[i][j+1]==0)  input_occ_map[i][j+1] =50;
                 if(input_occ_map[i-1][j-1]==0)  input_occ_map[i-1][j-1] =50;
                 if(input_occ_map[i][j-1]==0)  input_occ_map[i][j-1] =50;
                 if(input_occ_map[i+1][j-1]==0)  input_occ_map[i+1][j+1] =50;
                 if(input_occ_map[i+1][j]==0)  input_occ_map[i+1][j] =50; 
                }
            }
        }
                






     }*/
  //}

// The following code takes dilated points and puts in nav message as well as gives back negative ones
  
  //int8_t temp_int;
  int counter = 0;
  for(int rowz = 0; rowz <datz.height; rowz++){
    for(int colz = 0; colz<datz.width; colz++){
      int j = colz*datz.height;
     
      if(input_occ_map[rowz][colz] == -1){
        counter = 0;

        if(rowz>0 && input_occ_map[rowz-1][colz]==0) counter++;
        if(colz>0 && input_occ_map[rowz][colz-1]==0) counter++;
        if(rowz+1<datz.height && input_occ_map[rowz+1][colz]==0) counter++;
        if(colz+1<datz.width && input_occ_map[rowz][colz+1]==0) counter++;
        if((rowz>0 && colz>0)&& input_occ_map[rowz-1][colz-1] == 0) counter++;
        if((rowz+1<datz.height && colz+1< datz.width) && input_occ_map[rowz+1][colz+1] == 0) counter++;
        if((colz>0 && rowz+1<datz.height) && input_occ_map[rowz+1][colz-1]== 0) counter++;
        if((rowz>0 && colz+1<datz.width) && input_occ_map[rowz-1][colz+1]== 0) counter++;
      
      if(counter >= 4){  
        ele_occ.data[j+rowz] = 0;
      }

     }
     else ele_occ.data[j+rowz] = input_occ_map[rowz][colz];
      //temp_int = i
      
    }
  }
   
    pubmap.publish(ele_occ);
}

//void Dilation(int input[][]){
  
//  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_build_node");
  ROS_INFO("Starting BuildMap.cpp");
  Map_Builder mb;

  ros::spin();
  ros::waitForShutdown();
  return 0;
}

