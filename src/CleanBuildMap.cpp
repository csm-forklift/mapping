
#include <ros/ros.h>
#include <string>
#include <algorithm>
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
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <math.h>
#include <mapping/mapstitch.h>
using namespace cv;


class Map_builder {

private:
ros::NodeHandle nh_;
ros::Subscriber submap;
ros::Publisher pubmap, pubmap1;
float ele_min_data = 0.0;
float ele_max_data = 1.0;
float ele_clearance =0.00;
double base_height,robot_width,resolution, pose_x,pose_y;
nav_msgs::OccupancyGrid ele_occupancy,orig_Map,dilated_Map1;
nav_msgs::MapMetaData elevation_data;
grid_map::GridMap map;
int dilationSize;
//cv::Mat src,dst;

public:
	Map_builder() :nh_("~"){
		pubmap = nh_.advertise<nav_msgs::OccupancyGrid>("/original_map",1,true);
		pubmap1 = nh_.advertise<nav_msgs::OccupancyGrid>("/obstacle_map",1,true);
		nh_.param("/map_build_node/base_height",base_height,0.05);
		nh_.param("/map_build_node/robot_width",robot_width,0.4);
		nh_.param("/mapping/resolution",resolution,0.4);
		nh_.param("/map_build_node/erode_size",dilationSize,0);
		submap = nh_.subscribe("/mapping/ele_map",1,&Map_builder::GridMapCallback, this);
        //submap = nh_.subscribe("/map", 1, &Map_builder::GridMapCallback, this);
		// orig_Map = CreateMap();
		// dilated_Map1 = DilateMap(orig_Map);
		// pubmap.publish(orig_Map);
		// pubmap1.publish(dilated_Map1);
	}

	void GridMapCallback(const grid_map_msgs::GridMap& message){
 		grid_map::GridMapRosConverter::fromMessage(message, map);
    	const std::string& layer = "elevation";
    	float temp_value;
    	for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
            if(map.at(layer,*iterator) > 2.0){
                map.at(layer,*iterator) = 2.0;
            }
            

        	//if(map.at(layer, *iterator) < 0.0){
        		//temp_value = abs(map.at(layer,*iterator));
      		//}
      		//else{
        	//	temp_value = map.at(layer,*iterator);
      		//}
      	//	map.at(layer, *iterator) = temp_value;
    	}
		grid_map::GridMapRosConverter::toOccupancyGrid(map,layer,0.0,1.0, ele_occupancy);

		pubmap.publish(ele_occupancy);
		dilated_Map1 = DilateMap(ele_occupancy);
		pubmap1.publish(dilated_Map1);

	}
	nav_msgs::OccupancyGrid CreateMap(){
	   	nav_msgs::OccupancyGrid *test_map_ = new nav_msgs::OccupancyGrid;
	   	test_map_->header.seq=1; //this might need to be updated depending on how many time the occupancy grid is called
	   	test_map_->header.stamp=ros::Time::now();
	   	test_map_->header.frame_id="odom"; //will be use robot odometry frame

	   	test_map_->info.map_load_time=ros::Time::now();
	   	test_map_->info.width = (int)10/resolution;
	   	test_map_->info.height = (int)10/resolution;
	   	test_map_->info.resolution = resolution; // 1 m per cell

	   	ros::Rate r(0.1);  // frequency

	  	while(nh_.ok()){

	      if(acquire_robotpose()){ // update the robot pose
	        test_map_->info.origin.position.x = round(pose_x)-test_map_->info.width*resolution/2;
	        test_map_->info.origin.position.y = round(pose_y)-test_map_->info.height*resolution/2;
	      }
	      else{ROS_ERROR("An error has occurred acquiring robot pose");}

	      int p[test_map_->info.width*test_map_->info.height];

	      for (int i=0;i<test_map_->info.width*test_map_->info.height;i++){
		      	if(i==99 || i==0){
		    		p[i] = -1;
		    	}
		    	else if (i%3==1 && i%7==2){
		    		p[i] =100;
		    	}
		     	else if (i%5==1 && i%9==2){
		        	p[i] =100;
		      	}
		      	else if (i%4==1 && i%8==2){
		        	p[i] =100;
		      	}
		    	else{
		            p[i] = 0;
		     	}
	      }
	      vector<signed char> a(p,p+test_map_->info.width*test_map_->info.height);
	      test_map_->header.seq=test_map_->header.seq + 1;
	      test_map_->data = a;
	      ROS_INFO("Occupany map produced");
	      getchar();
	      return *test_map_;
    	}
	}
	bool acquire_robotpose(){
		  tf::StampedTransform odom_base_transform;
		  tf::Matrix3x3 mat;
		  double roll;
		  double pitch;
		  double yaw;
		  tf::TransformListener listener;
		  try{
		  //	ros::Time now = ros::Time(0);
		  	listener.waitForTransform("/base_link", "/base_link", ros::Time(0), ros::Duration(4.0));
		  	listener.lookupTransform("/base_link","/base_link", ros::Time(0), odom_base_transform);
		  }
		  catch(tf::TransformException &ex){
		    ROS_ERROR("%s",ex.what());
		    return false;
		  }
		  mat.setRotation(odom_base_transform.getRotation());
		  mat.getRPY(roll, pitch, yaw);
		  pose_x = odom_base_transform.getOrigin().getX();
		  pose_y = odom_base_transform.getOrigin().getY();
		  return true;
	}
	nav_msgs::OccupancyGrid DilateMap(nav_msgs::OccupancyGrid occ_grid){
		nav_msgs::OccupancyGrid temp_grid = occ_grid;
		Mat src(occ_grid.info.height,occ_grid.info.width,CV_32F);
		for(int k = 0; k<occ_grid.info.height*occ_grid.info.width;k++){
			int row = k/occ_grid.info.height;
		 	int col = k%occ_grid.info.height;
		 	if(occ_grid.data[k]== -1){
		 		// occ_grid.data[k] = -1;
				// src.at<float>(row,col) = 2; // This is for an open square (white)
                occ_grid.data[k] = 0;
                src.at<float>(row,col) = 1;
		 	}
		 	else if(occ_grid.data[k] <= 100*(base_height - 0.0)){
		 		occ_grid.data[k] = 0;
				src.at<float>(row,col) = 1; // This is for an open square (white)
		 	}
		 	else{
		 		occ_grid.data[k] = 100;
				src.at<float>(row,col) = 0; // This for a closed square (black)
		 	}
		}
		int morph_elem = MORPH_RECT;
		Mat kernalMorpDilate = getStructuringElement(morph_elem, Size( 2*dilationSize + 1, 2*dilationSize+1 ),Point(-1,-1));

		erode(src,src,kernalMorpDilate);
		for(int rows = 0; rows< occ_grid.info.height; rows++){
			for(int cols = 0; cols<occ_grid.info.width; cols++){
				int k = rows*occ_grid.info.width;
				int counter = 0;
				if(occ_grid.data[k+cols] == -1){
					counter = 0;
					if(rows>0 && src.at<float>(rows-1,cols)==1) counter++;
			        if(cols>0 && src.at<float>(rows,cols-1)==1) counter++;
			        if(rows+1<occ_grid.info.height && src.at<float>(rows+1,cols)==1) counter++;
			        if(cols+1<occ_grid.info.width && src.at<float>(rows,cols+1)==1) counter++;
			        if((rows>0 && cols>0)&& src.at<float>(rows-1,cols-1) == 1) counter++;
			        if((rows+1<occ_grid.info.height && cols+1< occ_grid.info.width) && src.at<float>(rows+1,cols+1) == 1) counter++;
			        if((cols>0 && rows+1<occ_grid.info.height) && src.at<float>(rows+1,cols-1)== 1) counter++;
	            	if((rows>0 && cols+1<occ_grid.info.width) && src.at<float>(rows-1,cols+1)== 1) counter++;
			      	if(counter >= 4){
			        	temp_grid.data[k+cols] = 0;
			      	}

				}
				else if(src.at<float>(rows,cols)==0){
					temp_grid.data[k+cols] = 100;
				}
				else{
					temp_grid.data[k+cols] = 0;// src.at<float>(rows,cols);
				}
			}
		}
		return temp_grid;
	}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "map_build_node");
  ROS_INFO("Starting CleanBuildMap.cpp");
  Map_builder mb;
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
