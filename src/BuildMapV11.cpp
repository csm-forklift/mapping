
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


class Map_Builder {

private:
ros::NodeHandle nh_;
ros::Subscriber submap;
ros::Publisher pubmap, pubmap1,pubmap2, bigmappub;
float ele_min_data = 0.0;
float ele_max_data = 1.0;
float ele_clearance =0.00;
double base_height;// = 0.13;
double robot_width;
double resolution;
nav_msgs::OccupancyGrid ele_occ;
nav_msgs::OccupancyGrid orig_Map;
nav_msgs::OccupancyGrid dilated_Map1;
nav_msgs::OccupancyGrid dilated_Map2;
nav_msgs::OccupancyGrid dilated_Map0;
nav_msgs::MapMetaData datz;
grid_map::GridMap map;
cv::Mat src;
cv::Mat dst;
cv::Mat dst2;
cv::Mat dst3;
Mat H; // transformation matrix
Mat image1,image2,dscv1, dscv2,image;
int offset_y = 0;
int offset_x = 0;
int new_offset_x = 0;
int new_offset_y = 0;
int old_offset_x = 0;
int old_offset_y = 0;
int dilationSize;
int starting_x, starting_y;
vector<KeyPoint> kpv1,kpv2;
vector<KeyPoint> fil1,fil2;
vector<Point2f>  coord1,coord2;
vector<DMatch>   matches;
double rotation,transx,transy,scalex,scaley;

public:
  Map_Builder(){

    pubmap = nh_.advertise<nav_msgs::OccupancyGrid>("orig_map", 1,true);
    pubmap1 = nh_.advertise<nav_msgs::OccupancyGrid>("dilation1_map1", 1,true);
    pubmap2 = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1,true);
    bigmappub = nh_.advertise<nav_msgs::OccupancyGrid>("/bigmap", 1,true);

    submap = nh_.subscribe("/mapping/ele_map_raw",1,&Map_Builder::GridMapCallback, this);
    nh_.param("base_height", base_height, 0.3);
    nh_.param("robot_width", robot_width,0.4);
    nh_.param("/resolution", resolution, 0.4);
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

    //ele_occ.header.frame_id = "/base_footprint";
    orig_Map = ele_occ;
    dilated_Map0 = orig_Map;
    dilated_Map1 = orig_Map;
    dilated_Map2 = orig_Map;

  pubmap.publish(ele_occ);
//Following loops select data from grid maps and places -1, 0, or 100 into 2D array for dilation.

   int input_occ_map[datz.width][datz.height];

     //  for (int rows = 0; rows < datz.height; rows++){
     //    for(int cols = 0; cols < datz.width; cols++){
     //    int i = rows*datz.width;
     //    if(ele_occ.data[i+cols] == -1){
     //      ele_occ.data[i+cols] = -1;
     //      input_occ_map[rows][cols] = -1;
     //      //continue;
     //    }
     //    else if(ele_occ.data[i+cols] <= 100*(base_height - ele_clearance - ele_min_data)/(ele_max_data-ele_min_data)){
     //      ele_occ.data[i+cols] = 0;
     //      input_occ_map[rows][cols] = 0;
     //    }
     //    else{
     //      ele_occ.data[i+cols] = 1;
     //      input_occ_map[rows][cols] = 1;
     //    }
     //  }

     // }
   int w = datz.width;
   int h = datz.height;
   for(int i = 0; i<h*w;i++){
    int row = i/h;
    int col = i%h;
    if(ele_occ.data[i] == -1){
          ele_occ.data[i] = -1;
          input_occ_map[row][col] = -1;
          //continue;
        }
        else if(ele_occ.data[i] <= 100*(base_height- ele_clearance - ele_min_data)/(ele_max_data-ele_min_data)){
          ele_occ.data[i] = 0;
          input_occ_map[row][col] = 0;
        }
        else{
          ele_occ.data[i] = 100;
          input_occ_map[row][col] = 100;
        }
    }


    //pubmap1.publish(dilated_Map0);
    Mat src(h,w,CV_32F);//,input_occ_map);
    for(int i= 0 ; i<h; i++){
    	for(int j = 0; j<w; j++){
    		src.at<float>(i,j) = input_occ_map[i][j];
    	}
    }

    //std::memcpy(src.data,input_occ_map, h*w*sizeof(int));

   //imshow("4", src);
  // cvWaitKey(7000);
    // for(int rows = 0; rows < h; rows++){
    //   for (int cols = 0; cols < w; cols++){
    //     = src.at<int>(rows,cols);
    //     }
    //   }
    //   std::cout << "got here 3" << '\n';


//std::cout << src.size() <<'\n';
// first time
//int operation = morph_operator + 2;
int morph_size = 50;
int morph_elem = MORPH_RECT;
// Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point(-1,-1 ) );
// //erode( src, dst, 3, element );
// dst = src.clone();
// // second time
// int morph_size2 = 3;
// Mat element2 = getStructuringElement( morph_elem,Size( 2*morph_size2 + 1, 2*morph_size2+1 ), Point(-1,-1 ) );
// morphologyEx( src, dst2, 3, element2 );
nh_.param("erode_size", dilationSize, 0);


Mat kernalMorpDilate = getStructuringElement(morph_elem, Size( 2*dilationSize + 1, 2*dilationSize+1 ),Point(-1,-1));
Mat dst3(h,w,CV_32F);//,input_occ_map);

erode(src,dst3,kernalMorpDilate);

//imshow("5", dst3);
//cvWaitKey(7000);
//third time
//int morph_size3 = 0;
//Mat element3 = getStructuringElement( morph_elem, Size( 2*morph_size3 + 1, 2*morph_size3+1 ), Point(-1,-1) );
//morphologyEx( src, dst3, 3, element3 );

// The following code takes dilated points and puts in nav message as well as gives back negative ones

  int counter = 0;
  int counter1 = 0;
  int counter2 = 0;

  // for(int rowz = 0; rowz <datz.height; rowz++){
  //   for(int colz = 0; colz<datz.width; colz++){

  //     int j = rowz*datz.width;
  //     //temp_int = i
		// /////////////////////-----------2
		// if(ele_occ.data[j+colz] == -1){
		//         counter = 0;

		//         if(rowz>0 && dst2.at<int>(rowz-1,colz)==0) counter++;
		//         if(colz>0 && dst2.at<int>(rowz,colz-1)==0) counter++;
		//         if(rowz+1<datz.height && dst2.at<int>(rowz+1,colz)==0) counter++;
		//         if(colz+1<datz.width && dst2.at<int>(rowz,colz+1)==0) counter++;
		//         if((rowz>0 && colz>0)&& dst2.at<int>(rowz-1,colz-1) == 0) counter++;
		//         if((rowz+1<datz.height && colz+1< datz.width) && dst.at<int>(rowz+1,colz+1) == 0) counter++;
		//         if((colz>0 && rowz+1<datz.height) && dst2.at<int>(rowz+1,colz-1)== 0) counter++;
		//         if((rowz>0 && colz+1<datz.width) && dst2.at<int>(rowz-1,colz+1)== 0) counter++;

		//       if(counter >= 4){
		//         dilated_Map1.data[j+colz] = 0;
		//       }

		// }

  //   if(dst2.at<int>(rowz,colz)==1){
  //           dilated_Map1.data[j+colz] = 100;
  // }else{
  //           dilated_Map1.data[j+colz] = dst2.at<int>(rowz,colz);
  //         }
  //       }
  //     }


		////////////////-----3
  for(int rowz = 0; rowz <datz.height; rowz++){
    for(int colz = 0; colz<datz.width; colz++){

      int j = rowz*datz.width;

		if(ele_occ.data[j+colz] == -1){
		        counter1 = 0;

		        if(rowz>0 && dst3.at<float>(rowz-1,colz)==0) counter1++;
		        if(colz>0 && dst3.at<float>(rowz,colz-1)==0) counter1++;
		        if(rowz+1<datz.height && dst3.at<float>(rowz+1,colz)==0) counter1++;
		        if(colz+1<datz.width && dst3.at<float>(rowz,colz+1)==0) counter1++;
		        if((rowz>0 && colz>0)&& dst3.at<float>(rowz-1,colz-1) == 0) counter1++;
		        if((rowz+1<datz.height && colz+1< datz.width) && dst3.at<float>(rowz+1,colz+1) == 0) counter1++;
		        if((colz>0 && rowz+1<datz.height) && dst3.at<float>(rowz+1,colz-1)== 0) counter++;
            if((rowz>0 && colz+1<datz.width) && dst3.at<float>(rowz-1,colz+1)== 0) counter1++;

		      if(counter1 >= 4){
		        dilated_Map2.data[j+colz] = 0;
		      }

		     }

    if(dst3.at<float>(rowz,colz)==1){
            dilated_Map2.data[j+colz] = 100;
  }else{
            dilated_Map2.data[j+colz] = dst3.at<float>(rowz,colz);
          }
        }
      }

  // for(int rowz = 0; rowz <datz.height; rowz++){
  //   for(int colz = 0; colz<datz.width; colz++){

  //     int j = rowz*datz.width;


		//  if(ele_occ.data[j+colz] == -1){
		//         counter2 = 0;

		//         if(rowz>0 && dst.at<int>(rowz-1,colz)==0) counter2++;
		//         if(colz>0 && dst.at<int>(rowz,colz-1)==0) counter2++;
		//         if(rowz+1<datz.height && dst.at<int>(rowz+1,colz)==0) counter2++;
		//         if(colz+1<datz.width && dst.at<int>(rowz,colz+1)==0) counter2++;
		//         if((rowz>0 && colz>0)&& dst.at<int>(rowz-1,colz-1) == 0) counter2++;
		//         if((rowz+1<datz.height && colz+1< datz.width) && dst.at<int>(rowz+1,colz+1) == 0) counter2++;
		//         if((colz>0 && rowz+1<datz.height) && dst.at<int>(rowz+1,colz-1)== 0) counter2++;
		//         if((rowz>0 && colz+1<datz.width) && dst.at<int>(rowz-1,colz+1)== 0) counter2++;

		//       if(counter2 >= 4){
		//        dilated_Map0.data[j+colz] = 0;
		//       }

		//      }

  //   if(dst.at<int>(rowz,colz)==1){
  //           dilated_Map0.data[j+colz] = 100;
  // }else{
  //           dilated_Map0.data[j+colz] = dst.at<int>(rowz,colz);
  //         }
  //       }
  //     }
    //std::cout <<"Position in x: " << ele_occ.info.origin.position.x << '\n';
    //std::cout <<"Position in y: " << ele_occ.info.origin.position.y << '\n';
    //pubmap1.publish(dilated_Map1);
    pubmap2.publish(dilated_Map2);
    //pubmap.publish(dilated_Map0);
    Merger(dst3);


}
void Merger(Mat &img){

  tf::TransformListener listener;
  tf::StampedTransform odom_base_transform;
  tf::Matrix3x3 mat;
  try{
    listener.waitForTransform("odom","base_link", ros::Time(0), ros::Duration(1.5));
    listener.lookupTransform("odom","base_link", ros::Time(0), odom_base_transform);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
  }
  //mat.setRotation(odom_base_transform.getRotation());
  // These robot x and y are flipped because image coornidates are x = cols and y = rows, while
  // moving "forward" for robot(in global x frame) is moving down rows in image coordinates which is y.
  int global_size = 100; // global_size*resolution = length of map side
  int robot_y = odom_base_transform.getOrigin().getX()/ele_occ.info.resolution +global_size/2;
  int robot_x = odom_base_transform.getOrigin().getY()/ele_occ.info.resolution +global_size/2;
  bool empty;
  transpose(img,img);
  if(image.empty()){
  	Size tempsize(global_size,global_size);
    image = Mat(tempsize,CV_32F);
    starting_x = robot_x;
    starting_y = robot_y;
   	empty = true;
  }
  else{
  	empty =false;
  }
  cv::Size newSize(image.cols,image.rows);

  /* if(robot_x+img.cols/2>newSize.width){
    newSize.width = robot_x+img.cols/2;
  }
  if(robot_x-img.cols/2 < 0){
    offset_x += abs(robot_x-img.cols/2);
    newSize.width += abs(robot_x-img.cols/2);
  }
  if(robot_y+img.rows/2 > newSize.height){
    newSize.height = robot_y+img.rows/2;
  }
  if(robot_y-img.rows/2<0){
    offset_y += abs(robot_y-img.rows/2);
    newSize.height += abs(robot_y-img.rows/2);
  }
*/
  float tempdata[newSize.width*newSize.height];
  std::fill_n(tempdata,newSize.width*newSize.height,-1);
  Mat temp = Mat(newSize,CV_32F,tempdata);

  /*if(image.rows != temp.rows && offset_y !=old_offset_y){
    new_offset_y = offset_y-old_offset_y;
    old_offset_y = offset_y;
  }
  else{
    new_offset_y = 0;
    old_offset_y = offset_y;
  }
  if(image.cols != temp.cols && offset_x != old_offset_x){
    new_offset_x = offset_x-old_offset_x;
  	old_offset_x = offset_x;
  }
  else{
    new_offset_x = 0;
    old_offset_x = offset_x;
  }
  std::cout <<"(offset x, offset y): " << offset_x << ", " << offset_y << '\n';
  std::cout <<"(new offset x, new offset y): " << new_offset_x << ", " << new_offset_y << '\n';
*/
  std::cout <<"(robot x, robot y): " << robot_x << ", " << robot_y << '\n';

  //imshow("image before", image);
  //cvWaitKey(7000);
  //temp.create(newSize,CV_8U, tempdata);
 // image.copyTo(temp(Rect(new_offset_y,new_offset_x,image.cols,image.rows)));
  if(!empty){
  	  image.copyTo(temp(Rect(0,0,image.cols,image.rows)));
  }
    std::cout <<"(start x, start y): " << starting_x << ", " << starting_y << '\n';

  //imshow("temp before", temp);
  //cvWaitKey(7000);
// imshow("img", img);
//   cvWaitKey(7000);
  img.copyTo(temp(Rect(max(0,robot_x-img.cols/2),max(0,robot_y-img.rows/2),img.cols,img.rows)));
  //img.copyTo(temp(Rect(robot_y-img.cols/2,robot_x-img.rows/2,img.cols,img.rows)));
  std::cout <<"Size: (temp,img, image): " << temp.size() << ", " <<img.size() << ", "<< image.size() << '\n';

  // imshow("temp after", temp);
  // cvWaitKey(7000);


  image = Mat(newSize,CV_32F,tempdata);
  image = temp.clone();//(image(Rect(0,0,temp.cols,temp.rows)));
  //imshow("image after", image);
  //cvWaitKey(7000);
  nav_msgs::OccupancyGrid bigmap;
  bigmap.info.resolution = ele_occ.info.resolution;
  bigmap.info.width = newSize.width;
  bigmap.info.height = newSize.height;
  geometry_msgs::Pose temp_pose;
  temp_pose.position.x = (-global_size/2)*ele_occ.info.resolution;
  temp_pose.position.y = (-global_size/2)*ele_occ.info.resolution;
  temp_pose.orientation.w = 1.0;
  bigmap.info.origin = temp_pose;
  bigmap.header.frame_id = "odom";
  bigmap.header.stamp = ros::Time(0);
  int w = newSize.width;
  int h = newSize.height;
  int bigdata[w*h];
  bigmap.data[w*h];
  //Point2f src_center(temp.cols/2.0F, temp.rows/2.0F);
 //Mat rot_mat = getRotationMatrix2D(src_center, angle, 90.0);
  //warpAffine(temp,temp,rot_mat,temp.size());
  cv::transpose(temp,temp);
  //flip(temp,temp,1);
  vector<signed char> a;
  for(int i=0; i< h; i++){
      for(int j=0; j<w; j++){
        int k = i*w + j;
        float tempo = temp.at<float>(i,j);
       // std::cout << tempo <<'\n';
        a.push_back(tempo);
      }
  }

  bigmap.data = a;
  //std::cout << "why here?" <<'\n';
  a.clear();
  // vector<signed char> a;
  // for(int i=0; i<w*h; i++){
  //   a.push_back(bigdata[i]);
  // }
  // //vector<signed char> a(bigdata,bigdata+temp.cols*temp.rows);
  // bigmap.data = a;
  // a.clear();
  //std::cout <<"Size: (bigdata, bigmap): " << bigdata.size() << ", " <<bigmap.data.size() << '\n';

  bigmappub.publish(bigmap);
}

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
