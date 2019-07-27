#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
class PCLMerger{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber pcl_sub;
		ros::Publisher cloud_pub;
		sensor_msgs::PointCloud2 current_cloud,old_cloud, down_old_cloud;
		Eigen::Affine3d transformationSensorToOdom_;
		tf::TransformListener transformListener_;
		tf::StampedTransform transformTf;
        int num_scans;
        int counter =0;
		int seq = 0;
	public:
		PCLMerger() : nh_("~") {
            nh_.param<int>("num_scans", num_scans, 31);
			pcl_sub = nh_.subscribe("segmented_scan", 1,&PCLMerger::PCLCallback, this);
            cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("single_scan",4);
		}
		void PCLCallback(const sensor_msgs::PointCloud2& msg){
			pcl::PCLPointCloud2 pcl_pc;
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);

            pcl_conversions::toPCL(msg, pcl_pc);
            pcl_conversions::fromPCL(pcl_pc,current_cloud);
            if(counter==0){
                old_cloud = current_cloud;
            	old_cloud.header.seq = seq;
            	counter++;
            }
            else if(counter<=num_scans){
             	counter++;
                sensor_msgs::PointCloud2 temp_cloud;
             	pcl::concatenatePointCloud(current_cloud,old_cloud,temp_cloud);
                old_cloud = temp_cloud;
            }
            else{
              	old_cloud.header.frame_id = "lidar0_link";
            	old_cloud.header.seq = old_cloud.header.seq + 1;
            	old_cloud.header.stamp = ros::Time::now();
              	cloud_pub.publish(old_cloud);
              	counter=0;
            }
	  		/*if(TransformPointCloud(pointCloud, pointCloudTransformed,"/odom")){
	  			ROS_INFO("Transform Complete!");

            }
	  		else{
	  			ROS_INFO("Failure to Transform :( ");
	  		}

			pcl::toPCLPointCloud2(*pointCloudTransformed,pcl_pc);
			pcl_conversions::fromPCL(pcl_pc,current_cloud);

			if(seq != 0){
                std::cout<<old_cloud.data.size()<<'\n';
                sensor_msgs::PointCloud2 temp_cloud;
		  		pcl::concatenatePointCloud(current_cloud,old_cloud,temp_cloud);
                pcl::PointCloud<pcl::PointXYZ>::Ptr RawOldCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PCLPointCloud2::Ptr pointCloudDownSampled(new pcl::PCLPointCloud2 ());
                pcl::PCLPointCloud2::Ptr pcl_pc_ptr(new pcl::PCLPointCloud2 ());
                pcl_conversions::toPCL(temp_cloud, pcl_pc);
    	  		//pcl::fromPCLPointCloud2(pcl_pc, *RawOldCloud);
                // DOWN SAMPLE HERE!!!!!
                *pcl_pc_ptr = pcl_pc;
                if(DownSample(pcl_pc_ptr,pointCloudDownSampled)){
                    //pcl::toPCLPointCloud2(*pointCloudDownSampled,pcl_pc);
                    pcl_conversions::fromPCL(*pointCloudDownSampled,temp_cloud);
                    temp_cloud.header.frame_id = "/odom";
    				temp_cloud.header.seq = old_cloud.header.seq + 1;
    				temp_cloud.header.stamp = ros::Time::now();
                    cloud_pub.publish(temp_cloud);
                    old_cloud = temp_cloud;

                }
			}
			else{
				seq++;
				old_cloud = current_cloud;
				old_cloud.header.frame_id = "/odom";
				old_cloud.header.seq = seq;
				old_cloud.header.stamp = ros::Time::now();
                cloud_pub.publish(old_cloud);

			}*/
		}
		bool TransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudTransformed, const std::string& targetFrame){
            transformListener_.waitForTransform("odom", "os1", ros::Time(0), ros::Duration(0.1));
            transformListener_.lookupTransform("odom", "os1", ros::Time(0), transformTf);
    		poseTFToEigen(transformTf, transformationSensorToOdom_);
			pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transformationSensorToOdom_.cast<float>());
    		pointCloudTransformed->header.frame_id = targetFrame;
  			ROS_DEBUG("Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
            ros::Time(pointCloudTransformed->header.stamp).toSec());
    		return true;
        }
        bool DownSample(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr pointCloudDown){
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter(*pointCloudDown);

            return true;
        }
};

int main(int argc, char** argv){
	ros::init(argc,argv,"lidar_merge_node");
	ROS_INFO("Initializing LIDAR Merger Node");
	PCLMerger pcl_merge;
	ros::spin();
	return 0;
}
