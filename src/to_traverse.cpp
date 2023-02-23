//General/ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
//PC2 Processing Includes
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>

class height_mapper
{
    private:
    ros::NodeHandle hm;
    ros::NodeHandle sv;

    ros::Subscriber sub;

    ros::Publisher pub1;
    ros::Publisher pub2;

    ros::CallbackQueue sp_queue;

    ros::ServiceServer server;

    public:
    tf2_ros::Buffer tfBuffer;

    height_mapper(){
        hm.setCallbackQueue(&sp_queue);

        //Initialize the publisher&subscriber objects
        pub1 = hm.advertise<pcl::PCLPointCloud2>("/ozyegin/zed2i/pc_obstacle", 1, true);
        pub2 = hm.advertise<pcl::PCLPointCloud2>("/ozyegin/zed2i/pc", 1, true);
        sub = hm.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 1, &height_mapper::pcp, this);
        
        //Initialize the service callback.
        server = sv.advertiseService("/ozyegin/services/cloud_process", &height_mapper::pcp_master, this);
        ROS_INFO("Object, complete.");
    }

    void pcp(const pcl::PCLPointCloud2ConstPtr& cloud_sensor){
        ROS_INFO("Running.");
		//Create the containers for processed clouds and processor objects .
		pcl::PCLPointCloud2Ptr cloud_voxel1(new pcl::PCLPointCloud2());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform0 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2Ptr cloud_transform2(new pcl::PCLPointCloud2());

		//Create the filtering objects.
		pcl::VoxelGrid<pcl::PCLPointCloud2> vds1;
        pcl::VoxelGrid<pcl::PointXYZ> vds2;
        pcl::PassThrough<pcl::PointXYZ> pass;

		//Apply the filter.
		vds1.setInputCloud(cloud_sensor);
		vds1.setLeafSize(0.05, 0.05, 0.1);
		vds1.filter(*cloud_voxel1);
        
        //PCLPC2 to PCLPC
        pcl::fromPCLPointCloud2(*cloud_voxel1, *cloud_transform0);

        pass.setInputCloud(cloud_transform0);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.2, 10.0);
        pass.filter(*cloud_transform0);

        //Transform the cloud_voxel into the map frame.
        pcl_ros::transformPointCloud("map", *cloud_transform0, *cloud_transform1, tfBuffer);

        //Use an iterator to append points to the cloud_restructured
        for (std::size_t idx = 0; idx < cloud_transform1->size(); ++idx){
            (*cloud_transform1)[idx].z = 0;
        }
        
        //Filter the PointCloud delicately to retrieve the desired high density areas.
        vds2.setInputCloud(cloud_transform1);
        vds2.setLeafSize(0.03, 0.03, 1.0);
        vds2.setMinimumPointsNumberPerVoxel(2);
        vds2.filter(*cloud_transform1);

        vds2.setInputCloud(cloud_transform1);
        vds2.setLeafSize(0.11, 0.11, 1.0);
        vds2.setMinimumPointsNumberPerVoxel(2);
        vds2.filter(*cloud_transform1);

        //Transform pcl::PointCloud to pcl::PCLPointCLoud2
        pcl::toPCLPointCloud2(*cloud_transform1, *cloud_transform2);

		//Publish the result and set the frame.
        cloud_transform2->header.frame_id = "map";
		pub1.publish(*cloud_transform2);
        pub2.publish(*cloud_voxel1);
    }

    bool pcp_master(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
        sp_queue.callOne(ros::WallDuration());
        ROS_INFO("Service delivered.");
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "height_mapper");
    height_mapper traveler;
    tf2_ros::TransformListener tflistener(traveler.tfBuffer);
    ros::spin();
}