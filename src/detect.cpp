#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <random>
#include <visualization_msgs/Marker.h>


const float search_granularity = 0.1; //50 mm

ros::Publisher cloud_pub;
ros::Publisher marker_pub;

visualization_msgs::Marker marker;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	//sensor_msgs::PointCloud2 input_cloud;
	pcl::PointCloud<pcl::PointXYZ> input_cloud;	

	pcl::fromROSMsg(*cloud, input_cloud);
//ROS_INFO("cloud size: %lu",input_cloud.size());

	for(int search_count = 0; search_count <1; search_count++)
	{

		//pick a random point to start search
		std::random_device rand_dev;
		std::mt19937 generator(rand_dev());
		std::uniform_int_distribution<int> distr(0, input_cloud.size());

		unsigned long int starting_point = distr(generator);

		//ROS_INFO("num generated: %lu",starting_point);

		//TODO: check if this point/area has already been searched
		
		//remove the point (and surrounding points) from the cloud
		//try different orientations of a larger volume around the point, to see if you can find one which is clear
		
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "starting_point";
		marker.id = search_count;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = input_cloud.points[starting_point].x;
		marker.pose.position.y = input_cloud.points[starting_point].y;
		marker.pose.position.z = input_cloud.points[starting_point].z;
		marker.pose.orientation.x = 0;//qfinal.x();
		marker.pose.orientation.y = 0;//qfinal.y();
		marker.pose.orientation.z = 0;//qfinal.z();
		marker.pose.orientation.w = 1;//qfinal.w();
		marker.scale.x = search_granularity;//max_pt.x - min_pt.x;
		marker.scale.y = search_granularity;//max_pt.y - min_pt.y;
		marker.scale.z = search_granularity;//max_pt.z - min_pt.z;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 0.5;
		marker.lifetime = ros::Duration();
		marker_pub.publish(marker);

		//check surrounding area in a 3x3x3 grid, excluding center (starting point)
		marker.header.stamp = ros::Time::now();
		marker.ns = "search_grid";
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		for(int gridx = -1; gridx <= 1; gridx++)
		{
			for(int gridy = -1; gridy <= 1; gridy++)
			{
				for(int gridz = -1; gridz <= 1; gridz++)
				{
					//check if there's points in the cube
					if((gridx == 0) && (gridy == 0) && (gridz == 0)) //to skip the center
					{
						//don't search, this is starting block and center of search grid
					}
					else
					{
						marker.header.stamp = ros::Time::now();
						marker.id++;
						marker.pose.position.x = input_cloud.points[starting_point].x+search_granularity*gridx;
						marker.pose.position.y = input_cloud.points[starting_point].y+search_granularity*gridy;
						marker.pose.position.z = input_cloud.points[starting_point].z+search_granularity*gridz;
						marker_pub.publish(marker);						
					}
				}
			}
		}


	}
	ROS_INFO("Done searching");
  	//cloud_pub.publish (input_cloud);
}

int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "pc_lfd_detector");
    ros::start();

    // get node handle
    ros::NodeHandle n;
    std::string topicName = "cloud_topic";

	//Subscribers
	ros::Subscriber cloud_sub = n.subscribe(topicName.c_str(), 1, cloud_cb);

	//Publishers
	cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_debug", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("marker", 100);


    ROS_INFO("Subscribed to clouds on topic \"%s\".", topicName.c_str());



	ros::spin();
}
