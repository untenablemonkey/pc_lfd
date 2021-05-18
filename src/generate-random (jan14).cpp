#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "PCD_file_publisher");
    ros::start();

    // get node handle
    ros::NodeHandle n;
    ros::Rate loopRate(1.0);
    std::string topicName = "cloud_topic";

    ros::Publisher demoPublisher = n.advertise<sensor_msgs::PointCloud2> (topicName.c_str(),10);

    ROS_INFO("Publishing point cloud on topic \"%s\" once every second.", topicName.c_str());

    while (ros::ok())
    {
        // create point cloud object
        pcl::PointCloud<pcl::PointXYZ> myCloud;

        // fill cloud with random points
        for (int v=0; v<1000; ++v)
        {
            pcl::PointXYZ newPoint;
            newPoint.x = 5-(rand() * 10.0) / RAND_MAX;
            newPoint.y = 5-(rand() * 10.0) / RAND_MAX;
            newPoint.z = 5-(rand() * 10.0) / RAND_MAX;
            myCloud.points.push_back(newPoint);
        }

		// Convert to ROS data type
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(myCloud, output);
		output.header.frame_id = "map";
		output.header.stamp = ros::Time::now();

        // publish point cloud
        demoPublisher.publish(output);

        // pause for loop delay
        loopRate.sleep();
    }

    return 1;
}
