#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <random>

int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "PCD_file_publisher");
    ros::start();

    // get node handle
    ros::NodeHandle n;
    //ros::Rate loopRate(1.0);
    ros::Rate loopRate(1);
    std::string topicName = "cloud_topic";

    ros::Publisher demoPublisher = n.advertise<sensor_msgs::PointCloud2> (topicName.c_str(),10);

    ROS_INFO("Publishing point cloud on topic \"%s\" once every second.", topicName.c_str());

	const double FEATURE_DIAMETER = 0.05;

	double feature_path_coordinates[3] = {0,0,0};
	double feature_radius = FEATURE_DIAMETER/2;//0.025; //meters
	double radius_noise = 0.00005;
	double path_variation_noise = 0.0005;
	double radius_change = 0;
	double path_change = 0;
	double feature_path_end[3] = {0,0,0};

  	std::default_random_engine generator;
  	std::normal_distribution<double> radius_distribution(0,radius_noise);
  	std::normal_distribution<double> path_variation_distribution(0,path_variation_noise);
  	std::normal_distribution<double> rotation_distribution(0,3.1415);
  	std::normal_distribution<double> translation_distribution(0,4);

    pcl::PointCloud<pcl::PointXYZ> bigCloud;

	while (ros::ok())
    {
        // create point cloud object
        pcl::PointCloud<pcl::PointXYZ> myCloud;


		for(int i = 0; i<200; ++i)
		{
		    // draw the cross section
			int num_points_in_section = ((feature_radius/0.05)*20);
			if (num_points_in_section < 0){num_points_in_section *= -1;}
		    for (int v=0; v<num_points_in_section; ++v)
		    {
		        pcl::PointXYZ newPoint;
				newPoint.x = feature_path_coordinates[0] + feature_radius*cos((2*3.1415)*((float)v/num_points_in_section));
				newPoint.y = feature_path_coordinates[1] + feature_radius*sin((2*3.1415)*((float)v/num_points_in_section));
				newPoint.z = feature_path_coordinates[2];

		       // newPoint.x = 5-(rand() * 10.0) / RAND_MAX;
		       // newPoint.y = 5-(rand() * 10.0) / RAND_MAX;
		       // newPoint.z = 5-(rand() * 10.0) / RAND_MAX;
		        myCloud.points.push_back(newPoint);
		    }

			feature_path_coordinates[2] += 0.01;
			radius_change += radius_distribution(generator);
			feature_radius += radius_change;
			//feature_radius = feature_radius - (feature_radius-0.05);
			//ROS_INFO("feature radius: %f", feature_radius);
		 	//feature_radius = 0.05; //meters

			path_change += path_variation_distribution(generator);
			//if(path_change < 0){path_change *=-1;}
			//path_change = path_change - path_change*path_change;
			feature_path_coordinates[0] += path_change;
			feature_path_coordinates[1] += path_change;
		}
		feature_path_coordinates[0]=0;
		feature_path_coordinates[1]=0;
		feature_path_coordinates[2]=0;
		feature_radius = FEATURE_DIAMETER/2;
		radius_change = 0;
		path_change = 0;

//ROS_INFO("# points in cloud: %lu:",myCloud.size());

  		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		// Define a translation.
		float x = translation_distribution(generator);
		float y = translation_distribution(generator);
		float z = translation_distribution(generator);
  		transform.translation() << x, y, z;

		// The same rotation matrix as before; theta radians around Z axis
		transform.rotate (Eigen::AngleAxisf (rotation_distribution(generator), Eigen::Vector3f::UnitZ()));
		transform.rotate (Eigen::AngleAxisf (rotation_distribution(generator), Eigen::Vector3f::UnitY()));
		transform.rotate (Eigen::AngleAxisf (rotation_distribution(generator), Eigen::Vector3f::UnitX()));

	    // Executing the transformation
	    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	    pcl::transformPointCloud (myCloud, *transformed_cloud, transform);

		bigCloud += *transformed_cloud;

ROS_INFO("# points in cloud: %lu:",bigCloud.size());

		// Convert to ROS data type
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(bigCloud, output);
		output.header.frame_id = "map";
		output.header.stamp = ros::Time::now();

        // publish point cloud
        demoPublisher.publish(output);
	    ROS_INFO("published cloud");

        // pause for loop delay
        loopRate.sleep();
    }

    return 1;
}
