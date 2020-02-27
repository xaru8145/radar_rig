#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

int main (int argc, char** argv)
{

  ros::init(argc, argv, "generate_pcl");
  ros::NodeHandle n;

  //ros::Publisher cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloud_pcl", 100);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_pcl", 100);
  ros::Rate loop_rate(10);
  // get your point cloud message from somewhere
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 cloud;
  //sensor_msgs::PointCloud2 output_cloud;
  cloud.width  = 9;
  cloud.height = 1;
  cloud.point_step = 12;
  cloud.row_step = 108;
  cloud.is_dense = 0;
  cloud.header.frame_id = "andromeda";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3,"x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32);

  // = [
  //  PointField('x', 0, PointField.FLOAT32, 1),
  ///  PointField('y', 4, PointField.FLOAT32, 1),
    //PointField('z', 8, PointField.FLOAT32, 1),
    //PointField('intensity', 12, PointField.FLOAT32, 1),
    //PointField('range', 16, PointField.FLOAT32, 1),
    //PointField('doppler', 20, PointField.FLOAT32, 1),
    //];


  while (ros::ok())
  {
    cloud.header.stamp = ros::Time::now();
    cloud.data.clear();
    for (int i = 0; i < 108; ++i)
    {

      int a = rand() % 256; //1024 * rand () / (RAND_MAX + 1.0f);
      std::cout << a << std::endl;
      cloud.data.push_back( a );
    }

    // Fill in the cloud data
    //cloud->width  = 5;
    //cloud->height = 1;
    //cloud->points.resize (cloud->width * cloud->height);

    //for (std::size_t i = 0; i < cloud->points.size (); ++i)
    //{
    //  cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //  cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //  cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    //}
    std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(cloud, output_cloud, indices);
    pub.publish(cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}
