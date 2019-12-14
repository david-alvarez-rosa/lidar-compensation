#ifndef LIDARCOMPENSATION_HH
#define LIDARCOMPENSATION_HH


#include <iostream> // TODO: get rid of this.

// Ros.
#include <ros/ros.h>

// Nodelet.
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Point Cloud Library.
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Time Synchronizer.
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Messages.
#include <nav_msgs/Odometry.h>

// Auxiliar functions.
#include "AuxiliarFunctions.hh"


class LidarCompensation : public nodelet::Nodelet{
public:
  ros::Publisher cloudCompensatedPublisher;
  pcl::PointCloud<pcl::PointXYZ> cloudFiltered;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  LidarCompensation();
  ~LidarCompensation();

  void onInit(void);
  void callback(const sensor_msgs::PointCloud2ConstPtr&, 
                const sensor_msgs::PointCloud2ConstPtr&, 
                const nav_msgs::OdometryConstPtr&);
};


PLUGINLIB_EXPORT_CLASS(LidarCompensation, nodelet::Nodelet)


#endif
