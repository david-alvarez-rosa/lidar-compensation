#include "LidarCompensation.hh"




float atan(float x, float y) {
  float theta = std::atan(y/x);
  if (x >= 0 and y >= 0)
    return theta;
  if (x >= 0 and y < 0)
    return theta + 2*M_PI;
  return theta + M_PI;
}






const int FREQUENCY = 10; // LiDAR frequency (in Hz);


LidarCompensation::LidarCompensation() {
  std::cout << "LidarCompensation is being created." << std::endl;
}


LidarCompensation::~LidarCompensation() {
  std::cout << "LidarCompensation is being deleted." << std::endl;
}



void LidarCompensation::onInit(void){
  ros::NodeHandle nh = getNodeHandle();

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "/velodyne_points", 2);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/velodyne_points/filtered_final", 2);
  message_filters::Subscriber<nav_msgs::Odometry> sub3(nh, "/vectornav/Odom", 2);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub1, sub2, sub3);
  sync.registerCallback(boost::bind(&LidarCompensation::callback, this, _1, _2, _3));

  cloudCompensatedPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/lidar/cloudCompensated", 2);

  ros::spin();
}


void LidarCompensation::callback(const sensor_msgs::PointCloud2ConstPtr& rawCloud,
                                 const sensor_msgs::PointCloud2ConstPtr& rawCloudFiltered,
                                 const nav_msgs::OdometryConstPtr& odom) {
  // Get linear velocity from /vectornav/Odom.
  geometry_msgs::Vector3 velocity = odom->twist.twist.linear;

  std::cout << "v_x= " << velocity.x * 3.6 << "\t"
            << "v_y= " << velocity.y * 3.6 << "\t"
            << "v_z= " << velocity.z * 3.6 << "\t" << std::endl;

  pcl::fromROSMsg(*rawCloudFiltered, cloudFiltered);
  pcl::fromROSMsg(*rawCloud, cloud);

  // Define compensated positions.
  pcl::PointCloud<pcl::PointXYZ> cloudCompensated = cloudFiltered;

  // Special case.
  if (cloudCompensated.points.size() == 0) {
    cloudCompensatedPublisher.publish(cloudCompensated);
    return;
  }

  pcl::PointXYZ& point0 = cloud.points[0];
  float theta0 = atan(-point0.y, point0.x);
  for (int i = 1; i < int(cloudCompensated.points.size()); ++i) {
    // Compensate positions.
    pcl::PointXYZ& point = cloudCompensated.points[i];
    float theta = atan(-point.y, point.x);
    float deltaTheta = theta0 - theta;

    if (deltaTheta < 0)
      deltaTheta += 2 * M_PI;
    float time = deltaTheta / ( 2 * M_PI * FREQUENCY );

    point.x += time * velocity.x;
    point.y += time * velocity.y;
    point.z += time * velocity.z;
  }

  cloudCompensatedPublisher.publish(cloudCompensated);
}
