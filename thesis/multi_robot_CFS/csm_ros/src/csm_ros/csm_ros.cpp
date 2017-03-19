/*
 * csm_ros.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: stephen
 */

#include <csm_ros/csm_ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

namespace csm_ros {

/**
 *
 * @param scan_frame_cloud
 * @param destination
 */
void populateCsmPoints(const sensor_msgs::PointCloud2& scan_frame_cloud, struct laser_data* destination);

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan::ConstPtr& scan, double sigma, bool laser_inverted) {
  return toCsmLaserData(*scan, sigma, laser_inverted);
}

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan& scan, double sigma, bool laser_inverted) {
  struct laser_data* destination;

  // Calculate and validate the correct array sizes
  size_t ray_count = scan.ranges.size();

  // Allocate a new laser_data structure
  destination = ld_alloc_new(ray_count);

  // Populate the laser_data structure from the provided LaserScan message
  destination->tv.tv_sec = scan.header.stamp.sec;
  destination->tv.tv_usec = scan.header.stamp.nsec / 1000;
  memset(destination->hostname, 0, sizeof(destination->hostname)); // Clear any existing hostname before concatenating
  strncat(destination->hostname, scan.header.frame_id.c_str(), sizeof(destination->hostname) - 1); // -1 for the NULL character

  // Add the laser scan ranges to the csm data structure
  for(size_t input_index = 0; input_index < ray_count; ++input_index) {
    // Handle the case when the laser is upside down
    size_t output_index;
    double theta_multiplier;
    if(laser_inverted) {
      output_index = ray_count - input_index - 1;
      theta_multiplier = -1.0;
    } else {
      output_index = input_index;
      theta_multiplier = +1.0;
    }
    destination->theta[output_index] = theta_multiplier * (scan.angle_min + input_index*scan.angle_increment);
    destination->valid[output_index] = (scan.ranges[input_index] >= scan.range_min) && (scan.ranges[input_index] < scan.range_max);
    destination->readings_sigma[output_index] = sigma;
    destination->cluster[output_index]  = -1;
    if(destination->valid[output_index]) {
      destination->readings[output_index] = scan.ranges[input_index];
    } else {
      destination->readings[output_index] = -1;
    }
  }

  // Set the min and max angle
  destination->min_theta = destination->theta[0];
  destination->max_theta = destination->theta[ray_count - 1];

  // Clear any odometry information
  destination->odometry[0] = 0.0;
  destination->odometry[1] = 0.0;
  destination->odometry[2] = 0.0;
  destination->estimate[0] = 0.0;
  destination->estimate[1] = 0.0;
  destination->estimate[2] = 0.0;
  destination->true_pose[0] = 0.0;
  destination->true_pose[1] = 0.0;
  destination->true_pose[2] = 0.0;

  return destination;
}

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan::ConstPtr& scan, tf::Transformer& tf_transformer, const std::string& fixed_frame, double sigma, bool laser_inverted) {
  return toCsmLaserData(*scan, tf_transformer, fixed_frame, sigma, laser_inverted);
}

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan& scan, tf::Transformer& tf_transformer, const std::string& fixed_frame, double sigma, bool laser_inverted) {
  // Use the standard function to populate the laserscan data in the output object
  struct laser_data* destination = toCsmLaserData(scan, sigma, laser_inverted);

  // Use the provided tf data to correctly project the laserscan data
  try {
    // Project the laserscan into the fixed frame
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 fixed_frame_cloud;
    double range_cutoff = -1.0;
    int channel_options = laser_geometry::channel_option::Index;
    projector.transformLaserScanToPointCloud(fixed_frame, scan, fixed_frame_cloud, tf_transformer, range_cutoff, channel_options);
    // Un-project the pointcloud to be centered at the laser again
    tf::StampedTransform tf_transform;
    tf_transformer.lookupTransform(scan.header.frame_id, fixed_frame, scan.header.stamp, tf_transform);
    geometry_msgs::TransformStamped geometry_transform;
    tf::transformStampedTFToMsg(tf_transform, geometry_transform);
    sensor_msgs::PointCloud2 scan_frame_cloud;
    tf2::doTransform(fixed_frame_cloud, scan_frame_cloud, geometry_transform);
    // Populate the CSM structure with the projected points
    populateCsmPoints(fixed_frame_cloud, destination);
  } catch(const std::exception& e) {
    ROS_ERROR_STREAM("Transformation of laserscan data into frame '" << fixed_frame << "' failed.\n"
        << e.what() << "\n"
        << "Reverting to standard CSM laser projection method.");
  }

  return destination;
}

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan::ConstPtr& scan, tf2::BufferCore& tf2_buffer, const std::string& fixed_frame, double sigma, bool laser_inverted) {
  return toCsmLaserData(*scan, tf2_buffer, fixed_frame, sigma, laser_inverted);
}

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan& scan, tf2::BufferCore& tf2_buffer, const std::string& fixed_frame, double sigma, bool laser_inverted) {
  // Use the standard function to populate the laserscan data in the output object
  struct laser_data* destination = toCsmLaserData(scan, sigma, laser_inverted);

  // Use the provided tf data to correctly project the laserscan data
  try {
    // Project the laserscan into the fixed frame
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 fixed_frame_cloud;
    double range_cutoff = -1.0;
    int channel_options = laser_geometry::channel_option::Index;
    projector.transformLaserScanToPointCloud(fixed_frame, scan, fixed_frame_cloud, tf2_buffer, range_cutoff, channel_options);
    // Un-project the pointcloud to be centered at the laser again
    geometry_msgs::TransformStamped geometry_transform = tf2_buffer.lookupTransform(scan.header.frame_id, fixed_frame, scan.header.stamp);
    sensor_msgs::PointCloud2 scan_frame_cloud;
    tf2::doTransform(fixed_frame_cloud, scan_frame_cloud, geometry_transform);
    // Populate the CSM structure with the projected points
    populateCsmPoints(fixed_frame_cloud, destination);
  } catch(const std::exception& e) {
    ROS_ERROR_STREAM("Transformation of laserscan data into frame '" << fixed_frame << "' failed.\n"
        << e.what() << "\n"
        << "Reverting to standard CSM laser projection method.");
  }

  return destination;
}

/* ************************************************************************* */
struct laser_data* toCsmLaserData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, double sigma) {
  return toCsmLaserData(*cloud, sigma);
}
/* ************************************************************************* */
struct laser_data* toCsmLaserData(const pcl::PointCloud<pcl::PointXYZ>& cloud, double sigma) {
  struct laser_data* destination;

  // Calculate and validate the correct array sizes
  size_t ray_count = cloud.width*cloud.height;

  // Allocate a new laser_data structure
  destination = ld_alloc_new(ray_count);

  // Populate the laser_data structure from the provided LaserScan message and additional provided data
  destination->tv.tv_sec = (int32_t)(cloud.header.stamp / 1000000);
  destination->tv.tv_usec = (int32_t)(cloud.header.stamp % 1000000);
  memset(destination->hostname, 0, sizeof(destination->hostname)); // Clear any existing hostname before concatenating
  strncat(destination->hostname, cloud.header.frame_id.c_str(), sizeof(destination->hostname) - 1); // -1 for the NULL character

  // Add the pointcloud points to the csm data structure
  for(size_t cloud_index = 0; cloud_index < ray_count; ++cloud_index) {
    // Fill in the CSM data structure
    destination->theta[cloud_index] = std::atan2(cloud.points[cloud_index].y, cloud.points[cloud_index].x);
    destination->valid[cloud_index] = !std::isnan(cloud.points[cloud_index].x) && !std::isnan(cloud.points[cloud_index].y);
    destination->readings_sigma[cloud_index] = sigma;
    destination->cluster[cloud_index]  = -1;
    if(destination->valid[cloud_index]) {
      destination->readings[cloud_index] = std::sqrt((cloud.points.at(cloud_index).x * cloud.points.at(cloud_index).x) + (cloud.points.at(cloud_index).y * cloud.points.at(cloud_index).y));
    } else {
      destination->readings[cloud_index] = -1;
    }
  }

  // Set the min and max angle
  destination->min_theta = destination->theta[0];
  destination->max_theta = destination->theta[ray_count - 1];

  if(std::isnan(destination->theta[ray_count - 1])) {
    ROS_ERROR_STREAM("Conversion from pointcloud to CSM Scan failed. Pointcloud contains NaNs.");
  }

  // Clear any odometry information
  destination->odometry[0] = 0.0;
  destination->odometry[1] = 0.0;
  destination->odometry[2] = 0.0;
  destination->true_pose[0] = 0.0;
  destination->true_pose[1] = 0.0;
  destination->true_pose[2] = 0.0;

  return destination;
}

/* ************************************************************************* */
sensor_msgs::LaserScan::Ptr toLaserScanMsg(const struct laser_data* source, float scan_time, float time_increment, float range_min, float range_max) {
  sensor_msgs::LaserScan::Ptr destination(new sensor_msgs::LaserScan());

  destination->header.stamp.sec = source->tv.tv_sec;
  destination->header.stamp.nsec = source->tv.tv_usec * 1000;
  destination->header.frame_id = source->hostname;
  destination->angle_min = source->min_theta;
  destination->angle_max = source->max_theta;
  destination->angle_increment = (source->max_theta - source->min_theta) / (source->nrays - 1);
  destination->time_increment = (time_increment > 0) ? (time_increment) : (scan_time / source->nrays);
  destination->scan_time = scan_time;
  destination->range_min = range_min;
  destination->range_max = range_max;
  destination->ranges.reserve(source->nrays);
  for(size_t i = 0; i < source->nrays; ++i) {
    if(source->valid[i]) {
      destination->ranges.push_back(source->readings[i]);
    } else {
      destination->ranges.push_back(range_max + 1.0);
    }
  }
  // CSM does not contain intensity data

  return destination;
}

/* ************************************************************************* */
void populateCsmPoints(const sensor_msgs::PointCloud2& scan_frame_cloud, struct laser_data* destination) {
  // Find the x, y, and index channel offsets
  unsigned int x_offset, y_offset, index_offset = std::numeric_limits<unsigned int>::max();
  for(size_t i = 0; i < scan_frame_cloud.fields.size(); ++i) {
    if(scan_frame_cloud.fields.at(i).name == "index") index_offset = scan_frame_cloud.fields.at(i).offset;
    if(scan_frame_cloud.fields.at(i).name == "x") index_offset = scan_frame_cloud.fields.at(i).offset;
    if(scan_frame_cloud.fields.at(i).name == "y") index_offset = scan_frame_cloud.fields.at(i).offset;
  }
  // Check that the offset are all valid
  if((x_offset == std::numeric_limits<unsigned int>::max()) || (y_offset == std::numeric_limits<unsigned int>::max()) || (index_offset == std::numeric_limits<unsigned int>::max())) {
    throw(std::runtime_error("Supplied pointcloud does not have all required fields."));
  }
  // Populate the (x,y) points into the output object
  for(size_t row = 0; row < scan_frame_cloud.height; ++row) {
    for(size_t col = 0; col < scan_frame_cloud.width; ++col) {
      size_t point_offset = row*scan_frame_cloud.row_step + col*scan_frame_cloud.point_step;
      float x = *(float*)(&scan_frame_cloud.data.at(point_offset + x_offset));
      float y = *(float*)(&scan_frame_cloud.data.at(point_offset + y_offset));
      int index = *(int*)(&scan_frame_cloud.data.at(point_offset + index_offset));
      destination->points[index].p[0] = x;
      destination->points[index].p[1] = y;
    }
  }
}

/* ************************************************************************* */
} ///@namespace csm_ros
