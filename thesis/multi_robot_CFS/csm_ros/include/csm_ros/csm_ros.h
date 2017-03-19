/*
 * csm_ros.h
 *
 *  Created on: Nov 17, 2014
 *      Author: stephen
 */

#ifndef CSM_ROS_H_
#define CSM_ROS_H_

#include <csm/csm.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf2/buffer_core.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

namespace csm_ros {

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. Only the
 * ray information is translated into the destination message. The 3D points in the
 * laser frame are populated using simple laser projection; the derived fields (world
 * points, surface normals, and odometry poses) are left empty. The data is copied
 * into the destination object, so no handle to the laser scan message is maintained.
 * The laser_data structure is allocated using 'ld_alloc_new'; it is up to the user
 * to deallocate it.
 * @param scan A ROS LaserScan message
 * @param sigma Laser measurement uncertainty
 * @param laser_inverted A flag indicating the laser is upside down (so the readings must be interpreted in the reverse order)
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan& scan, double sigma = 0.05, bool laser_inverted = false);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. Only the
 * ray information is translated into the destination message. The 3D points in the
 * laser frame are populated using simple laser projection; the derived fields (world
 * points, surface normals, and odometry poses) are left empty. The data is copied
 * into the destination object, so no handle to the laser scan message is maintained.
 * The laser_data structure is allocated using 'ld_alloc_new'; it is up to the user
 * to deallocate it.
 * @param scan A ROS LaserScan message
 * @param sigma Laser measurement uncertainty
 * @param laser_inverted A flag indicating the laser is upside down (so the readings must be interpreted in the reverse order)
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan::ConstPtr& scan, double sigma = 0.05, bool laser_inverted = false);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. In addition to the
 * ray information, the 3D points in the laser frame are populated using the high-precision
 * ROS laser projection. The points are first transformed into the provided fixed frame, then
 * transformed back into the laser frame *at the scan start time*. The derived fields (world
 * points, surface normals, and odometry poses) are left empty. The data is copied
 * into the destination object, so no handle to the laser scan message is maintained.
 * The laser_data structure is allocated using 'ld_alloc_new'; it is up to the user
 * to deallocate it.
 * @param scan A ROS LaserScan message
 * @param tf_transformer A tf::Transformer object used to perform the laserscan projection
 * @param fixed_frame A fixed frame used to temporary transform the laserscan points into
 * @param sigma Laser measurement uncertainty
 * @param laser_inverted A flag indicating the laser is upside down (so the readings must be interpreted in the reverse order)
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan::ConstPtr& scan, tf::Transformer& tf_transformer, const std::string& fixed_frame, double sigma = 0.05, bool laser_inverted = false);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. In addition to the
 * ray information, the 3D points in the laser frame are populated using the high-precision
 * ROS laser projection. The points are first transformed into the provided fixed frame, then
 * transformed back into the laser frame *at the scan start time*. The derived fields (world
 * points, surface normals, and odometry poses) are left empty. The data is copied
 * into the destination object, so no handle to the laser scan message is maintained.
 * The laser_data structure is allocated using 'ld_alloc_new'; it is up to the user
 * to deallocate it.
 * @param scan A ROS LaserScan message
 * @param tf_transformer A tf::Transformer object used to perform the laserscan projection
 * @param fixed_frame A fixed frame used to temporary transform the laserscan points into
 * @param sigma Laser measurement uncertainty
 * @param laser_inverted A flag indicating the laser is upside down (so the readings must be interpreted in the reverse order)
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan& scan, tf::Transformer& tf_transformer, const std::string& fixed_frame, double sigma = 0.05, bool laser_inverted = false);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. In addition to the
 * ray information, the 3D points in the laser frame are populated using the high-precision
 * ROS laser projection. The points are first transformed into the provided fixed frame, then
 * transformed back into the laser frame *at the scan start time*. The derived fields (world
 * points, surface normals, and odometry poses) are left empty. The data is copied
 * into the destination object, so no handle to the laser scan message is maintained.
 * The laser_data structure is allocated using 'ld_alloc_new'; it is up to the user
 * to deallocate it.
 * @param scan A ROS LaserScan message
 * @param tf2_buffer A tf2::BufferCore object used to perform the laserscan projection
 * @param fixed_frame A fixed frame used to temporary transform the laserscan points into
 * @param sigma Laser measurement uncertainty
 * @param laser_inverted A flag indicating the laser is upside down (so the readings must be interpreted in the reverse order)
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan::ConstPtr& scan, tf2::BufferCore& tf2_buffer, const std::string& fixed_frame, double sigma = 0.05, bool laser_inverted = false);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. In addition to the
 * ray information, the 3D points in the laser frame are populated using the high-precision
 * ROS laser projection. The points are first transformed into the provided fixed frame, then
 * transformed back into the laser frame *at the scan start time*. The derived fields (world
 * points, surface normals, and odometry poses) are left empty. The data is copied
 * into the destination object, so no handle to the laser scan message is maintained.
 * The laser_data structure is allocated using 'ld_alloc_new'; it is up to the user
 * to deallocate it.
 * @param scan A ROS LaserScan message
 * @param tf2_buffer A tf2::BufferCore object used to perform the laserscan projection
 * @param fixed_frame A fixed frame used to temporary transform the laserscan points into
 * @param sigma Laser measurement uncertainty
 * @param laser_inverted A flag indicating the laser is upside down (so the readings must be interpreted in the reverse order)
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const sensor_msgs::LaserScan& scan, tf2::BufferCore& tf2_buffer, const std::string& fixed_frame, double sigma = 0.05, bool laser_inverted = false);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. Only the
 * ray information is translated into the destination message. The 3D points in the
 * laser frame are populated from the supplied pointcloud. It is assumed that only
 * valid laser scan points have been populated into the pointcloud. Thus, the
 * pointcloud size may be smaller than the laser scan size. Pointclouds can be
 * generated using ROS's laser_geometry package. The derived fields (world points,
 * surface normals, and odometry poses) are left empty. The data is copied into the
 * destination object, so no handle to the laser scan message is maintained. The
 * laser_data structure is allocated using 'ld_alloc_new'; it is up to the user to
 * deallocate it.
 * @param scan A ROS LaserScan message
 * @param cloud The LaserScan points projected into 3D
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const pcl::PointCloud<pcl::PointXYZ>& cloud, double sigma = 0.05);

/**
 * Convert a ROS laser scan message into a CSM laser_data structure. Only the
 * ray information is translated into the destination message. The 3D points in the
 * laser frame are populated from the supplied pointcloud. It is assumed that only
 * valid laser scan points have been populated into the pointcloud. Thus, the
 * pointcloud size may be smaller than the laser scan size. Pointclouds can be
 * generated using ROS's laser_geometry package. The derived fields (world points,
 * surface normals, and odometry poses) are left empty. The data is copied into the
 * destination object, so no handle to the laser scan message is maintained. The
 * laser_data structure is allocated using 'ld_alloc_new'; it is up to the user to
 * deallocate it.
 * @param scan A ROS LaserScan message
 * @param cloud The LaserScan points projected into 3D
 * @return An equivalent CSM laser_data structure.
 */
struct laser_data* toCsmLaserData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, double sigma = 0.05);

/**
 * Convert a CMS laser_data structure into a ROS LaserScan message. The CSM data structure does not
 * contain timing information about the internals of a scan, so this must be provided by the user.
 * This version copies all source data into the destination object.
 * @param source A CSM laser_data structure
 * @param scan_time The nominal time between complete scans
 * @param time_increment The time between individual measurements within a scan. If time_increment <= 0, the increment is computed from the scan_time
 * @param range_min The minimum detectable range of the laser
 * @param range_max The maximum detectable range of the laser
 * @return A pointer to a ROS LaserScan message
 */
sensor_msgs::LaserScan::Ptr toLaserScanMsg(const struct laser_data* source, float scan_time, float time_increment = 0.0, float range_min = 0.0, float range_max = 30.0);

} ///@namespace csm_ros

#endif /* CSM_ROS_H_ */
