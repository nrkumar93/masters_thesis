/*
 * csm_processor.h
 *
 *  Created on: Mar 19, 2017
 *      Author: Ramkumar Natarajan
 */

#ifndef CSM_PROCESSOR_H_
#define CSM_PROCESSOR_H_

#include <csm/csm.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <fstream>

namespace csm_processor {

/**
 * Use the CSM library to compute relative poses between scans
 * @param scan1 ROS laserscan message from timestamp1
 * @param scan2 ROS laserscan message from timestamp2
 * @param csm_params The CSM configuration parameters to use during scan matching
 * @param initial_guess An initial guess of the relative pose from timestamp1 to timestamp2
 * @param base_T_laser An optional frame transformation from the sensor frame to the robot frame
 * @param laser_sigma An optional measurement error estimate for the laser data
 * @param covariance_trace_threshold An outlier detection threshold. All matches that have a covaraince trace larger than this threshold will report as failed.
 * @param initial_guess_error_threshold An outlier detection threshold. All matches with a euclidean distance between the initial guess and the final match greater than the threshold will report as failed.
 * @param csm_filename An optional filename. When provided, a CSM log file will be generated.
 * @return The relative pose and covariance based on laser scan matching
 */
std::string computeLaserScanMatch(const sensor_msgs::LaserScan& scan1,
    const sensor_msgs::LaserScan& scan2,
    struct sm_params& csm_params,
    const gtsam::Pose2& initial_pose,
    const gtsam::Pose3& base_T_laser = gtsam::Pose3::identity(),
    double laserscan_sigma = 0.05,
    double covariance_trace_threshold = std::numeric_limits<double>::max(),
    double initial_guess_error_threshold = std::numeric_limits<double>::max(),
    const std::string& csm_filename = "");
}

#endif /* CSM_PROCESSOR_H_ */
