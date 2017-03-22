/*
 * csm_processor.cpp
 *
 *  Created on: Mar 19, 2017
 *      Author: Ramkumar Natarajan
 */

#include <csm_processor/csm_processor.h>
#include <csm_ros/csm_ros.h>
extern "C" {
  #include <csm/icp/icp.h>
}

namespace csm_processor {


Pose2DWithCovariance computeLaserScanMatch(
    const sensor_msgs::LaserScan& scan1,
    const sensor_msgs::LaserScan& scan2,
    struct sm_params& csm_params,
    const gtsam::Pose2& initial_pose,
    const gtsam::Pose3& base_T_laser,
    double laserscan_sigma,
    double covariance_trace_threshold,
    double initial_guess_error_threshold,
    const std::string& csm_filename)
{
  // Apply specific required parameters
  csm_params.do_compute_covariance = true; // We need to covariance matrix for sensor fusion
  csm_params.use_ml_weights = false; // Use the computed alpha angle to weight the correspondences. Must compute the angle for this to work.
  csm_params.use_sigma_weights = false; // Use the "readings_sigma" field to weight the correspondences. If false, no weight is used. If all the weights are the same, this is identical to not weighting them.
  csm_params.debug_verify_tricks = false; // Do not run the debug check


  // Set the laser transformation (and determine if it is inverted)
  double roll = base_T_laser.rotation().roll();
  double pitch = base_T_laser.rotation().pitch();
  bool laser_inverted = (roll > 3.0) || (roll < -3.0) || (pitch > 3.0) || (pitch < -3.0);
  csm_params.laser[0] = base_T_laser.x();
  csm_params.laser[1] = base_T_laser.y();
  csm_params.laser[2] = base_T_laser.rotation().yaw();

  // Transform the initial pose into laserscan coordinates
  gtsam::Pose2 first_guess;
  {
    gtsam::Pose3 map_T_base1 = gtsam::Pose3::identity();
    gtsam::Pose3 map_T_laser1 = map_T_base1*base_T_laser;
    gtsam::Pose3 map_T_base2 = gtsam::Pose3(gtsam::Rot3::Rz(initial_pose.theta()), gtsam::Point3(initial_pose.x(), initial_pose.y(), 0.0));
    gtsam::Pose3 map_T_laser2 = map_T_base2*base_T_laser;
    gtsam::Pose3 delta = map_T_laser1.between(map_T_laser2);
    first_guess = gtsam::Pose2(delta.translation().x(), delta.translation().y(), delta.rotation().yaw());
  }

  // Convert the ROS laserscan messages into CSM laser structures (Note: This allocates memory)
  /// @todo: Do I need the 'laser_inverted' flag since I'm doing the above 3D operations?

  csm_params.laser_ref  = csm_ros::toCsmLaserData(scan1, laserscan_sigma, laser_inverted);
  csm_params.laser_sens = csm_ros::toCsmLaserData(scan2, laserscan_sigma, laser_inverted);

  // Set the min and max allowed laser range
  csm_params.min_reading = std::min(scan1.range_min, scan2.range_min);
  csm_params.max_reading = std::max(scan1.range_max, scan2.range_max);
  // Calculate an initial guess based on odometry
  csm_params.first_guess[0] = first_guess.x();
  csm_params.first_guess[1] = first_guess.y();
  csm_params.first_guess[2] = first_guess.theta();
  // Also write the first guess into the odometry for debugging
  csm_params.laser_ref->odometry[0] = 0.0;
  csm_params.laser_ref->odometry[1] = 0.0;
  csm_params.laser_ref->odometry[2] = 0.0;
  csm_params.laser_sens->odometry[0] = csm_params.first_guess[0];
  csm_params.laser_sens->odometry[1] = csm_params.first_guess[1];
  csm_params.laser_sens->odometry[2] = csm_params.first_guess[2];

  // Use CSM to do the scan matching
  sm_result output;
  sm_icp(&csm_params, &output);

  // Release allocated memory
  ld_free(csm_params.laser_ref);
  ld_free(csm_params.laser_sens);

  // Check if the match was successful
  if(!output.valid) {
    throw(std::runtime_error("CSM was unable to find a valid scan match from " + boost::lexical_cast<std::string>(scan1.header.stamp.toSec()) + " to " + boost::lexical_cast<std::string>(scan2.header.stamp.toSec())));
  }

  // Transform the scan match pose back to robot coordinates
  gtsam::Pose2 relative_pose_diagnostics;
  Pose2DWithCovariance relative_pose;
  {
    gtsam::Pose3 map_T_laser1 = gtsam::Pose3::identity();
    gtsam::Pose3 map_T_base1 = map_T_laser1*(base_T_laser.inverse());
    gtsam::Pose3 map_T_laser2 = gtsam::Pose3(gtsam::Rot3::Rz(output.x[2]), gtsam::Point3(output.x[0], output.x[1], 0.0));
    gtsam::Pose3 map_T_base2 = map_T_laser2*(base_T_laser.inverse());
    gtsam::Pose3 delta = map_T_base1.between(map_T_base2);
    relative_pose_diagnostics = gtsam::Pose2(delta.translation().x(), delta.translation().y(), delta.rotation().yaw());
    relative_pose.x = delta.translation().x();
    relative_pose.y = delta.translation().y();
    relative_pose.theta = delta.rotation().yaw();
  }

  // Extract the covariance for returning and diagnostics
  gtsam::Matrix cov = gtsam::zeros(3,3);
  for(size_t m = 0; m < 3; ++m) {
    for(size_t n = 0; n < 3; ++n) {
      relative_pose.covariance[3*m + n] = gsl_matrix_get(output.cov_x_m, m, n);
      cov(m,n) = gsl_matrix_get(output.cov_x_m, m, n);
    }
  }

  // Release additional allocated memory
  gsl_matrix_free(output.cov_x_m);
  gsl_matrix_free(output.dx_dy1_m);
  gsl_matrix_free(output.dx_dy2_m);

  // Add some error detection
  double initial_guess_error = initial_pose.localCoordinates(relative_pose_diagnostics).norm();
  if(initial_guess_error > initial_guess_error_threshold) throw std::runtime_error("Scanmatch deviation from initial guess is too large.");
  if(cov.trace() > covariance_trace_threshold) throw std::runtime_error("Scanmatch covariance is too large.");

  return relative_pose;
}

}
