/*
 * csm_caller.cpp
 *
 *  Created on: Mar 19, 2017
 *      Author: Ramkumar Natarajan
 */

#include <csm_processor/csm_processor.h>
#include <time.h>

using namespace std;
    
int main(int argc, char **argv) {

	sm_params csm_params;
	sensor_msgs::LaserScan scan1, scan2;
    sensor_msgs::LaserScan firstScan, secondScan;
	gtsam::Pose2 initialGuess;
    ros::Time timer;
	int nrays = 0;

	srand(time(NULL));

//	if(argc == 1) {
//		cout << "Usage: " << argv[0] << endl;
//		cout << "-minAngle <The start angle of the scan>" << endl;
//		cout << "-maxAngle <The end angle of the scan>" << endl;
//		cout << "-deltaAngle <The angular distance between measurements>" << endl;
//		cout << "-time1 <The acquisition time of scan1>" << endl;
//		cout << "-scan1 <The range data of first scan>" << endl;
//		cout << "-time2 <The acquisition time of scan2>" << endl;
//		cout << "-scan2 <The range data of second scan>" << endl;
//		cout << "-initialGuess <The initial pose guess for scan matching>" << endl;
//
//		return 0;
//	}

//	for(int i = 1; i < argc; ++i) {
//		if(!strcmp(argv[i], "-minAngle")) {
//			scan1.angle_min = atof(argv[++i]);
//			scan2.angle_min = atof(argv[i]);
//		}
//		else if(!strcmp(argv[i], "-maxAngle")) {
//			scan1.angle_max = atof(argv[++i]);
//			scan2.angle_max = atof(argv[i]);
//		}
//		else if(!strcmp(argv[i], "-deltaAngle")) {
//			scan1.angle_increment = atof(argv[++i]);
//			scan2.angle_increment = atof(argv[i]);
//		}
//		else if(!strcmp(argv[i], "-time1")) {
//			ros::Time timer;
//			scan1.header.stamp = timer.fromSec(atof(argv[++i]));
//		}
//		else if(!strcmp(argv[i], "-time2")) {
//			ros::Time timer;
//			scan2.header.stamp = timer.fromSec(atof(argv[++i]));
//		}
//		else if(!strcmp(argv[i], "-nrays")) {
//			nrays = atoi(argv[++i]);
//		}
//		else if(!strcmp(argv[i], "-scan1")) {
//			if(nrays == 0) {
//				nrays = (int)(scan1.angle_max - scan1.angle_min)/scan1.angle_increment;
//			}
//			for(int j = i + nrays, k = 0; i < j; ++k) {
//				scan1.ranges[k] = atof(argv[++i]);
//			}
//		}
//		else if(!strcmp(argv[i], "-scan2")) {
//			if(nrays == 0) {
//				nrays = (int)(scan2.angle_max - scan2.angle_min)/scan2.angle_increment;
//			}
//			for(int j = i + nrays, k = 0; i < j; ++k) {
//				scan2.ranges[k] = atof(argv[++i]);
//			}
//		}
//		else if(!strcmp(argv[i], "-initialGuess")) {
//			initialGuess = gtsam::Pose2(atof(argv[++i]), atof(argv[++i]), atof(argv[++i]));
//		}
//  }
//    csm_processor::Pose2DWithCovariance result = csm_processor::computeLaserScanMatch(scan1, scan2, csm_params, initialGuess);

    // Create a CSM parameters structure from the supplied configuration
    csm_params.use_point_to_line_distance = true;
    csm_params.use_corr_tricks = true;
    csm_params.max_iterations = 1000;
    csm_params.max_angular_correction_deg = 1.57 * 180.0 / M_PI;
    csm_params.max_linear_correction = 2.0;
    csm_params.max_correspondence_dist = 2.0;
    csm_params.sigma = 0.05;
    csm_params.epsilon_xy = 0.0001;
    csm_params.epsilon_theta = 0.0001;
    csm_params.outliers_maxPerc = 0.95;
    csm_params.outliers_adaptive_order = 0.70;
    csm_params.outliers_adaptive_mult = 2.0;
    csm_params.outliers_remove_doubles = true;
    csm_params.do_visibility_test = false;
    csm_params.do_alpha_test = false;
    csm_params.do_alpha_test_thresholdDeg = 0.35 * 180.0 / M_PI;
    csm_params.clustering_threshold = 0.05;
    csm_params.orientation_neighbourhood = 3;
    csm_params.restart = true;
    csm_params.restart_threshold_mean_error = 0.01;
    csm_params.restart_dt = 0.01;
    csm_params.restart_dtheta = 0.025;
    csm_params.do_compute_covariance = true; // We need to covariance matrix for sensor fusion
    csm_params.use_ml_weights = false; // Use the computed alpha angle to weight the correspondences. Must compute the angle for this to work.
    csm_params.use_sigma_weights = false; // Use the "readings_sigma" field to weight the correspondences. If false, no weight is used. If all the weights are the same, this is identical to not weighting them.
    csm_params.debug_verify_tricks = false; // Do not run the debug check

    firstScan.angle_min = secondScan.angle_min = -1.57079637;
    firstScan.angle_max = secondScan.angle_max = 1.57079637;
    firstScan.angle_increment = secondScan.angle_increment = 0.0173568651;
    firstScan.range_min = secondScan.range_min = 0.20;
    firstScan.range_max = secondScan.range_max = 22;
    firstScan.header.frame_id = secondScan.header.frame_id = "base_link";
    firstScan.header.stamp = timer.fromSec(1075329512.53);
    secondScan.header.stamp = timer.fromSec(1075329512.6400001);
        for(int i = 0; i < 181; ++i) {
      firstScan.ranges.push_back(6 + ((double) rand() / (RAND_MAX)));
      secondScan.ranges.push_back(6 + ((double) rand() / (RAND_MAX)));
    }
    initialGuess = gtsam::Pose2(0.0, 0.0, 0.0);


        csm_processor::Pose2DWithCovariance result = csm_processor::computeLaserScanMatch(firstScan, secondScan, csm_params, initialGuess);
        return 0;
}
