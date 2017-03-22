#include "mex.h"
#include <csm_processor/csm_processor.h>

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], /* Output variables */
        int nrhs, const mxArray *prhs[]) /* Input variables */
{
    sensor_msgs::LaserScan firstScan, secondScan;
    gtsam::Pose2 initialGuess;
    ros::Time timer;
    
    if(nrhs < 4) {
        mexErrMsgTxt("Wrong number of input arguments.");        
    } else if(nrhs > 11) {
        mexErrMsgTxt("Too many input arguments.");
    }
    
    double *timestamp1 = mxGetPr(prhs[0]);
    double *scan1 = mxGetPr(prhs[1]);
    double *timestamp2 = mxGetPr(prhs[2]);
    double *scan2 = mxGetPr(prhs[3]);
    double *firstGuess = mxGetPr(prhs[4]);
    double *nrays = mxGetPr(prhs[5]);
    double *minAngle = mxGetPr(prhs[6]);
    double *maxAngle = mxGetPr(prhs[7]);
    double *angleIncrement = mxGetPr(prhs[8]);
    double *range_min = mxGetPr(prhs[9]);
    double *range_max = mxGetPr(prhs[10]);
    
    firstScan.angle_min = secondScan.angle_min = *minAngle;
    firstScan.angle_max = secondScan.angle_max = *maxAngle;
    firstScan.angle_increment = secondScan.angle_increment = *angleIncrement;
    firstScan.range_min = secondScan.range_min = *range_min;
    firstScan.range_max = secondScan.range_max = *range_max;
    firstScan.header.stamp = timer.fromSec(*timestamp1);
    secondScan.header.stamp = timer.fromSec(*timestamp2);
    initialGuess = gtsam::Pose2(firstGuess[0], firstGuess[1], firstGuess[2]);
    for(int i = 0; i < (int)*nrays; ++i) {
        firstScan.ranges.push_back(scan1[i]);
        secondScan.ranges.push_back(scan2[i]);
    }
    
    // Create a CSM parameters structure from the supplied configuration
    sm_params csm_params;
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
    
    
    csm_processor::Pose2DWithCovariance result = 
            csm_processor::computeLaserScanMatch(firstScan, secondScan, csm_params, initialGuess);
        
    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(9, 1, mxREAL);    
    
    double *T = mxGetPr(plhs[0]);
    double *theta = mxGetPr(plhs[1]);
    double *covariance = mxGetPr(plhs[2]);
    
    T[0] = result.x;
    T[1] = result.y;
    
    theta[0] = result.theta;
    
    for(int i = 0; i < 9; ++i) {
        covariance[i] = result.covariance[i];
    }
    
    return;
}
