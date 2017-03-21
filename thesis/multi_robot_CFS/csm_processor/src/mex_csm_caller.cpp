#include "mex.h"
#include <csm_processor/csm_processor.h>

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], /* Output variables */
        int nrhs, const mxArray *prhs[]) /* Input variables */
{
    sm_params csm_params;
    sensor_msgs::LaserScan firstScan, secondScan;
    gtsam::Pose2 initialGuess;
    ros::Time timer;
    
    if(nrhs < 4) {
        mexErrMsgTxt("Wrong number of input arguments.");        
    } else if(nrhs > 9) {
        mexErrMsgTxt("Too many input arguments.");
    } else if(nrhs == 5) {
        mexPrintf("Default values for min, max angle, nrays and angle increment");
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
    
    firstScan.angle_min = secondScan.angle_min = *minAngle;
    firstScan.angle_max = secondScan.angle_max = *maxAngle;
    firstScan.angle_increment = secondScan.angle_increment = *angleIncrement;
    firstScan.header.stamp = timer.fromSec(*timestamp1);
    secondScan.header.stamp = timer.fromSec(*timestamp2);
    initialGuess = gtsam::Pose2(firstGuess[0], firstGuess[1], firstGuess[2]);
    for(int i = 0; i < (int)*nrays; ++i) {
        firstScan.ranges[i] = scan1[i];
        secondScan.ranges[i] = scan2[i];
    }
    
//     csm_processor::Pose2DWithCovariance result;
    csm_processor::Pose2DWithCovariance result = 
            csm_processor::computeLaserScanMatch(firstScan, secondScan, csm_params, initialGuess);        
    
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
