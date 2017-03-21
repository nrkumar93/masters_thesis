/*
 * csm_caller.cpp
 *
 *  Created on: Mar 19, 2017
 *      Author: Ramkumar Natarajan
 */

#include <csm_processor/csm_processor.h>

using namespace std;
    
int main(int argc, char **argv) {

	sm_params csm_params;
	sensor_msgs::LaserScan scan1, scan2;
	gtsam::Pose2 initialGuess;
	int nrays = 0;

	if(argc == 1) {
		cout << "Usage: " << argv[0] << endl;
		cout << "-minAngle <The start angle of the scan>" << endl;
		cout << "-maxAngle <The end angle of the scan>" << endl;
		cout << "-deltaAngle <The angular distance between measurements>" << endl;
		cout << "-time1 <The acquisition time of scan1>" << endl;
		cout << "-scan1 <The range data of first scan>" << endl;
		cout << "-time2 <The acquisition time of scan2>" << endl;
		cout << "-scan2 <The range data of second scan>" << endl;
		cout << "-initialGuess <The initial pose guess for scan matching>" << endl;

		return 0;
	}

	for(int i = 1; i < argc; ++i) {
		if(!strcmp(argv[i], "-minAngle")) {
			scan1.angle_min = atof(argv[++i]);
			scan2.angle_min = atof(argv[i]);
		}
		else if(!strcmp(argv[i], "-maxAngle")) {
			scan1.angle_max = atof(argv[++i]);
			scan2.angle_max = atof(argv[i]);
		}
		else if(!strcmp(argv[i], "-deltaAngle")) {
			scan1.angle_increment = atof(argv[++i]);
			scan2.angle_increment = atof(argv[i]);
		}
		else if(!strcmp(argv[i], "-time1")) {
			ros::Time timer;
			scan1.header.stamp = timer.fromSec(atof(argv[++i]));
		}
		else if(!strcmp(argv[i], "-time2")) {
			ros::Time timer;
			scan2.header.stamp = timer.fromSec(atof(argv[++i]));
		}
		else if(!strcmp(argv[i], "-nrays")) {
			nrays = atoi(argv[++i]);
		}
		else if(!strcmp(argv[i], "-scan1")) {
			if(nrays == 0) {
				nrays = (int)(scan1.angle_max - scan1.angle_min)/scan1.angle_increment;
			}
			for(int j = i + nrays, k = 0; i < j; ++k) {
				scan1.ranges[k] = atof(argv[++i]);
			}
		}
		else if(!strcmp(argv[i], "-scan2")) {
			if(nrays == 0) {
				nrays = (int)(scan2.angle_max - scan2.angle_min)/scan2.angle_increment;
			}
			for(int j = i + nrays, k = 0; i < j; ++k) {
				scan2.ranges[k] = atof(argv[++i]);
			}
		}
		else if(!strcmp(argv[i], "-initialGuess")) {
			initialGuess = gtsam::Pose2(atof(argv[++i]), atof(argv[++i]), atof(argv[++i]));
		}

		std::string result = csm_processor::computeLaserScanMatch(scan1, scan2, csm_params, initialGuess);
	}
}
