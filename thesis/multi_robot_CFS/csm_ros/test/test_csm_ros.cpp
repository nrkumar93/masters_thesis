/**
 * test_csm_ros.cpp
 */

#include <csm_ros/csm_ros.h>
#include <gtest/gtest.h>

/* ************************************************************************* */
TEST(csm_ros, toCsmLaserData){

  // Manually create a LaserScan message
  size_t ray_count = 11;
  sensor_msgs::LaserScan::Ptr source(new sensor_msgs::LaserScan());
  source->header.stamp.sec = 100;
  source->header.stamp.nsec = 54321;
  source->header.frame_id = "scan_link";
  source->angle_min = -0.50;
  source->angle_max = +0.50;
  source->angle_increment = +0.10;
  source->time_increment = 0.01;
  source->scan_time = 0.25;
  source->range_min = 1.0;
  source->range_max = 10.0;
  source->ranges.reserve(ray_count);
  source->ranges.push_back(0.0);
  source->ranges.push_back(1.0);
  source->ranges.push_back(2.0);
  source->ranges.push_back(3.0);
  source->ranges.push_back(4.0);
  source->ranges.push_back(5.0);
  source->ranges.push_back(6.0);
  source->ranges.push_back(7.0);
  source->ranges.push_back(8.0);
  source->ranges.push_back(9.0);
  source->ranges.push_back(10.0);
  // intensity data is not used.

  // Manually create the equivalent laser_data structure
  struct laser_data* expected = ld_alloc_new(ray_count);
  expected->tv.tv_sec = 100;
  expected->tv.tv_usec = 54;
  strcpy(expected->hostname, "scan_link");
  expected->min_theta = -0.50;
  expected->max_theta = +0.50;
  expected->theta[0]  = -0.50;
  expected->theta[1]  = -0.40;
  expected->theta[2]  = -0.30;
  expected->theta[3]  = -0.20;
  expected->theta[4]  = -0.10;
  expected->theta[5]  =  0.00;
  expected->theta[6]  = +0.10;
  expected->theta[7]  = +0.20;
  expected->theta[8]  = +0.30;
  expected->theta[9]  = +0.40;
  expected->theta[10] = +0.50;
  expected->valid[0]  = 0;
  expected->valid[1]  = 1;
  expected->valid[2]  = 1;
  expected->valid[3]  = 1;
  expected->valid[4]  = 1;
  expected->valid[5]  = 1;
  expected->valid[6]  = 1;
  expected->valid[7]  = 1;
  expected->valid[8]  = 1;
  expected->valid[9]  = 1;
  expected->valid[10] = 0;
  expected->readings[0]  =  GSL_NAN;
  expected->readings[1]  =  1.0;
  expected->readings[2]  =  2.0;
  expected->readings[3]  =  3.0;
  expected->readings[4]  =  4.0;
  expected->readings[5]  =  5.0;
  expected->readings[6]  =  6.0;
  expected->readings[7]  =  7.0;
  expected->readings[8]  =  8.0;
  expected->readings[9]  =  9.0;
  expected->readings[10] =  GSL_NAN;
  expected->readings_sigma[0]  = GSL_NAN;
  expected->readings_sigma[1]  = 0.10;
  expected->readings_sigma[2]  = 0.10;
  expected->readings_sigma[3]  = 0.10;
  expected->readings_sigma[4]  = 0.10;
  expected->readings_sigma[5]  = 0.10;
  expected->readings_sigma[6]  = 0.10;
  expected->readings_sigma[7]  = 0.10;
  expected->readings_sigma[8]  = 0.10;
  expected->readings_sigma[9]  = 0.10;
  expected->readings_sigma[10] = GSL_NAN;

  // Use csm_ros to convert the LaserScan message to a laser_data structure
  struct laser_data* actual = csm_ros::toCsmLaserData(source, 0.10);

  // Compare the expected laser_data structure to the actual laser_data structure
  EXPECT_EQ(expected->nrays, actual->nrays);
  EXPECT_EQ(expected->tv.tv_sec, actual->tv.tv_sec);
  EXPECT_EQ(expected->tv.tv_usec, actual->tv.tv_usec);
  EXPECT_STREQ(expected->hostname, actual->hostname);
  EXPECT_NEAR(expected->min_theta, actual->min_theta, 1.0e-6);
  EXPECT_NEAR(expected->max_theta, actual->max_theta, 1.0e-6);
  EXPECT_EQ(sizeof(expected->theta), sizeof(actual->theta));
  EXPECT_NEAR(expected->theta[0],  actual->theta[0],  1.0e-6);
  EXPECT_NEAR(expected->theta[1],  actual->theta[1],  1.0e-6);
  EXPECT_NEAR(expected->theta[2],  actual->theta[2],  1.0e-6);
  EXPECT_NEAR(expected->theta[3],  actual->theta[3],  1.0e-6);
  EXPECT_NEAR(expected->theta[4],  actual->theta[4],  1.0e-6);
  EXPECT_NEAR(expected->theta[5],  actual->theta[5],  1.0e-6);
  EXPECT_NEAR(expected->theta[6],  actual->theta[6],  1.0e-6);
  EXPECT_NEAR(expected->theta[7],  actual->theta[7],  1.0e-6);
  EXPECT_NEAR(expected->theta[8],  actual->theta[8],  1.0e-6);
  EXPECT_NEAR(expected->theta[9],  actual->theta[9],  1.0e-6);
  EXPECT_NEAR(expected->theta[10], actual->theta[10], 1.0e-6);
  EXPECT_EQ(sizeof(expected->valid), sizeof(actual->valid));
  EXPECT_EQ(expected->valid[0],  actual->valid[0]);
  EXPECT_EQ(expected->valid[1],  actual->valid[1]);
  EXPECT_EQ(expected->valid[2],  actual->valid[2]);
  EXPECT_EQ(expected->valid[3],  actual->valid[3]);
  EXPECT_EQ(expected->valid[4],  actual->valid[4]);
  EXPECT_EQ(expected->valid[5],  actual->valid[5]);
  EXPECT_EQ(expected->valid[6],  actual->valid[6]);
  EXPECT_EQ(expected->valid[7],  actual->valid[7]);
  EXPECT_EQ(expected->valid[8],  actual->valid[8]);
  EXPECT_EQ(expected->valid[9],  actual->valid[9]);
  EXPECT_EQ(expected->valid[10], actual->valid[10]);
  EXPECT_EQ(sizeof(expected->readings), sizeof(actual->readings));
  EXPECT_TRUE(isnan(expected->readings[0])); // Unfortunately we have to have special treatment for NANs
  EXPECT_NEAR(expected->readings[1],  actual->readings[1],  1.0e-6);
  EXPECT_NEAR(expected->readings[2],  actual->readings[2],  1.0e-6);
  EXPECT_NEAR(expected->readings[3],  actual->readings[3],  1.0e-6);
  EXPECT_NEAR(expected->readings[4],  actual->readings[4],  1.0e-6);
  EXPECT_NEAR(expected->readings[5],  actual->readings[5],  1.0e-6);
  EXPECT_NEAR(expected->readings[6],  actual->readings[6],  1.0e-6);
  EXPECT_NEAR(expected->readings[7],  actual->readings[7],  1.0e-6);
  EXPECT_NEAR(expected->readings[8],  actual->readings[8],  1.0e-6);
  EXPECT_NEAR(expected->readings[9],  actual->readings[9],  1.0e-6);
  EXPECT_TRUE(isnan(expected->readings[10]));
  EXPECT_EQ(sizeof(expected->readings_sigma), sizeof(actual->readings_sigma));
  EXPECT_TRUE(isnan(expected->readings_sigma[0]));
  EXPECT_NEAR(expected->readings_sigma[1],  actual->readings_sigma[1],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[2],  actual->readings_sigma[2],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[3],  actual->readings_sigma[3],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[4],  actual->readings_sigma[4],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[5],  actual->readings_sigma[5],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[6],  actual->readings_sigma[6],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[7],  actual->readings_sigma[7],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[8],  actual->readings_sigma[8],  1.0e-6);
  EXPECT_NEAR(expected->readings_sigma[9],  actual->readings_sigma[9],  1.0e-6);
  EXPECT_TRUE(isnan(expected->readings_sigma[10]));

	// Clean up
  ld_free(expected);
  ld_free(actual);
}

/* ************************************************************************* */
TEST(csm_ros, toLaserScanMsg){

  // Manually create a CSM laser_data structure
  size_t ray_count = 11;
  struct laser_data* source = ld_alloc_new(ray_count);
  source->tv.tv_sec = 100;
  source->tv.tv_usec = 54;
  strcpy(source->hostname, "scan_link");
  source->min_theta = -0.50;
  source->max_theta = +0.50;
  source->theta[0]  = -0.50;
  source->theta[1]  = -0.40;
  source->theta[2]  = -0.30;
  source->theta[3]  = -0.20;
  source->theta[4]  = -0.10;
  source->theta[5]  =  0.00;
  source->theta[6]  = +0.10;
  source->theta[7]  = +0.20;
  source->theta[8]  = +0.30;
  source->theta[9]  = +0.40;
  source->theta[10] = +0.50;
  source->valid[0]  = 0;
  source->valid[1]  = 1;
  source->valid[2]  = 1;
  source->valid[3]  = 1;
  source->valid[4]  = 1;
  source->valid[5]  = 1;
  source->valid[6]  = 1;
  source->valid[7]  = 1;
  source->valid[8]  = 1;
  source->valid[9]  = 1;
  source->valid[10] = 0;
  source->readings[0]  =  GSL_NAN;
  source->readings[1]  =  1.0;
  source->readings[2]  =  2.0;
  source->readings[3]  =  3.0;
  source->readings[4]  =  4.0;
  source->readings[5]  =  5.0;
  source->readings[6]  =  6.0;
  source->readings[7]  =  7.0;
  source->readings[8]  =  8.0;
  source->readings[9]  =  9.0;
  source->readings[10] =  GSL_NAN;
  source->readings_sigma[0]  = GSL_NAN;
  source->readings_sigma[1]  = 0.10;
  source->readings_sigma[2]  = 0.10;
  source->readings_sigma[3]  = 0.10;
  source->readings_sigma[4]  = 0.10;
  source->readings_sigma[5]  = 0.10;
  source->readings_sigma[6]  = 0.10;
  source->readings_sigma[7]  = 0.10;
  source->readings_sigma[8]  = 0.10;
  source->readings_sigma[9]  = 0.10;
  source->readings_sigma[10] = GSL_NAN;

  // Manually create the equivalent LaserScan message
  sensor_msgs::LaserScan::Ptr expected(new sensor_msgs::LaserScan());
  expected->header.stamp.sec = 100;
  expected->header.stamp.nsec = 54000;
  expected->header.frame_id = "scan_link";
  expected->angle_min = -0.50;
  expected->angle_max = +0.50;
  expected->angle_increment = +0.10;
  expected->time_increment = 0.01;
  expected->scan_time = 0.25;
  expected->range_min = 1.0;
  expected->range_max = 10.0;
  expected->ranges.reserve(ray_count);
  expected->ranges.push_back(11.0);
  expected->ranges.push_back(1.0);
  expected->ranges.push_back(2.0);
  expected->ranges.push_back(3.0);
  expected->ranges.push_back(4.0);
  expected->ranges.push_back(5.0);
  expected->ranges.push_back(6.0);
  expected->ranges.push_back(7.0);
  expected->ranges.push_back(8.0);
  expected->ranges.push_back(9.0);
  expected->ranges.push_back(11.0);
  // intensity data is not used.

  // Use csm_ros to convert the laser_data structure to a LaserScan message
  sensor_msgs::LaserScan::Ptr actual = csm_ros::toLaserScanMsg(source, 0.25, 0.01, 1.0, 10.0);

  // Compare the expected LaserScan message to the actual LaserScan message
  EXPECT_EQ(expected->header.stamp.sec, actual->header.stamp.sec);
  EXPECT_EQ(expected->header.stamp.nsec, actual->header.stamp.nsec);
  //EXPECT_EQ(expected->header.frame_id, actual->header.frame_id); // How do I compare strings in gtest?
  EXPECT_NEAR(expected->angle_min, actual->angle_min, 1.0e-6);
  EXPECT_NEAR(expected->angle_max, actual->angle_max, 1.0e-6);
  EXPECT_NEAR(expected->angle_increment, actual->angle_increment, 1.0e-6);
  EXPECT_NEAR(expected->time_increment, actual->time_increment, 1.0e-6);
  EXPECT_NEAR(expected->scan_time, actual->scan_time, 1.0e-6);
  EXPECT_NEAR(expected->range_min, actual->range_min, 1.0e-6);
  EXPECT_NEAR(expected->range_max, actual->range_max, 1.0e-6);
  EXPECT_EQ(expected->ranges.size(), actual->ranges.size());
  EXPECT_NEAR(expected->ranges[0],  actual->ranges[0],  1.0e-6);
  EXPECT_NEAR(expected->ranges[1],  actual->ranges[1],  1.0e-6);
  EXPECT_NEAR(expected->ranges[2],  actual->ranges[2],  1.0e-6);
  EXPECT_NEAR(expected->ranges[3],  actual->ranges[3],  1.0e-6);
  EXPECT_NEAR(expected->ranges[4],  actual->ranges[4],  1.0e-6);
  EXPECT_NEAR(expected->ranges[5],  actual->ranges[5],  1.0e-6);
  EXPECT_NEAR(expected->ranges[6],  actual->ranges[6],  1.0e-6);
  EXPECT_NEAR(expected->ranges[7],  actual->ranges[7],  1.0e-6);
  EXPECT_NEAR(expected->ranges[8],  actual->ranges[8],  1.0e-6);
  EXPECT_NEAR(expected->ranges[9],  actual->ranges[9],  1.0e-6);
  EXPECT_NEAR(expected->ranges[10], actual->ranges[10], 1.0e-6);

  ///@todo: test using default parameters: csm_ros::toLaserScanMsg(source, 0.25)

  // Clean up
  ld_free(source);
}

/* ************************************************************************* */
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
