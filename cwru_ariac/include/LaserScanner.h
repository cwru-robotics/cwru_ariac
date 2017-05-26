// RN
// Laser Scanner Utility Class
// 15/5/17

#ifndef CWRU_ARIAC_LASERSCANNER_H
#define CWRU_ARIAC_LASERSCANNER_H

#include <LaserScanner.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>

#include <AriacBase.h>
#include <ariac_xform_utils/ariac_xform_utils.h>
#include <Eigen/Eigen>

using namespace std;
using namespace cv;

class LaserScanner: public AriacBase {

public:
	LaserScanner(ros::NodeHandle &nodeHandle);

	PartList conveyor_partlist;
	Part latest_part;

	void part_identification(cv::Mat dewarped_mat, int& type_id);

	vector<float> uni_vec(const vector<float>& vec_0, const vector<float>& vec_1);

	vector<float> cross_product(const vector<float>& vec_1, const vector<float>& vec_2);

	void integrate_info(const int& part_id, const vector<float>& origin, const vector<float>& pin_origin, const ros::Time& ros_t_stamp, const float& belt_speed);

	void type_a_stamped_center(cv::Mat dewarped_mat_c, float& t_0, float& t_1, float& t_stamp, ros::Time& ros_t_0, ros::Time& ros_t_1, ros::Time& ros_t_stamp, vector<float>& origin, vector<int>& origin_pixel, int& part_id);

	void type_a_asymmetric(cv::Mat dewarped_mat_a, float scan_height, vector<float>& pin_origin, vector<int>& origin_pixel, int part_id);

	// MK1 obsolete
	// void type_b_stamped_center(cv::Mat dewarped_mat_c, float& t_0, float& t_1, float& t_stamp, ros::Time& ros_t_0, ros::Time& ros_t_1, ros::Time& ros_t_stamp, vector<float>& origin, vector<int>& origin_pixel);
	// void type_b_asymmetric(cv::Mat dewarped_mat_a, float scan_height, vector<float>& pin_origin, vector<int>& origin_pixel, int part_id);

	// MK2 current
	void type_b_asymmetric(cv::Mat dewarped_mat_a, float scan_height, vector<float>& pin_origin, vector<int>& pin_pixel, int part_id);

	void type_b_stamped_center(cv::Mat dewarped_mat_c, float& t_0, float& t_1, float& t_stamp, ros::Time& ros_t_0, ros::Time& ros_t_1, ros::Time& ros_t_stamp, vector<float>& origin, vector<int>& pin_pixel, vector<int>& origin_pixel);


	void ForceUpdate();

	void publish_part();

protected:
	ros::NodeHandle nh;
	ros::Subscriber belt_speed_subscriber;
	ros::Subscriber scanner_2_subscriber;
	ros::Publisher part_publisher;

	int output_flag = 1;


	void laserCallback(const sensor_msgs::LaserScan& laser_scan);

	void belt_speed_cb(const std_msgs::Float64& msg);

	float ping_dist_= 0.0;
	int ping_index_min_= -1;
	int ping_index_max_ = -1;
	int center_index_ = -1;
	double phi_ = 0.0;
	double phi_temp;
	float z_  = 0.0;
	double angle_min_=0.0;
	double angle_max_=0.0;
	double angle_increment_=0.0;
	double range_min_ = 0.0;
	double range_max_ = 0.0;
	bool part_on_belt_ = false;
	bool part_detected_=false;
	bool prev_part_on_belt_ = false;

	std::vector<bool> vec_part_on_belt;
	float belt_depth_ = 1.6;
	double tolerance_ = 0.005;
	// laser_scanner::BeltImage part_;
	int item_count = 0;
	std::vector<float> row_z;
	std::vector<float> row_z_prev_0;
	std::vector<float> row_z_prev_1;
	std::vector<float> row_z_prev_2;
	std::vector<float> row_z_prev_3;
	float time_0 = 0;
	float time0_0 = 0;
	float time0_1 = 0;
	float time0_2 = 0;
	float time0_3 = 0;
	float time_1 = 0;
	ros::Time ros_time_0;
	ros::Time ros_time0_0;
	ros::Time ros_time0_1;
	ros::Time ros_time0_2;
	ros::Time ros_time0_3;
	ros::Time ros_time_1;

	std::vector<std::vector<float> > image_z;
	int image_x_size;
	int image_y_size;

	float part_z_max = 0.028; //Disk part's height + tolorance
	float scan_width_ = 0;
	ros::Publisher scan_publisher_;
	float belt_speed = 0;
	float speed_temp_0 = 0;
	float speed_temp_1 = 0;
	// float rod_sqr_l = 0.052; // 0.052 x 0.059
	float rod_sqr_l = 0.0556; // 0.052 x 0.059 rectangle diagonal

	const float ht_disk = 0.02316;
	const float ht_gasket = 0.02;
	const float ht_gear = 0.012;
	const float ht_piston_rod = 0.0075;

	std::vector<float> laser_profiler_2_origin; // RN's own config TODO update to CWRU's config

	//For ForceUpdate();
	int updateCount_cb_speed;
	int checkedCount_cb_speed;
	int updateCount_cb_scan;
	int checkedCount_cb_scan;

	// For LaserScanner::type_b_stamped_center()
	// int square_l;
	// int window_r;
	// float window_r_float;
	// std::vector<float> theta;
	// std::vector<int> width_delta;
	// vector<float> debug;
	// int temp_b_c;


};

#endif //CWRU_ARIAC_LASERSCANNER_H
