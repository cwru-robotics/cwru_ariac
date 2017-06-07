// created by RN 
// 10/5/17

#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <vector> 
// #include <laser_scanner/BeltImage.h>
// #include <laser_scanner/Row.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "std_msgs/String.h"


using namespace std;

// a bundle from the laser scanner node

	// these values to be set within the laser callback
	float ping_dist_l1= 0.0;
	float ping_dist_l2= 0.0;
	int ping_index_min_l1 = -1;
	int ping_index_min_l2 = -1;
	int ping_index_max_ = 399; 
	int center_index_ = -1;
	double phi_ = 0.0;
	double phi_temp;
	float z_  = 0.0;
	double angle_min_=0.0;
	double angle_max_=0.0;
	double angle_increment_=0.0;
	double range_min_ = 0.0;
	double range_max_ = 0.0;
	bool part_on_belt_l1 = false;
	bool part_on_belt_l2 = false;
	bool part_detected_=false;
	bool prev_part_on_belt_ = false;
	float belt_depth_ = 1.6; 
	double tolerance_ = 0.005;
	std::vector<bool> vec_part_on_belt_l1(3);
	std::vector<bool> vec_part_on_belt_l2(3);
	// laser_scanner::BeltImage part_;
	int item_count_l1 = 0;
	int item_count_l2 = 0;

	std::vector<float> row_z_prev_0(1);
	std::vector<float> row_z_prev_1(1);
	std::vector<float> row_z_prev_2(1);
	std::vector<float> row_z_prev_3(1);
	std::vector<std::vector<float> > image_z;
	int image_x_size;
	int image_y_size;
	// float part_z_max = 0.017; //gear part
	// float part_z_max = 0.62; //pulley
	float part_z_max = 0.0595; //disk part

// .. a bundle from the laser scanner node

std::vector<float> t_temp_l1(1);
std::vector<float> t_temp_l2(1);
std::vector<float> t0_stack(100, 0);
std::vector<float> t1_stack(100, 0);

int row_count_l1 = 0;
int row_count_l2 = 0;
bool lock_l1 = 0;
bool lock_l2 = 0;

double scanner_dist = 1.0; // DIFFERENT CONFIG HAS DIFFERNT DISTANCE BETWEEN THE 2 SCANNERS.
double temp_speed = 0.0;

void laser_1_Callback(const sensor_msgs::LaserScan& laser_scan) {
	// laser_scanner::Row row;

    	part_on_belt_l1 = false;

    	if (ping_index_min_l1<0)  {
		//for first message received, set up the desired index of LIDAR range to eval
		angle_min_ = laser_scan.angle_min;	//minimum possible angle
		angle_max_ = laser_scan.angle_max;	//maximum possible angle
		angle_increment_ = laser_scan.angle_increment;	//angle difference between measurements
		range_min_ = laser_scan.range_min;	//minimum measurable distance
		range_max_ = laser_scan.range_max;	//maximum measurable distance
		center_index_ = (int) ((0.0 -angle_min_)/angle_increment_); //index of center of arc
		ping_index_min_l1 = 0;	//set starting index
		//ping_index_max_ = (int) (angle_max_ - angle_min_)/angle_increment_;	//set maximum index //THIS WILL CREATE CHAOS
		ping_index_max_ = 399;
		float belt_depth_temp = 0;
		int n2 = 1;
		for (int n = center_index_-5; n<center_index_+5; n++){
		
			phi_temp = (n - ping_index_min_l1)*angle_increment_+angle_min_;
			belt_depth_temp = belt_depth_temp + laser_scan.ranges[n]*cos(phi_temp); //distance from laser scanner to belt 
		
			belt_depth_ = belt_depth_temp/n2;
			n2++;  
	}

        //ROS_INFO("Scanner setup complete: Belt Depth is %f",belt_depth_);


    	} else {
    	//loop through all of the measured angles
	    
	    for (int i = ping_index_min_l1; i <= ping_index_max_; i++){
	    	ping_dist_l1 = laser_scan.ranges[i];    //store ith distance
	    	phi_ = (i - ping_index_min_l1)*angle_increment_+angle_min_;    //store ith angle
            	z_ = ping_dist_l1*cos(phi_);  //convert from polar to height above belt
		if (z_ >1) {z_ = belt_depth_;}
		if (z_ <0.1) {z_ = 1;}

	    	if (z_ < (belt_depth_ - tolerance_)){ //if something is measured at a height higher than the belt
	    		part_on_belt_l1 = true;    //flag that a part is currently on the belt
	    	}
	    }
	}

	if (part_on_belt_l1 || vec_part_on_belt_l1[1] || vec_part_on_belt_l1[0]){    //if there is a part on the belt or there was a part 1 or 2 steps ago
		// ROS_INFO("part on belt..");

    		//Initialise the vector for double confirmation
		vec_part_on_belt_l1[0] = vec_part_on_belt_l1[1]; //vec[0] stores the 2 steps back state
		vec_part_on_belt_l1[1] = vec_part_on_belt_l1[2]; //vec[1] store the 1 step back state
		vec_part_on_belt_l1[2] = part_on_belt_l1; //vec[2] store the current state

		if (vec_part_on_belt_l1[0] == false && vec_part_on_belt_l1[1] == true && vec_part_on_belt_l1[2] == true) {
	   		ROS_INFO("An object is entering Scanner 1's range..");
			t_temp_l1[0] = (ros::Time::now().toSec() ); 
			
			row_count_l1++;

		} else if ((vec_part_on_belt_l1[0] == true && vec_part_on_belt_l1[1]== true && vec_part_on_belt_l1[2]== true && lock_l1==0 )||(vec_part_on_belt_l1[0] == true && vec_part_on_belt_l1[1]== true && vec_part_on_belt_l1[2]== false && lock_l1==0)) {
				// ROS_INFO("The object is passing Scanner 1..");
				row_count_l1++;
				// This mechanism allows us to know the speed before the part has completely passed the scanner.
            if (row_count_l1 > 6) {
			
			    			item_count_l1 = item_count_l1 + 1;
						t0_stack[item_count_l1] = t_temp_l1[0];
						cout << "\033[1;31mScanner 1 LOCKED..\033[0m\n";
						row_count_l1 = 0; //reset
						lock_l1 = 1;
				}
			
			} else if (vec_part_on_belt_l1[0] == true && vec_part_on_belt_l1[1]== false && vec_part_on_belt_l1[2]== false){

	   			ROS_INFO("The object has passed Scanner 1.. ");
				cout << "\033[1;34mUnlocking scanner 1..\033[0m\n";
				lock_l1 = 0;
	    			} 
		
	} 


} 





void laser_2_Callback(const sensor_msgs::LaserScan& laser_scan) {
	// laser_scanner::Row row;

    	part_on_belt_l2 = false;

    	if (ping_index_min_l2<0)  {
		//for first message received, set up the desired index of LIDAR range to eval
		angle_min_ = laser_scan.angle_min;	//minimum possible angle
		angle_max_ = laser_scan.angle_max;	//maximum possible angle
		angle_increment_ = laser_scan.angle_increment;	//angle difference between measurements
		range_min_ = laser_scan.range_min;	//minimum measurable distance
		range_max_ = laser_scan.range_max;	//maximum measurable distance
		center_index_ = (int) ((0.0 -angle_min_)/angle_increment_); //index of center of arc
		ping_index_min_l2 = 0;	//set starting index
		//ping_index_max_ = (int) (angle_max_ - angle_min_)/angle_increment_;	//set maximum index //THIS WILL CREATE CHAOS
		ping_index_max_ = 399;
		float belt_depth_temp = 0;
		int n2 = 1;
		for (int n = center_index_-5; n<center_index_+5; n++){
		
			phi_temp = (n - ping_index_min_l2)*angle_increment_+angle_min_;
			belt_depth_temp = belt_depth_temp + laser_scan.ranges[n]*cos(phi_temp); //distance from laser scanner to belt 
		
			belt_depth_ = belt_depth_temp/n2;
			n2++;  
	}

        //ROS_INFO("Scanner setup complete: Belt Depth is %f",belt_depth_);


    	} else {
    	//loop through all of the measured angles
	    
	    for (int i = ping_index_min_l2; i <= ping_index_max_; i++){
	    	ping_dist_l2 = laser_scan.ranges[i];    //store ith distance
	    	phi_ = (i - ping_index_min_l2)*angle_increment_+angle_min_;    //store ith angle
            	z_ = ping_dist_l2*cos(phi_);  //convert from polar to height above belt
		if (z_ >1) {z_ = belt_depth_;}
		if (z_ <0.1) {z_ = 1;}

	    	if (z_ < (belt_depth_ - tolerance_)){ //if something is measured at a height higher than the belt
	    		part_on_belt_l2 = true;    //flag that a part is currently on the belt
	    	}
	    }
	}

	if (part_on_belt_l2 || vec_part_on_belt_l2[1] || vec_part_on_belt_l2[0]){    //if there is a part on the belt or there was a part 1 or 2 steps ago
		// ROS_INFO("part on belt..");

    		//Initialise the vector for double confirmation
		vec_part_on_belt_l2[0] = vec_part_on_belt_l2[1]; //vec[0] stores the 2 steps back state
		vec_part_on_belt_l2[1] = vec_part_on_belt_l2[2]; //vec[1] store the 1 step back state
		vec_part_on_belt_l2[2] = part_on_belt_l2; //vec[2] store the current state

		if (vec_part_on_belt_l2[0] == false && vec_part_on_belt_l2[1] == true && vec_part_on_belt_l2[2] == true) {
	   		ROS_INFO("An object is entering Scanner 1's range..");
			t_temp_l2[0] = (ros::Time::now().toSec() ); 
			row_count_l2++;

		} else if ((vec_part_on_belt_l2[0] == true && vec_part_on_belt_l2[1]== true && vec_part_on_belt_l2[2]== true && lock_l2 == 0)||(vec_part_on_belt_l2[0] == true && vec_part_on_belt_l2[1]== true && vec_part_on_belt_l2[2]== false && lock_l2 == 0)) {
				// ROS_INFO("The object is passing Scanner 1..");
				row_count_l2++;

            if (row_count_l2 > 6) { //used to be 15 but when belt set to .2m/s gear part will not be recogised.
			
			    			item_count_l2 = item_count_l2 + 1;
						t1_stack[item_count_l2] = t_temp_l2[0];
						cout << "\033[1;31mScanner 2 LOCKED..\033[0m\n";
						row_count_l2 = 0;
						lock_l2 = 1; //lock l2
				}
			
			} else if (vec_part_on_belt_l2[0] == true && vec_part_on_belt_l2[1]== false && vec_part_on_belt_l2[2]== false){

	   			ROS_INFO("The object has passed Scanner 2..");
				cout << "\033[1;34mUnlocking scanner 2..\033[0m\n";
				lock_l2 = 0;
				if (temp_speed > 0) cout << "\033[1;34m[Laser Scanner] Current Measured Speed: \033[0m"<< temp_speed << "\n";
	    			}

	} 

} 


int main(int argc, char **argv) {

	ros::init(argc, argv, "laser_speedometer");
	ros::NodeHandle nh; 

	ros::Publisher pub_speed = nh.advertise<std_msgs::Float64>("/ariac/belt_speed", 1);
	std_msgs::Float64 current_speed;
	current_speed.data = 0;

	cout << "\033[1;34mLaser Speedometer activated..\033[0m\n";

	//scan_publisher_ = pub;
	//laser_scanner::BeltImage beltimage;
	//part_ = beltimage; 

	ros::Subscriber laser_1_subscriber = nh.subscribe("/ariac/laser_profiler_1", 1, laser_1_Callback);
	ros::Subscriber laser_2_subscriber = nh.subscribe("/ariac/laser_profiler_2", 1, laser_2_Callback);

	while(ros::ok()){

		ros::spinOnce();
		if ((item_count_l2 != 0)  && (item_count_l1 != 0) && (item_count_l1 >= item_count_l2)) {
			temp_speed = scanner_dist/(t1_stack[item_count_l2] - t0_stack[item_count_l2]);
			current_speed.data = temp_speed;
		} else { //cout << "speed not known \n";
			}

	
		pub_speed.publish(current_speed); 
		ros::Duration(0.01).sleep(); 
		    
	}

	return 0; 
    
}

