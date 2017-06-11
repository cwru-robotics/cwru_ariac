// RN
// Laser Scanner Utility Class
// 15/5/17

#ifndef CWRU_ARIAC_LASERSCANNER_H
#define CWRU_ARIAC_LASERSCANNER_H

#include <AriacBase.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <sstream>
#include <math.h>
#include <ariac_xform_utils/ariac_xform_utils.h>
#include <ctime>

using namespace cv;

class LaserScanner : public AriacBase {

public:
    LaserScanner(ros::NodeHandle &nodeHandle);

    PartList conveyor_partlist;
    Part latest_part;
    mutex updateLock;

    void part_identification(cv::Mat dewarped_mat, int &type_id);

    vector<float> uni_vec(const vector<float> &vec_0, const vector<float> &vec_1);

    vector<float> cross_product(const vector<float> &vec_1, const vector<float> &vec_2);

    void integrate_info(const int &part_id, const vector<float> &origin, const vector<float> &pin_origin,
                        const ros::Time &ros_t_stamp, const float &belt_speed);

    void type_a_stamped_center(cv::Mat dewarped_mat_c, float &t_0, float &t_1, float &t_stamp, ros::Time &ros_t_0,
                               ros::Time &ros_t_1, ros::Time &ros_t_stamp, vector<float> &origin,
                               vector<int> &origin_pixel, int &part_id);

    void
    type_a_asymmetric(cv::Mat dewarped_mat_a, float scan_height, vector<float> &pin_origin, vector<int> &origin_pixel,
                      int part_id);

    // MK1 obsolete
    // Type B part solution
    // void type_b_stamped_center(cv::Mat dewarped_mat_c, float& t_0, float& t_1, float& t_stamp, ros::Time& ros_t_0, ros::Time& ros_t_1, ros::Time& ros_t_stamp, vector<float>& origin, vector<int>& origin_pixel);
    // void type_b_asymmetric(cv::Mat dewarped_mat_a, float scan_height, vector<float>& pin_origin, vector<int>& origin_pixel, int part_id);

    // MK2 currently deployed
    // Type B part solution
    void type_b_asymmetric(cv::Mat dewarped_mat_a, float scan_height, vector<float> &pin_origin, vector<int> &pin_pixel,
                           int part_id);

    void type_b_stamped_center(cv::Mat dewarped_mat_c, float scan_height, float &t_0, float &t_1, float &t_stamp,
                               ros::Time &ros_t_0, ros::Time &ros_t_1, ros::Time &ros_t_stamp, vector<float> &origin,
                               vector<int> &pin_pixel, vector<int> &origin_pixel);


    void forceUpdate();

    void publish_part();


    // MK3 part list management
    int removal_event = 0;
    bool removal_log_call = false;
    string rm_log_name;
    int rm_log_id;

    template<typename T>
    void remove_part(T &partlist, int part_id) {

        if (partlist.empty() == 1) {
            ROS_WARN("The part list is empty!");
        } else {
            typename T::iterator target_iterator;
            target_iterator = findPart(partlist, part_id);
            rm_log_name = target_iterator->name;
            rm_log_id = part_id;
            partlist.erase(target_iterator);
            cout << "\033[1;34mDesignated part has been romoved!\033[0m" << endl << ">>>Part id: " << part_id << endl;
            removal_event++;
            removal_log_call = true;
        }

    }

    int belt_partlist_size;
    int status_update_count = 0;

    template<typename T>
    void conveyor_list_status(T &partlist) {
        status_update_count++;
        if (status_update_count > 20) {
            status_update_count = 0;
            // int size;
            belt_partlist_size = partlist.size();
            cout << "\n\033[1;34mconveyor Belt Partlist Status Report:\033[0m" << endl
                 << "Current quantity of parts on belt: " << belt_partlist_size << endl;

            report_log_call = true;
        }
    }

    // A log function
    bool new_file = true;
    char log_name[30];
    time_t log_number;
    time_t event_time;
    int event_count = 0;

    template<typename T>
    void conveyor_list_blackbox(T &partlist) {
        if (new_file == true) {
            new_file = false;
            log_number = time(0);
            sprintf(log_name, "RN_Belt_Event_Recorder_%ld.log", log_number);
            ofstream ofstream_1(log_name);
            ofstream_1 << "Belt Event Recorder \n" << "Activating Laser Scanner " << log_number << "]\n";
            ofstream_1.close();
        } else {
            ofstream log;
            log.open(log_name, ofstream::app);

            if (exp_log_call == true) {
                event_count++;
                event_time = time(0);
                exp_log_call = false;
                log << "\n";
                log << "event number: " << event_count << "\n";
                log << "[Part Expiration Event " << event_time << "_" << exp_event << "]\n";
                log << "Expired part type: " << exp_log_name << "\n";
                log << "Part ID: " << exp_log_id << "\n";

            }

            if (removal_log_call == true) {
                event_count++;
                event_time = time(0);
                removal_log_call = false;
                log << "\n";
                log << "event number: " << event_count << "\n";
                log << "[Part Removal Event " << event_time << "_" << removal_event << "]\n";
                log << "Removed part type: " << rm_log_name << "\n";
                log << "Part ID: " << rm_log_id << "\n";


            }

            if (add_part_log_call == true) {
                event_count++;
                event_time = time(0);
                add_part_log_call = false;
                log << "\n";
                log << "event number: " << event_count << "\n";
                log << "[Part Addition Event " << event_time << "_" << add_event << "]\n";
                log << "Added part type: " << add_log_name << "\n";
                log << "Part ID: " << add_log_id << "\n";


            }

            if (report_log_call == true) {
                event_time = time(0);
                report_log_call = false;
                log << "\n";
                log << "[Current part count " << event_time << "]: " << belt_partlist_size << "\n";


            }

            log.close();
            // log <<

        }

    }

    void checkLog() {
        if (exp_log_call) {
            event_count++;
            exp_log_call = false;
        }
        if (removal_log_call) {
            event_count++;
            removal_log_call = false;
        }
        if (add_part_log_call) {
            event_count++;
            add_part_log_call = false;
        }
    }

    // some other log variables used in the function
    int add_event = 0; // for recording whatever item that has been added on the list
    bool add_part_log_call = false;
    int add_log_id;
    string add_log_name;
    bool report_log_call = false;


    template<typename T>
    void conveyor_list_clean(T &partlist) {
        partlist.clear();
    }


    int exp_event = 0;
    bool exp_log_call = false;
    string exp_log_name;
    int exp_log_id;

    template<typename T>
    void check_exp(T &partlist) {

        if (partlist.empty() == 1) {
            // ROS_WARN("The part list is empty REMOVE AFTER DEBUG");
        } else {
            auto oldest_part = partlist.front();
            auto oldest_part_iterator = partlist.begin();

            int oldest_part_id = oldest_part.id;
            float t_0 = oldest_part.pose.header.stamp.toSec();
            float t_1 = ros::Time::now().toSec();
            float v = abs(oldest_part.linear.y);
            float dist = (t_1 - t_0) * v;

            if (dist > conveyor_cut_off) {
                cout << "\033[1;31mA conveyor part has expired!\033[0m" << endl << ">>>Part id: " << oldest_part_id
                     << endl;

                exp_log_name = oldest_part.name;
                exp_log_id = oldest_part.id;
                remove_part(partlist, oldest_part_id);
                exp_event++;
                exp_log_call = true;

            }

        }

    }

    void check_exp() {
        if (conveyor_partlist.empty() == 1) {
            // ROS_WARN("The part list is empty REMOVE AFTER DEBUG");
        } else {
            auto oldest_part = conveyor_partlist.front();
            auto oldest_part_iterator = conveyor_partlist.begin();

            int oldest_part_id = oldest_part.id;
            float t_0 = oldest_part.pose.header.stamp.toSec();
            float t_1 = ros::Time::now().toSec();
            float v = abs(oldest_part.linear.y);
            float dist = (t_1 - t_0) * v;

            if (dist > conveyor_cut_off) {
                cout << "\033[1;31mA conveyor part has expired!\033[0m" << endl << ">>>Part id: " << oldest_part_id
                     << endl;

                exp_log_name = oldest_part.name;
                exp_log_id = oldest_part.id;
                remove_part(conveyor_partlist, oldest_part_id);
                exp_event++;
                exp_log_call = true;

            }

        }

    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber belt_speed_subscriber;
    ros::Subscriber scanner_2_subscriber;
    ros::Publisher part_publisher;

    int output_flag = 0;


    void laserCallback(const sensor_msgs::LaserScan &laser_scan);

    void belt_speed_cb(const std_msgs::Float64 &msg);

    float ping_dist_ = 0.0;
    int ping_index_min_ = -1;
    int ping_index_max_ = -1;
    int center_index_ = -1;
    double phi_ = 0.0;
    double phi_temp;
    float z_ = 0.0;
    double angle_min_ = 0.0;
    double angle_max_ = 0.0;
    double angle_increment_ = 0.0;
    double range_min_ = 0.0;
    double range_max_ = 0.0;
    bool part_on_belt_ = false;
    bool part_detected_ = false;
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

    // MK3 Conveyor belt management
    // conveyor belt cut-off
    int conveyor_cut_off = 3;


};

#endif //CWRU_ARIAC_LASERSCANNER_H
