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

    void forceUpdate();

    void publish_part();

    template<typename T>
    void remove_part(T &partlist, int part_id) {

        if (partlist.empty() == 1) {
            ROS_WARN("The part list is empty!");
        } else {
            auto target_iterator = findPart(partlist, part_id);
            if (target_iterator == partlist.end()) {
                return;
            }
            rm_log_name = target_iterator->name;
            rm_log_id = part_id;
            partlist.erase(target_iterator);
            cout << "\033[1;34mDesignated part has been romoved!\033[0m" << endl << ">>>Part id: " << part_id << endl;
            removal_event++;
            removal_log_call = true;
        }

    }

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


    template<typename T>
    void conveyor_list_blackbox(T &partlist) {
        if (new_file == true) {
            new_file = false;
            log_number = time(0);
            log_name = "RN_Belt_Event_Recorder_" + to_string(log_number) + ".log";
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

    template<typename T>
    void conveyor_list_clean(T &partlist) {
        partlist.clear();
    }

    void check_exp() {
        if (conveyor_partlist.empty() == 1) {
            // ROS_WARN("The part list is empty REMOVE AFTER DEBUG");
        } else {
            auto oldest_part = conveyor_partlist.front();
            auto oldest_part_iterator = conveyor_partlist.begin();

            int oldest_part_id = oldest_part.id;

            if (oldest_part.pose.pose.position.y < (laser_profiler_2_origin[1] - 0.06 - conveyor_cut_off)) {
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

    double ping_dist_ = 0.0;
    int ping_index_min_ = -1;
    int ping_index_max_ = -1;
    int center_index_ = -1;
    double phi_ = 0.0;
    double phi_temp;
    double z_ = 0.0;
    double angle_min_ = 0.0;
    double angle_max_ = 0.0;
    double angle_increment_ = 0.0;
    double range_min_ = 0.0;
    double range_max_ = 0.0;
    bool part_on_belt_ = false;
    bool part_detected_ = false;
    bool prev_part_on_belt_ = false;

    std::vector<bool> vec_part_on_belt;
    double belt_depth_ = 1.6;
    double tolerance_ = 0.005;
    // laser_scanner::BeltImage part_;
    int item_count = 0;
    std::vector<double> row_z;
    std::vector<double> row_z_prev_0;
    std::vector<double> row_z_prev_1;
    std::vector<double> row_z_prev_2;
    std::vector<double> row_z_prev_3;
    double time_0 = 0;
    double time0_0 = 0;
    double time0_1 = 0;
    double time0_2 = 0;
    double time0_3 = 0;
    double time_1 = 0;
    ros::Time ros_time_0;
    ros::Time ros_time0_0;
    ros::Time ros_time0_1;
    ros::Time ros_time0_2;
    ros::Time ros_time0_3;
    ros::Time ros_time_1;

    std::vector<std::vector<double> > image_z;
    int image_x_size;
    int image_y_size;

    double part_z_max = 0.028; //Disk part's height + tolorance
    double scan_width_ = 0;
    ros::Publisher scan_publisher_;
    double belt_speed = 0;
    double speed_temp_0 = 0;
    double speed_temp_1 = 0;
    // double rod_sqr_l = 0.052; // 0.052 x 0.059
    double rod_sqr_l = 0.0556; // 0.052 x 0.059 rectangle diagonal

    const double ht_disk = 0.02316;
    const double ht_gasket = 0.02;
    const double ht_gear = 0.012;
    const double ht_piston_rod = 0.0075;

    std::vector<double> laser_profiler_2_origin; // RN's own config TODO update to CWRU's config

    //For ForceUpdate();
    int updateCount_cb_speed;
    int checkedCount_cb_speed;
    int updateCount_cb_scan;
    int checkedCount_cb_scan;

    int conveyor_cut_off = 3;

    // some other log variables used in the function
    int add_event = 0; // for recording whatever item that has been added on the list
    bool add_part_log_call = false;
    int add_log_id;
    string add_log_name;
    bool report_log_call = false;

    // A log function
    bool new_file = true;
    string log_name;
    time_t log_number;
    time_t event_time;
    int event_count = 0;

    // MK3 part list management
    int removal_event = 0;
    bool removal_log_call = false;
    string rm_log_name;
    int rm_log_id;

    int exp_event = 0;
    bool exp_log_call = false;
    string exp_log_name;
    int exp_log_id;


    int belt_partlist_size;
    int status_update_count = 0;

    void part_identification(cv::Mat dewarped_mat, int &type_id);

    vector<double> uni_vec(const vector<double> &vec_0, const vector<double> &vec_1);

    vector<double> cross_product(const vector<double> &vec_1, const vector<double> &vec_2);

    void integrate_info(const int &part_id, const vector<double> &origin, const vector<double> &pin_origin,
                        const ros::Time &ros_t_stamp, const double &belt_speed);

    void type_a_stamped_center(cv::Mat dewarped_mat_c, double &t_0, double &t_1, double &t_stamp, ros::Time &ros_t_0,
                               ros::Time &ros_t_1, ros::Time &ros_t_stamp, vector<double> &origin,
                               vector<int> &origin_pixel, int &part_id);

    void
    type_a_asymmetric(cv::Mat dewarped_mat_a, double scan_height, vector<double> &pin_origin, vector<int> &origin_pixel,
                      int part_id);

    // MK1 obsolete
    // Type B part solution
    // void type_b_stamped_center(cv::Mat dewarped_mat_c, double& t_0, double& t_1, double& t_stamp, ros::Time& ros_t_0, ros::Time& ros_t_1, ros::Time& ros_t_stamp, vector<double>& origin, vector<int>& origin_pixel);
    // void type_b_asymmetric(cv::Mat dewarped_mat_a, double scan_height, vector<double>& pin_origin, vector<int>& origin_pixel, int part_id);

    // MK2 currently deployed
    // Type B part solution
    void
    type_b_asymmetric(cv::Mat dewarped_mat_a, double scan_height, vector<double> &pin_origin, vector<int> &pin_pixel,
                      int part_id);

    void type_b_stamped_center(cv::Mat dewarped_mat_c, double scan_height, double &t_0, double &t_1, double &t_stamp,
                               ros::Time &ros_t_0, ros::Time &ros_t_1, ros::Time &ros_t_stamp, vector<double> &origin,
                               vector<int> &pin_pixel, vector<int> &origin_pixel);
};

#endif //CWRU_ARIAC_LASERSCANNER_H
