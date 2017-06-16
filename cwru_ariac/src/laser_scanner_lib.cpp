// RN
// Laser Scanner Utility Library
// Created on 15/5/17
// Updated on 25/5/17
// Version: MK2

#include <LaserScanner.h>


LaserScanner::LaserScanner(ros::NodeHandle &nodeHandle) : nh(nodeHandle) {

    ROS_INFO("Constructing a Laser Scanner Object..");

    updateCount_cb_speed = 0;
    checkedCount_cb_speed = 0;
    updateCount_cb_scan = 0;
    checkedCount_cb_scan = 0;

    vec_part_on_belt.resize(3, 0);

    row_z_prev_0.resize(1, 0);
    row_z_prev_1.resize(1, 0);
    row_z_prev_2.resize(1, 0);
    row_z_prev_3.resize(1, 0);

    laser_profiler_2_origin.clear();
    laser_profiler_2_origin.push_back(1.21);
    laser_profiler_2_origin.push_back(3.50);
    laser_profiler_2_origin.push_back(1.59);

    belt_speed_subscriber = nh.subscribe("/ariac/belt_speed", 10, &LaserScanner::belt_speed_cb, this);
    scanner_2_subscriber = nh.subscribe("/ariac/laser_profiler_2", 10, &LaserScanner::laserCallback, this);

    part_publisher = nh.advertise<Part>("/ariac/latest_conveyor_part", 10);

    ROS_INFO("subscribers ready..");

    // cout << "\n\033[0;31mDo you wish to output images and related .csv files? Press 1 if you do, otherwise press 0: \033[0m";
//		cin >> output_flag;

    ROS_INFO("Laser Scanner activated..");
}


void LaserScanner::forceUpdate() {

    int spin_count = 0;
    while (ros::ok() && spin_count < 100) {
        publish_part();
        ros::spinOnce();
        ros::Duration(0.0025).sleep();
        spin_count++;
    }

}


void LaserScanner::belt_speed_cb(const std_msgs::Float64 &msg) {

    updateCount_cb_speed++;
    speed_temp_0 = speed_temp_1;
    speed_temp_1 = msg.data;
    if (speed_temp_1 != speed_temp_0) {
        belt_speed = msg.data;
    }
}


void LaserScanner::part_identification(cv::Mat dewarped_mat, int &type_id) {
    int part_pt_count = 0;
    double ht_total = 0;
    double ht_avg;
    cv::Size mat_size = dewarped_mat.size();
    int mat_ht = mat_size.height;
    int mat_wd = mat_size.width;

    for (int i = 0; i < mat_ht; i++) {

        for (int j = 0; j < mat_wd; j++) {

            if ((dewarped_mat.at<double>(i, j) > 35) && (dewarped_mat.at<double>(i, j) < 256)) {
                part_pt_count++;
                ht_total = ht_total + dewarped_mat.at<double>(i, j);
            } else if (dewarped_mat.at<double>(i, j) > 255) {
//				ROS_WARN("DEPTH GREATER THAN 255! at (i = %d, j = %d) = %f.", i, j, dewarped_mat.at<double>(i, j));
            }
        }
    }

    ht_avg = ht_total / part_pt_count;

    /** RN 11/5/17
    [Only for temporary, experimental purpose]
    1. Disk: ID = 1, range: 200 - 225
    2. Gasket: ID = 2, range: 170 - 195
    3. Gear: ID = 3, range: 110 - 125
    4. Piston rod: ID = 4, range: 60 - 85
    */

    if ((ht_avg > 200) && (ht_avg < 225)) {
        type_id = 1;
        cout << "Target identified as a " << "\033[1;31mDisk Part.\033[0m\n";
    } else if ((ht_avg > 170) && (ht_avg < 195)) {
        type_id = 2;
        cout << "Target identified as a " << "\033[1;31mGasket Part.\033[0m\n";
    } else if ((ht_avg > 100) && (ht_avg < 125)) {
        type_id = 3;
        cout << "Target identified as a " << "\033[1;31mGear Part.\033[0m\n";
    } else if ((ht_avg > 60) && (ht_avg < 85)) {
        type_id = 4;
        cout << "Target identified as a " << "\033[1;31mPiston Rod.\033[0m\n";
    } else {
        type_id = 999;
        ROS_WARN("Failed to identify target.");
        cout << "ht_avg: [DEBUG] " << ht_avg << "\n";
        cout << "part_pt_count: [DEBUG] " << part_pt_count << "\n";
        cout << "ht_total: [DEBUG] " << ht_total << "\n";
        cout << "mat_ht: " << mat_ht << "\n";
        cout << "mat_wd: " << mat_wd << "\n";
    }

}


vector<double> LaserScanner::uni_vec(const vector<double> &vec_0, const vector<double> &vec_1) {
    // ONLY deal with our very speical case in which part z axis always points up
    double delta_x = vec_1[0] - vec_0[0];
    double delta_y = vec_1[1] - vec_0[1];
    double delta_z = 0;
    double norm = sqrt(delta_x * delta_x + delta_y * delta_y);
    vector<double> uni_vec_;
    uni_vec_.clear();
    double x = delta_x / norm;
    double y = delta_y / norm;
    double z = delta_z / norm;
    uni_vec_.push_back(x);
    uni_vec_.push_back(y);
    uni_vec_.push_back(z);

    return uni_vec_;

}

// RN Feedback 17/5/17
// Actually Eigen Lib has the cross() function.
vector<double> LaserScanner::cross_product(const vector<double> &vec_1, const vector<double> &vec_2) {
    double u1 = vec_1[0];
    double u2 = vec_1[1];
    double u3 = vec_1[2];
    double v1 = vec_2[0];
    double v2 = vec_2[1];
    double v3 = vec_2[2];

    double x = u2 * v3 - u3 * v2;
    double y = u3 * v1 - u1 * v3;
    double z = u1 * v2 - u2 * v1;

    vector<double> crossed;
    crossed.clear();
    crossed.push_back(x);
    crossed.push_back(y);
    crossed.push_back(z);

    return crossed;

}


void LaserScanner::publish_part() {

    part_publisher.publish(latest_part);

}


void LaserScanner::integrate_info(const int &part_id, const vector<double> &origin, const vector<double> &pin_origin,
                                  const ros::Time &ros_t_stamp, const double &belt_speed) {

    Part part;
    Eigen::Affine3d affine_temp;
    Eigen::Vector3d origin_temp;
    geometry_msgs::Pose pose_temp;

    origin_temp[0] = (origin[0]);
    origin_temp[1] = (origin[1]);
    origin_temp[2] = (origin[2]);

    vector<double> vec_x;
    vector<double> vec_y;
    vector<double> vec_z(3, 0);
    // vec_y = uni_vec(pin_origin, origin);
    vec_y = uni_vec(origin, pin_origin);
    vec_z[2] = 1;
    vec_x = cross_product(vec_y, vec_z);

    Eigen::Matrix3d rot_z_45d; //disk
    rot_z_45d(0, 0) = sqrt(2) / 2;
    rot_z_45d(0, 1) = -sqrt(2) / 2;
    rot_z_45d(0, 2) = 0;
    rot_z_45d(1, 0) = sqrt(2) / 2;
    rot_z_45d(1, 1) = sqrt(2) / 2;
    rot_z_45d(1, 2) = 0;
    rot_z_45d(2, 0) = 0;
    rot_z_45d(2, 1) = 0;
    rot_z_45d(2, 2) = 1;

    Eigen::Matrix3d rot_z_90d; //gasket
    rot_z_90d(0, 0) = 0;
    rot_z_90d(0, 1) = -1;
    rot_z_90d(0, 2) = 0;
    rot_z_90d(1, 0) = 1;
    rot_z_90d(1, 1) = 0;
    rot_z_90d(1, 2) = 0;
    rot_z_90d(2, 0) = 0;
    rot_z_90d(2, 1) = 0;
    rot_z_90d(2, 2) = 1;

    Eigen::Matrix3d default_mat;
    default_mat(0, 0) = vec_x[0];
    default_mat(1, 0) = vec_x[1];
    default_mat(2, 0) = vec_x[2];
    default_mat(0, 1) = vec_y[0];
    default_mat(1, 1) = vec_y[1];
    default_mat(2, 1) = vec_y[2];
    default_mat(0, 2) = vec_z[0];
    default_mat(1, 2) = vec_z[1];
    default_mat(2, 2) = vec_z[2];


    if (part_id == 2) default_mat = rot_z_90d * default_mat;
    if (part_id == 1) default_mat = rot_z_45d * default_mat;

    affine_temp.linear() = default_mat;
    affine_temp.translation() = origin_temp;

    XformUtils XformUtils_;
    pose_temp = XformUtils_.transformEigenAffine3dToPose(affine_temp);

    if (part_id == 1) { part.name = "disk_part"; }
    else if (part_id == 2) { part.name = "gasket_part"; }
    else if (part_id == 3) { part.name = "gear_part"; }
    else if (part_id == 4) { part.name = "piston_rod_part"; }

    part.id = 20000 + item_count;
    part.traceable = true;
    part.location = 11;
    part.linear.x = 0;
    part.linear.z = 0;
    part.linear.y = -belt_speed; // belt direction is opposite to the world frame y direction.
    part.pose.pose = pose_temp;
    part.pose.header.stamp = ros_t_stamp;
    part.pose.header.frame_id = "world";

    std::lock_guard <std::mutex> guard(updateLock);
    conveyor_partlist.push_back(part);

    latest_part = part;


//	cout << "part.id: " << part.id << endl
//	     << "part.name: " << part.name << endl
//	     << "part.traceable: " << part.traceable << endl
//	     << "part.location: " << part.location << endl
//	     << "part.linear.x: " << part.linear.x << endl
//	     << "part.linear.y: " << part.linear.y << endl
//	     << "part.linear.z: " << part.linear.z << endl
//	     << "part.pose.header.stamp: " << part.pose.header.stamp << endl
//	     << "part.pose.pose: " << part.pose.pose << endl;

    ROS_INFO("Publishing its part msg to /ariac/latest_conveyor_part");

    vec_x.clear();
    vec_y.clear();
    vec_z.clear();

    // MK3
    // Belt partlist management
    add_event++;
    add_part_log_call = true;
    add_log_id = part.id;
    add_log_name = part.name;

}


void
LaserScanner::type_a_stamped_center(cv::Mat dewarped_mat_c, double &t_0, double &t_1, double &t_stamp,
                                    ros::Time &ros_t_0,
                                    ros::Time &ros_t_1, ros::Time &ros_t_stamp, vector<double> &origin,
                                    vector<int> &origin_pixel, int &part_id) {
    origin.clear();
    origin_pixel.clear();
    cv::Size mat_size = dewarped_mat_c.size();
    double ht_total = 0;
    double ht_avg;
    int part_pt_count = 0;
    int mat_ht = mat_size.height;
    int mat_wd = mat_size.width; // bit redundant but will be improved in the future
    int i_total = 0;
    int j_total = 0;
    double i_avg, j_avg;
    double x, y, z;
    double ht_part;
    for (int i = 0; i < mat_ht; i++) {

        for (int j = 0; j < mat_wd; j++) {

            if (dewarped_mat_c.at<double>(i, j) > 35) {
                part_pt_count++;
                i_total = i_total + i;
                j_total = j_total + j;
                ht_total = ht_total + dewarped_mat_c.at<double>(i, j);
            }
        }

    }
    ht_avg = ht_total / part_pt_count;
    i_avg = i_total / part_pt_count;
    j_avg = j_total / part_pt_count;

    if (part_id == 1) {
        ht_part = ht_disk;
    } else if (part_id == 2) {
        ht_part = ht_gasket;
    } else if (part_id == 3) {
        ht_part = ht_gear;
    }


    x = laser_profiler_2_origin[0] + scan_width_ * (j_avg - mat_wd / 2) / mat_wd;
    y = laser_profiler_2_origin[1] - 0.06; // i: scan img height along world x axis
    // z = laser_profiler_2_origin[2] - belt_depth_ + ht_part;
    z = laser_profiler_2_origin[2] -
        belt_depth_; // A MK3 modification: the origin is always at bottom surface, not relevant to the thickness of the part.
    origin.push_back(x);
    origin.push_back(y);
    origin.push_back(z);

    ROS_INFO("This part's origin xyz: %f %f %f", x, y, z);

    // Calculate Time Stamp

    t_stamp = t_0 + (t_1 - t_0) * i_avg / mat_ht;
    // ROS_INFO("Origin time stamp: %f",t_stamp);
    ros::Duration delta_t = ros::Duration((t_1 - t_0) * i_avg / mat_ht);
    ros_t_stamp = ros_t_0 + delta_t;

    cv::Point pt;
    pt = cv::Point(static_cast<int>(j_avg), static_cast<int>(i_avg));
    origin_pixel.push_back(static_cast<int>(j_avg)); //width
    origin_pixel.push_back(static_cast<int>(i_avg)); //height
    int r = 2;
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    cv::circle(dewarped_mat_c, pt, r, cv::Scalar(255, 255, 255, 255), thickness, lineType, shift);

}


void LaserScanner::type_a_asymmetric(cv::Mat dewarped_mat_a, double scan_height, vector<double> &pin_origin,
                                     vector<int> &origin_pixel, int part_id) {
    pin_origin.clear();

    cv::Size mat_size = dewarped_mat_a.size();
    double ht_total = 0;
    double ht_avg = 0;
    int part_pt_count = 0;
    int mat_ht = mat_size.height;
    int mat_wd = mat_size.width;
    int i_tallest = 0;
    int j_tallest = 0;
    int z_tallest = 0;
    double i_avg, j_avg;
    double x, y, z;

    z_tallest = 0;

    // IN ORDER TO AVOID THE WHITE EDGE CAUSING ANY TROUBLES
    // Set a smaller search window based on part type (size) and the detected centre location.
    int window_lt = 0;
    int window_rt = 0;
    int z_cap = 255; // restrict the depth of the pt must be under this value
    double ht_part; // use this as the asymmetric pin's z value instead of the real pin height
    if (part_id == 1) {
        ROS_INFO("part_id == 1");
        z_cap = 245;
        // window_lt = origin_pixel[0] - 22; //rt = lt + 45
        window_lt = origin_pixel[0] - 22 * 4; //rt = lt + 45*4
        if (window_lt < 0) window_lt = 0;
        // window_rt = window_lt + 45;
        window_rt = window_lt + 45 * 4;
        if (window_rt > mat_wd) window_rt = mat_wd;
        ht_part = ht_disk;
    } else if (part_id == 3) {
        ROS_INFO("part_id == 3");
        z_cap = 145;
        // window_lt = origin_pixel[0] - 12; //rt = lt + 25
        window_lt = origin_pixel[0] - 12 * 4; //rt = lt + 25*4
        if (window_lt < 0) window_lt = 0;
        // window_rt = window_lt + 25;
        window_rt = window_lt + 25 * 4;
        if (window_rt > mat_wd) window_rt = mat_wd;
        ht_part = ht_gear;

    } else if (part_id == 2) {
        ROS_INFO("part_id == 2");
        z_cap = 245;
        // window_lt = origin_pixel[0] - 30; //rt = lt + 60
        window_lt = origin_pixel[0] - 30 * 4; //rt = lt + 60*4
        if (window_lt < 0) window_lt = 0;
        // window_rt = window_lt + 60;
        window_rt = window_lt + 60 * 4;
        if (window_rt > mat_wd) window_rt = mat_wd;
        ht_part = ht_gasket;
    } else { ROS_WARN("THIS PART DOES NOT HAVE A ASYMMETRIC PART, IS THIS A PISTON ROD?"); }

    if (part_id != 999) {
        // searching for the tallest point



        for (int i = 0; i < mat_ht; i++) {

            for (int j = window_lt; j < window_rt; j++) {

                if ((dewarped_mat_a.at<double>(i, j) > z_tallest) && (dewarped_mat_a.at<double>(i, j) < z_cap)) {
                    z_tallest = dewarped_mat_a.at<double>(i, j);
                    i_tallest = i;
                    j_tallest = j;

                } else {}


            }
        }

        // searching its neighbours
        // Gasket's pin is relatively taller than pins of gear parts and disk parts.
        int count_target;
        int depth_tol;
        if (part_id == 1) {
            depth_tol = 20;
            count_target = 10;
        }
        if (part_id == 2) {
            depth_tol = 40;
            count_target = 10;
        }
        if (part_id == 3) {
            depth_tol = 20;
            count_target = 10;
        }

        int pin_pt_count = 0;
        int pin_i_total = 0;
        int pin_j_total = 0;
        int pin_i_avg, pin_j_avg;

        // a 10x10 search window

        int i_0 = i_tallest - 4;
        int i_1 = i_tallest + 5;
        int j_0 = j_tallest - 4;
        int j_1 = j_tallest + 5;

        for (int i = i_0; i < i_1; i++) {
            for (int j = j_0; j < j_1; j++) {
                if (z_tallest - dewarped_mat_a.at<double>(i, j) <= depth_tol) {
                    pin_pt_count++;
                    pin_i_total = pin_i_total + i;
                    pin_j_total = pin_j_total + j;

                }
            }
        }


        if (pin_pt_count >= count_target) {
            pin_i_avg = pin_i_total / pin_pt_count;
            pin_j_avg = pin_j_total / pin_pt_count;

            x = laser_profiler_2_origin[0] + scan_width_ * (pin_j_avg - mat_wd / 2) / mat_wd;
            y = laser_profiler_2_origin[1] - 0.06 +
                scan_height * (pin_i_avg - (int) (mat_ht / 2)) / mat_ht; // i: scan img height along world x axis
            z = laser_profiler_2_origin[2] - belt_depth_ + ht_part;
            pin_origin.push_back(x);
            pin_origin.push_back(y);
            pin_origin.push_back(z);

            ROS_INFO("The Asymmetric part is found at xyz: %f %f %f", x, y, z);

            // TODO delete after test

            cv::Point pt;
            pt = cv::Point(static_cast<int>(pin_j_avg), static_cast<int>(pin_i_avg));
            int r = 2;
            int thickness = 1;
            int lineType = 8;
            int shift = 0;
            cv::circle(dewarped_mat_a, pt, r, cv::Scalar(0, 255, 0, 255), thickness, lineType, shift);

        } else { ROS_WARN("Failed to find the asymmetric part of this object!"); }
    }

}


void
LaserScanner::type_b_stamped_center(cv::Mat dewarped_mat_c, double scan_height, double &t_0, double &t_1,
                                    double &t_stamp,
                                    ros::Time &ros_t_0, ros::Time &ros_t_1, ros::Time &ros_t_stamp,
                                    vector<double> &origin, vector<int> &pin_pixel, vector<int> &origin_pixel) {

    origin.clear();
    origin_pixel.clear();
    cv::Size mat_size = dewarped_mat_c.size();

    double ht_total = 0;
    double ht_avg;
    int part_pt_count = 0;
    int mat_ht = mat_size.height;
    int mat_wd = mat_size.width; // bit redundant but will be improved in the future
    int h_total, w_total;
    double h_avg, w_avg;
    double x, y, z;
    int window_r;
    double window_r_float;
    int winner_win_count = 0;
    int winner_penalty_score = 10000;
    int current_penalty = 0;
    int pin_centre_penalty = 0; // the distance betwen pin and centre
    int pin_centre_dist;
    int winner_i = 0;
    int winner_j = 0;
    std::vector<double> theta;
    std::vector<int> width_delta;
    //std::vector<int> width_end;
    int temp;
    theta.clear();
    width_delta.clear();
    // vector<double> debug;

    // Target area match
    int target_match;
    target_match = (mat_wd / scan_width_) * (mat_wd / scan_width_) * (0.052 * 0.059 + (0.0556 - 0.052) * 0.031);
    int pin_centre_dist_match;
    pin_centre_dist_match = (0.04258) * (mat_wd / scan_width_);
    // 0.04258 is the calculated dist between the geometry centre to the origin of this part

    // calculate the suqare search region based on the geometry centre (in this case, the pin) coordinate.
    int square_l;
    square_l = (mat_wd * 0.17 / scan_width_) * 0.5; // mat_wd is actually equal to 400.
    int square_lt;
    square_lt = pin_pixel[0] - square_l / 2;
    if (square_lt < 0) square_lt = 5;
    int square_rt;
    square_rt = pin_pixel[0] + square_l / 2;
    if (square_rt > mat_wd) square_lt = mat_wd - 5;
    int square_up;
    square_up = pin_pixel[1] - square_l / 2;
    if (square_up < 0) square_up = 5;
    int square_dw;
    square_dw = pin_pixel[1] + square_l / 2;
    if (square_dw > mat_ht) square_dw = mat_ht - 5;

    // calculate the window r's pixel size (how many)
    window_r = (int) ((sqrt(2) / 2) * rod_sqr_l * mat_wd / scan_width_);
    window_r_float = ((sqrt(2) / 2) * rod_sqr_l * mat_wd / scan_width_);


    // caculate angles and delta_width
    for (int i = 0; i < (window_r + 1); i++) {
        theta.push_back(asin((window_r_float - i) / window_r_float));
        // debug.push_back( ((window_r_float - i)/window_r_float) );
        temp = (int) window_r * cos(theta[i]);
        width_delta.push_back(temp);

    }

    for (int i = 1; i < (window_r); i++) {
        theta.push_back(asin((i) / window_r_float));
        // debug.push_back( ((i)/window_r_float) );
        temp = (int) window_r * cos(theta[i + window_r]);
        width_delta.push_back(temp);

    }

    // search for the centre of the circular window that contains the square part of the piston rod
    // for (int i = 5; i < (mat_ht - 5); i++) {
    for (int i = square_up; i < square_dw; i++) {


        // for (int j = 5; j < (mat_wd - 5); j++) {
        for (int j = square_lt; j < square_rt; j++) {

            int h_upper;
            int h_lower;
            h_upper = i - window_r + 1;
            h_lower = i + window_r;
            int stack_count = 0;
            part_pt_count = 0;
            for (int h = h_upper; h < h_lower; h++) {
                int w_left;
                int w_right;
                w_left = j - width_delta[stack_count] + 1;
                w_right = j + width_delta[stack_count];
                stack_count++;

                for (int w = w_left; w < w_right; w++) {

                    if ((w > 0) && (w <= mat_wd) && (h > 0) && (h < mat_ht)) {
                        if ((dewarped_mat_c.at<double>(h, w) > 40) && (dewarped_mat_c.at<double>(h, w) < 90)) {
                            part_pt_count++;
                            h_total = h_total + h;
                            w_total = w_total + w;
                            ht_total = ht_total + dewarped_mat_c.at<double>(h, w);
                            // note that mat.at(height, width) BUT Point pt = Point(width, height);
                        }
                    }
                }
            }


            // MK1 score function Obsolete
            // if (part_pt_count > winner_win_count) {
            //	winner_win_count = part_pt_count;
            //	winner_i = i;
            //	winner_j = j;
            //	ht_avg = ht_total/part_pt_count;
            //
            // }

            // MK2 penalty function Current
            pin_centre_dist = sqrt((j - pin_pixel[0]) * (j - pin_pixel[0]) + (i - pin_pixel[1]) * (i - pin_pixel[1]));

            // pin_centre_penalty = -0*pin_centre_dist; // the weight of this penalty is 100, which is a tested number

            pin_centre_penalty =
                    40 * (pin_centre_dist - pin_centre_dist_match) * (pin_centre_dist - pin_centre_dist_match);

            //current_penalty = abs(part_pt_count - target_match) + pin_centre_penalty;
            current_penalty = -part_pt_count + pin_centre_penalty;

            if (current_penalty < winner_penalty_score) {
                winner_penalty_score = current_penalty;
                winner_i = i;
                winner_j = j;
                ht_avg = ht_total / part_pt_count;

            }

        }

    }

    // x = laser_profiler_2_origin[0] + scan_width_*(winner_j - 50)/101 ; // j: scan img width along world x axis
    x = laser_profiler_2_origin[0] + scan_width_ * (winner_j - mat_wd / 2) / mat_wd;
    // y = laser_profiler_2_origin[1] - 0.06 + scan_height*(winner_i - mat_ht/2)/mat_ht ; // i: scan img height along world x axis
    // y = laser_profiler_2_origin[1] + scan_height*(winner_i - mat_ht/2)/mat_ht ; // i: scan img height along world x axis
    y = laser_profiler_2_origin[1] - 0.06;
    // z = laser_profiler_2_origin[2] - belt_depth_ + (ht_avg * part_z_max)/255;
    // z = laser_profiler_2_origin[2] - belt_depth_ + 0.0075;
    z = laser_profiler_2_origin[2] -
        belt_depth_; // A MK3 modification: the origin is always at bottom surface, not relevant to the thickness of the part.
    origin.push_back(x);
    origin.push_back(y);
    origin.push_back(z);

    ROS_INFO("This part's origin xyz: %f %f %f", x, y, z);

    // Calculate Time Stamp
    t_stamp = t_0 + (t_1 - t_0) * winner_i / mat_ht;
    // t_stamp = t_0 + (t_1 - t_0)*(1 - winner_i/mat_ht);
    ROS_INFO("Origin time stamp: %f", t_stamp);


    ros::Duration delta_t = ros::Duration((t_1 - t_0) * winner_i / mat_ht);
    ros_t_stamp = ros_t_0 + delta_t;

    cv::Point pt;
    pt = cv::Point(static_cast<int>(winner_j), static_cast<int>(winner_i));
    origin_pixel.push_back(static_cast<int>(winner_j)); //width
    origin_pixel.push_back(static_cast<int>(winner_i)); //height
    int r = 2;
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    cv::circle(dewarped_mat_c, pt, r, cv::Scalar(255, 255, 255, 255), thickness, lineType, shift);

}


void LaserScanner::type_b_asymmetric(cv::Mat dewarped_mat_a, double scan_height, vector<double> &pin_origin,
                                     vector<int> &pin_pixel, int part_id) {
    pin_pixel.clear();

    cv::Size mat_size = dewarped_mat_a.size();
    double ht_total = 0;
    double ht_avg;
    int part_pt_count = 0;
    int mat_ht = mat_size.height;
    int mat_wd = mat_size.width; // bit redundant but will be improved in the future
    int i_tallest, j_tallest, z_tallest;
    int i_total = 0;
    int j_total = 0;
    double i_avg, j_avg;
    double x, y, z;

    for (int i = 5; i < (mat_ht - 5); i++) {

        for (int j = 5; j < (mat_wd - 5); j++) {

            if ((dewarped_mat_a.at<double>(i, j) > 40) && (dewarped_mat_a.at<double>(i, j) < 90)) {

                i_total = i_total + i;
                j_total = j_total + j;
                part_pt_count++;

            }

        }

    }
    i_avg = i_total / part_pt_count;
    j_avg = j_total / part_pt_count;

    // x = laser_profiler_2_origin[0] + scan_width_*(j_avg - 50)/101 ; // j: scan img width along world x axis
    x = laser_profiler_2_origin[0] + scan_width_ * (j_avg - mat_wd / 2) / mat_wd;
    y = laser_profiler_2_origin[1] - 0.06 + scan_height * (i_avg - (int) (mat_ht / 2)) / mat_ht;
    // z = laser_profiler_2_origin[2] - belt_depth_ + (ht_avg * part_z_max)/255;
    z = laser_profiler_2_origin[2] - belt_depth_ + 0.0075;
    pin_origin.push_back(x);
    pin_origin.push_back(y);
    pin_origin.push_back(z);

    pin_pixel.push_back(j_avg);
    pin_pixel.push_back(i_avg);

    ROS_INFO("The Asymmetric part is found at xyz: %f %f %f", x, y, z);

    // TODO delete after test

    cv::Point pt;
    pt = cv::Point(static_cast<int>(j_avg), static_cast<int>(i_avg));
    int r = 2;
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    cv::circle(dewarped_mat_a, pt, r, cv::Scalar(0, 255, 0, 255), thickness, lineType, shift);
}


void LaserScanner::laserCallback(const sensor_msgs::LaserScan &laser_scan) {
    updateCount_cb_scan++;

    int part_id;

    vector<double> origin;
    vector<double> pin_origin;
    vector<int> pin_pixel;
    double t_stamp;
    ros::Time ros_t_stamp;
    vector<int> origin_pixel;

    part_on_belt_ = false;
    row_z.clear();
    if (ping_index_min_ < 0) {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;    //minimum possible angle
        angle_max_ = laser_scan.angle_max;    //maximum possible angle
        angle_increment_ = laser_scan.angle_increment;    //angle difference between measurements
        range_min_ = laser_scan.range_min;    //minimum measurable distance
        range_max_ = laser_scan.range_max;    //maximum measurable distance
        center_index_ = (int) ((0.0 - angle_min_) / angle_increment_); //index of center of arc
        ping_index_min_ = 0;    //set starting index
        // ping_index_max_ =   (angle_max_ - angle_min_)/angle_increment_;	//set maximum index //THIS WILL CREATE CHAOS
        ping_index_max_ = 399;
        double belt_depth_temp = 0;
        int n2 = 1;
        for (int n = center_index_ - 5; n < center_index_ + 5; n++) {

            phi_temp = (n - ping_index_min_) * angle_increment_ + angle_min_;
            belt_depth_temp =
                    belt_depth_temp + laser_scan.ranges[n] * cos(phi_temp); //distance from laser scanner to belt

            belt_depth_ = belt_depth_temp / n2;
            n2++;
        }

        ROS_INFO("Belt Depth is %f", belt_depth_);

        scan_width_ = 2 * belt_depth_ * tan(angle_max_);
        ROS_INFO("Scan Width is %f", scan_width_);

        //TODO delete the following few lines after debug
        cout << "angle_increment_: " << angle_increment_ << endl;
        cout << "ping_index_max_: " << ping_index_max_ << endl;
        cout << "angle_min_: " << angle_min_ << endl;
        cout << "angle_max_: " << angle_max_ << endl;
        cout << "angle_increment_: " << angle_increment_ << endl;
        double ping_index_max_float;
        ping_index_max_float = (angle_max_ - angle_min_) / angle_increment_;
        cout << "ping_index_max_float: " << ping_index_max_float << endl;

        // initialise vectors
        std::vector<double> init_row(ping_index_max_, 0);
        row_z_prev_0 = init_row;
        row_z_prev_1 = init_row;
        row_z_prev_2 = init_row;
        row_z_prev_3 = init_row;
        ROS_INFO("Temp row storage vectors initilised ");

        // ros::Duration(5).sleep(); // This is ESSENTIAL to avoid crash
        cout << "\033[1;34mScanner setup complete..\033[0m\n";


    } else {
        //loop through all of the measured angles

        for (int i = ping_index_min_; i <= ping_index_max_; i++) {
            ping_dist_ = laser_scan.ranges[i];    //store ith distance
            phi_ = (i - ping_index_min_) * angle_increment_ + angle_min_;    //store ith angle
            z_ = ping_dist_ * cos(phi_);  //convert from polar to height above belt
            if (z_ > 1) { z_ = belt_depth_; }
            if (z_ < 0.1) { z_ = 1; }
            // row.z.push_back(z_);  //add current height to row
            row_z.push_back(z_);


            if (z_ < (belt_depth_ - tolerance_)) { //if something is measured at a height higher than the belt
                part_on_belt_ = true;    //flag that a part is currently on the belt
            }
        }
        row_z_prev_0 = row_z_prev_1;
        row_z_prev_1 = row_z_prev_2;
        row_z_prev_2 = row_z_prev_3;
        row_z_prev_3 = row_z;
        time0_0 = time0_1;
        time0_1 = time0_2;
        time0_2 = time0_3;
        time0_3 = ros::Time::now().toSec();
        ros_time0_0 = ros_time0_1;
        ros_time0_1 = ros_time0_2;
        ros_time0_2 = ros_time0_3;
        ros_time0_3 = ros::Time::now();

    }

    if (part_on_belt_ || vec_part_on_belt[1] ||
        vec_part_on_belt[0]) {    //if there is a part on the belt or there was a part 1 or 2 steps ago

        //Initialise the vector for triple confirmation
        vec_part_on_belt[0] = vec_part_on_belt[1]; //vec[0] stores the 2 steps back state
        vec_part_on_belt[1] = vec_part_on_belt[2]; //vec[1] store the 1 step back state
        vec_part_on_belt[2] = part_on_belt_; //vec[2] store the current state

        if (vec_part_on_belt[0] == false && vec_part_on_belt[1] == true &&
            vec_part_on_belt[2] == true) { // unlock the triple lock
//	   		ROS_INFO("saving the first row");
            image_z.push_back(row_z_prev_0);
            image_z.push_back(row_z_prev_1);
            image_z.push_back(row_z_prev_2);
            image_z.push_back(row_z);
            // saving the first time stamp
            time_0 = time0_0;
            ros_time_0 = ros_time0_0;

        } else if ((vec_part_on_belt[0] == true && vec_part_on_belt[1] == true && vec_part_on_belt[2] == true) ||
                   (vec_part_on_belt[0] == true && vec_part_on_belt[1] == true && vec_part_on_belt[2] == false)) {
            // ROS_INFO("saving rows in between");
            image_z.push_back(row_z);

        } else if (vec_part_on_belt[0] == true && vec_part_on_belt[1] == false && vec_part_on_belt[2] == false) {

//	   			ROS_INFO("saving the last row");

            // saving the last time stamp and last row

            time_1 = ros::Time::now().toSec();
            ros_time_1 = ros::Time::now();

            image_z.push_back(row_z);
            item_count = item_count + 1;

            // Then publish the image and reset the vectors
            image_x_size = image_z.size();
            image_y_size = row_z.size();

            cout << "image_x_size " << image_x_size << " image_y_size " << image_y_size << "\n"; //DEBUG TODO delete

            cv::Mat image(image_x_size, image_y_size, CV_64FC1);
            cv::Mat image_resized(image_x_size, image_y_size, CV_64FC1);

            string imgName = "RN_" + to_string(item_count) + ".jpg";
            string imgName_dewarped = "RN_" + to_string(item_count) + "_dewarped.jpg";
            string fileName = "RN_" + to_string(item_count) + "_height_info.txt";
            string fileName_2 = "RN_" + to_string(item_count) + "_dewarped_depth_info.csv";
            string fileName_3 = "RN_" + to_string(item_count) + "_id.txt";

            // calculate a depth image
            for (int i = 0; i < image_x_size; i++) {
                for (int j = 0; j < image_y_size; j++) {

                    image_z.at(i).at(j) = (int) 255 * (-image_z.at(i).at(j) + belt_depth_) / (part_z_max);
                    image.at<double>(i, j) = image_z.at(i).at(j);

                }
            }

            // dewarp the image
            int y_pixel_number;
            double scan_height_; // scan image height not part height

            // y_pixel_number = 101*(time_1 - time_0)*belt_speed/scan_width_;
            y_pixel_number = 400 * (time_1 - time_0) * belt_speed / scan_width_;
            scan_height_ = (time_1 - time_0) * belt_speed;

            // y_pixel_number could be 0 due to noise
            // Must avoid
            if ((y_pixel_number > 3) && (scan_height_ > 0)) {

                cout << "y_pixel_number: " << y_pixel_number << "\n";
                // cv::Size size(101,y_pixel_number);
                cv::Size size(400, y_pixel_number);
                resize(image, image_resized, size);

                image_z.clear();

                cv::Size size_dewarped = image_resized.size();
                int image_resized_x = size_dewarped.height;
                int image_resized_y = size_dewarped.width;

                // Identify part here
                part_identification(image_resized, part_id);


                // Find the a type A part's centre and asymmetric point
                if (part_id == 1 || part_id == 2 || part_id == 3) {
                    type_a_stamped_center(image_resized, time_0, time_1, t_stamp, ros_time_0, ros_time_1, ros_t_stamp,
                                          origin, origin_pixel, part_id);

                    type_a_asymmetric(image_resized, scan_height_, pin_origin, origin_pixel, part_id);
                } else if (part_id == 4) {
                    // Piston Rod
                    // Because we use a method with greater computational price to search for the origin (circular window)
                    // we need to first determine the centre point of the geometry centre
                    // then search define a square range based on this centre to search for the origin, thus reduce the loops.

                    // Old Mk1 functions
                    // type_b_stamped_center(image_resized, time_0, time_1, t_stamp, ros_time_0, ros_time_1, ros_t_stamp, origin, origin_pixel);
                    // type_b_asymmetric(image_resized, scan_height_, pin_origin, origin_pixel, part_id);

                    // New Mk2 functions
                    type_b_asymmetric(image_resized, scan_height_, pin_origin, pin_pixel, part_id);
                    type_b_stamped_center(image_resized, scan_height_, time_0, time_1, t_stamp, ros_time_0, ros_time_1,
                                          ros_t_stamp, origin, pin_pixel, origin_pixel);


                }
                // put the info into a part type
                if ((!origin.empty()) && (!pin_origin.empty())) {
                    integrate_info(part_id, origin, pin_origin, ros_t_stamp, belt_speed);
                } else {
                    ROS_WARN("Cannot generate a Part message!");
                }


            } else {
                ROS_WARN("Huge Noise Detected -- This image will not be saved.. Discard Image and Continue.");
                cout << "y_pixel_number: [DEBUG]  " << y_pixel_number << "\n";
            }


        }
    } // ..if (part_on_belt_ || vec_part_on_belt[1] || vec_part_on_belt[0])
} // ..laserCallback
