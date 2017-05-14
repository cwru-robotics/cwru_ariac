//
// Created by shipei on 2/17/17.
//

#ifndef ROBOT_MOVE_AS_ROBOT_MOVE_AS_H
#define ROBOT_MOVE_AS_ROBOT_MOVE_AS_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <actionlib/server/simple_action_server.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <cwru_ariac/RobotMoveAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <ariac_xform_utils/ariac_xform_utils.h>
#include <ariac_ur_fk_ik/ur_kin.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <RobotInterface.h>

using namespace std;
using namespace Eigen;
using namespace cwru_ariac;

//part frame notes:
// GASKET: frame is centered, but with origin at BOTTOM surface; x-axis points towards pin feature
// PISTON ROD: frame origin is at BOTTOM surface of part, centered in hole; y-axis points towards opposite end of rod
// GEAR: origin is at BOTTOM of part (centered), and Y-AXIS points towards pin feature
//DISK: origin is at BOTTOM of part (centered), pin feature is 45 deg (between x-axis and y_axis)
const double PISTON_ROD_PART_THICKNESS= 0.007; // wsn modified for qual3; 0.0075; //works for qual2
const double GEAR_PART_THICKNESS = 0.0124; // modified for qual3; 0.015; // 0.015 works for qual2
const double DISK_PART_THICKNESS = 0.0247;  //note sure about this one...maybe OK
const double GASKET_PART_THICKNESS = 0.02; // 0.0336; //wsn change for gear 1.1
//per  /opt/ros/indigo/share/osrf_gear/models/pulley_part_ariac, pulley-part collision model should be 0.0720 thick
const double PULLEY_PART_THICKNESS = 0.0728;  //0.7255 = z on bin; 0.7983 on top of another pulley: 0.0728 thickness; origin on bottom

//here are some hand-tuned "kludge" parameters to tweak grasp transforms;
const double GEAR_PART_GRASP_Y_OFFSET = 0.04;
const double DISK_PART_GRASP_Z_OFFSET = 0.005;
const double GASKET_PART_GRASP_Z_OFFSET = 0.0; //0.006;
const double GASKET_PART_GRASP_X_OFFSET = 0.03;
const double PULLEY_PART_GRASP_Z_OFFSET = 0.007; //0.01; //0.005;



 
const bool UP = true;
const bool DOWN = false;

//surface heights:
// had to increase tray height by 0.010 to get drop-off height of tray correct.  Don't know why
const double TRAY1_HEIGHT = 0.755+0.010; //pad tray height as manual fix...gravity droop problem?
const double BIN_HEIGHT = 0.725;
const double CONVEYOR_HEIGHT = 0.907;
const double BASE_LINK_HEIGHT = 1.0;

const double QUAL2_CONVEYOR_SPEED = -0.2;

const double CONVEYOR_TRACK_FUDGE_TIME = 0.0; //0.5;
const double CONVEYOR_FETCH_QRAIL_MIN = -1.0; // don't go more negative than this


class RobotMoveActionServer {
private:
    ros::NodeHandle nh;
    RobotInterface robotInterface;
    actionlib::SimpleActionServer<cwru_ariac::RobotMoveAction> as;
    cwru_ariac::RobotMoveGoal goal_;
    cwru_ariac::RobotMoveFeedback feedback_;
    cwru_ariac::RobotMoveResult result_;
    bool isPreempt;
    bool goalComplete;
    RobotState robotState;
    unordered_map<int8_t, string> placeFinder;
    ros::Publisher joint_trajectory_publisher_;
    control_msgs::FollowJointTrajectoryGoal traj_goal_;
    trajectory_msgs::JointTrajectory traj_;
    trajectory_msgs::JointTrajectory jspace_pose_to_traj(Eigen::VectorXd joints, double dtime=2.0);
    void move_to_jspace_pose(Eigen::VectorXd q_vec, double dtime=2.0);
    Eigen::VectorXd q_des_7dof_,q_cruise_pose_,bin_cruise_jspace_pose_,bin_hover_jspace_pose_;
    Eigen::VectorXd agv_hover_pose_,agv_cruise_pose_;
    Eigen::VectorXd pickup_jspace_pose_,dropoff_jspace_pose_;
    Eigen::VectorXd approach_pickup_jspace_pose_,approach_dropoff_jspace_pose_;
    Eigen::VectorXd q_agv1_hover_pose_,q_agv1_cruise_pose_;  
    Eigen::VectorXd q_agv2_hover_pose_,q_agv2_cruise_pose_;    
    Eigen::VectorXd q_conveyor_hover_pose_,q_conveyor_cruise_pose_;    
    Eigen::VectorXd q_bin8_cruise_pose_,q_bin8_hover_pose_,q_bin8_retract_pose_;    
    Eigen::VectorXd q_bin7_cruise_pose_,q_bin7_hover_pose_,q_bin7_retract_pose_;  
    Eigen::VectorXd q_bin6_cruise_pose_,q_bin6_hover_pose_,q_bin6_retract_pose_;  
    Eigen::VectorXd q_bin5_cruise_pose_,q_bin5_hover_pose_,q_bin5_retract_pose_;  
    Eigen::VectorXd q_bin4_cruise_pose_,q_bin4_hover_pose_,q_bin4_retract_pose_;  
    Eigen::VectorXd q_bin3_cruise_pose_,q_bin3_hover_pose_,q_bin3_retract_pose_;  
    Eigen::VectorXd q_bin2_cruise_pose_,q_bin2_hover_pose_,q_bin2_retract_pose_;  
    Eigen::VectorXd q_bin1_cruise_pose_,q_bin1_hover_pose_,q_bin1_retract_pose_;
    Eigen::VectorXd q_bin_pulley_flip_;
    Eigen::Affine3d grasp_transform_;
    Eigen::VectorXd j1;
    
    Eigen::Affine3d affine_vacuum_pickup_pose_wrt_base_link_;
    Eigen::Affine3d affine_vacuum_dropoff_pose_wrt_base_link_;
    //double rail_stops[];
    //unordered_map<int8_t, int> placeIndex;

    //fncs to get key joint-space poses:
    //each bin gets a corresponding rail pose; return "true" if valid bin code
    bool rail_prepose(int8_t location, double &q_rail);
    //each bin has a corresponding "hover" pose; set q_vec and return true if valid bin code
    bool bin_hover_jspace_pose(int8_t bin, Eigen::VectorXd &q_vec);
    //cruise pose depends on bin code and whether to point towards agv1 or agv2
    // provide bin code and agv code; get back q_vec to prepare for cruise to agv
    bool bin_cruise_jspace_pose(int8_t bin, int8_t agv, Eigen::VectorXd &q_vec);
    bool agv_cruise_jspace_pose(int8_t agv, Eigen::VectorXd &q_vec);
    
    //trivial func to compute affine3 for robot_base w/rt world;  only depends on rail displacement
    Eigen::Affine3d  affine_base_link(double q_rail);

    double get_pickup_offset(Part part); //fnc to return offset values for gripper: part top relative to part frame
    double get_dropoff_offset(Part part);
    double get_surface_height(Part part);
    
    //"Part" should include part pose w/rt world, so can determine if part is right-side up or up-side down
    bool get_grasp_transform(Part part,Eigen::Affine3d &grasp_transform);
    unsigned short int fetch_from_conveyor(const cwru_ariac::RobotMoveGoalConstPtr& goal); 
    unsigned short int flip_part_fnc(const cwru_ariac::RobotMoveGoalConstPtr& goal); 
    unsigned short int grasp_fnc(double timeout=2.0);  //default timeout; rtns error code
    unsigned short int release_fnc(double timeout=2.0); //default timeout for release
    unsigned short int pick_part_fnc(const cwru_ariac::RobotMoveGoalConstPtr& goal); //rtns err code; used within other fncs
    

    bool eval_up_down(geometry_msgs::PoseStamped part_pose_wrt_world);
    //given rail displacement, and given Part description (including name and pose info) compute where the gripper should be, as
    //an Affine3 w/rt base_link frame
    Eigen::Affine3d affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail);
    //similarly, compute gripper pose for dropoff, accounting for part height
    Eigen::Affine3d affine_vacuum_dropoff_pose_wrt_base_link(Part part, double q_rail);
    //do IK to place gripper at specified affine3; choose solution that is closest to provided jspace pose
    bool get_pickup_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,Eigen::VectorXd &q_vec_soln);
    //similarly for drop-off solution
    //bool get_dropoff_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,Eigen::VectorXd &q_vec_soln);
    //compute q_vec_soln corresponding to approach to specified grasp pose; specify approach distance; choose IK soln that is closest
    //to grasp IK soln
    bool compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    void grab();
    void release();  
    RobotState calcRobotState();
    osrf_gear::VacuumGripperState getGripperState();
    bool attached_;
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);
  
    ros::ServiceClient gripper_client;
    osrf_gear::VacuumGripperState currentGripperState_;
    osrf_gear::VacuumGripperControl attach_;
    osrf_gear::VacuumGripperControl detach_;
    
    tf::TransformListener* tfListener_;
    tf::StampedTransform tfCameraWrtWorld_,tfTray1WrtWorld_,tfTray2WrtWorld_;
    XformUtils xformUtils_;
    UR10FwdSolver fwd_solver_;
    UR10IkSolver ik_solver_;
    Eigen::Affine3d agv1_tray_frame_wrt_world_,agv2_tray_frame_wrt_world_;
    double approach_dist_;
public:
    RobotMoveActionServer(ros::NodeHandle nodeHandle, string topic);
    void executeCB(const cwru_ariac::RobotMoveGoalConstPtr &goal);
    void preemptCB();
};


#endif //ROBOT_MOVE_AS_ROBOT_MOVE_AS_H
