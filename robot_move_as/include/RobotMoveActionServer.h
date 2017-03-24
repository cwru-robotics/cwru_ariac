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

#include <RobotPlanner.h>

using namespace std;
using namespace Eigen;
using namespace cwru_ariac;

const double PISTON_ROD_PART_THICKNESS=0.0074;
const double GEAR_PART_THICKNESS = 0.0127;
const double DISK_PART_THICKNESS = 0.0247;
const double GASKET_PART_THICKNESS = 0.0336;

//surface heights:
const double TRAY1_HEIGHT = 0.755+0.005; //pad tray height as manual fix...gravity droop problem?
const double BIN_HEIGHT = 0.725;
const double CONVEYOR_HEIGHT = 0.907;
const double BASE_LINK_HEIGHT = 1.0;


class RobotMoveActionServer {
private:
    ros::NodeHandle nh;
    RobotPlanner robotPlanner;
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
    trajectory_msgs::JointTrajectory jspace_pose_to_traj(Eigen::VectorXd joints);
    void move_to_jspace_pose(Eigen::VectorXd q_vec);
    Eigen::VectorXd q_des_7dof_,q_cruise_pose_,bin_cruise_jspace_pose_,bin_hover_jspace_pose_;
    Eigen::VectorXd agv_hover_pose_,agv_cruise_pose_;
    Eigen::VectorXd pickup_jspace_pose_,dropoff_jspace_pose_;
    Eigen::VectorXd approach_pickup_jspace_pose_,approach_dropoff_jspace_pose_;
    Eigen::VectorXd q_agv1_hover_pose_,q_agv1_cruise_pose_;  
    Eigen::VectorXd q_agv2_hover_pose_,q_agv2_cruise_pose_;      
    Eigen::VectorXd q_bin8_cruise_pose_,q_bin8_hover_pose_,q_bin8_retract_pose_;    
    Eigen::VectorXd q_bin7_cruise_pose_,q_bin7_hover_pose_,q_bin7_retract_pose_;  
    Eigen::VectorXd q_bin6_cruise_pose_,q_bin6_hover_pose_,q_bin6_retract_pose_;  
    Eigen::VectorXd q_bin5_cruise_pose_,q_bin5_hover_pose_,q_bin5_retract_pose_;  
    Eigen::VectorXd q_bin4_cruise_pose_,q_bin4_hover_pose_,q_bin4_retract_pose_;  
    Eigen::VectorXd q_bin3_cruise_pose_,q_bin3_hover_pose_,q_bin3_retract_pose_;  
    Eigen::VectorXd q_bin2_cruise_pose_,q_bin2_hover_pose_,q_bin2_retract_pose_;  
    Eigen::VectorXd q_bin1_cruise_pose_,q_bin1_hover_pose_,q_bin1_retract_pose_;

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
