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

using namespace std;
using namespace Eigen;
using namespace cwru_ariac;

class RobotMoveActionServer {
private:
    ros::NodeHandle nh;
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
    Eigen::VectorXd q_des_7dof_,q_cruise_pose_,bin_cruise_jspace_pose_,agv_hover_pose_,bin_hover_jspace_pose_;
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
public:
    RobotMoveActionServer(ros::NodeHandle nodeHandle, string topic);
    void executeCB(const cwru_ariac::RobotMoveGoalConstPtr &goal);
    void preemptCB();
};


#endif //ROBOT_MOVE_AS_ROBOT_MOVE_AS_H
