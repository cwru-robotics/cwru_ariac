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
public:
    RobotMoveActionServer(ros::NodeHandle nodeHandle, string topic);
    void executeCB(const cwru_ariac::RobotMoveGoalConstPtr &goal);
    void preemptCB();
};


#endif //ROBOT_MOVE_AS_ROBOT_MOVE_AS_H
