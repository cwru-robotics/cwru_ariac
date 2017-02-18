//test module for sending pick/place commands

#include <AriacBase.h>
#include <RobotMove.h>
#include <cwru_ariac/RobotMoveAction.h>

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
using namespace std;
using namespace Eigen;
using namespace cwru_ariac;


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_tester"); //name this node
    ros::NodeHandle nh;
    RobotMove robotMove(nh);
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::NONE;
    robotMove.sendGoal(goal);
 
    robotMove.grab();
    robotMove.release();

    //robotMove.toCruisePose(0.0);
    //robotMove.toAgv1HoverPose(0.0);
    robotMove.toPredefinedPose(RobotMoveGoal::AGV1_HOVER_POSE);
    robotMove.toPredefinedPose(RobotMoveGoal::BIN8_CRUISE_POSE);
}
