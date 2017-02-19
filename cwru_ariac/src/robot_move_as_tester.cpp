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
#include <cwru_ariac/RobotMoveGoal.h>
using namespace std;
using namespace Eigen;
using namespace cwru_ariac;

//Part g_pick_part,g_place_part;
void set_part_vals(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    pick_pose.header.frame_id="world";
    pick_pose.pose.orientation.x=0;
    pick_pose.pose.orientation.y=0;
    pick_pose.pose.orientation.z=0;
    pick_pose.pose.orientation.w=1;
    
    pick_pose.pose.position.x = -0.369; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = -0.597;
    pick_pose.pose.position.z = 0.726;    
    
    place_pose = pick_pose; //use same frame and orientation; change position
    place_pose.pose.position.x = 0.300; //AGV1 frame: Translation: [0.300, 3.300, 0.750]
    place_pose.pose.position.y = 3.300;
    place_pose.pose.position.z = 0.750;    
    
    pick_part.name="gear_part";
    pick_part.location= Part::BIN6;
    pick_part.pose = pick_pose;
    place_part = pick_part;
    place_part.pose = place_pose;
    place_part.location= Part::AGV1;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_tester"); //name this node
    ros::NodeHandle nh;
    Part pick_part,place_part;
    RobotMove robotMove(nh);
    RobotMoveGoal goal;
    set_part_vals(pick_part,place_part); //populate with test vals
    
    //populate a goal message for manipulation
    goal.sourcePart=pick_part;
    goal.targetPart=place_part;
    goal.type = RobotMoveGoal::MOVE;
    goal.timeout = 10.0;
    
    robotMove.sendGoal(goal);
 
    //robotMove.grab();
    //robotMove.release();

    //robotMove.toCruisePose(0.0);
    //robotMove.toAgv1HoverPose(0.0);
    //robotMove.toPredefinedPose(RobotMoveGoal::AGV1_HOVER_POSE);
    //robotMove.toPredefinedPose(RobotMoveGoal::BIN8_CRUISE_POSE);
}
