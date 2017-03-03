//
// Created by tianshipei on 12/3/16.
//

#include "GlobalPlanner.h"

GlobalPlanner::GlobalPlanner(ros::NodeHandle nodeHandle, OraclePlanner &planner, RobotMove &robot):
        nh_(nodeHandle), planner_(&planner), robot_(&robot){
    allow_planning = true;
}

GlobalPlanner::GlobalPlanner(ros::NodeHandle nodeHandle, RobotMove &robot):
        nh_(nodeHandle), robot_(&robot){
    allow_planning = false;
}

Part GlobalPlanner::getClosestPart(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
}

Part GlobalPlanner::getClosestPartBest(PartList searchRange) {
    if (!allow_planning) {
        ROS_ERROR("Not allowed to call getClosestPartBest without Planner");
        return searchRange[0];
    }
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b) {
        double aTime = INFINITY, bTime = INFINITY;
        planner_->pick(a, state, aTime);
        planner_->pick(b, state, bTime);
        return aTime < bTime; });
}

PartList GlobalPlanner::getClosestPartList(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
    return searchRange;
}

PartList GlobalPlanner::getClosePartListBest(PartList searchRange) {
    if (!allow_planning) {
        ROS_ERROR("Not allowed to call getClosePartListBest without Planner");
        return searchRange;
    }
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b) {
        double aTime = INFINITY, bTime = INFINITY;
        planner_->pick(a, state, aTime);
        planner_->pick(b, state, bTime);
        return aTime < bTime; });
    return searchRange;
}