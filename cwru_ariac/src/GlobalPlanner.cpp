//
// Created by tianshipei on 12/3/16.
//

#include "GlobalPlanner.h"

GlobalPlanner::GlobalPlanner(ros::NodeHandle nodeHandle, OraclePlanner &planner, RobotMove &robot):
        nh_(nodeHandle), planner_(&planner), robot_(&robot){
    allow_planning = true;
    approachTimes = 5;
    approachAheadTime = 0.5;
}

GlobalPlanner::GlobalPlanner(ros::NodeHandle nodeHandle, RobotMove &robot):
        nh_(nodeHandle), robot_(&robot){
    allow_planning = false;
}

Part GlobalPlanner::getEuclideanBestPart(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
}

Part GlobalPlanner::getArrivalBestPart(PartList searchRange) {
    if (!allow_planning) {
        ROS_ERROR("Not allowed to call getArrivalBestPart without Planner");
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

PartList GlobalPlanner::sortByEuclidean(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
    return searchRange;
}

PartList GlobalPlanner::sortByArrivalTime(PartList searchRange) {
    if (!allow_planning) {
        ROS_ERROR("Not allowed to call sortByArrivalTime without Planner");
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

Part GlobalPlanner::getTargetDistanceBestPart(PartList searchRange, Part target) {
    return *min_element(searchRange.begin(), searchRange.end(), [this, target](Part a, Part b){
        return euclideanDistance((target.pose.pose.position), (a.pose.pose.position))
               < euclideanDistance((target.pose.pose.position), (b.pose.pose.position)); });
}

PartList GlobalPlanner::sortByTargetDistance(PartList searchRange, Part target) {
    sort(searchRange.begin(), searchRange.end(), [this, target](Part a, Part b){
        return euclideanDistance((target.pose.pose.position), (a.pose.pose.position))
               < euclideanDistance((target.pose.pose.position), (b.pose.pose.position)); });
    return searchRange;
}


bool GlobalPlanner::estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose) {
    if (!allow_planning) {
        ROS_ERROR("Not allowed to call sortByArrivalTime without Planner");
        return false;
    }
    RobotState state;
    robot_->getRobotState(state);
    Part estimatedPart = part;
    double dplan = 0;
    double dt = 0;
    double tempExeTime = 0;
    double lastPlanningTime = INFINITY;
    double lastExecutingTime = INFINITY;
    estimatedPose = part.pose;
    estimatedPose.header.stamp = ros::Time::now();

    // planning time for the first time
    part.pose.pose.position.x += part.linear.x * dt;
    part.pose.pose.position.y += part.linear.y * dt;
    part.pose.pose.position.z += part.linear.z * dt;

    for (int i = 0; i < approachTimes; ++i) {
        if (planner_->pick(part, state, lastExecutingTime, lastPlanningTime))
            return false;
        // calculate time cost for next planPose
        dplan += lastPlanningTime;
        dt = lastExecutingTime + dplan;
        estimatedPart.pose.pose.position.x = part.pose.pose.position.x + part.linear.x * dt;
        estimatedPart.pose.pose.position.y = part.pose.pose.position.y + part.linear.y * dt;
        estimatedPart.pose.pose.position.z = part.pose.pose.position.z + part.linear.z * dt;
    }
    bool reachable = false;
    while (!reachable) {
        estimatedPart.pose.pose.position.x += part.linear.x * approachAheadTime;
        estimatedPart.pose.pose.position.y += part.linear.y * approachAheadTime;
        estimatedPart.pose.pose.position.z += part.linear.z * approachAheadTime;
        if (planner_->pick(part, state, lastExecutingTime, lastPlanningTime))
            return false;
        if (lastExecutingTime - tempExeTime + lastPlanningTime < approachAheadTime)
            reachable = true;
        // reach that pose at estimated time
        estimatedPose.header.stamp = ros::Time::now() + ros::Duration(approachAheadTime);
    }
    return true;
}