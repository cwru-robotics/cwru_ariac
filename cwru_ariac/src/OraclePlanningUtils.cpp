//
// Created by tianshipei on 4/24/17.
//

#include "OraclePlanningUtils.h"

OraclePlanningUtils::OraclePlanningUtils(ros::NodeHandle &nodeHandle, OraclePlanner &oraclePlanner, RobotMove &robot) :
        planner_(&oraclePlanner), PlanningUtils(nodeHandle, robot) {
    approachTimes = 5;
    approachAheadTime = 0.5;
}

Part OraclePlanningUtils::getArrivalBestPart(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b) {
        double aTime = INFINITY, bTime = INFINITY;
        planner_->pick(a, state, aTime);
        planner_->pick(b, state, bTime);
        return aTime < bTime;
    });
}

PartList OraclePlanningUtils::sortByArrivalTime(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b) {
        double aTime = INFINITY, bTime = INFINITY;
        planner_->pick(a, state, aTime);
        planner_->pick(b, state, bTime);
        return aTime < bTime;
    });
    return searchRange;
}

bool OraclePlanningUtils::estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose) {
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