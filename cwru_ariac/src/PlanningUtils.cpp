//
// Created by tianshipei on 12/3/16.
//

#include "PlanningUtils.h"

PlanningUtils::PlanningUtils(ros::NodeHandle nodeHandle, RobotMove &robot) :
        nh_(nodeHandle), robot_(&robot){
    allow_planning = false;
}

Part PlanningUtils::getEuclideanBestPart(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
}

PartList PlanningUtils::sortByEuclidean(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
    return searchRange;
}

Part PlanningUtils::getTargetDistanceBestPart(PartList searchRange, Part target) {
    return *min_element(searchRange.begin(), searchRange.end(), [this, target](Part a, Part b){
        return euclideanDistance((target.pose.pose.position), (a.pose.pose.position))
               < euclideanDistance((target.pose.pose.position), (b.pose.pose.position)); });
}

PartList PlanningUtils::sortByTargetDistance(PartList searchRange, Part target) {
    sort(searchRange.begin(), searchRange.end(), [this, target](Part a, Part b){
        return euclideanDistance((target.pose.pose.position), (a.pose.pose.position))
               < euclideanDistance((target.pose.pose.position), (b.pose.pose.position)); });
    return searchRange;
}