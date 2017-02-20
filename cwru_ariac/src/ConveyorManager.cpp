//
// Created by shipei on 11/1/16.
//

#include "ConveyorManager.h"

ConveyorManager::ConveyorManager(ros::NodeHandle nodeHandle, RobotPlanner &planner, RobotMove &robot): nh_(nodeHandle),
                                                                                                       planner_(&planner),
                                                                                                       robot_(&robot){
}

Part ConveyorManager::getClosestPart(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
}

Part ConveyorManager::getClosestPartBest(PartList searchRange) {
    return *min_element(searchRange.begin(), searchRange.end(), [this](Part a, Part b) {
        double aTime = 0, bTime = 0;
        if (planner_->planPart(a))
            aTime = planner_->getLastExecutingTime();
        if (planner_->planPart(b))
            bTime = planner_->getLastExecutingTime();
        return aTime < bTime; });
}

PartList ConveyorManager::getClosestPartList(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b){
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position)); });
    return searchRange;
}

PartList ConveyorManager::getClosePartListBest(PartList searchRange) {
    sort(searchRange.begin(), searchRange.end(), [this](Part a, Part b) {
        double aTime = 0, bTime = 0;
        if (planner_->planPart(a))
            aTime = planner_->getLastExecutingTime();
        if (planner_->planPart(b))
            bTime = planner_->getLastExecutingTime();
        return aTime < bTime; });
    return searchRange;
}