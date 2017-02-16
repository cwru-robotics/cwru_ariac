//
// Created by shipei on 11/1/16.
//

#include "ConveyorManager.h"

ConveyorManager::ConveyorManager(ros::NodeHandle nodeHandle, CameraEstimator &estimator, RobotPlanner &planner): nh_(nodeHandle), estimator_(&estimator), planner_(&planner) {
}

Part ConveyorManager::getClosestPart() {
    return *min_element(estimator_->onConveyor.begin(), estimator_->onConveyor.end(), [this](Part a, Part b){
        return euclideanDistance((planner_->getCurrentGripperPose().position), (a.pose.pose.position))
               < euclideanDistance((planner_->getCurrentGripperPose().position), (b.pose.pose.position)); });
}

Part ConveyorManager::getClosestPartBest() {
    return *min_element(estimator_->onConveyor.begin(), estimator_->onConveyor.end(), [this](Part a, Part b) {
        double aTime = 0, bTime = 0;
        if (planner_->planPart(a))
            aTime = planner_->getLastExecutingTime();
        if (planner_->planPart(b))
            bTime = planner_->getLastExecutingTime();
        return aTime < bTime; });
}

PartList ConveyorManager::getClosestPartList() {
    PartList parts = estimator_->onConveyor;
    sort(parts.begin(), parts.end(), [this](Part a, Part b){
        return euclideanDistance((planner_->getCurrentGripperPose().position), (a.pose.pose.position))
               < euclideanDistance((planner_->getCurrentGripperPose().position), (b.pose.pose.position)); });
    return parts;
}

PartList ConveyorManager::getClosePartListBest() {
    PartList parts = estimator_->onConveyor;
    sort(parts.begin(), parts.end(), [this](Part a, Part b) {
        double aTime = 0, bTime = 0;
        if (planner_->planPart(a))
            aTime = planner_->getLastExecutingTime();
        if (planner_->planPart(b))
            bTime = planner_->getLastExecutingTime();
        return aTime < bTime; });
    return parts;
}