//
// Created by shipei on 11/8/16.
//

#include "Cheater.h"

Cheater::Cheater(ros::NodeHandle &nodeHandle) : nh(nodeHandle) {
    conveyorStateSubscriber = nh.subscribe(
            "/ariac/conveyor/state", 10,
            &Cheater::conveyorStateCallback, this);
    populationStateSubscriber = nh.subscribe(
            "/ariac/population/state", 10,
            &Cheater::populationStateCallback, this);
    conveyorControl = nh.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    populationControl = nh.serviceClient<osrf_gear::PopulationControl>("/ariac/population/control");
    conveyorCalled = false;
    populationCalled = false;
    while(!conveyorCalled || !populationCalled && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    if (!conveyorControl.exists()) {
        conveyorControl.waitForExistence();
    }
    if (!populationControl.exists()) {
        populationControl.waitForExistence();
    }
    conveyorMaxSpeed = 0.2;
}

void Cheater::conveyorStateCallback(const osrf_gear::ConveyorBeltState::ConstPtr &conveyorStateMsg) {
    conveyorBeltState = *conveyorStateMsg;
    conveyorCalled = true;
}
void Cheater::populationStateCallback(const osrf_gear::PopulationState::ConstPtr &populationStateMsg) {
    populationState = *populationStateMsg;
    populationCalled = true;
}
bool Cheater::setConveyorSpeed(double speed) {
    osrf_gear::ConveyorBeltControl service;
    double power = fmin(100, speed / conveyorMaxSpeed);
    ROS_INFO("Conveyor speed is limited to %f m/s", conveyorMaxSpeed);
    service.request.state.power = power;
    if (conveyorControl.call(service))
        return service.response.success;
    return false;
}
double Cheater::getConveyorSpeed() {
    return conveyorBeltState.power * conveyorMaxSpeed;
}
bool Cheater::pausePopulation() {
    osrf_gear::PopulationControl service;
    service.request.action = "pause";
    if (populationControl.call(service))
        return service.response.success;
    return false;
}
bool Cheater::resumePopulation() {
    osrf_gear::PopulationControl service;
    service.request.action = "resume";
    if (populationControl.call(service))
        return service.response.success;
    return false;
}
bool Cheater::restartPopulation() {
    osrf_gear::PopulationControl service;
    service.request.action = "restart";
    if (populationControl.call(service))
        return service.response.success;
    return false;
}
bool Cheater::isPopulationEnabled() {
    return populationState.enabled;
}