//
// Created by shipei on 11/8/16.
//

#include "Cheater.h"

Cheater::Cheater(ros::NodeHandle nodeHandle) : nh_(nodeHandle) {
    conveyorStateSubscriber = nh_.subscribe(
            "/ariac/conveyor/state", 10,
            &Cheater::conveyorStateCallback, this);
    populationStateSubscriber = nh_.subscribe(
            "/ariac/population/state", 10,
            &Cheater::populationStateCallback, this);
    conveyorControl = nh_.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    populationControl = nh_.serviceClient<osrf_gear::PopulationControl>("/ariac/population/control");
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
}

void Cheater::conveyorStateCallback(const osrf_gear::ConveyorBeltState::ConstPtr &conveyorStateMsg) {
    conveyorBeltState = *conveyorStateMsg;
    conveyorCalled = true;
}
void Cheater::populationStateCallback(const osrf_gear::PopulationState::ConstPtr &populationStateMsg) {
    populationState = *populationStateMsg;
    populationCalled = true;
}
bool Cheater::setConveyorVelocity(double velocity) {
    osrf_gear::ConveyorBeltControl service;
    service.request.state.velocity = velocity;
    if (conveyorControl.call(service))
        return service.response.success;
    return false;
}
double Cheater::getConveyorVelocity() {
    return conveyorBeltState.velocity;
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