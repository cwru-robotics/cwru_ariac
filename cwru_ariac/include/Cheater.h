//
// Created by shipei on 11/8/16.
//

#ifndef CWRU_ARIAC_CHEATER_H
#define CWRU_ARIAC_CHEATER_H

#include <AriacBase.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/ConveyorBeltState.h>
#include <osrf_gear/PopulationControl.h>
#include <osrf_gear/PopulationState.h>

class Cheater {
public:
    Cheater(ros::NodeHandle nodeHandle);
    bool setConveyorVelocity(double velocity);
    double getConveyorVelocity();
    bool pausePopulation();
    bool resumePopulation();
    bool restartPopulation();
    bool isPopulationEnabled();

private:
    ros::NodeHandle nh_;
    ros::Subscriber conveyorStateSubscriber;
    ros::Subscriber populationStateSubscriber;
    ros::ServiceClient conveyorControl;
    ros::ServiceClient populationControl;

    void conveyorStateCallback(const osrf_gear::ConveyorBeltState::ConstPtr &conveyorStateMsg);
    void populationStateCallback(const osrf_gear::PopulationState::ConstPtr &populationStateMsg);

    osrf_gear::ConveyorBeltState conveyorBeltState;
    osrf_gear::PopulationState populationState;

    bool conveyorCalled;
    bool populationCalled;
};


#endif //CWRU_ARIAC_CHEATER_H
