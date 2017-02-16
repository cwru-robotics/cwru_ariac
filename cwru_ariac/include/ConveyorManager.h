//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_CONVEYORMANAGER_H
#define CWRU_ARIAC_CONVEYORMANAGER_H

#include <AriacBase.h>
#include <CameraEstimator.h>
#include <RobotPlanner.h>

class ConveyorManager {
public:
    ConveyorManager(ros::NodeHandle nodeHandle, CameraEstimator &estimator, RobotPlanner &planner);

    Part getClosestPart();
    PartList getClosestPartList();
    Part getClosestPartBest();
    PartList getClosePartListBest();

private:
    ros::NodeHandle nh_;
    CameraEstimator *estimator_;
    RobotPlanner *planner_;
};


#endif //CWRU_ARIAC_CONVEYORMANAGER_H
