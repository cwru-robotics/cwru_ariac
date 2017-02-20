//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_CONVEYORMANAGER_H
#define CWRU_ARIAC_CONVEYORMANAGER_H

#include <AriacBase.h>
#include <RobotMove.h>
#include <RobotPlanner.h>

class ConveyorManager {
public:
    ConveyorManager(ros::NodeHandle nodeHandle, RobotPlanner &planner, RobotMove &robot);

    Part getClosestPart(PartList searchRange);
    PartList getClosestPartList(PartList searchRange);
    Part getClosestPartBest(PartList searchRange);
    PartList getClosePartListBest(PartList searchRange);

private:
    ros::NodeHandle nh_;
    RobotPlanner *planner_;
    RobotMove *robot_;
};


#endif //CWRU_ARIAC_CONVEYORMANAGER_H
