//
// Created by tianshipei on 12/3/16.
//

#ifndef CWRU_ARIAC_GLOBALPLANNER_H
#define CWRU_ARIAC_GLOBALPLANNER_H

#include <AriacBase.h>
#include <RobotMove.h>
#include <OraclePlanner.h>

class GlobalPlanner {
public:
    GlobalPlanner(ros::NodeHandle nodeHandle, OraclePlanner &planner, RobotMove &robot);
    GlobalPlanner(ros::NodeHandle nodeHandle, RobotMove &robot);

    Part getClosestPart(PartList searchRange);
    PartList getClosestPartList(PartList searchRange);
    Part getClosestPartBest(PartList searchRange);
    PartList getClosePartListBest(PartList searchRange);

private:
    ros::NodeHandle nh_;
    OraclePlanner *planner_;
    RobotMove *robot_;
    bool allow_planning;
};


#endif //CWRU_ARIAC_GLOBALPLANNER_H
