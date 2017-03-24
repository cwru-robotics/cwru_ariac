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

    Part getEuclideanBestPart(PartList searchRange);
    PartList sortByEuclidean(PartList searchRange);
    Part getArrivalBestPart(PartList searchRange);
    PartList sortByArrivalTime(PartList searchRange);
    Part getTargetDistanceBestPart(PartList searchRange, Part target);
    PartList sortByTargetDistance(PartList searchRange, Part target);

private:
    ros::NodeHandle nh_;
    OraclePlanner *planner_;
    RobotMove *robot_;
    bool allow_planning;
};


#endif //CWRU_ARIAC_GLOBALPLANNER_H
