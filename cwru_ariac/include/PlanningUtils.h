//
// Created by tianshipei on 12/3/16.
//

#ifndef CWRU_ARIAC_GLOBALPLANNER_H
#define CWRU_ARIAC_GLOBALPLANNER_H

#include <AriacBase.h>
#include <RobotMove.h>
#include <OraclePlanner.h>

class PlanningUtils {
public:

    PlanningUtils(ros::NodeHandle &nodeHandle, RobotMove &robot);

    Part getEuclideanBestPart(PartList searchRange);
    PartList sortByEuclidean(PartList searchRange);

    Part getTargetDistanceBestPart(PartList searchRange, Part target);
    PartList sortByTargetDistance(PartList searchRange, Part target);

    bool findDroppedParts(PartList searchList, PartList targetList, vector<pair<Part, Part>> &wrongLocationParts, PartList &lostParts, PartList &redundantParts);


protected:
    ros::NodeHandle nh;
    RobotMove *robot_;
    bool allow_planning;
    double approachTimes;
    double approachAheadTime;

    bool eval_up_down(geometry_msgs::Quaternion orientation);
};


#endif //CWRU_ARIAC_GLOBALPLANNER_H
