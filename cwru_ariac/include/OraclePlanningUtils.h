//
// Created by tianshipei on 4/24/17.
//

#ifndef CWRU_ARIAC_ORACLEPLANNINGUTILS_H
#define CWRU_ARIAC_ORACLEPLANNINGUTILS_H

#include <AriacBase.h>
#include <PlanningUtils.h>

class OraclePlanningUtils : public PlanningUtils {

    OraclePlanningUtils(ros::NodeHandle &nodeHandle, OraclePlanner &oraclePlanner, RobotMove &robot);

    Part getArrivalBestPart(PartList searchRange);

    PartList sortByArrivalTime(PartList searchRange);

    bool estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose);

protected:
    OraclePlanner *planner_;
    double approachTimes;
    double approachAheadTime;
};


#endif //CWRU_ARIAC_ORACLEPLANNINGUTILS_H
