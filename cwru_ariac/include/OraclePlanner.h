//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ORACLEPLANNER_H
#define CWRU_ARIAC_ORACLEPLANNER_H

#include <AriacBase.h>

class OraclePlanner {
public:
    OraclePlanner(ros::NodeHandle& nodeHandle);
    bool pick(Part part, RobotState robotState, double &executingTime = _placeHolderDouble, double &planningTime = _placeHolderDouble, int &errorCode = _placeHolderInt,
                  int &planID = _placeHolderInt);
    bool place(Part destination, RobotState robotState, double &executingTime = _placeHolderDouble, double &planningTime = _placeHolderDouble, int &errorCode = _placeHolderInt,
               int &planID = _placeHolderInt);
    bool move(Part part, Part destination, RobotState robotState, double &executingTime = _placeHolderDouble, double &planningTime = _placeHolderDouble, int &errorCode = _placeHolderInt,
              int &planID = _placeHolderInt);
    bool setMaxPlanningTime(double maxPlanningTime, int &errorCode = _placeHolderInt);
    double getMaxPlanningTime() {return maxPlanningTime;}

protected:
    ros::NodeHandle nh;
    ros::ServiceClient oracle;
    double maxPlanningTime;
};
#endif //CWRU_ARIAC_ORACLEPLANNER_H
