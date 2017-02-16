//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ORACLEPLANNER_H
#define CWRU_ARIAC_ORACLEPLANNER_H

#include <AriacBase.h>

class OraclePlanner {
public:
    OraclePlanner(ros::NodeHandle& nodeHandle);
    bool pick(Part part, RobotState robotState, double &planningTime = _fakeDouble, double &executingTime = _fakeDouble, int &errorCode = _fakeInt, int &planID = _fakeInt);
    bool place(Part destination, RobotState robotState, double &planningTime = _fakeDouble, double &executingTime = _fakeDouble, int &errorCode = _fakeInt, int &planID = _fakeInt);
    bool move(Part part, Part destination, RobotState robotState, double &planningTime = _fakeDouble, double &executingTime = _fakeDouble, int &errorCode = _fakeInt, int &planID = _fakeInt);
    bool setMaxPlanningTime(double maxPlanningTime, int &errorCode = _fakeInt);
    double getMaxPlanningTime() {return maxPlanningTime;}

private:
    ros::NodeHandle nh_;
    ros::ServiceClient oracle;
    double maxPlanningTime;
};
#endif //CWRU_ARIAC_ORACLEPLANNER_H
