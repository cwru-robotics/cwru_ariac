//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ROBOTMOVE_H
#define CWRU_ARIAC_ROBOTMOVE_H


#include <AriacBase.h>

class RobotMove {
public:
    RobotMove(ros::NodeHandle& nodeHandle);
    bool planToHome();
    bool pick(Part part, double timeout);
    bool place(Part destination, double timeout);
    bool move(Part part, Part destination, double timeout);
    void sendJointsValue(vector<double> joints, double timeout);
    vector<double> getJointsState();
    void grab();
    void release();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);
    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    bool getRobotState(RobotState& robotState);

private:
    ros::NodeHandle nh_;
    ros::ServiceClient oracle;
    ros::ServiceClient robot;
    double maxPlanningTime;
    OracleQueryRequest request;
    OracleQueryResponse response;

    geometry_msgs::Pose homePose;
    RobotState currentRobotState;
};


#endif //CWRU_ARIAC_ROBOTMOVE_H
