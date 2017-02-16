//
// Created by tianshipei on 11/29/16.
//

#include "OraclePlanner.h"

OraclePlanner::OraclePlanner(ros::NodeHandle &nodeHandle): nh_(nodeHandle) {
    oracle = nh_.serviceClient<cwru_ariac::OracleQuery>("/cwru_ariac/oracle");
    if (!oracle.exists()) {
        oracle.waitForExistence();
    }
}
bool OraclePlanner::pick(Part part, RobotState robotState, double &planningTime, double &executingTime, int &errorCode,
                         int &planID) {
    OracleQuery query;
    query.request.type = OracleQuery::Request::PICK;
    query.request.robotState = robotState;
    query.request.sourcePart = part;
    if (oracle.call(query)) {
        planningTime = query.response.planningTime;
        executingTime = query.response.executingTime;
        errorCode = query.response.errorCode;
        planID = query.response.planID;
        return query.response.success;
    }
    return false;
}
bool OraclePlanner::place(Part destination, RobotState robotState, double &planningTime, double &executingTime,
                          int &errorCode, int &planID) {
    OracleQuery query;
    query.request.type = OracleQuery::Request::PLACE;
    query.request.robotState = robotState;
    query.request.targetPart = destination;
    if (oracle.call(query)) {
        planningTime = query.response.planningTime;
        executingTime = query.response.executingTime;
        errorCode = query.response.errorCode;
        planID = query.response.planID;
        return query.response.success;
    }
    return false;
}
bool OraclePlanner::move(Part part, Part destination, RobotState robotState, double &planningTime,
                         double &executingTime, int &errorCode, int &planID) {
    OracleQuery query;
    query.request.type = OracleQuery::Request::MOVE;
    query.request.robotState = robotState;
    query.request.sourcePart = part;
    query.request.targetPart = destination;
    if (oracle.call(query)) {
        planningTime = query.response.planningTime;
        executingTime = query.response.executingTime;
        errorCode = query.response.errorCode;
        planID = query.response.planID;
        return query.response.success;
    }
    return false;
}
bool OraclePlanner::setMaxPlanningTime(double maxPlanningTime, int &errorCode) {
    this->maxPlanningTime = maxPlanningTime;
    OracleQuery query;
    query.request.type = OracleQuery::Request::SET_MAX_PLANNING_TIME;
    query.request.maxPlanningTime = maxPlanningTime;
    if (oracle.call(query)) {
        errorCode = query.response.errorCode;
        return query.response.success;
    }
    return false;
}