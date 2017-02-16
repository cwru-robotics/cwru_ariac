//
// Created by tianshipei on 11/29/16.
//

#include <Planner.h>

class PlannerService {
public:
    PlannerService(ros::NodeHandle nodeHandle): nh(nodeHandle), planner(nh) {
        service = nh.advertiseService("/cwru_ariac/oracle", &PlannerService::callback, this);
    }
private:
    ros::ServiceServer service;
    ros::NodeHandle nh;
    Planner planner;

    bool callback(OracleQueryRequest& request, OracleQueryResponse& response)
    {
        switch (request.type) {
            case OracleQueryRequest::NONE:
                response.success = (unsigned char) true;
                response.errorCode = OracleQueryResponse::NO_ERROR;
                break;
            case OracleQueryRequest::PICK:
                response.success = (unsigned char) planner.pick(request.sourcePart, response.planningTime, response.executingTime,
                                                                (int&)response.errorCode, response.planID);
                break;
            case OracleQueryRequest::PLACE:
                response.success = (unsigned char) planner.place(request.sourcePart, response.planningTime, response.executingTime,
                                                                 (int&)response.errorCode, response.planID);
                break;
            case OracleQueryRequest::MOVE:
                response.success = (unsigned char) planner.move(request.sourcePart, request.targetPart, response.planningTime, response.executingTime,
                                                                (int&)response.errorCode, response.planID);
                break;
            case OracleQueryRequest::SET_MAX_PLANNING_TIME:
                planner.setMaxPlanningTime(request.maxPlanningTime);
                response.success = (unsigned char) true;
                response.errorCode = OracleQueryResponse::NO_ERROR;
                break;
            default:
                response.success = (unsigned char) false;
                response.errorCode = OracleQueryResponse::INVALID_QUERY;
        }
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oracle");
    PlannerService plannerService(ros::NodeHandle nh);
    ROS_INFO("Start planner service.");
    ros::spin();
    return 0;
}