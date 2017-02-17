//
// Created by shipei on 2/17/17.
//

#include <RobotMoveActionServer.h>

RobotMoveActionServer::RobotMoveActionServer(ros::NodeHandle nodeHandle, string topic):
        nh(nodeHandle), as(nh, topic, boost::bind(&RobotMoveActionServer::executeCB, this, _1), false) {
    as.registerPreemptCallback(boost::bind(&RobotMoveActionServer::preemptCB, this));
}

void RobotMoveActionServer::executeCB(const cwru_ariac::RobotMoveGoalConstPtr &goal) {
    switch (goal->type) {
        case RobotMoveGoal::NONE:
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            break;
        case RobotMoveGoal::PICK:
            break;
        case RobotMoveGoal::PLACE:
            break;
        case RobotMoveGoal::MOVE:
            break;
        case RobotMoveGoal::TO_HOME:
            break;
        case RobotMoveGoal::SET_JOINT_VALUE:
            break;
        case RobotMoveGoal::GET_ROBOT_STATE:
            break;
        case RobotMoveGoal::GRASP:
            break;
        case RobotMoveGoal::RELEASE:
            break;
        case RobotMoveGoal::IS_ATTACHED:
            break;
        case RobotMoveGoal::WAIT_FOR_ATTACHED:
            break;
        default:
            ROS_INFO("Wrong parameter received for goal");
    }
    as.setSucceeded(result_);
}

void RobotMoveActionServer::preemptCB() {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_as"); //name this node
    ros::NodeHandle nh;
    RobotMoveActionServer actionServer(nh, "robot_move");
    ros::spin();
    return 0;
}