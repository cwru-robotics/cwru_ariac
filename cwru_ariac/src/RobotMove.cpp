//
// Created by tianshipei on 12/4/16.
//

#include "RobotMove.h"

RobotMove::RobotMove(ros::NodeHandle &nodeHandle): nh_(nodeHandle) {
    robot = nh_.serviceClient<cwru_ariac::RobotMoveAction>("/cwru_ariac/robot_move");
}
bool RobotMove::move(Part part, Part destination, double timeout) {

}
bool RobotMove::getRobotState(RobotState &robotState) {
    return true;
}