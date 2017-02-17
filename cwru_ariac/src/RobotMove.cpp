//
// Created by tianshipei on 12/4/16.
//

#include "RobotMove.h"

RobotMove::RobotMove(ros::NodeHandle &nodeHandle): nh_(nodeHandle) {
    //robot = nh_.serviceClient<cwru_ariac::RobotMoveAction>("/cwru_ariac/robot_move");
}
bool RobotMove::planToHome() {

}
bool RobotMove::pick(Part part, double timeout) {

}
bool RobotMove::place(Part destination, double timeout) {

}
bool RobotMove::move(Part part, Part destination, double timeout) {
    return true;
}
void RobotMove::setJointValues(vector<double> joints, double timeout) {

}
void RobotMove::grab() {

}
void RobotMove::release() {

}
bool RobotMove::isGripperAttached() {

}
bool RobotMove::waitForGripperAttach(double timeout) {

}
bool RobotMove::getRobotState(RobotState &robotState) {
    return true;
}