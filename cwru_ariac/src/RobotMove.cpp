//
// Created by tianshipei on 12/4/16.
//edited wsn 2/18/17
//

#include "RobotMove.h"

RobotMove::RobotMove(ros::NodeHandle &nodeHandle): nh_(nodeHandle), ac("/robot_move", true) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    time_tolerance = 1.0;
}
bool RobotMove::toHome(double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::TO_HOME;
    goal.timeout = timeout;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::toCruisePose(double timeout) {
    ROS_INFO("requesting move to cruise pose");
    goal.timeout = timeout;
    goal.type = RobotMoveGoal::TO_CRUISE_POSE;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;    
}

bool RobotMove::toAgv1HoverPose(double timeout) {
    ROS_INFO("requesting move to agv1 hover pose");
    goal.timeout = timeout;
    goal.type = RobotMoveGoal::TO_AGV1_HOVER_POSE;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;    
}

bool RobotMove::toPredefinedPose(int8_t predefined_pose_code) {
    ROS_INFO("requesting move to  pose code %d",predefined_pose_code);
    
    goal.type = RobotMoveGoal::TO_PREDEFINED_POSE;
    goal.predfinedPoseCode = predefined_pose_code;
    sendGoal(goal);
    double timeout= 5.0; //hard-coded timeout; FIX THIS
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;        
}



bool RobotMove::pick(Part part, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::PICK;
    goal.timeout = timeout;
    goal.sourcePart = part;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::place(Part destination, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::PLACE;
    goal.timeout = timeout;
    goal.targetPart = destination;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::move(Part part, Part destination, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::MOVE;
    goal.timeout = timeout;
    goal.sourcePart = part;
    goal.targetPart = destination;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::setJointValues(vector<double> joints, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::PLACE;
    goal.timeout = timeout;
    goal.jointsValue = joints;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::grab() {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::GRASP;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(time_tolerance + 1.0));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::release() {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::RELEASE;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(time_tolerance + 1.0));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::isGripperAttached() {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::IS_ATTACHED;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(time_tolerance + 1.0));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::waitForGripperAttach(double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::WAIT_FOR_ATTACHED;
    goal.timeout = timeout;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    return finished_before_timeout && goal_success_;
}

bool RobotMove::getRobotState(RobotState &robotState) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::GET_ROBOT_STATE;
    sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(time_tolerance + 1.0));
    if (!finished_before_timeout)
        errorCode = RobotMoveResult::TIMEOUT;
    robotState = currentRobotState;
    return finished_before_timeout && goal_success_;
}
vector<double> RobotMove::getJointsState() {
    RobotState robotState;
    bool goal_success_ = getRobotState(robotState);
    return robotState.jointStates;
}

void RobotMove::activeCb() {

}

void RobotMove::showJointState(vector<string> joint_names, vector<double> joint_values) {
    ROS_INFO("Current joints state: {");
    for (int i = 0; i < joint_names.size(); i++) {
        cout << "    [" << i + 1 << "] " << joint_names[i] << ":\t\t\t" <<	joint_values[i] << ",\n";
    }
    cout << "}" << endl;
}

void RobotMove::doneCb(const actionlib::SimpleClientGoalState &state, const RobotMoveResultConstPtr &result) {
    action_server_returned_=true;
    goal_success_ = result->success;
    errorCode = result->errorCode;
    currentRobotState = result->robotState;
    ROS_INFO("Action finished in state [%s]: %s with error code: %d",
             state.toString().c_str(), goal_success_?"success":"failed", errorCode);
    ROS_INFO("Gripper position is: %f, %f, %f\n",
             currentRobotState.gripperPose.pose.position.x, currentRobotState.gripperPose.pose.position.y,
             currentRobotState.gripperPose.pose.position.z);
    showJointState(currentRobotState.jointNames, currentRobotState.jointStates);
}

void RobotMove::feedbackCb(const RobotMoveFeedbackConstPtr &feedback) {
    currentRobotState = feedback->robotState;
}