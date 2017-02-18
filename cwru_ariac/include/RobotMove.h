//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ROBOTMOVE_H
#define CWRU_ARIAC_ROBOTMOVE_H


#include <AriacBase.h>

class RobotMove {
public:
    double time_tolerance;

    RobotMove(ros::NodeHandle& nodeHandle);
    bool toHome(double timeout = 0);
    bool toCruisePose(double timeout = 0);
    bool toAgv1HoverPose(double timeout = 0);
    bool toPredefinedPose(int8_t goal_code);
    bool pick(Part part, double timeout = 0);
    bool place(Part destination, double timeout = 0);
    bool move(Part part, Part destination, double timeout = 0);
    bool setJointValues(vector<double> joints, double timeout = 0);
    bool grab();
    bool release();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout = 0);
    bool getRobotState(RobotState& robotState);
    vector<double> getJointsState();
    int8_t getErrorCode() { return errorCode; }
    void doneCb(const actionlib::SimpleClientGoalState& state, const RobotMoveResultConstPtr& result);
    void activeCb();
    void feedbackCb(const RobotMoveFeedbackConstPtr& feedback);
    void sendGoal(RobotMoveGoal goal) {
//            ac.sendGoal(goal, boost::bind(&RobotMove::doneCb, this, _1, _2), &RobotMove::activeCb, boost::bind(&RobotMove::feedbackCb, this, _1));
            ac.sendGoal(goal, boost::bind(&RobotMove::doneCb, this, _1, _2));
    }
    void showJointState(vector<string> joint_names, vector<double> joint_values);
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<cwru_ariac::RobotMoveAction> ac;
    RobotMoveGoal goal;
    int8_t errorCode;
    bool success;
    RobotState currentRobotState;
};


#endif //CWRU_ARIAC_ROBOTMOVE_H
