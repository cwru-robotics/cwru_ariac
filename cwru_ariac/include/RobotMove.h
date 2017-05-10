//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ROBOTMOVE_H
#define CWRU_ARIAC_ROBOTMOVE_H
  

#include <AriacBase.h>

class RobotMove {
public:
    RobotMove(ros::NodeHandle& nodeHandle, string topic = "/robot_move");
    bool toHome(double timeout = 0);

    bool toPredefinedPose(int8_t goal_code, double timeout = 0);
    bool pick(Part part, double timeout = 0);
    bool place(Part destination, double timeout = 0);
    bool move(Part part, Part destination, double timeout = 0);
    bool fetchPartFromConveyor(Part part,Part destination, double timeout=0);
    bool flipPart(Part part, double timeout = 0);
    bool setJointValues(vector<double> joints, double timeout = 0);
    bool grasp(double timeout = 0);
    bool release(double timeout = 0);
    bool isGripperAttached();
    bool getRobotState(RobotState& robotState);
    vector<double> getJointsState();
    void sendGoal(RobotMoveGoal goal);
    void showJointState(vector<string> joint_names, vector<double> joint_values);
    void cancel();
    int8_t getErrorCode() { return errorCode; }
    string getErrorCodeString() { return errorCodeFinder[getErrorCode()]; }
    bool getResult() { return goal_success_; }
    bool actionFinished() { return action_server_returned_; }
    void setTimeTolerance(double newTimeTolerance) { time_tolerance = newTimeTolerance; }
    void enableAsync() { async_mode = true; }
    void disableAsync() { async_mode = false; }

protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<cwru_ariac::RobotMoveAction> ac;
    int8_t errorCode;
    bool goal_success_;
    RobotState currentRobotState;
    bool action_server_returned_;
    double time_tolerance;
    bool async_mode;
    unordered_map<int8_t, string> errorCodeFinder;

    void doneCb(const actionlib::SimpleClientGoalState& state, const RobotMoveResultConstPtr& result);
    void activeCb();
    void feedbackCb(const RobotMoveFeedbackConstPtr& feedback);
};


#endif //CWRU_ARIAC_ROBOTMOVE_H
