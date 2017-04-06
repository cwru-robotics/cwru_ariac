//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ROBOTPLANNER_H
#define CWRU_ARIAC_ROBOTPLANNER_H

#include <AriacBase.h>

class Planner {
public:
    Planner(ros::NodeHandle& nodeHandle);
    bool pick(Part part, double &planningTime = _placeHolderDouble, double &executingTime = _placeHolderDouble, int &errorCode = _placeHolderInt, int &planID = _placeHolderInt);
    bool place(Part destination, double &planningTime = _placeHolderDouble, double &executingTime = _placeHolderDouble, int &errorCode = _placeHolderInt, int &planID = _placeHolderInt);
    bool move(Part part, Part destination, double &planningTime = _placeHolderDouble, double &executingTime = _placeHolderDouble, int &errorCode = _placeHolderInt, int &planID = _placeHolderInt);
    bool estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose);
    vector<double> getJointsState();

    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    double getMaxPlanningTime() { return maxPlanningTime;}

private:
    ros::NodeHandle nh_;

    vector<trajectory_msgs::JointTrajectory> plans;
    int assignedID;
    double maxPlanningTime;     // deadline for planning, must return before this time
    ros::Subscriber joint_state_subscriber;
    sensor_msgs::JointState current_joint_states;
    bool called;
    double arrivalTime;
    double approachTimes;
    double approachAheadTime;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
};


#endif //CWRU_ARIAC_ROBOTPLANNER_H
