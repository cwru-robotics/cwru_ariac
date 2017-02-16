//
// Created by shipei on 10/18/16.
//

#include "Planner.h"

Planner::Planner(ros::NodeHandle &nodeHandle): nh_( nodeHandle ){
    joint_state_subscriber = nh_.subscribe(
            "/ariac/joint_states", 10,
            (void (Planner::*)(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr)) &Planner::jointStateCallback, this);
    called = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    arrivalTime = 0.5;
    approachTimes = 5;
    approachAheadTime = 0.5;
    assignedID = 1;
}

void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStateMsg)
{
    current_joint_states = *jointStateMsg;
    called = true;
}

bool Planner::pick(Part part, double &planningTime, double &executingTime, int &errorCode, int &planID) {
    // TODO
    planningTime = (rand()/RAND_MAX)*maxPlanningTime;
    executingTime = (rand()/RAND_MAX)*6;
    errorCode = OracleQueryResponse::NO_ERROR;
    planID = assignedID++;
    ros::spinOnce();
    return true;
}

bool Planner::place(Part destination, double &planningTime, double &executingTime, int &errorCode, int &planID) {
    // TODO
    planningTime = (rand()/RAND_MAX)*maxPlanningTime;
    executingTime = (rand()/RAND_MAX)*6;
    errorCode = OracleQueryResponse::NO_ERROR;
    planID = assignedID++;
    ros::spinOnce();
    return true;
}

bool Planner::move(Part part, Part destination, double &planningTime, double &executingTime, int &errorCode, int &planID) {
    // TODO
    planningTime = (rand()/RAND_MAX)*maxPlanningTime;
    executingTime = (rand()/RAND_MAX)*6;
    errorCode = OracleQueryResponse::NO_ERROR;
    planID = assignedID++;
    ros::spinOnce();
    return true;
}

vector<double> Planner::getJointsState() {
    called = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    vector<double> joints;
    joints.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        joints = current_joint_states.position;
    }
    return joints;
}

bool Planner::estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose) {
    Part estimatedPart = part;
    double dplan = 0;
    double dt = 0;
    double tempExeTime = 0;
    double planTime;
    double exeTime;
    estimatedPose = part.pose;
    estimatedPose.header.stamp = ros::Time::now();

    // planning time for the first time
    part.pose.pose.position.x += part.linear.x * dt;
    part.pose.pose.position.y += part.linear.y * dt;
    part.pose.pose.position.z += part.linear.z * dt;

    for (int i = 0; i < approachTimes; ++i) {
        if (pick(part, planTime, exeTime))
            return false;
        // calculate time cost for next planPose
        dplan += planTime;
        dt = exeTime + dplan;
        estimatedPart.pose.pose.position.x = part.pose.pose.position.x + part.linear.x * dt;
        estimatedPart.pose.pose.position.y = part.pose.pose.position.y + part.linear.y * dt;
        estimatedPart.pose.pose.position.z = part.pose.pose.position.z + part.linear.z * dt;
    }
    bool reachable = false;
    while (!reachable) {
        estimatedPart.pose.pose.position.x += part.linear.x * approachAheadTime;
        estimatedPart.pose.pose.position.y += part.linear.y * approachAheadTime;
        estimatedPart.pose.pose.position.z += part.linear.z * approachAheadTime;
        if (pick(part, planTime, exeTime))
            return false;
        if (planTime - tempExeTime + exeTime < approachAheadTime)
            reachable = true;
        // reach that pose at estimated time
        estimatedPose.header.stamp = ros::Time::now() + ros::Duration(approachAheadTime);
    }
    return true;
}