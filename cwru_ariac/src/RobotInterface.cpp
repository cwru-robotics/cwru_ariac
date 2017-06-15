//
// Created by shipei on 10/18/16.
//

#include "RobotInterface.h"

RobotInterface::RobotInterface(ros::NodeHandle &nodeHandle): nh( nodeHandle ){
    joint_state_subscriber = nh.subscribe(
            "/ariac/joint_states", 10,
            &RobotInterface::jointStateCallback, this);
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    gripper = nh.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    gripperStateSubscriber = nh.subscribe("/ariac/gripper/state", 10, &RobotInterface::gripperStateCallback, this);
    called = false;
    attached = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    if (!gripper.exists()) {
        gripper.waitForExistence();
    }
    attach.request.enable = 1;
    detach.request.enable = 0;
    arrivalTime = 0.5;
}

void RobotInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStateMsg)
{
    current_joint_states = *jointStateMsg;
    current_joint_states.name.pop_back();
    current_joint_states.effort.pop_back();
    current_joint_states.position.pop_back();
    current_joint_states.velocity.pop_back();
    called = true;
}

void RobotInterface::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) {
    currentGripperState = *msg;
    attached = msg->attached;
}

osrf_gear::VacuumGripperState RobotInterface::getGripperState() {
    ros::spinOnce();
    return currentGripperState;
}

bool RobotInterface::isGripperAttached() {
    ros::spinOnce();
    return attached;
}

bool RobotInterface::waitForGripperAttach(double timeout) {
    timeout = timeout <= 0? FLT_MAX:timeout;
    ros::spinOnce();
    while((!attached) && timeout > 0 && ros::ok()) {
        ROS_INFO("Retry grasp");
        release();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 1.0;
    }
    return attached;
}

void RobotInterface::sendJointsValue(vector<double> joints) {
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names = current_joint_states.name;
    msg.points.resize(1);
    msg.points[0].positions = joints;
    msg.points[0].time_from_start = ros::Duration(arrivalTime);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
    ros::Duration(arrivalTime).sleep();                         // wait for finish
    ros::spinOnce();
}

vector<double> RobotInterface::getJointsState() {
    called = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    vector<double> joints = current_joint_states.position;
    return joints;
}

vector<string> RobotInterface::getJointsNames() {
    if (!called) {
        while(!called && ros::ok()) {
            //ROS_INFO("Waiting for joint feedback...");
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
    }
    vector<string> names = current_joint_states.name;
    return names;
}


void RobotInterface::grab() {
    //ROS_INFO("enable gripper");
    gripper.call(attach);
}

void RobotInterface::release() {
    //ROS_INFO("release gripper");
    gripper.call(detach);
}
