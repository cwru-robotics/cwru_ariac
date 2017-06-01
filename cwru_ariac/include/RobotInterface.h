//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ROBOTINTERFACE_H
#define CWRU_ARIAC_ROBOTINTERFACE_H

#include <AriacBase.h>

class RobotInterface {
public:
    RobotInterface(ros::NodeHandle& nodeHandle);

    void sendJointsValue(vector<double> joints);
    vector<double> getJointsState();
    vector<string> getJointsNames();
    void grab();
    void release();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);

protected:
    ros::NodeHandle nh;

    ros::Publisher joint_trajectory_publisher;
    ros::Subscriber joint_state_subscriber;
    ros::ServiceClient gripper;
    ros::Subscriber gripperStateSubscriber;

    sensor_msgs::JointState current_joint_states;
    osrf_gear::VacuumGripperState currentGripperState;
    bool called;
    bool attached;
    osrf_gear::VacuumGripperControl attach;
    osrf_gear::VacuumGripperControl detach;
    double arrivalTime;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);
};


#endif //CWRU_ARIAC_ROBOTINTERFACE_H
