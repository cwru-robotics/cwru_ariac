//
// Created by shipei on 10/25/16.
//

#include <AriacBase.h>

ros::Publisher joint_trajectory_publisher;
sensor_msgs::JointState current_joint_states;
bool called = false;

/// Create a JointTrajectory with all positions set to zero, and command the arm.
void sendArmCommand(vector<double> joints) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    msg.joint_names = current_joint_states.name;
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.2);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
}


void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states = *joint_state_msg;
    called = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_tunner");

    ros::NodeHandle nh;                                 // standard ros node handle
    ros::Subscriber joint_state_subscriber = nh.subscribe(
            "/ariac/joint_states", 10,
            &joint_state_callback);
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    int joint;
    double angle;
    vector<double> my_pose;
    while(!called && ros::ok()) {
        ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }

    while (ros::ok()) {
        my_pose.resize(current_joint_states.name.size(), 0.0);
        my_pose = current_joint_states.position;
        ROS_WARN("Current joints: {\n    [0]%s:\t\t\t%f, \n    [1]%s:\t%f, \n    [2]%s:\t\t%f, \n    [3]%s:\t\t%f, \n"
                         "    [4]%s:\t\t\t%f, \n    [5]%s:\t\t\t%f, \n    [6]%s:\t\t\t%f}",
                 current_joint_states.name[0].c_str(), my_pose[0], current_joint_states.name[1].c_str(), my_pose[1],
                 current_joint_states.name[2].c_str(), my_pose[2], current_joint_states.name[3].c_str(), my_pose[3],
                 current_joint_states.name[4].c_str(), my_pose[4], current_joint_states.name[5].c_str(), my_pose[5],
                 current_joint_states.name[6].c_str(), my_pose[6]);
        std::cout << "Select joint number:";
        std::cin >> joint;
        std::cout << "Increase angles (can be negative): ";
        std::cin >> angle;
        my_pose[joint] += angle;
        sendArmCommand(my_pose);
        ros::Duration(0.2).sleep();                         // wait for finish
        ros::spinOnce();
        called = false;
        while(!called) {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
    }
    return 0;
}