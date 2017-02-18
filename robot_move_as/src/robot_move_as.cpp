//
// Created by shipei on 2/17/17.
//

#include <RobotMoveActionServer.h>

RobotMoveActionServer::RobotMoveActionServer(ros::NodeHandle nodeHandle, string topic):
        nh(nodeHandle), as(nh, topic, boost::bind(&RobotMoveActionServer::executeCB, this, _1), false) {
    isPreempt = false;
    as.registerPreemptCallback(boost::bind(&RobotMoveActionServer::preemptCB, this));
    as.start();
    ROS_INFO("Start Robot Move Action Server");
    placeFinder.insert(pair<int8_t, string>(Part::AGV, "AGV"));
    placeFinder.insert(pair<int8_t, string>(Part::AGV1, "AGV1"));
    placeFinder.insert(pair<int8_t, string>(Part::AGV2, "AGV2"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN, "BIN"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN1, "BIN1"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN2, "BIN2"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN3, "BIN3"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN4, "BIN4"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN5, "BIN5"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN6, "BIN6"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN7, "BIN7"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN8, "BIN8"));
    placeFinder.insert(pair<int8_t, string>(Part::CAMERA, "CAMERA"));
    placeFinder.insert(pair<int8_t, string>(Part::CONVEYOR, "CONVEYOR"));
    placeFinder.insert(pair<int8_t, string>(Part::GROUND, "GROUND"));
    placeFinder.insert(pair<int8_t, string>(Part::UNASSIGNED, "UNASSIGNED"));

    robotState.basePose.pose.position.x = 1.0;
    robotState.basePose.pose.position.y = 2.0;
    robotState.basePose.pose.position.z = 3.0;
    robotState.basePose.header.frame_id = "/world";
    robotState.basePose.header.stamp = ros::Time::now();
    robotState.gripperPose.pose.position.x = 1.5;
    robotState.gripperPose.pose.position.y = 2.5;
    robotState.gripperPose.pose.position.z = 3.5;
    robotState.gripperPose.header.frame_id = "/world";
    robotState.gripperPose.header.stamp = ros::Time::now();
    //robotState.jointNames = {"linear_arm_actuator_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    // wsn reordered these to be consistent w/ order from joint_states:
    robotState.jointNames = {"elbow_joint","linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint","wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    robotState.jointStates = {0, 1, 2, 3, 4, 5, 6};
    joint_trajectory_publisher_ = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    q_cruise_pose_.resize(7);
    q_cruise_pose_<<1.85, -0.535, -2.0, 1.57, 3.33, -1.57, 0.50; //need to set rail q[1] to current rail pose
    q_agv1_hover_pose_.resize(7);
    q_agv1_hover_pose_<<1.0, 2.1, -0.3, 1.4, 4.0, -1.57, 0.0;
    q_bin8_retract_pose_.resize(7);
    q_bin8_retract_pose_<<1.85,  0.4, -2.0, 1.57, 3.33, -1.57, 0.50;//1.85,  0.4, -2.0, 1.57, 3.33, -1.57, 0.50
    q_bin8_cruise_pose_.resize(7);
    q_bin8_cruise_pose_<<1.85,  0.4, -2.0, 1.57, 3.33, -1.57, 0.50;//1.85,  0.4, -2.0, 1.57, 3.33, -1.57, 0.50
    //Eigen::VectorXd q_bin8_cruise_pose_,q_bin8_hover_pose_,q_bin8_retract_pose_;    
    
}

double RobotMoveActionServer::rail_prepose(int8_t location) {
    switch (location) {
        case Part::AGV1:
            return 2.1;
        break;
        default:
            return 0; 
    }
}

trajectory_msgs::JointTrajectory RobotMoveActionServer::jspace_pose_to_traj(Eigen::VectorXd joints) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    int njnts = robotState.jointNames.size();
    msg.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    msg.joint_names = robotState.jointNames;
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(njnts);
    for (int i = 0; i < njnts; ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.2);
    ROS_INFO_STREAM("populated traj msg:\n" << msg);
    return msg;    
}

void RobotMoveActionServer::executeCB(const cwru_ariac::RobotMoveGoalConstPtr &goal) {
    ROS_INFO("Received goal type: %d", goal->type);
    double start_time = ros::Time::now().toSec();
    double dt;
    switch (goal->type) {
        case RobotMoveGoal::NONE:
            ROS_INFO("NONE");
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);
            break;
        case RobotMoveGoal::PICK:
            ROS_INFO("PICK");
            ROS_INFO("The part is %s, locate at %s, with pose:", goal->sourcePart.name.c_str(), placeFinder[goal->sourcePart.location].c_str());
            ROS_INFO_STREAM(goal->sourcePart.pose);
            ROS_INFO("And moving speed:");
            ROS_INFO_STREAM(goal->sourcePart.linear);
            ROS_INFO("Time limit is %f", goal->timeout);
            ROS_INFO("I am moving myself towards the part");
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I got the part");
            dt = ros::Time::now().toSec() - start_time;
            if (dt < goal->timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = RobotMoveResult::NO_ERROR;
                result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = RobotMoveResult::TIMEOUT;
                result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
        case RobotMoveGoal::PLACE:
            ROS_INFO("PLACE");
            ROS_INFO("The part is %s, should be place to %s, with pose:", goal->targetPart.name.c_str(), placeFinder[goal->targetPart.location].c_str());
            ROS_INFO_STREAM(goal->targetPart.pose);
            ROS_INFO("Time limit is %f", goal->timeout);
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I got the part");
            dt = ros::Time::now().toSec() - start_time;
            if (dt < goal->timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = RobotMoveResult::NO_ERROR;
                result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = RobotMoveResult::TIMEOUT;
                result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
        case RobotMoveGoal::MOVE:
            ROS_INFO("MOVE");
            ROS_INFO("The part is %s, should be move from %s to %s, with source pose:", goal->sourcePart.name.c_str(), placeFinder[goal->sourcePart.location].c_str(), placeFinder[goal->targetPart.location].c_str());
            ROS_INFO_STREAM(goal->sourcePart.pose);
            ROS_INFO("target pose:");
            ROS_INFO_STREAM(goal->targetPart.pose);
            ROS_INFO("Time limit is %f", goal->timeout);
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I got the part");
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I place the part to the target location");
            dt = ros::Time::now().toSec() - start_time;
            if (dt < goal->timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = RobotMoveResult::NO_ERROR;
                result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = RobotMoveResult::TIMEOUT;
                result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
        case RobotMoveGoal::TO_CRUISE_POSE:
            ROS_INFO("moving to cruise pose");
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);            
            break;
            
        case RobotMoveGoal::TO_AGV1_HOVER_POSE:
            ROS_INFO("moving to AGV1 hover pose");
            //q_agv1_hover_pose_
            traj_ = jspace_pose_to_traj(q_agv1_hover_pose_);
            joint_trajectory_publisher_.publish(traj_);
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);            
            break;         

        case RobotMoveGoal::TO_PREDEFINED_POSE:
            ROS_INFO("moving to AGV1 hover pose");
            result_.success = true;
            switch(goal->predfinedPoseCode){
                //various cases for pre-defined poses go here:
                case RobotMoveGoal::AGV1_HOVER_POSE:
                   traj_ = jspace_pose_to_traj(q_agv1_hover_pose_);                   
                   break;
                case RobotMoveGoal::BIN8_CRUISE_POSE:
                    traj_ = jspace_pose_to_traj(q_bin8_cruise_pose_); 
                    break;
                default:
                    ROS_WARN("predefined move code not implemented!");
                    result_.success = false;
                    result_.errorCode = RobotMoveResult::WRONG_PARAMETER;
                    as.setAborted(result_);
            }
            if (result_.success == true) {
              joint_trajectory_publisher_.publish(traj_);
              result_.errorCode = RobotMoveResult::NO_ERROR;
              result_.robotState = robotState;
              ros::Duration(2.0).sleep();
              as.setSucceeded(result_);  
            }
            break;   
                       
            
        case RobotMoveGoal::TO_HOME:
            ROS_INFO("TO_HOME");
            ROS_INFO("Time limit is %f", goal->timeout);
            ROS_INFO("I am releasing the gripper");
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I am moving myself to home pose");
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            if (int((((double)rand())/RAND_MAX) * 6) == 1) {
                ROS_INFO("I drop the part");
                result_.success = false;
                result_.errorCode = RobotMoveResult::PART_DROPPED;
                result_.robotState = robotState;
                as.setAborted(result_);
                return;
            }
            if (int((((double)rand())/RAND_MAX) * 6) == 2) {
                ROS_INFO("Target unreachable");
                result_.success = false;
                result_.errorCode = RobotMoveResult::UNREACHABLE;
                result_.robotState = robotState;
                as.setAborted(result_);
                return;
            }
            if (int((((double)rand())/RAND_MAX) * 6) == 3) {
                ROS_INFO("Self Collision happened");
                result_.success = false;
                result_.errorCode = RobotMoveResult::COLLISION;
                result_.robotState = robotState;
                as.setAborted(result_);
                return;
            }
            if (int((((double)rand())/RAND_MAX) * 6) == 4) {
                ROS_INFO("Some thing wrong with gripper");
                result_.success = false;
                result_.errorCode = RobotMoveResult::GRIPPER_FAULT;
                result_.robotState = robotState;
                as.setAborted(result_);
                return;
            }
            dt = ros::Time::now().toSec() - start_time;
            if (dt < goal->timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = RobotMoveResult::NO_ERROR;
                result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = RobotMoveResult::TIMEOUT;
                result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
        case RobotMoveGoal::SET_JOINT_VALUE:
            ROS_INFO("SET_JOINT_VALUE");
            ROS_INFO("New joint values are:");
            for (auto j:goal->jointsValue) {
                cout << j << " ";
            }
            ROS_INFO("Time limit is %f", goal->timeout);
            ros::Duration(0.5).sleep();
            feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            dt = ros::Time::now().toSec() - start_time;
            if (dt < goal->timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = RobotMoveResult::NO_ERROR;
                result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = RobotMoveResult::TIMEOUT;
                result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
        case RobotMoveGoal::GET_ROBOT_STATE:
            ROS_INFO("GET_ROBOT_STATE");
            ROS_INFO("Return robot state of the robot");
            ROS_INFO("I completed the action");
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);
            break;
        case RobotMoveGoal::GRASP:
            ROS_INFO("GRASP");
            ROS_INFO("grasp block");
            ROS_INFO("I completed the action");
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);
            break;
        case RobotMoveGoal::RELEASE:
            ROS_INFO("RELEASE");
            ROS_INFO("release block");
            ROS_INFO("I completed the action");
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);
            break;
        case RobotMoveGoal::IS_ATTACHED:
            ROS_INFO("IS_ATTACHED");
            ROS_INFO("get gripper attached state");
            ROS_INFO("I completed the action");
            result_.success = true;
            result_.errorCode = RobotMoveResult::NO_ERROR;
            result_.robotState = robotState;
            as.setSucceeded(result_);
            break;
        case RobotMoveGoal::WAIT_FOR_ATTACHED:
            ROS_INFO("WAIT_FOR_ATTACHED");
            ROS_INFO("waiting for attached");
            dt = ros::Time::now().toSec() - start_time;
            if (dt < goal->timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = RobotMoveResult::NO_ERROR;
                result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = RobotMoveResult::TIMEOUT;
                result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
        default:
            ROS_INFO("Wrong parameter received for goal");
            result_.success = false;
            result_.errorCode = RobotMoveResult::WRONG_PARAMETER;
            as.setAborted(result_);
    }
    isPreempt = false;
}

void RobotMoveActionServer::preemptCB() {
    isPreempt = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_as"); //name this node
    ros::NodeHandle nh;
    RobotMoveActionServer actionServer(nh, "robot_move");
    ros::spin();
    return 0;
}