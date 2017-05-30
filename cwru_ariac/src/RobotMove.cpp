//
// Created by tianshipei on 12/4/16.
//edited wsn 2/18/17
//

#include "RobotMove.h"

RobotMove::RobotMove(ros::NodeHandle &nodeHandle, string topic): nh(nodeHandle), ac(topic, true) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    time_tolerance = 1.0;
    async_mode = false;
    ROS_INFO("Robot Move Action Client is ready.");
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::NO_ERROR, "NO_ERROR"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::CANCELLED, "CANCELLED"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::WRONG_PARAMETER, "WRONG_PARAMETER"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::TIMEOUT, "TIMEOUT"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::UNREACHABLE, "UNREACHABLE"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::GRIPPER_FAULT, "GRIPPER_FAULT"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::COLLISION, "COLLISION"));
    errorCodeFinder.insert(pair<int8_t, string>(RobotMoveResult::PART_DROPPED, "PART_DROPPED"));
}
bool RobotMove::toHome(double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::TO_HOME;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::toPredefinedPose(int8_t predefined_pose_code, double timeout) {
    ROS_INFO("requesting move to pose code %d", predefined_pose_code);
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::TO_PREDEFINED_POSE;
    goal.predfinedPoseCode = predefined_pose_code;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::fetchPartFromConveyor(Part part, Part destination, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::CONVEYOR_FETCH;
    goal.timeout = timeout;
    goal.sourcePart = part;
    goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::flipPart(Part part, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::FLIP_PART;
    goal.timeout = 0; //timeout;
    goal.sourcePart = part;
    //goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::pick(Part part, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::PICK;
    goal.timeout = timeout;
    goal.sourcePart = part;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::place(Part destination, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::PLACE;
    goal.timeout = timeout;
    goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::move(Part part, Part destination, double timeout) {
    RobotMoveGoal goal;
    goal_success_ = false;
    goal.type = RobotMoveGoal::MOVE;
    goal.timeout = timeout;
    goal.sourcePart = part;
    goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        ros::spinOnce();
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::setJointValues(vector<double> joints, double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::PLACE;
    goal.timeout = timeout;
    goal.jointsValue = joints;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::grasp(double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::GRASP;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::release(double timeout) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::RELEASE;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::isGripperAttached() {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::IS_ATTACHED;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout = ac.waitForResult();
        if (!finished_before_timeout)
            errorCode = RobotMoveResult::TIMEOUT;
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::getRobotState(RobotState &robotState) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::GET_ROBOT_STATE;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout = ac.waitForResult();
        if (!finished_before_timeout)
            errorCode = RobotMoveResult::TIMEOUT;
        return finished_before_timeout && goal_success_;
    }
    return true;
}
vector<double> RobotMove::getJointsState() {
    RobotState robotState;
    bool goal_success_ = getRobotState(robotState);
    return robotState.jointStates;
}

void RobotMove::sendGoal(RobotMoveGoal goal) {
    action_server_returned_ = false;
    ac.sendGoal(goal, boost::bind(&RobotMove::doneCb, this, _1, _2), boost::bind(&RobotMove::activeCb, this), boost::bind(&RobotMove::feedbackCb, this, _1));
}

void RobotMove::cancel() {
    ac.cancelGoal();
}

void RobotMove::activeCb() {
    // ROS_INFO("Goal sent");
}

void RobotMove::showJointState(vector<string> joint_names, vector<double> joint_values) {
    ROS_INFO("Current joints state: {");
    for (int i = 0; i < joint_names.size(); i++) {
        cout << "    [" << i + 1 << "] " << joint_names[i] << ":   " <<	joint_values[i] << ",\n";
    }
    cout << "}" << endl;
}

void RobotMove::doneCb(const actionlib::SimpleClientGoalState &state, const RobotMoveResultConstPtr &result) {
    action_server_returned_ = true;
    goal_success_ = result->success;
    errorCode = result->errorCode;
    currentRobotState = result->robotState;
//    ROS_INFO("Action finished in state [%s]: %s with error code: %d",
//             state.toString().c_str(), goal_success_?"success":"failed", errorCode);
//    ROS_INFO("Gripper position is: %f, %f, %f\n",
//             currentRobotState.gripperPose.pose.position.x, currentRobotState.gripperPose.pose.position.y,
//             currentRobotState.gripperPose.pose.position.z);
//    showJointState(currentRobotState.jointNames, currentRobotState.jointStates);
}

void RobotMove::feedbackCb(const RobotMoveFeedbackConstPtr &feedback) {
    currentRobotState = feedback->robotState;
}
