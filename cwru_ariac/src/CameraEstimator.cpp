//
// Created by shipei on 10/19/16.
//

#include <cwru_ariac/Part.h>
#include "CameraEstimator.h"

CameraEstimator::CameraEstimator(ros::NodeHandle nodeHandle, string cameraTopic) :
        nh_(nodeHandle), onAGV(totalAGVs), onBin(totalBins) {
    cameraSubscriber = nh_.subscribe(cameraTopic, 1,
                                     &CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.02;
    untraceableTolerance = 0.1;
    assignedID = 1;
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "/world";
    cameraFrame = cameraTopic.substr(cameraTopic.find_last_of("/")) + "_frame";
    //tf_listener.waitForTransform(cameraFrame, worldFrame, ros::Time(0), ros::Duration(2.0));
}

void CameraEstimator::cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
    PartSet updateView;
    ros::Time currentTime = ros::Time::now();
    double dt = currentTime.toSec() - lastTime.toSec();
    // update all exist objects
    for (auto part: inView) {
        part.pose.pose.position.x += part.linear.x * dt;
        part.pose.pose.position.y += part.linear.y * dt;
        part.pose.pose.position.z += part.linear.z * dt;
        part.pose.header.stamp = ros::Time::now();
    }
    lastTime.fromSec(currentTime.toSec());
    updateCount++;
    // TODO refactor to: match all exist object first, then all untraceable object then add new object
//    ROS_INFO("inView: %d, size: %d, dt = %f",(int)inView.size(), (int)image_msg->models.size(), (float)dt);
    for (int i = 0; i < image_msg->models.size(); ++i) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = cameraFrame;
        bool flag = false;
        bool tferr = true;
        double distance;
        Part nextPart;
        nextPart.name = image_msg->models[i].type;
        inPose.pose = image_msg->models[i].pose;
        while (tferr && ros::ok()) {
            tferr = false;
            try {
                inPose.header.stamp = ros::Time::now();
                tf_listener.transformPose(worldFrame, inPose, outPose);
            } catch (tf::TransformException &exception) {
                return;
//                ROS_ERROR("%s", exception.what());
//                tferr = true;
//                ros::Duration(0.05).sleep();
//                ros::spinOnce();
            }
        }
        nextPart.pose = outPose;

        // find object by estimated distance;
        for (auto part:inView) {
            distance = euclideanDistance(nextPart.pose.pose.position, part.pose.pose.position);
            if ((distance < distanceTolerance) && (nextPart.name == part.name)) {
                nextPart.linear.x = (nextPart.linear.x + (nextPart.pose.pose.position.x - part.pose.pose.position.x)/dt)/2;
                nextPart.linear.y = (nextPart.linear.y + (nextPart.pose.pose.position.y - part.pose.pose.position.y)/dt)/2;
                nextPart.linear.z = (nextPart.linear.z + (nextPart.pose.pose.position.z - part.pose.pose.position.z)/dt)/2;
                nextPart.traceable = true;
                nextPart.id = part.id;
                inView.erase(part);
                flag = true;
                break;
            }
        }
        // try to find untraceable object by nearest distance
        if (!flag) {
            for (auto part:inView) {
                distance = euclideanDistance(nextPart.pose.pose.position, part.pose.pose.position);
                if ((!part.traceable) && (distance < untraceableTolerance) && (nextPart.name == part.name)) {
                    if ((distance < untraceableTolerance) && (nextPart.name == part.name)) {
                        nextPart.linear.x = (nextPart.pose.pose.position.x - part.pose.pose.position.x)/dt;
                        nextPart.linear.y = (nextPart.pose.pose.position.y - part.pose.pose.position.y)/dt;
                        nextPart.linear.z = (nextPart.pose.pose.position.z - part.pose.pose.position.z)/dt;
                        nextPart.traceable = true;
                        nextPart.id = part.id;
                        inView.erase(part);
                        flag = true;
                        break;
                    }
                }
            }
        }
        // make new object untraceable
        if (!flag) {
//            ROS_INFO("add part %d : %s", assignedID, nextPart.name.c_str());
            nextPart.linear.x = 0;
            nextPart.linear.y = 0;
            nextPart.linear.z = 0;
            nextPart.traceable = false;
            nextPart.id = assignedID++;
        }
        updateView.insert(nextPart);
    }
//    for (auto part: inView) {
//        ROS_INFO("part %d : %s disappeared", part.id, part.name.c_str());
//    }
    inView.swap(updateView);
    splitLocation();
}
void CameraEstimator::splitLocation() {
    onConveyor.clear();
    for (int j = 0; j < onAGV.size(); ++j) {
        onAGV[j].clear();
    }
    for (int k = 0; k < onBin.size(); ++k) {
        onBin[k].clear();
    }
    onGround.clear();
    for (auto part: inView) {
        if (checkBound(part.pose.pose.position, conveyorBoundBox)) {
            part.location = Part::CONVEYOR;
//            if (!part.traceable){
//                part.traceable = true;
//                part.linear.x = 0;
//                part.linear.y = -0.05;
//                part.linear.z = 0;
//            }
            onConveyor.push_back(part);
            continue;
        }
        bool jump = false;
        for (int j = 0; j < onAGV.size(); ++j) {
            if (checkBound(part.pose.pose.position, agvBoundBox[j])) {
                part.location = Part::AGV1 + j;
                onAGV[j].push_back(part);
                jump = true;
                break;
            }
        }
        if (jump)
            continue;
        for (int j = 0; j < onBin.size(); ++j) {
            // TODO
            if (checkBound(part.pose.pose.position, binBoundBox[j])) {
                part.location = Part::BIN1 + j;
                onBin[j].push_back(part);
                jump = true;
                break;
            }
        }
        if (jump)
            continue;
        onGround.push_back(part);
    }
}

void CameraEstimator::waitForUpdate() {
    while (updateCount == checkedCount && ros::ok()){
        //ROS_INFO("Waiting for camera callback...");
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    checkedCount = updateCount;
}