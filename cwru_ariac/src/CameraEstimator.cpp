//
// Created by shipei on 10/19/16.
//

#include "CameraEstimator.h"

CameraEstimator::CameraEstimator(ros::NodeHandle &nodeHandle, string topic) :
        nh(nodeHandle), onAGV(totalAGVs), onBin(totalBins) {
    cameraSubscriber = nh.subscribe(topic, 10,
                                     &CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.02;
    untraceableTolerance = 0.1;
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "/world";
    cameraFrame = topic.substr(topic.find_last_of("/")) + "_frame";
    updateLock = false;
    //tf_listener.waitForTransform(cameraFrame, worldFrame, ros::Time(0), ros::Duration(2.0));
}

void CameraEstimator::cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
    if (updateLock) {
        return;
    }
    inUpdate = true;
    PartSet updateView;
    PartSet estimatedView;
    ros::Time currentTime = ros::Time::now();
    double dt = currentTime.toSec() - lastTime.toSec();
    // update all exist objects
    for (auto part: inView) {
        Part temp = part;
        temp.pose.pose.position.x += temp.linear.x * dt;
        temp.pose.pose.position.y += temp.linear.y * dt;
        temp.pose.pose.position.z += temp.linear.z * dt;
        estimatedView.insert(temp);
    }
    inView.swap(estimatedView);
    lastTime.fromSec(currentTime.toSec());
    updateCount++;
    auto models = image_msg->models;
    // TODO refactor to: match all exist object first, then all untraceable object then add new object
    // TODO noise filtering
    for (int i = 0; i < models.size(); ++i) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = cameraFrame;
        bool flag = false;
        bool tferr = true;
        double distance;
        Part nextPart;
        nextPart.name = models[i].type;
        inPose.pose = models[i].pose;
        while (tferr && ros::ok()) {
            tferr = false;
            try {
                inPose.header.stamp = ros::Time(0);
                tf_listener.transformPose(worldFrame, inPose, outPose);
            } catch (tf::TransformException &exception) {
//                return;
//                ROS_WARN("%s", exception.what());
                tferr = true;
                ros::Duration(0.02).sleep();
                ros::spinOnce();
            }
        }
        nextPart.pose = outPose;

        // find object by estimated distance;
        for (auto part:inView) {
            distance = euclideanDistance(nextPart.pose.pose.position, part.pose.pose.position);
            if ((distance < distanceTolerance) && (nextPart.name == part.name)) {
                nextPart.linear.x = speedFilter(nextPart.pose.pose.position.x, part.pose.pose.position.x, part.linear.x,
                                                dt);
                nextPart.linear.y = speedFilter(nextPart.pose.pose.position.y, part.pose.pose.position.y, part.linear.y,
                                                dt);
                nextPart.linear.z = speedFilter(nextPart.pose.pose.position.z, part.pose.pose.position.z, part.linear.z,
                                                dt);
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
                        nextPart.linear.x = speedFilter(nextPart.pose.pose.position.x, part.pose.pose.position.x, 0,
                                                        dt);
                        nextPart.linear.y = speedFilter(nextPart.pose.pose.position.y, part.pose.pose.position.y, 0,
                                                        dt);
                        nextPart.linear.z = speedFilter(nextPart.pose.pose.position.z, part.pose.pose.position.z, 0,
                                                        dt);
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
            nextPart.id = idGenerator.genId();
        }
        updateView.insert(nextPart);
    }
//    for (auto part: inView) {
//        ROS_INFO("part %d : %s disappeared", part.id, part.name.c_str());
//    }
    inView.swap(updateView);
    splitLocation();
    inUpdate = false;
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
//                part.linear.y = -0.2;
//                part.linear.z = 0;
//            }
            onConveyor.push_back(part);
            continue;
        }
        bool jump = false;
        for (int j = 0; j < onAGV.size(); ++j) {
            if (checkBound(part.pose.pose.position, agvBoundBox[j])) {
                if (part.name.compare("kit_tray") == 0) {
                    jump = true;
                    break;
                }
                part.location = Part::AGV1 + j;
                onAGV[j].push_back(part);
                jump = true;
                break;
            }
        }
        if (jump)
            continue;
        for (int j = 0; j < onBin.size(); ++j) {
            if (checkBound(part.pose.pose.position, binBoundBox[j])) {
                part.location = Part::BIN1 + j;
                onBin[j].push_back(part);
                jump = true;
                // ROS_INFO("add part %d to bin %d", part.id, j);
                break;
            }
        }
        if (jump)
            continue;
        part.location = Part::GROUND;
        onGround.push_back(part);
    }
}

void CameraEstimator::forceUpdate() {
    while (updateCount == checkedCount && ros::ok()){
        //ROS_INFO("Waiting for camera callback...");
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    checkedCount = updateCount;
}