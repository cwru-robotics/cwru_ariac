// wsn, 10/15/17 based on CameraEstimator.cpp
// Created by shipei on 10/19/16.
//

#include <cwru_ariac/Part.h>
#include "Agv1CameraEstimator.h"

string kit_tray_string("kit_tray");

Agv1CameraEstimator::Agv1CameraEstimator(ros::NodeHandle nodeHandle, string topic) :
        nh_(nodeHandle), onAGV(totalAGVs), onBin(totalBins) {
    cameraSubscriber = nh_.subscribe(topic, 1,
                                     &Agv1CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.02;
    untraceableTolerance = 0.1;
    assignedID = 1;
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "/world";
    cameraFrame = topic.substr(topic.find_last_of("/")) + "_frame";
    //tf_listener.waitForTransform(cameraFrame, worldFrame, ros::Time(0), ros::Duration(2.0));
}

void Agv1CameraEstimator::cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
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

    auto models = image_msg->models;
//    vector<double> distances;
//    for (int j = 0; j < models.size(); ++j) {
//        if (defaultParts.find(models[j].type) == defaultParts.end()) {
//            models.erase(models.begin() + j);
//            continue;
//        }
//        for (int i = j + 1; i < models.size(); ++i) {
//            double distance = euclideanDistance(models[j].pose.position, models[i].pose.position);
//            distances.push_back(distance);
//            if (distance < 0.02) {
//                models.erase(models.begin() + i);
//            }
//        }
//    }
//    sort(distances.begin(), distances.end());
    // TODO refactor to: match all exist object first, then all untraceable object then add new object
//    ROS_INFO("inView: %d, size: %d, dt = %f",(int)inView.size(), (int)image_msg->models.size(), (float)dt);
    if(updateCount == checkedCount) {
       ROS_INFO("AGV1 camera sees %d objects",(int) models.size());
    }
    for (int i = 0; i < models.size(); ++i) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = cameraFrame;
        bool flag = false;
        bool tferr = true;
        double distance;
        Part nextPart;

        nextPart.name = models[i].type;
        string part_string(nextPart.name);
        if (kit_tray_string.compare(part_string)==0)
          continue; // skip this part if it is the kit tray
        inPose.pose = models[i].pose;
        while (tferr && ros::ok()) {
            tferr = false;
            try {
                inPose.header.stamp = ros::Time(0);
                tf_listener.transformPose(worldFrame, inPose, outPose);
            } catch (tf::TransformException &exception) {
//                return;
                ROS_WARN("%s", exception.what());
                tferr = true;
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
        }
        nextPart.pose = outPose;
        if(updateCount == checkedCount) {
          ROS_INFO("evaluating part:");
          ROS_INFO_STREAM(nextPart);
         }
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
    updateCount++;
}

//only test for parts within view on conveyor or parts on AGV1:
void Agv1CameraEstimator::splitLocation() {
    onConveyor.clear(); //consider parts in view on conveyor
    onAGV[0].clear(); //only consider AGV1, not AGV2


    for (auto part: inView) {
        if (checkBound(part.pose.pose.position, conveyorBoundBox)) {
            part.location = Part::CONVEYOR;
            onConveyor.push_back(part); //assign this part to conveyor
            continue; //done w/ this part; examine next part
        }
        //part was not on conveyor, so test if it is on agv1
        if(updateCount == checkedCount) {
          ROS_INFO("checking if part on agv:");
          ROS_INFO_STREAM(part.pose.pose.position);
         }
        if (checkBound(part.pose.pose.position, agvBoundBox[0])) {
         if(updateCount == checkedCount) {
          ROS_INFO("part is on AGV1:");
         }                
                part.location = Part::AGV1;
                onAGV[0].push_back(part); //assign this part to AGV1
        }
        else {
         if(updateCount == checkedCount) {
          ROS_INFO("part is not on AGV1:");
         }  
        }
    }
}

void Agv1CameraEstimator::waitForUpdate() {
    checkedCount = updateCount;
    while (updateCount == checkedCount && ros::ok()){
        //ROS_INFO("Waiting for camera callback...");
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
}
