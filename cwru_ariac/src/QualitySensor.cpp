//
// Created by rockwell on 5/1/17.
//

#include "QualitySensor.h"

QualitySensor::QualitySensor(ros::NodeHandle &nodeHandle) : nh(nodeHandle), AGVbadParts(totalAGVs) {
    sensor1Subscriber = nh.subscribe("/ariac/quality_control_sensor_1", 10, &QualitySensor::sensor1Callback, this);
    sensor2Subscriber = nh.subscribe("/ariac/quality_control_sensor_2", 10, &QualitySensor::sensor2Callback, this);
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "world";
    sensor1Frame = "quality_control_sensor_1_frame";
    sensor2Frame = "quality_control_sensor_2_frame";
}

void QualitySensor::sensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &sensor_msg) {
    PartList newList;
    for (auto model: sensor_msg->models) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = sensor1Frame;
        inPose.pose = model.pose;
        bool tferr = true;
        while (tferr && ros::ok()) {
            try {
                inPose.header.stamp = ros::Time(0);
                tf_listener.transformPose(worldFrame, inPose, outPose);
                tferr = false;
            } catch (tf::TransformException &exception) {
//                ROS_WARN("%s", exception.what());
                ros::Duration(0.02).sleep();
                ros::spinOnce();
            }
        }
        Part newPart;
        newPart.name = model.type;
        newPart.location = Part::AGV1;
        newPart.traceable = false;
        newPart.pose = outPose;
        bool exist = false;
        for (auto currentPart: AGVbadParts[0]) {
            if (isSamePart(currentPart, newPart)) {
                newList.push_back(currentPart);
                exist = true;
            }
        }
        if (!exist) {
            newPart.id = idGenerator.genFakeId();
            newList.push_back(newPart);
        }
    }
    AGVbadParts[0].swap(newList);
    updateCount++;
}

void QualitySensor::sensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &sensor_msg) {
    PartList newList;
    for (auto model: sensor_msg->models) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = sensor2Frame;
        inPose.pose = model.pose;
        bool tferr = true;
        while (tferr && ros::ok()) {
            try {
                inPose.header.stamp = ros::Time(0);
                tf_listener.transformPose(worldFrame, inPose, outPose);
                tferr = false;
            } catch (tf::TransformException &exception) {
//                ROS_WARN("%s", exception.what());
                ros::Duration(0.02).sleep();
                ros::spinOnce();
            }
        }
        Part newPart;
        newPart.name = model.type;
        newPart.location = Part::AGV2;
        newPart.traceable = false;
        newPart.pose = outPose;
        bool exist = false;
        for (auto currentPart: AGVbadParts[1]) {
            if (isSamePart(currentPart, newPart)) {
                newList.push_back(currentPart);
                exist = true;
            }
        }
        if (!exist) {
            newPart.id = idGenerator.genFakeId();
            newList.push_back(newPart);
        }
    }
    AGVbadParts[1].swap(newList);
    updateCount++;
}

void QualitySensor::forceUpdate() {
    while (updateCount == checkedCount && ros::ok()){
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    checkedCount = updateCount;
}