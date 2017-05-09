//
// Created by rockwell on 5/1/17.
//

#include "QualitySensor.h"

QualitySensor::QualitySensor(ros::NodeHandle &nodeHandle) : nh(nodeHandle) {
    sensor1Subscriber = nh.subscribe("/ariac/quality_control_sensor_1", 10, &QualitySensor::sensor1Callback, this);
    sensor2Subscriber = nh.subscribe("/ariac/quality_control_sensor_2", 10, &QualitySensor::sensor2Callback, this);
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "world";
    sensor1Frame = "quality_control_sensor_1_frame";
    sensor2Frame = "quality_control_sensor_2_frame";
}

void QualitySensor::sensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &sensor_msg) {

}

void QualitySensor::sensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &sensor_msg) {

}

void QualitySensor::ForceUpdate() {
    while (updateCount == checkedCount && ros::ok()){
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    checkedCount = updateCount;
}