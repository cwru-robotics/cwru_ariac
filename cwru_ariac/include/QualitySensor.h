//
// Created by rockwell on 5/1/17.
//

#ifndef CWRU_ARIAC_QUALITYSENSOR_H
#define CWRU_ARIAC_QUALITYSENSOR_H

#include <AriacBase.h>

class QualitySensor {
public:
    QualitySensor(ros::NodeHandle &nodeHandle);

    vector<PartList> AGVbadParts;

    void forceUpdate();
protected:
    ros::NodeHandle nh;
    ros::Subscriber sensor1Subscriber;
    ros::Subscriber sensor2Subscriber;
    IDGenerator idGenerator;
    tf::TransformListener tf_listener;
    int updateCount;
    int checkedCount;
    string worldFrame;
    string sensor1Frame;
    string sensor2Frame;
    void sensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & sensor_msg);
    void sensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & sensor_msg);
};


#endif //CWRU_ARIAC_QUALITYSENSOR_H
