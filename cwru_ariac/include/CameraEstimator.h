//
// Created by shipei on 10/19/16.
//

#ifndef CWRU_ARIAC_CAMERAESTIMATOR_H
#define CWRU_ARIAC_CAMERAESTIMATOR_H

#include <AriacBase.h>


class CameraEstimator: public AriacBase {
public:
    double distanceTolerance;
    double untraceableTolerance;
    PartSet inView;
    PartList onGround;
    PartList onConveyor;
    vector<PartList> onAGV;
    vector<PartList> onBin;
    CameraEstimator(ros::NodeHandle nodeHandle, string topic = "/ariac/logical_camera_1");

    void ForceUpdate();

    int getAssignedID() { return assignedID; }

    void setAssignedID(int assignedID) { this->assignedID = assignedID; }

protected:
    ros::NodeHandle nh_;
    ros::Subscriber cameraSubscriber;
    void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void splitLocation();
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    ros::Time lastTime;
    int assignedID;
    int updateCount;
    int checkedCount;
    string worldFrame;
    string cameraFrame;

    friend class SensorManager;
};


#endif //CWRU_ARIAC_CAMERAESTIMATOR_H
