// created by wsn on 3/15/17, based on:
// Created by shipei on 10/19/16.
// set up a second logical camera viewing agv1
// topic is hard-coded as logical_camera_2

#ifndef CWRU_ARIAC_AGV1CAMERAESTIMATOR_H
#define CWRU_ARIAC_AGV1CAMERAESTIMATOR_H

#include <AriacBase.h>


class Agv1CameraEstimator: public AriacBase {
public:
    double distanceTolerance;
    double untraceableTolerance;
    PartSet inView;
    PartList onGround;
    PartList onConveyor;
    vector<PartList> onAGV;
    vector<PartList> onBin;
    string worldFrame;
    string cameraFrame;
    Agv1CameraEstimator(ros::NodeHandle nodeHandle, string topic = "/ariac/logical_camera_2");
    //CameraEstimator *agv_camera_ptr;  //camera above AGV1 tray
    void waitForUpdate();
    int getMaxID() { return assignedID; }

private:
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
};


#endif //CWRU_ARIAC_AGV1CAMERAESTIMATOR_H
