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

    CameraEstimator(ros::NodeHandle nodeHandle, string topic);
    void ForceUpdate();
    int getUpdateCount() { return updateCount; }
    void setUpdateCount(int updateCount) { this->updateCount = updateCount; }

protected:
    ros::NodeHandle nh;
    ros::Subscriber cameraSubscriber;
    IDGenerator idGenerator;
    void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void splitLocation();
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    ros::Time lastTime;
    int updateCount;
    int checkedCount;
    string worldFrame;
    string cameraFrame;
};


#endif //CWRU_ARIAC_CAMERAESTIMATOR_H
