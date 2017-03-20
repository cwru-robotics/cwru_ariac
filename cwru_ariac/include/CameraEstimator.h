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
    string worldFrame;
    string cameraFrame;
    CameraEstimator(ros::NodeHandle nodeHandle, string topic = "/ariac/logical_camera_1");
    //CameraEstimator *agv_camera_ptr;  //camera above AGV1 tray
    void waitForUpdate();
    int getMaxID() { return assignedID; }
//    bool find_dropped_part(vector <osrf_gear::KitObject> partialKit, PartList onAGV1,Part &droppedPart);
//    bool match_poses_on_tray(geometry_msgs::Pose camera_part_pose,geometry_msgs::Pose kit_part_pose);
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


#endif //CWRU_ARIAC_CAMERAESTIMATOR_H
