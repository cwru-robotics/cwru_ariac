//
// Created by tianshipei on 4/3/17.
//

#ifndef CWRU_ARIAC_SENSORMANAGER_H
#define CWRU_ARIAC_SENSORMANAGER_H

#include <AriacBase.h>
#include <CameraEstimator.h>

class SensorManager: public AriacBase {
public:
    SensorManager(ros::NodeHandle nodeHandle);

    PartSet inView;
    PartList onGround;
    PartList onConveyor;
    vector<PartList> onAGV;
    vector<PartList> onBin;

    void addCamera(string topic);

    void addLaserScanner(string topic);

    void ForceUpdate();

    PartList combineLocations(int locationCode);

    PartList combineLocations(int locationCode, PartList extras);

    // location code
    const int GROUND = 0x01;
    const int CONVEYOR = 0x02;
    const int BINS = 0x04;
    const int AGV1 = 0x08;
    const int AGV2 = 0x10;
    const int AGVS = 0x18;

private:
    ros::NodeHandle nh_;
    vector<unique_ptr<CameraEstimator>> cameras;
    vector<int> updateCounts;
    int globalID;
    ros::AsyncSpinner spinner;
    ros::Timer updateTimer;

    void updateCallback(const ros::TimerEvent &event);
};


#endif //CWRU_ARIAC_SENSORMANAGER_H
