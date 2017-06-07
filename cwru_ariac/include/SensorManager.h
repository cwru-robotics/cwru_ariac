//
// Created by tianshipei on 4/3/17.
//

#ifndef CWRU_ARIAC_SENSORMANAGER_H
#define CWRU_ARIAC_SENSORMANAGER_H

#include <AriacBase.h>
#include <CameraEstimator.h>
#include <LaserScanner.h>

class SensorManager : public AriacBase {
public:
    SensorManager(ros::NodeHandle &nodeHandle);

    PartSet inView;
    PartList onGround;
    PartList onConveyor;
    PartList laserScannerConveyor;
    vector<PartList> onAGV;
    vector<PartList> onBin;

    void addCamera(string topic);
    void forceUpdate();
    PartList combineLocations(int locationCode);
    PartList combineLocations(int locationCode, PartList extras);
    void startUpdate();
    void stopUpdate();

    void partFetched(int partID) {
        laserScanner.remove_part(laserScanner.conveyor_partlist, partID);
    }

    // location code
    enum {
        GROUND = 0x01
    };
    enum {
        CONVEYOR = 0x02
    };
    enum {
        BINS = 0x04
    };
    enum {
        AGV1 = 0x08
    };
    enum {
        AGV2 = 0x10
    };
    enum {
        AGVS = 0x18
    };

protected:
    ros::NodeHandle nh;
    vector<unique_ptr<CameraEstimator>> cameras;
    vector<int> updateCounts;
    int laserScannerUpdateCount;
    ros::AsyncSpinner spinner;
    ros::Timer updateTimer;
    LaserScanner laserScanner;
    bool inUpdate;
    bool updateLock;
    void updateCallback(const ros::TimerEvent &event);
};


#endif //CWRU_ARIAC_SENSORMANAGER_H
