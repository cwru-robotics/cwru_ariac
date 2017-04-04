//
// Created by tianshipei on 4/3/17.
//

#include "SensorManager.h"

SensorManager::SensorManager(ros::NodeHandle nodeHandle) : nh_(nodeHandle), spinner(4) {
    spinner.start();
    updateTimer = nh_.createTimer(ros::Duration(0.02), &SensorManager::updateCallback, this);
}

void SensorManager::addCamera(string topic) {
    CameraEstimator newCamera(nh_);
//    std::shared_ptr ptr = &newCamera;
//    cameras.push_back(newCamera);
    updateCounts.resize(cameras.size());
}

void SensorManager::addLaserScanner(string topic) {
    ROS_WARN("No laser scanner support!");
}

void SensorManager::updateCallback(const ros::TimerEvent &event) {
    // implement your code here, it will be executed every 0.02s
    bool cleared = false;
    bool performUpdate = false;
    for (int i = 0; i < cameras.size(); ++i) {
        if (updateCounts[i] != cameras[i].updateCount | performUpdate) {
            performUpdate = true;
            updateCounts[i] = cameras[i].updateCount;
            if (!cleared) {
                onConveyor.clear();
                for (int j = 0; j < onAGV.size(); ++j) {
                    onAGV[j].clear();
                }
                for (int k = 0; k < onBin.size(); ++k) {
                    onBin[k].clear();
                }
                onGround.clear();
                cleared = true;
            }
            for (auto p : cameras[i].onGround) {
                onGround.push_back(p);
            }
            for (auto p : cameras[i].onConveyor) {
                onGround.push_back(p);
            }
            for (int l = 0; l < onBin.size(); ++l) {
                for (int m = 0; m < cameras[i].onBin[l].size(); ++m) {
                    onBin[l].push_back(cameras[i].onBin[l][m]);
                }
            }
            for (int n = 0; n < onAGV.size(); ++n) {
                for (int j = 0; j < cameras[i].onAGV[n].size(); ++j) {
                    onAGV[n].push_back(cameras[i].onAGV[n][j]);
                }
            }
        }
    }
    // TODO: merge redundant parts
}

void SensorManager::ForceUpdate() {
    for (int i = 0; i < cameras.size(); ++i) {
        cameras[i].ForceUpdate();
    }
}

PartList SensorManager::combineLocations(int locationCode) {
    PartList parts;
    if (locationCode & GROUND) {
        for (auto p : onGround) {
            parts.push_back(p);
        }
    }
    if (locationCode & CONVEYOR) {
        for (auto p : onConveyor) {
            parts.push_back(p);
        }
    }
    if (locationCode & BINS) {
        for (auto bin : onBin) {
            for (auto p : bin) {
                parts.push_back(p);
            }
        }
    }
    if (locationCode & AGV1) {
        for (auto p : onAGV[0]) {
            parts.push_back(p);
        }
    }
    if (locationCode & AGV2) {
        for (auto p : onAGV[1]) {
            parts.push_back(p);
        }
    }
    return parts;
}

PartList SensorManager::combineLocations(int locationCode, PartList extras) {
    PartList parts = combineLocations(locationCode);
    for (auto p : extras) {
        parts.push_back(p);
    }
    return parts;
}