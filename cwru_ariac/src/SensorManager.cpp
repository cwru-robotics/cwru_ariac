//
// Created by tianshipei on 4/3/17.
//

#include "SensorManager.h"

SensorManager::SensorManager(ros::NodeHandle &nodeHandle) : nh(nodeHandle),
                                                            spinner(std::thread::hardware_concurrency()),
                                                            onAGV(totalAGVs),
                                                            onBin(totalBins) {
    updateTimer = nh.createTimer(ros::Duration(0.02), &SensorManager::updateCallback, this, false, false);
    inUpdate = false;
}

void SensorManager::addCamera(string topic) {
    stopUpdate();
    while (inUpdate);
    cameras.emplace_back(new CameraEstimator(nh, topic));
    updateCounts.resize(cameras.size());
    startUpdate();
}

void SensorManager::addLaserScanner(string topic) {
    ROS_WARN("No laser scanner support!");
}

void SensorManager::updateCallback(const ros::TimerEvent &event) {
    // implement your code here, it will be executed at 50Hz
    bool cleared = false;
    bool performUpdate = false;
    int errorCode = 0;
    inUpdate = true;
    for (int i = 0; i < cameras.size(); ++i) {
        cameras[i]->lock();
        try {
            if (updateCounts[i] != cameras[i]->getUpdateCount() | performUpdate) {
                performUpdate = true;
                updateCounts[i] = cameras[i]->getUpdateCount();
                errorCode = 1;
                if (!cleared) {
                    errorCode = 2;
                    onGround.clear();
                    cleared = true;
                    errorCode = 3;
                    onConveyor.clear();
                    errorCode = 40;
                    for (int j = 0; j < onAGV.size(); ++j) {
                        errorCode++;
                        onAGV[j].clear();
                    }
                    errorCode = 50;
                    for (int k = 0; k < onBin.size(); ++k) {
                        errorCode++;
                        onBin[k].clear();
                    }
                }
                errorCode = 60;
                for (auto p : cameras[i]->onGround) {
                    errorCode++;
                    onGround.push_back(p);
                }
                errorCode = 70;
                for (auto p : cameras[i]->onConveyor) {
                    errorCode++;
                    onConveyor.push_back(p);
                }
                errorCode = 800;
                for (int l = 0; l < onBin.size(); ++l) {
                    errorCode += 10;
                    for (int m = 0; m < cameras[i]->onBin[l].size(); ++m) {
                        errorCode++;
                        onBin[l].push_back(cameras[i]->onBin[l][m]);
                    }
                }
                errorCode = 900;
                for (int n = 0; n < onAGV.size(); ++n) {
                    errorCode += 10;
                    for (int j = 0; j < cameras[i]->onAGV[n].size(); ++j) {
                        errorCode++;
                        onAGV[n].push_back(cameras[i]->onAGV[n][j]);
                    }
                }
            }
        } catch (const std::bad_alloc e) {
            ROS_ERROR("Error: %s, when update camera %d, at step %d", e.what(), i + 1, errorCode);
            ROS_ERROR(
                    "Please report this error and check your memory page in your system monitor, hit enter to continue...");
            getchar();
        }
        cameras[i]->unlock();
    }
    // TODO: merge redundant parts
    inUpdate = false;
}

void SensorManager::startUpdate() {
    while (!spinner.canStart());
    spinner.start();
    updateTimer.start();
}

void SensorManager::stopUpdate() {
    spinner.stop();
    updateTimer.stop();
}

void SensorManager::forceUpdate() {
    for (int i = 0; i < cameras.size(); ++i) {
        cameras[i]->forceUpdate();
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