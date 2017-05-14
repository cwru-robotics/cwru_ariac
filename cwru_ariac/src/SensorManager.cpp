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
    int process = 10;
    inUpdate = true;
    for (int i = 0; i < cameras.size(); ++i) {
        cameras[i]->lock();
        try {
            if (updateCounts[i] != cameras[i]->getUpdateCount() | performUpdate) {
                performUpdate = true;
                updateCounts[i] = cameras[i]->getUpdateCount();
                process = 20;
                if (!cleared) {
                    process = 30;
                    onGround.clear();
                    cleared = true;
                    process = 40;
                    onConveyor.clear();
                    process = 50;
                    for (int j = 0; j < onAGV.size(); ++j) {
                        process++;
                        onAGV[j].clear();
                    }
                    process = 60;
                    for (int k = 0; k < onBin.size(); ++k) {
                        process++;
                        onBin[k].clear();
                    }
                }
                process = 70;
                for (auto p : cameras[i]->onGround) {
                    onGround.push_back(p);
                }
                process = 80;
                for (auto p : cameras[i]->onConveyor) {
                    process++;
                    onConveyor.push_back(p);
                }
                process = 900;
                for (int l = 0; l < onBin.size(); ++l) {
                    process += 10;
                    for (int m = 0; m < cameras[i]->onBin[l].size(); ++m) {
                        process++;
                        onBin[l].push_back(cameras[i]->onBin[l][m]);
                    }
                }
                process = 1000;
                for (int n = 0; n < onAGV.size(); ++n) {
                    process += 10;
                    for (int j = 0; j < cameras[i]->onAGV[n].size(); ++j) {
                        process++;
                        onAGV[n].push_back(cameras[i]->onAGV[n][j]);
                    }
                }
            }
        } catch (const std::bad_alloc e) {
            ROS_ERROR("Error: %s, when update camera %d at process %d", e.what(), i + 1, process);
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