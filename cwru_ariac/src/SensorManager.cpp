//
// Created by tianshipei on 4/3/17.
//

#include "SensorManager.h"

SensorManager::SensorManager(ros::NodeHandle &nodeHandle) : nh(nodeHandle),
                                                            spinner(0),
                                                            onAGV(totalAGVs),
                                                            onBin(totalBins),
                                                            laserScanner(nh) {
    updateTimer = nh.createTimer(ros::Duration(0.02), &SensorManager::updateCallback, this);
    inUpdate = false;
    updateLock = false;
    spinner.start();
    laserScanner.conveyor_partlist.clear();
}

void SensorManager::addCamera(string topic) {
    stopUpdate();
    cameras.emplace_back(new CameraEstimator(nh, topic));
    updateCounts.resize(cameras.size());
    updateCounts[updateCounts.size() - 1] = 0;
    startUpdate();
}

void SensorManager::updateCallback(const ros::TimerEvent &event) {
    // implement your code here, it will be executed at 50Hz
    if (updateLock) {
        return;
    }
    inUpdate = true;
    bool cleared = false;
    bool performUpdate = false;
    int errorCode = 0;
    for (int i = 0; i < cameras.size(); ++i) {
        cameras[i]->stopUpdate();
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
        }
        cameras[i]->startUpdate();
    }
    std::lock_guard<std::mutex> guard(laserScanner.updateLock);
    laserScanner.check_exp();
    laserScanner.checkLog();
    for (auto &part: laserScanner.conveyor_partlist) {
        auto last_stamp = part.pose.header.stamp;
        auto current_stamp = ros::Time::now();
        double dt = (current_stamp - last_stamp).toSec();
        part.pose.pose.position.x += part.linear.x * dt;
        part.pose.pose.position.y += part.linear.y * dt;
        part.pose.pose.position.z += part.linear.z * dt;
        part.pose.header.stamp = current_stamp;
    }
    laserScannerConveyor.clear();
    for (auto p: laserScanner.conveyor_partlist) {
        laserScannerConveyor.push_back(p);
    }
    // TODO: merge redundant parts
    inUpdate = false;
}

void SensorManager::startUpdate() {
    updateLock = false;
}

void SensorManager::stopUpdate() {
    updateLock = true;
    while (inUpdate);
}

void SensorManager::forceUpdate() {
    inUpdate = true;
    for (int i = 0; i < cameras.size(); ++i) {
        cameras[i]->forceUpdate();
    }
    laserScanner.forceUpdate();
    while(inUpdate);
}

PartList SensorManager::combineLocations(int locationCode) {
    stopUpdate();
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
        for (auto p: laserScannerConveyor) {
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
    startUpdate();
    return parts;
}

PartList SensorManager::combineLocations(int locationCode, PartList extras) {
    PartList parts = combineLocations(locationCode);
    for (auto p : extras) {
        parts.push_back(p);
    }
    return parts;
}