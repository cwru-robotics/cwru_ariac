//
// Created by shipei on 11/1/16.
//

#include "BinManager.h"

BinManager::BinManager(ros::NodeHandle &nodeHandle) : nh(nodeHandle) {
    for (int i = 0; i < totalBins; ++i) {
        bins.push_back(defaultBin);
    }
    bins[0].name = "Bin4";
    bins[0].priority = 6.0;
    bins[0].pose.pose.position.x = -1.000000;
    bins[0].pose.pose.position.y = 0.995000;
    bins[0].pose.pose.position.z = 0.0;

    bins[1].name = "Bin8";
    bins[1].priority = 10.0;
    bins[1].pose.pose.position.x = -0.300000;
    bins[1].pose.pose.position.y = 0.995000;
    bins[1].pose.pose.position.z = 0.0;

    bins[2].name = "Bin3";
    bins[2].priority = 4.0;
    bins[2].pose.pose.position.x = -1.000000;
    bins[2].pose.pose.position.y = 0.230000;
    bins[2].pose.pose.position.z = 0.0;

    bins[3].name = "Bin7";
    bins[3].priority = 8.0;
    bins[3].pose.pose.position.x = -0.300000;
    bins[3].pose.pose.position.y = 0.230000;
    bins[3].pose.pose.position.z = 0.0;

    bins[3].name = "Bin2";
    bins[3].priority = 3.0;
    bins[3].pose.pose.position.x = -1.000000;
    bins[3].pose.pose.position.y = -0.535000;
    bins[3].pose.pose.position.z = 0.0;

    bins[3].name = "Bin6";
    bins[3].priority = 7.0;
    bins[3].pose.pose.position.x = -0.300000;
    bins[3].pose.pose.position.y = -0.535000;
    bins[3].pose.pose.position.z = 0.0;

    bins[3].name = "Bin1";
    bins[3].priority = 5.0;
    bins[3].pose.pose.position.x = -1.000000;
    bins[3].pose.pose.position.y = -1.330000;
    bins[3].pose.pose.position.z = 0.0;

    bins[3].name = "Bin5";
    bins[3].priority = 9.0;
    bins[3].pose.pose.position.x = -0.300000;
    bins[3].pose.pose.position.y = -1.330000;
    bins[3].pose.pose.position.z = 0.0;

    for (auto &&bin: bins) {
        int index = stoi(bin.name.substr(bin.name.find("Bin") + 3));
        bin.bound = binBoundBox[index];
    }
    //sort(bins.begin(), bins.end(), [](Bin a, Bin b){ return a.priority > b.priority;});
}

bool BinManager::allLocationsForPut(string partName, PartList &locations) {
    return true;
}

bool BinManager::allLocationsForTake(string partName, PartList &locations) {
    return true;
}

bool BinManager::advisedLocationForPut(string partName, Part &location) {
    Part fakePart;
    fakePart.name = partName;
    fakePart.id = 20000;
    fakePart.location = Part::BIN1;
    fakePart.pose.header.frame_id = "world";
    fakePart.pose.header.stamp = ros::Time::now();
    fakePart.pose.pose.position.x = -1.000000;
    fakePart.pose.pose.position.y = -1.000000;
    fakePart.pose.pose.position.z = 0.755;
    fakePart.pose.pose.orientation.z = 1;
    fakePart.traceable = true;
    location = fakePart;
    return true;
}

bool BinManager::advisedLocationForTake(string partName, Part &partToTake) {
    return true;
}