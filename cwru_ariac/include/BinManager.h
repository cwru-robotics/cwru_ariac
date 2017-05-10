//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_BINMANAGER_H
#define CWRU_ARIAC_BINMANAGER_H

#include <AriacBase.h>
#include <CameraEstimator.h>

class BinManager: public AriacBase {
public:
    BinManager(ros::NodeHandle &nodeHandle);
    // BinManager should remember all the parts inside all bins and keep update this list
    vector<Bin> bins;
    // init all bins with camera information.
    void initBins(vector<PartList> onBins);
    // assign a camera to bins, BinManager should remember this camera object and keep updating bins that are inside camera FOV
    void assignCamera(CameraEstimator &camera);
    // query all available locations for perform taking a certain type of part. return false when no available space,
    // locations are converted to a fake Part just like the trick in OrderManager and stores in the PartList
    bool allLocationsForTake(string partName, PartList &locations);
    // similar to AllLocationsForTake, get candidate locations for put a part to any of bins. Return false when no such part available
    bool allLocationsForPut(string partName, PartList &locations);
    // query for the best location to take a certain type of part, return false when no available space
    bool advisedLocationForTake(string partName, Part &partToTake);
    // query for the best location to store a certain type of part, return false when no such part available
    bool advisedLocationForPut(string partName, Part &location);
    // this function will be called when robot put a part to a bin to inform BinManager the action, this part should be stored inside BinManager for future take action
    void put(Part part);
    // this function will be called when robot take a part from a bin, this part should be removed from BinManager
    void taken(Part part);
    // When there have been a lot of bin in and out operations, bin layout can be messy, so BinManager is allowed to perform a in position rearrangement
    // If allowRearrange sets to true, BinManager can use more aggressive scheme to organize parts, rearrangePriority indicates the necessity of rearrange bins
    // This function returns false if no need to rearrange or no available scheme to rearrange or allowRearrange sets to false
    // actions is the movements that requires when rearranging the bin, part will be moved from pair.first to pair.second in the order of the vector grows.
//    bool rearrangeBins(vector<pair<Part, Part>> &actions);
//    double getRearrangePriority();
//    void setAllowRearrange(bool value);

protected:
    ros::NodeHandle nh;
    PartList binParts;
//    bool allowRearrange;
//    double rearrangePriority;
};


#endif //CWRU_ARIAC_BINMANAGER_H
