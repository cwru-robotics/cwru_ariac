
#ifndef CWRU_ARIAC_SWAPPING_H
#define CWRU_ARIAC_SWAPPING_H


#include <AriacBase.h>
#include <BinManager.h>
#include <RobotMove.h>


class SwappingAlgorithm: public AriacBase {
public:
    SwappingAlgorithm(ros::NodeHandle nodeHandle, RobotMove robotMove, BinManager binManager);

    bool swapParts(vector<pair<Part, Part>> action);
    // first Part is a current exist part and second part is the target location of the part. This function should correct the location for all the pairs of part.

//    Part toAGVPart(string agvName, osrf_gear::KitObject object);

private:
    ros::NodeHandle nh;
	void parts_location(Part &part_1, Part &part_2);
	void storage_onAGV(Part &storage);

	geometry_msgs::PoseStamped temp_location1,temp_location2;
	Part part_1,part_2,storage;
    shared_ptr<RobotMove> robotMovePtr;
    shared_ptr<BinManager> binManagerPtr;

};

#endif //CWRU_ARIAC_SWAPPING_H