
#ifndef CWRU_ARIAC_SWAPPING_H
#define CWRU_ARIAC_SWAPPING_H


#include <AriacBase.h>
#include <cwru_ariac.h>
#include <RobotMove.h>


class SwappingAlgorithm: public AriacBase {
public:
	SwappingAlgorithm(ros::NodeHandle nodeHandle);

    Part toAGVPart(string agvName, osrf_gear::KitObject object);

private:
    ros::NodeHandle nh;
	void parts_location(Part &part_1, Part &part_2);
	void storage_onAGV(Part &storage);

	geometry_msgs::PoseStamped temp_location1,temp_location2;
	Part part_1,part_2,storage;
	RobotMove robotMove(nh);

};

#endif //CWRU_ARIAC_SWAPPING_H