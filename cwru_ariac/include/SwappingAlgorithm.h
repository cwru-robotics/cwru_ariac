//
// Created by Ammar Nahari on 5/5/2017.
//

#ifndef CWRU_ARIAC_SWAPPING_H
#define CWRU_ARIAC_SWAPPING_H


#include <AriacBase.h>
#include <BinManager.h>
#include <RobotMove.h>


class SwappingAlgorithm: public AriacBase {
public:
    SwappingAlgorithm(ros::NodeHandle& nodeHandle, RobotMove& robotMove, BinManager& binManager);

    bool swapParts(vector<pair<Part, Part>> action);
    // first Part is a current exist part and second part is the target location of the part. This function should correct the location for all the pairs of part.


private:
    ros::NodeHandle nh;

    //defined variable for storing the first part
	Part temp_part; 


    RobotMove *robotMovePtr;
    BinManager *binManagerPtr;

};

#endif //CWRU_ARIAC_SWAPPING_H