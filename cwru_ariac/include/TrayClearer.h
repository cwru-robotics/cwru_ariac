//
// Created by tianshipei on 4/25/17.
//

#ifndef CWRU_ARIAC_TRAYCLEARER_H
#define CWRU_ARIAC_TRAYCLEARER_H

#include <AriacBase.h>
#include <BinManager.h>
#include <RobotMove.h>

class TrayClearer {
    TrayClearer(ros::NodeHandle nodeHandle, RobotMove robotMove, BinManager binManager);

    bool clearPartsFromTray(PartList partsToClear);

protected:
    ros::NodeHandle nh;
    shared_ptr<RobotMove> robotMovePtr;
    shared_ptr<BinManager> binManagerPtr;

};


#endif //CWRU_ARIAC_TRAYCLEARER_H
