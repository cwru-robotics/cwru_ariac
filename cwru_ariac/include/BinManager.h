//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_BINMANAGER_H
#define CWRU_ARIAC_BINMANAGER_H

#include <AriacBase.h>


class BinManage: public AriacBase {
public:
    BinManage(ros::NodeHandle nodeHandle);

    vector<Bin> bins;

    bool initBin(PartList& onBin);
    PartList getPutLocation(PartType type);
    PartList getTakeLocation(PartType type);
    bool put(Part part);
    bool take(Part part);

private:
    ros::NodeHandle nh_;
    PartList binParts;
};


#endif //CWRU_ARIAC_BINMANAGER_H
