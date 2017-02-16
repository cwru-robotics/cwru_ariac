//
// Created by tianshipei on 12/3/16.
//

#include <AriacBase.h>
#include <OraclePlanner.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "oracle_tester");
    ros::NodeHandle nh;
    OraclePlanner oraclePlanner(nh);
    return 0;
}