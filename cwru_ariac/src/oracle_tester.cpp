//
// Created by tianshipei on 12/3/16.
//

#include <OraclePlanner.h>
#include <IDGenerator.h>

int main(int argc, char** argv) {
//    ros::init(argc, argv, "oracle_tester");
//    ros::NodeHandle nh;
//    OraclePlanner oraclePlanner(nh);
    IDGenerator id;
    for (int i = 0; i < 100; ++i) {
        cout << id.genId() << endl;
    }
    for (int i = 10000; i < 10100; ++i) {
        cout << id.genFakeId(i) << endl;
    }
    IDGenerator id2;
    for (int i = 0; i < 100; ++i) {
        cout << id2.genId() << endl;
    }
    return 0;
}