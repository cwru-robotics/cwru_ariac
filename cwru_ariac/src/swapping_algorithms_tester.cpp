//
// Created by tianshipei on 12/3/16.
//

#include <SwappingAlgorithm.h>
#include <OrderManager.h>
#include <CameraEstimator.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "swapping_algorithms_tester");
    ros::NodeHandle nh;
    RobotMove robotMove(nh);
    BinManager binManager(nh);
    SwappingAlgorithm swappingAlgorithm(nh, robotMove, binManager);
    OrderManager orderManager(nh);
    CameraEstimator cameraEstimator(nh);
    vector<pair<Part, Part>> correction;
    PartList lost;
    PartList redundant;

    if (orderManager.findDroppedParts(orderManager.AGVs[0].contains, cameraEstimator.onAGV[0], correction, lost,
                                      redundant)) {
        if (swappingAlgorithm.swapParts(correction)) {
            ROS_INFO("wrong location part corrected");
        }

    }

    return 0;
}