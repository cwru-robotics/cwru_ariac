//
// Created by tianshipei on 2/16/17.
//

#include <AriacBase.h>
#include <CameraEstimator.h>
#include <OrderManager.h>
#include <RobotPlanner.h>
#include <RobotMove.h>
#include <ConveyorManager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ariac_qual1");
    ros::NodeHandle nh;
    CameraEstimator camera(nh);
    RobotPlanner robot(nh);
    ConveyorManager conveyor(nh, camera, robot);
    while (ros::ok()) {
        camera.waitForUpdate();

    }
    return 0;
}