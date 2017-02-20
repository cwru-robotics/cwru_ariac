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
    RobotPlanner robotPlanner(nh);
    OrderManager orderManager(nh);
    RobotMove robotMove(nh);
    ConveyorManager conveyorManager(nh, robotPlanner, robotMove);
    orderManager.startCompetition();
    string agvName = orderManager.AGVs[0].name;
    while (ros::ok()) {
        camera.waitForUpdate();
        if (orderManager.orders.empty()) {
            ROS_INFO("Got no order, waiting...");
            continue;
        }
        for (auto order: orderManager.orders) {
            ROS_INFO("Working on order id: %s", order.order_id.c_str());
            for (auto kit: order.kits) {
                ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
                for (auto object: kit.objects) {
                    ROS_INFO("Working on object type: %s", object.type.c_str());
                    for (auto bin: camera.onBin) {
                        ROS_INFO("Try to find such part in next bin");
                        PartList candidates = findPart(bin, object.type);
                        bool complete = false;
                        for (int i = 0; (i < candidates.size()) && !complete; ++i) {
                            PartList set(candidates.begin() + i, candidates.end());
                            Part best =  conveyorManager.getClosestPart(set);
                            ROS_INFO("got candidate part:");
                            ROS_INFO_STREAM(best);
                            if (robotMove.move(best, orderManager.toAGVPart(agvName, object))) {
                                ROS_INFO("Successfully move part to %s", agvName.c_str());
                                complete = true;
                            }
                        }
                        if (complete)
                            break;
                    }
                    ROS_INFO("complete one object: %s", object.type.c_str());
                }
                ROS_INFO("complete one kit: %s", kit.kit_type.c_str());
                ROS_INFO("Submitting order...");
                bool order_result = orderManager.submitOrder(agvName, kit);
                ROS_INFO("Submission %s", order_result? "success":"failed");
                ros::Duration(21.0).sleep();
                ROS_INFO("Continue");
            }
            ROS_INFO("complete one order: %s", order.order_id.c_str());
        }

    }
    return 0;
}