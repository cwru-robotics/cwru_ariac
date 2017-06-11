//
// Created by tianshipei on 2/16/17.
//

#include <cwru_ariac.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ariac_qual1");
    ros::NodeHandle nh;
    CameraEstimator camera(nh, "/ariac/logical_camera_1");
    OrderManager orderManager(nh);
    RobotMove robotMove(nh);
    PlanningUtils planningUtils(nh, robotMove);
    robotMove.disableAsync();
    ROS_INFO("Try to start the competition");
    while(!orderManager.startCompetition());
    ROS_INFO("Competition started");
    string agvName = orderManager.AGVs[0].name;
    while (ros::ok() && !orderManager.isCompetitionEnd()) {
        camera.forceUpdate();
        if (orderManager.orders.empty()) {
            ROS_INFO("Got no order, waiting...");
            continue;
        }
        ROS_INFO("Got %d orders", (int)orderManager.orders.size());
        for (auto order: orderManager.orders) {
            ROS_INFO("Working on order id: %s", order.order_id.c_str());
            for (auto kit: order.kits) {
                ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
                for (auto object: kit.objects) {
                    ROS_INFO("Working on object type: %s", object.type.c_str());
                    camera.forceUpdate();
                    PartList all_bins;
                    int bin_cnt = 1;
                    for (auto bin: camera.onBin) {
                        ROS_INFO("bin %d have %d parts", bin_cnt++, (int)bin.size());
                        for (auto p: bin) {
                            all_bins.push_back(p);
                        }
                    }

                    ROS_INFO("Got %d parts from camera, try to find such part in all bins", (int)all_bins.size());
                    PartList candidates = findPart(all_bins, object.type);
                    ROS_INFO("Found %d parts from bins", (int)candidates.size());
                    while (!candidates.empty()) {
                        Part best = planningUtils.getEuclideanBestPart(candidates);
                        Part target = orderManager.toAGVPart(0, object);
                        candidates.erase(findPart(candidates, best.id));
                        ROS_INFO("got candidate part from total %d candidates:", (int)candidates.size());
                        ROS_INFO_STREAM(best);
                        ROS_INFO("moving part to target:");
                        ROS_INFO_STREAM(target);
                        if (robotMove.move(best, target)) {
                            ROS_INFO("Successfully move part to %s", agvName.c_str());
                            camera.forceUpdate();
                            break;
                        }
                        ROS_INFO("Failed to fetch the part, reason: %s", robotMove.getErrorCodeString().c_str());
                        switch (robotMove.getErrorCode()) {
                            case RobotMoveResult::PART_DROPPED:
                                ROS_WARN("Continue with a new part? (hit enter)");
                                getchar();
                                break;
                            case RobotMoveResult::GRIPPER_FAULT:
                                ROS_WARN("Gripper fault, continue? (hit enter)");
                                getchar();
                                break;
                            case RobotMoveResult::COLLISION:
                                ROS_WARN("Move robot back to home");
                                robotMove.toHome();
                                break;
                            case RobotMoveResult::UNREACHABLE:
                                ROS_INFO("try next part");
                                break;
                            case RobotMoveResult::WRONG_PARAMETER:
                                ROS_WARN("Wrong parameter! going to quit");
                                return 0;
                            default:
                                break;
                        }
                        camera.forceUpdate();
                    }
                    if (candidates.size() != 0) {
                        ROS_INFO("Complete one object: %s in kit %s, order %s", object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                    } else {
                        ROS_WARN("Failed all candidate parts to complete object: %s in kit %s, order %s",
                                 object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                        ROS_WARN("Ignore this object and continue? (hit enter)");
                        getchar();
                    }
                }
                ROS_INFO("complete one kit: %s in order %s", kit.kit_type.c_str(), order.order_id.c_str());
                ROS_INFO("Submitting order...");
                bool order_result = orderManager.submitOrder(0, kit);
                ROS_INFO("Submission %s", order_result? "success":"failed");
                ros::Duration(2.0).sleep();
                ROS_INFO("Current score: %f", orderManager.getCurrentScore());
            }
            ROS_INFO("complete one order: %s", order.order_id.c_str());
            ROS_INFO("Current score: %f", orderManager.getCurrentScore());
        }
        ROS_INFO("complete all orders, waiting for new order");
    }
    ROS_INFO("Competition Ended, final score: %f", orderManager.getCurrentScore());
    return 0;
}
