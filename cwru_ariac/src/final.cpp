//
// Created by tianshipei on 5/1/17.
//


#include <cwru_ariac.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ariac_final");
    ros::NodeHandle nh;
    SensorManager camera(nh);
    camera.addCamera("/ariac/logical_camera_1");
    camera.addCamera("/ariac/logical_camera_2");
    camera.addCamera("/ariac/logical_camera_3");
    camera.addCamera("/ariac/logical_camera_4");
    camera.addCamera("/ariac/logical_camera_5");
    camera.addCamera("/ariac/logical_camera_6");
//    camera.addCamera("/ariac/logical_camera_7");
    OrderManager orderManager(nh);
    RobotMove robotMove(nh);
    robotMove.disableAsync();
    robotMove.toPredefinedPose(RobotMoveGoal::BIN6_HOVER_POSE);
    PlanningUtils planningUtils(nh, robotMove);
    QualitySensor qualitySensor(nh);
    ROS_INFO("Trying to start the competition");
    while (!orderManager.startCompetition());
    ROS_INFO("Competition started");
    int useAGV = 0; //0; change to 1 to force using agv2
    while (ros::ok() && !orderManager.isCompetitionEnd()) {
        if (orderManager.orders.empty()) {
            ROS_INFO_THROTTLE(0.5, "Got no order, waiting...");
            ros::Duration(0.02).sleep();
            continue;
        }
        ROS_INFO("Got %d orders", (int) orderManager.orders.size());
        for (auto order : orderManager.orders) {
            ROS_INFO("Working on order id: %s", order.order_id.c_str());
            for (auto kit : order.kits) {
                PartList extraParts;
                ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
                ROS_INFO("size of kit: %d", (int) kit.objects.size());
                while (!orderManager.isAGVReady(useAGV)) {
                    ROS_WARN_THROTTLE(1.0, "waiting on %s", orderManager.AGVs[useAGV].name.c_str());
                    useAGV = (useAGV == 1 ? 0 : 1);
                    ROS_WARN_THROTTLE(0.5, "Try AGV%d", useAGV + 1);
                    ros::Duration(0.02).sleep();
                }
                ROS_INFO("%s is Ready", orderManager.AGVs[useAGV].name.c_str());
                orderManager.AGVs[useAGV].kitAssigned = kit;
                orderManager.AGVs[useAGV].kitCompleted.kit_type = kit.kit_type;
                orderManager.AGVs[useAGV].kitCompleted.objects.clear();
                while (!orderManager.AGVs[useAGV].kitAssigned.objects.empty()) {
                    for (auto object : orderManager.AGVs[useAGV].kitAssigned.objects) {
                        cout << "=============================================================================" << endl;
                        ROS_INFO("Working on object type: %s", object.type.c_str());
                        ROS_INFO("Remain objects for this kit: %d",
                                 (int) orderManager.AGVs[useAGV].kitAssigned.objects.size());
                        bool succeed = false;
                        PartList failedParts;
                        while (ros::ok() && !succeed) {
                            camera.forceUpdate();
                            PartList allParts = camera.combineLocations(SensorManager::CONVEYOR + SensorManager::BINS
                                                                        + SensorManager::GROUND, extraParts);
                            ROS_INFO("Got %d parts from camera, try to find such part in all places",
                                     (int) allParts.size());
                            PartList candidates = findPart(allParts, object.type);
                            for (auto failed: failedParts) {
                                auto failedIt = findPart(candidates, failed.id);
                                if (failedIt != candidates.end()) {
                                    candidates.erase(failedIt);
                                }
                            }
                            if (candidates.size() == 0) {
                                ROS_WARN("No candidate parts to complete object: %s in kit %s, order %s",
                                         object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                                ROS_WARN("back to this object later");
                                break;
                            }
                            Part target = orderManager.toAGVPart(useAGV, object);
                            Part best = planningUtils.getTargetDistanceBestPart(candidates, target);
                            ROS_INFO("got candidate part from total %d candidates:", (int) candidates.size());
                            ROS_INFO_STREAM(best);
                            ROS_INFO("moving part to target:");
                            ROS_INFO_STREAM(target);
                            if (robotMove.move(best, target)) {
                                ROS_INFO("Successfully moved part to %s, error code is %s",
                                         orderManager.AGVs[useAGV].name.c_str(),
                                         robotMove.getErrorCodeString().c_str());
                                orderManager.AGVs[useAGV].contains.push_back(target);
                                succeed = true;
                                ROS_INFO("Recheck part pose");
                                camera.forceUpdate();
                                vector<pair<Part, Part>> wrong;
                                PartList lost, redundant;
                                PartList compareSource = camera.onAGV[useAGV];
                                PartList compareTarget = orderManager.AGVs[useAGV].contains;
                                if (planningUtils.findDroppedParts(compareSource, compareTarget, wrong, lost,
                                                                   redundant)) {
                                    ROS_WARN("Found parts not in correct pose");
                                    for (auto currentTarget: wrong) {
                                        ROS_INFO("try to adjust part from:");
                                        ROS_INFO_STREAM(currentTarget.first);
                                        ROS_INFO("to");
                                        ROS_INFO_STREAM(currentTarget.second);
                                        if (robotMove.move(currentTarget.first, currentTarget.second)) {
                                            ROS_INFO("move part succeed");
                                        } else {
                                            ROS_INFO("move part failed");
                                        }
                                    }
                                    for (auto lostPart: lost) {
                                        ROS_INFO("add lost parts to kit list for future processing");
                                        ROS_INFO_STREAM(lostPart);
                                        orderManager.AGVs[useAGV].kitAssigned.objects.push_back(
                                                orderManager.toKitObject(useAGV, lostPart));
                                    }
                                    for (auto redundantPart: redundant) {
                                        ROS_INFO("add redundant parts to future list");
                                        extraParts.push_back(redundantPart);
                                    }
                                } else {
                                    ROS_INFO("all parts in correct pose");
                                }
                                orderManager.AGVs[useAGV].kitCompleted.objects.push_back(object);
                                orderManager.AGVs[useAGV].kitAssigned.objects.erase(find_if(
                                        orderManager.AGVs[useAGV].kitAssigned.objects.begin(),
                                        orderManager.AGVs[useAGV].kitAssigned.objects.end(),
                                        [object](osrf_gear::KitObject obj) {
                                            return obj.type == object.type && matchPose(obj.pose, object.pose);
                                        }));
                            } else {
                                ROS_INFO("Failed to transfer the part, reason: %s",
                                         robotMove.getErrorCodeString().c_str());
                                switch (robotMove.getErrorCode()) {
                                    case RobotMoveResult::NO_ERROR:
                                        ROS_INFO("robot move returned NO_ERROR");
                                        break;
                                    case RobotMoveResult::PART_DROPPED: {
                                        ROS_INFO("Checking dropped part location");
                                        camera.forceUpdate();
                                        vector<pair<Part, Part>> wrong;
                                        PartList lost, redundant;
                                        if (planningUtils.findDroppedParts(camera.onAGV[useAGV],
                                                                           orderManager.AGVs[useAGV].contains, wrong,
                                                                           lost, redundant)) {
                                            ROS_INFO("Found parts not in correct position");
                                            for (auto lostPart: lost) {
                                                ROS_INFO("add lost parts to kit object list");
                                                orderManager.AGVs[useAGV].kitAssigned.objects.push_back(
                                                        orderManager.toKitObject(useAGV, lostPart));
                                            }
                                            for (auto redundantPart: redundant) {
                                                ROS_INFO("try to pick the dropped part");
                                                ROS_INFO("numb redundant parts = %d", (int) redundant.size());
                                                if (robotMove.move(redundantPart, target)) {
                                                    orderManager.AGVs[useAGV].contains.push_back(target);
                                                    succeed = true;
                                                    orderManager.AGVs[useAGV].kitCompleted.objects.push_back(object);
                                                    orderManager.AGVs[useAGV].kitAssigned.objects.erase(find_if(
                                                            orderManager.AGVs[useAGV].kitAssigned.objects.begin(),
                                                            orderManager.AGVs[useAGV].kitAssigned.objects.end(),
                                                            [object](osrf_gear::KitObject obj) {
                                                                return obj.type == object.type &&
                                                                       matchPose(obj.pose, object.pose);
                                                            }));
                                                    ROS_INFO("Successfully picked the dropped part");
                                                }
                                                if (!succeed) {
                                                    ROS_WARN("move part failed, add redundant parts to future list");
                                                    extraParts.push_back(redundantPart);
                                                }
                                            }
                                        } else {
                                            ROS_INFO("failed to find target part");
                                        }
                                        break;
                                    }
                                    case RobotMoveResult::GRIPPER_FAULT:
                                        ROS_WARN("Gripper fault, continue? (hit enter)");
                                        getchar();
                                        break;
                                    case RobotMoveResult::COLLISION:
                                        ROS_WARN("Move robot back to home");
                                        robotMove.toHome();
                                        failedParts.push_back(best);
                                        break;
                                    case RobotMoveResult::UNREACHABLE:
                                        ROS_INFO("unreachable; try next part");
                                        failedParts.push_back(best);
                                        break;
                                    case RobotMoveResult::WRONG_PARAMETER:
                                        ROS_WARN("Wrong parameter! going to quit");
                                        return 0;
                                    default:
                                        break;
                                }
                            }
                            ROS_INFO("checking bad parts...");
                            qualitySensor.forceUpdate();
                            if (!qualitySensor.AGVbadParts[useAGV].empty()) {
                                ROS_WARN("Found bad part! Going to discard them");
                                for (auto badPart: qualitySensor.AGVbadParts[useAGV]) {
                                    ROS_INFO("Working on bad part:");
                                    ROS_INFO_STREAM(badPart);
                                    if (robotMove.pick(badPart)) {
                                        bool moveResult = false;
                                        if (useAGV == 0) {
                                            moveResult = robotMove.toPredefinedPose(RobotMoveGoal::AGV1_CRUISE_POSE);
                                        } else {
                                            moveResult = robotMove.toPredefinedPose(RobotMoveGoal::AGV2_CRUISE_POSE);
                                        }
                                        if (moveResult) {
                                            if (robotMove.release()) {
                                                ROS_INFO("Successfully removed bad part from tray");
                                                ROS_INFO("add bad parts to kit list to fill them");
                                                auto kitObject = orderManager.toKitObject(useAGV, badPart);
                                                orderManager.AGVs[useAGV].kitAssigned.objects.push_back(kitObject);
                                                orderManager.AGVs[useAGV].kitCompleted.objects.erase(
                                                        find_if(orderManager.AGVs[useAGV].kitCompleted.objects.begin(),
                                                                orderManager.AGVs[useAGV].kitCompleted.objects.end(),
                                                                [kitObject](osrf_gear::KitObject obj) {
                                                                    return obj.type == kitObject.type &&
                                                                           matchPose(obj.pose, kitObject.pose);
                                                                }));
                                                orderManager.AGVs[useAGV].contains.erase(
                                                        find_if(orderManager.AGVs[useAGV].contains.begin(),
                                                                orderManager.AGVs[useAGV].contains.end(),
                                                                [badPart](Part part) {
                                                                    return isSamePart(badPart, part);
                                                                }));
                                                continue;
                                            } else {
                                                ROS_WARN("Failed to release the gripper, reason: %s",
                                                         robotMove.getErrorCodeString().c_str());
                                            }
                                        } else {
                                            ROS_WARN("Failed to move robot to AGV1_CRUISE_POSE, reason: %s",
                                                     robotMove.getErrorCodeString().c_str());
                                        }
                                    } else {
                                        ROS_WARN("Failed to pick the bad part, reason: %s",
                                                 robotMove.getErrorCodeString().c_str());
                                    }
                                    ROS_WARN("Not able to discard bad part");
                                }
                            } else {
                                ROS_INFO("bad part check passed!");
                            }
                        }
                        if (succeed) {
                            break;
                        }
                    }
                }
                ROS_INFO("complete objects in kit: %s in order %s", kit.kit_type.c_str(), order.order_id.c_str());
                ROS_INFO("Submitting order...");
                bool order_result = orderManager.submitOrder(useAGV, kit);
                ROS_INFO("Submission %s", order_result ? "success" : "failed");
                ros::spinOnce();
                orderManager.AGVs[useAGV].state = AGV::DELIVERING; // need to wait on AGV1
                ros::Duration(2.0).sleep();
                ROS_INFO("Current score: %f", orderManager.getCurrentScore());
                orderManager.AGVs[useAGV].contains.clear();
                order.kits.erase(find_if(
                        order.kits.begin(), order.kits.end(),
                        [kit](osrf_gear::Kit obj) {
                            return obj.kit_type == kit.kit_type;
                        }));
            }
            ROS_INFO("completed one order: %s", order.order_id.c_str());
            orderManager.orders.erase(find_if(
                    orderManager.orders.begin(), orderManager.orders.end(),
                    [order](osrf_gear::Order obj) {
                        return obj.order_id == order.order_id;
                    }));
        }
        ROS_INFO("completed all orders; waiting for new order");
    }
    ROS_INFO("Competition Ended, final score: %f", orderManager.getCurrentScore());
    return 0;
}
