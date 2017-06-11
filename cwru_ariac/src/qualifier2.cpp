//
// Created by tianshipei on 2/16/17.
// Modified wsn 3/15/17

#include <cwru_ariac.h>

const int useAGV = 0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ariac_qual2");
    ros::NodeHandle nh;
    CameraEstimator binCamera(nh, "/ariac/logical_camera_1");
    CameraEstimator agv1Camera(nh, "/ariac/logical_camera_2");
    OrderManager orderManager(nh);
    RobotMove robotMove(nh);
    PlanningUtils planningUtils(nh, robotMove);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    robotMove.disableAsync();
    robotMove.toPredefinedPose(RobotMoveGoal::BIN6_HOVER_POSE, 0);
    ROS_INFO("Trying to start the competition");
    while(!orderManager.startCompetition()) ;
    ROS_INFO("Competition started");
    while (ros::ok() && !orderManager.isCompetitionEnd()) {
        agv1Camera.forceUpdate();
        if (orderManager.orders.empty()) {
            ROS_INFO("Got no order, waiting...");
            continue;
        }
        ROS_INFO("Got %d orders", (int)orderManager.orders.size());
        for (auto order : orderManager.orders) {
            ROS_INFO("Working on order id: %s", order.order_id.c_str());
            for (auto kit : order.kits) {
                PartList extraParts;
                ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
                ROS_INFO("size of kit: %d",(int)kit.objects.size());
                while (!orderManager.isAGVReady(useAGV)) {
                    ROS_WARN_ONCE("waiting on AGV%d", useAGV);
                    agv1Camera.forceUpdate();
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("AGV%d is Ready", useAGV);
                orderManager.AGVs[useAGV].kitAssigned = kit;
                orderManager.AGVs[useAGV].kitCompleted.kit_type = kit.kit_type;
                orderManager.AGVs[useAGV].kitCompleted.objects.clear();
                while (!orderManager.AGVs[useAGV].kitAssigned.objects.empty()) {
                    for (auto object : orderManager.AGVs[useAGV].kitAssigned.objects) {
                        ROS_INFO("Working on object type: %s", object.type.c_str());
                        bool succeed = false;
                        while (ros::ok() && !succeed) {
                            agv1Camera.forceUpdate();
                            binCamera.forceUpdate();
                            PartList allParts;
                            for (auto p : extraParts) {
                                allParts.push_back(p);
                            }
                            for (auto bin : binCamera.onBin) {
                                for (auto p : bin) {
                                    allParts.push_back(p);
                                }
                            }
                            for (auto p : binCamera.onGround) {
                                allParts.push_back(p);
                            }
                            for (auto p : binCamera.onConveyor) {
                                allParts.push_back(p);
                            }
                            for (auto p : agv1Camera.onGround) {
                                allParts.push_back(p);
                            }
                            for (auto p : agv1Camera.onConveyor) {
                                allParts.push_back(p);
                            }
                            ROS_INFO("Got %d parts from binCamera, try to find such part in all places", (int)allParts.size());
                            PartList candidates = findPart(allParts, object.type);
                            if (candidates.size() == 0) {
                                ROS_WARN("No candidate parts to complete object: %s in kit %s, order %s",
                                         object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                                ROS_WARN("back to this object later");
                                break;
                            }
                            ROS_INFO("Found %d parts from all places", (int)candidates.size());
//                        Part best = planningUtils.getEuclideanBestPart(candidates);
                            Part target = orderManager.toAGVPart(useAGV, object);
                            Part best = planningUtils.getTargetDistanceBestPart(candidates, target);
                            ROS_INFO("got candidate part from total %d candidates:", (int)candidates.size());
                            ROS_INFO_STREAM(best);
                            ROS_INFO("moving part to target:");
                            ROS_INFO_STREAM(target);
                            candidates.erase(findPart(candidates, best.id));
                            if (robotMove.move(best, target)) {
                                ROS_INFO("Successfully moved part to %s, error code is %s",
                                         orderManager.AGVs[useAGV].name.c_str(),
                                         robotMove.getErrorCodeString().c_str());
                                orderManager.AGVs[useAGV].contains.push_back(target);
                                succeed = true;
                                ROS_INFO("Recheck part pose");
                                agv1Camera.forceUpdate();
                                binCamera.forceUpdate();
                                vector<pair<Part, Part>> wrong;
                                PartList lost, redundant;
                                if (planningUtils.findDroppedParts(agv1Camera.onAGV[useAGV], orderManager.AGVs[useAGV].contains, wrong, lost, redundant)) {
                                    ROS_INFO("Found parts not in correct pose");
                                    for (auto currentTarget:wrong) {
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
                                        ROS_INFO("add lost parts to future list");
                                        osrf_gear::KitObject add_to_list;
                                        add_to_list.pose = lostPart.pose.pose;
                                        add_to_list.type = lostPart.name;
                                        orderManager.AGVs[useAGV].kitAssigned.objects.push_back(add_to_list);
                                    }
                                    for (auto redundantParts: redundant) {
                                        ROS_INFO("add redundant parts to future list");
                                        extraParts.push_back(redundantParts);
                                    }
                                } else {
                                    ROS_INFO("all parts in correct pose");
                                }
                                orderManager.AGVs[useAGV].kitCompleted.objects.push_back(object);
                                orderManager.AGVs[useAGV].kitAssigned.objects.erase(find_if(
                                        orderManager.AGVs[useAGV].kitAssigned.objects.begin(), orderManager.AGVs[useAGV].kitAssigned.objects.end(),
                                        [object](osrf_gear::KitObject obj) {
                                            return obj.type == object.type && matchPose(obj.pose, object.pose);
                                        }));
                                break;
                            }
                            ROS_INFO("Failed to transfer the part, reason: %s", robotMove.getErrorCodeString().c_str());
                            switch (robotMove.getErrorCode()) {
                                case RobotMoveResult::NO_ERROR:
                                    ROS_INFO("robot move returned NO_ERROR");
                                    break;
                                case RobotMoveResult::PART_DROPPED:
                                {
                                    ROS_INFO("Checking dropped part location");
                                    agv1Camera.forceUpdate();
                                    binCamera.forceUpdate();
                                    vector<pair<Part, Part>> wrong;
                                    PartList lost, redundant;
                                    if (planningUtils.findDroppedParts(agv1Camera.onAGV[useAGV], orderManager.AGVs[useAGV].contains, wrong, lost, redundant)) {
                                        ROS_INFO("Found parts not in correct position");
                                        for (auto lostPart: lost) {
                                            ROS_INFO("add lost parts to kit object list");
                                            osrf_gear::KitObject add_to_list;
                                            add_to_list.pose = lostPart.pose.pose;
                                            add_to_list.type = lostPart.name;
                                            orderManager.AGVs[useAGV].kitAssigned.objects.push_back(add_to_list);
                                        }
                                        for (auto redundantParts: redundant) {
                                            ROS_INFO("try to pick the dropped part");
                                            ROS_INFO("numb redundant parts = %d",(int) redundant.size());
                                            if (robotMove.move(redundantParts, target)) {
                                                orderManager.AGVs[useAGV].contains.push_back(target);
                                                succeed = true;
                                                orderManager.AGVs[useAGV].kitCompleted.objects.push_back(object);
                                                orderManager.AGVs[useAGV].kitAssigned.objects.erase(find_if(
                                                        orderManager.AGVs[useAGV].kitAssigned.objects.begin(), orderManager.AGVs[useAGV].kitAssigned.objects.end(),
                                                        [object](osrf_gear::KitObject obj) {
                                                            return obj.type == object.type && matchPose(obj.pose, object.pose);
                                                        }));
                                                break;
                                            }
                                            if (!succeed) {
                                                ROS_INFO("move part failed, add redundant parts to future list");
                                                extraParts.push_back(redundantParts);
                                                continue;
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
                                    break;
                                case RobotMoveResult::UNREACHABLE:
                                    ROS_INFO("unreachable; try next part");
                                    break;
                                case RobotMoveResult::WRONG_PARAMETER:
                                    ROS_WARN("Wrong parameter! going to quit");
                                    return 0;
                                default:
                                    break;
                            }
                            binCamera.forceUpdate();
                            agv1Camera.forceUpdate();
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
            }
            ROS_INFO("completed one order: %s", order.order_id.c_str());

            ROS_INFO("Current score: %f", orderManager.getCurrentScore());
        }
        ROS_INFO("completed all orders; waiting for new order");
    }
    ROS_INFO("Competition Ended, final score: %f", orderManager.getCurrentScore());
    return 0;
}
