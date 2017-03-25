//
// test_conveyor_fetch: for devel/test of robot move action-server function for fetching part from conveyor
// based on qualifier2
// wsn 3/24/17

#include <cwru_ariac.h>

const int useAGV = 0;

bool findDroppedParts(PartList searchList, PartList targetList, vector<pair<Part, Part>> &wrongLocationParts, PartList &lostParts, PartList &redundantParts) {
    wrongLocationParts.clear();
    lostParts.clear();
    redundantParts.clear();
    PartList not_in_search;
    PartList kit_tray = findPart(searchList, "kit_tray");
    searchList.erase(findPart(searchList, kit_tray[0].id));
    for (auto searching: searchList) {
        bool ignored = false;
        for (int i = 0; i < targetList.size(); ++i) {
            if (searching.name == targetList[i].name && matchPose(searching.pose.pose, targetList[i].pose.pose)) {
                targetList.erase(targetList.begin() + i);
                ignored = true;
            }
        }
        if (!ignored) {
            // ROS_INFO("Found part mismatch");
            not_in_search.push_back(searching);
        }
    }
    for (auto matching: not_in_search) {
        PartList candidates = findPart(targetList, matching.name);
        if (candidates.empty()) {
            redundantParts.push_back(matching);
            continue;
        }
        auto closest = min_element(candidates.begin(), candidates.end(), [matching](Part A, Part B) {
            return euclideanDistance(matching.pose.pose.position, A.pose.pose.position) < euclideanDistance(matching.pose.pose.position, B.pose.pose.position);
        });
        wrongLocationParts.push_back(pair<Part, Part>(matching, *closest));
        targetList.erase(findPart(targetList, (*closest).id));
    }
    for (auto p: targetList) {
        lostParts.push_back(p);
    }
    return !(not_in_search.empty() && targetList.empty());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ariac_qual2");
    ros::NodeHandle nh;
    CameraEstimator binCamera(nh, "/ariac/logical_camera_1");
    CameraEstimator agv1Camera(nh, "/ariac/logical_camera_2");
    RobotPlanner robotPlanner(nh);
    OrderManager orderManager(nh);
    RobotMove robotMove(nh);
    GlobalPlanner globalPlanner(nh, robotMove);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    robotMove.disableAsync();
    robotMove.toPredefinedPose(RobotMoveGoal::BIN6_HOVER_POSE);
    ROS_INFO("Trying to start the competition");
    while(!orderManager.startCompetition()) ;
    ROS_INFO("Competition started");
    string agvName = orderManager.AGVs[useAGV].name;
    while (ros::ok() && !orderManager.isCompetitionEnd()) {
        agv1Camera.waitForUpdate();
        if (orderManager.orders.empty()) {
            ROS_INFO("Got no order, waiting...");
            continue;
        }
        ROS_INFO("Got %d orders", (int)orderManager.orders.size());
        for (auto order : orderManager.orders) {
            ROS_INFO("Working on order id: %s", order.order_id.c_str());
            for (auto kit : order.kits) {
                PartList completedObjects;
                PartList extraParts;
                ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
                ROS_INFO("size of kit: %d",(int)kit.objects.size());
                while (!orderManager.isAGVReady(useAGV)) {
                    ROS_WARN_ONCE("waiting on AGV%d", useAGV);
                    agv1Camera.waitForUpdate();
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("AGV%d is Ready", useAGV);
                for (auto object : kit.objects) {
                    ROS_INFO("Working on object type: %s", object.type.c_str());
                    bool succeed = false;
                    while (ros::ok() && !succeed) {
                        agv1Camera.waitForUpdate();
                        binCamera.waitForUpdate();
                        PartList allParts;
                        int bin_cnt = 1;
                        for (auto p : extraParts) {
                            allParts.push_back(p);
                        }
                        for (auto bin : binCamera.onBin) {
                            ROS_INFO("bin %d has %d parts", bin_cnt++, (int)bin.size());
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
                        ROS_INFO("Got %d parts from binCamera, try to find such part in all bins", (int)allParts.size());
                        PartList candidates = findPart(allParts, object.type);
                        while (candidates.size() == 0) {
                            allParts.clear();
                            ROS_WARN("No candidate parts to complete object: %s in kit %s, order %s",
                                     object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                            ROS_WARN("waiting...");
                            ros::spinOnce();
                            agv1Camera.waitForUpdate();
                           for (auto p : agv1Camera.onConveyor) {
                            allParts.push_back(p);
                           }
                            candidates = findPart(allParts, object.type);
                            //getchar();
                            //break;
                        }
                        ROS_INFO("Found %d parts on conveyor", (int)candidates.size());
//                        Part best = globalPlanner.getEuclideanBestPart(candidates);
                        Part target = orderManager.toAGVPart(agvName, object);
                        Part best = globalPlanner.getTargetDistanceBestPart(candidates, target);
                        ROS_INFO("got candidate part from total %d candidates:", (int)candidates.size());
                        ROS_INFO_STREAM(best);
                        ROS_INFO("attempting fetch from conveyor: ");
                        if (robotMove.fetchPartFromConveyor(best,target)) {
                          ROS_INFO("fetchPartFromConveyor returned true");
                        }
                        else {
                            ROS_INFO("fetchPartFromConveyor returned false");
                        }
                         return 0; //ignore rest of code from here

                        ROS_INFO("moving part to target:");
                        ROS_INFO_STREAM(target);
                        candidates.erase(findPart(candidates, best.id));
                        if (robotMove.move(best, target)) {
                            ROS_INFO("Successfully moved part to %s", agvName.c_str());
                            completedObjects.push_back(target);
                            succeed = true;
                            ROS_INFO("Recheck part pose");
                            agv1Camera.waitForUpdate();
                            binCamera.waitForUpdate();
                            vector<pair<Part, Part>> wrong;
                            PartList lost, redundant;
                            if (findDroppedParts(agv1Camera.onAGV[0], completedObjects, wrong, lost, redundant)) {
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
                                    kit.objects.push_back(add_to_list);
                                }
                                for (auto redundantParts: redundant) {
                                    ROS_INFO("add redundant parts to future list");
                                    extraParts.push_back(redundantParts);
                                }
                            } else {
                                ROS_INFO("all parts in correct pose");
                            }
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
                                agv1Camera.waitForUpdate();
                                binCamera.waitForUpdate();
                                vector<pair<Part, Part>> wrong;
                                PartList lost, redundant;
                                if (findDroppedParts(agv1Camera.onAGV[0], completedObjects, wrong, lost, redundant)) {
                                    ROS_INFO("Found parts not in correct position");
                                    for (auto lostPart: lost) {
                                        ROS_INFO("add lost parts to kit object list");
                                        osrf_gear::KitObject add_to_list;
                                        add_to_list.pose = lostPart.pose.pose;
                                        add_to_list.type = lostPart.name;
                                        kit.objects.push_back(add_to_list);
                                    }
                                    for (auto redundantParts: redundant) {
                                        ROS_INFO("try to pick the dropped part");
                                        ROS_INFO("numb redundant parts = %d",(int) redundant.size());
                                        if (robotMove.move(redundantParts, target)) {
                                            completedObjects.push_back(target);
                                            succeed = true;
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
                        binCamera.waitForUpdate();
                        agv1Camera.waitForUpdate();
                    }
                }
                ROS_INFO("complete objects in kit: %s in order %s", kit.kit_type.c_str(), order.order_id.c_str());
                ROS_INFO("Submitting order...");
                bool order_result = orderManager.submitOrder(agvName, kit);
                ROS_INFO("Submission %s", order_result ? "success" : "failed");
                ros::spinOnce();
                orderManager.AGVs[0].state = AGV::DELIVERING; // need to wait on AGV1
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
