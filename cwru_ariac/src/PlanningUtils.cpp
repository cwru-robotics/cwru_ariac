//
// Created by tianshipei on 12/3/16.
//

#include "PlanningUtils.h"

PlanningUtils::PlanningUtils(ros::NodeHandle &nodeHandle, RobotMove &robot) :
        nh(nodeHandle), robot_(&robot) {
    upsideDownPenalty = 20.0;
    conveyorBonus = 20.0;
}

Part PlanningUtils::getEuclideanBestPart(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    return *min_element(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b) {
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position));
    });
}

PartList PlanningUtils::sortByEuclidean(PartList searchRange) {
    RobotState state;
    robot_->getRobotState(state);
    sort(searchRange.begin(), searchRange.end(), [this, state](Part a, Part b) {
        return euclideanDistance((state.gripperPose.pose.position), (a.pose.pose.position))
               < euclideanDistance((state.gripperPose.pose.position), (b.pose.pose.position));
    });
    return searchRange;
}

Part PlanningUtils::getTargetDistanceBestPart(PartList searchRange, Part target) {
    return *min_element(searchRange.begin(), searchRange.end(), [this, target](Part a, Part b) {
        double distance_a = euclideanDistance((target.pose.pose.position), (a.pose.pose.position))
                            + (evalUpDown(target.pose.pose.orientation)
                               == evalUpDown(a.pose.pose.orientation) ? 0 : upsideDownPenalty)
                            - (a.location == Part::CONVEYOR ? conveyorBonus : 0);
        double distance_b = euclideanDistance((target.pose.pose.position), (b.pose.pose.position))
                            + (evalUpDown(target.pose.pose.orientation)
                               == evalUpDown(b.pose.pose.orientation) ? 0 : upsideDownPenalty)
                            - (b.location == Part::CONVEYOR ? conveyorBonus : 0);
        return distance_a < distance_b;
    });
}

PartList PlanningUtils::sortByTargetDistance(PartList searchRange, Part target) {
    sort(searchRange.begin(), searchRange.end(), [this, target](Part a, Part b) {
        double distance_a = euclideanDistance((target.pose.pose.position), (a.pose.pose.position))
                            + (evalUpDown(target.pose.pose.orientation)
                               == evalUpDown(a.pose.pose.orientation) ? 0 : upsideDownPenalty);
        double distance_b = euclideanDistance((target.pose.pose.position), (b.pose.pose.position))
                            + (evalUpDown(target.pose.pose.orientation)
                               == evalUpDown(b.pose.pose.orientation) ? 0 : upsideDownPenalty);
        return distance_a < distance_b;
    });
    return searchRange;
}

/// This method is designed for Kit tray, it tries to match parts in searchList and targetList,
/// in order to find parts in wrong location + part should appears on kit try but not + part appears on kit tray btu should not be.
/// Usually searchList comes from camera meaning the current existing parts on kit tray. And targetList comes from memorized completed parts
/// wrongLocationParts is a vector of pairs, the first element of each pair is a current existing part with wrong location and
/// the second element of the pair is where the part should be.
/// lostParts should be remedied and redundantParts should be removed from kit tray.
bool
PlanningUtils::findDroppedParts(PartList searchList, PartList targetList, vector<pair<Part, Part>> &wrongLocationParts,
                                PartList &lostParts, PartList &redundantParts) {
    wrongLocationParts.clear();
    lostParts.clear();
    redundantParts.clear();
    PartList notInSearch;
    // part list from camera can contains kit tray, search list should not contains kit tray since it is nto a part
    PartList kit_tray = findPart(searchList, "kit_tray");
    for (auto tray: kit_tray) {
        searchList.erase(findPart(searchList, tray.id)); // remove kit tray from search list
    }
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
            notInSearch.push_back(searching);
        }
    }
    for (auto matching: notInSearch) {
        PartList candidates = findPart(targetList, matching.name);
        if (candidates.empty()) {
            redundantParts.push_back(matching);
            continue;
        }
        auto closest = min_element(candidates.begin(), candidates.end(), [matching](Part A, Part B) {
            return euclideanDistance(matching.pose.pose.position, A.pose.pose.position) <
                   euclideanDistance(matching.pose.pose.position, B.pose.pose.position);
        });
        wrongLocationParts.push_back(pair<Part, Part>(matching, *closest));
        targetList.erase(findPart(targetList, (*closest).id));
    }
    for (auto lost: targetList) {
        lostParts.push_back(lost);
    }
    return !(notInSearch.empty() && targetList.empty());
}