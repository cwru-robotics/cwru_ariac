//
// Created by tianshipei on 12/3/16.
//

#include <BinManager.h>
#include <CameraEstimator.h>
#include <Cheater.h>
#include <OrderManager.h>
#include <OraclePlanner.h>
#include <RobotPlanner.h>
#include <RobotMove.h>

enum State {NONE = 0, INIT, END, FILL_ORDER, WAIT};

int main(int argc, char** argv) {
    ros::init(argc, argv, "factory_planner");
    ros::NodeHandle nh;
    CameraEstimator camera(nh);
    RobotMove robot(nh);
    OraclePlanner oracle(nh);
    OrderManager orderManager(nh);
    State state = INIT;
    vector<tuple<Part, string, osrf_gear::KitObject>> available;
    while (ros::ok()) {
        camera.waitForUpdate();
        available.clear();
        switch (state) {
            case INIT:
                orderManager.startCompetition();
                state = WAIT;
                break;
            case END:
                ROS_INFO("End Competition!");
                return 0;
            case FILL_ORDER:
                state = orderManager.isCompetitionEnd()? END:state;
                if (orderManager.AGVs[0].state == AGV::READY || orderManager.AGVs[1].state == AGV::READY) {
                    if (available.size() != 0) {
                        RobotState robotState;
                        while (!robot.getRobotState(robotState));
                        double planningTime, executingTime;
                        if (oracle.move(get<0>(*available.begin()), orderManager.toAGVPart(get<1>(*available.begin()), get<2>(*available.begin())), robotState, planningTime, executingTime)) {
                            robot.move(get<0>(*available.begin()), orderManager.toAGVPart(get<1>(*available.begin()), get<2>(*available.begin())), executingTime);
                            state = WAIT;
                        }
                    }
                }
                break;
            case WAIT:
                state = orderManager.isCompetitionEnd()? END:state;
                if (orderManager.AGVs[0].state == AGV::READY || orderManager.AGVs[1].state == AGV::READY) {
                    RobotState robotState;
                    while (!robot.getRobotState(robotState));
                    for(auto order: orderManager.orders) {
                        for(auto kit: order.second.kits) {
                            for (auto object: kit.objects) {
                                string required = object.type;
                                PartList conveyorParts = findPart(camera.onConveyor, required);
                                PartList binParts;
                                for(auto bin: camera.onBin){
                                    PartList temp = findPart(bin, required);
                                    for (auto item: temp) {
                                        binParts.push_back(item);
                                    }
                                }
                                if (conveyorParts.size()!=0 || binParts.size() != 0) {
                                    for (auto agv: orderManager.AGVs) {
                                        if (agv.state != AGV::READY)
                                            continue;
                                        for (auto cPart: conveyorParts) {
                                            double planningTime, executingTime;
                                            if (oracle.move(cPart, orderManager.toAGVPart(agv.name, object), robotState, planningTime, executingTime)) {
                                                cPart.priority = (10 / executingTime) * 3 + 5;
                                                available.push_back(tuple<Part, string, osrf_gear::KitObject>(cPart, agv.name, object));
                                            }
                                        }
                                        for (auto bPart: binParts) {
                                            double planningTime, executingTime;
                                            if (oracle.move(bPart, orderManager.toAGVPart(agv.name, object), robotState, planningTime, executingTime)) {
                                                bPart.priority = (10 / executingTime) * 1 + 5;
                                                available.push_back(tuple<Part, string, osrf_gear::KitObject>(bPart, agv.name, object));
                                            }
                                        }
                                    }
                                    sort(available.begin(), available.end(), [](tuple<Part, string, osrf_gear::KitObject> a, tuple<Part, string, osrf_gear::KitObject> b){
                                        return get<0>(a).priority > get<0>(b).priority;});
                                    state = FILL_ORDER;
                                    break;
                                }
                            }
                            if (state == FILL_ORDER)
                                break;
                        }
                        if (state == FILL_ORDER)
                            break;
                    }
                }
                break;
            default:
                ROS_INFO("State machine error!");
                state = INIT;
        }
    }
    return 0;
}

