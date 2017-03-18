//
// Created by tianshipei on 2/16/17.
// Modified wsn 3/15/17

#include <cwru_ariac.h>

string ready_to_deliver_string("ready_to_deliver");
string delivering_string("delivering");
string returning_string("returning");
string g_agv1_state_string("init");
bool g_agv1_ready=false;

void agv1CB(const std_msgs::String& agv1_state) {
 string local_state(agv1_state.data);
 if (local_state.compare(g_agv1_state_string)!=0) {
   ROS_INFO("agv1 state changed to %s",local_state.c_str());
   g_agv1_state_string = local_state; //report state as global string
   //if just returned to ready_to_deliver state, then agv is newly returned
   if (local_state.compare(ready_to_deliver_string)==0) {
     g_agv1_ready=true; 
   }
  }
} 

int main(int argc, char** argv) {
    ros::init(argc, argv, "ariac_qual2");
    ros::NodeHandle nh;
    CameraEstimator camera(nh);
    Agv1CameraEstimator agv1Camera(nh);
    RobotPlanner robotPlanner(nh);
    OrderManager orderManager(nh);
    RobotMove robotMove(nh);
    GlobalPlanner globalPlanner(nh, robotMove);
    robotMove.disableAsync();
    vector <osrf_gear::KitObject> partialKit;
    osrf_gear::KitObject kitObject;
    Part droppedPart;
    int nparts_partial, nparts_seen_on_AGV1;
    PartList onAGV1;
    ros::Subscriber agv1_sub = nh.subscribe("/ariac/agv1/state",1,agv1CB); //agv1 state subscriber
    ROS_INFO("Trying to start the competition");
    while(!orderManager.startCompetition());
    ROS_INFO("Competition started");
    string agvName = orderManager.AGVs[0].name;
    bool moveSuccess=false;
    while (ros::ok() && !orderManager.isCompetitionEnd()) {
        camera.waitForUpdate();
        agv1Camera.waitForUpdate();
        if (orderManager.orders.empty()) {
            ROS_INFO("Got no order, waiting...");
            continue;
        }
        ROS_INFO("Got %d orders", (int)orderManager.orders.size());
        for (auto order: orderManager.orders) {
            ROS_INFO("Working on order id: %s", order.order_id.c_str());
            for (auto kit: order.kits) {
                ROS_INFO("Working on kit type: %s", kit.kit_type.c_str());
		ROS_INFO("size of kit: %d",(int)kit.objects.size());
                partialKit.clear();
                while (!g_agv1_ready) {
                   ROS_WARN("waiting on AGV1");
                   ros::spinOnce();
                   ros::Duration(0.5).sleep();
                }
                for (auto object: kit.objects) {
                    ROS_INFO("Working on object type: %s", object.type.c_str());
                    camera.waitForUpdate();
                    PartList all_bins;
                    int bin_cnt = 1;
                    for (auto bin: camera.onBin) {
                        ROS_INFO("bin %d has %d parts", bin_cnt++, (int)bin.size());
                        for (auto p: bin) {
                            all_bins.push_back(p);
                        }
                    }

                    ROS_INFO("Got %d parts from camera, try to find such part in all bins", (int)all_bins.size());
                    PartList candidates = findPart(all_bins, object.type);
                    ROS_INFO("Found %d parts from bins", (int)candidates.size());
                    while (!candidates.empty()) {
                        Part best =  globalPlanner.getClosestPart(candidates);
                        Part target = orderManager.toAGVPart(agvName, object);
                        candidates.erase(findPart(candidates, best.id));
                        ROS_INFO("got candidate part from total %d candidates:", (int)candidates.size());
                        ROS_INFO_STREAM(best);
                        ROS_INFO("moving part to target:");
                        ROS_INFO_STREAM(target);
                        moveSuccess=false;
                        if (robotMove.move(best, target)) {
                           moveSuccess=true;
                            ROS_INFO("Successfully moved part to %s", agvName.c_str());
                           kitObject.type = object.type;
                           kitObject.pose = target.pose.pose;
                           partialKit.push_back(kitObject);
                           nparts_partial = partialKit.size();
                           ROS_INFO("partial kit has %d part(s): ",nparts_partial);
                           //for (int i=0;i<nparts_partial;i++) {
                           //  ROS_INFO_STREAM(partialKit[i]);
                           //}
                           }
                         //if (!moveSuccess) {
                            camera.waitForUpdate();
                            agv1Camera.waitForUpdate();
                            onAGV1 =agv1Camera.onAGV[0];
                            nparts_seen_on_AGV1 = onAGV1.size();
                            ROS_INFO("AGV1 camera sees %d part(s) on AGV",nparts_seen_on_AGV1);
                            //for (int i=0;i<nparts_seen_on_AGV1;i++) {
                            // ROS_INFO_STREAM(onAGV1[i]);
                            //}
                            ROS_INFO("expect %d parts placed on tray",nparts_partial);
                            if (nparts_seen_on_AGV1>nparts_partial) {
                              ROS_WARN("trying to identify dropped part on AGV1: ");
                              if (agv1Camera.find_dropped_part(partialKit,onAGV1,droppedPart)) {

                                ROS_INFO("identified dropped part; trying to move it");
                                if (robotMove.move(droppedPart, target)) {
                                  ROS_INFO("Successfully moved part to %s", agvName.c_str());
                                  kitObject.type = object.type;
                                  kitObject.pose = target.pose.pose;
                                  partialKit.push_back(kitObject);
                                  nparts_partial = partialKit.size();
                                  ROS_INFO("partial kit has %d part(s): ",nparts_partial);   
                                  moveSuccess=true;    
                                }                    
                                else {
                                  ROS_WARN("could not identify dropped part on AGV1; should not happen!");
                                                    return 0; //DEBUG
                                }
                              }
                            }

                            if (moveSuccess) break;  //TODO: check this logic
                        //}
                        ROS_INFO("Failed to transfer the part, reason: %s", robotMove.getErrorCodeString().c_str());

                        switch (robotMove.getErrorCode()) {
                            case RobotMoveResult::NO_ERROR:
                               ROS_INFO("robot move returned NO_ERROR");
                               break;
                            case RobotMoveResult::PART_DROPPED:
                                //ROS_WARN("Continue with a new part? (hit enter)");
				ROS_WARN("trying a new part...");
                                //getchar();
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
                                ROS_INFO("unreachable; try next part");
                                break;
                            case RobotMoveResult::WRONG_PARAMETER:
                                ROS_WARN("Wrong parameter! going to quit");
                                return 0;
                            default:
                                break;
                        }
                        camera.waitForUpdate();
                        agv1Camera.waitForUpdate();
                    }
                    if (candidates.size() != 0) {
                        ROS_INFO("Completed one object: %s in kit %s, order %s", object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                    } else {
                        ROS_WARN("Failed all candidate parts to complete object: %s in kit %s, order %s",
                                 object.type.c_str(), kit.kit_type.c_str(), order.order_id.c_str());
                        ROS_WARN("Ignore this object and continue? (hit enter)");
                        getchar();
                    }
                }
                ROS_INFO("completed one kit: %s in order %s", kit.kit_type.c_str(), order.order_id.c_str());
                ROS_INFO("Submitting order...");
                bool order_result = orderManager.submitOrder(agvName, kit);
                ROS_INFO("Submission %s", order_result? "success":"failed");
                ros::spinOnce();
                g_agv1_ready=false; // need to wait on AGV1
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
