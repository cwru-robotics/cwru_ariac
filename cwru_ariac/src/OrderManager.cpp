//
// Created by shipei on 11/1/16.
//

#include "OrderManager.h"

OrderManager::OrderManager(ros::NodeHandle nodeHandle): nh_(nodeHandle){
    orderSubscriber = nh_.subscribe(
            "/ariac/orders", 10, &OrderManager::orderCallback, this);
    scoreSubscriber = nh_.subscribe(
            "/ariac/current_score", 10, &OrderManager::scoreCallback, this);
    competitionStateSubscriber = nh_.subscribe(
            "/ariac/competition_state", 10, &OrderManager::competitionStateCallback, this);
    AGV1StateSubscriber = nh_.subscribe("/ariac/agv1/state", 10, &OrderManager::AGV1StateCallback, this);
    AGV2StateSubscriber = nh_.subscribe("/ariac/agv1/state", 10, &OrderManager::AGV2StateCallback, this);
    AGV1Client =
            nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    AGV2Client =
            nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
    if (!AGV1Client.exists()) {
        AGV1Client.waitForExistence();
    }
    if (!AGV2Client.exists()) {
        AGV2Client.waitForExistence();
    }
    AGV agv1, agv2;
    agv1.name = "AGV1";
    agv1.state = AGV::READY;
    agv1.priority = 2.0;
    agv1.basePose.pose.position.x = 0.12;
    agv1.basePose.pose.position.y = 3.46;
    agv1.basePose.pose.position.z = 0.75;
    agv1.bound = agvBoundBox[0];

    agv2.name = "AGV2";
    agv2.state = AGV::READY;
    agv2.priority = 1.0;
    agv2.bound = agvBoundBox[1];

    AGVs.push_back(agv1);
    AGVs.push_back(agv2);

    worldFrame = "/world";
//    AGV1Frame = "/agv1_load_point_frame";
    AGV1Frame = "/kit_tray_1_frame";
    ready_to_deliver_string = "ready_to_deliver";
    delivering_string = "delivering";
    returning_string = "returning";
    g_agv1_state_string = "init";
    assignedID = 10000;
}

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr &orderMsg) {
    if (orderFinder.find(orderMsg->order_id) == orderFinder.end()) {
        orderFinder.insert(pair<string, osrf_gear::Order>(orderMsg->order_id, *orderMsg));
        orders.push_back(*orderMsg);
        ROS_INFO_STREAM("Received order:\n" << *orderMsg);
    }
}

void OrderManager::scoreCallback(const std_msgs::Float32::ConstPtr &msg) {
    if (msg->data != score)
    {
        ROS_INFO("Score: %f", msg->data);
    }
    score = msg->data;
}

void OrderManager::competitionStateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "done" && competitionState != "done")
    {
        ROS_INFO("Competition ended.");
    }
    competitionState = msg->data;
}

void OrderManager::AGV1StateCallback(const std_msgs::String &state) {
    if (state.data == "init") {
        AGVs[0].state = AGV::INIT;
    } else if (state.data == "ready_to_deliver") {
        AGVs[0].state = AGV::READY;
    } else if (state.data == "delivering") {
        AGVs[0].state = AGV::DELIVERING;
    } else if (state.data == "returning") {
        AGVs[0].state = AGV::RETURNING;
    }
}

void OrderManager::AGV2StateCallback(const std_msgs::String &state) {
    if (state.data == "init") {
        AGVs[1].state = AGV::INIT;
    } else if (state.data == "ready_to_deliver") {
        AGVs[1].state = AGV::READY;
    } else if (state.data == "delivering") {
        AGVs[1].state = AGV::DELIVERING;
    } else if (state.data == "returning") {
        AGVs[1].state = AGV::RETURNING;
    }
}

bool OrderManager::startCompetition() {
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient serviceClient =
            nh_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!serviceClient.exists()) {
        serviceClient.waitForExistence();
    }
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    serviceClient.call(srv);  // Call the start Service.
    return !srv.response.success;
}

bool OrderManager::submitOrder(string agvName, osrf_gear::Kit kit) {
    osrf_gear::AGVControl srv;
    srv.request.kit_type = kit.kit_type;
    if (agvName == AGVs[0].name) {
        bool result = AGV1Client.call(srv);  // Call the start Service.
        return result && srv.response.success;
    }
    if (agvName == AGVs[1].name) {
        bool result = AGV2Client.call(srv);  // Call the start Service.
        return result && srv.response.success;
    }
    return false;
}
Part OrderManager::toAGVPart(string agvName, osrf_gear::KitObject object) {
    Part part;
    geometry_msgs::PoseStamped inPose;
    geometry_msgs::PoseStamped outPose;
    bool tferr = true;
    inPose.pose = object.pose;
    if (agvName == AGVs[0].name) {
        part.location = Part::AGV1;
        inPose.header.frame_id = AGV1Frame;
    }
    else if(agvName == AGVs[1].name) {
        part.location = Part::AGV2;
    }
    else {
        part.location = Part::AGV;
    }
    ROS_INFO("Looking for transform between %s and %s", worldFrame.c_str(), inPose.header.frame_id.c_str());
    while (tferr && ros::ok()) {
        tferr = false;
        try {
            inPose.header.stamp = ros::Time(0);
            tf_listener.transformPose(worldFrame, inPose, outPose);
            if (agvName == AGVs[0].name) {
                if (checkBound(outPose.pose.position, AGVs[0].bound)) {
                    ROS_INFO("target pose passes agv-bound test:");
                } else {
                    ROS_WARN("got xform, but target pose fails agv-bound test");
                    tferr = true;
                    ros::Duration(0.2).sleep();
                    ros::spinOnce();
                }
            }
            else if(agvName == AGVs[1].name) {
                if (checkBound(outPose.pose.position, AGVs[1].bound)) {
                    ROS_INFO("target pose passes agv-bound test:");
                } else {
                    ROS_WARN("got xform, but target pose fails agv-bound test");
                    tferr = true;
                    ros::Duration(0.2).sleep();
                    ros::spinOnce();
                }
            }
         }        
         catch (tf::TransformException &exception) {
//            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }
    }
    ROS_INFO("Got transform");
    part.pose = outPose;
    part.traceable = false;
    part.name = object.type;
    part.id = assignedID++;
    return part;
}
double OrderManager::scoreFunction(double TC, double TT) {
    //double TC = 1;
    double AC = 1;
    //double TT;
    double AT = 1;
    double CS = getCurrentScore();
    double CF = AC/TC;
    double EF = AT/TT;
    return CF*CS+EF*CS;
}
