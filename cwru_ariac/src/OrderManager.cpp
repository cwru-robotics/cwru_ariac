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
    AGVs[0].name = "AGV1";
    AGVs[0].state = AGV::READY;
    AGVs[0].priority = 2.0;
    AGVs[0].basePose.pose.position.x = 0.12;
    AGVs[0].basePose.pose.position.y = 3.46;
    AGVs[0].basePose.pose.position.z = 0.75;
    AGVs[0].bound = agvBoundBox[0];

    AGVs[1].name = "AGV2";
    AGVs[1].state = AGV::READY;
    AGVs[1].priority = 1.0;
    AGVs[1].bound = agvBoundBox[1];
}

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr &order_msg) {
    if (orders.find(order_msg->order_id) == orders.end()) {
        orders.insert(pair<string, osrf_gear::Order>(order_msg->order_id, *order_msg));
        ROS_INFO_STREAM("Received order:\n" << *order_msg);
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
        AGV1Client.call(srv);  // Call the start Service.
        return !srv.response.success;
    }
    if (agvName == AGVs[1].name) {
        AGV2Client.call(srv);  // Call the start Service.
        return !srv.response.success;
    }
    return false;
}
Part OrderManager::toAGVPart(string agvName, osrf_gear::KitObject object) {

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