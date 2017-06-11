//
// Created by shipei on 11/1/16.
//

#include "OrderManager.h"

OrderManager::OrderManager(ros::NodeHandle &nodeHandle) : nh(nodeHandle) {
    orderSubscriber = nh.subscribe(
            "/ariac/orders", 10, &OrderManager::orderCallback, this);
    scoreSubscriber = nh.subscribe(
            "/ariac/current_score", 10, &OrderManager::scoreCallback, this);
    competitionStateSubscriber = nh.subscribe(
            "/ariac/competition_state", 10, &OrderManager::competitionStateCallback, this);
    AGV1StateSubscriber = nh.subscribe("/ariac/agv1/state", 10, &OrderManager::AGV1StateCallback, this);
    AGV2StateSubscriber = nh.subscribe("/ariac/agv2/state", 10, &OrderManager::AGV2StateCallback, this);
    AGV1Client =
            nh.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    AGV2Client =
            nh.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
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
    agv1.basePose.pose.position.x = 0.3;
    agv1.basePose.pose.position.y = 3.15;   // try 3.3?
    agv1.basePose.pose.position.z = 0.755;
    agv1.basePose.pose.orientation.x = 0.0;
    agv1.basePose.pose.orientation.y = 0.0;
    agv1.basePose.pose.orientation.z = 1.0;
    agv1.basePose.pose.orientation.w = 0.0;
    agv1.bound = agvBoundBox[0];

    agv2.name = "AGV2";
    agv2.state = AGV::READY;
    agv2.priority = 1.0;
    agv2.basePose.pose.position.x = 0.3;
    agv2.basePose.pose.position.y = -3.15;  // try -3.3?
    agv2.basePose.pose.position.z = 0.755;
    agv2.basePose.pose.orientation.x = 0.0;
    agv2.basePose.pose.orientation.y = 0.0;
    agv2.basePose.pose.orientation.z = 0.0;
    agv2.basePose.pose.orientation.w = 1.0;
    agv2.bound = agvBoundBox[1];

    AGVs.push_back(agv1);
    AGVs.push_back(agv2);

    worldFrame = "world";
    AGVs[0].frameName = "kit_tray_1_frame";
    AGVs[1].frameName = "kit_tray_2_frame";
    AGVFrameBroadcastTimer = nh.createTimer(ros::Duration(0.02), &OrderManager::broadcastTimerCallback, this);
    priorityOrderReceived = false;
    orderCnt = 0;
}

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr &orderMsg) {
    auto it = find_if(orders.begin(), orders.end(), [orderMsg](osrf_gear::Order order) {
        return orderMsg->order_id == order.order_id;
    });
    if (it == orders.end()) {
        orders.push_back(*orderMsg);
        ROS_INFO_STREAM("Received order:\n" << *orderMsg);
    }
    orderCnt++;
    if (orderCnt > 1) {
        priorityOrderReceived = true;
    }
}

void OrderManager::scoreCallback(const std_msgs::Float32::ConstPtr &msg) {
    if (msg->data != score)
    {
        ROS_INFO("New Score: %f", msg->data);
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

void OrderManager::broadcastTimerCallback(const ros::TimerEvent &event) {
    broadcastAGVTF(AGVs[0].frameName, AGVs[0].basePose.pose);
    broadcastAGVTF(AGVs[1].frameName, AGVs[1].basePose.pose);
}

bool OrderManager::startCompetition() {
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient serviceClient =
            nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!serviceClient.exists()) {
        serviceClient.waitForExistence();
    }
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    serviceClient.call(srv);  // Call the start Service.
    return !srv.response.success;
}

bool OrderManager::submitOrder(int agvNumber, osrf_gear::Kit kit) {
    osrf_gear::AGVControl srv;
    srv.request.kit_type = kit.kit_type;
    if (agvNumber == 0) {
        bool result = AGV1Client.call(srv);  // Call the start Service.
        return result && srv.response.success;
    }
    if (agvNumber == 1) {
        bool result = AGV2Client.call(srv);  // Call the start Service.
        return result && srv.response.success;
    }
    return false;
}

Part OrderManager::toAGVPart(int agvNumber, osrf_gear::KitObject object) {
    Part part;
    geometry_msgs::PoseStamped inPose, outPose;
    bool tferr = true;
    inPose.pose = object.pose;
    if (agvNumber == 0) {
        part.location = Part::AGV1;
        inPose.header.frame_id = AGVs[0].frameName;
    } else if (agvNumber == 1) {
        part.location = Part::AGV2;
        inPose.header.frame_id = AGVs[1].frameName;
    }
    else {
        part.location = Part::AGV;
    }

    inPose.header.stamp = ros::Time(0);
    ROS_INFO("Looking for transform between %s and %s", worldFrame.c_str(), inPose.header.frame_id.c_str());
    while (tferr && ros::ok()) {
        tferr = false;
        try {
            inPose.header.stamp = ros::Time(0);
            tf_listener.transformPose(worldFrame, inPose, outPose);
         }        
         catch (tf::TransformException &exception) {
//            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }
    }
    part.traceable = false;
    part.name = object.type;
    part.id = idGenerator.genFakeId();
    part.pose = outPose;
    return part;
}

osrf_gear::KitObject OrderManager::toKitObject(int agvNumber, Part part) {
    osrf_gear::KitObject kit;
    kit.type = part.name;
    geometry_msgs::PoseStamped inPose, outPose;
    bool tferr = true;
    inPose.pose = part.pose.pose;
    inPose.header.frame_id = worldFrame;
    inPose.header.stamp = ros::Time(0);
    while (tferr && ros::ok()) {
        tferr = false;
        try {
            inPose.header.stamp = ros::Time(0);
            if (agvNumber == 0) {
                tf_listener.transformPose(AGVs[0].frameName, inPose, outPose);
            } else {
                tf_listener.transformPose(AGVs[1].frameName, inPose, outPose);
            }
        }
        catch (tf::TransformException &exception) {
//            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }
    }
    kit.pose = outPose.pose;
    return kit;
}

void OrderManager::broadcastAGVTF(string frameName, geometry_msgs::Pose AGVPose) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(AGVPose.position.x, AGVPose.position.y, AGVPose.position.z) );
    tf::Quaternion q;
    q.setValue(AGVPose.orientation.x, AGVPose.orientation.y, AGVPose.orientation.z, AGVPose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frameName));
//    ROS_INFO("publishing %s", frameName.c_str());
}

