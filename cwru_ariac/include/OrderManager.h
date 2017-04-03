//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_ORDERMANAGER_H
#define CWRU_ARIAC_ORDERMANAGER_H

#include <AriacBase.h>

class OrderManager: public AriacBase {
public:
    OrderManager(ros::NodeHandle nodeHandle);

    vector<AGV> AGVs;
    vector<osrf_gear::Order> orders;

    Part toAGVPart(string agvName, osrf_gear::KitObject object);
    bool startCompetition();
    bool submitOrder(string agvName, osrf_gear::Kit kit);

    bool isCompetitionEnd() {
        ros::spinOnce();
        return (competitionState == "done");
    }
    double getCurrentScore() {
        ros::spinOnce();
        return score;
    }
    bool isAGVReady(int agvNumber) {
        return AGVs[agvNumber].state == AGV::READY;
    }

private:
    ros::NodeHandle nh_;
    ros::Time startTime;
    ros::Subscriber orderSubscriber;
    ros::Subscriber scoreSubscriber;
    ros::Subscriber competitionStateSubscriber;
    ros::Subscriber AGV1StateSubscriber;
    ros::Subscriber AGV2StateSubscriber;
    ros::ServiceClient AGV1Client;
    ros::ServiceClient AGV2Client;
    string competitionState;
    double score;
    void orderCallback(const osrf_gear::Order::ConstPtr &order_msg);
    void scoreCallback(const std_msgs::Float32::ConstPtr &msg);
    void competitionStateCallback(const std_msgs::String::ConstPtr &msg);
    void AGV1StateCallback(const std_msgs::String &state);
    void AGV2StateCallback(const std_msgs::String &state);
    string worldFrame;
    string AGV1Frame;
    tf::TransformListener tf_listener;
    tf::Transform tfWorldToTray1_;
    tf::StampedTransform stfWorldToTray1_,stfTray1ToPart_,stfWorldToPart_;
    tf::StampedTransform stfStaticWorldToTray1_;
    geometry_msgs::PoseStamped stpTray1_;
    string ready_to_deliver_string;
    string delivering_string;
    string returning_string;
    string g_agv1_state_string;
    int assignedID;
    XformUtils xform_utils_; //instantiate an object of XformUtils
};


#endif //CWRU_ARIAC_ORDERMANAGER_H
