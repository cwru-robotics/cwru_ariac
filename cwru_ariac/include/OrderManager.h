//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_ORDERMANAGER_H
#define CWRU_ARIAC_ORDERMANAGER_H

#include <AriacBase.h>

class OrderManager: public AriacBase {
public:
    OrderManager(ros::NodeHandle &nodeHandle);

    vector<AGV> AGVs;
    vector<osrf_gear::Order> orders;

    bool priorityOrderReceived;

    Part toAGVPart(int agvNumber, osrf_gear::KitObject object);

    osrf_gear::KitObject toKitObject(int agvNumber, Part part);

    bool startCompetition();

    bool submitOrder(int agvNumber, osrf_gear::Kit kit);

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

protected:
    ros::NodeHandle nh;
    ros::Time startTime;
    ros::Subscriber orderSubscriber;
    ros::Subscriber scoreSubscriber;
    ros::Subscriber competitionStateSubscriber;
    ros::Subscriber AGV1StateSubscriber;
    ros::Subscriber AGV2StateSubscriber;
    ros::ServiceClient AGV1Client;
    ros::ServiceClient AGV2Client;
    IDGenerator idGenerator;
    string competitionState;
    double score;
    void orderCallback(const osrf_gear::Order::ConstPtr &order_msg);
    void scoreCallback(const std_msgs::Float32::ConstPtr &msg);
    void competitionStateCallback(const std_msgs::String::ConstPtr &msg);
    void AGV1StateCallback(const std_msgs::String &state);
    void AGV2StateCallback(const std_msgs::String &state);
    void broadcastAGVTF(string frameName, geometry_msgs::Pose AGVPose);
    void broadcastTimerCallback(const ros::TimerEvent &event);
    string worldFrame;
    tf::TransformListener tf_listener;
    ros::Timer AGVFrameBroadcastTimer;
    int orderCnt;
};


#endif //CWRU_ARIAC_ORDERMANAGER_H
