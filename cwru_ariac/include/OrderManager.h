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
    unordered_map<string, osrf_gear::Order> orderFinder;

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
    double scoreFunction(double TC, double TT);

private:
    ros::NodeHandle nh_;
    ros::Time startTime;
    ros::Subscriber orderSubscriber;
    ros::Subscriber scoreSubscriber;
    ros::Subscriber competitionStateSubscriber;
    ros::ServiceClient AGV1Client;
    ros::ServiceClient AGV2Client;
    string competitionState;
    double score;
    void orderCallback(const osrf_gear::Order::ConstPtr &order_msg);
    void scoreCallback(const std_msgs::Float32::ConstPtr &msg);
    void competitionStateCallback(const std_msgs::String::ConstPtr &msg);
    string worldFrame;
    string AGV1Frame;
    tf::TransformListener tf_listener;
};


#endif //CWRU_ARIAC_ORDERMANAGER_H
