//
// Created by shipei on 11/8/16.
//

#include <AriacBase.h>
#include <OrderManager.h>
#include <RobotPlanner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_and_place_demo");
    ros::NodeHandle nh;                                 // standard ros node handle
    RobotPlanner robotArm(nh);
    OrderManager comp(nh);
    vector<double> my_pose = robotArm.getJointsState();

    comp.startCompetition();
    ROS_INFO("Competition started!");

    ros::Duration(20).sleep();

    while (ros::ok()) {
        my_pose[0] = 1.288514;
        my_pose[1] = 1.962313;
        my_pose[2] = -0.623798;
        my_pose[3] = 0.134005;
        my_pose[4] = 4.051464;
        my_pose[5] = -1.527353;
        my_pose[6] = -3.179198;
        robotArm.sendJointsValue(my_pose);

        robotArm.grab();
        robotArm.waitForGripperAttach(0);

        my_pose[0] = 0.988514;
        my_pose[1] = 1.962313;
        my_pose[2] = -1.023798;
        my_pose[3] = 0.134005;
        my_pose[4] = 4.051464;
        my_pose[5] = -1.527353;
        my_pose[6] = -3.179198;
        robotArm.sendJointsValue(my_pose);

        my_pose[0] = 0.598882;
        my_pose[1] = 2.100598;
        my_pose[2] = -0.784037;
        my_pose[3] = 1.387977;
        my_pose[4] = 4.017667;
        my_pose[5] = -1.592512;
        my_pose[6] = -2.217604;
        robotArm.sendJointsValue(my_pose);

        my_pose[0] = 1.029344;
        my_pose[1] = 2.100731;
        my_pose[2] = -0.379199;
        my_pose[3] = 1.397113;
        my_pose[4] = 3.943804;
        my_pose[5] = -1.601538;
        my_pose[6] = -1.717464;
        robotArm.sendJointsValue(my_pose);
        ros::Duration(1.0).sleep();
        robotArm.release();

        my_pose[0] = 0.598882;
        my_pose[1] = 2.100598;
        my_pose[2] = -0.784037;
        my_pose[3] = 1.387977;
        my_pose[4] = 4.017667;
        my_pose[5] = -1.592512;
        my_pose[6] = -2.217604;
        robotArm.sendJointsValue(my_pose);
        ros::Duration(1.0).sleep();

        comp.submitOrder(comp.AGVs[0].name, comp.orders[comp.orders.begin()->first].kits[0]);
        ros::Duration(21.0).sleep();
    }


    return 0;
}