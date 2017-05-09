//
// Created by tianshipei on 4/25/17.
//

#include "TrayClearer.h"

TrayClearer::TrayClearer(ros::NodeHandle nodeHandle, RobotMove robotMove, BinManager binManager): nh(nodeHandle) {
    robotMovePtr.allocate_shared(robotMove);
    binManagerPtr.allocate_shared(binManager);

}

bool TrayClearer::clearPartsFromTray(PartList partsToClear) {
    ROS_INFO("tray clear request received");

    //get the data from the request in a more useful form
    std::vector<string> part_names_vec(request.part_names, request.part_names + sizeof request.part_names / sizeof request.part_names[0]);

    std::vector<geometry_msgs/PoseStamped> initial_poses_vec(request.initial_poses, request.initial_poses + sizeof request.initial_poses / sizeof request.initial_poses[0]);

    //declarations to be used in the following while loop
    string next_part_name;
    geometry_msgs/PoseStamped next_initial_location;
    geometry_msgs/PoseStamped next_final_destination;
    Part pick_part, place_part;//for populating a move command

    while ((!part_names_vec.empty())&&(!initial_poses_vec.empty())){
        //take out the next part and desired pose, then remove from the list
        next_part_name = part_names_vec[part_names_vec.size - 1];
        next_initial_location = initial_poses_vec[initial_poses_vec.size - 1];
        part_names_vec.pop_back;
        initial_poses_vec.pop_back;

        //get the destination pose for this part from Bruce
        storageFinderMsg.request.part_name = next_part_name;

        if(client.call(storageFinderMsg)){
            //executes if a location was found
            next_final_destination = storageFinderMsg.response.poses[0];

            //populate a part data type with the info needed
            pick_part.name = next_part_name;
            place_part.name = next_part_name;
            pick_part.pose = next_initial_location;
            place_part.pose = next_final_destination;


            //TODO: ask shipei how to populate the location portion or why it's needed
            pick_part.location = Part::AGV1;
            place_part.location = Part::Bin6;

            //order the motion
//            robotMove.enableAsync();//TODO: is this needed?


            robotMovePtr->disableAsync();
            for (auto pick_part: partsToClear) {
                if (binManagerPtr->advisedLocationForPut(pick_part.name, place_part)) {
                    if (robotMovePtr->move(pick_part, place_part)) {
                        ROS_INFO("move success");
                    }
                }
            }

//
//            //wait for the motion to finish
//            while(!robotMove.actionFinished()){
//                ROS_INFO("Waiting for result");
//                ros::Duration(1).sleep();
//            }

        }else{
            ROS_WARN("failed to find location for a part");
        }

    }
}