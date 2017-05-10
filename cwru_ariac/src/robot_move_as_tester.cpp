//test module for sending pick/place commands

#include <RobotMove.h>

//Part g_pick_part,g_place_part;
void set_part_vals(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    pick_pose.header.frame_id="world";
    pick_pose.pose.orientation.x=0;
    pick_pose.pose.orientation.y=0;
    //yaw = 0.785; 
//--> q.z = 0.3825
    //q.w = 0.924
    pick_pose.pose.orientation.z=0.3825;
    pick_pose.pose.orientation.w=0.924;
    
    pick_pose.pose.position.x = -0.2; //-0.369; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 0.33; //-0.597;
    pick_pose.pose.position.z = 0.725; //0.726;    
    
    place_pose = pick_pose; //use same frame and orientation; change position
    place_pose.header.frame_id= "world"; //agv1_load_point_frame";
    place_pose.pose.position.x = 0.23; //0;//0.300; //AGV1 frame: Translation: [0.300, 3.300, 0.750]
    place_pose.pose.position.y = 3.13; //0; //3.300;
    place_pose.pose.position.z = 0.754; //0; //0.750;    
    
    pick_part.name="gasket_part"; //gear_part";
    pick_part.location= Part::BIN7; //BIN6;
    pick_part.pose = pick_pose;
    place_part = pick_part;
    place_part.pose = place_pose;
    place_part.location= Part::AGV1;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_tester"); //name this node
    ros::NodeHandle nh;
    Part pick_part,place_part;
    RobotMove robotMove(nh);
    set_part_vals(pick_part,place_part); //populate with test vals
    
    //populate a goal message for manipulation
    robotMove.enableAsync();
    robotMove.move(pick_part, place_part, 10.0);

    while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(1).sleep();
    }
 
    //robotMove.grasp();
    //robotMove.release();

    //robotMove.toCruisePose(0.0);
    //robotMove.toAgv1HoverPose(0.0);
    //robotMove.toPredefinedPose(RobotMoveGoal::AGV1_HOVER_POSE);
    //robotMove.toPredefinedPose(RobotMoveGoal::BIN8_CRUISE_POSE);
}
