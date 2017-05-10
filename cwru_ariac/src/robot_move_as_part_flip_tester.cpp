//test module for part-flip function
// talks directly with robot_move_as
//start up qual3; hard-code pose of a large pulley to flip

#include <RobotMove.h>
int g_select_pulley=0;

//Part g_pick_part,g_place_part;
void set_part_vals(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    pick_pose.header.frame_id="world";
    pick_pose.pose.orientation.x=0;
    pick_pose.pose.orientation.y=0;
    pick_pose.pose.orientation.z=0;
    pick_pose.pose.orientation.w=1;
    
    pick_pose.pose.position.x = -0.45; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 1.145;
    pick_pose.pose.position.z = 0.7255;    
    
    place_pose = pick_pose; //use same frame and orientation; change position  
    place_pose.pose.position.x = 0.26;
    place_pose.pose.position.y = 3.18;
    place_pose.pose.position.z = 0.755;

    
    pick_part.name="pulley_part";
    pick_part.location= Part::BIN8;
    pick_part.pose = pick_pose;
    place_part = pick_part;
    place_part.location = Part::AGV1;
    place_part.pose = place_pose;
}

void set_part_vals_inverted(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    pick_pose.header.frame_id="world";
    pick_pose.pose.orientation.x=1;
    pick_pose.pose.orientation.y=0;
    pick_pose.pose.orientation.z=0;
    pick_pose.pose.orientation.w=0;
    
    pick_pose.pose.position.x = -0.45; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 1.145;
    pick_pose.pose.position.z = 0.7255;    
    
    place_pose = pick_pose; //use same frame and orientation; change position  
    place_pose.pose.position.x = 0.26;
    place_pose.pose.position.y = 3.18;
    place_pose.pose.position.z = 0.755;

    
    pick_part.name="pulley_part";
    pick_part.location= Part::BIN8;
    pick_part.pose = pick_pose;
    place_part = pick_part;
    place_part.location = Part::AGV1;
    place_part.pose = place_pose;
}

void set_part_vals_flip(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    ROS_INFO("enter pulley choice, 0 through 3: ");
    std::cin>>g_select_pulley;
    pick_pose.header.frame_id="world";
    pick_pose.pose.orientation.x=0;
    pick_pose.pose.orientation.y=0;
    pick_pose.pose.orientation.z=0;
    pick_pose.pose.orientation.w=1;
    
    pick_pose.pose.position.x = -0.45; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 1.145;
    pick_pose.pose.position.z = 0.7255;   

    if (g_select_pulley==1) { pick_pose.pose.position.y = 0.845; }
    if (g_select_pulley==2) { pick_pose.pose.position.x = -0.15; }
    if (g_select_pulley==3) { pick_pose.pose.position.x = -0.15;
      pick_pose.pose.position.y = 0.845;
    }
    //(-.45, 0.845); (-0.15,0.845); (-0.15;1.145)
    
    place_pose = pick_pose; //use same frame and orientation; change position  
    place_pose.pose.position.x = 0.26;
    place_pose.pose.position.y = 3.18;
    place_pose.pose.position.z = 0.755;
    place_pose.pose.orientation.x=1;
    place_pose.pose.orientation.y=0;
    place_pose.pose.orientation.z=0;
    place_pose.pose.orientation.w=0;
    
    pick_part.name="pulley_part";
    pick_part.location= Part::BIN8;
    pick_part.pose = pick_pose;
    place_part = pick_part;
    place_part.location = Part::AGV1;
    place_part.pose = place_pose;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "part_flip_tester"); //name this node
    ros::NodeHandle nh;
    Part pick_part,place_part;

    RobotMove robotMove(nh);

    //test: set source and target poses, up/up, dn/dn, up/dn choices
    //set_part_vals(pick_part,place_part); //populate with test vals
    //set_part_vals_inverted(pick_part,place_part);
    set_part_vals_flip(pick_part,place_part);
    //populate a goal message for manipulation
    robotMove.enableAsync();
    //robotMove.flipPart(pick_part);
    robotMove.move(pick_part,place_part);
    while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(0.3).sleep();
    }
 
    //robotMove.grasp();
    //robotMove.release();

    //robotMove.toCruisePose(0.0);
    //robotMove.toAgv1HoverPose(0.0);
    //robotMove.toPredefinedPose(RobotMoveGoal::AGV1_HOVER_POSE);
    //robotMove.toPredefinedPose(RobotMoveGoal::BIN8_CRUISE_POSE);
}
