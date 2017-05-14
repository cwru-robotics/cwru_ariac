//test module for sending pick command

#include <RobotMove.h>

void set_part_vals(Part &pick_part) {
    //can use these values to grab a pulley from bin8, per qual3b
    geometry_msgs::PoseStamped pick_pose;
    pick_pose.header.frame_id="world";
    pick_pose.pose.orientation.x=0;
    pick_pose.pose.orientation.y=0;
    pick_pose.pose.orientation.z=0;
    pick_pose.pose.orientation.w=1;
    
    pick_pose.pose.position.x = -0.45; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 1.145;
    pick_pose.pose.position.z = 0.74; //0.7255;    //can set deliberately offset coords to see reaction
    
    pick_part.name="pulley_part";
    pick_part.location= Part::BIN8;
    pick_part.pose = pick_pose;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_tester"); //name this node
    ros::NodeHandle nh;
    Part pick_part;
    RobotMove robotMove(nh);
    set_part_vals(pick_part); //populate with test vals
    
    //populate a goal message for manipulation
    robotMove.enableAsync();
    robotMove.pick(pick_part);

    while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(1).sleep();
    }
 
    ROS_INFO("releasing part");
    robotMove.release();
    //robotMove.grasp();
    //robotMove.release();

    //robotMove.toCruisePose(0.0);
    //robotMove.toAgv1HoverPose(0.0);
    //robotMove.toPredefinedPose(RobotMoveGoal::AGV1_HOVER_POSE);
    //robotMove.toPredefinedPose(RobotMoveGoal::BIN8_CRUISE_POSE);
}
