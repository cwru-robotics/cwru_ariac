
#include <SwappingAlgorithm.h>

SwappingAlgorithm::SwappingAlgorithm(ros::NodeHandle nodeHandle) : nh(nodeHandle)
{
	ros::init(argc, argv, "swapping_algorithm");
	
	//Part part_1,part_2,storage;
	//RobotMove robotMove(nh);


	parts_location(part_1,part_2);
	storage_onAGV(storage);



	//save the location of part_1 and part_2
    //geometry_msgs::PoseStamped temp_location1,temp_location2;

    temp_location1.pose.position.x=part_1.pose.position.x;
    temp_location1.pose.position.y=part_1.pose.position.y;
    temp_location1.pose.position.z=part_1.pose.position.z;
    temp_location1.pose.orientation.x=part_1.pose.orientation.x;
    temp_location1.pose.orientation.y=part_1.pose.orientation.y;
    temp_location1.pose.orientation.z=part_1.pose.orientation.z;
    temp_location1.pose.orientation.w=part_1.pose.orientation.w;

    temp_location2.pose.position.x=part_2.pose.position.x;
    temp_location2.pose.position.y=part_2.pose.position.y;
    temp_location2.pose.position.z=part_2.pose.position.z;
    temp_location2.pose.orientation.x=part_2.pose.orientation.x;
    temp_location2.pose.orientation.y=part_2.pose.orientation.y;
    temp_location2.pose.orientation.z=part_2.pose.orientation.z;
    temp_location2.pose.orientation.w=part_2.pose.orientation.w;


    //populate a goal message for manipulation
    robotMove.enableAsync();
    robotMove.move(part_1, storage);


	while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(1).sleep();
    }
    
    robotMove.enableAsync();
    robotMove.move(part_2, temp_location1);

	while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(1).sleep();
    }

    robotMove.enableAsync();
    robotMove.move(storage, temp_location2);

	while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(1).sleep();
    }

	return 0;
}
	


void SwappingAlgorithm::parts_location(Part &part_1, Part &part_2){
	geometry_msgs::PoseStamped [part_1,part_2];

	part_1.header.frame_id="agv1_load_point_frame";
    part_1.pose.orientation.x=0;
    part_1.pose.orientation.y=0;
    part_1.pose.orientation.z=0;
    part_1.pose.orientation.w=1;
    
    part_1.pose.position.x = -0.369; //Translation: [-0.369, -0.597, 0.726]
    part_1.pose.position.y = -0.597;
    part_1.pose.position.z = 0.726;   

	part_2.header.frame_id="agv1_load_point_frame";
    part_2.pose.orientation.x=0;
    part_2.pose.orientation.y=0;
    part_2.pose.orientation.z=0;
    part_2.pose.orientation.w=1;
    
    part_2.pose.position.x = 0.5; //Translation: [-0.369, -0.597, 0.726]
    part_2.pose.position.y = 0.2;
    part_2.pose.position.z = -0.4;
}

void SwappingAlgorithm::storage_onAGV(Part &storage){
	//check the type in position p
	geometry_msgs::PoseStamped storage;

	storage.header.frame_id="agv1_load_point_frame";
    storage.pose.orientation.x=0;
    storage.pose.orientation.y=0;
    storage.pose.orientation.z=0;
    storage.pose.orientation.w=1;
    
    storage.pose.position.x = -0.0; //Translation: [-0.369, -0.597, 0.726]
    storage.pose.position.y = -0.0;
    storage.pose.position.z = 0.0;    
}


