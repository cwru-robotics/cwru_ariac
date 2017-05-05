//this fnc should flip a part (and return its new pose)
// focus on pulley_part; 

Eigen::Matrix3d compute_roty(double theta_y) {
   Eigen::Matrix3d Roty;
   Eigen::Vector3d roty_x,roty_y,roty_z;

            roty_x<<cos(theta_y),0,-sin(theta_y);
            roty_y<<0,1,0;
            roty_z<<sin(theta_y),0,cos(theta_y);
            Roty.col(0) = roty_x;
            Roty.col(1) = roty_y;
            Roty.col(2) = roty_z;
    return Roty;
}

Eigen::Matrix3d compute_rotz(double theta_z) {
   Eigen::Matrix3d Rotz;
   Eigen::Vector3d rotz_x,rotz_y,rotz_z;

            rotz_x<<cos(theta_z),sin(theta_z),0;
            rotz_y<< -sin(theta_z),cos(theta_z),0;
            rotz_z<< 0,0,1;
            Rotz.col(0) = rotz_x;
            Rotz.col(1) = rotz_y;
            Rotz.col(2) = rotz_z;
    return Rotz;
}

Eigen::Matrix3d compute_rotx(double theta_x) {
   Eigen::Matrix3d Rotx;
   Eigen::Vector3d rotx_x,rotx_y,rotx_z;

            rotx_x<<1,0,0; //cos(theta_z),sin(theta_z),0;
            rotx_y<< 0, cos(theta_x),sin(theta_x);//-sin(theta_z),cos(theta_z),0;
            rotx_z<< 0,-sin(theta_x),cos(theta_x);//0,1;
            Rotx.col(0) = rotx_x;
            Rotx.col(1) = rotx_y;
            Rotx.col(2) = rotx_z;
    return Rotx;
}

unsigned short int RobotMoveActionServer:: flip_part_fnc(const cwru_ariac::RobotMoveGoalConstPtr& goal) {
  unsigned short int errorCode = RobotMoveResult::CANCELLED; //change to NO_ERROR, if ultimately successful

    ROS_INFO("flip_part function received goal type: %d", goal->type);
    cwru_ariac::Part part = goal->sourcePart;
            ROS_INFO("The part is %s, should be fetched from location code %s ", part.name.c_str(),
               placeFinder[part.location].c_str());
            ROS_INFO("part info: ");
            ROS_INFO_STREAM(part);
    if (part.name.compare("pulley_part") == 0) {
       //compute poses to pick up pulley:
            ROS_INFO("attempting to flip pulley");
            if (!bin_hover_jspace_pose(goal->sourcePart.location, bin_hover_jspace_pose_)) {
                ROS_WARN("bin_hover_jspace_pose() failed for source bin %d", (int) goal->sourcePart.location);
                return errorCode;
            }
            ROS_INFO_STREAM("bin_hover: " << bin_hover_jspace_pose_.transpose());
            Eigen::VectorXd q6dof_hover, q6dof_dropoff;
            q6dof_hover = fwd_solver_.map726dof(bin_hover_jspace_pose_); //convert to 6dof
            Eigen::Affine3d affine_gripper_hover, affine_gripper_tilted, affine_gripper_tilted2,affine_gripper_tilted3,affine_gripper_dropoff;
            Eigen::Affine3d affine_gripper_dropoff_depart, affine_gripper_dropff_depart2, affine_gripper_depart3;
            Eigen::Affine3d affine_gripper_regrasp_approach, affine_gripper_regrasp;
            affine_gripper_hover = fwd_solver_.fwd_kin_solve(q6dof_hover); // given vector of q angles, compute fwd kin
            ROS_INFO("hover gripper pose w/rt robot base: ");
            xformUtils_.printAffine(affine_gripper_hover);
            Eigen::Matrix3d R_hover, Roty, Rotz, R_tilted, R_tilted2;
            double theta_y = -M_PI/2.0; //3.0*M_PI/4.0;
            Roty = compute_roty(theta_y);
            double theta_z = -M_PI/2.0;
            Rotz = compute_rotz(theta_z);
 
            // rotate gripper by roty(theta_y):
            R_hover = affine_gripper_hover.linear();
            R_tilted = Roty*R_hover;
            R_tilted2 = Rotz*R_tilted;
            affine_gripper_tilted = affine_gripper_hover;
            affine_gripper_tilted.linear() = R_tilted;
            affine_gripper_tilted2 = affine_gripper_tilted;
            affine_gripper_tilted2.linear() = R_tilted2;
            affine_gripper_tilted3=affine_gripper_tilted2;
            //affine_gripper_tilted3.translation()= 
            Eigen::Vector3d O_hover,O_drop;
            //O_drop = 
            O_hover = affine_gripper_tilted.translation();
            Eigen::VectorXd gripper_tilted_jspace,gripper_tilted_jspace2,gripper_tilted_jspace3;
            Eigen::VectorXd gripper_regrasp_jspace;
            if (!get_pickup_IK(affine_gripper_tilted, bin_hover_jspace_pose_, gripper_tilted_jspace)) {
                ROS_WARN("could not compute IK soln for tilted pose!");
                return errorCode;
            } 

            if (!get_pickup_IK(affine_gripper_tilted2, gripper_tilted_jspace, gripper_tilted_jspace2)) {
                ROS_WARN("could not compute IK soln for tilted pose2!");
                return errorCode;
            } 


            //cruise pose, adjacent to bin:
            if (!bin_cruise_jspace_pose(goal->sourcePart.location, goal->targetPart.location,
                                        bin_cruise_jspace_pose_)) {
                ROS_WARN("bin_cruise_jspace_pose() failed");
                return errorCode;
            }
            //cruise pose, adjacent to bin, pointing towards target agv
            ROS_INFO_STREAM("bin_cruise_jspace_pose_: " << bin_cruise_jspace_pose_.transpose());

            //what is the gripper cartesian pose at this hover position?
            //Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec);


            //compute the IK for this pickup pose: pickup_jspace_pose_
            //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
            //need to provide the Part info and the rail displacement
            //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
            affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(goal->sourcePart,
                                                                                               bin_hover_jspace_pose_[1]);
            Eigen::Vector3d O_pickup;
            O_pickup = affine_vacuum_pickup_pose_wrt_base_link_.translation();
            ROS_INFO_STREAM("O_pickup: "<<O_pickup);
            //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
            //note: may need to go to approach pose first; default motion is in joint space
            //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
            if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, pickup_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for pickup pose!");
                result_.errorCode = RobotMoveResult::UNREACHABLE;
                return errorCode;
            } 
            ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());
            //compute approach_pickup_jspace_pose_
            //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
            if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                                     approach_pickup_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for pickup approach pose!");
                result_.errorCode = RobotMoveResult::UNREACHABLE;
                return errorCode;
            }

            affine_gripper_dropoff = affine_gripper_tilted2;
            Eigen::Vector3d O_dropoff,O_regrasp;

            O_dropoff = O_pickup; //affine_vacuum_pickup_pose_wrt_base_link_.translation();
            O_dropoff[2]+=0.04; //elevation higher than pickup pose
            O_dropoff[1]+=0.2;  //should not need this...
            affine_gripper_dropoff.translation() = O_dropoff;
            ROS_INFO_STREAM("O_dropoff: "<<O_dropoff);
            O_regrasp = O_dropoff;
            O_dropoff[1]-=PULLEY_PART_THICKNESS;
            Eigen::Matrix3d R_regrasp;
            R_regrasp = compute_rotx(M_PI)*R_tilted2;
            affine_gripper_regrasp.linear() = R_regrasp;
            affine_gripper_regrasp.translation() = O_regrasp;

            if (!get_pickup_IK(affine_gripper_dropoff, bin_hover_jspace_pose_, gripper_tilted_jspace3)) {
                ROS_WARN("could not compute IK soln for dropoff pose!");
                result_.errorCode = RobotMoveResult::UNREACHABLE;
                return errorCode;
            }
            q_bin_pulley_flip_[1] = bin_hover_jspace_pose_[1]; // copy over the track position
            if (!get_pickup_IK(affine_gripper_regrasp, q_bin_pulley_flip_, gripper_regrasp_jspace)) {
                ROS_WARN("could not compute IK soln for dropoff pose!");
                result_.errorCode = RobotMoveResult::UNREACHABLE;
                return errorCode;
            }

            //pick up the pulley
            ROS_INFO("moving to bin_cruise_jspace_pose_ ");
            move_to_jspace_pose(bin_cruise_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            ros::Duration(2.0).sleep(); //TUNE ME!!
            //now move to bin hover pose:

            //ROS_INFO("moving to bin_hover_jspace_pose_ ");
            //move_to_jspace_pose(bin_hover_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            //ros::Duration(2.0).sleep(); //TUNE ME!!

            //now move to pickup approach pose:
            ROS_INFO("moving to approach_pickup_jspace_pose_ ");
            move_to_jspace_pose(approach_pickup_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            ros::Duration(2.0).sleep(); //TUNE ME!!

            //now move to bin pickup pose:
            ROS_INFO("moving to pickup_jspace_pose_ ");
            move_to_jspace_pose(pickup_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            ros::Duration(2.0).sleep(); //TUNE ME!!

            ROS_INFO("enabling gripper");
            grab();
            while (!robotInterface.isGripperAttached()) {
                ros::Duration(0.5).sleep();
                ROS_INFO("waiting for gripper attachment");
            }
            ROS_INFO("part is attached to gripper");

            // ros::Duration(2.0).sleep(); //TUNE ME!!
            //ROS_INFO("I %s got the part", robotInterface.isGripperAttached()? "still": "did not");
            //enable gripper

            ROS_INFO("moving to bin hover pose");
            move_to_jspace_pose(bin_hover_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            ros::Duration(2.0).sleep(); //TUNE ME!!

            //if here, part is grasped and held at hover pose;
            // now need trajectory to flip it...
            // thoughts:  for bin, try:
            // rotate gripper -pi/2 (or more) about y axis to tilt part
            // translate to pose above original pick-up, keeping same orientation
            // descend to zzz
            // release gripper
            // do cartesian move...pick vector

            //where is the gripper?
            ROS_INFO("moving to tilt pose");
            move_to_jspace_pose(gripper_tilted_jspace);
            ros::Duration(2.0).sleep(); 
            ROS_INFO("moving to tilt pose2");
            move_to_jspace_pose(gripper_tilted_jspace2);
            ros::Duration(2.0).sleep(); 
            ROS_INFO("moving to tilt pose3");
            move_to_jspace_pose(gripper_tilted_jspace3);
            ros::Duration(3.0).sleep(); 

            //translate gripper back towards grasp x,y,z, only set z higher
            //ROS_INFO("moving to dropoff pose");
            //move_to_jspace_pose(q6dof_dropoff);
            //ros::Duration(2.0).sleep(); 
            

            release();  //release gripper
            while (robotInterface.isGripperAttached()) {
                ros::Duration(0.5).sleep();
                ROS_INFO("waiting for gripper release");
            }     
            
            ROS_INFO("moving to tilt pose3");
            move_to_jspace_pose(gripper_tilted_jspace3);
            ros::Duration(3.0).sleep(); 
            gripper_tilted_jspace3[3]-= 0.2; //pan away from disk
            move_to_jspace_pose(gripper_tilted_jspace3);
            ros::Duration(3.0).sleep();             
            
            gripper_tilted_jspace3[2]-= 0.5; // lift
            move_to_jspace_pose(gripper_tilted_jspace3);
            ros::Duration(3.0).sleep(); 
            
            gripper_tilted_jspace3[3]+= 0.5; //pan towards regrasp pose
             move_to_jspace_pose(gripper_tilted_jspace3);
            ros::Duration(2.0).sleep();            

            gripper_tilted_jspace3[2]+= 0.2; //pan towards regrasp pose
             move_to_jspace_pose(gripper_tilted_jspace3);
            ros::Duration(2.0).sleep();   
            
            
            move_to_jspace_pose(gripper_regrasp_jspace);
            ros::Duration(2.0).sleep(); 
            

     errorCode = RobotMoveResult::CANCELLED; //debug--return error
    return errorCode;
    }

   else {
       ROS_WARN("don't know how to flip this part");
     errorCode = RobotMoveResult::CANCELLED; //return error
     return errorCode;
   }

}
