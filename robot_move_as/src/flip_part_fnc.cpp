//this fnc should flip a part (and return its new pose)
// focus on pulley_part; 

Eigen::Matrix3d compute_roty(double theta_y) {
    Eigen::Matrix3d Roty;
    Eigen::Vector3d roty_x, roty_y, roty_z;

    roty_x << cos(theta_y), 0, -sin(theta_y);
    roty_y << 0, 1, 0;
    roty_z << sin(theta_y), 0, cos(theta_y);
    Roty.col(0) = roty_x;
    Roty.col(1) = roty_y;
    Roty.col(2) = roty_z;
    return Roty;
}

Eigen::Matrix3d compute_rotz(double theta_z) {
    Eigen::Matrix3d Rotz;
    Eigen::Vector3d rotz_x, rotz_y, rotz_z;

    rotz_x << cos(theta_z), sin(theta_z), 0;
    rotz_y << -sin(theta_z), cos(theta_z), 0;
    rotz_z << 0, 0, 1;
    Rotz.col(0) = rotz_x;
    Rotz.col(1) = rotz_y;
    Rotz.col(2) = rotz_z;
    return Rotz;
}

Eigen::Matrix3d compute_rotx(double theta_x) {
    Eigen::Matrix3d Rotx;
    Eigen::Vector3d rotx_x, rotx_y, rotx_z;

    rotx_x << 1, 0, 0; //cos(theta_z),sin(theta_z),0;
    rotx_y << 0, cos(theta_x), sin(theta_x);//-sin(theta_z),cos(theta_z),0;
    rotx_z << 0, -sin(theta_x), cos(theta_x);//0,1;
    Rotx.col(0) = rotx_x;
    Rotx.col(1) = rotx_y;
    Rotx.col(2) = rotx_z;
    return Rotx;
}


unsigned short int RobotMoveActionServer::flip_part_fnc(const cwru_ariac::RobotMoveGoalConstPtr &goal) {
    unsigned short int errorCode = RobotMoveResult::CANCELLED; //change to NO_ERROR, if ultimately successful

    ROS_INFO("flip_part function received goal type: %d", goal->type);
    q_bin_pulley_flip_ << 1.77, 1.13, -0.68, 3.2, 4.9, -3, 0; //a ref pose to help with IK
    Eigen::VectorXd q_edge_dropoff_ref;
    q_edge_dropoff_ref.resize(7);
    q_edge_dropoff_ref << 1.77, 1.20, -0.62, 2.94, 5.14, -0.20, 1.57; //ref pose for IK assistance for edge-dropoff pose
    bool is_attached = false;

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
        Eigen::Affine3d affine_gripper_hover, affine_gripper_tilted;
        Eigen::Affine3d affine_gripper_tilted2, affine_gripper_tilted3, affine_gripper_dropoff;
        Eigen::Affine3d affine_gripper_dropoff_depart, affine_gripper_dropff_depart2, affine_gripper_depart3;
        Eigen::Affine3d affine_gripper_regrasp_approach, affine_gripper_regrasp;
        affine_gripper_hover = fwd_solver_.fwd_kin_solve(q6dof_hover); // given vector of q angles, compute fwd kin
        ROS_INFO("hover gripper pose w/rt robot base: ");
        xformUtils_.printAffine(affine_gripper_hover);
        Eigen::Matrix3d R_hover, Roty, Rotz, R_tilted, R_tilted2;
        double theta_y = -M_PI / 2.0; //3.0*M_PI/4.0;
        Roty = compute_roty(theta_y);
        double theta_z = -M_PI / 2.0;
        Rotz = compute_rotz(theta_z);

        // rotate gripper by roty(theta_y):
        R_hover = affine_gripper_hover.linear();
        R_tilted = Roty * R_hover;
        R_tilted2 = Rotz * R_tilted;
        affine_gripper_tilted = affine_gripper_hover;
        affine_gripper_tilted.linear() = R_tilted;
        affine_gripper_tilted2 = affine_gripper_tilted;
        affine_gripper_tilted2.linear() = R_tilted2;
        affine_gripper_tilted3 = affine_gripper_tilted2;
        //affine_gripper_tilted3.translation()=
        Eigen::Vector3d O_hover, O_drop;
        //O_drop =
        O_hover = affine_gripper_tilted.translation();
        Eigen::VectorXd gripper_tilted_jspace, gripper_tilted_jspace2, gripper_tilted_jspace3;
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
        ROS_INFO_STREAM("O_pickup: " << O_pickup);
        //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
        //note: may need to go to approach pose first; default motion is in joint space
        //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
        if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, pickup_jspace_pose_)) {
            ROS_WARN("could not compute IK soln for pickup pose!");
            errorCode = RobotMoveResult::UNREACHABLE;
            return errorCode;
        }
        ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());
        //compute approach_pickup_jspace_pose_
        //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);

        if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                                 approach_pickup_jspace_pose_)) {
            ROS_WARN("could not compute IK soln for pickup approach pose!");
            errorCode = RobotMoveResult::UNREACHABLE;
            return errorCode;
        }
        ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());

        affine_gripper_dropoff = affine_gripper_tilted2; //re-use this orientation after part rotation
        Eigen::Vector3d O_dropoff, O_regrasp;
        //"dropoff" really means to set part back on bin one its rim
        // the following seems to work, based on pickup origin
        O_dropoff = O_pickup; //affine_vacuum_pickup_pose_wrt_base_link_.translation();
        //keep same O_dropoff[0];
        O_dropoff[1] -= 0.05;  //dropoff shifted to robot's left, so have room to knock over, if necessary
        O_dropoff[2] += 0.04; //elevation higher than pickup pose, hard coded to set on disk's rim

        //O_dropoff<<-0.802, -0.032, -0.156; //from t_echo; does this work?
        ROS_INFO_STREAM("using O_dropoff= " << O_dropoff.transpose());
        affine_gripper_dropoff.translation() = O_dropoff;

        //now compute regrasp pose, for grasp on opposite face w/ pulley on its edge
        O_regrasp = O_dropoff;
        //O_regrasp[0]-=0.02; //could tweak this slightly--if want to reach out a bit more; depends on how much pulley rolls to halt
        O_regrasp[1] -= PULLEY_PART_THICKNESS; //y-value should be shifted by part thickness; this should be OK

        O_regrasp[2] += 0.065; //compute regrasp height relative to dropoff height
        ROS_INFO_STREAM("computed regrasp origin: " << O_regrasp.transpose());
        //choose pick-up position ~ -0.770, -0.094, -0.107
        //O_regrasp<<-0.770, -0.094, -0.107;  //
        ROS_INFO_STREAM("using O_regrasp= " << O_regrasp.transpose());
        Eigen::Matrix3d R_regrasp;
        Eigen::Vector3d x_vec, y_vec, z_vec;
        x_vec << 1, 0, 0;
        y_vec << 0, 0, 1;
        z_vec << 0, -1, 0;
        R_regrasp.col(0) = x_vec;
        R_regrasp.col(1) = y_vec;
        R_regrasp.col(2) = z_vec;
        //R_regrasp = compute_rotx(M_PI)*R_tilted2;
        affine_gripper_regrasp.linear() = R_regrasp;
        affine_gripper_regrasp.translation() = O_regrasp;


        ROS_WARN("computing edge-dropoff pose; using affine: ");
        ROS_INFO_STREAM(affine_gripper_dropoff.linear());
        ROS_INFO_STREAM(affine_gripper_dropoff.translation().transpose());
        //try more targeted reference pose for dropoff
        if (!get_pickup_IK(affine_gripper_dropoff, q_edge_dropoff_ref, gripper_tilted_jspace3)) {
            ROS_WARN("could not compute IK soln for dropoff pose!");
            errorCode = RobotMoveResult::UNREACHABLE;
            return errorCode;
        }
        ROS_INFO_STREAM("jspace dropoff: " << std::endl << gripper_tilted_jspace3.transpose());
        ROS_INFO_STREAM("reference jpose:" << std::endl << bin_hover_jspace_pose_.transpose());

        ROS_INFO("computing regrasp pose; using affine: ");
        ROS_INFO_STREAM(R_regrasp);
        ROS_INFO_STREAM(O_regrasp.transpose());
        q_bin_pulley_flip_[1] = bin_hover_jspace_pose_[1]; // copy over the track position
        if (!get_pickup_IK(affine_gripper_regrasp, q_bin_pulley_flip_, gripper_regrasp_jspace)) {
            ROS_WARN("could not compute IK soln for regrasp pose!");
            errorCode = RobotMoveResult::UNREACHABLE;
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

        errorCode = pick_part_fnc(goal);
        if (errorCode != RobotMoveResult::NO_ERROR) {
            return errorCode;
        }
        //if here, then part is grasped and held at bin hover pose
        //check if part is still attached
        if (!robotInterface.isGripperAttached()) {
            ROS_WARN("dropped part!");
            errorCode = RobotMoveResult::PART_DROPPED; //debug--return error
            return errorCode;
        }

        /*         // the following is dead code--encapsulated in pick_part_fnc()

                   //now move to pickup approach pose:
                   ROS_INFO("moving to approach_pickup_jspace_pose_ ");
                   move_to_jspace_pose(approach_pickup_jspace_pose_,1.0); //so far, so good, so move to cruise pose in front of bin
                   //at this point, have already confired bin ID is good
                   ros::Duration(1.0).sleep(); //TUNE ME!!

                   ROS_INFO("enabling gripper");
                   grab(); //do this early, so grasp at first contact

                   //now move to bin pickup pose:
                   ROS_INFO_STREAM("moving to pickup_jspace_pose_ "<<pickup_jspace_pose_);
                   move_to_jspace_pose(pickup_jspace_pose_); // try to pick up part

                   errorCode = grasp_fnc(3.0); //use the grasp fnc; timeout set for 3 sec
                   //ROS_INFO("got grasp error code %d",(int) errorCode);

                   if(errorCode!=RobotMoveResult::NO_ERROR) { //if not successful, try moving to attach
                      ROS_WARN("did not attach; trying lower for up to 5 sec");
                      pickup_jspace_pose_[2]+= 0.1; // crude move...lower via shoulder-lift joint
                      bool is_attached=false;
                      double t_wait_timeout=5.0;
                      double t_wait=0.0;
                      double dt_wait = 0.2;
                      move_to_jspace_pose(pickup_jspace_pose_,t_wait_timeout); //5 sec is a slow move
                      while ((!is_attached) && (t_wait<t_wait_timeout)) {
                       is_attached = robotInterface.isGripperAttached();
                       ros::Duration(dt_wait).sleep();
                       t_wait+=dt_wait; //keep testing attachment state; halt move as soon as attached
                       ROS_INFO("waiting for gripper attachment");
                      }
                      if(!is_attached) {
                       ROS_WARN("could not grasp part; giving up");
                       release_fnc(1.0);
                       //move to safe pose
                       ROS_INFO("moving to bin_hover_jspace_pose_ ");
                       move_to_jspace_pose(bin_hover_jspace_pose_, 1.0);
                       ros::Duration(1.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec
                       errorCode = RobotMoveResult::GRIPPER_FAULT;
                       return errorCode;
                      }
                   }

                   //if here, part is now grasped
                   ROS_INFO("part is attached to gripper");

                   // ros::Duration(2.0).sleep(); //TUNE ME!!
                   //ROS_INFO("I %s got the part", robotInterface.isGripperAttached()? "still": "did not");
                   //enable gripper

                   ROS_INFO("moving to bin hover pose");
                   move_to_jspace_pose(bin_hover_jspace_pose_,1.0); //so far, so good, so move to cruise pose in front of bin
                   //at this point, have already confired bin ID is good
                   ros::Duration(1.0).sleep(); //TUNE ME!!
       */
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
        move_to_jspace_pose(gripper_tilted_jspace, 1.0);
        ros::Duration(1.0).sleep();
        ROS_INFO("moving to tilt pose2");
        move_to_jspace_pose(gripper_tilted_jspace2, 1.0);
        ros::Duration(1.0).sleep();
        ROS_INFO("moving to tilt pose3 for resetting part on edge");
        move_to_jspace_pose(gripper_tilted_jspace3, 2.0);
        ros::Duration(3.0).sleep(); //wait extra settling time

        //release the gripper
        errorCode = release_fnc(5.0); //wait up to max of 5 sec for release
        if (errorCode != RobotMoveResult::NO_ERROR) {
            return errorCode;
        }

        ROS_INFO("yaw depart:");
        gripper_tilted_jspace3[3] -= 0.1; //pan away from disk
        ROS_INFO_STREAM("moving away from disk: ");
        ROS_INFO_STREAM(gripper_tilted_jspace3.transpose());
        move_to_jspace_pose(gripper_tilted_jspace3, 1.0);
        ros::Duration(1.0).sleep();

        gripper_tilted_jspace3[2] -= 0.5; // lift
        ROS_INFO_STREAM("lift: " << std::endl << gripper_tilted_jspace3.transpose());
        move_to_jspace_pose(gripper_tilted_jspace3, 1.0);
        ros::Duration(1.0).sleep();

        //gripper_tilted_jspace3[3]+= 0.5; //pan towards regrasp pose
        gripper_tilted_jspace3[3] = gripper_regrasp_jspace[3] + 0.1;//pan towards regrasp pose
        double pan_angle = gripper_tilted_jspace3[3];
        ROS_INFO_STREAM("pan towards regrasp: " << std::endl << gripper_tilted_jspace3.transpose());
        move_to_jspace_pose(gripper_tilted_jspace3, 1.0);
        ros::Duration(1.0).sleep();

        //gripper_tilted_jspace3[2]+= 0.2; //lower
        // move_to_jspace_pose(gripper_tilted_jspace3);
        //ros::Duration(2.0).sleep();

        //do wrist flip:
        gripper_tilted_jspace3[4] = q_bin_pulley_flip_[4];
        gripper_tilted_jspace3[5] = q_bin_pulley_flip_[5];
        gripper_tilted_jspace3[6] = q_bin_pulley_flip_[6];

        ROS_INFO_STREAM("wrist flip: " << std::endl << gripper_tilted_jspace3.transpose());
        move_to_jspace_pose(gripper_tilted_jspace3, 1.0);
        ros::Duration(1.0).sleep();
//xxx bug here; q_bin_pulley_flip_[2] is wrong
        gripper_tilted_jspace3 = gripper_regrasp_jspace; // lower
        gripper_tilted_jspace3[3] = pan_angle;
        ROS_INFO_STREAM("lower: " << std::endl << gripper_tilted_jspace3.transpose());
        move_to_jspace_pose(gripper_tilted_jspace3, 1.0);
        ros::Duration(1.0).sleep();

        //gripper_tilted_jspace3[3]+= q_bin_pulley_flip_[3];  // pan back towards part

        ROS_INFO("enabling gripper");
        grab(); //do this early, so grasp at first contact
        ROS_INFO("moving into part--trying to grasp");
        // q_bin_pulley_flip_[3]; //-=0.02;
        ROS_INFO_STREAM("move to gripper_regrasp_jspace: " << std::endl << gripper_regrasp_jspace.transpose());
        move_to_jspace_pose(gripper_regrasp_jspace, 2.0); //computed grasp pose in jspace

        //try to grasp at computed regrasp pose
        errorCode = grasp_fnc(3.0); //use the grasp fnc; timeout set for 3 sec
        if (errorCode == RobotMoveResult::NO_ERROR) is_attached = true;


        //ros::Duration(2.5).sleep();
        //NOT using

        //gripper_tilted_jspace3 = q_bin_pulley_flip_;
        //gripper_tilted_jspace3[3]+=0.01; // move into part
        //q_bin_pulley_flip_[3]+=0.02;
        //move_to_jspace_pose(q_bin_pulley_flip_);
        //ros::Duration(2.0).sleep();
/*
             t_wait=0.0;
             is_attached=false;
             while (!is_attached && (t_wait<3.0)) {
                is_attached = robotInterface.isGripperAttached();
                ros::Duration(0.5).sleep();
                t_wait+=0.5;
                ROS_INFO("waiting for gripper attachment");
            }
             
            }
*/

        if (errorCode != RobotMoveResult::NO_ERROR) { //here if grasp failed
            ROS_WARN("timed out waiting for attachment; try moving into part");
            //knock the part over;
            //release();  //release gripper
            gripper_regrasp_jspace[3] -= 0.3; //yaw to knock part over
            move_to_jspace_pose(gripper_regrasp_jspace, 3.0);
            errorCode = grasp_fnc(3.0); //use the grasp fnc; timeout set for 3 sec
            if (errorCode != RobotMoveResult::NO_ERROR) {
                errorCode = release_fnc(5.0); //wait up to max of 5 sec for release
            } else is_attached = true;
        }

        if (!is_attached) {
            ROS_INFO("part attachment failed; tried to knock part over; retracting gripper");
            ROS_INFO("moving to tilt pose2");
            move_to_jspace_pose(gripper_tilted_jspace2, 1.0);
            ros::Duration(1.0).sleep();
            ROS_INFO("moving to tilt pose1");
            move_to_jspace_pose(gripper_tilted_jspace, 1.0);
            ros::Duration(1.0).sleep();
            ROS_INFO("moving to bin cruise pose");
            move_to_jspace_pose(bin_cruise_jspace_pose_, 1.0);
            ros::Duration(1.0).sleep();
            ROS_WARN("returning dropped-part to invoke regrasp");
            errorCode = RobotMoveResult::PART_DROPPED; //debug--return error
            return errorCode;
        }

        ROS_INFO("part is attached to gripper; lifting part");
        gripper_regrasp_jspace[2] -= 0.5; //lift
        move_to_jspace_pose(gripper_regrasp_jspace, 1.0);
        ros::Duration(1.0).sleep();


        ROS_INFO("moving to tilt pose2");
        move_to_jspace_pose(gripper_tilted_jspace2, 1.0);
        ros::Duration(1.0).sleep();
        // ROS_INFO("moving to tilt pose1");
        //move_to_jspace_pose(gripper_tilted_jspace);
        //ros::Duration(2.0).sleep();
        O_pickup[1] += 0.047; //magic number to account for y-offset of regrasp
        affine_vacuum_pickup_pose_wrt_base_link_.translation() = O_pickup;
        if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                                 approach_pickup_jspace_pose_)) {
            ROS_WARN("could not compute IK soln for flipped reset pose; using original");
        }
        ROS_INFO("moving to approach_pickup_jspace_pose_ ");
        move_to_jspace_pose(approach_pickup_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
        //at this point, have already confired bin ID is good
        ros::Duration(2.0).sleep(); //TUNE ME!!
        release_fnc(1.0);
        if (errorCode != RobotMoveResult::NO_ERROR) {
            return errorCode;
        }

        //say part is dropped to invoke regrasp
        ROS_WARN("flipped part, but returning dropped-part to invoke regrasp");
        errorCode = RobotMoveResult::PART_DROPPED; //debug--return error
        return errorCode;
    } else {
        ROS_WARN("don't know how to flip this part");
        errorCode = RobotMoveResult::CANCELLED; //return error
        return errorCode;
    }

}
