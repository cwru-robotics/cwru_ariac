unsigned short int RobotMoveActionServer::fetch_from_conveyor(const cwru_ariac::RobotMoveGoalConstPtr &goal) {
    unsigned short int errorCode = RobotMoveResult::NO_ERROR;
    ROS_INFO("Received goal type: %d", goal->type);
    cwru_ariac::Part part = goal->sourcePart;
    ROS_INFO("The part is %s, should be fetched from location code %s ", part.name.c_str(),
             placeFinder[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    //HACK FOR QUAL2b:  xxxxxxxxxxxxxxxxxxxxxxxxx
    //part.linear.y =QUAL2_CONVEYOR_SPEED;
    double start_time = ros::Time::now().toSec();
    double part_header_time = part.pose.header.stamp.toSec();
    double dt = start_time - part_header_time;
    ROS_INFO("part was spotted %f sec ago", dt);
    double vy = part.linear.y;
    double part_y_start = part.pose.pose.position.y;
    ROS_INFO("travel speed vy = %f", vy);
    double dt_move;
    int npts = 15; //arbitrary number of points in conveyor-tracking trajectory
    vector<double> q_rail_vals, arrival_times;
    double q_rail_start = 2.0;  //FIX ME!  choose rail position from which to begin conveyor tracking of part
    //this looks OK for parts just observed; fetch uses about 1m of conveyor motion
    //don't go more negative than -1m on rail, or robot will likely hit frame during turn
    // therefore, q_rail_start>= 0.0
    // for part further downstream, anticipate 2sec move to q_rail_start, so estimate where part
    // will be approx 2.5sec in the future, and send robot there to wait

    //where will the part be in 2.5 sec?
    double fudge_wait_time = 6.0; //4 sec
    double part_dock_y = part_y_start + vy * fudge_wait_time;
    ROS_INFO("part_y_start = %f", part_y_start);
    ROS_INFO("earliest part_dock_y = %f", part_dock_y);
    if (part_dock_y < 0) {
        ROS_WARN("can't reach this conveyor part in time");
        errorCode = RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    if (part_dock_y < q_rail_start) {
        q_rail_start = part_dock_y;
    }
    ROS_INFO("will try to start tracking at y = %f", q_rail_start);

    double part_y_travel = q_rail_start - part_y_start;
    double part_travel_time = part_y_travel / vy;
    double part_arrival_time = part_header_time + part_travel_time;
    ROS_INFO("part should arrive at y= %f after traveling distance %f in time %f",
             q_rail_start, part_y_travel, part_travel_time);
    ROS_INFO("expect arrival at t = %f", part_arrival_time);
    robotState = calcRobotState();
    double cur_q_rail = robotState.jointStates[1];

    Eigen::VectorXd q_intermediate_pose;
    q_intermediate_pose = q_conveyor_cruise_pose_; //q_conveyor_hover_pose_;//
    double q_rail_safe_spin = 0.0;
    q_intermediate_pose[1] = q_rail_safe_spin; //force turret rotation at safe rail position //cur_q_rail;



    q_rail_vals.push_back(q_rail_start + vy * 1.0); //allow 1 sec to get to first point
    arrival_times.push_back(1.0);  // + 0.5*vy*(arrival_times[1]-arrival_times[0]);
    ROS_INFO("q_rail_vals[0] = %f", q_rail_vals[0]);

    int ipnt = 0;
    while (q_rail_vals[ipnt] > CONVEYOR_FETCH_QRAIL_MIN) {
        ipnt++;
        arrival_times.push_back(arrival_times[ipnt - 1] + 1.0); //points at 1-sec intervals
        q_rail_vals.push_back(q_rail_vals[ipnt - 1] + vy * (arrival_times[ipnt] - arrival_times[ipnt - 1]));
        //ROS_INFO("q_rail_vals[ %d ] = %f",ipnt,q_rail_vals[ipnt]);
    }
    npts = q_rail_vals.size();
    ROS_INFO("num pts in tracking traj = %d", npts);

    //set the part y-value to zero, w/rt robot: fix this later
    part.pose.pose.position.y = q_rail_start;

    //strategy:
    // compute approach and grasp IK solns (assuming dy=0)
    //move robot to a conveyor-hover pose:  approach pose at some y_start of q_rail
    //compute a balistic docking trajectory:
    // pre-compute q_rail(t_ramp), q_rail(t_const_v), q_rail,q_grasp(dt_grasp),
    q_conveyor_hover_pose_[1] = q_rail_start;
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part, q_conveyor_hover_pose_[1]);
    if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, q_conveyor_hover_pose_, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
        errorCode = RobotMoveResult::UNREACHABLE;
        return errorCode;
        //  as.setAborted(result_);
    }
    ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                             approach_pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode = RobotMoveResult::UNREACHABLE;
        return errorCode;
    }

    if (!bin_hover_jspace_pose(goal->targetPart.location, agv_hover_pose_)) {
        ROS_WARN("bin_hover_jspace_pose() failed for target agv");
        errorCode = RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    affine_vacuum_dropoff_pose_wrt_base_link_ = affine_vacuum_dropoff_pose_wrt_base_link(goal->targetPart,
                                                                                         agv_hover_pose_[1]);
    ROS_INFO("gripper pose for drop-off: ");
    std::cout << affine_vacuum_dropoff_pose_wrt_base_link_.translation().transpose() << std::endl;
    if (!get_pickup_IK(affine_vacuum_dropoff_pose_wrt_base_link_, agv_hover_pose_, dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for drop-off pose!");
        errorCode = RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("dropoff_jspace_pose_: " << dropoff_jspace_pose_.transpose());
    //compute offset for dropoff
    if (!compute_approach_IK(affine_vacuum_dropoff_pose_wrt_base_link_, dropoff_jspace_pose_, approach_dist_,
                             approach_dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff approach pose!");
        errorCode = RobotMoveResult::UNREACHABLE;
        return errorCode;
    }


    dt_move = 2.0;
    ROS_INFO("moving to conveyor intermediate pose at qrail=%f", q_rail_safe_spin);
    cout << "q_soln: " << q_intermediate_pose.transpose() << endl;
    move_to_jspace_pose(q_intermediate_pose, dt_move);
    ros::Duration(dt_move).sleep();


    ROS_INFO("moving to approach pose ");
    move_to_jspace_pose(approach_pickup_jspace_pose_, 2.0);
    ros::Duration(2.0).sleep(); //TUNE ME!!

    trajectory_msgs::JointTrajectory traj;
    int njnts = robotInterface.getJointsNames().size();
    ROS_INFO("njnts = %d", njnts);
    traj.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    traj.joint_names = robotInterface.getJointsNames();
    // Create one point in the trajectory.
    traj.points.resize(npts); //arbitrary

    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    ROS_INFO("populating tracking trajectory");
    for (int i = 0; i < 2; i++) {
        //ROS_INFO("i= %d",i);
        traj.points[i].positions.resize(njnts);
        for (int j = 0; j < njnts - 1; ++j) {
            traj.points[i].positions[j] = approach_pickup_jspace_pose_[j];
        }
        traj.points[i].positions[1] = q_rail_vals[i];
        traj.points[i].positions[njnts - 1] = 0;
        traj.points[i].time_from_start = ros::Duration(arrival_times[i]);
    }

    for (int i = 2; i < npts; i++) {
        //ROS_INFO("i= %d",i);
        traj.points[i].positions.resize(njnts);
        for (int j = 0; j < njnts - 1; ++j) {
            traj.points[i].positions[j] = pickup_jspace_pose_[j];
        }
        traj.points[i].positions[1] = q_rail_vals[i];
        traj.points[i].positions[njnts - 1] = 0;
        traj.points[i].time_from_start = ros::Duration(arrival_times[i]);
    }
    //have traj queued up, but wait to launch it.  Compute when part is expected to arrive at y=0;
    double cur_time = ros::Time::now().toSec();
    double wait_time =
            part_arrival_time - cur_time - CONVEYOR_TRACK_FUDGE_TIME; //need to start about 0.5sec early...not clear why
    ROS_INFO("waiting %f sec to start intercept motion", wait_time);
    ros::Duration(wait_time).sleep();
    traj.header.stamp = ros::Time::now();
    ROS_INFO("publishing tracking trajectory");
    //ROS_INFO_STREAM(traj);
    joint_trajectory_publisher_.publish(traj);
    ROS_INFO("enabling gripper");
    grab();
    robotState = calcRobotState();
    cur_q_rail = robotState.jointStates[1];
    while ((!robotInterface.isGripperAttached()) && (cur_q_rail > CONVEYOR_FETCH_QRAIL_MIN)) {
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for gripper attachment");
        robotState = calcRobotState();
        cur_q_rail = robotState.jointStates[1];
    }
    if (!robotInterface.isGripperAttached()) {
        ROS_INFO("could not grab part");
        errorCode = RobotMoveResult::GRIPPER_FAULT;
        return errorCode;
    }
    robotState = calcRobotState();
    cur_q_rail = robotState.jointStates[1];
    double lift_time = 1.0;
    approach_pickup_jspace_pose_[1] = cur_q_rail + lift_time * vy;
    ROS_INFO("performing lift at qrail = %f", approach_pickup_jspace_pose_[1]);
    move_to_jspace_pose(approach_pickup_jspace_pose_, lift_time);
    ROS_INFO("lifting part...");
    ros::Duration(lift_time).sleep();

    q_intermediate_pose = q_conveyor_cruise_pose_; //q_conveyor_hover_pose_;//
    q_intermediate_pose[1] = q_rail_safe_spin; //force turret rotation at safe rail position
    q_intermediate_pose[3] = approach_pickup_jspace_pose_[3]; //but keep same turret angle for now;

    q_conveyor_cruise_pose_[1] = q_rail_safe_spin;


    dt_move = 2.0;
    ROS_INFO("moving to conveyor intermediate pose at qrail=%f", q_rail_safe_spin);
    cout << "q_soln: " << q_intermediate_pose.transpose() << endl;
    move_to_jspace_pose(q_intermediate_pose, dt_move);
    ros::Duration(dt_move).sleep();

    ROS_INFO("moving to cruise pose at qrail = %f", q_conveyor_cruise_pose_[1]);
    cout << "q_soln: " << q_conveyor_cruise_pose_.transpose() << endl;
    move_to_jspace_pose(q_conveyor_cruise_pose_, dt_move);
    ros::Duration(dt_move).sleep();

    ROS_INFO("testing if part is still grasped");

    if (!robotInterface.isGripperAttached()) {
        errorCode = RobotMoveResult::PART_DROPPED;
        ROS_WARN("part dropped!");
        return errorCode;
    }
    ROS_INFO("part is still grasped");


    ROS_INFO("moving to agv1_cruise_pose_");
    cout << "q_soln: " << q_agv1_cruise_pose_.transpose() << endl;
    move_to_jspace_pose(q_agv1_cruise_pose_, dt_move); //move to agv cruise pose
    ros::Duration(dt_move).sleep(); //TUNE ME!!
    if (!robotInterface.isGripperAttached()) {
        errorCode = RobotMoveResult::PART_DROPPED;
        ROS_WARN("part dropped!");
        return errorCode;
    }
    ROS_INFO("moving to approach_dropoff_jspace_pose_");
    move_to_jspace_pose(approach_dropoff_jspace_pose_, dt_move); //move to agv hover pose
    ros::Duration(dt_move).sleep(); //TUNE ME!!
    if (!robotInterface.isGripperAttached()) {
        errorCode = RobotMoveResult::PART_DROPPED;
        ROS_WARN("part dropped!");
        return errorCode;
    }

    ROS_INFO("moving to dropoff_jspace_pose_");
    dt_move = 1.0;
    move_to_jspace_pose(dropoff_jspace_pose_, dt_move); //move to agv hover pose
    ros::Duration(dt_move).sleep(); //TUNE ME!!
    if (!robotInterface.isGripperAttached()) {
        errorCode = RobotMoveResult::PART_DROPPED;
        ROS_WARN("part dropped!");
        return errorCode;
    }
    ROS_INFO("releasing gripper");
    release();
    while (robotInterface.isGripperAttached()) {
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for gripper release");
    }
    ROS_INFO("moving to approach_dropoff_jspace_pose_");
    move_to_jspace_pose(approach_dropoff_jspace_pose_, dt_move); //move to agv hover pose
    ros::Duration(dt_move).sleep(); //TUNE ME!!

    ROS_INFO("moving to agv1_cruise_pose_");
    cout << "q_soln: " << q_agv1_cruise_pose_.transpose() << endl;
    move_to_jspace_pose(q_agv1_cruise_pose_, dt_move); //move to agv cruise pose
    ros::Duration(dt_move).sleep(); //TUNE ME!!
    ROS_INFO("part has been placed at target location");

    return errorCode;
}
