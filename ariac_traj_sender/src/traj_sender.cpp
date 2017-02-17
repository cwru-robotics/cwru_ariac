//wsn traj sender
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <ariac_ur_fk_ik/ur_kin.h>
#include <sensor_msgs/JointState.h>

vector<string> g_jnt_names;
ros::Publisher g_joint_trajectory_publisher;

//fnc to fill a trajectory message with a single joint-space pose
//provide a 7-dof joint-space pose, and get back this pose in a trajectory message
trajectory_msgs::JointTrajectory jspace_pose_to_traj(Eigen::VectorXd joints) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    int njnts = g_jnt_names.size();
    msg.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    msg.joint_names = g_jnt_names;
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(njnts);
    for (int i = 0; i < njnts; ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.2);
    ROS_INFO_STREAM("populated traj msg:\n" << msg);
    return msg;
}

//given a trajectory, append a point to the trajectory; set arrival time to 
//previous arrival time, plus delta_t seconds
void add_pose_to_traj(Eigen::VectorXd joints, double delta_t, trajectory_msgs::JointTrajectory &traj) {
    int njnts = g_jnt_names.size();
    int npts = traj.points.size();
    //vector<double> new_pos;
    trajectory_msgs::JointTrajectoryPoint new_point;
    //populate this new point with position values:
    for (int i=0;i<njnts;i++) new_point.positions.push_back(joints[i]);
    //get previous arrival time:
    ros::Duration prev_time = traj.points[npts-1].time_from_start;  
    double secs = prev_time.toSec();
    ROS_INFO("previous arrival time = %f",secs);
    secs+=delta_t; //increment the arrival time
    new_point.time_from_start = ros::Duration(secs); //and specify in the new point
    traj.points.push_back(new_point);       // add this point to the trajectory.  

    ROS_INFO_STREAM("appended traj msg:\n" << traj);
}


int main(int argc, char ** argv){
    //set up a ros node
    ros::init(argc, argv, "trajectory_sender");
    ros::NodeHandle nh;
    control_msgs::FollowJointTrajectoryGoal goal;
    UR10FwdSolver fwd_solver;
    UR10IkSolver ik_solver;
    Eigen::VectorXd q_fit,q_7dof,q_6dof,q_7dof_soln;
    trajectory_msgs::JointTrajectory traj;
    g_joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);    
    
    fwd_solver.get_joint_names_7dof(g_jnt_names);
    
    //start w/ an example qvec to generate a test affine pose;
    q_7dof.resize(7);
    q_7dof<<1.5, 0.4, -0.8, 2.15, 4.0, -1.57, 0.50;
    double rail_pos = q_7dof[1];
    //q_7dof_bin8_approach<<1.85, -0.535, -0.47, 3.14, 3.33, -1.57, 0.50;
    q_6dof = fwd_solver.map726dof(q_7dof);  
    //here is a reachable affine pose for testing:          
    Eigen::Affine3d A_fwd_DH = fwd_solver.fwd_kin_solve(q_6dof); //fwd_kin_solve
    
    //display this pose:
       std::cout << "q_in: " << q_6dof.transpose() << std::endl;
        std::cout << "A rot: " << std::endl;
        std::cout << A_fwd_DH.linear() << std::endl;
        std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;
        Eigen::Matrix3d R_flange = A_fwd_DH.linear();

        Eigen::Quaterniond quat(R_flange);
        std::cout<<"quat: "<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl; 
        
     //now compute IK for this pose:
        std::vector<Eigen::VectorXd> q6dof_solns;
        
        int nsolns = ik_solver.ik_solve(A_fwd_DH,q6dof_solns);
        std::cout << "number of IK solutions: " << nsolns << std::endl;    
        //select the solution that is closest to some reference--try q_6dof_bin8_approach

        q_fit = fwd_solver.closest_soln(q_6dof,q6dof_solns);
        cout<<"best fit soln: "<<q_fit.transpose()<<endl;
        
        q_7dof_soln = fwd_solver.map627dof(rail_pos,q_fit);
        std::cout << "q_in_7dof: " << q_7dof.transpose() << std::endl;
        std::cout << "q_soln: " << q_7dof_soln.transpose() << std::endl;        
       
       traj = jspace_pose_to_traj(q_7dof_soln);
       add_pose_to_traj(q_7dof_soln, 1.0, traj); 
       
       //try sending this trajectory:
       g_joint_trajectory_publisher.publish(traj);
       return 0; 
            
}


