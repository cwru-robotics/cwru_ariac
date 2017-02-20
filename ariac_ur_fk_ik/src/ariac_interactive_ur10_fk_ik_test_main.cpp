// ur10_fk_ik_test_main.cpp
// wsn, October 2016
// test function for ur10_fk_ik kinematics library

#include <ariac_ur_fk_ik/ur_kin.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

Eigen::VectorXd g_q_vec;
using namespace std;
#define VECTOR_DIM 6 // e.g., a 6-dof vector
//Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices; 
vector<string> g_ur_jnt_names,g_ur_jnt_names_7dof;

double sgn(double x) {
    if (x>0.0) return 1.0;
    if (x<0.0) return -1.0;
    return 0.0; //should virtually never happen for floating-point numbers
}
//parse the names in joint_names vector; find the corresponding indices of arm joints
//provide joint_names, as specified in message
void map_arm_joint_indices(vector<string> joint_names) {
 //vector<string> joint_names = joint_state->name;
 //   vector<string> jnt_names;
        
    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

   for (int j=0;j<VECTOR_DIM;j++) {
       j_name = g_ur_jnt_names[j]; //known name, in preferred order
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            //cout<<"found match at index = "<<i<<endl;
            g_arm_joint_indices.push_back(index);
            break;
        }
       }
   }   
   cout<<"indices of arm joints: "<<endl;
   for (int i=0;i<VECTOR_DIM;i++) {
       cout<<g_arm_joint_indices[i]<<", ";
   }
   cout<<endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size()<1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts",njnts);
       map_arm_joint_indices(js_msg.name);
    }
       for (int i=0;i<VECTOR_DIM;i++)
       {
        g_q_vec[i] = js_msg.position[g_arm_joint_indices[i]];
        }
        cout<<"CB: q_vec_right_arm: "<<g_q_vec.transpose()<<endl;       
    
}

//obsolete...
/*
void set_ur_jnt_names() {
  g_ur_jnt_names.push_back("shoulder_pan_joint");
  g_ur_jnt_names.push_back("shoulder_lift_joint");
  g_ur_jnt_names.push_back("elbow_joint");
  g_ur_jnt_names.push_back("wrist_1_joint");
  g_ur_jnt_names.push_back("wrist_2_joint");
  g_ur_jnt_names.push_back("wrist_3_joint");
}

void set_ur_jnt_names_7dof() {
  g_ur_jnt_names_7dof.push_back("elbow_joint");
  g_ur_jnt_names_7dof.push_back("linear_arm_actuator_joint");  
  g_ur_jnt_names_7dof.push_back("shoulder_lift_joint");
  g_ur_jnt_names_7dof.push_back("shoulder_pan_joint");
  g_ur_jnt_names_7dof.push_back("wrist_1_joint");
  g_ur_jnt_names_7dof.push_back("wrist_2_joint");
  g_ur_jnt_names_7dof.push_back("wrist_3_joint");
}
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "ur10_kinematics_test_main");
    ros::NodeHandle nh;
    
    UR10FwdSolver fwd_solver;
    UR10IkSolver ik_solver;
    
    fwd_solver.get_joint_names_6dof(g_ur_jnt_names);
    fwd_solver.get_joint_names_7dof(g_ur_jnt_names_7dof);
    //set_ur_jnt_names();
    //set_ur_jnt_names_7dof();
    
    Eigen::VectorXd q_in,q_7dof_bin8_approach,q_6dof_bin8_approach;
    Eigen::VectorXd  q_7dof_bin6_pickup_pose,q_6dof_bin6_pickup_pose;
    Eigen::VectorXd  q_7dof_agv1_pickup_pose,q_6dof_agv1_pickup_pose;    
    Eigen::VectorXd q_7dof_soln,q_7dof,q_6dof;
    Eigen::Matrix4d A61;
    q_in.resize(NJNTS);
    g_q_vec.resize(NJNTS);
    

    //q_in << 0,0,0,0,0,0;
    //q_in << 0.1, -0.2, 0.3, 0.4, 0.5, 0.6;
    //g_q_vec = q_in;
    q_7dof_bin6_pickup_pose.resize(7);
    //1.85, -0.54, -0.47, 3.14, 3.33, -1.57, 1.57
    q_7dof_bin6_pickup_pose<<1.850, -0.540, -0.470, 3.140, 3.333, -1.571, 1.569;
    
    q_7dof_agv1_pickup_pose.resize(7);
    q_7dof_agv1_pickup_pose<<2.364, 2.1, -1.297, 1.57, 3.646, -1.571, 1.480;
    q_6dof_agv1_pickup_pose.resize(6);
    q_6dof_agv1_pickup_pose = fwd_solver.map726dof(q_7dof_agv1_pickup_pose);    
            
    q_6dof_bin6_pickup_pose.resize(6);
    q_6dof_bin6_pickup_pose = fwd_solver.map726dof(q_7dof_bin6_pickup_pose);
    
    q_7dof_bin8_approach.resize(7);
    q_7dof_bin8_approach<<1.5, 0.4, -0.8, 2.15, 4.0, -1.57, 0.50;
    //q_7dof_bin8_approach<<1.85, -0.535, -0.47, 3.14, 3.33, -1.57, 0.50;
    q_6dof_bin8_approach = fwd_solver.map726dof(q_7dof_bin8_approach);
    
    q_7dof_soln.resize(7);
    q_7dof.resize(7);
    //choose which pose to iterate about:
    q_7dof = q_7dof_agv1_pickup_pose; //XXXX THIS IS WHERE POSE IS SELECTED
    
    
    q_6dof = fwd_solver.map726dof(q_7dof);
    double q_rail = q_7dof[1];

   // Eigen::Affine3d affine_vacuum;
        Eigen::Affine3d A_fwd_DH = fwd_solver.fwd_kin_solve(q_6dof); //fwd_kin_solve
        // rotate DH frame6 to reconcile with URDF frame7:
        //Eigen::Affine3d A_fwd_URDF = A_fwd_DH*a_tool;
        std::cout << "q_in: " << q_6dof.transpose() << std::endl;
        std::cout << "A rot: " << std::endl;
        std::cout << A_fwd_DH.linear() << std::endl;
        std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;
        Eigen::Matrix3d R_flange = A_fwd_DH.linear();

        Eigen::Quaterniond quat(R_flange);
        std::cout<<"quat: "<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl; 
        
        std::vector<Eigen::VectorXd> q6dof_solns;
        
        int nsolns = ik_solver.ik_solve(A_fwd_DH,q6dof_solns);
        nsolns = q6dof_solns.size();
        std::cout << "number of IK solutions: " << nsolns << std::endl;   
        nsolns = fwd_solver.prune_solns_by_jnt_limits(q6dof_solns);
        
        //select the solution that is closest to some reference--try q_6dof_bin6_pickup_pose
        Eigen::VectorXd q_fit;
        q_fit = fwd_solver.closest_soln(q_6dof,q6dof_solns);
        cout<<"best fit soln: "<<q_fit.transpose()<<endl;
        

        q_7dof_soln = fwd_solver.map627dof(q_rail, q_fit);
        ROS_INFO("q7: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]",q_7dof_soln[0],
                q_7dof_soln[1],q_7dof_soln[2],q_7dof_soln[3],q_7dof_soln[4],q_7dof_soln[5],q_7dof_soln[6]);
        
        //now prompt user to adjust origin, with fixed orientation;
        Eigen::Quaterniond q;
        Eigen::Vector3d Oe;
        Oe = A_fwd_DH.translation();
        q.x() = 0;
        q.y() = 0;
        q.z() = 0;
        q.w() = 1;
        Eigen::Matrix3d Re(q);
        Eigen::Affine3d affine_des;
        affine_des.linear() = Re;
        affine_des.translation() = Oe;
        
        //orientation is now precise; re-do IK:
        nsolns = ik_solver.ik_solve(affine_des,q6dof_solns);
        nsolns = q6dof_solns.size();
        std::cout << "number of IK solutions: " << nsolns << std::endl;   
        nsolns = fwd_solver.prune_solns_by_jnt_limits(q6dof_solns);
        
        //select the solution that is closest to some reference--try q_6dof_bin6_pickup_pose
        q_fit = fwd_solver.closest_soln(q_6dof,q6dof_solns);
        cout<<"best fit soln: "<<q_fit.transpose()<<endl;
        
        q_7dof_soln = fwd_solver.map627dof(q_rail, q_fit);
        ROS_INFO("q7: [%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f]",q_7dof_soln[0],
                q_7dof_soln[1],q_7dof_soln[2],q_7dof_soln[3],q_7dof_soln[4],q_7dof_soln[5],q_7dof_soln[6]);    
        
        double dx,dy,dz;
        while (true) {
            cout<<"enter desired dx: ";
            cin>>dx;
            cout<<"enter desired dy: ";
            cin>>dy;
            cout<<"enter desired dz: ";
            cin>>dz;
            Oe[0]+=dx;
            Oe[1]+=dy;
            Oe[2]+=dz;
            cout<<"desired origin: "<<Oe.transpose()<<endl;
            affine_des.translation() = Oe;
            nsolns = ik_solver.ik_solve(affine_des,q6dof_solns);
            nsolns = q6dof_solns.size();
            std::cout << "number of IK solutions: " << nsolns << std::endl;   
            nsolns = fwd_solver.prune_solns_by_jnt_limits(q6dof_solns);
        
        //select the solution that is closest to some reference--try q_6dof_bin6_pickup_pose
        q_fit = fwd_solver.closest_soln(q_6dof,q6dof_solns);
        cout<<"best fit soln: "<<q_fit.transpose()<<endl;
        
        q_7dof_soln = fwd_solver.map627dof(q_rail, q_fit);
        ROS_INFO("q7: [%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f]",q_7dof_soln[0],
                q_7dof_soln[1],q_7dof_soln[2],q_7dof_soln[3],q_7dof_soln[4],q_7dof_soln[5],q_7dof_soln[6]);               
        }

        return 0;  //DEBUG
        Eigen::Matrix4d A_wrist;
    //***********************************************************
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size()<1) {
        ros::spinOnce(); 
        ros::Duration(0.01).sleep();
    }    
    std::cout << "==== Test for UR10  kinematics solver ====" << std::endl;
    Eigen::VectorXd q_soln;
    q_soln.resize(6);
    
    while (ros::ok()) {
        double rval;
        for (int i = 0; i < 6; i++) { //6dof joints only
            q_in[i] = g_q_vec[i]; // assign q to actual joint states
        }
        //Eigen::Vector3d reach= fwd_solver.test_w61(g_q_vec);

        Eigen::Affine3d A_fwd_DH = fwd_solver.fwd_kin_solve(q_in); //fwd_kin_solve
        // rotate DH frame6 to reconcile with URDF frame7:
        //Eigen::Affine3d A_fwd_URDF = A_fwd_DH*a_tool;
        std::cout << "q_in: " << q_in.transpose() << std::endl;
        std::cout << "A rot: " << std::endl;
        std::cout << A_fwd_DH.linear() << std::endl;
        std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;
        Eigen::Matrix3d R_flange = A_fwd_DH.linear();
        Eigen::Matrix4d A_wrist;
        Eigen::Quaterniond quat(R_flange);
        std::cout<<"quat: "<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;


        Eigen::Matrix3d R_hand;
        //Eigen::Matrix3d R_Y = Eigen::AngleAxisd(euler_Y, Eigen::Vector3d::UnitZ());
        //R_hand = Eigen::AngleAxisd(euler_Y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler_P, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_R, Eigen::Vector3d::UnitX());

        //std::cout<<"R from RPY vals: "<<std::endl;
        //std::cout<<R_hand<<std::endl;

        A_wrist = fwd_solver.get_wrist_frame();


        std::vector<Eigen::VectorXd> q6dof_solns;
        int nsolns = ik_solver.ik_solve(A_fwd_DH,q6dof_solns);


        //ik_solver.get_solns(q6dof_solns);
        nsolns = q6dof_solns.size();
        std::cout << "number of IK solutions: " << nsolns << std::endl;        
        double q_err;
        int i_min = -1;
        std::cout << "found " << nsolns << " solutions:" << std::endl;
        for (int i = 0; i < nsolns; i++) {
            Eigen::VectorXd q_soln = q6dof_solns[i];
            //ik_solver.fit_joints_to_range(q_soln);
            std::cout << q_soln.transpose() << std::endl;
            q6dof_solns[i] = q_soln;
            q_err = (q_in - q_soln).norm(); //fabs(q_in[0] - q_soln[0]) + fabs(q_in[1] - q_soln[1]) + fabs(q_in[2] - q_soln[2]);
            if (q_err < 0.000001) {
                //std::cout<<"precise fit for soln "<<i<<std::endl;
                i_min = i;
            }

            //std::cout<< "q_err: "<<q_err<<std::endl;
        }
        std::cout << "precise fit for soln " << i_min << std::endl << std::endl;
        //std::cout << "des fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
        //std::cout << "fwd kin wrist points from these solutions:" << std::endl;
        for (int i = 0; i < nsolns; i++) {
            A_fwd_DH = fwd_solver.fwd_kin_solve(q6dof_solns[i]);
            //cout<<"FK_soln R:"<<endl; //(soln "<<i<<")"<<endl;
            //cout<<A_fwd_DH.linear()<<endl; 
            Eigen::VectorXd q_soln =  q6dof_solns[i];
            std::cout << "q: " << q_soln.transpose() << std::endl;
            Eigen::Matrix3d R_flange = A_fwd_DH.linear();
            Eigen::Matrix4d A_wrist;
            Eigen::Quaterniond quat(R_flange);
            std::cout<<"   quat: "<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;
            
            cout<<"   fwd kin flange origin: "<<A_fwd_DH.translation().transpose()<<endl;
            //A_wrist = fwd_solver.get_wrist_frame();
            //qstd::cout << "fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;

        }
        //A61= fwd_solver.test_A61(g_q_vec);
        //cout<<"A61: "<<endl;
        //cout<<A61<<endl;
        /*
        double bx_wrt1, by_wrt1, bz_wrt1;
        double s234, c234, s5, c5;
        s5 = sin(g_q_vec[4]);
        c5 = cos(g_q_vec[4]);
        double q234 = g_q_vec[1]+g_q_vec[2]+g_q_vec[3];
        ROS_INFO("true q234 = %f",q234);
        
        s234 = sin(q234);
        c234 = cos(q234);
        bx_wrt1 = c234*s5;
        by_wrt1 = s234*s5;
        ROS_INFO("true bx_wrt1, by_wrt1 = %f, %f",bx_wrt1,by_wrt1);
        double q234_atan;
        q234_atan = atan2(sgn(s5)*by_wrt1,sgn(s5)*bx_wrt1);
        ROS_INFO("s5 = %f",s5);
        ROS_INFO("q234 from atan2: %f",q234_atan);
        */
        ros::Duration(1.0).sleep();
        ros::spinOnce();

    }
    return 0;
}
