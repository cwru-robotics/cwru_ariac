//
// Created by shipei on 10/18/16.
//
// So many hard coded stuffs QAQ


#ifndef CWRU_ARIAC_ARIACBASE_H
#define CWRU_ARIAC_ARIACBASE_H


#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <mutex>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <osrf_gear/Order.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/AGVControl.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <cwru_ariac/AGV.h>
#include <cwru_ariac/Bin.h>
#include <cwru_ariac/BoundBox.h>
#include <cwru_ariac/Dimension.h>
#include <cwru_ariac/Grid.h>
#include <cwru_ariac/Part.h>
#include <cwru_ariac/PartType.h>
#include <cwru_ariac/RobotState.h>
#include <cwru_ariac/RobotMoveAction.h>
#include <cwru_ariac/OracleQuery.h>

#include <IDGenerator.h>

using namespace std;
using namespace cwru_ariac;

// overlaod hash function for Part to use unordered set
namespace std {
    template <> struct hash<Part> {
        typedef Part      argument_type;
        typedef std::size_t  result_type;
        result_type operator()(const Part & t) const {
            std::size_t val = 0;
            boost::hash_combine(val, hash<int>{}(t.id));
            boost::hash_combine(val, hash<string>{}(t.name));
            return val;
        }
    };
}
// hash set requires operator == to insert and erase
namespace cwru_ariac {
    template<typename ContainerAllocator> bool operator ==(
            const ::cwru_ariac::Part_<ContainerAllocator> & a, const ::cwru_ariac::Part_<ContainerAllocator> & b) {
        return (a.id == b.id) && (a.name == b.name);}
}

typedef unordered_set<Part> PartSet;    // for faster searching
typedef vector<Part> PartList;          // normal container
typedef list<Part> PartLinkedList;      // for some applications

static double _placeHolderDouble;   // declared for default parameter, please ignore
static int _placeHolderInt;         // declared for default parameter, please ignore

const int gridNumber = 60;
const int totalPartsTypes = 8;
const int totalAGVs = 2;
const int totalBins = 8;
const string defaultPartsName[totalPartsTypes] = {"piston_rod_part", "gear_part", "pulley_part", "gasket_part",
                                                  "part1", "part2", "part3", "part4"};
const double defaultPartsSize[totalPartsTypes][2] = {{0.059,0.052}, {0.078425,0.078425}, {0.23392,0.23392}, {0.31442,0.15684},
                                                     {0.3,0.1}, {0.06,0.015}, {0.13,0.07}, {0.09,0.06}};

// simple template find part by id
template<typename T> inline typename T::iterator findPart(T& parts, int id) {
    return find_if(parts.begin(), parts.end(), [id](Part p){return p.id == id;});
}
// simple template find parts by type
template<typename T> inline PartList findPart(T& parts, string type) {
    PartList result;
    for (auto p: parts) {
        if(p.name == type){
            result.push_back(p);
        }
    }
    return result;
}

template<typename T> inline typename T::iterator findBin(T& bins, string name) {
    return find_if(bins.begin(), bins.end(), [name](Bin bin){return bin.name == name;});
}
inline string locationToName(int32_t location) {
    if (location == Part::UNASSIGNED)
        return string("unassigned");
    if (location == Part::CAMERA)
        return string("camera");
    if (location == Part::CONVEYOR)
        return string("conveyor");
    if (location == Part::GROUND)
        return string("ground");
    if (location >= Part::AGV && location < Part::BIN)
        return string("AGV") + to_string(location - Part::AGV);
    if (location >= Part::BIN && location <= Part::BIN8)
        return string("BIN") + to_string(location - Part::BIN);
    return string("unassigned");
}

inline std::vector<double> quatToEuler(geometry_msgs::Quaternion orientation) {
    std::vector<double> euler(3);
    const double PI_OVER_2 = M_PI * 0.5;
    const double EPSILON = 1e-10;

    // quick conversion to Euler angles to give tilt to user
    double sqw = orientation.w*orientation.w;
    double sqx = orientation.x*orientation.x;
    double sqy = orientation.y*orientation.y;
    double sqz = orientation.z*orientation.z;

    euler[1] = asin(2.0 * (orientation.w*orientation.y - orientation.x*orientation.z));
    if (PI_OVER_2 - fabs(euler[1]) > EPSILON) {
        euler[2] = atan2(2.0 * (orientation.x*orientation.y + orientation.w*orientation.z),
                         sqx - sqy - sqz + sqw);
        euler[0] = atan2(2.0 * (orientation.w*orientation.x + orientation.y*orientation.z),
                         sqw - sqx - sqy + sqz);
    } else {
        // compute heading from local 'down' vector
        euler[2] = atan2(2*orientation.y*orientation.z - 2*orientation.x*orientation.w,
                         2*orientation.x*orientation.z + 2*orientation.y*orientation.w);
        euler[0] = 0.0;

        // If facing down, reverse yaw
        if (euler[1] < 0)
            euler[2] = M_PI - euler[2];
    }
    return euler;
}

inline double euclideanDistance(geometry_msgs::Point positionA, geometry_msgs::Point positionB) {
    return sqrt(pow(positionA.x - positionB.x, 2) + pow(positionA.y - positionB.y, 2) + pow(positionA.z - positionB.z, 2));
}

inline double euclideanDistance2D(geometry_msgs::Point positionA, geometry_msgs::Point positionB) {
    return sqrt(pow(positionA.x - positionB.x, 2) + pow(positionA.y - positionB.y, 2));
}

inline bool checkBound(geometry_msgs::Point position, BoundBox boundBox) {
    bool x = (boundBox.Xmin <= position.x && position.x <= boundBox.Xmax);
    bool y = (boundBox.Ymin <= position.y && position.y <= boundBox.Ymax);
    bool z = (boundBox.Zmin <= position.z && position.z <= boundBox.Zmax);
    return x & y & z;
}

inline bool evalUpDown(geometry_msgs::Quaternion orientation) {
    double qx = orientation.x;
    double qy = orientation.y;
    double sum_sqd = qx * qx + qy * qy;
    return sum_sqd > 0.5;
}

inline double unifyAngle(double input) {
    input = fmod(input, M_PI * 2);
    return (input > M_PI) ? input - 2 * M_PI : input; //assumes fmod always returns non-negative value
}

inline double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

inline bool matchPose(geometry_msgs::Pose A,geometry_msgs::Pose B) {
    // set tolerances of part placements on tray (per scoring)
    // only check position and yaw and part flip as the scorer only care about these
    bool distance = fabs(euclideanDistance2D(A.position, B.position)) < 0.027;  // 0.03 (old)
    bool yaw = unifyAngle(fabs(convertPlanarQuat2Phi(A.orientation) - convertPlanarQuat2Phi(B.orientation))) < 0.09;
    bool upDown = (evalUpDown(A.orientation) == evalUpDown(B.orientation));
//    ROS_WARN("match pose, upDown eval: %s, A: %s, B: %s", upDown? "true":"false", evalUpDown(A.orientation)? "up":"down", evalUpDown(B.orientation)? "up":"down");
//    ROS_WARN_STREAM("Pose A:" << A);
//    ROS_WARN_STREAM("Pose B:" << B);
//    ROS_WARN("A yaw:%f, B yaw:%f", convertPlanarQuat2Phi(A.orientation), convertPlanarQuat2Phi(B.orientation));
    return distance & upDown & yaw;// roll & pitch & yaw;
}

inline bool isSamePart(Part A, Part B) {
    bool name = (A.name == B.name);
    bool x = true;  // ignore linear (moving speed) if not traceable
    bool y = true;
    bool z = true;
    bool location = true;// (A.location == B.location);
    if (A.traceable && B.traceable) {
        x = fabs(A.linear.x - B.linear.x) < 0.03;  // 0.03 (old)
        y = fabs(A.linear.y - B.linear.y) < 0.03;
        z = fabs(A.linear.z - B.linear.z) < 0.03;
    }
    bool pose = matchPose(A.pose.pose, B.pose.pose);
    return name & location & x & y & z & pose;
}

inline double speedFilter(double currentPosition, double lastPosition, double lastSpeed, double dt) {
    const double gain = 0.1;        // tune this
    if (lastSpeed != 0) {
        double newSpeed = (currentPosition - lastPosition) / dt;
        newSpeed = lastSpeed + gain * (newSpeed - lastSpeed);
        return newSpeed;
    }
    return (currentPosition - lastPosition) / dt;
}

// this class is used to storage some basic information of the competition and common algorithms.
class AriacBase {
protected:
    unordered_map<string, PartType> defaultParts;
    Bin defaultBin;
    BoundBox binBoundBox[totalBins];
    BoundBox agvBoundBox[totalAGVs];
    BoundBox conveyorBoundBox;

    AriacBase() {
        defaultBin.name = "Bin";
        defaultBin.grid.x = 60;
        defaultBin.grid.y = 60;
        defaultBin.size.x = 0.6;
        defaultBin.size.y = 0.6;

        agvBoundBox[0].Xmin = 0.0;
        agvBoundBox[0].Ymin = 2.7;
        agvBoundBox[0].Zmin = 0.5;
        agvBoundBox[0].Xmax = 0.7;
        agvBoundBox[0].Ymax = 3.9;
        agvBoundBox[0].Zmax = 0.9;
        
        agvBoundBox[1].Xmin = 0.0;
        agvBoundBox[1].Ymin = -3.9;
        agvBoundBox[1].Zmin = 0.5;
        agvBoundBox[1].Xmax = 0.7;
        agvBoundBox[1].Ymax = -2.7;
        agvBoundBox[1].Zmax = 0.9;

        conveyorBoundBox.Xmin = 0.9;
        conveyorBoundBox.Ymin = -4.8;
        conveyorBoundBox.Xmax = 1.6;
        conveyorBoundBox.Ymax = 5.8;
        conveyorBoundBox.Zmin = 0.5;
        conveyorBoundBox.Zmax = 2;

        binBoundBox[0].Xmin = -1.000000 - 0.3;
        binBoundBox[0].Ymin = -1.330000 - 0.3;
        binBoundBox[0].Xmax = -1.000000 + 0.3;
        binBoundBox[0].Ymax = -1.330000 + 0.3;

        binBoundBox[1].Xmin = -1.000000 - 0.3;
        binBoundBox[1].Ymin = -0.535000 - 0.3;
        binBoundBox[1].Xmax = -1.000000 + 0.3;
        binBoundBox[1].Ymax = -0.535000 + 0.3;

        binBoundBox[2].Xmin = -1.00000 - 0.3;
        binBoundBox[2].Ymin = 0.230000 - 0.3;
        binBoundBox[2].Xmax = -1.00000 + 0.3;
        binBoundBox[2].Ymax = 0.230000 + 0.3;

        binBoundBox[3].Xmin = -1.00000 - 0.3;
        binBoundBox[3].Ymin = 0.995000 - 0.3;
        binBoundBox[3].Xmax = -1.00000 + 0.3;
        binBoundBox[3].Ymax = 0.995000 + 0.3;

        binBoundBox[4].Xmin = -0.300000 - 0.3;
        binBoundBox[4].Ymin = -1.330000 - 0.3;
        binBoundBox[4].Xmax = -0.300000 + 0.3;
        binBoundBox[4].Ymax = -1.330000 + 0.3;

        binBoundBox[5].Xmin = -0.300000 - 0.3;
        binBoundBox[5].Ymin = -0.535000 - 0.3;
        binBoundBox[5].Xmax = -0.300000 + 0.3;
        binBoundBox[5].Ymax = -0.535000 + 0.3;

        binBoundBox[6].Xmin = -0.30000 - 0.3;
        binBoundBox[6].Ymin = 0.230000 - 0.3;
        binBoundBox[6].Xmax = -0.30000 + 0.3;
        binBoundBox[6].Ymax = 0.230000 + 0.3;

        binBoundBox[7].Xmin = -0.30000 - 0.3;
        binBoundBox[7].Ymin = 0.995000 - 0.3;
        binBoundBox[7].Xmax = -0.30000 + 0.3;
        binBoundBox[7].Ymax = 0.995000 + 0.3;

        for (int j = 0; j < totalBins; ++j) {
            binBoundBox[j].Zmin = 0.5;
            binBoundBox[j].Zmax = 2.0;
        }

        PartType singlePart;
        for (int i = 0; i < totalPartsTypes; ++i) {
            singlePart.name = defaultPartsName[i];
            singlePart.size.x = defaultPartsSize[i][0];
            singlePart.size.y = defaultPartsSize[i][1];
            singlePart.grid.x = (int)ceil(defaultBin.size.x / singlePart.size.x);
            singlePart.grid.y = (int)ceil(defaultBin.size.y / singlePart.size.y);
            defaultParts.insert(make_pair(defaultPartsName[i], singlePart));
        }
    }
};

#endif //CWRU_ARIAC_ARIACBASE_H

// hidden unused stuffs
//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
