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

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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
#include <cwru_ariac/Parts.h>
#include <cwru_ariac/PartType.h>
#include <cwru_ariac/RobotState.h>
#include <cwru_ariac/RobotMoveAction.h>
#include <cwru_ariac/OracleQuery.h>

using namespace std;
using namespace Eigen;
using namespace cwru_ariac;

// overlaod hash function for Part to use unordered set
namespace std {
    template <> struct hash<Part>
    {
        typedef Part      argument_type;
        typedef std::size_t  result_type;
        result_type operator()(const Part & t) const {
            std::size_t val { 0 };
            boost::hash_combine(val,hash<int>{}(t.id));
            boost::hash_combine(val,hash<string>{}(t.name));
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

typedef unordered_set<Part> PartSet;  // int is id of the part, Part is part object, using map for fast search
typedef vector<Part> PartList;

static double _fakeDouble;   // declared for default parameter, please ignore
static int _fakeInt;         // declared for default parameter, please ignore

const int gridNumber = 60;
const int totalPartsTypes = 8;
const int totalAGVs = 2;
const int totalBins = 8;
const double averageCost = 1000;
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

template<typename T> inline typename T::iterator findBin(T& bins, int id) {
    string name = "Bin" + id;
    return findBinByName(bins, name);
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

inline double euclideanDistance(geometry_msgs::Point positionA, geometry_msgs::Point positionB) {
    return sqrt(pow(positionA.x - positionB.x, 2) + pow(positionA.y - positionB.y, 2) + pow(positionA.z - positionB.z, 2));
}
inline bool checkBound(geometry_msgs::Point position, BoundBox boundBox) {
    return (boundBox.Xmin<=position.x && position.x<=boundBox.Xmax) && (boundBox.Ymin <= position.y && position.y <= boundBox.Ymax) && (boundBox.Zmin <= position.z && position.z <= boundBox.Zmax);
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
        agvBoundBox[0].Xmax = 0.7;
        agvBoundBox[0].Ymax = 3.9;

        conveyorBoundBox.Xmin = 0.9;
        conveyorBoundBox.Ymin = -4.8;
        conveyorBoundBox.Xmax = 1.6;
        conveyorBoundBox.Ymax = 5.8;

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
public:
};

#endif //CWRU_ARIAC_ARIACBASE_H

//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/server/simple_action_server.h>
//#include <actionlib/client/terminal_state.h>
// 一些垃圾代码就藏在这里了
//enum Direction {
//    top, bottom, left, right,
//    topLeft, topRight, bottomLeft, bottomRight
//};
//class GridMap: public unordered_set<Grid, boost::hash<pair<int, int>>> {
//public:
//    bool checkCollision(Grid grid) {
//        return find(grid)==end();
//    }
//
//    bool addGrids(Grid start, Grid size, Direction direction) {
//        Grid A;
//        switch (direction) {
//            case topLeft:
//                A.first = start.first - size.first;
//                A.second = start.second;
//                break;
//            case topRight:
//                A = start;
//                break;
//            case bottomLeft:
//                A.first = start.first - size.first;
//                A.second = start.second - size.second;
//                break;
//            case bottomRight:
//                A.first = start.first;
//                A.second = start.second - size.second;
//                break;
//        }
//        return addGrids(A, size);
//    }
//
//    bool addGrids(Grid start, Grid size) {
//        if (start.first >= 0 && start.second >= 0 && size.first >= 0 && size.second >= 0 &&
//                start.first < gridNumber && start.second < gridNumber && size.first < gridNumber && size.second < gridNumber) {
//            for (int i = start.first; i < size.first; ++i) {
//                for (int j = start.second; j < size.second; ++j) {
//                    if (!checkCollision(Grid(i, j))) {
//                        insert(Grid(i, j));
//                    } else {
//                        return false;
//                    }
//                }
//            }
//        } else {
//            return false;
//        }
//        return true;
//    }
//};
