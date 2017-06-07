//
// Created by shipei on 4/25/17.
//

#include <SensorManager.h>
#include <OrderManager.h>

int main(int argc, char **argv) {
//    double speed_acc = 0;
    int index;
    if (argv[1] != NULL) {
        index = atoi(argv[1]);
    } else {
        cout
                << "Usage: rosrun cwru_ariac sensor_manager_tester [part_id] # the id of the part you want to keep tracking (positive integer)"
                << endl;
        return 0;
    }
    ros::init(argc, argv, "sensor_manager_tester");
    ros::NodeHandle nh;
    SensorManager sensorManager(nh);
    OrderManager orderManager(nh);
    sensorManager.addCamera("/ariac/logical_camera_1");
    sensorManager.addCamera("/ariac/logical_camera_2");
    sensorManager.addCamera("/ariac/logical_camera_3");
    sensorManager.addCamera("/ariac/logical_camera_4");
    sensorManager.addCamera("/ariac/logical_camera_5");
    sensorManager.addCamera("/ariac/logical_camera_6");
//    sensorManager.addCamera("/ariac/logical_camera_7");
    orderManager.startCompetition();
//    auto start = chrono::steady_clock::now();
    while (ros::ok()) {
//        sensorManager.forceUpdate();
        auto part_list = sensorManager.combineLocations(
                SensorManager::BINS | SensorManager::GROUND | SensorManager::AGVS | SensorManager::CONVEYOR);
        auto part = findPart(part_list, index);
        if (part != part_list.end()) {
            cout << "Id: " << part->id << endl
                 << "Name: " << part->name << endl
                 << "Traceable: " << (part->traceable ? "yes" : "no") << endl
                 << "Location: " << locationToName(part->location) << endl;
            ROS_INFO_STREAM("Pose:\n" << part->pose);
            ROS_INFO_STREAM("Linear:\n" << part->linear);
//            auto end = chrono::steady_clock::now();
//            double diff = chrono::duration_cast<chrono::seconds>(end - start).count();
//            speed_acc += abs(part->linear.x) + abs(part->linear.y) + abs(part->linear.z);
//            cout << "total speed error: " << speed_acc << ", " << (speed_acc / diff) * 1000 << " err/1000s" << endl;
        } else {
            cout << "Id: " << atoi(argv[1]) << " is out of view" << endl;
        }
        cout << "inView size: " << sensorManager.inView.size() << endl;
        cout << "onConveyor size: " << sensorManager.onConveyor.size() << ", laser scanner conveyor size: "
             << sensorManager.laserScannerConveyor.size() << endl;
        for (int i = 0; i < sensorManager.onBin.size(); ++i) {
//            cout << "onBin " << i + 1 << ": " << sensorManager.onBin[i].size() << endl;
        }
        for (int i = 0; i < sensorManager.onAGV.size(); ++i) {
            cout << "onAGV " << i + 1 << ": " << sensorManager.onAGV[i].size() << endl;
        }
        cout << "onGround: " << sensorManager.onGround.size() << endl;
        ros::Duration(0.5).sleep();
    }
}
