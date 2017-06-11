// RN
// Laser Scanner Integration
// 15/5/17

#include <LaserScanner.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "laser_scanner_test");
    ros::NodeHandle nh; 

    LaserScanner LaserScannerTest(nh);

    ROS_INFO("laser_scanner_test..");
    // LaserScannerTest.status_update_count = 0;

    // clean the list first
    LaserScannerTest.conveyor_list_clean(LaserScannerTest.conveyor_partlist);

    while(ros::ok()){


        LaserScannerTest.forceUpdate();

        LaserScannerTest.check_exp();

	LaserScannerTest.conveyor_list_status(LaserScannerTest.conveyor_partlist);

	LaserScannerTest.conveyor_list_blackbox(LaserScannerTest.conveyor_partlist);
    
    }
    return 0; 
    
}

