cmake_minimum_required(VERSION 2.8.3)
project(cwru_ariac)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(AriacLib
        src/OraclePlanner.cpp src/RobotInterface.cpp src/CameraEstimator.cpp src/BinManager.cpp
        src/OrderManager.cpp src/Cheater.cpp src/PlanningUtils.cpp src/RobotMove.cpp src/SensorManager.cpp
        src/OraclePlanningUtils.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(pose_tuner src/pose_tuner.cpp)
cs_add_executable(camera_tester src/camera_tester.cpp)
cs_add_executable(pick_and_place_demo src/pick_and_place_demo.cpp)
cs_add_executable(factory_planner src/factory_planner.cpp)
cs_add_executable(oracle_tester src/oracle_tester.cpp)
cs_add_executable(qualifier1 src/qualifier1.cpp)
cs_add_executable(qualifier2 src/qualifier2.cpp)
cs_add_executable(sensor_manager_tester src/sensor_manager_tester.cpp)
cs_add_executable(robot_move_as_tester src/robot_move_as_tester.cpp)
# the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(pose_tuner AriacLib)
target_link_libraries(camera_tester AriacLib)
target_link_libraries(pick_and_place_demo AriacLib)
target_link_libraries(factory_planner AriacLib)
target_link_libraries(oracle_tester AriacLib)
target_link_libraries(qualifier1 AriacLib)
target_link_libraries(qualifier2 AriacLib)
target_link_libraries(sensor_manager_tester AriacLib)
target_link_libraries(robot_move_as_tester AriacLib)
cs_install()
cs_export()
    
