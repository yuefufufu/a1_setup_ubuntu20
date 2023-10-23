cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    unitree_legged_msgs
    tf
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
)

find_package(PCL)
find_package(PCL 1.1 REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ~/unitree_legged_sdk/include
    ~/unitree_legged_sdk/include/unitree_legged_sdk
)

link_directories(/home/yue/unitree_legged_sdk/lib)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

set(CMAKE_CXX_FLAGS "-O3")


link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    /home/yue/catkin_ws/src/unitree_ros/unitree_controller/src/body.cpp 
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)


if( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_1")
    include_directories($ENV{ALIENGO_SDK_PATH}/include)
    link_directories($ENV{ALIENGO_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libaliengo_comm.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_1)

    add_executable(lcm_server_3_1 $ENV{ALIENGO_SDK_PATH}/examples/lcm_server.cpp)
    target_link_libraries(lcm_server_3_1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_3_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

elseif( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_2")
    include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
    link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_2)

    add_executable(lcm_server_3_2 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
    target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES} $ENV{LCM_PATH})
    add_dependencies(lcm_server_3_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(velocity_lcm src/exe/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(torque_lcm src/exe/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(walk_lcm src/exe/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_lcm src/test/test.cpp)
target_link_libraries(test_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(test_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test1_lcm src/test/test1.cpp)
target_link_libraries(test1_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(test1_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test2_lcm src/test/test2.cpp)
target_link_libraries(test2_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(test2_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test3_lcm src/test/test3.cpp)
target_link_libraries(test3_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(test3_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pub_test src/test/pub_test.cpp)
target_link_libraries(pub_test ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sub_test src/test/sub_test.cpp)
target_link_libraries(sub_test ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(sub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_move src/test/A1_move.cpp)
target_link_libraries(A1_move ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_stand src/test/A1_stand.cpp)
target_link_libraries(A1_stand ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_stand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_asibumi src/test/A1_asibumi.cpp)
target_link_libraries(A1_asibumi ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_asibumi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_hokou src/test/A1_hokou.cpp)
target_link_libraries(A1_hokou ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_hokou ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_baran src/test/A1_baran.cpp)
target_link_libraries(A1_baran ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_baran ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_baran2 src/test/A1_baran2.cpp)
target_link_libraries(A1_baran2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_baran2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_baran3 src/test/A1_baran3.cpp)
target_link_libraries(A1_baran3 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_baran3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_baran4 src/test/A1_baran4.cpp)
target_link_libraries(A1_baran4 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_baran4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(imu2tf_converter src/test/imu2tf_converter.cpp)
target_link_libraries(imu2tf_converter ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(imu2tf_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joytwist src/test/joytwist.cpp)
target_link_libraries(joytwist  ${catkin_LIBRARIES})

add_executable(joytest src/test/joytest.cpp)
target_link_libraries(joytest  ${catkin_LIBRARIES})