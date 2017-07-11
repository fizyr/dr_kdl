cmake_minimum_required(VERSION 3.1)
project(dr_kdl)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_CXX_EXTENSIONS off)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	kdl_parser
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS sensor_msgs
	DEPENDS EIGEN3 orocos_kdl
)

include_directories(include/${PROJECT_NAME})
include_directories(SYSTEM
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/dr_kdl.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${EIGEN3_LIBRARIES}
	${orocos_kdl_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}_test_kdl   test/kdl.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_eigen test/eigen.cpp)
	target_link_libraries(${PROJECT_NAME}_test_kdl   ${PROJECT_NAME} ${catkin_LIBRARIES})
	target_link_libraries(${PROJECT_NAME}_test_eigen ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
