cmake_minimum_required(VERSION 2.8.3)
project(dr_kdl)

find_package(catkin REQUIRED COMPONENTS dr_util kdl_parser sensor_msgs cmake_modules urpp)
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES ${PROJECT_NAME}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -Wall -Wextra")

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/dr_kdl.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.hpp"
)
