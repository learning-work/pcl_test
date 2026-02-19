cmake_minimum_required(VERSION 3.0.2)
project(pcl_pkg)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
  base
  laser_geometry
  tf2
  tf2_ros
)


find_package(PCL REQUIRED COMPONENTS
  search
  io
  common
  visualization
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_pkg
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs visualization_msgs laser_geometry tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCKUDE_DIRS}
)


## work
add_executable(pcl_test src/pcl_test.cpp)
add_dependencies(pcl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_test ${catkin_LIBRARIES})

add_executable(scanto_cloud src/scanto_cloud.cpp)
add_dependencies(scanto_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scanto_cloud ${catkin_LIBRARIES})

add_executable(KdTree src/KdTree.cpp)
add_dependencies(KdTree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KdTree ${catkin_LIBRARIES})

add_executable(ksearch src/ksearch.cpp)
add_dependencies(ksearch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ksearch ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(remove_plane src/remove_plane.cpp)
add_dependencies(remove_plane ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remove_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(identify src/identify.cpp)
add_dependencies(identify ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(identify ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map src/map.cpp)
add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Eucondition src/Eucondition.cpp)
add_dependencies(Eucondition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Eucondition ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(solidfiltered src/solidfiltered.cpp)
add_dependencies(solidfiltered ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(solidfiltered ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Voxelmap src/Voxelmap.cpp)
add_dependencies(Voxelmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Voxelmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Pillar src/Pillar.cpp)
add_dependencies(Pillar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Pillar ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(FPFH src/FPFH.cpp)
add_dependencies(FPFH ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FPFH ${catkin_LIBRARIES} ${PCL_LIBRARIES})