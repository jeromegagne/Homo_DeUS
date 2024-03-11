cmake_minimum_required(VERSION 2.8.3)
project(homodeus_tests_prehension)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  dynamic_reconfigure
  tf
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
    PlaneCoefficients.msg
    ObjectPoseSize.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)


generate_dynamic_reconfigure_options(
  cfg/region.cfg
  cfg/segment.cfg
 )

catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure sensor_msgs message_runtime
)


include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
            src/pcl_filters.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(segment_table src/nodes/segment_table.cpp)
target_link_libraries(segment_table ${catkin_LIBRARIES})

#add_executable(detect_main_plane src/nodes/detect_main_plane.cpp)
#add_dependencies(detect_main_plane tiago_pcl_tutorial_generate_messages_cpp)
#target_link_libraries(detect_main_plane ${catkin_LIBRARIES})


add_executable(object_detector src/nodes/object_detector.cpp src/tf_transforms.cpp src/geometry.cpp src/pcl_filters.cpp)
target_link_libraries(object_detector ${PROJECT_NAME}
                                        ${catkin_LIBRARIES})

#add_executable(pcl_segment src/nodes/pcl_segment.cpp)
#add_dependencies(pcl_segment ${tiago_pcl_tutorial_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} tiago_pcl_tutorial_gencfg)
#target_link_libraries(pcl_segment ${catkin_LIBRARIES})

#add_executable(region_segment src/nodes/region_segment.cpp)
#add_dependencies(region_segment ${tiago_pcl_tutorial_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} tiago_pcl_tutorial_gencfg)
#target_link_libraries(region_segment ${catkin_LIBRARIES})

#install(TARGETS ${PROJECT_NAME} segment_table cylinder_detector region_segment detect_main_plane
install(TARGETS ${PROJECT_NAME} segment_table object_detector
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



