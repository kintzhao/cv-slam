cmake_minimum_required(VERSION 2.8)
       include(CheckCXXCompilerFlag)
      CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
      CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
      if(COMPILER_SUPPORTS_CXX11)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
      elseif(COMPILER_SUPPORTS_CXX0X)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
      else()
              message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
      endif()

      if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
      endif(COMMAND cmake_policy)

project(cv_slam)

find_package( OpenCV REQUIRED )
message("OPENCV_INC =   ${OpenCV_INCLUDE_DIRS}")
message("OPENCV_INC =   ${OpenCV_LIBS}")

find_package(catkin     REQUIRED        COMPONENTS
                        roscpp          rospy
                        std_msgs        message_generation
                        cv_bridge       image_transport
                        sensor_msgs     std_msgs
                        tf              geometry_msgs
                        nav_msgs        visualization_msgs)

add_message_files(  FILES          arMessage.msg  MarkGpos.msg
                                   odom.msg       robot.msg
                                   landmark.msg   systemState.msg )

generate_messages(  DEPENDENCIES   std_msgs)
catkin_package()

set(AR_SOURCE_DIR       ${PROJECT_SOURCE_DIR})
option(ARTK_USE_BOOST   "Compile with boost (OpenCV calibration file support)"      FALSE)
if(ARTK_USE_BOOST)
        add_definitions(-DAR_WITH_BOOST)
endif(ARTK_USE_BOOST)


include_directories(    ${AR_SOURCE_DIR}/ARToolkitPlus/include/ARToolKitPlus
                        ${AR_SOURCE_DIR}/ARToolkitPlus/include
                        ${AR_SOURCE_DIR}/ARToolkitPlus/include/ARToolKitPlus/extra)
#include_directories(${AR_SOURCE_DIR}/qrslam)

include_directories(    ${catkin_INCLUDE_DIRS} )
message("==catkin_INCLUDE_DIRS==: "     ${catkin_INCLUDE_DIRS})
link_directories(       ${AR_SOURCE_DIR}/lib   ${catkin_LIBRARIES} )

# ARToolkitPlus core files (headers and sources)
file(GLOB_RECURSE   HEADERS_AR
                            ${AR_SOURCE_DIR}/ARToolkitPlus/include/ARToolKitPlus/*.h)
file(GLOB           SOURCES_AR
                            ${AR_SOURCE_DIR}/ARToolkitPlus/src/*.cpp
                            ${AR_SOURCE_DIR}/ARToolkitPlus/src/core/*.cpp
                            ${AR_SOURCE_DIR}/ARToolkitPlus/src/extra/*.cpp
                            ${AR_SOURCE_DIR}/ARToolkitPlus/src/librpp/*.cpp)
file(GLOB           ROS_IMG_PUB
                            ${AR_SOURCE_DIR}/image_convert/image_converter.h
                            ${AR_SOURCE_DIR}/image_convert/image_converter.cpp)
file(GLOB_RECURSE   EKFSLAM_HEAD
                            ${AR_SOURCE_DIR}/qrslam/qrslam.cpp
                            ${AR_SOURCE_DIR}/qrslam/class/detectqrcode.cpp
                            ${AR_SOURCE_DIR}/qrslam/class/detectqrcode.h)

#file(GLOB_RECURSE   HEADERS_G2O
#                            ${AR_SOURCE_DIR}/graph_slam/class/detectqrcode.cpp
#                            ${AR_SOURCE_DIR}/graph_slam/class/detectqrcode.h
#                            ${AR_SOURCE_DIR}/graph_slam/graph_slam.cpp
#                            ${AR_SOURCE_DIR}/graph_slam/graph_slam.h
#)


#========================= g2o ================================
message("CMAKE_MODULE_PATH "    ${CMAKE_MODULE_PATH} )
set(     CMAKE_MODULE_PATH      ${CMAKE_MODULE_PATH}    "${AR_SOURCE_DIR}/graph_slam/cmake-modules/")
message("CMAKE_MODULE_PATH "    CMAKE_MODULE_PATH       ${CMAKE_MODULE_PATH})

#============ g2o =============
find_package(G2O REQUIRED)
include_directories(    ${G2O_INCLUDE_DIR})
link_directories(       ${G2O_LIBRARY_DIRS})
link_libraries(         ${G2O_LIBRARIES})

message("G2o-libraries      ${G2O_LIBRARIES}")
message("G2O_INCLUDE_DIRS   ${G2O_INCLUDE_DIR}")

#=========== CSparse ===========
find_package(CSparse REQUIRED)

include_directories(    ${CSPARSE_INCLUDE_DIR})
link_directories(       ${CSparse_LIBRARY_DIRS})
link_libraries(         ${CSPARSE_LIBRARY})

message("CSPARSE_INCLUDE_DIRS"      ${CSPARSE_INCLUDE_DIR})
message("CSparse-libraries          ${CSPARSE_LIBRARY}")

#============================ EKFSLAM + g2o =================================
add_library(EKFSLAMLibrary SHARED   ${HEADERS_AR} ${SOURCES_AR} ${ROS_IMG_PUB}
                                    ${EKFSLAM_HEAD}  ${catkin_LIBRARIES}  )
target_link_libraries( EKFSLAMLibrary ${OpenCV_LIBS} ${catkin_LIBRARIES}
                                    ${G2O_LIBRARIES}  g2o_core g2o_types_slam2d g2o_solver_csparse
                                    g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

add_executable(SLAM_main            ${AR_SOURCE_DIR}/qrslam/main.cpp )
target_link_libraries(SLAM_main      EKFSLAMLibrary  )

add_executable(SLAM_view            ${AR_SOURCE_DIR}/view/view.cpp )
target_link_libraries(SLAM_view     ${OpenCV_LIBS} ${catkin_LIBRARIES} )


add_executable(camera_calibration   ${AR_SOURCE_DIR}/camera_calibration/calibrate.cpp
                                    ${AR_SOURCE_DIR}/camera_calibration/class/CameraCalibrator.h
                                    ${AR_SOURCE_DIR}/camera_calibration/class/CameraCalibrator.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS} EKFSLAMLibrary ${catkin_LIBRARIES}  )

add_executable(img_tran             ${AR_SOURCE_DIR}/new_node/main.cpp )
target_link_libraries(img_tran      ${OpenCV_LIBS} EKFSLAMLibrary ${catkin_LIBRARIES}  )

add_executable(vel_odom             ${AR_SOURCE_DIR}/qrslam/pub/velpub.cpp )
target_link_libraries(vel_odom      ${catkin_LIBRARIES}  )

add_executable(velControl           ${AR_SOURCE_DIR}/qrslam/pub/velcontrol.cpp
                                    ${AR_SOURCE_DIR}/qrslam/pub/velcontrol.h)
target_link_libraries(velControl    ${catkin_LIBRARIES}  )
