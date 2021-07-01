cmake_minimum_required(VERSION 2.8.3)
project(sopvo)

add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")


list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdPartLib/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED) 

find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (CSparse REQUIRED)
find_package (Sophus REQUIRED)
find_package (yaml-cpp REQUIRED)
find_package (DBoW3 REQUIRED)
#find_package (LAPACK REQUIRED)
#find_package (BLAS REQUIRED)

# SDPA solver ################################################################
# make sure the SDPA has been properly installed
# sudo apt-get install liblapack-dev liblapack3 libopenblas-base libopenblas-dev
# sudo apt-get install libmumps-seq-dev
# sudo apt-get install sdpa
# sudo apt-get install libsdpa-dev
set(SDPA_INCLUDE_DIR "/usr/include")
set(SDPA_LIBRARY_DIR "/usr/lib/libsdpa.a")

# LAPACK_LIBS =  /usr/lib/atlas-base/atlas/liblapack.a
# BLAS_LIBS   =  /usr/lib/atlas-base/atlas/libblas.a

OPTION(SDPA_SOLVER "Enable SDPA solver" ON)

IF (SDPA_SOLVER)
    FIND_PACKAGE(LAPACK REQUIRED)	
    FIND_PATH(SDPA_INCLUDE sdpa_call.h ${SDPA_INCLUDE_DIR})
    IF (SDPA_INCLUDE STREQUAL "SDPA_INCLUDE-NOTFOUND")
        MESSAGE(WARNING "SDPA includes not found: please set SDPA_INCLUDE_DIR in order to enable SDPA solver")
    ELSE (SDPA_INCLUDE STREQUAL "SDPA_INCLUDE-NOTFOUND")
        INCLUDE_DIRECTORIES(${SDPA_INCLUDE})
        MESSAGE(STATUS "Found SDPA includes: " ${SDPA_INCLUDE})

        FIND_LIBRARY(SDPA_LIBRARY "sdpa" PATHS ${SDPA_LIBRARY_DIR})
        IF (SDPA_LIBRARY STREQUAL "SDPA_LIBRARY-NOTFOUND")
            MESSAGE(WARNING "SDPA library not found: please set SDPA_LIBRARY_DIR in order to enable SDPA solver")
        ELSE (SDPA_LIBRARY STREQUAL "SDPA_LIBRARY-NOTFOUND")
            MESSAGE(STATUS "Found SDPA library: " ${SDPA_LIBRARY})
		
            FIND_LIBRARY(DMUMPS_LIBRARY "dmumps_seq" PATHS ${DMUMPS_LIBRARY_DIR})
            IF (DMUMPS_LIBRARY STREQUAL "DMUMPS_SEQ_LIBRARY-NOTFOUND")

                FIND_LIBRARY(DMUMPS_LIBRARY "dmumps" PATHS ${DMUMPS_LIBRARY_DIR})
                IF (DMUMPS_LIBRARY STREQUAL "DMUMPS_LIBRARY-NOTFOUND")
                    MESSAGE(WARNING "DMUMPS library not found: please set DMUMPS_LIBRARY_DIR in order to enable SDPA solver")
	        ELSE (DMUMPS_LIBRARY STREQUAL "DMUMPS_LIBRARY-NOTFOUND")
                    MESSAGE(STATUS "Found dmumps library: " ${DMUMPS_LIBRARY})
	            SET(SDPA_SOLVER_FOUND TRUE)
	        ENDIF (DMUMPS_LIBRARY STREQUAL "DMUMP_LIBRARY-NOTFOUND")

    	    ELSE (DMUMPS_LIBRARY STREQUAL "DMUMPS_SEQ_LIBRARY-NOTFOUND")
                MESSAGE(STATUS "Found dmumps library: " ${DMUMPS_LIBRARY})
	        SET(SDPA_SOLVER_FOUND TRUE)
	    ENDIF (DMUMPS_LIBRARY STREQUAL "DMUMPS_SEQ_LIBRARY-NOTFOUND")

        ENDIF (SDPA_LIBRARY STREQUAL "SDPA_LIBRARY-NOTFOUND")
    ENDIF (SDPA_INCLUDE STREQUAL "SDPA_INCLUDE-NOTFOUND")
ELSE (SDPA_SOLVER)
    MESSAGE(WARNING "SDPA solver support is disabled.")
ENDIF (SDPA_SOLVER)



# pcl
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    sensor_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    message_generation
    message_filters
    image_transport
    tf
    )

add_message_files(
    FILES
    CorrectionInf.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    )

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )


include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/backend/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/octofeeder/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/"
    )

## adding sdpa-c 

SET(G2O_LIBS cholmod cxsparse -lg2o_cli -lg2o_core
    -lg2o_csparse_extension -lg2o_ext_freeglut_minimal -lg2o_incremental
    -lg2o_interactive -lg2o_interface -lg2o_opengl_helper -lg2o_parser
    -lg2o_simulator -lg2o_solver_cholmod -lg2o_solver_csparse
    -lg2o_solver_dense -lg2o_solver_pcg -lg2o_solver_slam2d_linear
    -lg2o_solver_structure_only -lg2o_stuff -lg2o_types_data -lg2o_types_icp
    -lg2o_types_sba -lg2o_types_sclam2d -lg2o_types_sim3 -lg2o_types_slam2d
    -lg2o_types_slam3d)

## Declare a C++ library
add_library(sopvo
    src/frontend/vo_tracking.cpp
    src/frontend/f2f_tracking.cpp
    src/frontend/feature_dem.cpp
    src/frontend/depth_camera.cpp
    src/frontend/landmark.cpp
    src/frontend/camera_frame.cpp
    src/frontend/triangulation.cpp
    src/frontend/lkorb_tracking.cpp
    src/frontend/orientation_prior.cpp
    src/frontend/optimize_in_frame.cpp

    src/visualization/rviz_frame.cpp
    src/visualization/rviz_odom.cpp
    src/visualization/rviz_path.cpp
    src/visualization/rviz_pose.cpp
    )

add_dependencies(sopvo ${catkin_EXPORTED_TARGETS})
add_dependencies(sopvo
    sopvo_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(sopvo
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${DBoW3_LIBRARIES}
    ${G2O_LIBS}
    ${PCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    
    )

IF (SDPA_SOLVER_FOUND) 
	TARGET_LINK_LIBRARIES(sopvo 
	${SDPA_LIBRARY}
	${DMUMPS_LIBRARY}
	${LAPACK_LIBRARIES}
	)
ENDIF (SDPA_SOLVER_FOUND)      

