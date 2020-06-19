cmake_minimum_required(VERSION 2.8)
project(ImageProcessing_Tools)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wextra -pedantic")

add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
set(${PYTHONHOME} ${PYTHON_INCLUDE_DIRS})
include_directories(${PYTHONHOME}/include)
include_directories(${PYTHONHOME}/Lib/site-packages/numpy/core/include)
link_directories(${PYTHONHOME}/libs)
add_definitions(-DMATPLOTLIBCPP_PYTHON_HEADER=Python.h)
include_directories(include/plot_tools)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4.6 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        include
        src
)


find_package(Boost COMPONENTS filesystem program_options system REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})

add_library(camera_model SHARED
        src/camera_models/chessboard/Chessboard.cc
        src/camera_models/calib/CameraCalibration.cc
        src/camera_models/camera_models/Camera.cc
        src/camera_models/camera_models/CameraFactory.cc
        src/camera_models/camera_models/CostFunctionFactory.cc
        src/camera_models/camera_models/PinholeCamera.cc
        src/camera_models/camera_models/CataCamera.cc
        src/camera_models/camera_models/EquidistantCamera.cc
        src/camera_models/camera_models/ScaramuzzaCamera.cc
        src/camera_models/sparse_graph/Transform.cc
        src/camera_models/gpl/gpl.cc
        src/camera_models/gpl/EigenQuaternionParameterization.cc)
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})



set(SRC_FILES
        src/io_tools.cpp
        src/parameters.cpp
        )


#TODO 相机标定工具--------------------------------------------------------------------------
add_executable(cam_calib
        APPS/cam_calib.cpp
        ${SRC_FILES}
        )
target_link_libraries(
        cam_calib
        ${OpenCV_LIBS}
)


#TODO 图像降采样工具--------------------------------------------------------------------------
add_executable(img_downsample
        APPS/downSample.cpp
        ${SRC_FILES}
        )

target_link_libraries(
        img_downsample
        ${OpenCV_LIBS}
)

#TODO 图像平滑工具--------------------------------------------------------------------------
add_executable(l0_gradient_smooth APPS/l0_minimization.cpp ${SRC_FILES})
target_link_libraries(l0_gradient_smooth
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${OpenCV_LIBS}
        )


#TODO 图像特征追踪--------------------------------------------------------------------------

add_executable(feature_tracker APPS/main_feature_tracker.cpp
        ${SRC_FILES}
        src/feature_tracker/feature_tracker.cpp
        )

target_link_libraries(feature_tracker
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${OpenCV_LIBS}
        camera_model
        )

#TODO 图像去畸变--------------------------------------------------------------------------
add_executable(quardTree_undistorter
        ${SRC_FILES}

        APPS/main_quardTree_undistorter.cpp)

target_link_libraries(quardTree_undistorter
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${OpenCV_LIBS}
        camera_model
        )








