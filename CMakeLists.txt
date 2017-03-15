cmake_minimum_required(VERSION 2.8.12)
project(samaust_trackerkudan_osvr_plugin) 

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )

find_package(osvr REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("${EIGEN3_INCLUDE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")

osvr_convert_json(com_samaust_trackerkudan_osvr_json
    com_samaust_trackerkudan_osvr.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_samaust_trackerkudan_osvr_json.h")

osvr_add_plugin(NAME com_samaust_trackerkudan_osvr
    CPP
    SOURCES
    com_samaust_trackerkudan_osvr.cpp
	PositionReader.h
	PositionReader.cpp
	OrientationReader.h
	OrientationReader.cpp
	TrackerKudan.cpp
	TrackerKudan.h
	stdafx.h
	FusionMath.h
	FusionMath.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/com_samaust_trackerkudan_osvr_json.h")

target_link_libraries(com_samaust_trackerkudan_osvr osvr::osvrClientKitCpp osvr::osvrAnalysisPluginKit jsoncpp_lib)