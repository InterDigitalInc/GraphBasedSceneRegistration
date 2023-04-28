#Copyright © 2023 InterDigital
#All Rights Reserved
#This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).

cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)
set(DCMAKE_BUILD_TYPE Release)
project(sgRegistration)

find_package(PCL REQUIRED COMPONENTS common filters io visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED core imgproc highgui)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB SRC_FILES "src/*.cpp")

add_executable(sgRegistration ${SRC_FILES})
target_link_libraries(sgRegistration ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES})
