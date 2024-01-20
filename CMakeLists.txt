cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(kv3)#kinect azure 

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Default build type: Release" FORCE)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14) #
set(CMAKE_CXX_STANDARD_REQUIRED ON) #
set(CMAKE_CXX_EXTENSIONS OFF) #
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

find_package(PkgConfig)
find_package(k4a REQUIRED)

# ----------------------------------------------------------------------------
# Options
# ----------------------------------------------------------------------------
OPTION(ENABLE_K4ABT "Enable k4abt support" OFF)
OPTION(ENABLE_ODAS "Enable ODAS support" ON)
OPTION(ENABLE_PCL "Enable PCL support" ON)
OPTION(ENABLE_OpenCV "Enable OpenCV support" ON)
OPTION(ENABLE_OpenCvGui "Enable OpenCV GUI" ON)

if(ENABLE_OpenCV)
  add_definitions(-DUSE_CV)
  find_package(OpenCV REQUIRED)
  list(APPEND EXTRA_INCL_DIR ${OpenCV_INCLUDE_DIRS})
  list(APPEND EXTRA_LIBS_DIR ${OpenCV_LIB_DIR})  
  list(APPEND EXTRA_LIBS ${OpenCV_LIBS}) 
ENDIF(ENABLE_OpenCV)

if(ENABLE_OpenCvGui)
  add_definitions(-DUSE_CVGUI)
ENDIF(ENABLE_OpenCvGui)

if(ENABLE_PCL)
  add_definitions(-DUSE_PCL)
  find_package(PCL 1.13 REQUIRED)
  add_definitions(${PCL_DEFINITIONS})
  list(APPEND EXTRA_INCL_DIR ${PCL_INCLUDE_DIRS})
  list(APPEND EXTRA_LIBS_DIR ${PCL_LIBRARY_DIRS})
  list(APPEND EXTRA_LIBS ${PCL_LIBRARIES})   
  
  find_package(Eigen3 REQUIRED)
  list(APPEND EXTRA_INCL_DIR ${EIGEN3_INCLUDE_DIR})
  list(APPEND EXTRA_LIBS Eigen3::Eigen)
ENDIF(ENABLE_PCL)

if(ENABLE_ODAS)
  add_definitions(-DUSE_ODAS)
ENDIF(ENABLE_ODAS)

if(ENABLE_K4ABT)
  add_definitions(-DUSE_K4ABT)
  find_package(k4abt REQUIRED)
  list(APPEND EXTRA_LIBS "k4abt") 
ENDIF(ENABLE_K4ABT)


include_directories(
 include
 sup/kv3/include
 sup/audio/include
 sup/pcl/include
 ${EXTRA_INCL_DIR}
)

link_directories(
  ${EXTRA_LIBS_DIR}
)

add_executable ( kv3 src/main.cpp sup/kv3/src/kv3.cpp sup/audio/src/kv3audio.cpp sup/audio/src/kv3audiovisu.cpp sup/audio/src/kv3odas.cpp sup/audio/src/audioMeasurement.cpp sup/pcl/src/pclApp.cpp sup/pcl/src/pclUtilitys.cpp )

target_link_libraries (kv3
  ${EXTRA_LIBS}
  k4a
  k4arecord
  asound
  pthread
)
