cmake_minimum_required(VERSION 3.14)

project(HandySLAM VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HANDY_DIR_SRC ${CMAKE_SOURCE_DIR}/src)
set(HANDY_DIR_INC ${CMAKE_SOURCE_DIR}/include)
set(HANDY_DIR_SCRIPTS ${CMAKE_SOURCE_DIR}/scripts)
set(HANDY_DIR_RTABMAP ${CMAKE_SOURCE_DIR}/rtabmap)
set(HANDY_DIR_PROMPTDA ${CMAKE_SOURCE_DIR}/PromptDA)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(OpenCV REQUIRED QUIET COMPONENTS core calib3d imgproc highgui stitching photo video videoio OPTIONAL_COMPONENTS aruco objdetect xfeatures2d nonfree gpu cudafeatures2d cudaoptflow cudaimgproc)
find_package(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation)

execute_process(
  COMMAND python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
  OUTPUT_VARIABLE HANDY_DIR_PY_ENV_PURE
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
  COMMAND python3 -c "import sysconfig; print(sysconfig.get_paths()['platlib'])"
  OUTPUT_VARIABLE HANDY_DIR_PY_ENV_PLAT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

option(WITH_QT             "Include Qt support"                     OFF)
option(WITH_FREENECT       "Include Freenect support"               OFF)
option(WITH_FREENECT2      "Include Freenect2 support"              OFF)
option(WITH_K4W2           "Include Kinect for Windows v2 support"  OFF)
option(WITH_K4A            "Include Kinect for Azure support"       OFF)
option(WITH_OPENNI         "Include OpenNI support"                 OFF)
option(WITH_OPENNI2        "Include OpenNI2 support"                OFF)
option(WITH_DC1394         "Include dc1394 support"                 OFF)
option(WITH_G2O            "Include g2o support"                    OFF)
option(WITH_GTSAM          "Include GTSAM support"                  OFF)
option(WITH_MRPT           "Include MRPT support"                   OFF)
option(WITH_VERTIGO        "Include Vertigo support"                OFF)
option(WITH_POINTMATCHER   "Include libpointmatcher support"        OFF)
option(WITH_FLYCAPTURE2    "Include FlyCapture2/Triclops support"   OFF)
option(WITH_ZED            "Include ZED sdk support"                OFF)
option(WITH_ZEDOC          "Include ZED Open Capture support"       OFF)
option(WITH_REALSENSE      "Include RealSense support"              OFF)
option(WITH_REALSENSE_SLAM "Include RealSenseSlam support"          OFF)
option(WITH_REALSENSE2     "Include RealSense support"              OFF)
option(WITH_MYNTEYE        "Include mynteye-s support"              OFF)
option(WITH_OCTOMAP        "Include OctoMap support"                OFF)
option(WITH_MADGWICK       "Include Madgwick IMU filtering support" OFF)
option(WITH_FASTCV         "Include FastCV support"                 OFF)
option(WITH_OPENGV         "Include OpenGV support"                 OFF)
option(WITH_CERES          "Include Ceres support"                  OFF)
add_subdirectory(rtabmap)

add_executable(HandySLAM
  ${HANDY_DIR_SRC}/main.cpp
  ${HANDY_DIR_SRC}/PyScript.cpp
  ${HANDY_DIR_SRC}/Config.cpp
  ${HANDY_DIR_SRC}/Dataloader.cpp
  ${HANDY_DIR_SRC}/DataloaderStray.cpp
  ${HANDY_DIR_SRC}/DataloaderScanNet.cpp)

target_include_directories(HandySLAM PRIVATE
  ${HANDY_DIR_INC} 
  SYSTEM ${HANDY_DIR_RTABMAP}/corelib/include
  SYSTEM ${HANDY_DIR_RTABMAP}/utilite/include
  SYSTEM ${Python3_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS})

target_link_libraries(HandySLAM
  Python3::Python
  Python3::Module
  rtabmap::core
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

target_compile_definitions(HandySLAM PRIVATE ${PCL_DEFINITIONS})
target_compile_definitions(HandySLAM PRIVATE HANDY_W=1008)
target_compile_definitions(HandySLAM PRIVATE HANDY_H=756)
target_compile_definitions(HandySLAM PRIVATE HANDY_DIR_SCRIPTS="${HANDY_DIR_SCRIPTS}")
target_compile_definitions(HandySLAM PRIVATE HANDY_PY_SCRIPT_MODULE_NAME="PyScript")
target_compile_definitions(HandySLAM PRIVATE HANDY_DIR_PROMPTDA="${HANDY_DIR_PROMPTDA}")
target_compile_definitions(HandySLAM PRIVATE HANDY_DIR_PY_ENV_PURE="${HANDY_DIR_PY_ENV_PURE}")
target_compile_definitions(HandySLAM PRIVATE HANDY_DIR_PY_ENV_PLAT="${HANDY_DIR_PY_ENV_PLAT}")