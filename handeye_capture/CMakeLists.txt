cmake_minimum_required(VERSION 3.16)
project(handeye_capture LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d highgui)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

# --- SDK locations -----------------------------------------------------------
# 设置SDK路径
set(ORBBEC_SDK_ROOT "$ENV{HOME}/opt_user/OrbbecSDK" CACHE PATH "OrbbecSDK root")
set(RM_API2_ROOT "$ENV{HOME}/opt_user/RM_API2/C++" CACHE PATH "RealMan API2 C++ root")

# 架构判断
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(ORBBEC_LIB_SUBDIR arm64)
    set(RM_LIB_ARCH arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(ORBBEC_LIB_SUBDIR linux_x64)
    set(RM_LIB_ARCH x86)
else()
    message(FATAL_ERROR "Unsupported processor: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 查找头文件库文件
find_path(ORBBEC_INCLUDE_DIR
    NAMES libobsensor/ObSensor.hpp
    HINTS "${ORBBEC_SDK_ROOT}/include"
    REQUIRED)
find_library(ORBBEC_LIBRARY
    NAMES OrbbecSDK
    HINTS "${ORBBEC_SDK_ROOT}/lib/${ORBBEC_LIB_SUBDIR}"
    REQUIRED)
set(ORBBEC_LIB_DIR "${ORBBEC_SDK_ROOT}/lib/${ORBBEC_LIB_SUBDIR}")

find_path(RM_INCLUDE_DIR
    NAMES rm_interface.h
    HINTS "${RM_API2_ROOT}/include"
    REQUIRED)
file(GLOB RM_LIB_CANDIDATES
    "${RM_API2_ROOT}/linux/linux_${RM_LIB_ARCH}_c++_*/libapi_cpp.so")
if(NOT RM_LIB_CANDIDATES)
    message(FATAL_ERROR
        "libapi_cpp.so not found under ${RM_API2_ROOT}/linux/linux_${RM_LIB_ARCH}_c++_*")
endif()
list(SORT RM_LIB_CANDIDATES)
list(GET RM_LIB_CANDIDATES -1 RM_LIBRARY)
get_filename_component(RM_LIB_DIR "${RM_LIBRARY}" DIRECTORY)

message(STATUS "OpenCV       : ${OpenCV_VERSION}")
message(STATUS "yaml-cpp     : ${YAML_CPP_TARGET}")
message(STATUS "OrbbecSDK    : ${ORBBEC_LIBRARY}")
message(STATUS "RealMan API2 : ${RM_LIBRARY}")

# 收集 src 下所有 cpp
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# --- Target ------------------------------------------------------------------
add_executable(handeye_capture
    main.cpp
    ${SRC_FILES}
)

target_include_directories(handeye_capture PRIVATE
    include
    ${ORBBEC_INCLUDE_DIR}
    ${RM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(handeye_capture PRIVATE
    ${OpenCV_LIBS}
    ${YAML_CPP_TARGET}
    ${ORBBEC_LIBRARY}
    ${RM_LIBRARY}
    pthread
)

target_compile_options(handeye_capture PRIVATE -Wall -Wextra -Wpedantic)


# Neither SDK is installed system-wide, so embed their paths in the binary.
set_target_properties(handeye_capture PROPERTIES
    BUILD_RPATH "${ORBBEC_LIB_DIR};${RM_LIB_DIR}"
    INSTALL_RPATH "${ORBBEC_LIB_DIR};${RM_LIB_DIR}")

# --- TEST -------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
    add_executable(test_config tests/test_config.cpp src/config.cpp)
    target_include_directories(test_config PRIVATE include)
    target_link_libraries(test_config PRIVATE ${YAML_CPP_TARGET})
    target_compile_options(test_config PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME config COMMAND test_config)

    add_executable(test_robot_rm
        tests/test_robot_rm.cpp
        src/config.cpp
        src/robot_rm.cpp
    )
    target_include_directories(test_robot_rm PRIVATE include ${RM_INCLUDE_DIR})
    target_link_libraries(test_robot_rm PRIVATE ${YAML_CPP_TARGET} ${RM_LIBRARY} pthread)
    target_compile_options(test_robot_rm PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(test_robot_rm PROPERTIES
        BUILD_RPATH "${RM_LIB_DIR}"
        INSTALL_RPATH "${RM_LIB_DIR}")

    add_executable(test_camera_orbbec
        tests/test_camera_orbbec.cpp
        src/config.cpp
        src/camera_orbbec.cpp
    )
    target_include_directories(test_camera_orbbec PRIVATE include ${ORBBEC_INCLUDE_DIR})
    target_link_libraries(test_camera_orbbec PRIVATE ${YAML_CPP_TARGET} ${ORBBEC_LIBRARY} pthread)
    target_compile_options(test_camera_orbbec PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(test_camera_orbbec PROPERTIES
        BUILD_RPATH "${ORBBEC_LIB_DIR}"
        INSTALL_RPATH "${ORBBEC_LIB_DIR}")

    add_executable(test_capture
        tests/test_capture.cpp
        src/sample_writer.cpp
        src/camera_opencv.cpp
    )
    target_include_directories(test_capture PRIVATE include ${ORBBEC_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_capture PRIVATE ${YAML_CPP_TARGET} ${OpenCV_LIBS})
    target_compile_options(test_capture PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME capture COMMAND test_capture)
endif()
