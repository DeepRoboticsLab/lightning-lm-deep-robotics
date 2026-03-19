# find_package(glog REQUIRED) # 系统的glog版本太旧 <0.4则这样找不到
# --- 手动强行指定新版本 glog (0.6.0) ---
# 定义路径 (请确认你的新版本确实在这里)
set(MY_GLOG_DIR "/usr/local")

# 强制搜索头文件和库，禁止搜索系统默认路径
find_path(MY_GLOG_INC NAMES glog/logging.h PATHS "${MY_GLOG_DIR}/include" NO_DEFAULT_PATH)
find_library(MY_GLOG_LIB NAMES glog PATHS "${MY_GLOG_DIR}/lib" NO_DEFAULT_PATH)

if(MY_GLOG_INC AND MY_GLOG_LIB)
    message(STATUS "SUCCESS: Using Custom Glog: ${MY_GLOG_LIB}")
    # 创建一个全局唯一的 Target，强制所有模块指向这里
    if(NOT TARGET glog::glog)
        add_library(glog::glog UNKNOWN IMPORTED GLOBAL)
        set_target_properties(glog::glog PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${MY_GLOG_INC}"
            IMPORTED_LOCATION "${MY_GLOG_LIB}"
        )
    endif()
else()
    message(FATAL_ERROR "FAILED: Could not find Custom Glog at ${MY_GLOG_DIR}")
endif()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gflags REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

#if (BUILD_WITH_MARCH_NATIVE)
#    add_compile_options(-march=native)
    #else ()
if (BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
else ()
    # 仅在 x86/x86_64 架构上启用 SSE 指令集
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i386|i686")
        add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|armv8")
        # ARM64: 不加任何 SSE，可选启用 NEON（如果需要）
        # add_compile_options(-mfpu=neon)  # 可选，一般不需要
    else ()
        # 其他架构（如 RISC-V）也跳过 SSE
    endif()
endif() 
#add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
#endif ()


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDR_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        glog::glog gflags
	${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
	# glog gflags
        ${yaml-cpp_LIBRARIES}
        ${pcl_conversions_LIBRARIES}
        tbb
        ${rosbag2_cpp_LIBRARIES}
)

