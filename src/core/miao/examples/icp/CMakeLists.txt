add_executable(gicp_demo gicp_demo.cc)
target_link_libraries(gicp_demo
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
        glog gflags
)