add_executable(incremental incremental.cc)
target_link_libraries(incremental
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
        glog gflags
)
