add_executable(sphere sphere.cc)
target_link_libraries(sphere
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
        glog gflags
)
