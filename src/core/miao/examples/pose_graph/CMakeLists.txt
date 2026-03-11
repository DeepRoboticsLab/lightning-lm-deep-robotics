add_executable(pose_graph pose_graph.cc)
target_link_libraries(pose_graph
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
        glog gflags
)
