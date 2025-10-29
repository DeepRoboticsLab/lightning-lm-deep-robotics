add_executable(bal_example bal_example.cc)
target_link_libraries(bal_example
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
        glog gflags
)
