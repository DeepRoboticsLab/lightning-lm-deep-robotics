add_executable(sba_demo sba_demo.cc)
set_target_properties(sba_demo PROPERTIES OUTPUT_NAME sba_demo)

target_link_libraries(sba_demo ${PROJECT_NAME}.core
        gflags glog
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
)
