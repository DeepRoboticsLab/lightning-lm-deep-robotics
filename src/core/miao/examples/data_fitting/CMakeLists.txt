add_executable(circle_fit circle_fit.cc)
set_target_properties(circle_fit PROPERTIES OUTPUT_NAME circle_fit)

target_link_libraries(circle_fit ${PROJECT_NAME}.core
        gflags glog
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
)

add_executable(curve_fit curve_fit.cc)
set_target_properties(curve_fit PROPERTIES OUTPUT_NAME curve_fit)

target_link_libraries(curve_fit ${PROJECT_NAME}.core
        gflags glog
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.utils
)
