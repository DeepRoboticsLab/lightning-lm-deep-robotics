add_library(miao.utils SHARED
        sampler.cc
)

target_link_libraries(miao.utils
        glog::glog
)