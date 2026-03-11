//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <filesystem>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

namespace lightning {

void RosbagIO::Go(int sleep_usec) {
    std::filesystem::path p(bag_file_);
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_cpp::ConverterOptions cv_options{"cdr", "cdr"};
    reader.open({bag_file_, "sqlite3"}, cv_options);

    while (reader.has_next()) {
        auto msg = reader.read_next();
        auto iter = process_func_.find(msg->topic_name);
        if (iter != process_func_.end()) {
            iter->second(msg);
        }

        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }

        if (lightning::debug::flg_exit) {
            return;
        }
    }

    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

}  // namespace lightning