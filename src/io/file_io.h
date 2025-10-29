//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_FILE_IO_H
#define LIGHTNING_FILE_IO_H

#include "common/eigen_types.h"
#include "common/std_types.h"

namespace lightning {

/**
 * 检查某个路径是否存在
 * @param file_path 路径名
 * @return true if exist
 */
bool PathExists(const std::string& file_path);

/**
 * 若文件存在，则删除之
 * @param path
 * @return
 */
bool RemoveIfExist(const std::string& path);

/**
 * 判断某路径是否为目录
 * @param path
 * @return
 */
bool IsDirectory(const std::string& path);

}

#endif  // LIGHTNING_FILE_IO_H
