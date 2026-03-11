//
// Created by gaoxiang on 2020/5/13.
//

#ifndef LIGHTNING_YAML_IO_H
#define LIGHTNING_YAML_IO_H

#include <yaml-cpp/yaml.h>
#include <cassert>
#include <iostream>
#include <string>

namespace lightning {

/// 读取yaml配置文件的相关IO
class YAML_IO {
   public:
    explicit YAML_IO(const std::string &path);

    YAML_IO() = default;
    ~YAML_IO() = default;

    inline bool IsOpened() const { return is_opened_; }

    /// 保存文件，不指明路径时，覆盖原文件
    bool Save(const std::string &path = "");

    /// 获取类型为T的参数值
    template <typename T>
    T GetValue(const std::string &key) const {
        assert(is_opened_);
        return yaml_node_[key].as<T>();
    }

    /// 获取在NODE下的key值
    // 读取两层yaml参数
    template <typename T>
    T GetValue(const std::string &node, const std::string &key) const {
        assert(is_opened_);
        auto n = yaml_node_[node];
        if (!n || n.IsScalar()) {
            std::cerr << "Key " << node << " is not a map or not exist." << std::endl;
            throw std::runtime_error("YAML Key error");
        }
        T res = n[key].as<T>();
        return res;
    }
    // 读取三层yaml参数
    template <typename T>
    T GetValue(const std::string &node_1, const std::string &node_2, const std::string &key) const {
        assert(is_opened_);
        auto n1 = yaml_node_[node_1];
        if (!n1 || n1.IsScalar()) {
            std::cerr << "Key " << node_1 << " is not a map or not exist." << std::endl;
            throw std::runtime_error("YAML Key error");
        }
        auto n2 = n1[node_2];
        if (!n2 || n2.IsScalar()) {
            std::cerr << "Key " << node_2 << " is not a map or not exist in " << node_1 << std::endl;
            throw std::runtime_error("YAML Key error");
        }
        T res = n2[key].as<T>();
        return res;
    }

    /// 获取带有默认值的参数
    template <typename T, typename D>
    T GetValue(const std::string &node, const std::string &key, const D &default_val) const {
        if (!is_opened_) return T(default_val);
        try {
            auto n = yaml_node_[node];
            if (!n || n.IsScalar()) {
                return T(default_val);
            }
            auto k = n[key];
            if (!k) {
                return T(default_val);
            }
            return k.as<T>();
        } catch (...) {
            return T(default_val);
        }
    }

    /// 设定类型为T的参数值
    template <typename T>
    void SetValue(const std::string &key, const T &value) {
        yaml_node_[key] = value;
    }

    /// 设定NODE下的key值
    template <typename T>
    void SetValue(const std::string &node, const std::string &key, const T &value) {
        yaml_node_[node][key] = value;
    }

    const YAML::Node &yaml_node() const { return yaml_node_; }

   private:
    std::string path_;
    bool is_opened_ = false;
    YAML::Node yaml_node_;
};

}  // namespace lightning

#endif
