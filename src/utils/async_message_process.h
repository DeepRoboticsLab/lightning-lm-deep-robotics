//
// Created by xiang on 2022/2/9.
//

#pragma once

#include <glog/logging.h>
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "common/eigen_types.h"
#include "common/std_types.h"

namespace lightning {

/**
 * 异步消息处理类
 * 内部有线程和队列机制，保证回调是串行的
 * 主要用于分离各模块线程，避免写一堆锁或者条件变量
 *
 * @tparam T
 *
 * NOTE skip设为1的时候实际不会跳帧。。设为2的时候实际走一帧跳一帧
 */
template <typename T>
class AsyncMessageProcess {
   public:
    using ProcFunc = std::function<void(const T&)>;  // 消息回调函数
    AsyncMessageProcess() = default;
    AsyncMessageProcess(ProcFunc proc_func, std::string name = "");
    ~AsyncMessageProcess() { Quit(); }

    /// 设置处理函数
    void SetProcFunc(ProcFunc proc_func) { custom_func_ = proc_func; }

    /// 设置队列最大长度
    void SetMaxSize(size_t size) { max_size_ = size; }
    /// Optional bound on queued payload; configure before Start().
    void SetMaxBytes(size_t bytes) { max_bytes_ = bytes; }

    /// 开始处理消息
    void Start();

    /// 添加一条消息
    void AddMessage(const T& msg, size_t bytes = sizeof(T));

    /// 退出
    void Quit();
    void WaitUntilIdle();

    /// 清空跳帧计数器，下一个数据会立即执行
    void CleanSkipCnt();

    void SetName(std::string name) { name_ = std::move(name); }
    void SetSkipParam(bool enable_skip, int skip_num) { enable_skip_ = enable_skip, skip_num_ = skip_num; }

    AsyncMessageProcess(const AsyncMessageProcess&) = delete;
    void operator=(const AsyncMessageProcess&) = delete;

   private:
    void ProcLoop();

    std::thread proc_;
    std::mutex mutex_;
    std::condition_variable cv_msg_;
    std::condition_variable cv_idle_;
    bool processing_ = false;
    struct Entry { T message; size_t bytes; };
    std::deque<Entry> msg_buffer_;
    size_t queued_bytes_ = 0;
    size_t max_bytes_ = std::numeric_limits<size_t>::max();
    bool exit_flag_ = false;
    size_t max_size_ = 40;
    std::string name_;

    /// 跳帧
    bool enable_skip_ = false;
    int skip_num_ = 0;
    int skip_cnt_ = 0;

    ProcFunc custom_func_;
};

template <typename T>
void AsyncMessageProcess<T>::CleanSkipCnt() {
    UL lock(mutex_);
    skip_cnt_ = 0;
}

template <typename T>
AsyncMessageProcess<T>::AsyncMessageProcess(AsyncMessageProcess::ProcFunc proc_func, std::string name) {
    custom_func_ = std::move(proc_func);
    name_ = name;
}

template <typename T>
void AsyncMessageProcess<T>::Start() {
    exit_flag_ = false;
    proc_ = std::thread([this]() { ProcLoop(); });
}

template <typename T>
void AsyncMessageProcess<T>::ProcLoop() {
    while (true) {
        UL lock(mutex_);
        cv_msg_.wait(lock, [this]() { return exit_flag_ || !msg_buffer_.empty(); });
        if (exit_flag_ && msg_buffer_.empty()) break;

        // Retain only one in-flight message, so completed clouds are released
        // immediately instead of remaining alive for a whole queued batch.
        auto entry = std::move(msg_buffer_.front());
        msg_buffer_.pop_front();
        queued_bytes_ -= entry.bytes;
        processing_ = true;
        lock.unlock();

        custom_func_(entry.message);
        lock.lock();
        processing_ = false;
        cv_idle_.notify_all();
    }
}

template <typename T>
void AsyncMessageProcess<T>::AddMessage(const T& msg, size_t bytes) {
    UL lock(mutex_);
    if (exit_flag_) return;
    if (enable_skip_) {
        if (skip_cnt_ != 0) {
            skip_cnt_++;
            skip_cnt_ = skip_cnt_ % skip_num_;
            return;
        }

        skip_cnt_++;
        skip_cnt_ = skip_cnt_ % skip_num_;
    }

    if (bytes > max_bytes_ || max_size_ == 0) {
        LOG(ERROR) << name_ << " queue overflow: message exceeds queue limit";
        return;
    }
    size_t dropped = 0;
    while (!msg_buffer_.empty() &&
           (msg_buffer_.size() >= max_size_ || queued_bytes_ > max_bytes_ - bytes)) {
        queued_bytes_ -= msg_buffer_.front().bytes;
        msg_buffer_.pop_front();
        ++dropped;
    }
    if (dropped) LOG(ERROR) << name_ << " queue overflow: dropped " << dropped
                            << " oldest messages; reduce input rate or processing load";
    msg_buffer_.push_back({msg, bytes});
    queued_bytes_ += bytes;
    cv_msg_.notify_one();
}

template <typename T>
void AsyncMessageProcess<T>::Quit() {
    {
        UL lock(mutex_);
        exit_flag_ = true;
    }
    cv_msg_.notify_one();

    if (proc_.joinable()) {
        proc_.join();
    }
}

template <typename T>
void AsyncMessageProcess<T>::WaitUntilIdle() {
    if (!proc_.joinable()) return;
    UL lock(mutex_);
    cv_idle_.wait(lock, [this]() { return msg_buffer_.empty() && !processing_; });
}

}  // namespace lightning
