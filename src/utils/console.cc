#include "utils/console.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

DEFINE_string(console, "auto", "Terminal output: auto (live status on a TTY), plain, or verbose");

namespace lightning::console {
namespace {
using Clock = std::chrono::steady_clock;
using Lock = std::lock_guard<std::mutex>;

// Remove terminal control characters from paths/messages; preserve printable UTF-8.
std::string Clean(std::string text) {
    for (char& c : text) if (static_cast<unsigned char>(c) < 32 || c == 127) c = ' ';
    return text;
}

std::string Fit(std::string text, size_t width) {
    text = Clean(std::move(text));
    if (text.size() <= width) return text;
    if (width <= 3) return std::string(width, '.');
    size_t end = width - 3;
    while (end && (static_cast<unsigned char>(text[end]) & 0xc0) == 0x80) --end;
    return text.substr(0, end) + "...";
}

struct Output : google::LogSink {
    std::atomic_bool active{false};
    std::atomic_size_t lidar{0}, imu{0};
    std::mutex mutex;
    std::mutex metrics_mutex;
    FILE* stream = stderr;
    int original_stderr = -1, original_stdout = -1;
    std::condition_variable wake;
    bool stop = false, tty = false, verbose = false, online = false;
    std::string mode, state, last_issue;
    size_t updates = 0, keyframes = 0, matches = 0, accepted = 0;
    size_t points = 0, matched = 0, warnings = 0, errors = 0;
    std::array<double, 3> pose{};
    bool have_pose = false, have_match = false, last_accepted = false;
    double score = 0;
    Clock::time_point started, sampled;
    std::array<size_t, 3> previous{};
    std::array<double, 3> rates{};
    int drawn = 0, last_width = 0;
    struct Issue { size_t count = 0, printed = 0; Clock::time_point last{}; std::string text; };
    std::map<std::string, Issue> issues;

    int Width() {
        winsize size{};
        return ioctl(fileno(stream), TIOCGWINSZ, &size) == 0 && size.ws_col ? size.ws_col : 80;
    }

    void Clear() {
        // Resizing can reflow old lines. Do not erase unknown scrollback in that case.
        if (drawn && Width() == last_width) {
            std::fprintf(stream, "\r\033[%dA\033[J", drawn);
        }
        drawn = 0;
    }

    void Line(const std::string& text) {
        Clear();
        std::fprintf(stream, "%s\n", Clean(text).c_str());
        std::fflush(stream);
    }

    void send(google::LogSeverity severity, const char*, const char* filename, int line,
              const std::tm*, const char* message, size_t length) override {
        if (severity < google::WARNING) return;
        Lock lock(mutex);
        if (severity == google::WARNING) ++warnings; else ++errors;
        if (verbose) return;  // glog already prints the original message.
        if (severity == google::FATAL) {
            // glog's crash trace must go directly to the terminal even if the
            // process aborts before the foreign-output reader can drain its pipe.
            if (original_stderr >= 0) dup2(original_stderr, STDERR_FILENO);
            if (original_stdout >= 0) dup2(original_stdout, STDOUT_FILENO);
            stop = true;
            wake.notify_all();
        }
        const std::string label = severity == google::WARNING ? "WARNING" : severity == google::FATAL ? "FATAL" : "ERROR";
        const std::string key = label + " " + filename + ":" + std::to_string(line);
        auto& issue = issues[key];
        ++issue.count;
        issue.text = label + ": " + Clean(std::string(message, length));
        last_issue = issue.text;
        const auto now = Clock::now();
        if (issue.count == 1 || severity == google::FATAL || now - issue.last >= std::chrono::seconds(5)) {
            Line(issue.text + (issue.count > 1 ? " [" + std::to_string(issue.count) + " occurrences]" : ""));
            issue.last = now;
            issue.printed = issue.count;
        }
    }

    void Render(bool final = false) {
        // Never hold the metrics lock during terminal I/O: a slow SSH reader
        // must not block sensor processing merely to refresh the display.
        std::unique_lock<std::mutex> metrics(metrics_mutex);
        const auto now = Clock::now();
        const double interval = std::chrono::duration<double>(now - sampled).count();
        const std::array<size_t, 3> counts{lidar.load(), imu.load(), updates};
        if (interval >= 0.25) {
            for (size_t i = 0; i < counts.size(); ++i) rates[i] = (counts[i] - previous[i]) / interval;
            previous = counts;
            sampled = now;
        }
        std::string current = state;
        if (online && !final) {
            if (!counts[0]) current = "waiting for LiDAR";
            else if (!counts[1]) current = "waiting for IMU";
            else if (!updates) current = "initializing / waiting for synchronized scans";
            else if (rates[0] == 0 || rates[1] == 0) current = "input paused";
            else if (rates[2] == 0) current = "waiting for LIO update";
        }
        std::vector<std::string> lines;
        std::ostringstream title;
        title << "Lightning-LM | " << mode << (online ? " online" : " offline")
              << " | " << (final ? "stopped" : current) << " | "
              << static_cast<long>(std::chrono::duration<double>(now - started).count()) << " s";
        lines.push_back(title.str());
        std::ostringstream sensors;
        sensors << std::fixed << std::setprecision(1) << "Hz (wall): LiDAR " << rates[0] << " | IMU " << rates[1]
                << " | LIO " << rates[2] << "   received " << counts[0] << "/" << counts[1];
        lines.push_back(sensors.str());
        std::ostringstream progress;
        progress << "LIO updates " << updates;
        if (mode == "Mapping") progress << " | keyframes " << keyframes << " | points " << points << " / matched " << matched;
        else {
            progress << " | matches " << accepted << "/" << matches;
            if (have_match) progress << " | score " << std::fixed << std::setprecision(2) << score
                                     << (last_accepted ? " accepted" : " rejected");
        }
        lines.push_back(progress.str());
        std::ostringstream health;
        if (have_pose) health << (mode == "Mapping" ? "LIO xyz " : "Map xyz ") << std::fixed << std::setprecision(2)
                             << pose[0] << " " << pose[1] << " " << pose[2] << " m | ";
        health << "warnings " << warnings << " | errors " << errors;
        if (!last_issue.empty()) health << " | " << last_issue;
        lines.push_back(health.str());
        metrics.unlock();
        if (tty) {
            Clear();
            const int width = Width();
            winsize size{};
            ioctl(fileno(stream), TIOCGWINSZ, &size);
            if (width < 40 || (size.ws_row && size.ws_row < 6)) lines = {title.str()};
            for (const auto& text : lines) std::fprintf(stream, "%s\n", Fit(text, std::max(0, width - 1)).c_str());
            drawn = static_cast<int>(lines.size());
            last_width = width;
        } else {
            std::fprintf(stream, "%s | %s | %s | %s\n", lines[0].c_str(), lines[1].c_str(), lines[2].c_str(), lines[3].c_str());
        }
        std::fflush(stream);
    }
};

Output& Get() { static Output output; return output; }
}  // namespace

struct Session::Impl {
    std::thread worker;
    std::thread external_reader;
    int saved_stderr = -1, saved_stdout = -1, read_fd = -1;
    bool captured = false, sink_added = false;
    std::ofstream external_log;
    bool old_stderr = FLAGS_logtostderr, old_also = FLAGS_alsologtostderr;
    int old_threshold = FLAGS_stderrthreshold;

    // Capture only descriptors attached to this terminal. Library/ROS messages
    // then use the same erase/event/redraw lock, without swallowing rosout or
    // disturbing an independently redirected stdout. Plain/verbose need no capture.
    void Capture(Output& out, const std::filesystem::path& directory) {
        std::fflush(stdout); std::fflush(stderr);
        int pipe_fd[2];
        if (pipe2(pipe_fd, O_CLOEXEC) != 0) throw std::runtime_error("Cannot create terminal output pipe");
        read_fd = pipe_fd[0];
        saved_stderr = fcntl(STDERR_FILENO, F_DUPFD_CLOEXEC, 3);
        struct stat a{}, b{};
        if (isatty(STDOUT_FILENO) && fstat(STDOUT_FILENO, &a) == 0 && fstat(STDERR_FILENO, &b) == 0 && a.st_rdev == b.st_rdev)
            saved_stdout = fcntl(STDOUT_FILENO, F_DUPFD_CLOEXEC, 3);
        const int display_fd = saved_stderr < 0 ? -1 : fcntl(saved_stderr, F_DUPFD_CLOEXEC, 3);
        out.stream = display_fd < 0 ? nullptr : fdopen(display_fd, "w");
        if (!out.stream) {
            if (display_fd >= 0) close(display_fd);
            close(pipe_fd[1]); out.stream = stderr; throw std::runtime_error("Cannot preserve terminal output");
        }
        captured = true;
        external_log.open(directory / "external.log");
        if (!external_log) { close(pipe_fd[1]); throw std::runtime_error("Cannot open external diagnostic log"); }
        if (dup2(pipe_fd[1], STDERR_FILENO) < 0 || (saved_stdout >= 0 && dup2(pipe_fd[1], STDOUT_FILENO) < 0)) {
            close(pipe_fd[1]); throw std::runtime_error("Cannot route terminal output");
        }
        close(pipe_fd[1]);
        out.original_stderr = saved_stderr;
        out.original_stdout = saved_stdout;
        external_reader = std::thread([this, &out] {
            char buffer[4096];
            std::string pending;
            while (true) {
                const auto n = read(read_fd, buffer, sizeof(buffer));
                if (n < 0 && errno == EINTR) continue;
                if (n <= 0) break;
                external_log.write(buffer, n);
                external_log.flush();
                pending.append(buffer, n);
                size_t end;
                while ((end = pending.find_first_of("\r\n")) != std::string::npos || pending.size() >= 8192) {
                    const size_t consumed = end == std::string::npos ? 8192 : end + 1;
                    if (end == std::string::npos) end = 8192;
                    if (end) { Lock lock(out.mutex); out.Line(pending.substr(0, end)); }
                    pending.erase(0, consumed);
                }
            }
            if (!pending.empty()) { Lock lock(out.mutex); out.Line(pending); }
        });
    }

    void Restore() {
        std::fflush(stdout); std::fflush(stderr);
        if (saved_stderr >= 0) { dup2(saved_stderr, STDERR_FILENO); close(saved_stderr); saved_stderr = -1; }
        if (saved_stdout >= 0) { dup2(saved_stdout, STDOUT_FILENO); close(saved_stdout); saved_stdout = -1; }
        if (external_reader.joinable()) external_reader.join();
        if (read_fd >= 0) { close(read_fd); read_fd = -1; }
    }

    ~Impl() {
        // Also covers a constructor failure after capture/sink registration.
        auto& out = Get();
        if (worker.joinable()) {
            { Lock lock(out.mutex); out.stop = true; }
            out.wake.notify_all(); worker.join();
        }
        Restore();
        if (sink_added) {
            google::RemoveLogSink(&out); out.active = false;
            FLAGS_logtostderr = old_stderr; FLAGS_alsologtostderr = old_also;
            FLAGS_stderrthreshold = old_threshold;
        }
        if (captured && out.stream != stderr) { std::fclose(out.stream); out.stream = stderr; }
        if (captured) { out.original_stderr = -1; out.original_stdout = -1; }
    }
};

Session::Session(const std::string& mode, bool online) : impl_(new Impl) {
    if (FLAGS_console != "auto" && FLAGS_console != "plain" && FLAGS_console != "verbose")
        throw std::invalid_argument("--console must be auto, plain, or verbose");
    auto& out = Get();
    if (out.active) throw std::logic_error("Only one terminal session is allowed");
    const auto wall = std::time(nullptr);
    std::tm local{};
    localtime_r(&wall, &local);
    std::ostringstream name;
    name << "lightning-" << std::put_time(&local, "%Y%m%d-%H%M%S") << "-" << getpid();
    const auto directory = std::filesystem::absolute(std::filesystem::path(FLAGS_log_dir.empty() ? "log" : FLAGS_log_dir) / name.str());
    std::filesystem::create_directories(directory);
    directory_ = directory.string();
    google::SetLogDestination(google::INFO, (directory.string() + "/details.").c_str());
    for (int level = google::WARNING; level <= google::FATAL; ++level) google::SetLogDestination(level, "");
    const char* term = std::getenv("TERM");
    out.tty = FLAGS_console == "auto" && isatty(STDERR_FILENO) && term && std::string(term) != "dumb";
    out.verbose = FLAGS_console == "verbose";
    out.mode = mode;
    out.online = online;
    out.state = "starting";
    out.stop = false;
    out.lidar = 0; out.imu = 0;
    out.updates = out.keyframes = out.matches = out.accepted = 0;
    out.points = out.matched = out.warnings = out.errors = 0;
    out.have_pose = out.have_match = out.last_accepted = false;
    out.previous = {}; out.rates = {}; out.pose = {};
    out.last_issue.clear(); out.issues.clear(); out.drawn = 0;
    out.started = out.sampled = Clock::now();
    if (out.tty) impl_->Capture(out, directory);
    out.active = true;
    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = false;
    FLAGS_stderrthreshold = out.verbose ? google::INFO : google::NUM_SEVERITIES;
    google::AddLogSink(&out);
    impl_->sink_added = true;
    Event("Detailed log: " + directory.string());
    if (!out.verbose) {
        impl_->worker = std::thread([&out]() {
            std::unique_lock<std::mutex> lock(out.mutex);
            const auto period = std::chrono::seconds(out.tty ? 1 : 10);
            out.Render();
            while (!out.wake.wait_for(lock, period, [&out] { return out.stop; })) out.Render();
        });
    }
}

Session::~Session() {
    auto& out = Get();
    { Lock lock(out.mutex); out.stop = true; }
    out.wake.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
    impl_->Restore();  // Drain foreign output before leaving the final summary.
    google::RemoveLogSink(&out);
    impl_->sink_added = false;
    {
        Lock lock(out.mutex);
        if (!out.verbose) {
            for (const auto& pair : out.issues) {
                const auto& issue = pair.second;
                if (issue.count != issue.printed) out.Line(issue.text + " [" + std::to_string(issue.count) + " occurrences total]");
            }
            out.Render(true);
            out.drawn = 0;  // Leave the final summary in scrollback.
        }
        out.active = false;
        if (out.stream != stderr) { std::fclose(out.stream); out.stream = stderr; }
    }
    google::FlushLogFiles(google::INFO);
    FLAGS_logtostderr = impl_->old_stderr;
    FLAGS_alsologtostderr = impl_->old_also;
    FLAGS_stderrthreshold = impl_->old_threshold;
}

void ReceivedLidar() { auto& out = Get(); if (out.active) ++out.lidar; }
void ReceivedImu() { auto& out = Get(); if (out.active) ++out.imu; }
void Odometry(size_t points, size_t matched, double x, double y, double z) {
    auto& out = Get(); if (!out.active) return;
    Lock lock(out.metrics_mutex);
    ++out.updates; out.points = points; out.matched = matched;
    if (out.mode == "Mapping") { out.pose = {x, y, z}; out.have_pose = true; }
}
void Keyframe() { auto& out = Get(); if (out.active) { Lock lock(out.metrics_mutex); ++out.keyframes; } }
void Match(bool accepted, double score, double x, double y, double z) {
    auto& out = Get(); if (!out.active) return;
    Lock lock(out.metrics_mutex);
    ++out.matches; out.have_match = true; out.score = score; out.last_accepted = accepted;
    if (accepted) { ++out.accepted; out.pose = {x, y, z}; out.have_pose = true; out.state = "tracking"; }
    else out.state = "waiting for an accepted match";
}
void State(const std::string& state) {
    auto& out = Get(); if (out.active) { Lock lock(out.metrics_mutex); out.state = Clean(state); }
}
void Event(const std::string& message) {
    LOG(INFO) << message;
    auto& out = Get();
    if (out.active && !out.verbose) { Lock lock(out.mutex); out.Line(message); }
}

}  // namespace lightning::console
