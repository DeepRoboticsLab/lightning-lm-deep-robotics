#include <QApplication>
#include <QEvent>
#include <QPointer>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/visualizer_app.hpp>

namespace {

// Forwarded software rendering can still swap an X11 image while RViz is
// closing its window. Stop the render timer before the close handler, including
// its nested save-dialog event loop; resume only if the user cancels closing.
class CloseGuard : public QObject {
 public:
  CloseGuard(QWidget* frame, rviz_common::VisualizationManager* manager)
      : frame_(frame), manager_(manager) {
    frame->installEventFilter(this);
  }

  void stop() {
    if (manager_) manager_->stopUpdate();
  }

 protected:
  bool eventFilter(QObject* object, QEvent* event) override {
    if (object != frame_ || event->type() != QEvent::Close) return false;
    stop();
    frame_->removeEventFilter(this);
    QApplication::sendEvent(object, event);
    if (frame_) {
      frame_->installEventFilter(this);
      // RViz may restart updates after its save dialog returns.
      if (event->isAccepted()) stop();
      else if (manager_) manager_->startUpdate();
    }
    return true;
  }

 private:
  QPointer<QWidget> frame_;
  QPointer<rviz_common::VisualizationManager> manager_;
};

}  // namespace

int main(int argc, char** argv) {
  auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> qt_arguments;
  for (auto& argument : arguments) qt_arguments.push_back(argument.data());
  int qt_argc = static_cast<int>(qt_arguments.size());
  qt_arguments.push_back(nullptr);
  QApplication application(qt_argc, qt_arguments.data());

  const auto logger = rclcpp::get_logger("rviz2");
  rviz_common::set_logging_handlers(
      [logger](const std::string& message, const std::string&, size_t) {
        RCLCPP_DEBUG(logger, "%s", message.c_str());
      },
      [logger](const std::string& message, const std::string&, size_t) {
        RCLCPP_INFO(logger, "%s", message.c_str());
      },
      [logger](const std::string& message, const std::string&, size_t) {
        RCLCPP_WARN(logger, "%s", message.c_str());
      },
      [logger](const std::string& message, const std::string&, size_t) {
        RCLCPP_ERROR(logger, "%s", message.c_str());
      });

  rviz_common::VisualizerApp visualizer(
      std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  visualizer.setApp(&application);
  if (!visualizer.init(argc, argv)) return 1;
  QWidget* frame = nullptr;
  rviz_common::VisualizationManager* manager = nullptr;
  for (auto* window : QApplication::topLevelWidgets()) {
    // RenderPanel is public in Foxy and Humble; Foxy's frame header is private.
    if (auto* panel = window->findChild<rviz_common::RenderPanel*>()) {
      manager = dynamic_cast<rviz_common::VisualizationManager*>(panel->getManager());
      if (manager) {
        frame = window;
        break;
      }
    }
  }
  if (!frame) {
    std::cerr << "RViz did not create a visualization window.\n";
    return 1;
  }
  CloseGuard guard(frame, manager);
  QObject::connect(&application, &QApplication::aboutToQuit, &guard, &CloseGuard::stop);
  const int result = application.exec();
  guard.stop();
  return result;
}
