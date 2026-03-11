#include "ui/ui_trajectory.h"

#include <GL/gl.h>

namespace lightning::ui {

void UiTrajectory::AddPt(const SE3& pose) {
    // 如果轨迹点超出阈值 直接删除前一半点
    pos_.emplace_back(pose.translation().cast<float>());
    if (pos_.size() > max_size_) {
        pos_.erase(pos_.begin(), pos_.begin() + pos_.size() / 2);
    }
}

void UiTrajectory::Render() {
    // 点线形式
    glLineWidth(5.0);
    glBegin(GL_LINE_STRIP);
    glColor3f(color_[0], color_[1], color_[2]);

    for (const auto& p : pos_) {
        glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
}

}  // namespace lightning::ui
