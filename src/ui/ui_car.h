#pragma once

#include "common/eigen_types.h"

namespace lightning::ui {

/// 在UI里显示的小车
class UiCar {
   public:
    UiCar(const Vec3f& color) : color_(color) {}

    /// 设置小车 Pose，重设显存中的点
    void SetPose(const SE3& pose);

    /// 渲染小车
    void Render();

   private:
    Vec3f color_;
    std::vector<Vec3f> pts_;

    static std::vector<Vec3f> car_vertices_;  // 小车的顶点
};

}  // namespace lightning::ui
