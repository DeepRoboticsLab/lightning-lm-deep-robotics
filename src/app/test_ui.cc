//
// Created by xiang on 25-8-27.
//

#include "ui/pangolin_window.h"

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);

    lightning::ui::PangolinWindow ui;
    ui.Init();

    while (!ui.ShouldQuit()) {
        sleep(1);
    }

    ui.Quit();

    return 0;
}
