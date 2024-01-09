/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//               Copyright (C) 2011-2017 - The DESY CMS Group                  //
//                           All rights reserved                               //
//                                                                             //
//      The CMStkModLab source code is licensed under the GNU GPL v3.0.        //
//      You have the right to modify and/or redistribute this source code      //
//      under the terms specified in the license, which may be found online    //
//      at http://www.gnu.org/licenses or at License.txt.                      //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////

#ifndef ASSEMBLYZFOCUSFINDERV2_H
#define ASSEMBLYZFOCUSFINDERV2_H

#include <AssemblyVUEyeCamera.h>
#include <LStepExpressMotionManager.h>

#include <QObject>
#include <QString>
#include <QThread>
#include <QEventLoop>
#include <QLineSeries>
#include <QtCharts>

#include <vector>
#include <string>

#include <chrono>
#include <nqlogger.h>

#include <opencv2/opencv.hpp>

class ImageEventLoop : public QEventLoop
{
Q_OBJECT

  public:
    ImageEventLoop() : QEventLoop(){};

    cv::Mat get_img() {return mat_;};

  public slots:
    void do_wait_for_image(const cv::Mat& mat) {
        if(!mat.empty()) {
            mat_ = mat;
            quit();
        }
    };

  private:
    cv::Mat mat_;
};

class AssemblyZFocusFinderV2 : public QThread
{
 Q_OBJECT

  public:

    AssemblyZFocusFinderV2(const AssemblyVUEyeCamera*, const LStepExpressMotionManager*, QObject* parent=nullptr);

    const AssemblyVUEyeCamera*       camera_manager() const { return camera_manager_; }
    const LStepExpressMotionManager* motion_manager() const { return motion_manager_; }

    struct focus_info
    {
      double z_position;
      double focus_disc;
    };

    double zrange() const { return focus_zrange_; }
    int    points() const { return focus_pointN_; }


  protected:

    const AssemblyVUEyeCamera*       camera_manager_;
    const LStepExpressMotionManager* motion_manager_;

    std::vector<double>     v_z_vals_;
    std::vector<focus_info> v_focus_vals_;

    double zposi_init_;

    int    focus_pointN_max_;
    int    focus_pointN_;
    double focus_zrange_max_;
    double focus_zrange_;
    double focus_stepsize_min_;

    void run() override;

    double image_focus_value(const cv::Mat&);


  public slots:

    void acquire_image();

    void update_focus_config(const double, const int);
    void this_random_slot(const cv::Mat&);

    void emergencyStop();

  signals:

    void updated_focus_config();
    void focus_config_request();
    void image_acquired(const cv::Mat&);
    void move_relative_request(double, double, double, double);
    void acquire_next_image();

    void emergencyStopped();

    void show_zscan(QLineSeries&);
    void text_update_request(double);
    void sig_update_progBar(int);

};
#endif // ASSEMBLYZFOCUSFINDERV2_H
