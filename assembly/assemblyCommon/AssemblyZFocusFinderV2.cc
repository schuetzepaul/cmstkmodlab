/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//               Copyright (C) 2011-2022 - The DESY CMS Group                  //
//                           All rights reserved                               //
//                                                                             //
//      The CMStkModLab source code is licensed under the GNU GPL v3.0.        //
//      You have the right to modify and/or redistribute this source code      //
//      under the terms specified in the license, which may be found online    //
//      at http://www.gnu.org/licenses or at License.txt.                      //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////

#include <nqlogger.h>
#include <ApplicationConfig.h>

#include <AssemblyZFocusFinderV2.h>
#include <AssemblyUtilities.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <memory>

AssemblyZFocusFinderV2::AssemblyZFocusFinderV2(const AssemblyVUEyeCamera* camera, const LStepExpressMotionManager* motion_manager, QObject* parent)
 : camera_manager_(camera)
 , motion_manager_(motion_manager)
{
  // initialization
  ApplicationConfig* config = ApplicationConfig::instance();
  if(config == nullptr)
  {
    NQLog("AssemblyZFocusFinderV2", NQLog::Fatal) << "initialization error"
       << ": ApplicationConfig::instance() not initialized (null pointer), exiting constructor";

    return;
  }

  focus_pointN_max_ = config->getDefaultValue<int>   ("main", "AssemblyZFocusFinder_pointN_max", 200);
  focus_pointN_     = config->getDefaultValue<int>   ("main", "AssemblyZFocusFinder_pointN"    ,  50);

  focus_zrange_max_ = config->getDefaultValue<double>("main", "AssemblyZFocusFinder_zrange_max", 3.0);
  focus_zrange_     = config->getDefaultValue<double>("main", "AssemblyZFocusFinder_zrange"    , 0.5);

  focus_stepsize_min_ = config->getDefaultValue<double>("main", "AssemblyZFocusFinder_stepsize_min", 0.005);

  v_z_vals_.clear();
  v_focus_vals_.clear();
  // --------------

  // validation
  if(camera_manager_ == nullptr)
  {
    NQLog("AssemblyZFocusFinderV2", NQLog::Fatal) << "initialization error"
       << ": null pointer to AssemblyVUEyeCamera object, exiting constructor";

    return;
  }

  if(motion_manager_ == nullptr)
  {
    NQLog("AssemblyZFocusFinderV2", NQLog::Fatal) << "initialization error"
       << ": null pointer to LStepExpressMotionManager object, exiting constructor";

    return;
  }
  // --------------

  NQLog("AssemblyZFocusFinderV2", NQLog::Debug) << "constructed";
}

void AssemblyZFocusFinderV2::update_focus_config(const double zrange, const int pointN)
{
    if(zrange > 0.)
    {
      if(zrange > focus_zrange_max_)
      {
        NQLog("AssemblyZFocusFinderV2", NQLog::Warning) << "update_focus_config"
           << ": input value for z-motion range (" << zrange << ")"
           << " larger than allowed max ("   << focus_zrange_max_ << ")"
           << ", setting equal to allowed max";

        focus_zrange_ = focus_zrange_max_;
      }
      else
      {
        focus_zrange_ = zrange;
      }

      NQLog("AssemblyZFocusFinderV2", NQLog::Message) << "update_focus_config"
         << ": updated z-motion range to " << focus_zrange_;
    }
    else
    {
      NQLog("AssemblyZFocusFinderV2", NQLog::Critical) << "update_focus_config"
         << ": non-positive input value for z-motion range, value not updated";

      return;
    }

    if(pointN > 1)
    {
      if(pointN > focus_pointN_max_)
      {
        NQLog("AssemblyZFocusFinderV2", NQLog::Warning) << "update_focus_config"
           << ": input value for number of scans (" << pointN << ")"
           << " larger than allowed max ("    << focus_pointN_max_ << ")"
           << ", setting equal to allowed max";

        focus_pointN_ = focus_pointN_max_;
      }
      else
      {
        focus_pointN_ = pointN;
      }

      NQLog("AssemblyZFocusFinderV2", NQLog::Message) << "update_focus_config"
         << ": updated number of scans to " << focus_pointN_;
    }
    else
    {
      NQLog("AssemblyZFocusFinderV2", NQLog::Critical) << "update_focus_config"
         << ": invalid (<=1) value for number of scans, value not updated";

      return;
    }

    NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "update_focus_config"
       << ": emitting signal \"updated_focus_config\"";

    emit updated_focus_config();
}

void AssemblyZFocusFinderV2::run() {
    if(focus_pointN_ <= 1)
    {
      NQLog("AssemblyZFocusFinder", NQLog::Fatal) << "acquire_image"
         << ": invalid (<=1) value for number of scans (" << focus_pointN_ << "), stopping AssemblyZFocusFinder";

      return;
    }

    const double step_size = (2. * focus_zrange_ / double(focus_pointN_ - 1));

    if(step_size < focus_stepsize_min_)
    {
      NQLog("AssemblyZFocusFinder", NQLog::Fatal) << "acquire_image"
         << ": expected step size value (" << step_size << ") smaller than min-allowed value ("
         << focus_stepsize_min_ << "), stopping AssemblyZFocusFinder";

      return;
    }

    
    zposi_init_ = motion_manager_->get_position_Z();
    double zposi_max = zposi_init_+focus_zrange_;

    v_z_vals_.clear();
    v_focus_vals_.clear();

    for(int i=0; i<focus_pointN_; ++i)
    {
      v_z_vals_.emplace_back(zposi_max - i*step_size);
    }

    NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "RUUUUUUUN!!!!";

    int pos_index = 0;
    for(auto& zpos : v_z_vals_)
    {
        NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "Next step.";
        pos_index++;

        // First let's move to the next position
        QEventLoop wait_for_motion;
        connect(motion_manager_, SIGNAL(motion_finished()), &wait_for_motion, SLOT(quit()));

        connect(this, SIGNAL(move_relative_request(double, double, double, double)), motion_manager_, SLOT(moveRelative(double, double, double, double)));

        double rel_z = zpos - motion_manager_->get_position_Z();
        emit move_relative_request(0, 0, rel_z, 0);

        wait_for_motion.exec();
        double readback_z_pos = motion_manager_->get_position_Z();

        disconnect(this, SIGNAL(move_relative_request(double, double, double, double)), motion_manager_, SLOT(moveRelative(double, double, double, double)));
        disconnect(motion_manager_, SIGNAL(motion_finished()), &wait_for_motion, SLOT(quit()));

        NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "Motion done.";


        // Then we acquire an image

        ImageEventLoop* wait_for_image = new ImageEventLoop();
        connect(camera_manager_, SIGNAL(imageAcquired(cv::Mat)), wait_for_image, SLOT(do_wait_for_image(cv::Mat)));

        connect(this, SIGNAL(acquire_next_image()), camera_manager_, SLOT(acquireImage()));
        emit acquire_next_image();

        NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "Waiting for image.";

        wait_for_image->exec();
        NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "Waiting for image - done.";

        auto our_image = wait_for_image->get_img();

        disconnect(this, SIGNAL(acquire_next_image()), camera_manager_, SLOT(acquireImage()));
        disconnect(camera_manager_, SIGNAL(imageAcquired(cv::Mat)), wait_for_image, SLOT(do_wait_for_image(cv::Mat)));


        // Now we process the image

        AssemblyZFocusFinderV2::focus_info this_focus;
        this_focus.focus_disc = this->image_focus_value(our_image);
        this_focus.z_position = readback_z_pos;

        v_focus_vals_.emplace_back(this_focus);
        emit sig_update_progBar(int(pos_index*100./v_z_vals_.size())); //Update progress bar display
    }

    // Analyse data ...

    // Find best position
    NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "test_focus"
       << ": finding best-focus position";

    double zposi_best(zposi_init_);
    {

      QLineSeries* zscan_graph = new QLineSeries();

      double focus_best(-1.);
      for(unsigned int i=0; i<v_focus_vals_.size(); ++i)
      {
        const double i_zposi = v_focus_vals_.at(i).z_position;
        const double i_focus = v_focus_vals_.at(i).focus_disc;

        zscan_graph->append(i_zposi, i_focus);

        if((i == 0) || (i_focus > focus_best)){ focus_best = i_focus; zposi_best = i_zposi; }
      }

      NQLog("AssemblyZFocusFinder", NQLog::Spam) << "test_focus"
         << ": emitting signal \"show_zscan(...)\"";

      emit show_zscan(*zscan_graph);

      NQLog("AssemblyZFocusFinder", NQLog::Spam) << "test_focus"
         << ": emitting signal \"text_update_request(" << zposi_best << ")\"";

      emit text_update_request(zposi_best);
    }
    // ------------------


    // Move to best z position
    const double zposi_now = motion_manager_->get_position_Z();
    const double dz = (zposi_best-zposi_now);

    QEventLoop wait_for_motion;
    connect(motion_manager_, SIGNAL(motion_finished()), &wait_for_motion, SLOT(quit()));
    connect(this, SIGNAL(move_relative_request(double, double, double, double)), motion_manager_, SLOT(moveRelative(double, double, double, double)));

    NQLog("AssemblyZFocusFinder", NQLog::Spam) << "test_focus"
    << ": emitting signal \"move_relative_request(0, 0, " << dz << ", 0)\"";

    emit move_relative_request(0, 0, dz, 0);
    wait_for_motion.exec();

    disconnect(this, SIGNAL(move_relative_request(double, double, double, double)), motion_manager_, SLOT(moveRelative(double, double, double, double)));
    disconnect(motion_manager_, SIGNAL(motion_finished()), &wait_for_motion, SLOT(quit()));


    // Take final image

    ImageEventLoop* wait_for_image = new ImageEventLoop();
    connect(camera_manager_, SIGNAL(imageAcquired(cv::Mat)), wait_for_image, SLOT(do_wait_for_image(cv::Mat)));

    connect(this, SIGNAL(acquire_next_image()), camera_manager_, SLOT(acquireImage()));
    emit acquire_next_image();

    NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "Waiting for image.";

    wait_for_image->exec();
    NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "Waiting for image - done.";

    auto our_image = wait_for_image->get_img();

    disconnect(this, SIGNAL(acquire_next_image()), camera_manager_, SLOT(acquireImage()));
    disconnect(camera_manager_, SIGNAL(imageAcquired(cv::Mat)), wait_for_image, SLOT(do_wait_for_image(cv::Mat)));


    emit sig_update_progBar(int(100)); //Update progress bar display
    // ------------------

    emit image_acquired(our_image);

    return;
}

void AssemblyZFocusFinderV2::acquire_image()
{
    this->run();
}

void AssemblyZFocusFinderV2::this_random_slot(const cv::Mat& mat) {
    NQLog("AssemblyZFocusFinderV2", NQLog::Spam) << "IMAGE ACTUALLY ARRIVES.";
}

void AssemblyZFocusFinderV2::emergencyStop()
{
  NQLog("AssemblyZFocusFinder", NQLog::Message) << "emergencyStop"
     << ": will stop execution of AssemblyZFocusFinder";

  v_z_vals_.clear();
  v_focus_vals_.clear();

  NQLog("AssemblyZFocusFinder", NQLog::Message) << "emergencyStop"
     << ": emitting signal \"emergencyStopped\"";

  emit emergencyStopped();
}

// \Brief Image-focus discriminant based on Laplacian method in OpenCV
//        REF: https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/laplace_operator/laplace_operator.html
double AssemblyZFocusFinderV2::image_focus_value(const cv::Mat& img)
{
//  // Remove noise by blurring with a Gaussian filter
//  cv::Mat img_gaus;
//  cv::GaussianBlur(img, img_gaus, cv::Size(img.cols, img.rows), 0, 0, cv::BORDER_DEFAULT);
//
//  // Convert the image to grayscale
//  cv::Mat img_gray;
//  cv::cvtColor(img_gaus, img_gray, cv::COLOR_BGR2GRAY);

  // Apply laplacian function to GS image
  cv::Mat img_lap;
  cv::Laplacian(img, img_lap, CV_64F);

  // Calculate standard deviation of laplace image
  cv::Scalar mean, std_dev;
  cv::meanStdDev(img_lap, mean, std_dev);

  const float value = (std_dev.val[0] * std_dev.val[0]);

  return value;
}
