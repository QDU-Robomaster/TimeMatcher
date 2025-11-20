#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Subscriber image_topic and imu_topic to matcher theis timepoint then publish to image_data_topic
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <chrono>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>

#include "../HikCamera/HikCamera.hpp"
#include "../UartDataProcess/UartDataProcess.hpp"
#include "app_framework.hpp"
#include "lockfree_queue.hpp"
#include "message.hpp"

struct ImageAndImu
{
  cv::Mat image;
  cv::Quatf Quat;
  std::chrono::steady_clock::time_point time;
};

class TimeMatcher : public LibXR::Application
{
 public:
  struct range
  {
    std::chrono::milliseconds up, dn;
    range(unsigned u, unsigned d)
    {
      if (u >= d)
      {
        std::cerr << "TimeMatcher::range that up >= dn \n";
        exit(10);
      }
      this->up = std::chrono::milliseconds(u);
      this->dn = std::chrono::milliseconds(d);
    }
  };
  TimeMatcher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              const range& img, const range& imu);

  void OnMonitor() override {}

 private:
  const range differ_t;
  LibXR::Topic image_imu_topic;
  LibXR::LockFreeQueue<UartDataT> ImuQueue{3};
  void MatcherCallback(HikCamera::ImageData* img_msg);
};
