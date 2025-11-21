#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Subscriber image_topic and imu_topic to matcher theis timepoint then publish to image_data_topic
constructor_args:
  img:
    up: 100
    dn: 50

  imu:
    up: 100
    dn: 50
template_args: []
required_hardware: []
depends: 
  - qdu-future/HikCamera@dev
  - qdu-future/UartDataProcess
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>

#include "HikCamera.hpp"
#include "UartDataProcess.hpp"
#include "app_framework.hpp"
#include "lockfree_queue.hpp"
#include "message.hpp"

struct ImageAndImu
{
  cv::Mat image;
  LibXR::Quaternion<float> Quat;
  LibXR::MicrosecondTimestamp time;
};

class TimeMatcher : public LibXR::Application
{
 public:
  struct Range
  {
    // 使用 int64_t 存储微秒，以支持原始逻辑中的差值计算
    int64_t up, dn;
    Range(int64_t u, int64_t d) : up(u * 1000), dn(d * 1000) {}
  };
  TimeMatcher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              const Range& img, const Range& imu);

  void OnMonitor() override {}

 private:
  const Range DIFFER_T;
  LibXR::Topic image_imu_topic_;
  LibXR::LockFreeQueue<UartDataT> imu_queue_{3};
  void MatcherCallback(HikCamera::ImageData* img_msg);
};
