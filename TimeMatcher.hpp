#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Subscriber image_topic and imu_topic to matcher theis timepoint then publish to image_data_topic
constructor_args: 
  - img: 
    - up: 100
    - dn: 50
    
  - imu: 
    - up: 100
    - dn: 50
template_args: []
required_hardware: []
depends: 
  - qdu-future/HikCamera
  - qdu-future/UartDataProcess
=== END MANIFEST === */
// clang-format on

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
    Range(unsigned u, unsigned d)
    {
      if (u >= d)
      {
        XR_LOG_ERROR("TimeMatcher::range that up >= dn \n");
        exit(10);
      }
      // 原始入参为毫秒，转换为微秒
      this->up = static_cast<int64_t>(u) * 1000;
      this->dn = static_cast<int64_t>(d) * 1000;
    }
    // 内部构造函数，用于 differ_t 初始化 (已经计算好的微秒值)
    Range(int64_t u, int64_t d) : up(u), dn(d) {}
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
