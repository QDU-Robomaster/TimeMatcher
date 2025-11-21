#include "TimeMatcher.hpp"

#include "message.hpp"
TimeMatcher::TimeMatcher(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
                         const Range& img, const Range& imu)
    : DIFFER_T(img.dn - imu.up, img.up - imu.dn)
{
  auto topic = LibXR::Topic::Find("imu_topic");
  auto image_topic = LibXR::Topic(LibXR::Topic::Find("image_topic"));
  this->image_imu_topic_ = LibXR::Topic::CreateTopic<ImageAndImu>("image_imu_topic");

  auto imu_sub = LibXR::Topic::QueuedSubscriber(topic, this->imu_queue_);

  auto image_cb = LibXR::Topic::Callback::Create(
      [](bool, TimeMatcher* self, LibXR::RawData& data)
      {
        XR_LOG_DEBUG("Got uart!");
        auto* image_msg = reinterpret_cast<HikCamera::ImageData*>(data.addr_);
        self->MatcherCallback(image_msg);
      },
      this);

  image_topic.RegisterCallback(image_cb);

  app.Register(*this);
}

void TimeMatcher::MatcherCallback(HikCamera::ImageData* img_msg)
{
  auto& image_time = (*img_msg).time;
  while (true)
  {
    UartDataT uart_data;

    LibXR::ErrorCode ans = this->imu_queue_.Peek(uart_data);

    if (ans == LibXR::ErrorCode::EMPTY)
    {
      break;
    }

    auto& imu_time = uart_data.time;

    int64_t differ = static_cast<int64_t>(image_time) - static_cast<int64_t>(imu_time);

    if (differ < DIFFER_T.dn)
    {
      continue;  // 图片是旧的，丢弃照片
    }

    if (differ > DIFFER_T.up)  // imu数据是旧的，丢弃imu数据
    {
      this->imu_queue_.Pop();
      continue;
    }
    // 配对成功
    ImageAndImu data;
    data.image = (*img_msg).image;
    data.Quat = uart_data.data.Quat;
    data.time = uart_data.time;
    this->imu_queue_.Pop();
    this->image_imu_topic_.Publish(data);
  }
}
