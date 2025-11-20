#include "TimeMatcher.hpp"

#include <opencv2/core/quaternion.hpp>

#include "libxr_def.hpp"
#include "message.hpp"
TimeMatcher::TimeMatcher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                         const range& img, const range& imu)
    : differ_t((img.dn - imu.up).count(), (img.up - imu.dn).count())
{
  UNUSED(hw);

  //   LibXR::Topic::QueuedSubscriber sub("imu_topic", this->ImuQueue, sizeof(UartDataT));
  auto topic = LibXR::Topic::Find("imu_topic");
  auto image_topic = LibXR::Topic(LibXR::Topic::Find("image_topic"));
  this->image_imu_topic = LibXR::Topic::CreateTopic<ImageAndImu>("image_imu_topic");

  auto imu_sub = LibXR::Topic::QueuedSubscriber(topic, this->ImuQueue);

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
    UartDataT Udata;

    LibXR::ErrorCode ans = this->ImuQueue.Peek(Udata);

    if (ans == LibXR::ErrorCode::EMPTY) break;

    auto& imu_time = Udata.time;

    auto differ = image_time - imu_time;

    if (differ < differ_t.dn) continue;  // 图片是旧的，丢弃照片

    if (differ > differ_t.up)  // imu数据是旧的，丢弃imu数据
    {
      ImuQueue.Pop();
      continue;
    }
    // 配对成功
    ImageAndImu data;
    data.image = (*img_msg).image;
    data.Quat = cv::Quatf(Udata.data.Quat[0], Udata.data.Quat[1], Udata.data.Quat[2],
                          Udata.data.Quat[3]);
    data.time = Udata.time;
    this->ImuQueue.Pop();
    this->image_imu_topic.Publish(data);
  }
}