#ifndef RAE_HW_SPEAKERS_NODE_HPP_
#define RAE_HW_SPEAKERS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "audio_msgs/msg/audio.hpp"
#include "std_msgs/msg/int32.hpp"
#include <alsa/asoundlib.h>
#include <mpg123.h>

namespace rae_hw {

class SpeakersNode : public rclcpp::Node {
public:
  SpeakersNode(const rclcpp::NodeOptions &options);
  ~SpeakersNode();

private:
  void audio_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void play_mp3(const char *);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  snd_pcm_t *alsaHandle;

  unsigned char *buffer;
  mpg123_handle *mh;
};

} // namespace rae_hw

#endif // RAE_HW_SPEAKERS_NODE_HPP_
