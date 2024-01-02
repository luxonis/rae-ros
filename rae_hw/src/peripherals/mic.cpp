#include "rae_hw/peripherals/mic.hpp"

#include <vector>

#include "alsa/asoundlib.h"
#include "alsa/pcm.h"
#include "audio_msgs/msg/audio.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rae_hw
{
MicNode::MicNode(const rclcpp::NodeOptions & options) : Node("mic_node", options), handle_(nullptr)
{
  publisher_ = this->create_publisher<audio_msgs::msg::Audio>("audio_in", 10);
  configure_microphone();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&MicNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Mic node running!");
}
MicNode::~MicNode() { snd_pcm_close(handle_); }

void MicNode::configure_microphone()
{
  snd_pcm_hw_params_t * params;
  unsigned int sample_rate = 48000;
  int dir;

  int rc = snd_pcm_open(&handle_, "hw:0,1", SND_PCM_STREAM_CAPTURE, 0);
  if (rc < 0) {
    RCLCPP_FATAL(this->get_logger(), "Unable to open PCM device: %s", snd_strerror(rc));
    return;
  }

  snd_pcm_hw_params_alloca(&params);
  snd_pcm_hw_params_any(handle_, params);
  snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S32_LE);
  snd_pcm_hw_params_set_channels(handle_, params, 2);
  snd_pcm_hw_params_set_rate_near(handle_, params, &sample_rate, &dir);

  rc = snd_pcm_hw_params(handle_, params);
  if (rc < 0) {
    RCLCPP_FATAL(this->get_logger(), "Unable to set HW parameters: %s", snd_strerror(rc));
    return;
  }
}

void MicNode::timer_callback()
{
  snd_pcm_uframes_t frames = 48000 * 0.05;
  std::vector<int32_t> buffer(2 * frames);

  int rc = snd_pcm_readi(handle_, buffer.data(), frames);
  if (rc == -EPIPE) {
    snd_pcm_prepare(handle_);
    RCLCPP_ERROR(this->get_logger(), "Overrun occurred. Preparing the interface.");
    return;
  } else if (rc < 0) {
    RCLCPP_ERROR(this->get_logger(), "Error from ALSA readi: %s", snd_strerror(rc));
    return;
  }

  audio_msgs::msg::Audio msg;
  msg.header.stamp = this->now();
  msg.seq_num = seq_num_++;
  msg.frames = frames;
  msg.channels = 2;
  msg.sample_rate = 48000;
  msg.encoding = "S32LE";
  msg.is_bigendian = 0;
  msg.layout = msg.LAYOUT_INTERLEAVED;
  msg.step = 8;
  std::vector<uint8_t> dataVec(
    reinterpret_cast<uint8_t *>(buffer.data()),
    reinterpret_cast<uint8_t *>(buffer.data()) + 2 * frames * sizeof(int32_t));
  msg.data = dataVec;

  publisher_->publish(msg);
}

};  // namespace rae_hw
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rae_hw::MicNode);