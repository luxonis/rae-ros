#include "rae_hw/peripherals/speakers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "audio_msgs/msg/audio.hpp"

#include <alsa/asoundlib.h>
namespace rae_hw
{

  SpeakersNode::SpeakersNode(const rclcpp::NodeOptions &options)
      : Node("speakers_node", options), alsaHandle(NULL)
  {
    this->subscription_ = this->create_subscription<audio_msgs::msg::Audio>(
        "audio_out", 10, std::bind(&SpeakersNode::audio_callback, this, std::placeholders::_1));

    // Setup ALSA
    snd_pcm_open(&alsaHandle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    snd_pcm_set_params(alsaHandle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 2, 44100, 1, 50000); // 2 channel, 44100Hz, 50ms latency
    RCLCPP_INFO(this->get_logger(), "Speakers node running!");
  }

  SpeakersNode::~SpeakersNode()
  {
    snd_pcm_close(alsaHandle);
  }

  void SpeakersNode::audio_callback(const audio_msgs::msg::Audio::SharedPtr msg)
  {
    // Assuming that frames are 4 bytes (2 channels, 16 bits each)
    int frames = msg->data.size() / 4;

    int ret = snd_pcm_writei(alsaHandle, msg->data.data(), frames);
    if (ret == -EPIPE)
    {
      /* EPIPE means underrun */
      RCLCPP_ERROR(this->get_logger(), "Buffer underrun occurred");
      snd_pcm_prepare(alsaHandle);
    }
    else if (ret < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from snd_pcm_writei: %s", snd_strerror(ret));
    }
    else if (ret != frames)
    {
      RCLCPP_ERROR(this->get_logger(), "Short write, wrote %d frames", ret);
    }
  }

};
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rae_hw::SpeakersNode);