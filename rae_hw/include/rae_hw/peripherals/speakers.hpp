
#include "rclcpp/rclcpp.hpp"
#include "audio_msgs/msg/audio.hpp"

#include <alsa/asoundlib.h>
namespace rae_hw{
class SpeakersNode : public rclcpp::Node
{
public:
  SpeakersNode(const rclcpp::NodeOptions &options);
  ~SpeakersNode();

private:
  void audio_callback(const audio_msgs::msg::Audio::SharedPtr msg);

  rclcpp::Subscription<audio_msgs::msg::Audio>::SharedPtr subscription_;
  snd_pcm_t *alsaHandle;
};

}