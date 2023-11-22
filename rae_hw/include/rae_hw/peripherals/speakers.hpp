#ifndef RAE_HW_SPEAKERS_NODE_HPP_
#define RAE_HW_SPEAKERS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "audio_msgs/msg/audio.hpp"
#include "std_msgs/msg/int32.hpp"
#include <alsa/asoundlib.h>
#include <mpg123.h>
#include <rae_msgs/srv/play_audio.hpp>

namespace rae_hw {

class SpeakersNode : public rclcpp::Node {
public:
  SpeakersNode(const rclcpp::NodeOptions &options);
  ~SpeakersNode();

private:
  void play_mp3(const char *);
  rclcpp::Service<rae_msgs::srv::PlayAudio>::SharedPtr play_audio_service_;

  void play_audio_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<rae_msgs::srv::PlayAudio::Request> request,
      const std::shared_ptr<rae_msgs::srv::PlayAudio::Response> response);
  snd_pcm_t *alsaHandle;

  unsigned char *buffer;
  mpg123_handle *mh;
};

} // namespace rae_hw

#endif // RAE_HW_SPEAKERS_NODE_HPP_
