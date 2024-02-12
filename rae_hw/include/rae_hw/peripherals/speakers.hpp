#ifndef RAE_HW_SPEAKERS_NODE_HPP_
#define RAE_HW_SPEAKERS_NODE_HPP_

#include <alsa/asoundlib.h>
#include <mpg123.h>
#include <sndfile.h>
#include <rae_msgs/srv/play_audio.hpp>

#include "audio_msgs/msg/audio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

namespace rae_hw {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class SpeakersNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    SpeakersNode(const rclcpp::NodeOptions& options);
    ~SpeakersNode();
    void cleanup();

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

   private:
    void play_mp3(const char*);
    void play_wav(const char*);
    rclcpp::Service<rae_msgs::srv::PlayAudio>::SharedPtr play_audio_service_;

    void play_audio_service_callback(const std::shared_ptr<rae_msgs::srv::PlayAudio::Request> request,
                                     const std::shared_ptr<rae_msgs::srv::PlayAudio::Response> response);
    snd_pcm_t* alsaHandle;

    unsigned char* buffer;
    mpg123_handle* mh;
};

}  // namespace rae_hw

#endif  // RAE_HW_SPEAKERS_NODE_HPP_
