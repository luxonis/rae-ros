// mic_node.hpp

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rae_msgs/msg/rae_audio.hpp"
#include "alsa/asoundlib.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include "rae_msgs/action/recording.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <sndfile.h>

namespace rae_hw {

class MicNode : public rclcpp::Node {
public:
    MicNode(const rclcpp::NodeOptions &options);
    ~MicNode();

private:
    void configure_microphone();
    void timer_callback();
    void saveToWav(const std::vector<int32_t> &buffer, snd_pcm_uframes_t frames);
    rclcpp_action::Server<rae_msgs::action::Recording>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rae_msgs::msg::RAEAudio>::SharedPtr publisher_;
    uint64_t seq_num_ = 0;
    snd_pcm_t *handle_;
    bool recording_;
    std::string wav_filename_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    void startRecording(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);
    void stopRecording();

};

} // namespace rae_hw
