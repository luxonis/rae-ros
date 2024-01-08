// mic_node.hpp

#include <sndfile.h>

#include <rae_msgs/srv/record_audio.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>

#include "alsa/asoundlib.h"
#include "rae_msgs/action/recording.hpp"
#include "rae_msgs/msg/rae_audio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rae_hw {

class MicNode : public rclcpp::Node {
   public:
    MicNode(const rclcpp::NodeOptions& options);
    ~MicNode();

   private:
    void configure_microphone();
    void timer_callback();
    void saveToWav(const std::vector<int32_t>& buffer, snd_pcm_uframes_t frames);
    rclcpp_action::Server<rae_msgs::action::Recording>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rae_msgs::msg::RAEAudio>::SharedPtr publisher_;
    uint64_t seq_num_ = 0;
    snd_pcm_t* handle_;
    bool recording_;
    std::string wav_filename_;
    rclcpp::Service<rae_msgs::srv::RecordAudio>::SharedPtr start_service_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    void startRecording(const std::shared_ptr<rae_msgs::srv::RecordAudio::Request> request,
                        const std::shared_ptr<rae_msgs::srv::RecordAudio::Response> response);
    void stopRecording();
};

}  // namespace rae_hw
