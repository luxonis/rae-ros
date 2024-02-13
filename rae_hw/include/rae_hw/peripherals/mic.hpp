// mic_node.hpp

#include <sndfile.h>

#include <rae_msgs/srv/record_audio.hpp>
#include <rae_msgs/srv/stop_recording.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>

#include "alsa/asoundlib.h"
#include "rae_msgs/action/recording.hpp"
#include "rae_msgs/msg/rae_audio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rae_hw {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class MicNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    MicNode(const rclcpp::NodeOptions& options);
    ~MicNode();

    void cleanup();

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

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
    void applyLowPassFilter(std::vector<int32_t>& buffer);
    rclcpp::Service<rae_msgs::srv::RecordAudio>::SharedPtr start_service_;
    rclcpp::Service<rae_msgs::srv::StopRecording>::SharedPtr stop_service_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    void startRecording(const std::shared_ptr<rae_msgs::srv::RecordAudio::Request> request,
                        const std::shared_ptr<rae_msgs::srv::RecordAudio::Response> response);
    void timeoutRecording();
    void stopRecording(const std::shared_ptr<rae_msgs::srv::StopRecording::Request> stop_request,
                       const std::shared_ptr<rae_msgs::srv::StopRecording::Response> stop_response);
};

}  // namespace rae_hw
