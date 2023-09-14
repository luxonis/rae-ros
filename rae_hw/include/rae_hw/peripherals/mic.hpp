#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "audio_msgs/msg/audio.hpp"
#include "alsa/asoundlib.h"
namespace rae_hw{

class MicNode : public rclcpp::Node {
public:
    MicNode(const rclcpp::NodeOptions &options);
    ~MicNode();

private:
    void configure_microphone();

    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<audio_msgs::msg::Audio>::SharedPtr publisher_;
    uint64_t seq_num_ = 0;
    snd_pcm_t *handle_;
};

}