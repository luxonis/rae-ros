#include "rae_hw/peripherals/mic.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "alsa/asoundlib.h"
#include "alsa/pcm.h"

namespace rae_hw
{
    MicNode::MicNode(const rclcpp::NodeOptions &options) : Node("mic_node", options), handle_(nullptr)
    {
        publisher_ = this->create_publisher<rae_msgs::msg::RAEAudio>("audio_in", 10);
        configure_microphone();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MicNode::timer_callback, this));
        wav_filename_="/tmp/recording.wav";
        start_service_ = this->create_service<rae_msgs::srv::RecordAudio>(
        "start_recording",
        std::bind(&MicNode::startRecording, this, std::placeholders::_1, std::placeholders::_2)
    );
        RCLCPP_INFO(this->get_logger(), "Mic node running!");
    }
    MicNode::~MicNode()
    {
        snd_pcm_close(handle_);
    }

    void MicNode::configure_microphone()
    {
        snd_pcm_hw_params_t *params;
        unsigned int sample_rate = 44100;
        int dir;

        int rc = snd_pcm_open(&handle_, "hw:0,1", SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to open PCM device: %s", snd_strerror(rc));
            return;
        }
        recording_= false;

        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(handle_, params);
        snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S24_LE);
        snd_pcm_hw_params_set_channels(handle_, params, 2);
        snd_pcm_hw_params_set_rate_near(handle_, params, &sample_rate, &dir);

        rc = snd_pcm_hw_params(handle_, params);
        if (rc < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to set HW parameters: %s", snd_strerror(rc));
            return;
        }
    }

    void MicNode::timer_callback()
    {   
        if (recording_) {
        snd_pcm_uframes_t frames = 44100 * 0.05;
        std::vector<int32_t> buffer(2 * frames);

        int rc = snd_pcm_readi(handle_, buffer.data(), frames);
        if (rc == -EPIPE)
        {
            snd_pcm_prepare(handle_);
            RCLCPP_ERROR(this->get_logger(), "Overrun occurred. Preparing the interface.");
            return;
        }
        else if (rc < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error from ALSA readi: %s", snd_strerror(rc));
            return;
        }

        rae_msgs::msg::RAEAudio msg;
        msg.header.stamp = this->now();
        msg.seq_num = seq_num_++;
        msg.frames = frames;
        msg.channels = 2;
        msg.sample_rate = 44100;
        msg.encoding = "S24LE";
        msg.is_bigendian = 0;
        msg.layout = msg.LAYOUT_INTERLEAVED;
        msg.step = 6;

        std::vector<int32_t> dataVec(buffer.data(), buffer.data() + 2 * frames * sizeof(int32_t));

        msg.data = dataVec;

        // Save the audio buffer to a WAV file
        saveToWav(buffer, frames);

        publisher_->publish(msg);}
    }

    void MicNode::saveToWav(const std::vector<int32_t> &buffer, snd_pcm_uframes_t frames)
{
    SF_INFO sfinfo;
    sfinfo.channels = 2;
    sfinfo.samplerate = 44100;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_24;

    // Open the file in read-write mode
    SNDFILE *file = sf_open(wav_filename_.c_str(), SFM_RDWR, &sfinfo);

    if (!file)
    {
        RCLCPP_ERROR(this->get_logger(), "Error opening WAV file for writing: %s", sf_strerror(file));
        return;
    }

    // Seek to the end of the file
    sf_count_t count = sf_seek(file, 0, SEEK_END);
    if (count < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error seeking to the end of WAV file: %s", sf_strerror(file));
        sf_close(file);
        return;
    }

    // Write the new frames to the file
    count = sf_write_int(file, buffer.data(), frames * sfinfo.channels);
    if (count != frames * sfinfo.channels)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to WAV file: %s", sf_strerror(file));
    }

    sf_close(file);
}

void MicNode::startRecording(const std::shared_ptr<rae_msgs::srv::RecordAudio::Request> request,
                             const std::shared_ptr<rae_msgs::srv::RecordAudio::Response> response) {
    // Start recording when the service is called
    recording_ = true;
    wav_filename_ = request->file_location;
    stop_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MicNode::stopRecording, this));
    response->success = true;
    response->message = "Recording started. File will be saved to " + wav_filename_ + ".";
    RCLCPP_INFO(get_logger(), "Recording started.");
}

void MicNode::stopRecording() {
    recording_ = false;
    stop_timer_->cancel();  // Stop the timer
    RCLCPP_INFO(get_logger(), "Recording stopped.");
}

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::MicNode>(rclcpp::NodeOptions());
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}