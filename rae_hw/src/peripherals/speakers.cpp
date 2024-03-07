#include "rae_hw/peripherals/speakers.hpp"

namespace rae_hw {

SpeakersNode::SpeakersNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("speakers_node", options) {}

SpeakersNode::~SpeakersNode() {
    cleanup();
}

void SpeakersNode::cleanup() {
    delete[] buffer;
    mpg123_close(mh);
    mpg123_delete(mh);
    mpg123_exit();
}

CallbackReturn SpeakersNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Initialize ALSA or any other audio setup code here

    // Create Audio Service
    play_audio_service_ = create_service<rae_msgs::srv::PlayAudio>(
        "play_audio", std::bind(&SpeakersNode::play_audio_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Any other initialization code here

    RCLCPP_INFO(this->get_logger(), "Speakers node configured!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SpeakersNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Speakers node activating!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SpeakersNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Speakers node deactivating!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SpeakersNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Speakers node shutting down!");
    cleanup();
    return CallbackReturn::SUCCESS;
}

void SpeakersNode::play_audio_service_callback(const std::shared_ptr<rae_msgs::srv::PlayAudio::Request> request,
                                               const std::shared_ptr<rae_msgs::srv::PlayAudio::Response> response) {
    const std::string& file_location = request->file_location;
    const float gain = request->gain;


    // Check if the file ends with ".wav"
    if(file_location.size() >= 4 && file_location.substr(file_location.size() - 4) == ".wav") {
        // Call the play_wav function
        play_wav(file_location.c_str(), gain);
        response->success = true;
        return;
    }

    // Check if the file ends with ".mp3"
    if(file_location.size() >= 4 && file_location.substr(file_location.size() - 4) == ".mp3") {
        // Call the play_mp3 function
        play_mp3(file_location.c_str());
        response->success = true;
        return;
    }

    // Unsupported file format
    RCLCPP_ERROR(this->get_logger(), "Unsupported file format: %s", file_location.c_str());
    response->success = false;
}

void SpeakersNode::play_mp3(const char* mp3_file) {
    // Initialize libmpg123
    mpg123_init();
    mh = mpg123_new(NULL, NULL);
    long rate;  // Set your desired sample rate here
    int channels, encoding;
    if(mpg123_open(mh, mp3_file) != MPG123_OK || mpg123_getformat(mh, &rate, &channels, &encoding) != MPG123_OK) {
        RCLCPP_ERROR(this->get_logger(), "Cant read MP3 file");
        return;
    }

    // Open ALSA device
    if(snd_pcm_open(&alsaHandle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ALSA playback device.");
        return;
    }

    // Set ALSA parameters

    snd_pcm_set_params(alsaHandle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, channels, rate, 2, 50000);

    size_t buffer_size = mpg123_outblock(mh) * 4;
    unsigned char* buffer = new unsigned char[buffer_size];
    size_t err;

    while(mpg123_read(mh, buffer, buffer_size, &err) == MPG123_OK) {
        if(snd_pcm_writei(alsaHandle, buffer, buffer_size / (2 * channels)) < 0) {
            std::cerr << "Error in snd_pcm_writei: " << snd_strerror(err) << std::endl;
        }
    }

    // Cleanup
    delete[] buffer;
    snd_pcm_close(alsaHandle);
    mpg123_close(mh);
    mpg123_delete(mh);
    mpg123_exit();

    return;
}

void SpeakersNode::play_wav(const char* wav_file, const float gain) {
    // Open WAV file
    SF_INFO sfinfo;
    SNDFILE* file = sf_open(wav_file, SFM_READ, &sfinfo);
    if(!file) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open WAV file: %s", wav_file);
        return;
    }

    // Open ALSA device
    if(snd_pcm_open(&alsaHandle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ALSA playback device.");
        return;
    }

    // Set ALSA parameters
    snd_pcm_set_params(alsaHandle, SND_PCM_FORMAT_S32_LE, SND_PCM_ACCESS_RW_INTERLEAVED, sfinfo.channels, sfinfo.samplerate, 2, 50000);

    // Read and play WAV file
    const int BUFFER_SIZE = 4096;
    int32_t* buffer_wav = new int32_t[BUFFER_SIZE * sfinfo.channels];  // Use int32_t for 32-bit format
    sf_count_t readCount;

    while((readCount = sf_readf_int(file, buffer_wav, BUFFER_SIZE)) > 0) {
        // Apply gain to the samples
        for(int i = 0; i < readCount * sfinfo.channels; ++i) {
            float sample = static_cast<float>(buffer_wav[i]) / std::numeric_limits<int32_t>::max();
            sample *= gain;  // Apply gain
            buffer_wav[i] = static_cast<int32_t>(sample * std::numeric_limits<int32_t>::max());
        }

        // Write the processed buffer to the playback device
        if(snd_pcm_writei(alsaHandle, buffer_wav, readCount) < 0) {
            std::cerr << "Error in snd_pcm_writei: " << snd_strerror(readCount) << std::endl;
            break;
        }
    }

    // Cleanup
    delete[] buffer_wav;
    sf_close(file);
    snd_pcm_close(alsaHandle);

    return;
}

}  // namespace rae_hw

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::SpeakersNode>(rclcpp::NodeOptions());
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
