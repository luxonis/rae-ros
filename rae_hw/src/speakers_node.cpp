#include "rae_hw/peripherals/speakers.hpp"

namespace rae_hw
{

    SpeakersNode::SpeakersNode(const rclcpp::NodeOptions &options)
        : Node("speakers_node", options)
    {
        // Initialize ALSA or any other audio setup code here

        // Create Audio Service
        play_audio_service_ = create_service<rae_msgs::srv::PlayAudio>(
            "play_audio", std::bind(&SpeakersNode::play_audio_service_callback, this,
                                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Any other initialization code here

        RCLCPP_INFO(this->get_logger(), "Speakers node running!");
    }

    SpeakersNode::~SpeakersNode()
    {
        delete[] buffer;
        mpg123_close(mh);
        mpg123_delete(mh);
        mpg123_exit();
    }

    void SpeakersNode::play_audio_service_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rae_msgs::srv::PlayAudio::Request> request,
        const std::shared_ptr<rae_msgs::srv::PlayAudio::Response> response)
    {
        // Use request->mp3_file to get the MP3 file location
        const char *mp3_file = request->mp3_file.c_str();

        // Call the play_mp3 function
        play_mp3(mp3_file);

        // Respond with success (modify based on your play_mp3 result)
        response->success = true;
    }

    void SpeakersNode::play_mp3(const char *mp3_file)
    {
        // Initialize libmpg123
        mpg123_init();
        mh = mpg123_new(NULL, NULL);
        long rate; // Set your desired sample rate here
        int channels, encoding;
        if (mpg123_open(mh, mp3_file) != MPG123_OK ||
            mpg123_getformat(mh, &rate, &channels, &encoding) != MPG123_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Cant read MP3 file");
            return;
        }

        // Open ALSA device
        if (snd_pcm_open(&alsaHandle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ALSA playback device.");
            return;
        }

        // Set ALSA parameters

        snd_pcm_set_params(alsaHandle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, channels, rate, 2, 50000);

        // Read and play MP3 file
        size_t buffer_size = mpg123_outblock(mh) * 4;
        unsigned char *buffer = new unsigned char[buffer_size];
        size_t err;

        while (mpg123_read(mh, buffer, buffer_size, &err) == MPG123_OK)
        {
            if (snd_pcm_writei(alsaHandle, buffer, buffer_size / 2) < 0)
            {
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

} // namespace rae_hw

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::SpeakersNode>(rclcpp::NodeOptions());
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
