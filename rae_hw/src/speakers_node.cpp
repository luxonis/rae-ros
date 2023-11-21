#include "rae_hw/peripherals/speakers.hpp"

namespace rae_hw {

SpeakersNode::SpeakersNode(const rclcpp::NodeOptions &options)
    : Node("speakers_node", options) {
  // Initialize ALSA or any other audio setup code here

  // Subscribe to the audio topic
  subscription_ = create_subscription<std_msgs::msg::Int32>(
      "audio_topic", 10, std::bind(&SpeakersNode::audio_callback, this, std::placeholders::_1));

  // Any other initialization code here
  
  RCLCPP_INFO(this->get_logger(), "Speakers node running!");
}

SpeakersNode::~SpeakersNode() {
  delete[] buffer;
  mpg123_close(mh);
  mpg123_delete(mh);
  mpg123_exit();
}

void SpeakersNode::audio_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  // Assuming msg->data contains the raw audio data

  // You can implement your logic here to process and play the audio
  // For example, you can save it to a temporary file and then play it using a
  // library like libmpg123
  play_mp3("/ws/src/rae-ros/horn_cucaracha.mp3");
}

void SpeakersNode::play_mp3(const char *mp3_file) {
  // Initialize libmpg123
    mpg123_init();
    mh = mpg123_new(NULL, NULL);
    long rate; // Set your desired sample rate here
    int channels, encoding;
    if (mpg123_open(mh, mp3_file) != MPG123_OK ||
        mpg123_getformat(mh, &rate, &channels, &encoding) != MPG123_OK) {
        RCLCPP_ERROR(this->get_logger(), "Cant read MP3 file");
        return;
    }
    
    // Open ALSA device
    if (snd_pcm_open(&alsaHandle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ALSA playback device.");
        return ;
    }
    

    // Set ALSA parameters
    if (channels==1) {
 snd_pcm_set_params(alsaHandle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 1,44100, 2, 50000);
} else if (channels==2) {
snd_pcm_set_params(alsaHandle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 1,88200, 2, 50000);} 

    // Read and play MP3 file
    size_t buffer_size = mpg123_outblock(mh)*4;
    unsigned char *buffer = new unsigned char[buffer_size];
    size_t err;

    while (mpg123_read(mh, buffer, buffer_size, &err) == MPG123_OK) {
        if (snd_pcm_writei(alsaHandle, buffer, buffer_size/2) < 0) {
            std::cerr << "Error in snd_pcm_writei: " << snd_strerror(err) << std::endl;
        }
    }

    // Cleanup
    delete[] buffer;
    snd_pcm_close(alsaHandle);
    mpg123_close(mh);
    mpg123_delete(mh);
    mpg123_exit();

    return ;
}

} // namespace rae_hw

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rae_hw::SpeakersNode>(rclcpp::NodeOptions());
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
