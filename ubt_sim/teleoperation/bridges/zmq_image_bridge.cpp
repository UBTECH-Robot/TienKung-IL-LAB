#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shm_msgs/msg/image2m.hpp>
#include <sensor_msgs/msg/image.hpp>

// Minimal JSON parser — extracts integer values for known keys.
// Avoids nlohmann-json system dependency for a trivial {"width":N,"height":N} payload.
namespace tinyjson {
inline int get_int(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return -1;
    auto colon = json.find(':', pos + key.size() + 2);
    if (colon == std::string::npos) return -1;
    size_t i = colon + 1;
    while (i < json.size() && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n' || json[i] == '\r')) i++;
    int sign = 1;
    if (i < json.size() && json[i] == '-') { sign = -1; i++; }
    int val = 0;
    bool found = false;
    while (i < json.size() && json[i] >= '0' && json[i] <= '9') {
        val = val * 10 + (json[i] - '0');
        i++; found = true;
    }
    return found ? val * sign : -1;
}
} // namespace tinyjson

struct BridgeConfig {
    int zmq_port = 5557;
    std::string rgb_topic = "/ob_camera_head/color/image_raw";
    std::string depth_topic = "/ob_camera_head/depth/image_raw";
    // "Image2m" (shm_msgs/Image2m, default — walker S2) or "Image" (sensor_msgs/Image — 天工)
    std::string msg_type = "Image2m";
};

static BridgeConfig parse_args(int argc, char* argv[]) {
    BridgeConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--zmq-port") && i + 1 < argc) {
            cfg.zmq_port = std::stoi(argv[++i]);
        } else if ((arg == "--rgb-topic") && i + 1 < argc) {
            cfg.rgb_topic = argv[++i];
        } else if ((arg == "--depth-topic") && i + 1 < argc) {
            cfg.depth_topic = argv[++i];
        } else if ((arg == "--msg-type") && i + 1 < argc) {
            cfg.msg_type = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: zmq_image_bridge [OPTIONS]\n"
                      << "Options:\n"
                      << "  --zmq-port PORT       ZMQ image port (default: 5557)\n"
                      << "  --rgb-topic TOPIC     RGB image topic (default: /ob_camera_head/color/image_raw)\n"
                      << "  --depth-topic TOPIC   Depth image topic (default: /ob_camera_head/depth/image_raw)\n"
                      << "  --msg-type TYPE       Image type: Image (sensor_msgs/Image) or Image2m (shm_msgs/Image2m, default)\n";
            exit(0);
        }
    }
    return cfg;
}

static void set_shm_string(shm_msgs::msg::String& dst, const std::string& src) {
    std::fill(dst.data.begin(), dst.data.end(), '\0');
    const size_t n = std::min(src.size(), dst.data.size());
    std::memcpy(dst.data.data(), src.data(), n);
    dst.size = static_cast<uint8_t>(std::min<size_t>(n, 255));
}

static bool fill_image2m(
    shm_msgs::msg::Image2m& msg,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    int width,
    int height,
    const std::string& encoding,
    uint32_t step,
    const void* data,
    size_t size)
{
    const size_t byte_count = static_cast<size_t>(height) * step;
    if (size < byte_count) {
        return false;
    }
    if (byte_count > msg.data.size()) {
        return false;
    }

    msg.header.stamp = stamp;
    set_shm_string(msg.header.frame_id, frame_id);
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    set_shm_string(msg.encoding, encoding);
    msg.is_bigendian = 0;
    msg.step = step;
    std::fill(msg.data.begin(), msg.data.end(), 0);
    std::memcpy(msg.data.data(), data, byte_count);
    return true;
}

// Fill a sensor_msgs::msg::Image (variable-length data, no 2MB cap). Used for 天工.
static bool fill_image(
    sensor_msgs::msg::Image& msg,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    int width,
    int height,
    const std::string& encoding,
    uint32_t step,
    const void* data,
    size_t size)
{
    const size_t byte_count = static_cast<size_t>(height) * step;
    if (size < byte_count) {
        return false;
    }

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.encoding = encoding;
    msg.is_bigendian = 0;
    msg.step = step;
    const uint8_t* src = static_cast<const uint8_t*>(data);
    msg.data.assign(src, src + byte_count);
    return true;
}

class ZmqImageBridge : public rclcpp::Node
{
public:
    ZmqImageBridge(const BridgeConfig& cfg) : Node("zmq_image_bridge"), msg_type_(cfg.msg_type)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        if (msg_type_ == "Image") {
            pub_rgb_img_ = this->create_publisher<sensor_msgs::msg::Image>(cfg.rgb_topic, qos);
            pub_depth_img_ = this->create_publisher<sensor_msgs::msg::Image>(cfg.depth_topic, qos);
        } else {
            // default: Image2m (shm_msgs) — walker S2 path
            msg_type_ = "Image2m";
            pub_rgb_2m_ = this->create_publisher<shm_msgs::msg::Image2m>(cfg.rgb_topic, qos);
            pub_depth_2m_ = this->create_publisher<shm_msgs::msg::Image2m>(cfg.depth_topic, qos);
        }

        context_ = zmq::context_t(1);
        subscriber_ = zmq::socket_t(context_, ZMQ_SUB);

        std::string zmq_addr = "tcp://127.0.0.1:" + std::to_string(cfg.zmq_port);
        RCLCPP_INFO(this->get_logger(), "Connecting to ZMQ Image Server at %s", zmq_addr.c_str());
        subscriber_.connect(zmq_addr);
        subscriber_.set(zmq::sockopt::subscribe, "");
        subscriber_.set(zmq::sockopt::rcvhwm, 2);

        RCLCPP_INFO(this->get_logger(),
                    "C++ ZMQ Image Bridge Started (msg_type=%s, rgb: %s, depth: %s)",
                    msg_type_.c_str(), cfg.rgb_topic.c_str(), cfg.depth_topic.c_str());

        receive_thread_ = std::thread(&ZmqImageBridge::receive_loop, this);
    }

    ~ZmqImageBridge()
    {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void receive_loop()
    {
        while (rclcpp::ok() && running_) {
            zmq::message_t meta_msg, rgb_msg, depth_msg;

            try {
                auto res = subscriber_.recv(meta_msg, zmq::recv_flags::none);
                if (!res) continue;

                if (!subscriber_.get(zmq::sockopt::rcvmore)) continue;
                (void)subscriber_.recv(rgb_msg, zmq::recv_flags::none);

                bool has_depth = false;
                if (subscriber_.get(zmq::sockopt::rcvmore)) {
                    (void)subscriber_.recv(depth_msg, zmq::recv_flags::none);
                    has_depth = true;
                }

                publish_images(meta_msg, rgb_msg, depth_msg, has_depth);

            } catch (const zmq::error_t& e) {
                RCLCPP_ERROR(this->get_logger(), "ZMQ Error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void publish_images(zmq::message_t& meta_msg, zmq::message_t& rgb_msg, zmq::message_t& depth_msg, bool has_depth)
    {
        std::string meta_str(static_cast<char*>(meta_msg.data()), meta_msg.size());
        int w = tinyjson::get_int(meta_str, "width");
        int h = tinyjson::get_int(meta_str, "height");
        if (w <= 0 || h <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid metadata, skipping frame");
            return;
        }

        auto current_time = this->now();

        if (msg_type_ == "Image") {
            sensor_msgs::msg::Image img_msg;
            if (!fill_image(img_msg, current_time, "ob_camera_head_color_optical_frame",
                            w, h, "rgb8", static_cast<uint32_t>(w * 3),
                            rgb_msg.data(), rgb_msg.size())) {
                RCLCPP_WARN(this->get_logger(), "RGB frame invalid, skipping");
                return;
            }
            pub_rgb_img_->publish(img_msg);

            if (has_depth && depth_msg.size() > 0) {
                sensor_msgs::msg::Image depth_ros_msg;
                if (!fill_image(depth_ros_msg, current_time, "ob_camera_head_depth_optical_frame",
                                w, h, "16UC1", static_cast<uint32_t>(w * 2),
                                depth_msg.data(), depth_msg.size())) {
                    RCLCPP_WARN(this->get_logger(), "Depth frame invalid, skipping");
                    return;
                }
                pub_depth_img_->publish(depth_ros_msg);
            }
        } else {
            shm_msgs::msg::Image2m img_msg;
            if (!fill_image2m(img_msg, current_time, "ob_camera_head_color_optical_frame",
                              w, h, "rgb8", static_cast<uint32_t>(w * 3),
                              rgb_msg.data(), rgb_msg.size())) {
                RCLCPP_WARN(this->get_logger(), "RGB frame does not fit Image2m buffer, skipping");
                return;
            }
            pub_rgb_2m_->publish(img_msg);

            if (has_depth && depth_msg.size() > 0) {
                shm_msgs::msg::Image2m depth_ros_msg;
                if (!fill_image2m(depth_ros_msg, current_time, "ob_camera_head_depth_optical_frame",
                                  w, h, "16UC1", static_cast<uint32_t>(w * 2),
                                  depth_msg.data(), depth_msg.size())) {
                    RCLCPP_WARN(this->get_logger(), "Depth frame does not fit Image2m buffer, skipping");
                    return;
                }
                pub_depth_2m_->publish(depth_ros_msg);
            }
        }
    }

    // Only one pair is non-null, selected by msg_type_ at construction.
    rclcpp::Publisher<shm_msgs::msg::Image2m>::SharedPtr pub_rgb_2m_;
    rclcpp::Publisher<shm_msgs::msg::Image2m>::SharedPtr pub_depth_2m_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_img_;
    std::string msg_type_;

    zmq::context_t context_;
    zmq::socket_t subscriber_;
    std::thread receive_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char * argv[])
{
    BridgeConfig cfg = parse_args(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZmqImageBridge>(cfg);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
