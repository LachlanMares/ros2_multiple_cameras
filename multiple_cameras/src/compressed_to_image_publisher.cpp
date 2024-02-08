#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CompressedImageSubscriber : public rclcpp::Node {
public:
    CompressedImageSubscriber() : Node("camera_0_compressed_image_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/camera0/image/compressed", 10, std::bind(&CompressedImageSubscriber::imageCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera0/image/raw", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            // Decompress the received compressed image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Convert the decompressed image to ROS Image message
            sensor_msgs::msg::Image out_msg;
            out_msg.header = msg->header;
            out_msg.height = cv_ptr->image.rows;
            out_msg.width = cv_ptr->image.cols;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.is_bigendian = false;
            out_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(cv_ptr->image.step);
            size_t size = cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.channels();
            out_msg.data.resize(size);
            memcpy(out_msg.data.data(), cv_ptr->image.data, size);

            // Publish the decompressed image
            publisher_->publish(out_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}